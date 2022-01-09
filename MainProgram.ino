#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <SD.h>

#define RB04_GPScs 8
#define PiezoPin PD5
OneWire DS18B20(7);

//---------------------------Setup TTN Network---------------------------
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
static const u1_t PROGMEM DEVEUI[8]={ 0x41, 0xA6, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
static const u1_t PROGMEM APPKEY[16] = { 0xEB, 0x38, 0x14, 0xA7, 0x38, 0xAE, 0x74, 0xB1, 0x7D, 0x78, 0xB5, 0x5F, 0x03, 0xCB, 0x58, 0x7F };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t payload[18];
static osjob_t sendjob;
const unsigned TX_INTERVAL = 7;

const lmic_pinmap lmic_pins = {
    .nss = 9,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {2, 6, LMIC_UNUSED_PIN},
};
//-----------------------------------------------------------------------

bool device_state = 1;
int beep_count = 0;

float ext_temp;
byte ext_temp_addr[8] = {0x28, 0xFF, 0x64, 0x1E, 0x31, 0x83, 0x47, 0xA1};

char gps_buffer[80];
float GPS_latitude;
float GPS_longitude;
float GPS_speed;
int GPS_fix = 0;
int GPS_satellite = 0;
int tmp_array[2] = {0, 0};

const char filename[] = "data.txt";
File txtFile;
String txtFileBuffer;

const int GPS_TASK_DELAY = 5000;
const int EXT_TEMP_TASK_DELAY = 2000;
const int SD_TASK_DELAY = 1000;
int PIEZO_TASK_DELAY = 1000;

unsigned long ext_temp_sensor_timer = millis();
unsigned long GPS_task_timer = millis();
unsigned long piezo_timer = millis();
unsigned long SD_timer = millis();

void setup() {
  Serial.begin(115200);
  Serial.println("Advanced PCB Design: LoRa Instrument");
  Serial.println("------------------------------------");
  Serial.println();

  SPI.begin();
  os_init();
  LMIC_reset();
  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7,14);
  sendPayload(&sendjob);

  pinMode(RB04_GPScs, OUTPUT);
  pinMode(PiezoPin, OUTPUT);

  // init the SD card
  if (!SD.begin()) {
    Serial.println("SD Card unknown or not present");
    device_state = 0;
  }
  txtFile = SD.open(filename, FILE_WRITE);
  if (!txtFile) {
    Serial.print("Error opening ");
    Serial.println(filename);
    device_state = 0;
  }
}

void loop() {
  os_runloop_once();
  
  if (millis() - ext_temp_sensor_timer > EXT_TEMP_TASK_DELAY){
    ext_temp_sensor_timer = millis();
    uint16_t payload_temp = LMIC_f2sflt16(ext_temp/100);
    payload[0] = lowByte(payload_temp);
    payload[1] = highByte(payload_temp);
    readExternalTemperature();
  }
  
  if(millis() - GPS_task_timer > GPS_TASK_DELAY){
    readGPS();
    getFixGPS();
    getCoordinatesGPS();
    getSpeedGPS();
    
    floatToTwoInt(tmp_array, GPS_latitude);
    payload[2] = tmp_array[0];
    payload[3] = lowByte(tmp_array[1]);
    payload[4] = highByte(tmp_array[1]);
    
    floatToTwoInt(tmp_array, GPS_longitude);
    payload[5] = tmp_array[0];
    payload[6] = lowByte(tmp_array[1]);
    payload[7] = highByte(tmp_array[1]);

    uint16_t payload_speed = LMIC_f2sflt16(GPS_speed/100);
    payload[8] = lowByte(payload_speed);
    payload[9] = highByte(payload_speed);
    
    payload[12] = GPS_satellite;
    payload[13] = GPS_fix;
  }
  
  if (millis() - piezo_timer >= PIEZO_TASK_DELAY){
    piezo_timer = millis();
    switch (device_state){
      case 0: // setup failed = continous sound
        tone(PiezoPin, 500);
        break; 
      case 1: // setup success = 2 beep in 1s
        PIEZO_TASK_DELAY = 500;
        if (beep_count < 2){
          tone(PiezoPin, 1000, 250);
          beep_count++;
        }
        else{
          beep_count = 0;
          PIEZO_TASK_DELAY = 1000;
          device_state = 2;
        }
        break;
      default:
        beep_count = 0;
        PIEZO_TASK_DELAY = 1000;
        break;
    }
  }

  if (millis() - SD_timer > 1000){
    SD_timer = millis();
    if (!GPS_fix){
      // check if the SD card is available to write data without blocking
      // and if the buffered data is enough for the full chunk size
      unsigned int chunkSize = txtFile.availableForWrite();
      txtFileBuffer = payload;
      if (chunkSize && txtFileBuffer.length() >= chunkSize) {
        txtFile.write(txtFileBuffer.c_str(), chunkSize);
      }  
    }
  }
}

void onEvent (ev_t ev) {
  switch(ev) {
    case EV_JOINING:
      Serial.println(F("Joining LoRa TTN Network"));
      break;
    case EV_JOINED:
      Serial.println(F("Joined successfully"));
      LMIC_setLinkCheckMode(0);
      break;
    case EV_TXCOMPLETE:
      Serial.println("Payload sent successfully");
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), sendPayload);
      break;
    case EV_TXSTART:
      Serial.println();
      Serial.println("Starting new transmission");
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    default:
      Serial.println("ERROR: Unable to join");
      break;
  }
}

void sendPayload(osjob_t* j){
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
    Serial.println(F("Payload queued"));
  }
}

void readGPS() {
  bool start_reading = 0;
  char temp_buffer[1];
  // take the chip select low to select the device:
  digitalWrite(RB04_GPScs, LOW);
  if (!start_reading){
    temp_buffer[0] = SPI.transfer(0x00);
    if (temp_buffer[0] == '$'){
      start_reading = 1;
    }
    delay(2);
  }
  if (start_reading){
    gps_buffer[0] = temp_buffer[0];
    for(int j = 1; j < 80; j++) {
      gps_buffer[j] = SPI.transfer(0x00);
      if (gps_buffer[j] == '*'){
        break;
      }
      delay(2);
    }
    start_reading = 0;
  }  
  // take the chip select high to de-select:
  digitalWrite(RB04_GPScs, HIGH);
}

void getCoordinatesGPS() {
  char NMEA_code[6];
  for (int i = 0; i < 6; i++) {
    NMEA_code[i] = gps_buffer[i];
  }
  if (!strcmp(NMEA_code, "$GNGLL")){
    GPS_latitude = getDataFromComma(1,9);
    GPS_longitude = getDataFromComma(3,10);
  }
  else if (!strcmp(NMEA_code, "$GNRMC")){
    GPS_latitude = getDataFromComma(3,9);
    GPS_longitude = getDataFromComma(5,10);
  }
  else if (!strcmp(NMEA_code, "$GNGGA")){
    GPS_latitude = getDataFromComma(2,9);
    GPS_longitude = getDataFromComma(4,10);
  }
}

void getSpeedGPS() {
  char NMEA_code[6];
  for (int i = 0; i < 6; i++) {
    NMEA_code[i] = gps_buffer[i];
  }
  if (!strcmp(NMEA_code, "$GNRMC")){
    GPS_speed = getDataFromComma(7,5);
  }
  else if (!strcmp(NMEA_code, "$GNRMA")){
    GPS_speed = getDataFromComma(8,4);
  }
  else if (!strcmp(NMEA_code, "$GNVTG")){
    GPS_speed = getDataFromComma(5,5);
  }
}

void getFixGPS() {
  char NMEA_code[6];
  for (int i = 0; i < 6; i++) {
    NMEA_code[i] = gps_buffer[i];
  }
  if (!strcmp(NMEA_code, "$GNGGA")){
    GPS_fix = getDataFromComma(5,1);
    GPS_satellite = getDataFromComma(6,2);
  }
  if (!strcmp(NMEA_code, "$GNGSA")){
    GPS_fix = getDataFromComma(2,1) - 1;
  }
}

float getDataFromComma(int start_comma, int data_size) {
  char data[data_size];
  float result;
  int count = 0;
  int i=0, j=0;
  for (i = 0; i < 80; i++) {
    if(gps_buffer[i] == ','){
      count++;
    }
    if(count == start_comma){
      if (j == data_size){break;}
      data[j] = gps_buffer[i+1];
      j++;
    }
  }
  result = atof(data);
  return result;
}

void readExternalTemperature() {
  byte data[12];

  DS18B20.reset();
  DS18B20.select(ext_temp_addr);
  DS18B20.write(0x44, 1); // Start conversion
  delay(750);
  DS18B20.reset();
  DS18B20.select(ext_temp_addr);    
  DS18B20.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++) {
    data[i] = DS18B20.read();
  }
  
  // Convert the data to actual temperature
  int16_t raw = (data[1] << 8) | data[0];
  ext_temp = (float)raw / 16.0;
}

void floatToTwoInt (int myArray[], float myValue) {
  int firstPart = myValue;
  myArray[0] = firstPart;
  myValue -= firstPart;
  int secondPart = myValue * 10000;
  myArray[1] = secondPart;
}
