#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <Wire.h>
#include "DS3231.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SparkFun_Ublox_Arduino_Library.h>

//sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60       /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;

//temp
#define ONE_WIRE_BUS 4
#define ONE_WIRE_BUS2 0

//lora
#define nss 18
#define rst 23
#define dio0 26

//gps+axp
#define T_BEAM_V10  // AKA Rev1 for board versions T-beam_V1.0 and V1.1 (second board released)
#if defined(T_BEAM_V07)
#define GPS_RX_PIN      12
#define GPS_TX_PIN      15
#elif defined(T_BEAM_V10)
#include <Wire.h>
#include <axp20x.h>
AXP20X_Class axp;
#define I2C_SDA         21
#define I2C_SCL         22
#define GPS_RX_PIN      34
#define GPS_TX_PIN      12
#endif
SFE_UBLOX_GPS myGPS;
TinyGPSPlus gps;
int state = 0; // steps through states
HardwareSerial SerialGPS(1);
//String read_sentence;

OneWire oneWire(ONE_WIRE_BUS);
OneWire oneWire2(ONE_WIRE_BUS2);
DallasTemperature sensors(&oneWire);
DallasTemperature sensors2(&oneWire2);
RTClib RTC;
const long frequency = 450E6;  // LoRa Frequency
Adafruit_MPU6050 mpu;

//manage multi i2c 
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  //Serial.print(bus);
}

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);
  Wire.begin();
  sensors.begin();
  LoRa.setPins(nss, rst, dio0);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  //mpu6050setup
  TCA9548A(3); 
  {
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
  }

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();

  //LoRa.setSyncWord(0x12);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125000);
  LoRa.setCodingRate4(8);
  LoRa.setSyncWord(0x34);

  //pin
  pinMode(2, INPUT);

  //gps+axp
  {
  Wire.begin(I2C_SDA, I2C_SCL);
  #if defined(T_BEAM_V10)
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
        Serial.println("AXP192 Begin PASS");
    } else {
        Serial.println("AXP192 Begin FAIL");
    }
    // GPS main power
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); 
    // LoRa main power
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); 
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); 
    axp.setPowerOutPut(AXP192_DCDC3, AXP202_ON); 
  #endif 
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("All comms started");
  delay(100);

  do {
    if (myGPS.begin(SerialGPS)) {
      Serial.println("Connected to GPS");
      myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
      myGPS.saveConfiguration(); //Save the current settings to flash and BBR
      Serial.println("GPS serial connected, output set to NMEA");
      myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
      myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
      myGPS.saveConfiguration(); //Save the current settings to flash and BBR
      Serial.println("Enabled/disabled NMEA sentences");
      break;
    }
    delay(1000);
  } while(1);

  }
}

int counter = 0;
String nodeid = "04";
String gatewayid; //no gw number 0
char *gatewaylist[6] = {"01", "02", "03", "04", "05", "09"};
bool isaccept ;
bool isok ;
String submitdata ;
String datapayload;

void loop() {
    //wakeup
    /*
    Serial.println("Boot number: " + String(bootCount));
    print_wakeup_reason();*/
    
    //power
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); 
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); 
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    
    //collect data
    submitbuoy();
    Serial.print("[buoy]");
    Serial.print(" capture data : ");
    Serial.println(datapayload);

    /*
    //step3 trans data
    Serial.print("[->gateway");
    Serial.print(gatewayid);
    Serial.print("] ");
    Serial.print("sand data to gateway ");
    Serial.println(gatewayid);
    String fc3 = "3";
    //String msg = "iamdata";
    String trans = nodeid+gatewayid+fc3+datapayload;
    LoRa_sendMessage(trans); // send a trigger*/
    
    
    //reset
    isaccept = false;
    isok = false;
    gatewayid = "00";
    
    //step1 trigger
    Serial.println("[buoy->] Find gateway...");
    String fc1 = "1";
    String trigger = nodeid+gatewayid+fc1;
    LoRa_sendMessage(trigger); // send a trigger
    

    //step2 accepted
    int counteraccept = 20;
    while(!isaccept){
      Serial.print("[buoy<-]");
      Serial.println(" wait gateway...");
      //delay(20000);
      if(isaccept==false){
        delay(1000);
        counteraccept = counteraccept - 1;
        if(counteraccept==0){
          LoRa_sendMessage(trigger); // send a trigger
          counteraccept = 20;
        }
      }
    }

    //step3 trans data
    Serial.print("[->gateway");
    Serial.print(gatewayid);
    Serial.print("] ");
    Serial.print("sand data to gateway ");
    Serial.println(gatewayid);
    String fc3 = "3";
    //String msg = "iamdata";
    String trans = nodeid+gatewayid+fc3+datapayload;
    LoRa_sendMessage(trans); // send a trigger

    //step4 is data ok
    int counterdata = 20;
    while(!isok){
      Serial.print("[<-gateway");
      Serial.print(gatewayid);
      Serial.print("] ");
      Serial.print("wait status from gateway");
      Serial.println(gatewayid);
      if(isok==false){
        delay(1000);
        counterdata = counterdata - 1;
        if(counterdata==0){
          LoRa_sendMessage(trans); // send a trigger
          counterdata = 20;
        }
      }
    }

    Serial.println("[buoy] oh god this is successfully");
    Serial.println("[buoy] Sleep for next time");
    
    
    delay(1000);

    /*
    // Turn off all AXP202 power channels
    axp.setPowerOutPut(AXP202_LDO2, AXP202_OFF);
    axp.setPowerOutPut(AXP202_LDO3, AXP202_OFF);
    axp.setPowerOutPut(AXP202_LDO4, AXP202_OFF);
    //axp.setPowerOutPut(AXP202_DCDC2, AXP202_OFF);
    //axp.setPowerOutPut(AXP202_DCDC3, AXP202_OFF);
    //axp.setPowerOutPut(AXP202_EXTEN, AXP202_OFF);

    //about sleep
    ++bootCount;
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +" Seconds");
    Serial.println("Going to sleep now");
    delay(1000);
    Serial.flush(); 
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
    //delay(30000);*/
    
}

void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {

  while (LoRa.available()) {
    String message = LoRa.readString();
    //accept and get gatewayid
    String fc2 = "2";
    String fc4 = "4";
    
    if(message.substring(2,4)==nodeid and message.substring(4,5)==fc2 and isaccept==false){
      gatewayid = message.substring(0,2);
      for (int i = 0; i < 6; i++){
        String x = String(gatewaylist[i]);
        if(x==gatewayid){
          isaccept = true;
          Serial.println("checked gateway pass!");
        }
      }
      Serial.print("[<-gateway");
      Serial.print(message.substring(0,2));
      Serial.print("] ");
      Serial.print("now accept gateway.");
      Serial.println(message.substring(0,2));
      }
  
    //accept data status
    if(message.substring(0,2)== gatewayid and message.substring(2,4)==nodeid and message.substring(4,5)==fc4 and isok==false){
      isok = true;
      Serial.print("[<-gateway");
      Serial.print(message.substring(0,2));
      Serial.print("] ");
      Serial.println("Data is Ok.");
      }
  }
  //Serial.print("ALL:Node Receive: ");
  //Serial.println(message);
}

void onTxDone() {
  //Serial.println("TxDone");
  LoRa_rxMode();
}


void submitbuoy(){
  TCA9548A(3); 
  //first got gps
    bool gpsstate = false;
    while (SerialGPS.available() > 0 and !gpsstate){
      //Serial.println("begin...");
      gps.encode(SerialGPS.read());
      Serial.println("#capture data#");
      if (gps.location.isUpdated()){
        Serial.print("Latitude(real)= "); 
        Serial.print(gps.location.lat(), 6);
        Serial.print(" Longitude(real)= "); 
        Serial.println(gps.location.lng(), 6);
        gpsstate = true;
      }
      else{
        Serial.print("Latitude(real)= "); 
        Serial.print(0, 6);
        Serial.print(" Longitude(real)= "); 
        Serial.println(0, 6);
        gpsstate = true;
       } 
      //delay(3000);  
    }
    
  TCA9548A(2); 
  DateTime now = RTC.now(); 
  //date 2+2+4 byte scale + 10 
  //time 2+2+2 byte scale + 10 
  int pdd = now.day();
  int pmm = now.month();
  int pyy = now.year();
  int phh = now.hour();
  int pmn = now.minute();
  int pss = now.second();
  String dd = String(pdd+10);
  String mm = String(pmm+10);
  String yy = String(pyy);
  String hh = String(phh+10);
  String mn = String(pmn+10);
  String ss = String(pss+10);

  TCA9548A(3);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //gyro accr 3*6  = 18 byte //scale+500
  int pax = a.acceleration.x;
  int pay = a.acceleration.y;
  int paz = a.acceleration.z;
  int pgx = g.gyro.x;
  int pgy = g.gyro.y;
  int pgz = g.gyro.z;
  String ax = String(pax+500);
  String ay = String(pay+500);
  String az = String(paz+500);
  String gx = String(pgx+500);
  String gy = String(pgy+500);
  String gz = String(pgz+500);


  //id = src 2 byte 

  //battery 2 byte //scale * 10
  //int pbtr = random(25,42);
  int pbtr = int(axp.getBattVoltage());
  pbtr = pbtr/100;
  String btr;
  if (pbtr<1)
    btr = "00";
  else
    btr = String(pbtr);
  
  //temp 4 byte //sccale * 100
  //1
  sensors.requestTemperatures();
  int ptemp = sensors.getTempCByIndex(0)*100;

  //2
  sensors2.requestTemperatures();
  int ptemp2 = sensors2.getTempCByIndex(0)*100;
  int t = ptemp+ptemp2;
  t = t/2;
  String ttemp = String(t);

  //ec 2-3 byte
  int pec = random(650,680);
  String ec = String(pec);

  //lat-lng 8+9 byte 13.836887, 100.576442
  float pLt = 13.836887;
  float pLg = 100.576442;
  String Lt = String(int(pLt*1000000));
  String Lg = String(int(pLg*1000000));
  String S = " ";

  
  //serial output
  Serial.print("Time:");
  Serial.print(pdd);
  Serial.print("/");
  Serial.print(pmm);
  Serial.print("/");
  Serial.print(pyy);
  Serial.print(" ");
  Serial.print(phh);
  Serial.print(":");
  Serial.print(pmn);
  Serial.print(":");
  Serial.println(pss);

  Serial.print("Temp1: ");
  Serial.print(ptemp);
  Serial.print(" ");
  Serial.print("Temp2: ");
  Serial.println(ptemp2);
  Serial.print(" ");
  Serial.print("avg: ");
  Serial.println(t);

  Serial.print("EC: ");
  Serial.println(pec);

  Serial.print("BTR: ");
  Serial.println(pbtr);

  Serial.print("gy ");
  Serial.print(pgx);
  Serial.print("/");
  Serial.print(pgy);
  Serial.print("/");
  Serial.print(pgz);
  Serial.print(" accr ");
  Serial.print(pax);
  Serial.print("/");
  Serial.print(pay);
  Serial.print("/");
  Serial.println(paz);

  Serial.print("Lat: ");
  Serial.print(pLt);
  Serial.print(" ");
  Serial.print("Lng ");
  Serial.println(pLg);


  datapayload = "top"+btr+ttemp+ec+Lt+Lg+dd+mm+yy+hh+mn+ss+ax+ay+az+gx+gy+gz+"down"; //55byte
  //Serial.println(idata);
  //sdata = btr+S+temp+S+ec+S+Lt+S+Lg+S+dd+S+mm+S+yy+S+hh+S+mn+S+ss+S+ax+S+ay+S+az+S+gx+S+gy+S+gz;
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}
