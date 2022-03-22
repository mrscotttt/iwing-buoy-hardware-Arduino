#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define nss 18
#define rst 23
#define dio0 26

const char* ssid = "Scotttty";
const char* password = "11111111";
String serverName = "https://iwing-buoyserver-beta.vercel.app/s";
const long frequency = 450E6;
String gatewayid = "09";
String income;
bool sv=false;
char *nodelist[6] = {"01", "02", "03", "04", "05", "09"};

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);
  LoRa.setPins(nss, rst, dio0);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125000);
  LoRa.setCodingRate4(8);
  LoRa.setSyncWord(0x34);

  //wifi
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    //delay(500);
    //Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to WiFi network with IP Address: ");
}
String totheweb;
void loop(){
    if(sv){
         generate();
         HTTPClient client;
         client.begin(serverName);
         client.addHeader("Content-Type", "application/json");
         int httpResponseCode = client.POST(totheweb);
         Serial.print("[gateway] Send to Server > ");
         Serial.print("HTTP Response code: ");
         Serial.println(httpResponseCode);
         client.end();
         sv=false;
    }  
}
  
void LoRa_rxMode(){
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();                // active invert I and Q signals
}

void onReceive(int packetSize) {
  while (LoRa.available()) {
    String message = LoRa.readString();
    String fc1 = "1";
    String fc3 = "3";
    String fc2 = "2";
    String fc4 = "4";
    String broadgw= "00";
    String t = "top";
    String d = "down";
    
    //got trigger
    if(message.substring(2,4)==broadgw and message.substring(4,5)==fc1){
      for (int i = 0; i < 6; i++){
        String x = String(nodelist[i]);
        if(message.substring(0,2)==x){
          Serial.println("This node can truth.");
          Serial.print("[<-node");
          Serial.print(message.substring(0,2));
          Serial.print("] ");
          Serial.print("got req from node ");
          Serial.println(message.substring(0,2));
          String callback = gatewayid+message.substring(0,2)+fc2;
          LoRa_sendMessage(callback); // send a trigger
          Serial.print("[->node");
          Serial.print(message.substring(0,2));
          Serial.print("] ");
          Serial.println("send accept message already.");
        }
      }
    }
    
    //got data
    if(message.substring(2,4)==gatewayid and message.substring(4,5)==fc3 and message.substring(5,8)==t and message.substring(66,70)==d and sv == false){
      Serial.print("[<-node");
      Serial.print(message.substring(0,2));
      Serial.print("] ");
      Serial.print("got data from node");
      Serial.print(message.substring(0,2));
      Serial.print(" data:");
      Serial.println(message.substring(5,70));
      String callback = gatewayid+message.substring(0,2)+fc4;
      LoRa_sendMessage(callback); // send a trigger
      Serial.print("[->node");
      Serial.print(message.substring(0,2));
      Serial.print("] ");
      Serial.println("This data is OK thanks!");

      //generate data bit 8-66
      //String message = "03093top3124135261384052910057523214202021213236382630482411554636down";
                               //top41308121384052910057523237222021173117500500510500500500down
      income = (message.substring(0,70));
      sv=true;
    }
  }
}

void generate(){
      //id
      String buoyid = (income.substring(0,2));
      //btr
      float btr = ((income.substring(8,10)).toFloat())/10;
      //temp
      float tmp = ((income.substring(10,14)).toFloat())/100;
      //ec
      String ec = (income.substring(14,17));
      //lat
      String lt = income.substring(17,19)+"."+income.substring(19,25);
      //lg
      String lg = income.substring(25,28)+"."+income.substring(28,34);
      //date + time
      int dd = ((income.substring(34,36)).toInt())-10;
      int mm = ((income.substring(36,38)).toInt())-10;
      String yy = income.substring(38,42);
      int hh = ((income.substring(42,44)).toInt())-10;
      int mn = ((income.substring(44,46)).toInt())-10;
      int ss = ((income.substring(46,48)).toInt())-10;
      //gyro
      int gx = ((income.substring(48,51)).toInt())-500;
      int gy = ((income.substring(51,54)).toInt())-500;
      int gz = ((income.substring(54,57)).toInt())-500;
      int ax = ((income.substring(57,60)).toInt())-500;
      int ay = ((income.substring(60,63)).toInt())-500;
      int az = ((income.substring(63,66)).toInt())-500;
      //totheweb = "{\"buoyid\":\"" + buoyid + "\"}";
      totheweb = "{\"buoyid\":\"" + buoyid + "\",\"battery\":" + String(btr) + ",\"temp\":" + String(tmp) + ",\"ec\":" + ec + ",\"lat\":" + lt + ",\"long\":" + lg + ",\"dd\":" + String(dd) + ",\"mm\":" + String(mm) + ",\"yy\":" + String(yy) + ",\"hh\":" + String(hh) + ",\"mn\":" + String(mn) + ",\"ss\":" + String(ss) + ",\"gx\":" + String(gx) + ",\"gy\":" + String(gy) + ",\"gz\":" + String(gz)  + ",\"ax\":" + String(ax) + ",\"ay\":" + String(ay) + ",\"az\":" + String(az) + "}";
      Serial.print("[gateway] convert to JSON : ");
      Serial.println(totheweb);
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onTxDone() {
  //Serial.println("TxDone");
  LoRa_rxMode();
}
