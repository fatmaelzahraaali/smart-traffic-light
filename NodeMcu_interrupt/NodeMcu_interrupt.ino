#include <ESP8266WiFi.h>
/*
#include <ArduinoJson.h>
#include <FirebaseArduino.h>
*/
#include <FirebaseESP8266.h>

#define FIREBASE_HOST "trafficgraduate-ae3e0-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "iyEno7j4WJcM2rVMcheqXf3chUJZ3O6yMq6TX6k9"
/*
#define SSID_NAME "Fatima"
#define SSID_PASSWORD "Aa123455"*/

/*
#define SSID_NAME "WED74117"
#define SSID_PASSWORD "RWADN_22*"*/

/*
#define SSID_NAME "Wael"
#define SSID_PASSWORD "01221404350"*/


#define SSID_NAME "A23"
#define SSID_PASSWORD "ahmed12345"

FirebaseData firebaseData;

String result;
String Err;
String Fog;

char ErrFlag = 0;
String test;
char c = 0;

#define ERR_Clear_Pin 5
#define Fog_ON 4

#define CAM1_Pin 14
#define CAM2_Pin 12
#define CAM3_Pin 13
#define CAM4_Pin 15


ICACHE_RAM_ATTR void CAM1() {
  c=1;
}
ICACHE_RAM_ATTR void CAM2() {
  c=2;
}
ICACHE_RAM_ATTR void CAM3() {
  c=3;
}
ICACHE_RAM_ATTR void CAM4() {
  c=4;
}



void setup() {

  Serial.begin(9600);
  pinMode(CAM1_Pin, INPUT_PULLUP);
  pinMode(CAM2_Pin, INPUT_PULLUP);
  pinMode(CAM3_Pin, INPUT_PULLUP);
  pinMode(CAM4_Pin, INPUT_PULLUP);

  pinMode(ERR_Clear_Pin, OUTPUT);
  digitalWrite(ERR_Clear_Pin, LOW);
  pinMode(Fog_ON, OUTPUT);
  digitalWrite(Fog_ON, LOW);

  attachInterrupt(digitalPinToInterrupt(CAM1_Pin), CAM1, RISING);
  attachInterrupt(digitalPinToInterrupt(CAM2_Pin), CAM2, RISING);
  attachInterrupt(digitalPinToInterrupt(CAM3_Pin), CAM3, RISING);
  attachInterrupt(digitalPinToInterrupt(CAM4_Pin), CAM4, RISING);

  WiFi.disconnect();
  WiFi.begin(SSID_NAME, SSID_PASSWORD);
  Serial.println("Connecting....");

  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected");
  Serial.println(WiFi.localIP());
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);  // connect to firebase
  Firebase.reconnectWiFi(true);

  delay(10000);
}

void loop() {

  /************************** GET THE RESULT **********************************/
  if (ErrFlag == 0) {
    if (Firebase.getString(firebaseData, "LED_STATUS")) {
      result = firebaseData.stringData();
      Serial.print(result);
      delay(500);
    }
  }

  /************************** CHECK FOR ERROR VAR CLEARED***********************/
  if (ErrFlag == 1) {
    if (Firebase.getString(firebaseData, "ERROR")) {
      Err = firebaseData.stringData();
      if (Err != "CAM 1" && Err != "CAM 2" && Err != "CAM 3" && Err != "CAM 4") {
        digitalWrite(ERR_Clear_Pin, HIGH);
        delay(100);
        digitalWrite(ERR_Clear_Pin, LOW);
        ErrFlag = 0;
      }
    }
  }

  /************************** UPDATE ERR VAR **********************************/
  if (c == 1) {
    Serial.println("CAM 1");
    Firebase.setString(firebaseData, "ERROR", "CAM 1");
    ErrFlag = 1;
    c = 0;

  }

  if (c == 2) {
        Serial.println("CAM 2");
    Firebase.setString(firebaseData, "ERROR", "CAM 2");
    ErrFlag = 1;
    c = 0;
  }

  if (c == 3) {
     Serial.println("CAM 3");
    Firebase.setString(firebaseData, "ERROR", "CAM 3");
    ErrFlag = 1;
    c = 0;
  }

  if (c == 4) {
    Serial.println("CAM 4");
    Firebase.setString(firebaseData, "ERROR", "CAM 4");
    ErrFlag = 1;
    c = 0;
  }
}
