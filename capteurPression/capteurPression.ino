#include <RFduinoBLE.h>
#define SFR_DROIT 2
#define SFR_GAUCHE 3


char ble_buf[20];
bool sensorHDActivated = false ;
bool sensorHGActivated = false ;
bool useTwoSensors = true ;
float valueMax = 700 ;

int threshold = 30 ;

float toSend = 0;

void BLE_sendString(char *str){  
  int length = strlen(str);

  if( length <= 20){
    RFduinoBLE.send(str,length);
  }else{
    RFduinoBLE.send(str,20);  // BLE packet is 20 bytes max.
    Serial.print("XXXXX Error");
  }
}

void RFduinoBLE_onReceive(char *data, int len) {
//We don't care about what we receive, it's either 2 sensors or one

useTwoSensors = !useTwoSensors ;
//Serial.println(data[1]);
Serial.println(len);
  if (len > 0) {
    char mydata[len + 1];
    memcpy(mydata, data, len);
    mydata[len] = 0;
    Serial.println(mydata);
  }
}

void setup() {
  //** Initialisation du serial port pour la communication avec le pc **//
  Serial.begin(9600);

  // start the BLE stack
  RFduinoBLE.begin();
}

void loop() {
  toSend = 0.0 ;
  // Échantillonnage de l'émission en ms
  //RFduino_ULPDelay(0.1);
  RFduino_ULPDelay(1000);
  
  int sensorValueDroit = analogRead(SFR_DROIT);
  int sensorValueGauche = analogRead(SFR_GAUCHE);
 
  Serial.print(sensorValueDroit);
  Serial.print(" - ");
  Serial.println(sensorValueGauche);
  //Serial.print(sensorValueBasGauche);
  //Serial.print(" - ");
  //Serial.println(sensorValueBasDroit);

  if(sensorValueGauche > threshold){
    sensorHGActivated = true ;
    toSend+=sensorValueGauche/valueMax ;
  }

  if(sensorValueDroit > threshold && useTwoSensors){
    sensorHDActivated = true ;
    toSend+=sensorValueDroit/valueMax ;
  }
  
  if(sensorHDActivated || sensorHGActivated){
    Serial.print("Value sent = ");
    Serial.println(toSend);
  
    //RFduinoBLE.sendFloat(1.111000);
    RFduinoBLE.sendFloat(toSend);
  }

  sensorHGActivated = false ;
  sensorHGActivated = false ;
  
  
  //float voltage = sensorValue * (3.3 / 1023.0);
  //Serial.println(sensorValue);

  //RFduinoBLE.advertisementData = "1";
  //RFduinoBLE.sendInt(876);

  

  /*char cc[10] ;
  String(sensorValueHautDroit).toCharArray(cc, 10);
  strcpy(ble_buf, "2");
  strcat(ble_buf,",");
  strcat(ble_buf,cc);
  BLE_sendString(ble_buf);
  

  // send the sample via ble
  RFduinoBLE.advertisementData = "2";
  RFduinoBLE.sendInt(sensorValueHautDroit);

  // send the sample via ble
  RFduinoBLE.advertisementData = "3";
  RFduinoBLE.sendInt(sensorValueHautGauche);

  // send the sample via ble
  RFduinoBLE.advertisementData = "4";
  RFduinoBLE.sendInt(sensorValueBasGauche);

  // send the sample via ble
  RFduinoBLE.advertisementData = "5";
  RFduinoBLE.sendInt(sensorValueBasDroit);*/
}


