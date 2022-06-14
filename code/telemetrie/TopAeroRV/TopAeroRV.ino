/*
  Auteur:  Théo TUGAYE, Nicolas Armogathe, pour l'association TopAero - Sorbonne Université  
  - Réception des données 
  Libre de droit
  */
  
#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial lora(0,1);   /* Radio */

int val=0;

int ledState=LOW;
unsigned long preMillis=0;

void setup() {
  Wire.begin();
  pinMode(13,OUTPUT);
  Serial.begin(115200);
  
  lora.begin(115200);     /* Radio */
  lora.println("AT+BAND=869650000\r\n"); //commande pour appliquer la fréquence d'émission ou réception du module
  delay(200);
  lora.println("AT+PARAMETER=7,9,4,4\r\n"); //par défaut = 12,7,1,4 
  //Spreading factor = 7, Bandwidth = 9 (500kHz), Coding rate = 4, Programmed Preamble = 4
  
  delay(2000);
}

void loop() {
  /* LED blink */
  unsigned long ms=millis();   
  if((ms-preMillis)>=250){
    preMillis=ms;
    if(ledState==LOW){
      ledState=HIGH;
    }
    else{
      ledState=LOW;
    }
    digitalWrite(13,ledState);
  }

  /* Radio */
  String inString;     
  while(lora.available()){
    if(lora.available()){
      inString+=String(char(lora.read()));
    }
  }
  //Serial.println(inString);
  if(inString.length()>0){
    String val;
    val=inString;//on peut substring la chaine si besoin, substring(9,12);
    Serial.print(val);
  }
  
}
