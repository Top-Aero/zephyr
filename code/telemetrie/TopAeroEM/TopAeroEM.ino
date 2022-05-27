/*
  Auteurs: Nicolas Armogathe, Théo TUGAYE, pour l'association TopAero - Sorbonne Université  
  - Mesures d'accélération, altitude, position GPS
  - Transmission radio des données
  - Enregistrement des données sur carte uSD
  projet actuel: Zéphyr
  Libre de droit
  */
  
#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

#include <Wire.h>
#include <SoftwareSerial.h>

static const int RXPin = 0, TXPin = 1;      /* GPS */
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin);

#include <Adafruit_Sensor.h>  //librairie Adafruit générale des capteurs 
#include <Adafruit_BNO055.h>  //librairie du module inertiel /* Accélération */
#include <utility/imumaths.h> //librairie pour le fonctionnement du module inertiel 

#include <Adafruit_GPS.h> //librairie Adafruit du module GPS /* GPS */

int ledState=LOW;
unsigned long preMillis=0;

#define BNO055_SAMPLERATE_DELAY_MS (5)        /* Accélération */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); //by default address is 0x29 or 0x28 /* Accélération */

                      /*Pression*/
float tension1, P1, tension2, P2, tension3, P3, tension4, P4, tension5, P5, tension6, P6, tension7, P7, tension8, P8, tension9, P9, tension10, P10, tension11, P11, tension12, P12, tension13, P13, tension14, P14;//Déclaration des variables tension et pression

bool first=true;      /* Altitude */
float altm0=0.0;
#define ALTMODE; //comment out for barometer mode; default is altitude mode 
const int SENSORADDRESS = 0x60; // address specific to the MPL3115A1, value found in datasheet 
float altsmooth = 0; //for exponential smoothing          
byte IICdata[5] = {0,0,0,0,0}; //buffer for sensor data 
float firstalt=0.0;

#define mySerial Serial1      /* GPS */
Adafruit_GPS GPS(&mySerial);
bool firstGPS;

int comdata=0;      /* Radio */
int data_lenght;
String comstr;
char* frame;
SoftwareSerial lora(9,10);

#include <SD.h>       /* uSD */
#include <SPI.h>      
File myFile;
File tableau;
const int chipSelect = BUILTIN_SDCARD;
int i=0; //Compteur du nombre de mesures

// @mib
float tab_acl[3] = {0, 0, 0};

void setup() {
  Wire.begin(); //join i2c bus       
  Serial.begin(115200);//ouverture port série 1 USB
  pinMode(13,OUTPUT); //LED
  Serial2.begin(115200);//ouverture port série 2 pour radio
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
                                                  /* GPS */
                                                  
  GPS.begin(9600); //baudrate obligé du module
  ss.begin(GPSBaud);
  // this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); //taux de rafraichissement 10Hz, en réalité il n'est pas représentatif 
  //du rafraichissement réel, mais il vaut mieux entrer la commande tout de même
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
                                              /* Accélération */
  /* Initialise the sensor */
 /*  if(!bno.begin())
  {
    Serial.print("Couldnt find accelerator sensor");
    while(1);
  }*/

  bno.setExtCrystalUse(true);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
                                                /* Altitude */       
  IIC_Write(0x2D,0); //write altitude offset=0 (because calculation below is based on offset=0)
  //Altitude mode
  IIC_Write(0x26, 0b10111011); //bit 2 is one shot mode //0xB9 = 0b10111001
  IIC_Write(0x26, 0b10111001); //must clear oversampling (OST) bit, otherwise update will be once per second
  delay(1000); //wait for measurement
  IIC_ReadData(); //
  firstalt=Alt_Read();
  altsmooth=Alt_Read()-firstalt;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
                                                 /* Radio */ 
  lora.begin(115200); //baudrate obligé du module
  delay(20);
  lora.println("AT+BAND=869650000\r\n"); //commande pour appliquer la fréquence d'émission ou réception du module
  //ici 869,65Mhz
  delay(20);
  lora.println("AT+PARAMETER=7,9,4,4\r\n"); //par défaut = 12,7,1,4 
  //Spreading factor = 7, Bandwidth = 9 (500kHz), Coding rate = 4, Programmed Preamble = 4
  delay(2000);
  Serial.println("Communication prête");
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
                                                 /* uSD */
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  myFile = SD.open("zephyr.txt", FILE_WRITE);
  if (myFile) {
    myFile.println("Zephyr_new_stream");
    myFile.close();
  } else {
    Serial.println("error opening zephyr.txt");
  }

    /* Ouvre le fichier csv en écriture */
  Serial.println(F("Ouverture du fichier csv ... "));
  tableau = SD.open("zephyr.csv", FILE_WRITE);
  if (!tableau) {
    Serial.println(F("Erreur : Impossible d'ouvrir le fichier de sortie"));
    Serial.println(F("Verifiez la carte SD et appuyez sur le bouton RESET"));
    for (;;); // Attend appui sur bouton RESET
  }
  /* Ajoute l'entête CSV si le fichier est vide */
  if (tableau.size() == 0) {
    Serial.println(F("Ecriture de l'entete CSV ..."));
    tableau.flush();
  }
  delay(2000);
}//fin setup



uint32_t timer = millis();

bool alt_must_adjust = true; //condition pour ajuster l'altitude au bout de 1min

void loop() {
 
  String framestr;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
                                      /*Pression*/

  tension2 = analogRead(A16) * 5.0 / 1023.0;
  P2 = (tension2 / 0.002 - 39.75);
  Serial.println("P2 :");
  Serial.print(P2, 0);
  Serial.println(" hPa");
  
  tension4 = analogRead(A18) * 5.0 / 1023.0;
  P4 = (tension4 / 0.002 - 34.75);
  Serial.println("P4 :");
  Serial.print(P4, 0);
  Serial.println(" hPa");
  
  tension5 = analogRead(A14) * 5.0 / 1023.0;
  P5 = (tension5 / 0.002 - 48.75);
  Serial.println("P5 :");
  Serial.print(P5, 0);
  Serial.println(" hPa");
  
  tension6 = analogRead(A15) * 5.0 / 1023.0;
  P6 = (tension6 / 0.002 - 44.75);
  Serial.println("P6 :");
  Serial.print(P6, 0);
  Serial.println(" hPa");
  
  tension7 = analogRead(A17) * 5.0 / 1023.0;
  P7 = (tension7 / 0.002 - 39.75);
  Serial.println("P7 :");
  Serial.print(P7, 0);
  Serial.println(" hPa");
  
  tension8 = analogRead(A9) * 5.0 / 1023.0;
  P8 = (tension8 / 0.002 - 53.75);
  Serial.println("P8 :");
  Serial.print(P8, 0);
  Serial.println(" hPa");
  
  tension9 = analogRead(A8) * 5.0 / 1023.0;
  P9 = (tension9 / 0.002 - 24.75);
  Serial.println("P9 :");
  Serial.print(P9, 0);
  Serial.println(" hPa");
  
  tension10 = analogRead(A7) * 5.0 / 1023.0;
  P10 = (tension10 / 0.002 - 29.75);
  Serial.println("P10 :");
  Serial.print(P10, 0);
  Serial.println(" hPa");
  
  tension11 = analogRead(A6) * 5.0 / 1023.0;
  P11 = (tension11 / 0.002 - 31.75);
  Serial.println("P11 :");
  Serial.print(P11, 0);
  Serial.println(" hPa");
  
  tension12 = analogRead(A3) * 5.0 / 1023.0;
  P12 = (tension12 / 0.002 - 36.75);
  Serial.println("P12 :");
  Serial.print(P12, 0);
  Serial.println(" hPa");

  tension13 = analogRead(A2) * 5.0 / 1023.0;
  P13 = (tension13 / 0.002 - 53.75);
  Serial.println("P13 :");
  Serial.print(P13, 0);
  Serial.println(" hPa");

  tension14 = analogRead(A1) * 5.0 / 1023.0;//Lecture de la tension et conversion en V
  P14 = (tension14 / 0.002 - 49.75); //Obtention de la pression en hPa => valeur 93 = offset , a changer pour chaque capteur 
  //Affichage de la pression sur le3moniteur série
  Serial.println("P14 :");
  Serial.print(P14, 0);
  Serial.println(" hPa");
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////  
                                                      /* GPS */
  firstGPS=true;
  while (ss.available() > 0){
  char c = GPS.read();
  char* frame = GPS.lastNMEA();
  GPS.parse(frame);
    if(firstGPS){
      Serial.println(frame);
      firstGPS=not(firstGPS);
      framestr=String(frame);
   
    }
  }
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  unsigned long ms = millis();                     /* LED blink */
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
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
                                              /* Accélération (m/s^2) */
  // mib note : plus besoin a compter du 2021-07-20.
  //Serial.print(accelModule);
  //Serial.println("");
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
                                                  /* Altitude (m) */ 
  float altitudeSmoothed=altitude_read_data()-altm0;//soustraire altm0 pour commencer à l'altitude 0
  if(first==true){                            // +/- 1m de précision
    if(ms>60000){ //on attend 1min pour avoir une valeur stable d'altitude
      altm0=altitudeSmoothed;
      first=false;
    }
  }
  
  Serial.print(ms);
  Serial.println("");
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
                                                  /* Radio */
                                                  
  String mes=" alt="+String(altitudeSmoothed)+" acx="+String(tab_acl[0])+" acy="+String(tab_acl[1])+" acz="+String(tab_acl[2])+" tps="+String(millis())+" gps="+framestr;
  
  String datas=mes+" P11="+String(P11)+" P12="+String(P12);
  datas=datas.trim();
  String cmd="AT+SEND=0,"+String(mes.length()) +","+ datas +"\r\n";
  lora.println(cmd);
  while(lora.available()){
    Serial.write(lora.read());
  }
  Serial.println();
  Serial.println(cmd);
  delay(35); //le délai influe sur la transmission radio, si trop court il y a une probabilité +/- importante 
  //que les messages soient coupés

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
                                                      /* uSD */
  myFile = SD.open("zephyr.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("Mesure ");
    myFile.println(i);
    myFile.print("\nTemps = ");
    myFile.println(String(millis()));
    myFile.print("Altitude = ");
    myFile.println(String(altitudeSmoothed));
    myFile.print("Accéleration en X = ");
    myFile.println(String(tab_acl[0]));
    myFile.print("Accéleration en Y = ");
    myFile.println(String(tab_acl[1]));
    myFile.print("Accéleration en Z = ");
    myFile.println(String(tab_acl[2]));
    //myFile.println(String(accelModule));
    myFile.print("Coordonnées GPS = ");
    myFile.println(framestr);
    myFile.print("Pression 2 = "); myFile.println(String(P2));
    myFile.print("Pression 4 = "); myFile.println(String(P4));
    myFile.print("Pression 5 = "); myFile.println(String(P5));
    myFile.print("Pression 6 = "); myFile.println(String(P6));
    myFile.print("Pression 7 = "); myFile.println(String(P7));
    myFile.print("Pression 8 = "); myFile.println(String(P8));
    myFile.print("Pression 9 = "); myFile.println(String(P9));
    myFile.print("Pression 10 = "); myFile.println(String(P10));
    myFile.print("Pression 11 = "); myFile.println(String(P11));
    myFile.print("Pression 12 = "); myFile.println(String(P12));
    myFile.print("Pression 13 = "); myFile.println(String(P13));
    myFile.print("Pression 14 = "); myFile.println(String(P14));
    myFile.print("\n");
    myFile.close();
  }
  tableau = SD.open("zephyr.csv", FILE_WRITE);
  if (tableau) {
    tableau.print(i);
    tableau.print(";"); 
    tableau.print(millis());
    tableau.print(";"); 
    tableau.print(altitudeSmoothed);
    tableau.print(";");
    tableau.print(tab_acl[0]);
    tableau.print(";");
    tableau.print(tab_acl[1]);
    tableau.print(";");
    tableau.print(tab_acl[2]);
    tableau.print(";");
    tableau.print(framestr);
    tableau.print(";");
    tableau.print(P2);
    tableau.print(";");
    tableau.print(P4);
    tableau.print(";");
    tableau.print(P5);
    tableau.print(";");
    tableau.print(P6);
    tableau.print(";");
    tableau.print(P7);
    tableau.print(";");
    tableau.print(P8);
    tableau.print(";");
    tableau.print(P9);
    tableau.print(";");
    tableau.print(P10);
    tableau.print(";");
    tableau.print(P11);
    tableau.print(";");
    tableau.print(P12);
    tableau.print("\n");
    tableau.print(P13);
    tableau.print(";");
    tableau.print(P14);
    tableau.print("\n");
    tableau.close();
  }
  i=i+1;
 delay(200);
}//fin loop

///////////////////////////////////////////////////////////////////////////////////////////////////////
                                                  /* Accélération */
void accel_read_data(float tab_acl[3]){
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  /* Vecteur contenant les composantes des accélérations */
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  /*float mod_accel=sqrt(pow(accelerometer.x(),2)+pow(accelerometer.y(),2)+pow(accelerometer.z(),2))-9.80665;
  if (mod_accel<0){
   mod_accel=0;
  }*/
  /*
  Serial.print("Acceleration:");
  Serial.println(accelerometer.x());
  Serial.println(accelerometer.y());
  Serial.println(accelerometer.z());
  delay(BNO055_SAMPLERATE_DELAY_MS);*/

  tab_acl[0] = accelerometer.x();
  tab_acl[1] = accelerometer.y();
  tab_acl[2] = accelerometer.z();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    /* Altitude */
                                      //by Henry Lahr, 2013-02-27, libre de droit
float altitude_read_data(){
  // This function reads the altitude (or barometer) and temperature registers, then prints their values
  // variables for the calculations
  float altbaro;
  //One shot mode at 0b10101011 is slightly too fast, but better than wasting sensor cycles that increase precision
  //one reading seems to take 4ms (datasheet p.33);
  //oversampling at 32x=130ms interval between readings seems to be optimal for 10Hz
  //#ifdef ALTMODE //Altitude mode
    IIC_Write(0x26, 0b10111011); //bit 2 is one shot mode //0xB9 = 0b10111001
    IIC_Write(0x26, 0b10111001); //must clear oversampling (OST) bit, otherwise update will be once per second
  //#else //Barometer mode
    //IIC_Write(0x26, 0b00111011); //bit 2 is one shot mode //0xB9 = 0b10111001
    //IIC_Write(0x26, 0b00111001); //must clear oversampling (OST) bit, otherwise update will be once per second
  //#endif
  //delay(20); //read with 10Hz; drop this if calling from an outer loop
  IIC_ReadData(); //reads registers from the sensor
  #ifdef ALTMODE //converts byte data into float; change function to Alt_Read() or Baro_Read()
    altbaro = Alt_Read()-firstalt;
  #endif
  //altsmooth=(altsmooth*3+altbaro)/4; //exponential smoothing to get a smooth time series, pas bon pour des changements brutaux
  //Serial.print(altbaro); // in meters 
  //Serial.print(",");
  //Serial.print(altsmooth); // exponentially smoothed
  //Serial.println("");
  Serial.print("Altitude:");
  Serial.print(altbaro); //en m
  Serial.println("");
  return(altbaro-0.3);//+0,3 pour compenser la dérive du capteur dû au one shot mode
}

float Alt_Read(){
  //Reads altitude data (if CTRL_REG1 is set to altitude mode)
  int m_altitude = IICdata[0];
  int c_altitude = IICdata[1];
  float l_altitude = (float)(IICdata[2]>>4)/16;
  return((float)((m_altitude << 8)|c_altitude) + l_altitude);
}
 
byte IIC_Read(byte regAddr){
  // This function reads one byte over I2C
  Wire.beginTransmission(SENSORADDRESS);
  Wire.write(regAddr); // Address of CTRL_REG1
  Wire.endTransmission(false); // Send data to I2C dev with option for a repeated start. Works in Arduino V1.0.1
  Wire.requestFrom(SENSORADDRESS, 1);
  return Wire.read();
}
 
void IIC_ReadData(){  //Read Altitude/Barometer and Temperature data (5 bytes)
  //This is faster than reading individual register, as the sensor automatically increments the register address,
  //so we just keep reading...
  byte i=0;
  Wire.beginTransmission(SENSORADDRESS);
  Wire.write(0x01); // Address of CTRL_REG1
  Wire.endTransmission(false);
  Wire.requestFrom(SENSORADDRESS,5); //read 5 bytes: 3 for altitude or pressure, 2 for temperature
  while(Wire.available()) IICdata[i++] = Wire.read();
}
 
void IIC_Write(byte regAddr, byte value){
  // This function writes one byto over I2C
  Wire.beginTransmission(SENSORADDRESS);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission(true);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    /* Radio */
void send_data(float sensorvalue, int valuelength){
  String message;
  message=message+"AT+SEND=0"+","+valuelength+","+sensorvalue+"\r";
  Serial.println(message);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void _softRestart(){ //restart par logiciel si besoin, uniquement pour Teensy 3.x
  Serial.end();  //clears the serial monitor  if used
  SCB_AIRCR = 0x05FA0004;  //write value for restart
}
