/*
  Auteur: Théo TUGAYE, pour l'association TopAero - Sorbonne Université  
  - Mesures d'accélération, altitude, position GPS
  - Transmission radio des données
  - Enregistrement des données sur carte uSD
  projet actuel: Zéphyr
  Libre de droit

  Organisation du code :
  |--TopAeroTX.h
  |  |--Bibliothèques
  |  |--Définitions
  |  |--Déclarations globales
  |  |--Déclarations des structures
  |  |--Déclaration des fonctions
  |--TopAeroTX.cpp
  |  |--Initialisation des tâches
  |  |--Tâches
  |  |--Fonctions
  |  |--Mains
*/

#include "TopAeroTX.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Initialisation des tâches                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

 
                                     /* Initialisation des pins */
void mise_en_place(){
  delay(200);
  pinMode(7, OUTPUT);
  pinMode(8, INPUT_PULLUP);
  Wire.setSDA(8); // Activation du mode alternatif du pin
  Wire.setSCL(7); // Activation du mode alternatif du pin                       
  Wire.begin(); //join i2c bus    
  Serial.begin(115200);//ouverture port série 1 USB
  pinMode(13,OUTPUT); //LED
  Serial2.begin(115200);//ouverture port série 2 pour radio
  GPS.begin(9600); //baudrate obligé du module
  ss.begin(GPSBaud);/* Initialise the sensor */
  lora.begin(115200); //baudrate obligé du module
  delay(20);
}
 
                                                /* GPS */
void setup_gps() {
  // this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //taux de rafraichissement 10Hz, en réalité il n'est pas représentatif 
  //du rafraichissement réel, mais il vaut mieux entrer la commande tout de même
}

                                                /*Pression*/
void setup_pression(struct pression_s* pression_s, int timer, unsigned long period){
  pression_s->timer = timer;
  pression_s->period = period;
}

                                            /* Accélération */
void setup_accel(struct accel_s* accel_s, int timer, unsigned long period){
  accel_s->timer = timer;
  accel_s->period = period;if(!bno.begin())
  {
    Serial.print("Couldnt find accelerator sensor");
    while(1);
  }
  bno.setExtCrystalUse(true);
}
 
                                              /* Altitude */    
void setup_alt(struct alt_s* alt_s, int timer, unsigned long period){
  alt_s->timer = timer;
  alt_s->period = period;
  IIC_Write(0x2D,0); //write altitude offset=0 (because calculation below is based on offset=0)
  //Altitude mode
  IIC_Write(0x26, 0b10111011); //bit 2 is one shot mode //0xB9 = 0b10111001
  IIC_Write(0x26, 0b10111001); //must clear oversampling (OST) bit, otherwise update will be once per second
  delay(1000); //wait for measurement
  IIC_ReadData(); //
  firstalt=Alt_Read();
  altsmooth=Alt_Read()-firstalt;
}

                                                /* Radio */ 
void setup_radio(struct radio_s* radio_s, int timer, unsigned long period){
  radio_s->timer = timer;
  radio_s->period = period;
  lora.println("AT+BAND=869650000\r\n"); //commande pour appliquer la fréquence d'émission ou réception du module
  //ici 869,65Mhz
  delay(200);
  lora.println("AT+PARAMETER=7,9,4,4\r\n"); //par défaut = 12,7,1,4 
  //Spreading factor = 7, Bandwidth = 9 (500kHz), Coding rate = 4, Programmed Preamble = 4
  delay(2000);
  Serial.println("Communication prête");
}

                                                /* uSD */
void setup_uSD(struct uSD_s* uSD_s, int timer, unsigned long period){
  uSD_s->timer = timer;
  uSD_s->period = period;
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
}

                                                /*debugging*/
void setup_debug(struct debug_s* debug_s, int timer, unsigned long period){
  debug_s->timer = timer;
  debug_s->period = period;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Tâches                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////

                                                  /*Pression*/
void loop_pression(struct pression_s* pression_s){
  if (!(waitFor(pression_s->timer,pression_s->period))) return;
  tension2 = analogRead(A21) * 5.0 / 1023.0;
  P2 = (tension2 / 0.002 - 39.75);
  //Serial.println("P2 :");
  //Serial.print(P2, 0);
  //Serial.println(" hPa");
  
  tension4 = analogRead(A19) * 5.0 / 1023.0;
  P4 = (tension4 / 0.002 - 34.75);
  //Serial.println("P4 :");
  //Serial.print(P4, 0);
  //Serial.println(" hPa");
  
  tension5 = analogRead(A9) * 5.0 / 1023.0;
  P5 = (tension5 / 0.002 - 48.75);
  //Serial.println("P5 :");
  //Serial.print(P5, 0);
  //Serial.println(" hPa");
  
  tension6 = analogRead(A8) * 5.0 / 1023.0;
  P6 = (tension6 / 0.002 - 44.75);
  //Serial.println("P6 :");
  //Serial.print(P6, 0);
  //Serial.println(" hPa");
  
  tension7 = analogRead(A7) * 5.0 / 1023.0;
  P7 = (tension7 / 0.002 - 39.75);
  //Serial.println("P7 :");
  //Serial.print(P7, 0);
  //Serial.println(" hPa");
  
  tension8 = analogRead(A0) * 5.0 / 1023.0;
  P8 = (tension8 / 0.002 - 53.75);
  //Serial.println("P8 :");
  //Serial.print(P8, 0);
  //Serial.println(" hPa");
  
  tension9 = analogRead(A1) * 5.0 / 1023.0;
  P9 = (tension9 / 0.002 - 24.75);
  //Serial.println("P9 :");
  //Serial.print(P9, 0);
  //Serial.println(" hPa");
  
  tension10 = analogRead(A2) * 5.0 / 1023.0;
  P10 = (tension10 / 0.002 - 29.75);
  //Serial.println("P10 :");
  //Serial.print(P10, 0);
  //Serial.println(" hPa");
  
  tension11 = analogRead(A3) * 5.0 / 1023.0;
  P11 = (tension11 / 0.002 - 31.75);
  //Serial.println("P11 :");
  //Serial.print(P11, 0);
  //Serial.println(" hPa");
  
  tension12 = analogRead(A4) * 5.0 / 1023.0;
  P12 = (tension12 / 0.002 - 36.75);
  //Serial.println("P12 :");
  //Serial.print(P12, 0);
  //Serial.println(" hPa");

  tension13 = analogRead(A5) * 5.0 / 1023.0;
  P13 = (tension13 / 0.002 - 53.75);
  //Serial.println("P13 :");
  //Serial.print(P13, 0);
  //Serial.println(" hPa");

  tension14 = analogRead(A6) * 5.0 / 1023.0;//Lecture de la tension et conversion en V
  P14 = (tension14 / 0.002 - 49.75); //Obtention de la pression en hPa => valeur 93 = offset , a changer pour chaque capteur 
  //Affichage de la pression sur le3moniteur série
  //Serial.println("P14 :");
  //Serial.print(P14, 0);
  //Serial.println(" hPa");
}

                                                    /* GPS */
void loop_gps(struct gps_s* gps_s){
  while (ss.available() > 0){
    GPS.read();
    if (GPS.newNMEAreceived()) {
      char* frame = GPS.lastNMEA();
      GPS.parse(frame);
      if(GPS.fix){
        gps_s->framestr=String(frame);
        gps_s->lon=String(GPS.longitudeDegrees);
        gps_s->lat=String(GPS.latitudeDegrees);
      }else{
        gps_s->framestr=String(frame);
        gps_s->lon="waiting for fix";
        gps_s->lat="waiting for fix";
      }
    }
  }
}

                                                   /* LED blink */
void loop_led(struct time_s* time_s){
  if((time_s->ms-preMillis)>=250){
    preMillis=time_s->ms;
    if(ledState==LOW){
      ledState=HIGH;
    }
    else{
      ledState=LOW;
    }
    digitalWrite(13,ledState);
  }
}

                                                /* Accélération (m/s^2) */
void loop_accel(struct accel_s* accel_s){
  if (!(waitFor(accel_s->timer,accel_s->period))) return;
  accel_read_data();
}

                                                  /* Altitude (m) */ 
void loop_alt(struct time_s* time_s, struct alt_s* alt_s){
  if (!(waitFor(alt_s->timer,alt_s->period))) return;
  alt_s->altitude=altitude_read_data()-altm0;//soustraire altm0 pour commencer à l'altitude 0
  if(first==true){                            // +/- 1m de précision
    if(time_s->ms>10000){ //on attend 1min pour avoir une valeur stable d'altitude
      altm0=alt_s->altitude;
      first=false;
    }
  }
}

                                                      /* Radio */
void loop_radio(struct gps_s* gps_s, struct alt_s* alt_s,struct radio_s* radio_s){
  if (!(waitFor(radio_s->timer,radio_s->period))) return;
  String mes=String(alt_s->altitude)+";"+String(tab_acl[0])+";"+String(tab_acl[1])+";"+String(tab_acl[2])+";"+String(millis())+";"+gps_s->lon+";"+gps_s->lat;
  String datas=mes+";"+String(P12)+";"+String(P13);
  datas=datas.trim();
  String cmd="AT+SEND=0,"+String(mes.length()) +","+ datas +"\r\n";
  while(lora.available()){
    Serial.write(lora.read());
  }
  delay(35); //le délai influe sur la transmission radio, si trop court il y a une probabilité +/- importante 
  //que les messages soient coupés
}

                                                      /* uSD */
void loop_uSD(struct gps_s* gps_s, struct alt_s* alt_s,struct uSD_s* uSD_s){
  if (!(waitFor(uSD_s->timer,uSD_s->period))) return;
  myFile = SD.open("zephyr.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("Mesure ");
    myFile.println(i);
    myFile.print("\nTemps = ");
    myFile.println(String(millis()));
    myFile.print("Altitude = ");
    myFile.println(String(alt_s->altitude));
    myFile.print("Accéleration en X = ");
    myFile.println(String(tab_acl[0]));
    myFile.print("Accéleration en Y = ");
    myFile.println(String(tab_acl[1]));
    myFile.print("Accéleration en Z = ");
    myFile.println(String(tab_acl[2]));
    //myFile.println(String(accelModule));
    myFile.print("Coordonnées GPS = ");
    myFile.println(gps_s->framestr);
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
     printcsv(tableau,i);
     printcsv(tableau,millis());
     printcsv(tableau,alt_s->altitude);
     printcsv(tableau,tab_acl[0]);
     printcsv(tableau,tab_acl[1]);
     printcsv(tableau,tab_acl[2]);
     printcsv(tableau,gps_s->framestr);
     printcsv(tableau,P2);
     printcsv(tableau,P4);
     printcsv(tableau,P5);
     printcsv(tableau,P6);
     printcsv(tableau,P7);
     printcsv(tableau,P8);
     printcsv(tableau,P9);
     printcsv(tableau,P10);
     printcsv(tableau,P11);
     printcsv(tableau,P12);
     printcsv(tableau,P13);
     printcsv(tableau,P14);
     tableau.print("\n");
    tableau.close();
  }
}

                                                      /*debugging*/
void loop_debug(struct gps_s* gps_s, struct alt_s* alt_s, struct debug_s* debug_s){
  if (!(waitFor(debug_s->timer,debug_s->period))) return;
  Serial.print("Temps : ");Serial.println(String(millis()));
  Serial.print("Altitude : ");Serial.println(String(alt_s->altitude));
  Serial.print("Accel X : ");Serial.println(String(tab_acl[0]));
  Serial.print("Accel Y : ");Serial.println(String(tab_acl[1]));
  Serial.print("Accel Z : ");Serial.println(String(tab_acl[2]));
  Serial.print("GPS longitude : ");Serial.println(gps_s->lon);
  Serial.print("GPS latitude : ");Serial.println(gps_s->lat);
  Serial.print("GPS Frame : ");Serial.println(gps_s->framestr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Fonctions                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

 
                                                  /* Accélération */
void accel_read_data(){
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  /* Vecteur contenant les composantes des accélérations */
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  tab_acl[0] = accelerometer.x();
  tab_acl[1] = accelerometer.y();
  tab_acl[2] = accelerometer.z();
}
                                                    /* lisibilité d'écriture */
void printcsv(File tab, float val){
  tab.print(val);
  tab.print(";");
}

void printcsv(File tab, String val){
  tab.print(val);
  tab.print(";");
}

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
  //Serial.print("Altitude:");
  //Serial.print(altbaro); //en m
  //Serial.println("");
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

                                            /* Timer */
unsigned int waitFor(int timer, unsigned long period){
  static unsigned long waitForTimer[MAX_WAIT_FOR_TIMER];  // il y a autant de timers que de tâches périodiques
  unsigned long newTime = micros() / period;              // numéro de la période modulo 2^32 
  int delta = newTime - waitForTimer[timer];              // delta entre la période courante et celle enregistrée
  if ( delta < 0 ) delta = 1 + newTime;                   // en cas de dépassement du nombre de périodes possibles sur 2^32 
  if ( delta ) waitForTimer[timer] = newTime;             // enregistrement du nouveau numéro de période
  return delta;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Mains                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  mise_en_place();
  setup_gps();
  setup_pression(&pression_s, 5, 10000);
  setup_accel(&accel_s, 4, 10000);
  setup_alt(&alt_s, 3, 10000);
  setup_radio(&radio_s, 2, 10000);
  setup_uSD(&uSD_s, 1, 10000);
  setup_debug(&debug_s, 0, 10000);
}//fin setup

void loop() {
  loop_gps(&gps_s);   
  loop_pression(&pression_s);
  loop_led(&time_s);
  loop_accel(&accel_s);
  loop_alt(&time_s, &alt_s);
  loop_radio(&gps_s, &alt_s, &radio_s);
  loop_uSD(&gps_s, &alt_s, &uSD_s);
  loop_debug(&gps_s, &alt_s, &debug_s);
  i=i+1;
  //delay(200);
}//fin loop
