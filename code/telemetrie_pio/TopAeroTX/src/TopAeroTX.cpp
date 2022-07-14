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
  pinMode(7, OUTPUT);
  pinMode(8, INPUT_PULLUP);
  Wire.setSDA(8); // Activation du mode alternatif du pin
  Wire.setSCL(7); // Activation du mode alternatif du pin                       
  Wire.begin(); //join i2c bus    
  Serial.begin(115200);//ouverture port série 1 USB
  pinMode(13,OUTPUT); //LED
  Serial2.begin(115200);//ouverture port série 2 pour radio
  GPS.begin(9600); //baudrate obligé du module
  ss.begin(GPSBaud);
    /* Initialise the sensor */
  lora.begin(115200); //baudrate obligé du module
  delay(20);
}
 
                                                /* GPS */
void setup_gps() {
  // this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); //taux de rafraichissement 10Hz, en réalité il n'est pas représentatif 
  //du rafraichissement réel, mais il vaut mieux entrer la commande tout de même
}

                                            /* Accélération */
void setup_accel(){
  if(!bno.begin())
  {
    Serial.print("Couldnt find accelerator sensor");
    while(1);
  }
  bno.setExtCrystalUse(true);
}
 
                                              /* Altitude */    
void setup_alt(struct alt_s* alt_s){
  
  if (!baro.begin()) {
    Serial.println("Couldnt find altimeter sensor");
    while(1);
  }
}

                                                /* Radio */ 
void setup_radio(){
  lora.println("AT+BAND=869650000\r\n"); //commande pour appliquer la fréquence d'émission ou réception du module
  //ici 869,65Mhz
  delay(20);
  lora.println("AT+PARAMETER=7,9,4,4\r\n"); //par défaut = 12,7,1,4 
  //Spreading factor = 7, Bandwidth = 9 (500kHz), Coding rate = 4, Programmed Preamble = 4
  delay(2000);
  Serial.println("Communication prête");
}

                                                /* uSD */
void setup_uSD(){
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

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Tâches                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////

                                                  /*Pression*/
void loop_pression(){
  tension2 = analogRead(A16) * 5.0 / 1023.0;
  P2 = (tension2 / 0.002 - 39.75);
  //Serial.println("P2 :");
  //Serial.print(P2, 0);
  //Serial.println(" hPa");
  
  tension4 = analogRead(A18) * 5.0 / 1023.0;
  P4 = (tension4 / 0.002 - 34.75);
  //Serial.println("P4 :");
  //Serial.print(P4, 0);
  //Serial.println(" hPa");
  
  tension5 = analogRead(A14) * 5.0 / 1023.0;
  P5 = (tension5 / 0.002 - 48.75);
  //Serial.println("P5 :");
  //Serial.print(P5, 0);
  //Serial.println(" hPa");
  
  tension6 = analogRead(A15) * 5.0 / 1023.0;
  P6 = (tension6 / 0.002 - 44.75);
  //Serial.println("P6 :");
  //Serial.print(P6, 0);
  //Serial.println(" hPa");
  
  tension7 = analogRead(A17) * 5.0 / 1023.0;
  P7 = (tension7 / 0.002 - 39.75);
  //Serial.println("P7 :");
  //Serial.print(P7, 0);
  //Serial.println(" hPa");
  
  tension8 = analogRead(A9) * 5.0 / 1023.0;
  P8 = (tension8 / 0.002 - 53.75);
  //Serial.println("P8 :");
  //Serial.print(P8, 0);
  //Serial.println(" hPa");
  
  tension9 = analogRead(A8) * 5.0 / 1023.0;
  P9 = (tension9 / 0.002 - 24.75);
  //Serial.println("P9 :");
  //Serial.print(P9, 0);
  //Serial.println(" hPa");
  
  tension10 = analogRead(A7) * 5.0 / 1023.0;
  P10 = (tension10 / 0.002 - 29.75);
  //Serial.println("P10 :");
  //Serial.print(P10, 0);
  //Serial.println(" hPa");
  
  tension11 = analogRead(A6) * 5.0 / 1023.0;
  P11 = (tension11 / 0.002 - 31.75);
  //Serial.println("P11 :");
  //Serial.print(P11, 0);
  //Serial.println(" hPa");
  
  tension12 = analogRead(A3) * 5.0 / 1023.0;
  P12 = (tension12 / 0.002 - 36.75);
  //Serial.println("P12 :");
  //Serial.print(P12, 0);
  //Serial.println(" hPa");

  tension13 = analogRead(A2) * 5.0 / 1023.0;
  P13 = (tension13 / 0.002 - 53.75);
  //Serial.println("P13 :");
  //Serial.print(P13, 0);
  //Serial.println(" hPa");

  tension14 = analogRead(A1) * 5.0 / 1023.0;//Lecture de la tension et conversion en V
  P14 = (tension14 / 0.002 - 49.75); //Obtention de la pression en hPa => valeur 93 = offset , a changer pour chaque capteur 
  //Affichage de la pression sur le3moniteur série
  //Serial.println("P14 :");
  //Serial.print(P14, 0);
  //Serial.println(" hPa");
}

                                                    /* GPS */
void loop_gps(struct gps_s* gps_s){
  firstGPS=true;
  while (ss.available() > 0){
  GPS.read();
  char* frame = GPS.lastNMEA();
  GPS.parse(frame);
    if(firstGPS){
      Serial.println(frame);
      firstGPS=not(firstGPS);
      gps_s->framestr=String(frame);  
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
void loop_accel(){
  accel_read_data();
}

                                                  /* Altitude (m) */ 
void loop_alt(struct time_s* time_s, struct alt_s* alt_s){
  alt_s->altitudeSmoothed=baro.getAltitude();
}

                                                      /* Radio */
void loop_radio(struct gps_s* gps_s, struct alt_s* alt_s){
  String mes=" alt="+String(alt_s->altitudeSmoothed)+" acx="+String(tab_acl[0])+" acy="+String(tab_acl[1])+" acz="+String(tab_acl[2])+" tps="+String(millis())+" gps="+gps_s->framestr;
  String datas=mes+" P11="+String(P11)+" P12="+String(P12);
  datas=datas.trim();
  String cmd="AT+SEND=0,"+String(mes.length()) +","+ datas +"\r\n";
  lora.println(cmd);
  while(lora.available()){
    Serial.write(lora.read());
  }
  Serial.print(cmd);
  delay(35); //le délai influe sur la transmission radio, si trop court il y a une probabilité +/- importante 
  //que les messages soient coupés
}

                                                      /* uSD */
void loop_uSD(struct gps_s* gps_s, struct alt_s* alt_s){
    myFile = SD.open("zephyr.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("Mesure ");
    myFile.println(i);
    myFile.print("\nTemps = ");
    myFile.println(String(millis()));
    myFile.print("Altitude = ");
    myFile.println(String(alt_s->altitudeSmoothed));
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
     printcsv(tableau,alt_s->altitudeSmoothed);
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

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Mains                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  mise_en_place();
  setup_gps();
  setup_accel();
  setup_alt(&alt_s);
  setup_radio();
  setup_uSD();
}//fin setup

void loop() {
  loop_pression();
  loop_gps(&gps_s);   
  loop_led(&time_s);
  loop_accel();
  loop_alt(&time_s, &alt_s);
  loop_radio(&gps_s, &alt_s);
  loop_uSD(&gps_s, &alt_s);
  i=i+1;
  delay(200);
}//fin loop
