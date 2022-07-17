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

#pragma once

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Bibliothèques                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <WireKinetis.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>  //librairie Adafruit générale des capteurs 
#include <Adafruit_BNO055.h>  //librairie du module inertiel /* Accélération */
#include <utility/imumaths.h> //librairie pour le fonctionnement du module inertie
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GPS.h> //librairie Adafruit du module GPS /* GPS */
#include <SD.h>       /* uSD */
#include <SPI.h>    

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Définitions                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////

#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location
#define BNO055_SAMPLERATE_DELAY_MS (5)        /* Accélération */
#define mySerial Serial1      /* GPS */


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Déclarations globales                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////

                      /* GPS */
static const int RXPin = 0, TXPin = 1;      
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_GPS GPS(&mySerial);
bool firstGPS;

                      /* LED*/
int ledState=LOW;
unsigned long preMillis=0;

                      /* Accélération */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); //by default address is 0x29 or 0x28
// @mib
float tab_acl[3] = {0, 0, 0};

                      /*Pression*/
float tension1, P1, tension2, P2, tension3, P3, tension4, P4, tension5, P5, tension6, P6, tension7, P7, tension8, P8, tension9, P9, tension10, P10, tension11, P11, tension12, P12, tension13, P13, tension14, P14;//Déclaration des variables tension et pression

                     /* Altitude */
Adafruit_MPL3115A2 baro;

                       /* Radio */
int comdata=0;       
int data_lenght;
String comstr;
char* frame;
SoftwareSerial lora(31,32);

                        /* uSD */
File myFile;        
File tableau;
const int chipSelect = BUILTIN_SDCARD;
int i=0; //Compteur du nombre de mesures

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Déclarations des structures                              //
////////////////////////////////////////////////////////////////////////////////////////////////////

struct gps_s {
    String framestr; 
};

struct time_s {
    elapsedMillis ms;
};

struct alt_s {
    float altitudeSmoothed;
    int sensoraddress;
};

struct gps_s        gps_s;
struct time_s       time_s;
struct alt_s        alt_s;

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Déclaration des fonctions                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

void accel_read_data();
void printcsv(File tab, float val);
void printcsv(File tab, String val); //Surcharge