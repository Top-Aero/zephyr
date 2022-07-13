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
#define ALTMODE //comment out for barometer mode; default is altitude mode 
#define mySerial Serial1      /* GPS */
#define MAX_WAIT_FOR_TIMER 1    /* timer des fonctions */

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
Adafruit_BNO055 bno = Adafruit_BNO055(55); //by default address is 0x29 or 0x28
// @mib
float tab_acl[3] = {0, 0, 0};

                      /*Pression*/
float tension1, P1, tension2, P2, tension3, P3, tension4, P4, tension5, P5, tension6, P6, tension7, P7, tension8, P8, tension9, P9, tension10, P10, tension11, P11, tension12, P12, tension13, P13, tension14, P14;//Déclaration des variables tension et pression

              /* Altitude */

Adafruit_MPL3115A2 baro;
/*ool first=true;
float altm0=0.0;
//const int SENSORADDRESS = 0x60; // address specific to the MPL3115A1, value found in datasheet 
float altsmooth = 0; //for exponential smoothing          
byte IICdata[5] = {0,0,0,0,0}; //buffer for sensor data 
float firstalt=0.0;
bool alt_must_adjust = true; //condition pour ajuster l'altitude au bout de 1min*/

int comdata=0;       /* Radio */
int data_lenght;
String comstr;
char* frame;
SoftwareSerial lora(9,10);

File myFile;        /* uSD */
File tableau;
const int chipSelect = BUILTIN_SDCARD;
int i=0; //Compteur du nombre de mesures

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Déclarations des structures                              //
////////////////////////////////////////////////////////////////////////////////////////////////////

enum {EMPTY, FULL};

struct mailbox_s {
    int state; 
    int val;
};

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

//struct mailbox_s    mb = {.state = EMPTY};  NO NEED MAILBOX
struct gps_s        gps_s;
struct time_s       time_s;
struct alt_s        alt_s;

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        Déclaration des fonctions                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

void accel_read_data();
float altitude_read_data(struct alt_s*);
float Alt_Read();
byte IIC_Read(byte regAddr, struct alt_s*);
void IIC_ReadData(struct alt_s*);
void IIC_Write(byte regAddr, byte value, struct alt_s*);
void printcsv(File tab, float val);
void printcsv(File tab, String val); //Surcharge