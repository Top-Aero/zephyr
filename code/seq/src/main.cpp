#include <Arduino.h>

#define I_JCK (14)  // signal jack
#define O_ARM (2)  // signal armement
#define O_VOL (3)  // signal phase vol
#define O_EJC (4)  // signal ejection
#define O_SRV (9)  // commande servomoteur

uint8_t const c_ouvert(143);  // angle d'ouverture du systeme de separation
uint8_t const c_ferme(115);   // angle de fermeture du systeme de separation
uint16_t const c_dureeVol(19500);       // duree de la phase vol avant ejection
uint16_t const c_delaiAntiRebond(100);  // delai anti-rebond de la prise jack
uint16_t const c_delaiSRV(5000);        // delai de positionnement du servo

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
}