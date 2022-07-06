#include <Arduino.h>
#include <Servo.h>

#define I_JCK (23)  // signal jack
#define O_ARM (19)  // signal armement
#define O_VOL (20)  // signal phase vol
#define O_EJC (21)  // signal ejection
#define O_SRV (9)   // commande servomoteur

#define DEBUG

uint16_t const c_ouvert(1916);     // step ouverture systeme separation (us)
uint16_t const c_ferme(2200);      // step fermeture systeme separation (us)
uint16_t const c_dureeVol(19500);  // duree phase vol avant ejection (ms)
uint16_t const c_delaiAntiRebond(50);  // delai anti-rebond prise jack (ms)
uint32_t const c_delaiServo(5000);     // delai de positionnement servo (ms)
Servo grosServo;

void setup() {
  Serial.begin(9600);

  #ifdef DEBUG
    delay(5000);
    Serial.println("\netat\tservo\tjack\tduree vol");
  #endif

  pinMode(I_JCK, INPUT_PULLDOWN);
  pinMode(O_ARM, OUTPUT);
  pinMode(O_VOL, OUTPUT);
  pinMode(O_EJC, OUTPUT);
  pinMode(O_SRV, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  grosServo.attach(O_SRV);
  grosServo.writeMicroseconds(c_ouvert);
}

enum {E_INITIAL, E_RAMPE, E_VOL, E_PARACHUTE, E_FIN};
bool jack_precedent(HIGH), jack_courant(HIGH), jack(false);
bool etat_servo(LOW), etat_servo_precedent(LOW);
uint8_t etat_precedent(E_INITIAL), etat(E_INITIAL);
uint32_t instant_anti_rebond(0);
uint32_t instant_decollage, tps_courant, delai_parachute(0);

void loop() {
  #ifdef DEBUG
    Serial.print(etat); Serial.print('\t');
    Serial.print(etat_servo); Serial.print('\t');
    Serial.print(jack); Serial.print('\t');
    Serial.print(tps_courant - instant_decollage); Serial.print('\r');
    delay(10);
  #endif

  /* bloc jack anti-rebond */
  jack_courant = digitalRead(I_JCK);
  if (jack_courant == jack_precedent)
      instant_anti_rebond = millis();

  if ((millis() - instant_anti_rebond > c_delaiAntiRebond) && (jack_courant != jack_precedent)){
    jack = !jack_courant;
    jack_precedent = jack_courant;
  }

////////////////////////////////////////////////////////////////////////////////

  /* bloc machine a etat */
  switch (etat){
    case E_INITIAL:
      etat_servo = LOW;
      if (!jack){
        etat = E_RAMPE;
        digitalWrite(O_ARM, HIGH);
      }
      break;
    ////////////////////////////////////////
    case E_RAMPE:
      etat_servo = HIGH;
      if (jack){
        instant_decollage = millis();
        etat = E_VOL;
        digitalWrite(O_VOL, HIGH);
      }
      break;
    ////////////////////////////////////////
    case E_VOL:
      tps_courant = millis();
      if (tps_courant - instant_decollage > c_dureeVol){
        etat = E_PARACHUTE;
        etat_servo = LOW;
      }
      break;
    ////////////////////////////////////////
    case E_PARACHUTE:
      etat_servo = LOW;
      etat = E_FIN;
      digitalWrite(O_EJC, HIGH);
      break;
    ////////////////////////////////////////
    case E_FIN:
      break;
  }

////////////////////////////////////////////////////////////////////////////////

  /* bloc servomoteur econome */
  // if (etat != etat_precedent){
    grosServo.writeMicroseconds(etat_servo ? c_ferme : c_ouvert);
    // etat_precedent = etat;
  // }
}
