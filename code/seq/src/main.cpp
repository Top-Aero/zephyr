#include <Arduino.h>
#include <Servo.h>

#define DEBUG

#define I_JCK (23)  // signal jack
#define O_ARM (19)  // signal armement
#define O_VOL (20)  // signal phase vol
#define O_EJC (21)  // signal ejection
#define O_SRV (9)   // commande servomoteur

enum {E_VEILLE, E_ARME, E_VOL, E_EJECTION, E_FIN};  // etats du sequenceur
uint16_t const c_ouvert(1916);         // step ouverture systeme separation (us)
uint16_t const c_ferme(2200);          // step fermeture systeme separation (us)
uint16_t const c_dureeVol(19500);      // duree phase vol avant ejection (ms)
uint16_t const c_delaiAntiRebond(50);  // delai anti-rebond prise jack (ms)

bool etat_servo(LOW), etat_servo_precedent(LOW);
bool jack_precedent(HIGH), jack_courant(HIGH), jack(false);
uint8_t etat_precedent(E_VEILLE), etat(E_VEILLE);
elapsedMillis tpsJck, tpsVol; 
Servo grosServo;

void setup() {
  #ifdef DEBUG
    delay(5000);
    Serial.begin(9600);
    Serial.println("\netat\tservo\tjack\tduree vol");
  #endif

  pinMode(I_JCK, INPUT_PULLDOWN);
  pinMode(O_ARM, OUTPUT);
  pinMode(O_VOL, OUTPUT);
  pinMode(O_EJC, OUTPUT);
  pinMode(O_SRV, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  grosServo.writeMicroseconds(c_ouvert);
  grosServo.attach(O_SRV);
}

void loop() {
  #ifdef DEBUG
    Serial.print(etat); Serial.print('\t');
    Serial.print(etat_servo); Serial.print('\t');
    Serial.print(jack); Serial.print('\t');
    Serial.print(tpsVol); Serial.print('\r');
    delay(10);
  #endif

  /* bloc jack anti-rebond */
  jack_courant = digitalRead(I_JCK);
  if (jack_courant == jack_precedent){
      tpsJck = 0;
  }
  else if (tpsJck > c_delaiAntiRebond){
    jack = !jack_courant;
    jack_precedent = jack_courant;
  }

////////////////////////////////////////////////////////////////////////////////

  /* bloc machine a etat */
  switch (etat){
    case E_VEILLE:
      grosServo.writeMicroseconds(c_ouvert);
      if (!jack){
        etat = E_ARME;
        digitalWrite(O_ARM, HIGH);
      }
      break;
    ////////////////////////////////////////
    case E_ARME:
      grosServo.writeMicroseconds(c_ferme);
      if (jack){
        tpsVol = 0;
        etat = E_VOL;
        digitalWrite(O_VOL, HIGH);
      }
      break;
    ////////////////////////////////////////
    case E_VOL:
      grosServo.writeMicroseconds(c_ferme);
      if (tpsVol > c_dureeVol){
        etat = E_EJECTION;
        etat_servo = LOW;
      }
      break;
    ////////////////////////////////////////
    case E_EJECTION:
      grosServo.writeMicroseconds(c_ouvert);
      etat = E_FIN;
      digitalWrite(O_EJC, HIGH);
      break;
    ////////////////////////////////////////
    case E_FIN:
      break;
  }
}
