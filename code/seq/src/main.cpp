#include <Arduino.h>
#include <Servo.h>

#define DEBUG

#define I_JCK (23)  // signal jack
#define O_ARM (19)  // signal armement
#define O_VOL (21)  // signal phase vol
#define O_EJC (20)  // signal ejection
#define O_SRV (9)   // commande servomoteur

enum {E_VEILLE, E_ARME, E_VOL, E_EJECTION};  // etats du sequenceur
uint16_t const c_ouvert(1916);         // step ouverture systeme separation (us)
uint16_t const c_ferme(2200);          // step fermeture systeme separation (us)
uint16_t const c_delaiAntiRebond(100);  // delai anti-rebond prise jack (ms)
uint16_t const c_delaiClignotant(1000); // led clignotante au depart

#ifdef DEBUG
  uint16_t const c_dureeVol(5000);      // duree phase vol avant ejection (ms)
#else
  uint16_t const c_dureeVol(19500);      // duree phase vol avant ejection (ms)
#endif

bool etat_servo(LOW), etat_servo_precedent(LOW);
bool jack_precedent(HIGH), jack_courant(HIGH), jack(false);
bool clignotant(LOW);
uint8_t etat_precedent(E_VEILLE), etat(E_VEILLE);
elapsedMillis tpsArm, tpsJck, tpsVol;
Servo grosServo;

void setup() {
  pinMode(I_JCK, INPUT_PULLUP);
  pinMode(O_ARM, OUTPUT);
  pinMode(O_VOL, OUTPUT);
  pinMode(O_EJC, OUTPUT);
  pinMode(O_SRV, OUTPUT);

  #ifdef DEBUG
    Serial.begin(9600);
    delay(5000);
    Serial.println("LISTE DES ETATS :\n0\tetat initial\n1\tetat arme\n2\tetat vol\n3\tetat ejecte");
    Serial.println("\netat\tservo\tjack\tduree vol");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(O_ARM, HIGH); delay(500);
    digitalWrite(O_VOL, HIGH); delay(500);
    digitalWrite(O_EJC, HIGH); delay(1000);
    
    digitalWrite(O_ARM, LOW);
    digitalWrite(O_VOL, LOW);
    digitalWrite(O_EJC, LOW);
  #else
    delay(5000);
  #endif

  grosServo.write(c_ferme);
  grosServo.attach(O_SRV);
}

void loop() {
  #ifdef DEBUG
    Serial.print(etat); Serial.print('\t');
    Serial.print(etat_servo); Serial.print('\t');
    Serial.print(jack); Serial.print('\t');
    Serial.print(tpsVol); Serial.print('\r');

    digitalWrite(LED_BUILTIN, jack);

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
      if (tpsArm >= c_delaiClignotant){
        clignotant = !clignotant;
        tpsArm -= c_delaiClignotant;
        digitalWrite(O_ARM, clignotant);
      }
      grosServo.write(0);
      if (jack){
        etat = E_ARME;
        digitalWrite(O_ARM, HIGH);
      }
      break;
    ////////////////////////////////////////
    case E_ARME:
      grosServo.writeMicroseconds(c_ferme);
      // grosServo.detach();
      digitalWrite(O_ARM, HIGH);
      digitalWrite(O_VOL, LOW);
      digitalWrite(O_EJC, LOW);
      if (!jack){
        tpsVol = 0;
        etat = E_VOL;
        digitalWrite(O_VOL, HIGH);
      }
      break;
    ////////////////////////////////////////
    case E_VOL:
      grosServo.writeMicroseconds(c_ferme);
      // grosServo.attach(O_SRV);
      digitalWrite(O_VOL, HIGH);
      if (tpsVol > c_dureeVol){
        etat = E_EJECTION;
        etat_servo = LOW;
      }
      break;
    ////////////////////////////////////////
    case E_EJECTION:
      grosServo.writeMicroseconds(c_ouvert);
      digitalWrite(O_EJC, HIGH);
      etat = E_VEILLE;
      break;
  }
}
