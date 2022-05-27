/*******************************************************************************
 * @file sequenceur.ino
 * 
 * @brief TOP AERO - zephyr.seq : sequenceur liberant le parachute.
 * 
 * @author Mohamed-Iadh BANI <mohamed-iadh.bani@top-aero.com>
*******************************************************************************/

// Pins teensy 3.2 : 9, 14.
uint8_t const PIN_SERVO(9);
uint8_t const PIN_JACK(14);

// Pins LEDs teensy 3.2 : 2, 3, 4.
uint8_t const LED_ARMEMENT(2);
uint8_t const LED_JACK(3);
uint8_t const LED_PARACHUTE(4);

// constantes machine a etats
uint8_t const ETAT_INITIAL(0);
uint8_t const ETAT_RAMPE(1);
uint8_t const ETAT_VOL(2);
uint8_t const ETAT_PARACHUTE(3);
uint8_t const ETAT_OUVERT(4);
uint8_t const ETAT_FIN(-1);

// constantes position servomoteur
uint32_t const PWM_OUVERTURE(4365); //65535 / 20 * 1.333);  // 85 deg
uint32_t const PWM_FERMETURE(3700); //65535 / 20);          // 50 deg
// note mib : 3640 etait la bonne valeur de fermeture du servomoteur

// constantes de temps
uint16_t const DELAI_PARACHUTAGE(19500);  // ms : delai parachutage en phase vol
uint32_t const DELAI_ANTI_REBOND(100);    // ms : delai anti-rebond
uint32_t const DELAI_POS_SERVO(5e3);      // ms : delai positionnement servo


/* variables de travai */

// variables bloc jack anti-rebond
bool jack_precedent(HIGH), jack_courant(HIGH), jack(false);
uint32_t instant_anti_rebond(0);

// bloc servomoteur
uint8_t etat_precedent(ETAT_INITIAL), etat(ETAT_INITIAL);

// etat parachute
uint32_t instant_decollage, tps_courant, delai_parachute(0);

// etats servomoteurs
bool etat_servo(HIGH), etat_servo_precedent(HIGH);
uint32_t instant_servo;


void setup() {
  Serial.begin(9600);

  pinMode(PIN_JACK, INPUT_PULLUP);
  pinMode(PIN_SERVO, OUTPUT);

  // leds temoins
  pinMode(LED_ARMEMENT, OUTPUT);
  pinMode(LED_JACK, OUTPUT);
  pinMode(LED_PARACHUTE, OUTPUT);

  // calibrage pwm puis positionnement servomoteur
  analogWriteResolution(16);
  analogWriteFrequency(PIN_SERVO, 50);
  analogWrite(PIN_SERVO, PWM_FERMETURE);
  delay(3500);
  analogWrite(PIN_SERVO, 0);
}

void loop() {
  Serial.print("etat\t\t"); Serial.println(etat);
  Serial.print("etat_servo\t"); Serial.println(etat_servo);
  Serial.print("delai_parachute\t"); Serial.println(tps_courant - instant_decollage);
  Serial.print("jack mise\t\t"); Serial.println(jack);
  delay(10);

  /* bloc jack anti-rebond */
  jack_courant = digitalRead(PIN_JACK);
  if (jack_courant == jack_precedent)
      instant_anti_rebond = millis();

  if ((millis() - instant_anti_rebond > DELAI_ANTI_REBOND) && (jack_courant != jack_precedent)){
    jack = !jack_courant;
    jack_precedent = jack_courant;
  }    

  //jack = !digitalRead(PIN_JACK);

////////////////////////////////////////////////////////////////////////////////

  /* bloc machine a etat */
  switch (etat){
    case ETAT_INITIAL:  
      if (jack){
        etat = ETAT_RAMPE;
        digitalWrite(LED_ARMEMENT, HIGH);
      }
      break;
    ////////////////////////////////////////
    case ETAT_RAMPE:
      etat_servo = HIGH;
      if (!jack){
        instant_decollage = millis();
        etat = ETAT_VOL;
        digitalWrite(LED_JACK, HIGH);
      }
      break;
    ////////////////////////////////////////
    case ETAT_VOL:
      tps_courant = millis();
      if (tps_courant - instant_decollage > DELAI_PARACHUTAGE)
        etat = ETAT_PARACHUTE;
        etat_servo = LOW;
      break;
    ////////////////////////////////////////
    case ETAT_PARACHUTE:
      etat_servo = LOW;
      etat = ETAT_FIN;
      digitalWrite(LED_PARACHUTE, HIGH);
      break;
    ////////////////////////////////////////
    case ETAT_FIN:
      break;
  }

////////////////////////////////////////////////////////////////////////////////

  /* bloc servomoteur econome */
  if (etat != etat_precedent){
    instant_servo = millis();
    analogWrite(PIN_SERVO, (etat_servo ? PWM_FERMETURE : PWM_OUVERTURE));
    etat_precedent = etat;
  }

  if ((etat == ETAT_INITIAL || etat == ETAT_FIN) && millis() - instant_servo > DELAI_POS_SERVO)
    analogWrite(PIN_SERVO, 0);  // couper le servo en phase autre que vol
}
