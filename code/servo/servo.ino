/*******************************************************************************
 * @brief TOP-AERO -- zephir.seq : Controle du servmoteur.
 * 
 * @author Mohamed-Iadh BANI <mohamed-iadh.bani@top-aero.com>
*******************************************************************************/

#include <Servo.h>

// pins
uint8_t const p_srv(6);
uint8_t const p_led(13);
uint8_t const p_potard(15);

// variables de travail
Servo srv;
int th(0);

void mouvement_angulaire(int cible){
  int ini = srv.read();
  int inc = (ini < cible) - (ini > cible);

  Serial.print(ini); Serial.print(" -> "); Serial.print(cible);

  for (int angle(ini); angle != cible; angle += inc){
    srv.write(angle);
    delay(10);
  }
}

void setup() {
  pinMode(p_led, OUTPUT);
  pinMode(p_potard, INPUT);
  srv.attach(p_srv);
}

void loop() {
  th = map(analogRead(p_potard), 0, 1023, 0, 180);
  Serial.println(th);
  mouvement_angulaire(th);
  delay(10);
}
