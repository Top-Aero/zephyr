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

int fin, ini;
Servo srv;

void setup() {
  pinMode(p_led, OUTPUT);
  pinMode(p_potard, INPUT);

  srv.attach(p_srv);

  digitalWrite(p_led, HIGH);
}

void loop() {
  ini = srv.readMicroseconds();
  fin = map(analogRead(p_potard), 0, 1023, 1500, 2200);

  Serial.print(ini); Serial.print(" -> "); Serial.println(fin);

  if (abs(fin - ini) > 4)
    srv.writeMicroseconds(fin);

  delay(100);
}
