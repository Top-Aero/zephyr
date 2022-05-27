uint16_t t(0);

void setup() {
  Serial.begin(9600);
  pinMode(14, INPUT_PULLUP);
}

void loop() {
  t += 1;

  if (digitalRead(14) == HIGH)
    Serial.print("tps="); Serial.print(t);
    Serial.println(" alt=1 acl=2 gps=$GPRMC,,,3,N,5,W,,,,, P1=6 P2=7");
  
  delay(100);
}
