/*
 * Arduino program
 * test for main(EUSART_speed_control).c
 */

int i;
void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.write(0b00000000);
  delay(2000);

  Serial.write(0b00110000);
  delay(2000);

  Serial.write(0b01110000);
  delay(2000);

  Serial.write(0b11110000);
  delay(2000);

  for (i = 0; i < 20; i++) {
    Serial.write(0b11110000);
    delay(50);
    Serial.write(0b11100000);
    delay(50);
  }
}
