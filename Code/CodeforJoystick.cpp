#include <ESP32Servo.h>

int xPin = 32;
int yPin = 33;

Servo myServo;

void setup() {
  Serial.begin(115200);

  myServo.attach(18);  // servo signal pin
}

void loop() {
  int x = analogRead(xPin);
  int y = analogRead(yPin);

  int angle = map(x, 0, 4095, 0, 180);

  myServo.write(angle);

  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" | Angle: ");
  Serial.println(angle);

  delay(20);
}
