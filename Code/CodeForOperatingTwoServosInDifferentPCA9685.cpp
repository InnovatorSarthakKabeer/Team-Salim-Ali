#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca1(0x40);
Adafruit_PWMServoDriver pca2(0x60);

#define SERVOMIN 150
#define SERVOMAX 600

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void moveServo(Adafruit_PWMServoDriver &board, uint8_t channel, int angle) {
  board.setPWM(channel, 0, angleToPulse(angle));
}

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);

  pca1.begin();
  pca1.setPWMFreq(50);

  pca2.begin();
  pca2.setPWMFreq(50);

  delay(500);
}

void loop() {

  // Move both to 0°
  moveServo(pca1, 0, 0);
  moveServo(pca2, 0, 0);
  delay(1500);

  // Move both to 90°
  moveServo(pca1, 0, 90);
  moveServo(pca2, 0, 90);
  delay(1500);

  // Move both to 180°
  moveServo(pca1, 0, 180);
  moveServo(pca2, 0, 180);
  delay(1500);

}
