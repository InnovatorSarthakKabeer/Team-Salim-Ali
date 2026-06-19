#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca1(0x40);
Adafruit_PWMServoDriver pca2(0x60);

#define SERVOMIN 110
#define SERVOMAX 510

void setServo(Adafruit_PWMServoDriver &pca, uint8_t ch, int angle)
{
  angle = constrain(angle, 0, 180);

  int pulse = map(angle, 0, 180,
                  SERVOMIN, SERVOMAX);

  pca.setPWM(ch, 0, pulse);
}

void moveServoNumber(int servoNum, int angle)
{
  if (servoNum >= 0 && servoNum <= 8)
  {
    setServo(pca1, servoNum, angle);
  }
  else if (servoNum >= 9 && servoNum <= 17)
  {
    setServo(pca2, servoNum - 9, angle);
  }
}

void centerAll()
{
  for (int i = 0; i <= 8; i++)
    setServo(pca1, i, 90);

  for (int i = 0; i <= 8; i++)
    setServo(pca2, i, 90);
}

void setup()
{
  Serial.begin(115200);

  Wire.begin(21, 22);

  pca1.begin();
  pca2.begin();

  pca1.setPWMFreq(50);
  pca2.setPWMFreq(50);

  delay(1000);

  centerAll();

  Serial.println("=== HEXAPOD CALIBRATION MODE ===");
  Serial.println("Command Format:");
  Serial.println("servo angle");
  Serial.println("Example:");
  Serial.println("0 90");
  Serial.println("1 120");
  Serial.println("17 45");
}

void loop()
{
  if (Serial.available())
  {
    int servoNum = Serial.parseInt();
    int angle = Serial.parseInt();

    if (servoNum >= 0 &&
        servoNum <= 17 &&
        angle >= 0 &&
        angle <= 180)
    {
      moveServoNumber(servoNum, angle);

      Serial.print("Servo ");
      Serial.print(servoNum);
      Serial.print(" -> ");
      Serial.println(angle);
    }

    while (Serial.available())
      Serial.read();
  }
}
