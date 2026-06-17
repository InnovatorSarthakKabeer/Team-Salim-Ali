#include <ESP32Servo.h>
 
int startAngle = 0;
int targetAngle = 90;

Servo S1, S2, S3, S4, S5, S6;
Servo S7, S8, S9, S10, S11, S12;                                                                                                                vv
Servo S13, S14, S15, S16, S17, S18;

void setup() {
  Serial.begin(115200);

  S1.attach(19);
  S2.attach(23);
  S3.attach(22);
  S4.attach(18);
  S5.attach(21);
  S6.attach(25);

  // S7.attach(4);
  // S8.attach(2);
  // S9.attach(33);
  // S10.attach(32);
  // S11.attach(27);
  // S12.attach(26);

  // S13.attach(5);
  // S14.attach(17);
  // S15.attach(15);
  // S16.attach(13);
  // S17.attach(12);
  // S18.attach(34);

  // Set all servos to startAngle
  S1.write(startAngle);
  S2.write(startAngle);
  S3.write(startAngle);
  S4.write(startAngle);
  S5.write(startAngle);
  S6.write(startAngle);

  // S7.write(startAngle);
  // S8.write(startAngle);
  // S9.write(startAngle);
  // S10.write(startAngle);
  // S11.write(startAngle);
  // S12.write(startAngle);
  // S13.write(startAngle);
  // S14.write(startAngle);
  // S15.write(startAngle);
  // S16.write(startAngle);
  // S17.write(startAngle);
  // S18.write(startAngle);


  Serial.print("All servos at ");
  Serial.println(startAngle);

  delay(1500);

  //Move all servos to targetAngle
  S1.write(targetAngle);
  S2.write(targetAngle);
  S3.write(targetAngle);
  S4.write(targetAngle);
  S5.write(targetAngle);
  S6.write(targetAngle);
  
  // S7.write(targetAngle);
  // S8.write(targetAngle);
  // S9.write(targetAngle);
  // S10.write(targetAngle);
  // S11.write(targetAngle);
  // S12.write(targetAngle);

  // S13.write(targetAngle);
  // S14.write(targetAngle);
  // S15.write(targetAngle);
  // S16.write(targetAngle);
  // S17.write(targetAngle);
  // S18.write(targetAngle);


  Serial.print("All servos moved to ");
  Serial.println(targetAngle);

 
}

void loop() {
  

  Serial.print("All servos moved to ");
  Serial.println(targetAngle);
}
