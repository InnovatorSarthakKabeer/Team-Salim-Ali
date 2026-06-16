int xPin = 32;
int yPin = 33;

int xMid;
int yMid;
void setup() {
  Serial.begin(115200);

  delay(2000);

  xMid = analogRead(xPin);
  yMid = analogRead(yPin);

  Serial.println("Done");
}

void loop() {
  int x = abs(analogRead(xPin) - xMid);
  int y = abs(analogRead(yPin) - yMid);

  Serial.print("X = ");
  Serial.print(x);

  Serial.print("   Y = ");
  Serial.println(y);

  delay(150);
}
