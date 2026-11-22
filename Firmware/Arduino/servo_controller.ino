#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(9600);
  servo1.attach(9);
  servo2.attach(10);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int comma = data.indexOf(',');
    if (comma != -1) {
      int angle1 = data.substring(0, comma).toInt();
      int angle2 = data.substring(comma + 1).toInt();

      angle1 = constrain(angle1, 0, 180);
      angle2 = constrain(angle2, 0, 180);

      servo1.write(angle1);
      servo2.write(angle2);
    }
  }
}
