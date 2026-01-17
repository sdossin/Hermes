/*
 */
#include <Servo.h>

Servo servo1;
Servo servo2;
int servo1Pin = 10;
int servo2Pin = 11;
int maxAngle = -180;
int minAngle = 0;

float s = 1;
float r1 = 3.81; // cm
float r2 = 4.81;
float rmax = r1+r2;

void setup() {
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  Serial.begin(9600);

  servo1.write(90);
  servo2.write(90);
}

void setAngle(int theta1, int theta2) {
  if(theta1 > 0 || theta1 < -180) {
    Serial.println("Theta 1 Angle Invalid");
    return;
  }

  if(theta2 > 0 || theta1 < -180) {
    Serial.println("Theta 2 Angle Invalid");
    return;
  }

  servo1.write(theta1);
  servo2.write(theta2);
}

int dist(int x, int y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

void setPosition(int x, int y) {
    if((x > 0 && x > -s + sqrt(pow(rmax, 2) - pow(y, 2))) || (x < 0 && x < s - sqrt(pow(rmax, 2) - pow(y, 2)))) {
        Serial.println("Input point is out of bounds");
        return;
    }

    float l1x = x + s;
    float l1y = 0;
    float l2x = x - s;
    float l2y = 0;
    float l1 = dist(l1x, l1y);
    float l2 = dist(l2x, l2y);
    float theta1 = atan2(l1y,l1x) - acos((pow(l1, 2) + r1*r1 - r2*r2)/(2*r1*l1));
    float theta2 = atan2(l2y,l2x) + acos((pow(l2, 2) + r1*r1 - r2*r2)/(2*r1*l2));

    setAngle(theta1*M_PI/180, theta2*M_PI/180);
}

void loop() {
    Serial.println("Enter x: ");
    while (Serial.available() == 0) {}
    int x = Serial.parseInt();
    Serial.println("Enter y: ");
    while (Serial.available() == 0) {}
    int y = Serial.parseInt();
    
    setPosition(x, y);
    delay(1000);
} 
