
#include <Servo.h>

#define TRIG_PIN 13   // e.g. the pin used for trigger on the ultrasonic
#define ECHO_PIN 2  // the pin used for echo
#define SERVO_PIN 0  // the servo motor pin (attached to sensor mount)


//1 and 4 left motors, 2 3 right motors
#define ENA 5   // Left side speed (PWM pin)
#define ENB 6   // Right side speed (PWM pin)

#define IN1 7   // Left motor direction
#define IN2 11

#define IN3 12   // Right motor direction
#define IN4 8


Servo ultrasonicServo;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  ultrasonicServo.attach(SERVO_PIN);

  // Optionally center the servo at start
  ultrasonicServo.write(90);  // middle
  delay(500);
}

void moveForward(int speed) {
  #define IN1 7   // Left motor direction
  #define IN2 11

  #define IN3 12   // Right motor direction
  #define IN4 8
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void moveReverse(int speed) {
  #define IN1 7   // Left motor direction
  #define IN2 11

  #define IN3 12  // Right motor direction
  #define IN4 8
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void turnLeft(int speed)
{
  #define IN1 7   // Left motor direction
  #define IN2 11

  #define IN3 12  // Right motor direction
  #define IN4 8
  digitalWrite(IN1, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  
  analogWrite(ENB, speed);
  analogWrite(ENA, speed);
}

void turnRight(int speed)
{
  #define IN1 7   // Left motor direction
  #define IN2 11

  #define IN3 12  // Right motor direction
  #define IN4 8
  digitalWrite(IN1, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);

  analogWrite(ENB, speed);
  analogWrite(ENA, speed);
}


void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

long measureDistanceCM() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pulse width
  double duration = pulseIn(ECHO_PIN, HIGH, 30000);  // timeout ~30ms
  if (duration == 0) {
    // no echo (out of range)
    return -1;
  }
  // Convert to distance (cm)
  double distance = duration / 58.0;
  return distance;
}

void obstacleAvoidance()
{
  moveReverse(200);
  delay(500);
  stopCar();
  delay(500);
  turnLeft(200);
  delay(1500);
  double LeftDist = measureDistanceCM();
  stopCar();
  delay(500);
  turnRight(200);
  delay(3000);
  double RightDist = measureDistanceCM();
  stopCar();
  delay(500);

  if (RightDist > LeftDist)
  {
    turnRight(200);
  }
  else
  {
    turnLeft(200);
  }

  delay(1500);
  stopCar();
  delay(500);
}


void loop()
{
    delay(300);  // wait for servo to move
    
    double dist = measureDistanceCM();
    Serial.print(": ");
    if (dist < 0) {
      Serial.println("Out of range");
    } else {
      Serial.print(dist);
      Serial.println(" cm");
      bool Move = true;
      if (dist > 10)
      {
        moveForward(200);
      }
      else
      {
        obstacleAvoidance();
      }
    }
}

/**void turnLeft(int speed)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed*100);
  analogWrite(ENB, 0);
}

void turnRight(int speed)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed*5);
}

long measureDistanceCM() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pulse width
  double duration = pulseIn(ECHO_PIN, HIGH, 30000);  // timeout ~30ms
  if (duration == 0) {
    // no echo (out of range)
    return -1;
  }
  // Convert to distance (cm)
  double distance = duration / 58.0;
  return distance;
}

void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {
  // Optionally scan a few directions
  int scanAngles[] = {60, 90, 120};  // left, center, right
  for (int i = 0; i < 3; i++) {
    ultrasonicServo.write(scanAngles[i]);
    delay(300);  // wait for servo to move
    
    double dist = measureDistanceCM();
    Serial.print("Angle ");
    Serial.print(scanAngles[i]);
    Serial.print(": ");
    if (dist < 0) {
      Serial.println("Out of range");
    } else {
      Serial.print(dist);
      Serial.println(" cm");
      bool Move = true;
      if (Move)
      {
        turnLeft(500);
      }
      else
      {
        stopCar();
      }
    }
  }

  // Add logic: if any distance < threshold, take avoidance action
  delay(500);
}**/