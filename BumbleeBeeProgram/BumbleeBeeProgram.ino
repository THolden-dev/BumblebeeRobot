
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

#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4

int DefaultSpeed = 150;

Servo ultrasonicServo;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
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
  delay(700);
  double LeftDist = measureDistanceCM();
  stopCar();
  delay(500);
  turnRight(200);
  delay(1400);
  double RightDist = measureDistanceCM();
  stopCar();
  delay(500);
  turnLeft(200);
  delay(700);

  if (RightDist > LeftDist)
  {
    turnRight(200);
  }
  else
  {
    turnLeft(200);
  }

  delay(700);
  stopCar();
  delay(500);
}

int lineTracking(int s1, int s2, int s3, int s4, int s5)
{
  int MoveDelay = 0;
  if((s1 == 1) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 1))
  {
    Serial.println("Line tracking");
    moveForward(DefaultSpeed);
    return 1;
  }
  else if ((s1 == 1) && (s2 == 1) && (s3 == 1) && (s4 == 0) && (s5 == 0))
  {
    Serial.println("Line tracking turn Right minor 3");
    turnRight(DefaultSpeed);
    delay(MoveDelay);
    return 1;
  }
  else if ((s1 == 0) && (s2 == 0) && (s3 == 1) && (s4 == 1) && (s5 == 1))
  {
    Serial.println("Line tracking turn left minor 3");
    turnLeft(DefaultSpeed);
    delay(MoveDelay);
    return 1;
  }
  else if ((s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 1) && (s5 == 1))
  {
    Serial.println("Line tracking turn left minor 3");
    turnLeft(DefaultSpeed);
    delay(MoveDelay);
    return 1;
  }
  else if ((s1 == 1) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 0))
  {
    Serial.println("Line tracking turn Right minor");
    turnRight(DefaultSpeed);
    delay(MoveDelay);
    return 1;
  }
  else if ((s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 1))
  {
    Serial.println("Line tracking turn left minor 1");
    turnLeft(DefaultSpeed);
    delay(MoveDelay);
    return 1;
  }
  else if ((s1 == 0) && (s2 == 1) && (s3 == 1) && (s4 == 1) && (s5 == 1))
  {
    Serial.println("Line tracking turn left minor 2");
    turnLeft(DefaultSpeed);
    delay(MoveDelay);
    return 1;
  }
  else if ((s1 == 1) && (s2 == 1) && (s3 == 1) && (s4 == 1) && (s5 == 0))
  {
    Serial.println("Line tracking turn right minor 2");
    turnRight(DefaultSpeed);
    delay(MoveDelay);
    return 1;
  }  
  else if ((s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 0))
  {
    Serial.println("Line tracking turning right major");
    turnRight(DefaultSpeed);
    delay(200);
    return 1;
  }
  return 0;
}


void loop()
{
    //Reading Sensor Values
    int s1 = digitalRead(ir1);  //Left Most Sensor
    int s2 = digitalRead(ir2);  //Left Sensor
    int s3 = digitalRead(ir3);  //Middle Sensor
    int s4 = digitalRead(ir4);  //Right Sensor
    int s5 = digitalRead(ir5);  //Right Most Sensor
    
    double dist = measureDistanceCM();
    Serial.print(": ");
    int LineTracking = lineTracking(s1, s2, s3, s4, s5);
    if (dist < 0) {
      Serial.println("Out of range");
    }
    else if (dist > 5 && LineTracking == 0)
    {
      Serial.println("MovingForward");
      moveForward(DefaultSpeed);
    }
    else if (LineTracking == 0 && dist < 5) {
      Serial.println("Avoiding");
      //obstacleAvoidance();
    }
}
