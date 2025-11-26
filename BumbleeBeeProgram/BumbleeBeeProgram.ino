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

void turn(int speedL, int speedR)
{
    #define IN1 7   // Left motor direction
    #define IN2 11

    #define IN3 12   // Right motor direction
    #define IN4 8
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speedL);
    analogWrite(ENB, speedR);
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

double measureDistanceCM() {
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
  double distance = (double) duration / (double)58.0;
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
    // Convert: white=1 -> 0, black=0 -> 1
    int s[5] = {
        !s1,
        !s2,
        !s3,
        !s4,
        !s5
    };

    // Weights: left negative, right positive
    int w[5] = {-2, -1, 0, 1, 2};

    // Compute weighted position error
    int error = 0;
    for (int i = 0; i < 5; i++)
        error += s[i] * w[i];

    // Base speed
    int base = 100;

    // Steering gain
    int K = 40;

    // Determine motor speeds
    int left  = base + error * K;
    int right = base - error * K;

    // Limit speeds
    left  = constrain(left, 0, 255);
    right = constrain(right, 0, 255);

    turn(right, left);

    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("Left: ");
    Serial.println(left);
    Serial.print("Right: ");
    Serial.println(right);

    return error;
}


void loop()
{
    int s1 = digitalRead(ir1);
    int s2 = digitalRead(ir2);
    int s3 = digitalRead(ir3);
    int s4 = digitalRead(ir4);
    int s5 = digitalRead(ir5);
  

    double dist = measureDistanceCM();
    //Serial.println(dist);

    if (dist > 0 && dist < 10)
    {
      obstacleAvoidance();
    }
    else
    {
      lineTracking(s1, s2, s3, s4, s5);
    }
}