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
#define SERVO_PIN 9
#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4
Servo head;

int a = -1;

int DefaultSpeed = 150;

Servo ultrasonicServo;

int LineTracking = 0;

int Direction = 0; //-1 right 1 left

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

  head.attach(9);
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
    if (s1 == 1 || s2 == 1 || s3 == 1 || s4 == 1 || s5 == 1)
    {
      LineTracking = 1;
      Direction = 0;
    }
    else
    {
      LineTracking = 0;
    }
    // Weights: left negative, right positive
    int w[5] = {-2, -1, 0, 1, 2};

    // Compute weighted position error
    int error = 0;
    for (int i = 0; i < 5; i++)
        error += s[i] * w[i];

    // Base speed
    int base = 120;
    
      
    

    // Steering gain
    int K = 50;

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
   
    if (dist > 0 && dist < 25)
    {
      
      if(a == -1){
        a = random(0,2);
      }
     
      Serial.println(a);

      if (a == 0){
        turn(0,500);
        Direction = 1;
      }
      else{
        turn(500,0);
        Direction = -1;
      }

      LineTracking = 0;
      
    }
    else if (LineTracking == 0 && Direction == -1 || Direction == 1)
    {
      if (Direction == 1)
      {
        turn(500,0);
      }
      else if (Direction == -1)
      {
        turn(0,500);
      }
    }
    else {
      a = -1;
    }
    if (dist > 25 ){
      lineTracking(s1, s2, s3, s4, s5);
    }
}