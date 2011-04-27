
// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

// Servo for throttle - 180 deg is no throttle tension
// hall effect sensor for engine RPM - 24 teeth
// output to starter

// motor control is main loop
// CAN polling should be an interrupt

#include <Servo.h> 
#include <PID_v1.h>

#define Kp 0.03
#define Ki 0.1
#define Kd 0.00

#define MAX_RPM 4000
#define MIN_RPM 1200
#define SERVO_PIN 54
#define FAKE_SERVO 5
int pwm;
 
Servo myservo;  // create servo object to control a servo 



// Throttle positions
const int ZERO_THROTTLE = 179;
const int MAX_THROTTLE = 0;
const int PID_DIRECTION = REVERSE; // if the zero point is 179, this must be REVERSE, but if 0, use DIRECT


//PID variables
double Setpoint;  // This is the desired RPM of the engine
double Input;     // This is the current RPM of the engine
double Output;    // This is the value sent to the servo

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, PID_DIRECTION);  // create PID for engine RPM control; last three inputs are P, I, and D parameters.
 
const int potpin = 5;  // analog pin used to connect the potentiometer
const int potpin2 = 4;
const int HALL_INTERRUPT = 4;
int val;    // variable to read the value from the analog pin 
int val2;

int teeth = 24; // 24 teeth on hall effect sensor wheel

volatile int rpmcount;
unsigned long rpm;
unsigned long timeold;

unsigned long rpmold;

unsigned long denom;
unsigned long numer;

 
void setup() 
{ 
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object 

  // initialize PID
  Input = 0;
  Setpoint = 0;

  
  // limit the Output range to that which the servo can achieve
  myPID.SetOutputLimits(0, 179);
  myPID.SetMode(AUTOMATIC);
  
  Serial.begin(115200);
  Serial.flush();

  attachInterrupt(HALL_INTERRUPT, rpm_fun, RISING); // 0 means INT0 pin, which is digital 2

  rpmcount = 0;
  rpm = 0;
  rpmold = 0;
  timeold = 0;
} 

void rpm_fun()
{
  rpmcount++;
}
 
void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val2 = analogRead(potpin2);            // reads the value of the potentiometer (value between 0 and 1023) 


  Setpoint = map(val, 0, 1023, MIN_RPM, MAX_RPM);     // scale it to use it with the servo (value between 0 and 180) 
  // ^ using the potentiometer to change the setpoint

  Serial.print("Pot1: ");
  Serial.print(val);
  Serial.print("  ");
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print("  ");
  Serial.print("Pot2: ");
  Serial.print(val2);

  myservo.write(Output);                  // sets the servo position according to the scaled value; 0-179 deg
  
  if (rpmcount >= 20)
  {
    // Update RPM every 20 counts, increase this for better RPM resolution,
    // decrease for faster update (adjust rpm calc as needed)

    Serial.print("Pot: ");
    Serial.print(val, DEC);
    Serial.print("  rpmcount: ");
    Serial.print(rpmcount, DEC);

    // RPM = count / teeth / T
    numer = rpmcount * 60000;
    rpmcount = 0;
    denom = teeth * (millis() - timeold);
    timeold = millis();
    rpm = numer / denom;
    rpmold = rpm;
    Input = rpm;

    /*
    Serial.print(numer, DEC);
    Serial.print("  ");
    Serial.print(denom, DEC);
    Serial.print("  rpmcount: ");
    Serial.print(rpmcount, DEC);
    Serial.print("  d time: ");
    Serial.print(millis() - timeold, DEC);
    Serial.print("  timeold: ");
    Serial.print(timeold, DEC);
    Serial.print("  millis: ");
    Serial.print(millis(), DEC);
    */

    
    Serial.print("  RPM: ");
    Serial.print(rpm, DEC);

    Serial.print("  PID:");
    Serial.print("  SetPoint: ");
    Serial.print(Setpoint, DEC);
    Serial.print("  In: ");
    Serial.print(Input, DEC);
    Serial.print("  Out: ");
    Serial.print(Output, DEC);




    myPID.Compute();
    pwm = Output;
    analogWrite(FAKE_SERVO, pwm);
  }

  Serial.println();

  delay(15);                           // waits for the servo to get there 
} 
