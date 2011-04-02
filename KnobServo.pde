// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

// Servo for throttle - 180 deg is no throttle tension
// hall effect sensor for engine RPM - 24 teeth
// output to starter

// motor control is main loop
// CAN polling should be an interrupt

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
 
int potpin = 2;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 

int teeth = 24; // 24 teeth on hall effect sensor wheel

volatile byte rpmcount;
unsigned long rpm;
unsigned long timeold;

unsigned long rpmold;

unsigned long denom;
unsigned long numer;
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 

  Serial.begin(9600);
  Serial.flush();

  attachInterrupt(0, rpm_fun, RISING); // 0 meant INT0 pin, which is digital 2

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
  val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  myservo.write(val);                  // sets the servo position according to the scaled value 
  
  if (rpmcount >= 100)
  {
    // Update RPM every 20 counts, increase this for better RPM resolution,
    // decrease for faster update (adjust rpm calc as needed)

    // RPM = count / teeth / T
    numer = rpmcount * 60000;
    rpmcount = 0;
    denom = teeth * (millis() - timeold);
    rpm = numer / denom;

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

    rpmold = rpm;
    timeold = millis();
    
    Serial.print("  RPM: ");
    Serial.println(rpm, DEC);
  }

  delay(15);                           // waits for the servo to get there 
} 
