/* Competition flight code for the 2018 USLI Competition.
 *  Tanner Oakes
 *  Austen LeBeau
 */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// pointer to motor at port 1
Adafruit_DCMotor *myMotor = AFMS.getMotor(4);
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


// VARS
const int channelA = 3;
const int channelB = 4;
int encoderTicks = 0;                // ticks for encoder
bool hasLaunched = false;
bool hasBurnedOut = false;
bool flightDone = false;

float accelZ;                               //z acceleration
float altCurrent = 0;                       //current altitude
float altInitial = 0;                       //initial calculated altitude
float altPrevious = 0;                      //previous altitude
float apogee = 5280;                        //desired final height
float calculatedApogee;                     //apogee calculated from previous variables
float startTimer;                           //start of rocket launch
float velTimer1 = 0;                        //initial velocity timer
float velTimer2 = 0;                        //final velocity timer
float velZ = 0;                             //z velocity

// pid loop 
float P_naught = 0;
float I_naught = 0;


void setup() {
  // Initialize the sensor 
  Serial.begin(9600);
  // sets the modes of the channel pins to input
  pinMode(channelA, INPUT);
  pinMode(channelB, INPUT);

  // create with the default frequency 1.6KHz
  AFMS.begin();

  // attach interrupt to pin 3 for encoder output
  attachInterrupt(digitalPinToInterrupt(3), channelAEvent, CHANGE);

  // test acceleration sensor
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  // test barometric pressure sensor
  if(!bmp.begin())
  {
    // There was a problem detecting the BMP085 ... check your connections 
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  // set initial Alt... averaged from 20 readings
  for (int i = 0; i < 20; i++) {
    altInitial += calcAlt();
  }
  altInitial /= 20;

  // launch pad test actuation test 
  myMotor->setSpeed(50);
  while (encoderTicks < 170) {
    myMotor->run(BACKWARD);
    Serial.print("count: ");
    Serial.println(encoderTicks);
    Serial.println();
  }
  while (encoderTicks >= -15) {
    myMotor->run(FORWARD);
    Serial.print("count: ");
    Serial.println(encoderTicks);
    Serial.println();
  }
  myMotor->run(RELEASE);
  
}

// main execution
void loop() {
 
  // PHASE 1 (PRE-LAUNCH)
  while (!hasLaunched) {
    accelZ = getAccelZ();
    Serial.println(accelZ);
    //if accel is pos, we have launched
    if (accelZ < 0) {
      hasLaunched = true;
      // start timer
      startTimer = millis();  
    }
  }
  
  // PHASE 2 (Motor-Burn)
  while (!hasBurnedOut) {
    accelZ = getAccelZ();
    Serial.println(accelZ);
    //once accel is neg, the motor has burned out
    if (accelZ > 0) {
      hasBurnedOut = true;  
    }
  }

  // PHASE 3 (Post-Burn Action)
  altPrevious = altCurrent;
  altCurrent = calcAlt() - altInitial;

  //calculate velocity
  velTimer2 = millis();
  velZ = calcVelZ(velTimer1, velTimer2, altCurrent, altPrevious);
  velTimer1 = millis();


  int newPosition = pid_loop(encoderTicks, altCurrent, velZ, accelZ);
  
  while (encoderTicks <= newPosition) {
    myMotor->run(FORWARD);
  }
  myMotor->run(RELEASE);
  while (encoderTicks >= newPosition) {
    myMotor->run(BACKWARD);
  }
  myMotor->run(RELEASE);
  
  // PHASE 4 (Post-Action Regress)
  // detect significant drop in altitude and then return system home and shut off controller
  if (velZ < 0) {
    // return home and shut down
  }
  
  delay(500);

}

/* ISR that triggers whenever channel A changes state.
 keeps track of ticks on motor... opposite states mean cw rotation
 and same states mean ccw rotation of motor. */
void channelAEvent() {
  if (digitalRead(channelA) == HIGH) {
    if (digitalRead(channelB) == LOW) {
      encoderTicks++;
    }
    else {
      encoderTicks--;
    }
  }
  else {
    if (digitalRead(channelB) == LOW) {
      encoderTicks--;
    }
    else {
      encoderTicks++;
    }
  }
}

// produces deterministic output from drag plates
int pid_loop(int finPosition, float altitude, float velocity, float accel) {
    // Pid Loop constants.
    float kp = 0.075f;             // kp, ki, and kd 
    float ki = 0.01f;              // are the pid loop gain factors.
    float kd = 0.05f;
    float setpoint = 5280;          // Target altitude in feet.
    float projectedAltitude = projected_altitude(velocity, accel, altitude);

    // Proportional term.
    float P = (projectedAltitude) - setpoint;
    
    float timeDiff = (velTimer2 - velTimer1) / 1000;
    // Integral term.
    float I = I_naught + (P * timeDiff);
    if (I > 10) { I = 10; }         // These if statements are here to 
    else if (I < -10) { I = -10; }  // prevent integral windup.
    // Derivative term.
    float D = (P - P_naught) / 2; 

    // diffOutput is the amount that the position of the fins change.
    // For example, if the projected altitude is exactly 5280 feet (the setpoint),
    // the diffOutput will be equal to zero. If the projected altitude is below,
    // the diffOutput will become negative, and vice versa.
    float diffOutput = (P * kp) + (I * ki) + (D * kd);

    // This block saves the variables in this loop so it can be used in the next.
    P_naught = P;
    I_naught = I;
    altPrevious = altitude;

    // newPosition is the new position of the the fins.
    float newPosition = finPosition + diffOutput;
    newPosition = (int) newPosition;
    // These prevent the new position of the fins from going over their max and min values.
    if (newPosition > 170) { newPosition = 170; }
    if (newPosition < -15) { newPosition = -15; }
    return newPosition;
}

// returns current altitude
float calcAlt() {

  /* Get a new sensor event (needed for the 10 dof) */ 
  sensors_event_t event;
  bmp.getEvent(&event);

  float alt;

  if (event.pressure)
  {   
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

    //calc altitude, convert to feet (from meters)
    alt = bmp.pressureToAltitude(seaLevelPressure,event.pressure) * 3.280840;

    return alt;
  }
}

// calculates velocity for the Z-axis
float calcVelZ(float timer1, float timer2, float altCurrent, float altPrevious) {
  float velocity;
  float timerDiff;

  //loop timer
  timerDiff = (velTimer2 - velTimer1) / 1000;
  
  //calc vel
  velocity = (altCurrent - altPrevious) / timerDiff;

  return velocity;
}

// calculates acceleration for the z-axis 
float getAccelZ () {

  float zAccel;

  sensors_event_t event2; 
  accel.getEvent(&event2);

  zAccel = event2.acceleration.z;
  //calc to ft/s
  zAccel *= -3.280840;

  return zAccel;
  
}

float projected_altitude(float veloc, float accel, float currentAlt) {
    // Calculate the projected altitude, this is important because the PID
    // loop must know how far off the apogee is from the target altitude.
    float funcConst = (2 * veloc * veloc) / (accel - 32.174);
    float projectedAltitude = sqrt(funcConst) + currentAlt;
    return projectedAltitude;

}
