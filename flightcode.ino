/* Competition flight code for the 2018 USLI Competition.
 *  Tanner Oakes
 *  Austen LeBeau
 */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_MMA8451.h>
#include <SPI.h>
#include <SD.h>


#define SEALEVELPRESSURE_HPA (1013.25)

const int chipSelect = 10;

// motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// pointer to motor at port 1
Adafruit_DCMotor *myMotor = AFMS.getMotor(4);
/* Assign a unique ID to this sensor at the same time */
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_BME280 bme;


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
float startTimer;                           //start of rocket launch
float projectedAltitude = 0;
float loopCtr = 0;
float timeDiff = 0;
float velTimer1 = 0;                        //initial velocity timer
float velTimer2 = 0;                        //final velocity timer
float velZ = 0;                             //z velocity

File dataFile;

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

  // initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card fail");
  }
  dataFile = SD.open("launch.txt", FILE_WRITE);
  if (dataFile) {
    Serial.print("Writing to test.txt...");
    dataFile.println("testing 1, 2, 3.");
    dataFile.println("-------------- LAUNCH DATA --------------");
    dataFile.println();
    // close the file:
    dataFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
//  dataFile.close();
  
  // test pressure sensor 
  bool status;
  status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    
  // test acceleration sensor
  if (! mma.begin()) {
    Serial.println("Couldnt start mma !!!! Wiring check.");
    while (1);
  }
  mma.setRange(MMA8451_RANGE_2_G);

  // set initial Alt... averaged from 20 readings
  for (int i = 0; i < 20; i++) {
    altInitial += calcAlt();
  }
  altInitial /= 20;

  // launch pad test actuation test 
  myMotor->setSpeed(150);
  while (encoderTicks < 170) {
    myMotor->run(FORWARD);
    Serial.print("count: ");
    Serial.println(encoderTicks);
    Serial.println();
  }
  while (encoderTicks >= -15) {
    myMotor->run(BACKWARD);
    Serial.print("count: ");
    Serial.println(encoderTicks);
    Serial.println();
  }
  myMotor->run(RELEASE);
  
}

// main execution
void loop() {
  loopCtr++;
 
  // PHASE 1 (PRE-LAUNCH)
  while (!hasLaunched) {
    accelZ = getAccelZ();
    Serial.println(accelZ);
    //if accel is pos, we have launched
    if (accelZ > 0) {
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
    if (accelZ < 0) {
      hasBurnedOut = true;  
    }
  }

  // PHASE 3 (Post-Burn Action)
  altPrevious = altCurrent;
  altCurrent = calcAlt() - altInitial;

  //calculate velocity
  velTimer2 = millis();
  timeDiff = (velTimer2 - velTimer1) / 1000;
  velZ = calcVelZ(velTimer1, velTimer2, altCurrent, altPrevious);
  velTimer1 = millis();


  int newPosition = pid_loop(encoderTicks, altCurrent, velZ, accelZ, timeDiff);
  
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

  printData(dataFile, loopCtr, newPosition, altCurrent, velZ, accelZ, timeDiff);
    
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
int pid_loop(int finPosition, float altitude, float velocity, float accel, float timeDiff) {
    // Pid Loop constants.
    float kp = 0.075f;             // kp, ki, and kd 
    float ki = 0.01f;              // are the pid loop gain factors.
    float kd = 0.05f;
    float setpoint = 5280;          // Target altitude in feet.
    projectedAltitude = projected_altitude(velocity, accel, altitude);

    // Proportional term.
    float P = (projectedAltitude) - setpoint;
    
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
  
  float alt;
  alt = bme.readAltitude(SEALEVELPRESSURE_HPA) * 3.280840;

  return alt;
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
  
  mma.read();
  
  sensors_event_t event; 
  mma.getEvent(&event);

  zAccel = event.acceleration.z;
  //calc to ft/s
  zAccel *= -3.280840;

  return zAccel;
  
}

// calulation for projected altitude
float projected_altitude(float veloc, float accel, float currentAlt) {
    // Calculate the projected altitude, this is important because the PID
    // loop must know how far off the apogee is from the target altitude.
    float termV = sqrt((32.174 * veloc * veloc) / -(accel + 32.174));
    float funcConst = ((veloc * veloc) + (termV * termV)) / (termV * termV);
    float projection = ((termV * termV) / (2 * 32.174)) * log(funcConst) + currentAlt;
    return projection;
}

// prints out flight data to micro sd card breakout
void printData(File df, int loopCtrIn, float posIn, float altIn, float velIn, float accelIn, float timeDiffIn) {
  df = SD.open("launchdata.txt", FILE_WRITE);
  df.print("--------------   LOOP ");
  df.print(loopCtrIn);
  df.println("    --------------");
  df.print("plate position: ");
  df.println(posIn);
  df.print("current altitude: ");
  df.println(altIn);
  df.print("velocity: ");
  df.println(velIn);
  df.print("acceleration: ");
  df.println(accelIn);
  df.print("projected apogee: ");
  df.println(projectedAltitude);
  df.print("time difference: ");
  df.println(timeDiffIn);
  df.close();
}

