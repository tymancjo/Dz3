// refactored and hopefully made better code for Dżordż3
// The small robot project.
// version 2.00 - inital second build
// 11.2020 - start work

// idea and TODO:
// better read from serial implemented - or chenges to work as function
// non blocking main loop - how to handle imminent STOP and SoftSTOP?
// 

// **************************
// libraries import handling
#include <Arduino.h>
// for stepper motor controls
#include <AccelStepper.h>
#include <MultiStepper.h>
// for external I2c PWM driver
#include <Adafruit_PWMServoDriver.h>
// for I2C communication
#include <Wire.h>


// *************************
// General constans and initial definitions
//Binding stepper motors
AccelStepper stepper1(AccelStepper::DRIVER, 8, 9);
AccelStepper stepper2(AccelStepper::DRIVER, 6, 7);

// inputs of front and back "bumpers" sensors 
#define frontBumber A6
#define backBumper A7

// kicking of the pwm module definition 
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// defining the values used for PWM servo control 
// those were selected after experiment with the real used hw servos
#define SERVOMIN  160 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  505 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates



// Stuff for serial communication use
bool newData = false;
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
// variables to hold the parsed data
char messageFromPC[numChars] = {0};
// global variables to keep the received data 
float param[3];
int command;

// constans and variables describing the real object and mechanical limits
const float max_speed_limit = 1150 * 4;  // for 1/8 step
float max_speed = 900 * 4;  // for 1/8 step
const float safe_speed = 300 * 4; // for 1/8 step
const float accl = 1000 * 4; // for the 1/8 step
const int pulses_per_step = 200 * 8; // for 1/8 step

// mechanical dimensions based data
const float A = 159.0; //[mm] dist between wheels (width)
const float D = 90; //[mm] wheel diameter
const float wheel_lenght = PI * D;
const float turn_const = A / (180.0 * D);

// boolean flags for moving operations
// to hold the front and back bumber status
// and the move direction 
bool stop_front = false;
bool stop_back = false;
bool going_forward = false;
bool masterStop = false;

// variables and constans for path programming
const int maxStepsNumber = 100; // predefined max number of program steps
int program_steps = 0;
int current_program_step = 0;
// arrays to keep program steps
int memoryL[maxStepsNumber];
int memoryA[maxStepsNumber];
int mempryS[maxStepsNumber];
// variables for undo needs
int lastL;
int lastA;
int lastV;
int newV;

// for servos attached to the PWM module
// initial reset position
const int pos0 = 0;
// total servo canals count
const int servo_count = 16;

// *************************************
// declaration of functions to be defined later
void goNormal(float floatFrom_01, float floatFrom_02, float floatFrom_03);
void runSequence(bool inloop, bool resumeit);
void addToSequence(int p1, int p2, int p3);
void goSoftStop();
void goStop();
void recvWithStartEndMarkers();
void parseData();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Port operation to disable both stepper motors (to not keep them under current)
  // PB3 is for !enable
  PORTB |= 0b00000100;

  // Setting timer interrupt fuction and timing
  // used later to keep sendint hte run command to steppers
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 100  - 1; // 10ms to sent the run order 
  TCCR2A |= (1 << WGM21);

  // Setting up the pinmodes
  pinMode(10, OUTPUT);
  pinMode(frontBumber, INPUT_PULLUP);
  pinMode(backBumper, INPUT_PULLUP);

  // Defining the stepper motors objects
  stepper1.setMaxSpeed(max_speed);
  stepper1.setAcceleration(accl);

  stepper2.setMaxSpeed(max_speed);
  stepper2.setAcceleration(accl);

  // preparing the PWM module to work with servos:
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10); // Just to give it some time to catch up...

  Serial.println("Startup...");
  Serial.println("Zeroing servos on PWM module...");

  // looping over servos to put all in initial position
  for (int i=0; i < servo_count; i++){
    Serial.print(i);
    pwm.setPWM(i, 0, (int) map(pos0,0,180,SERVOMIN,SERVOMAX));
    delay(10);
    }

// // time related variables
// // just for timing matter
// unsigned long timerx = micros();
// unsigned long deltat;

}

void loop() {
  // put your main code here, to run repeatedly:
    //check bumpers
  if (analogRead(frontBumber) < 200 && !stop_front) stop_front = true;
  else stop_front = false;

  if (analogRead(backBumper) < 200 && !stop_back) stop_back = true;
  else stop_back = false;

  // grabstuff from serial
  recvWithStartEndMarkers();

  if (newData == true) {

    strcpy(tempChars, receivedChars);

    parseData();
    newData = false;

    Serial.print("command: ");
    Serial.print(command);

    // data are received in form <command, x - Dist[cm], y Angle[deg],z  speed>
    // making decisions base on command
    switch (command) {
      case 0:
        // stop action
        masterStop = true;
        goStop();
        break;

      case 1:
        // execute normal route <command, Dist[cm], Angle[deg],speed>
        //memorizing the move
        lastL = param[0];
        lastA = param[1];
        lastV = max_speed;
        newV = param[2];

        goNormal(param[0], param[1], param[2]);

        break;

      case 8:
        // soft stop action
        goSoftStop();
        masterStop = true;
        break;

      case 9:
        // undo last move
        goNormal(-lastL, -lastA, 0);// moving back
        // waiting to finish move
        while (!(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0)) {
          // just wait in loop for the step to finish.
          Serial.print(".");
        }
        goNormal(0, 0, lastV);

        break;

      case 20:
        // add to sequence
        addToSequence(param[0], param[1], param[2]);
        break;

      case 21:
        // add last move to sequence
        addToSequence(lastL, lastA, newV);
        Serial.println("Last move stored...");
        break;

      case 22:
        // execute normal route <command, Dist[cm], Angle[deg],speed>
        // and add to sequence
        lastL = param[0];
        lastA = param[1];
        lastV = max_speed;
        newV = param[2];

        // move
        goNormal(param[0], param[1], param[2]);
        // add to sequence
        addToSequence(param[0], param[1], param[2]);

        break;

      case 29:
        //remove last from memory

        if (program_steps > 0) {
          program_steps--;
        }
        break;

      case 30:
        // execute sequence
        runSequence(false, false);
        break;

      case 31:
        // resume sequence
        runSequence(false, true);
        break;

      case 33:
        // execute sequence in loop
        runSequence(true, false);
        break;

      case 39:
        //remove last from memory
        program_steps = 0;

        break;

      case 41:
        // set servos position control
        // params:
        // 0 - servo chanellnumber
        // 1- set degrees (0 to 180 deg)
        
        // int selectedServo = (int) param[0];
        // int pos = (int) constrain(param[1], 0,180);
        {
        int selectedServo = (int) param[0];
        int pos = (int) constrain(param[1], 0,180);

        // sent thecommand to servo
        pwm.setPWM(selectedServo, 0, (int) map(pos,0,180,SERVOMIN,SERVOMAX)); 
        
        Serial.print(pos);
        Serial.println(" servo, #D");
        }
        break;

      // case 42:
      //   break;
        
      default:
        // statements
        break;
    }




  }


  //engines turn off
  if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
    // PB3 is for !enable
    PORTB |= 0b00000100;
  }


  // deltat = micros() - timerx;
  // timerx = micros();
}



// **************************************
// *****         FUNCTIONS        *******
// **************************************

// Handling reading from the Serial
void recvWithStartEndMarkers() {
  //  Read data in this style <M, P, R>
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}
//============

void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  command = atoi(strtokIndx);     // convert this part to an float

  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC

  strtokIndx = strtok(NULL, ",");
  param[0] = atoi(strtokIndx);     // convert this part to a float


  strtokIndx = strtok(NULL, ",");
  param[1] = atoi(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  param[2] = atoi(strtokIndx);     // convert this part to a float

}

// *********************************************
// timer int subroutine
ISR(TIMER2_COMPA_vect) {

  // trying to read the bumber here - to be sure we do read it always
  //check bumpers
  // if (analogRead(frontBumber) < 200 && !stop_front) stop_front = true;
  // else stop_front = false;

  // if (analogRead(backBumper) < 200 && !stop_back) stop_back = true;
  // else stop_back = false;


  //emergency breaking front/back
  if (going_forward && (stepper1.isRunning() || stepper2.isRunning()) && stop_front) {

    stepper1.setAcceleration( 1e3 * accl );
    stepper2.setAcceleration( 1e3 * accl );

    stepper1.stop();
    stepper2.stop();

    PORTB |= 0b00000100;

    stepper1.move(0);
    stepper2.move(0);

    stop_front = false;

  } else if (!going_forward && (stepper1.isRunning() || stepper2.isRunning()) && stop_back) {

    stepper1.setAcceleration( 1e3 * accl );
    stepper2.setAcceleration( 1e3 * accl );

    stepper1.stop();
    stepper2.stop();

    PORTB |= 0b00000100;

    stepper1.move(0);
    stepper2.move(0);

    stop_back = false;

  } else {

    if (!masterStop){
    stepper1.run();
    stepper2.run();
    } else {
    PORTB |= 0b00000100;
    }

    // below is just for reference - not sed in Dz3
    //  stepper1.runSpeed();
    //  stepper2.runSpeed();
  }
}


// ***************************************
// ******* Dżordż 3 control Functions ****
// ***************************************

void goNormal(float floatFrom_01, float floatFrom_02, float floatFrom_03) {
  // releasing the potential master stop
  masterStop = false;
  // normal move as previously
  if (floatFrom_03 != 0) {
    // set new max speed for this move
    max_speed = floatFrom_03;
    if ( abs(max_speed) > max_speed_limit ) max_speed =  max_speed_limit;
  }

  going_forward = (floatFrom_01 > 0);

  float turn_dist = 0;
  float ride_dist = 10 * floatFrom_01 / wheel_lenght;
  float Alpha = floatFrom_02;

  if (Alpha != 0 ) {
    turn_dist = Alpha * turn_const * 0.5; // 0.5 as we add to one wheel and substract form the other
  }

  // Serial.print( ride_dist);
  // Serial.print(",");
  // Serial.println( turn_dist);

  // safety trigger to not start move if we are next to obstacle
  if ( (ride_dist < 0 && stop_back) || (ride_dist > 0 && stop_front)) {
    ride_dist = 0;
  }

  long right = (long) (-1 * (ride_dist + turn_dist) * pulses_per_step);
  long left = (long) ((ride_dist - turn_dist) * pulses_per_step);

  // Serial.print( right);
  // Serial.print(",");
  // Serial.println( left);


  if (abs(right) < abs(left)) {
    // need to recalc speeds to be sure getther in the sametime

    stepper1.setMaxSpeed( max_speed * (abs((float)right) / abs((float)left)) );
    stepper1.setAcceleration( accl * (abs((float)right) / abs((float)left)) );

    stepper2.setMaxSpeed( max_speed );
    stepper2.setAcceleration( accl);
  }
  if (abs(right) > abs(left)) {
    // need to recalc speeds to besure getther in the sametime
    stepper2.setMaxSpeed( max_speed * (abs((float)left) / abs((float)right)) );
    stepper2.setAcceleration( accl * (abs((float)left) / abs((float)right)) );

    stepper1.setMaxSpeed( max_speed );
    stepper1.setAcceleration( accl);
  }
  if (abs(right) == abs(left)) {
    // need to recalc speeds to besure getther in the sametime
    stepper1.setMaxSpeed( max_speed );
    stepper1.setAcceleration( accl );

    stepper2.setMaxSpeed( max_speed );
    stepper2.setAcceleration( accl );
  }

  // Turning the stepper motors ON
  // PB3 is for !enable
  PORTB &= 0b11111011;

  stepper1.move(right);
  stepper2.move(left);
} // end of goNormal

// *****************88

void runSequence(bool inloop, bool resumeit) {
  bool inSequence = false;
  int starti = 0;
  // case we are resuming - starting from last step
  if (resumeit) starti = current_program_step + 1;

  //masterloop - infinite, risky bot seems to be the oly solution.
  while (true) {
    // running saved sequence
    if ( program_steps > 0 ) {
      Serial.print("starting sequence of ");
      Serial.print(program_steps);
      Serial.println("steps");
      
      inSequence = true;

      // if we have something in the arrays
      for (int i = starti; i < program_steps; i++) {
        // safety check
        if (!inSequence) break;

        // making the step
        Serial.print("Step ");
        Serial.println(i);
        // memorizing where we are in program
        current_program_step = i;

        // moving normal command
        goNormal(memoryL[i], memoryA[i], mempryS[i]);

        // waiting to be completed
        while (!(stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0)) {
          // just wait in loop for the step to finish.
          // check bumpers
          if (analogRead(frontBumber) < 200 && !stop_front) {
            stop_front = true;
            inSequence = false;
            break;
          }
          else {
            stop_front = false;
          }

          if (analogRead(backBumper) < 200 && !stop_back) {
            stop_back = true;
            inSequence = false;
            break;
          }
          else {
            stop_back = false;
          }
                    
          // grabstuff from serial - as we are here in the loop something might come...
          recvWithStartEndMarkers();
          if (newData == true) {
            strcpy(tempChars, receivedChars);
            parseData();
            newData = false;

            Serial.print("command: ");
            Serial.print(command);

            if (command == 0 ) {
              goStop();
              inSequence = false;
              break;
            }
          }

        }
      }
      starti = 0;
    }
    // the exit from infinite while condition
    // we exit only if we are not in loop or we are not in sequence
    // elase we go back and start all over again.
    if (!inloop || !inSequence) break;
  }
}


// **************************


void addToSequence(int p1, int p2, int p3) {
  // adding the step to memory
  if (program_steps < maxStepsNumber) {

    memoryL[program_steps] = p1;
    memoryA[program_steps] = p2;
    mempryS[program_steps] = p3;

    program_steps++;

    Serial.print("Step added, total: ");
    Serial.print(program_steps);
    Serial.println(" steps");

  }

  else Serial.print("Memory full");
}

void goStop() {
  // emergency stop asap
  masterStop = true;
  stepper1.setAcceleration( 1e3 * accl );
  stepper2.setAcceleration( 1e3 * accl );

  stepper1.stop();
  stepper2.stop();

  PORTB |= 0b00000100;

  stepper1.move(0);
  stepper2.move(0);
}

void goSoftStop() {
  // soft stop asap
  stepper1.setAcceleration( accl );
  stepper2.setAcceleration( accl );

  stepper1.stop();
  stepper2.stop();
}


