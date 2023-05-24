#include <Servo.h>
#include <Math.h>
#include <Ramp.h>

// Debug
#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINTL(x)  Serial.println(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
#else
  #define DEBUG_PRINTL(x)
  #define DEBUG_PRINT(x)
#endif

// MIDDLE LEFT LEG
//          x
//          ^
//          |
//          |
// y <------o

// Servo pins
#define J1Pin 2
#define J2Pin 3
#define J3Pin 4

// Servos
Servo Joint1;
Servo Joint2;
Servo Joint3;

// Constants
const double J2L = 57.0; // Length of J2 (57 - 2.35)mm
const double J3L = 110.0; // Length of J3 110mm

const double Y_Rest =  70.0;
const double Z_Rest = -80.0;

const double J3_LegAngle = 15.4;

// Joint Variables
double J1Act = 0.0;
double J2Act = 0.0;
double J3Act = 40.0;

rampDouble J1Tar = 0.0;
rampDouble J2Tar = 0.0;
rampDouble J3Tar = 40.0;

// Command Variables
bool started = false;
bool ended = false;
uint8_t commandStep = 0;

// Commands
const double lines[62][4] = {{0.0, 0.0, 40.0, 1000},
                            
                            {-30.0, 40.0, 20.0, 200},
                            {-30.0, 40.0, -20.0, 200},
                            {60.0, 40.0, -20.0, 1600},
                            {60.0, 60.0, 20.0, 200},  

                            {-30.0, 30.0, 20.0, 200},
                            {-30.0, 30.0, -20.0, 200},
                            {60.0, 30.0, -20.0, 1600},
                            {60.0, 50.0, 20.0, 200},

                            {-30.0, 20.0, 20.0, 200},
                            {-30.0, 20.0, -20.0, 200},
                            {60.0, 20.0, -20.0, 1600},
                            {60.0, 40.0, 20.0, 200},

                            {-30.0, 10.0, 20.0, 200},
                            {-30.0, 10.0, -20.0, 200},
                            {60.0, 10.0, -20.0, 1600},
                            {60.0, 30.0, 20.0, 200},

                            {-30.0, 0.0, 20.0, 200},
                            {-30.0, 0.0, -20.0, 200},
                            {60.0, 0.0, -20.0, 1600},
                            {60.0, 20.0, 20.0, 200},                                                      



                            {-30.0, 40.0, 20.0, 200},
                            {-30.0, 40.0, -20.0, 200},
                            {-30.0, 0.0, -20.0, 800},
                            {-30.0, 20.0, 20.0, 200},

                            {-20.0, 40.0, 20.0, 200},
                            {-20.0, 40.0, -20.0, 200},
                            {-20.0, 0.0, -20.0, 800},
                            {-20.0, 20.0, 20.0, 200},

                            {-10.0, 40.0, 20.0, 200},
                            {-10.0, 40.0, -20.0, 200},
                            {-10.0, 0.0, -20.0, 800},
                            {-10.0, 20.0, 20.0, 200},
                            
                            {0.0, 40.0, 20.0, 200},
                            {0.0, 40.0, -20.0, 200},
                            {0.0, 0.0, -20.0, 800},
                            {0.0, 20.0, 20.0, 200},

                            {10.0, 40.0, 20.0, 200},
                            {10.0, 40.0, -20.0, 200},
                            {10.0, 0.0, -20.0, 800},
                            {10.0, 20.0, 20.0, 200},

                            {20.0, 40.0, 0.0, 200},
                            {20.0, 40.0, -20.0, 200},
                            {20.0, 0.0, -20.0, 800},
                            {20.0, 20.0, 0.0, 200},

                            {30.0, 40.0, 0.0, 200},
                            {30.0, 40.0, -20.0, 200},
                            {30.0, 0.0, -20.0, 800},
                            {30.0, 20.0, 0.0, 200},

                            {40.0, 40.0, 0.0, 200},
                            {40.0, 40.0, -20.0, 200},
                            {40.0, 0.0, -20.0, 800},
                            {40.0, 20.0, 0.0, 200},

                            {50.0, 40.0, 0.0, 200},
                            {50.0, 40.0, -20.0, 200},
                            {50.0, 0.0, -20.0, 800},
                            {50.0, 20.0, 0.0, 200},

                            {60.0, 40.0, 0.0, 200},
                            {60.0, 40.0, -20.0, 200},
                            {60.0, 0.0, -20.0, 800},
                            {60.0, 20.0, 0.0, 200},

                            {0.0, 0.0, 40.0, 200}};

void setup() {
  // DEBUG
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  // Servo instances
  Joint1.attach(J1Pin, 500, 2500);
  Joint2.attach(J2Pin, 400, 2400);
  Joint3.attach(J3Pin, 750, 2400);

  UpdatePosition(0, 50, 30);

  delay(5000);

}

void loop() {

// Update position
J1Act = J1Tar.update();
J2Act = J2Tar.update();
J3Act = J3Tar.update();

CartesianMove(J1Act, J2Act, J3Act);

if (ended == false){
  // Check if command finished
  if (started == false | J1Tar.isFinished() == true){
    commandStep++;
    if (commandStep > 61)
    {
      ended = true;
    }

    double xMove = lines[commandStep][0];
    double yMove = lines[commandStep][1];
    double zMove = lines[commandStep][2];
    uint16_t duration = lines[commandStep][3] * 2;

    J1Tar.go(xMove, duration);
    J2Tar.go(yMove, duration);
    J3Tar.go(zMove, duration);

    started = true;
  }
}

}

void CartesianMove(double X, double Y, double Z){
// OFFSET TO REST POSITION
Y += Y_Rest;
Z += Z_Rest;

// CALCULATE INVERSE KINEMATIC SOLUTION
double J1 = atan(X / Y) * (180 / PI);
double H = sqrt((Y * Y) + (X * X));
double L = sqrt((H * H) + (Z * Z));
double J3 = acos(   ((J2L * J2L) + (J3L * J3L) - (L * L))   /   (2 * J2L * J3L)   ) * (180 / PI);
double B = acos(   ((L * L) + (J2L * J2L) - (J3L * J3L))   /   (2 * L * J2L)   ) * (180 / PI);
double A = atan(Z / H) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
double J2 = (B + A);  // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'

UpdatePosition(J1, J2, J3);
}

void UpdatePosition(double J1, double J2, double J3){
  // MOVE TO POSITION
  Joint1.write(90 - J1);
  Joint2.write(90 - J2);
  Joint3.write(J3 + J3_LegAngle - 90);

  //DEBUG_PRINT("J1: "); DEBUG_PRINTL(J1);
  //DEBUG_PRINT("J2: "); DEBUG_PRINTL(J2);
  //DEBUG_PRINT("J3: "); DEBUG_PRINTL(J3);
}


