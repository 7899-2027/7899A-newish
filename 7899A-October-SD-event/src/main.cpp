/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      10-12-26                                  */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller1;

pneumatics clawPiston = pneumatics(Brain.ThreeWirePort.A);
pneumatics flipPiston = pneumatics(Brain.ThreeWirePort.H);

inertial imu = inertial(PORT17);
inertial Gyro = inertial(PORT1);

motor left_front = motor(PORT19, ratio6_1, true);
motor left_back = motor(PORT5, ratio6_1, false);

motor right_front = motor(PORT15, ratio6_1, false);
motor right_back = motor(PORT9, ratio6_1, true);

motor left_lift = motor(PORT20, ratio18_1, false);
motor right_lift = motor(PORT14, ratio18_1, true);

motor claw = motor(PORT20, ratio6_1, true);

int clawMode = 0;
int flipMode = 0;

struct nameMotor {
  motor &m;
  const char* name;
};
#define NAMED(mtr) { mtr, #mtr }

nameMotor driveMotors[] = {
  NAMED(left_front),
  NAMED(left_back),
  NAMED(right_front),
  NAMED(right_back),
  NAMED(left_lift),
  NAMED(right_lift),
  NAMED(claw)
};

const int NUM_DRIVE_MOTORS = sizeof(driveMotors) / sizeof(driveMotors[0]);
const int YOFFSET = 20;
const int ROW_HEIGHT = 30;

void motorDisplay(int y, double curr, double temp){
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(5, YOFFSET + y, "Current %.1fA", curr);
  if (curr < 1){
    Brain.Screen.setFillColor(green);
  }
  else if (curr <= 2.5){
    Brain.Screen.setFillColor(yellow);
  }
  else {
    Brain.Screen.setFillColor(red);
  }
  Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);

  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(160, YOFFSET + y, "Temp %.1fC", temp);
  if (temp < 45){
    Brain.Screen.setFillColor(green);
  }
  else if (temp <= 50){
    Brain.Screen.setFillColor(yellow);
  }
  else {
    Brain.Screen.setFillColor(red);
  }
  Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
  Brain.Screen.setFillColor(transparent);
}

void displayMotorData() {
  for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
    nameMotor &nm = driveMotors[i];
    int y = 1+ i * ROW_HEIGHT;

    if (nm.m.installed()){
      motorDisplay(y, nm.m.current(amp), nm.m.temperature(celsius));
      Brain.Screen.printAt(300, YOFFSET + y, nm.name);
    }
    else {
      Brain.Screen.printAt(5, YOFFSET + y,true, "%s Problem", nm.name);
    }
}
}
// ============================================================
// ODOMETRY STARTER  (drive-motor encoders + IMU heading)
// Paste this below `int flipMode = 0;` in your main.cpp
//
// Field coordinates (inches):
//   +Y = forward at heading 0, +X = right, heading is CLOCKWISE-positive degrees
//   (this matches how the VEX inertial sensor reports rotation)
// ============================================================

// ---------- CONFIG: change these to match your robot ----------
const double WHEEL_DIAMETER_IN = 3.25;  // 2.75, 3.25 or 4.0 for standard VEX omnis
const double GEAR_RATIO        = 4.0/3.0;   // wheel turns per ONE motor turn
                                        // (e.g. 48T motor gear -> 36T wheel gear = 48/36... see note below)
                                        // = (driving teeth) / (driven teeth)
const double IN_PER_DEG = (M_PI * WHEEL_DIAMETER_IN * GEAR_RATIO) / 360.0;

// ---------- Robot pose (shared between threads) ----------
vex::mutex poseMutex;
double poseX = 0.0;        // inches
double poseY = 0.0;        // inches
double poseHeading = 0.0;  // degrees, clockwise positive (from IMU)

// ---------- Helpers ----------
double wrap180(double deg) {
  while (deg > 180.0)  deg -= 360.0;
  while (deg <= -180.0) deg += 360.0;
  return deg;
}

double leftSideDegrees() {
  return (left_front.position(degrees) + left_back.position(degrees)) / 2.0;
}

double rightSideDegrees() {
  return (right_front.position(degrees) + right_back.position(degrees)) / 2.0;
}

void setPose(double x, double y, double headingDeg) {
  poseMutex.lock();
  poseX = x;
  poseY = y;
  poseHeading = headingDeg;
  poseMutex.unlock();
  imu.setRotation(headingDeg, degrees);
}

// Thread-safe getters
double getX()       { poseMutex.lock(); double v = poseX;       poseMutex.unlock(); return v; }
double getY()       { poseMutex.lock(); double v = poseY;       poseMutex.unlock(); return v; }
double getHeading() { poseMutex.lock(); double v = poseHeading; poseMutex.unlock(); return v; }

// ---------- The odometry loop ----------
int odometryLoop() {
  double prevLeft  = leftSideDegrees();
  double prevRight = rightSideDegrees();
  double prevHeadingRad = imu.rotation(degrees) * M_PI / 180.0;

  while (true) {
    double curLeft  = leftSideDegrees();
    double curRight = rightSideDegrees();
    double curHeadingDeg = imu.rotation(degrees);
    double curHeadingRad = curHeadingDeg * M_PI / 180.0;

    // Change in each side since last loop, converted to inches
    double dLeft  = (curLeft  - prevLeft)  * IN_PER_DEG;
    double dRight = (curRight - prevRight) * IN_PER_DEG;

    // Forward travel of the robot's center
    double dForward = (dLeft + dRight) / 2.0;

    // Use the average heading over this step (better than start or end alone)
    double avgHeading = (prevHeadingRad + curHeadingRad) / 2.0;

    poseMutex.lock();
    poseX += dForward * sin(avgHeading);
    poseY += dForward * cos(avgHeading);
    poseHeading = curHeadingDeg;
    poseMutex.unlock();

    prevLeft = curLeft;
    prevRight = curRight;
    prevHeadingRad = curHeadingRad;

    wait(10, msec);  // 100 Hz update
  }
  return 0;
}

// ---------- Brain screen debug ----------
int displayLoop() {
  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("X: %.2f in", getX());
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Y: %.2f in", getY());
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Heading: %.2f deg", getHeading());
    wait(50, msec);
  }
  return 0;
}

// ---------- Call this once from pre_auton() ----------
void initOdometry() {
  imu.calibrate();
  while (imu.isCalibrating()) wait(20, msec);   // IMU MUST finish calibrating, robot must be still

  left_front.resetPosition();  left_back.resetPosition();
  right_front.resetPosition(); right_back.resetPosition();
  imu.resetRotation();

  setPose(0, 0, 0);

  task odomTask(odometryLoop);
  task dispTask(displayLoop);
}

// ============================================================
// OPTIONAL: simple motion helpers that use the pose above
// ============================================================
// Spin each side of the drive at a percent speed (-100 to 100)
void drivePercent(double leftPct, double rightPct) {
  left_front.spin(forward, leftPct, percent);
  left_back.spin(forward, leftPct, percent);
  right_front.spin(forward, rightPct, percent);
  right_back.spin(forward, rightPct, percent);
}

void driveVolts(double leftPct, double rightPct) {
  double leftVolts = leftPct * 12.0 / 100.0;
  double rightVolts = rightPct * 12.0 / 100.0;
  left_front.spin(forward, leftVolts, voltageUnits::volt);
  left_back.spin(forward, leftVolts, voltageUnits::volt);
  right_front.spin(forward, rightVolts, voltageUnits::volt);
  right_back.spin(forward, rightVolts, voltageUnits::volt);
}

void stopDrive() {
  left_front.stop(brake);  left_back.stop(brake);
  right_front.stop(brake); right_back.stop(brake);
}

void stopDriveCoast() {
  left_front.stop(coast);  left_back.stop(coast);
  right_front.stop(coast); right_back.stop(coast);
}

// Turn in place to an absolute field heading (degrees)
void turnToHeading(double targetDeg, double timeoutMs = 2000) {
  const double kP = 1.0;          // percent per degree of error
  const double maxPct = 80;       // turn speed cap
  double start = Brain.timer(msec);
  while (Brain.timer(msec) - start < timeoutMs) {
    double err = wrap180(targetDeg - getHeading());
    if (fabs(err) < 1.0) break;
    double v = kP * err;
    if (v > maxPct) v = maxPct;
    if (v < -maxPct) v = -maxPct;
    drivePercent(v, -v);     // positive error = turn clockwise (left faster)
    wait(10, msec);
  }
  stopDrive();
}

// Drive straight a set distance in inches (negative = backward).
// Measures distance with the drive encoders and holds the heading the robot
// started with (via the IMU), so it drives in a straight line.
//   inches     : distance to travel, e.g. 24 or -12
//   maxPct     : speed cap in percent (0-100)
//   timeoutMs  : give up after this long
void inchDrive(double inches, double maxPct = 80, double timeoutMs = 4000) {
  const double kP = 7.0;          // percent per inch of distance error
  const double kD = 0.4;          // damping, reduces overshoot
  const double kP_heading = 1.25; // percent per degree of heading error
  const double minPct = 12;       // keeps the robot moving through the last inch
  const double tolerance = 0.5;   // inches

  double startLeft  = leftSideDegrees();
  double startRight = rightSideDegrees();
  double holdHeading = getHeading();   // keep facing this way the whole drive

  double start = Brain.timer(msec);
  double prevError = inches;
  int settled = 0;

  while (Brain.timer(msec) - start < timeoutMs) {
    // Distance traveled since the call started
    double traveled = ((leftSideDegrees()  - startLeft) +
                       (rightSideDegrees() - startRight)) / 2.0 * IN_PER_DEG;
    double error = inches - traveled;

    // Done once we've been inside tolerance for 5 loops (50 ms)
    if (fabs(error) < tolerance) {
      if (++settled >= 5) break;
    } else {
      settled = 0;
    }

    double fwd = kP * error + kD * (error - prevError) / 0.01;
    prevError = error;

    // Clamp to maxPct, but never below minPct (keeps the sign)
    if (fwd > maxPct)  fwd = maxPct;
    if (fwd < -maxPct) fwd = -maxPct;
    if (fabs(fwd) < minPct) fwd = (error > 0) ? minPct : -minPct;

    // Steer back toward the starting heading (positive error = turn clockwise)
    double turn = kP_heading * wrap180(holdHeading - getHeading());

    drivePercent(fwd + turn, fwd - turn);
    wait(10, msec);
  }
  stopDrive();
}

// ============================================================
// HOW TO USE
// ============================================================
// void pre_auton(void) {
//   initOdometry();
// }
//
// void autonomous(void) {
//   setPose(0, 0, 0);
//   inchDrive(24);           // 24 in straight ahead
//   turnToHeading(90);       // face right
//   inchDrive(24);           // 24 in to the right
//   inchDrive(-12, 40);      // 12 in backward, capped at 40% speed
// }


/*---------------------------------------------------------------------------*/
/*                              Tunable Constants                            */
/*---------------------------------------------------------------------------*/

constexpr int    JOYSTICK_DEADBAND   = 5;      // pct, ignore small stick drift
constexpr float  TURN_SENSITIVITY_K  = 0.5f;   // scales turn contribution in arcade drive
constexpr float  SLOW_MODE_FACTOR    = 0.4f;   // speed multiplier when ButtonB held

constexpr double TURN_KP             = 2.0;    // proportional gain for turn_to_angle
constexpr double TURN_TOLERANCE_DEG  = 1.0;    // acceptable heading error to stop
constexpr double TURN_MIN_SPEED      = 6.0;    // minimum pct to overcome static friction
constexpr int    TURN_TIMEOUT_MS     = 3000;   // safety cutoff if turn never settles
constexpr int    TURN_SETTLE_LOOPS   = 3;      // consecutive in-tolerance loops required

constexpr double TIP_ANGLE_THRESHOLD_DEG = 12.0; // pitch angle considered "tipping"

/*---------------------------------------------------------------------------*/
/*                              Helper Functions                             */
/*---------------------------------------------------------------------------*/

void drive(int left_speed, int right_speed, int wt) {
  left_front.spin(forward, left_speed, pct);
  left_back.spin(reverse, left_speed, pct);

  right_front.spin(forward, right_speed, pct);
  right_back.spin(reverse, right_speed, pct);

  wait(wt, msec);
}

void drive_brake() {
  left_front.stop(brake);
  left_back.stop(brake);

  right_front.stop(brake);
  right_back.stop(brake);
}

void claw_hold() {
  claw.stop(hold);
  claw.setVelocity(100, percent);
}

void lift(directionType dir, int speed) {
  left_lift.spin(dir, speed, pct);
  right_lift.spin(dir, speed, pct);
}

void claws(directionType dir, int speed) {
  claw.spin(dir, speed, pct);
}

void turn_to_angle(double target_angle, int max_speed) {
  imu.setRotation(0, degrees);

  timer tmr;
  tmr.clear();

  int settled_loops = 0;

  while (true) {
    double current_angle = imu.rotation(degrees);
    double error = target_angle - current_angle;

    if (fabs(error) < TURN_TOLERANCE_DEG) {
      settled_loops++;
      if (settled_loops >= TURN_SETTLE_LOOPS) {
        break;
      }
    } else {
      settled_loops = 0;
    }

    if (tmr.time(msec) > TURN_TIMEOUT_MS) {
      break;
    }

    double speed = error * TURN_KP;

    if (speed > max_speed) speed = max_speed;
    if (speed < -max_speed) speed = -max_speed;

    if (speed > 0 && speed < TURN_MIN_SPEED) speed = TURN_MIN_SPEED;
    if (speed < 0 && speed > -TURN_MIN_SPEED) speed = -TURN_MIN_SPEED;

    drive(speed, -speed, 10);
  }
  drive_brake();
}

bool is_tipping() {
  double current_pitch = imu.pitch(degrees);
  return fabs(current_pitch) > TIP_ANGLE_THRESHOLD_DEG;
}

void handle_tipping() {
  if (is_tipping()) {
    drive_brake();
    left_lift.stop(hold);
    right_lift.stop(hold);
    Controller1.rumble(".");
  }
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
    Brain.Screen.print("Calibrating IMU");
    initOdometry();
    while (imu.isCalibrating()) {
        wait(20, msec);
    }
    Brain.Screen.clearScreen();
    Brain.Screen.print("IMU Calibrated");
    drive_brake();
    claw_hold();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  inchDrive(5, 100);
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    displayMotorData();

    int forward_raw = Controller1.Axis3.position(pct);
    int turn_raw = Controller1.Axis1.position(pct);

    int forward_val = 0;
    int turn_val = 0;

    if (abs(forward_raw) > JOYSTICK_DEADBAND) {
        forward_val = forward_raw;
    }

    if (abs(turn_raw) > JOYSTICK_DEADBAND) {
        turn_val = turn_raw;
    }

    float slow_factor = Controller1.ButtonB.pressing() ? SLOW_MODE_FACTOR : 1.0f;

    float left_speed = (forward_val + (TURN_SENSITIVITY_K * turn_val)) * slow_factor;
    float right_speed = (forward_val - (TURN_SENSITIVITY_K * turn_val)) * slow_factor;

    drive(left_speed, right_speed, 0);

    if (Controller1.ButtonL1.pressing()) {
      lift(fwd, 100);
    }
    else if (Controller1.ButtonL2.pressing()) {
      lift(reverse, 100);
    }
    else {
      left_lift.stop(hold);
      right_lift.stop(hold);
    }

    if (Controller1.ButtonR1.pressing()) {
      clawMode ++;
      if (clawMode % 2 == 0){
        clawPiston.set(true);
        wait(0.5, sec);
      }
      else{
        clawPiston.set(false);
        wait(0.5, sec);
      }
    } 
    
    if (Controller1.ButtonR2.pressing()) {
      flipMode ++;
      if (flipMode % 2 == 0){
        flipPiston.set(true);
        wait(0.5, sec);
      }
      else {
        flipPiston.set(false);
        wait(0.5, sec);
      }
    } 
  }
    //handle_tipping();

    // Brain.Screen.print("Right Speed: ", right_speed);
    // Brain.Screen.newLine();
    // Brain.Screen.print("Left Speed: ", left_speed);
    // Brain.Screen.newLine();


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  drive_brake();
  claw_hold();

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}