/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      10-12-26                                                  */
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

// Drive motors. NOTE: the back motors are driven with `reverse` in drive() and
// drivePercent(), which is why their reversal flags are the opposite of the
// front motors on the same side.
motor left_front = motor(PORT19, ratio6_1, true);
motor left_back = motor(PORT5, ratio6_1, false);

motor right_front = motor(PORT15, ratio6_1, false);
motor right_back = motor(PORT9, ratio6_1, true);

motor left_lift = motor(PORT20, ratio18_1, false);
motor right_lift = motor(PORT14, ratio18_1, true);

int clawMode = 0;
motor claw(PORT12, ratio18_1, true);

/*---------------------------------------------------------------------------*/
/*                          Motor monitor (Brain screen)                     */
/*---------------------------------------------------------------------------*/

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
  NAMED(right_lift)
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
    int y = 1 + i * ROW_HEIGHT;

    if (nm.m.installed()){
      motorDisplay(y, nm.m.current(amp), nm.m.temperature(celsius));
      Brain.Screen.printAt(300, YOFFSET + y, nm.name);
    }
    else {
      Brain.Screen.printAt(5, YOFFSET + y, true, "%s Problem", nm.name);
    }
  }
}

/*---------------------------------------------------------------------------*/
/*                              Tunable Constants                            */
/*---------------------------------------------------------------------------*/

constexpr int    JOYSTICK_DEADBAND   = 5;      // pct, ignore small stick drift
constexpr float  TURN_SENSITIVITY_K  = 0.7f;   // scales turn contribution in arcade drive
constexpr float  SLOW_MODE_FACTOR    = 0.4f;   // speed multiplier when ButtonB held

constexpr double TIP_ANGLE_THRESHOLD_DEG = 12.0; // pitch angle considered "tipping"

/*---------------------------------------------------------------------------*/
/*                       Basic drive helpers (driver + auton)                */
/*---------------------------------------------------------------------------*/

// Arcade/tank drive. Left/right speed in percent (-100 to 100).
// wt = how long to wait afterwards (ms).
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

void drive_coast() {
  left_front.stop(coast);
  left_back.stop(coast);

  right_front.stop(coast);
  right_back.stop(coast);
}

// Same as drive() but takes doubles and never waits (used by the auton PID loops)
void drivePercent(double leftPct, double rightPct) {
  left_front.spin(forward, leftPct, percent);
  left_back.spin(reverse, leftPct, percent);

  right_front.spin(forward, rightPct, percent);
  right_back.spin(reverse, rightPct, percent);
}

/*---------------------------------------------------------------------------*/
/*                  Odometry (drive encoders + IMU heading)                  */
/*                                                                           */
/*  Field coordinates in inches: +Y = forward at heading 0, +X = right.      */
/*  Heading is in degrees, clockwise positive (matches the IMU).             */
/*---------------------------------------------------------------------------*/

// ---------- CONFIG: change these to match your robot ----------
const double WHEEL_DIAMETER_IN = 3.25;      // 2.75, 3.25 or 4.0 for standard VEX omnis
const double GEAR_RATIO        = 4.0 / 3.0; // wheel turns per ONE motor turn
                                            // = (driving teeth) / (driven teeth)
const double IN_PER_DEG = (M_PI * WHEEL_DIAMETER_IN * GEAR_RATIO) / 360.0;

// ---------- Robot pose (shared between threads) ----------
vex::mutex poseMutex;
double poseX = 0.0;        // inches
double poseY = 0.0;        // inches
double poseHeading = 0.0;  // degrees, clockwise positive (from IMU)

// ---------- Helpers ----------
double wrap180(double deg) {
  while (deg > 180.0)   deg -= 360.0;
  while (deg <= -180.0) deg += 360.0;
  return deg;
}

// Each side's travel in motor degrees. The back motors are spun in `reverse`
// (see drive()), so their encoders count NEGATIVE when the robot drives
// forward. We subtract them so all four encoders add up instead of cancelling.
double leftSideDegrees() {
  return (left_front.position(degrees) - left_back.position(degrees)) / 2.0;
}

double rightSideDegrees() {
  return (right_front.position(degrees) - right_back.position(degrees)) / 2.0;
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

// ---------- Optional Brain screen pose readout ----------
// NOT started by default: it clears the whole screen and would fight with
// displayMotorData() in driver control. To use it for testing, add
// `task dispTask(displayLoop);` at the end of initOdometry().
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
// Robot must be sitting still while the IMU calibrates.
void initOdometry() {
  imu.calibrate();
  while (imu.isCalibrating()) wait(20, msec);

  left_front.resetPosition();  left_back.resetPosition();
  right_front.resetPosition(); right_back.resetPosition();
  imu.resetRotation();

  setPose(0, 0, 0);

  task odomTask(odometryLoop);
}

// Voltage-based drive (percent -100 to 100 mapped to +/-12 V).
// Not used by auton yet; kept from the earlier branch.
void driveVolts(double leftPct, double rightPct) {
  double leftVolts = leftPct * 12.0 / 100.0;
  double rightVolts = rightPct * 12.0 / 100.0;
  left_front.spin(forward, leftVolts, voltageUnits::volt);
  left_back.spin(reverse, leftVolts, voltageUnits::volt);
  right_front.spin(forward, rightVolts, voltageUnits::volt);
  right_back.spin(reverse, rightVolts, voltageUnits::volt);
}

/*---------------------------------------------------------------------------*/
/*                        Autonomous motion functions                        */
/*---------------------------------------------------------------------------*/

// Turn in place to an absolute field heading (degrees).
//   targetDeg  : field heading to face (0 = +Y, clockwise positive)
//   maxPct     : turn speed cap in percent (0-100)
//   timeoutMs  : give up after this long
void turnToHeading(double targetDeg, double maxPct = 70, double timeoutMs = 2000) {
  const double kP = 0.2;          // percent per degree of error
  const double kD = 6.0;          // damping: fights overshoot
  const double minPct = 10;       // enough power to actually move the drive
  const double tolerance = 1;   // degrees

  double start = Brain.timer(msec);
  double prevErr = wrap180(targetDeg - imu.rotation(degrees));
  int settled = 0;

  while (Brain.timer(msec) - start < timeoutMs) {
    double err = wrap180(targetDeg - imu.rotation(degrees));

    // Only finish after staying inside tolerance for 8 loops (80 ms)
    if (fabs(err) < tolerance) {
      if (++settled >= 8) break;
    } else {
      settled = 0;
    }

    double v = kP * err + kD * (err - prevErr);
    prevErr = err;

    if (v > maxPct)  v = maxPct;
    if (v < -maxPct) v = -maxPct;
    if (fabs(v) < minPct) v = (err > 0) ? minPct : -minPct;

    drivePercent(v, -v);     // positive error = turn clockwise
    wait(10, msec);
  }
  drive_brake();
}

// Drive straight a set distance in inches (negative = backward).
// Measures distance with the drive encoders and holds the heading the robot
// started with (via the IMU), so it drives in a straight line.
//   inches     : distance to travel, e.g. 24 or -12
//   maxPct     : speed cap in percent (0-100)
//   timeoutMs  : give up after this long
void inchDrive(double inches, double maxPct = 100, double timeoutMs = 4000) {
  const double kP = 0.5;          // percent per inch of distance error
  const double kD = 0.4;          // damping, reduces overshoot
  const double kP_heading = 1.25;  // percent per degree of heading error
  const double minPct = 12;       // keeps the robot moving through the last inch
  const double tolerance = 0.5;   // inches

  double startLeft  = leftSideDegrees();
  double startRight = rightSideDegrees();
  double holdHeading = imu.rotation(degrees);   // keep facing this way the whole drive

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
    double turn = kP_heading * wrap180(holdHeading - imu.rotation(degrees));

    drivePercent(fwd + turn, fwd - turn);
    wait(10, msec);
  }
  drive_brake();
}

void liftFor(double deg, int speed = 100, bool wait = false) {
  left_lift.setStopping(hold);    // keep the arm up after it arrives
  right_lift.setStopping(hold);
  left_lift.spinFor(forward, deg, degrees, speed, velocityUnits::pct, false);
  right_lift.spinFor(forward, deg, degrees, speed, velocityUnits::pct, wait);
}
/*---------------------------------------------------------------------------*/
/*                              Helper Functions                             */
/*---------------------------------------------------------------------------*/

void lift(directionType dir, int speed) {
  left_lift.spin(dir, speed, pct);
  right_lift.spin(dir, speed, pct);
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
/*  Runs once after the V5 powers on. Must return so that autonomous and     */
/*  usercontrol can start.                                                   */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  Brain.Screen.print("Calibrating IMU");
  initOdometry();   // calibrates the IMU (blocks until done), zeroes encoders, starts odometry
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("IMU Calibrated");
  drive_brake();
  claw.stop(hold);
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // Examples:
  //   inchDrive(24);        // 24 in forward
  //   turnToHeading(90);    // face right (absolute heading)
  //   inchDrive(-12, 40);   // 12 in backward, capped at 40% speed
  liftFor(150);
  inchDrive(14, 100);
  wait(1, sec);
  clawPiston.set(true);
  wait(0.7, sec);
  liftFor(1416);
  inchDrive(13, 100);
  flipPiston.set(true);
  turnToHeading(90);
  wait(200,msec);
  inchDrive(33,100);
  liftFor(-1166);
  flipPiston.set(false);
  wait(0.5, sec);
  flipPiston.set(true);
  liftFor(-600);
  wait(1.1 ,sec);
  clawPiston.set(false);
  wait(0.3 ,sec);
  inchDrive(-40, 100);

}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (1) {
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

    // if(Controller1.ButtonA.pressing()){
    //   SlowDown++;
    //   if(SlowDown % 2 = 1){
    //     left_front.setVelocity(60, pct);
    //     left_back.setVelocity(60, pct);
    //     right_back.setVelocity(60, pct);
    //     right_front.setVelocity(60, pct);
    //   }
    // }
    float left_speed = (forward_val + (TURN_SENSITIVITY_K * turn_val)) * slow_factor;
    float right_speed = (forward_val - (TURN_SENSITIVITY_K * turn_val)) * slow_factor;

    drive(left_speed, right_speed, 0);

    if (Controller1.ButtonL1.pressing()) {
      lift(forward, 100);
    }
    else if (Controller1.ButtonL2.pressing()) {
      lift(reverse, 100);
    }
    else {
      left_lift.stop(hold);
      right_lift.stop(hold);
    }

    if (Controller1.ButtonR1.pressing()) {
      clawMode++;
      if (clawMode % 2 == 0) {
        claw.spin(fwd, 100, pct);
        wait(0.5, sec);
        claw.stop();
      }
      else {
        claw.spin(reverse, 100, pct);
        wait(0.5, sec);
        claw.stop();
      }
    }

    // handle_tipping();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}