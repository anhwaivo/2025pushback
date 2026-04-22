#include "auton.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "monte_config.h" // For sensor offsets and constants
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// External declarations for globals from main.cpp
extern lemlib::Chassis chassis;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Motor intake;
extern pros::Motor conveyor;
extern pros::Motor outtake;
extern pros::Optical colorSensor;
extern int intakeMode;
extern int intakeSpeed; // 0-127, set in auton to control intake/outtake speed
                        // when using intakeMode
extern pros::adi::Pneumatics stage;
extern pros::adi::Pneumatics descoreRight;
extern pros::adi::Pneumatics counterLoader;
extern pros::adi::Pneumatics descoreLeft;
extern pros::adi::Pneumatics odoLift;
extern pros::adi::Pneumatics blockblock;
extern pros::Distance dNorth; // Front (moveToDistance)
extern pros::Distance dSouth; // Back (X reset south)
extern pros::Distance dWest;  // Port 10 (Y/X reset west)
extern pros::Distance dEast;  // Port 1 (Y reset east)
extern bool manualPneumaticOverride;
extern pros::Controller controller;

/**
 * Super simple helper: drive straight forward until the front distance sensor
 * reads <= targetDistanceMM.
 *
 * NOTE: No timeout, no fancy filtering. If the sensor never sees the target
 * distance, this will keep driving until something else stops the robot.
 */
bool moveToDistance(float targetDistanceMM, int maxSpeed = 60,
                    int /*timeoutMS*/ = 3000) {
  const int speed = std::min(std::max(maxSpeed, 20), 127); // clamp 20..127

  while (true) {
    float dist = dNorth.get(); // front distance sensor, mm

    // Valid reading and at or inside target -> stop
    if (dist > 0 && dist <= targetDistanceMM)
      break;

    chassis.arcade(speed, 0);
    pros::delay(20);
  }

  chassis.arcade(0, 0);
  return true;
}

/**
 * Drive the robot so that the front (north) distance sensor ends up at
 * a specific distance from the wall, using drive wheel encoders
 * (3.25\" diameter) for precise movement.
 *
 * This samples the distance once, computes how far to move, and sends a
 * single encoder-based move to the drive motors.
 *
 * @param targetDistanceMM Desired distance from wall in millimeters.
 * @param maxSpeed Motor command magnitude (0–127).
 */
void driveToNorthDistanceEncoder(float targetDistanceMM, int maxSpeed = 80) {
  // Read current distance in mm
  float dist_mm = dNorth.get();
  if (dist_mm <= 0 || dist_mm >= 9999) {
    // Bad reading – do nothing
    return;
  }

  // Positive error means we are too far from the wall (need to drive forward)
  float error_mm = dist_mm - targetDistanceMM;
  float error_in = error_mm / 25.4f;

  // Convert linear inches to wheel rotations (3.25\" diameter)
  constexpr float WHEEL_DIAM_IN = 3.25f;
  constexpr float PI_F = 3.14159265f;
  float wheel_circ_in = WHEEL_DIAM_IN * PI_F;
  float rotations = error_in / wheel_circ_in; // + = forward, - = backward
  float degrees = rotations * 360.0f; // motor position units are degrees

  int speed = std::min(std::max(maxSpeed, 20), 127);

  // Command both sides the same relative move so we go straight
  leftMotors.move_relative(degrees, speed);
  rightMotors.move_relative(degrees, speed);
}

/**
 * @brief Re-localize pose using a single distance sensor on port 10
 * (RW-Template approach).
 *
 * One sensor (port 10 = West-facing). Only resets Y: when sensor faces north or
 * south wall, Y is set from the distance; X and heading are always kept from
 * current pose.
 *
 * - Field walls at X/Y = ±70.
 * - Sensor offsets from monte_config.h (WEST_SENSOR_*).
 *
 * @return true if pose was updated, false if reading invalid or sensor not
 * facing north/south.
 */
bool resetposeskilly(int val) {
  const float dist_mm = dWest.get();
  if (dist_mm <= 0 || dist_mm >= 5000)
    return false;

  constexpr float MM_TO_IN = 1.0f / 25.4f;
  const float dist_in =
      (dist_mm * MM_TO_IN) / DISTANCE_SENSOR_CORRECTION_FACTOR;
  constexpr float FIELD_HALF_SIZE = 70.0f;

  lemlib::Pose current = chassis.getPose();
  const float robot_heading_deg = current.theta;
  const float heading_rad = robot_heading_deg * 3.14159265f / 180.0f;
  const float c = std::cos(heading_rad);
  const float s = std::sin(heading_rad);

  const float wx = WEST_SENSOR_X_OFFSET;
  const float wy = WEST_SENSOR_Y_OFFSET;
  const float off_y_field = c * wx + s * wy;

  // West sensor faces left: when robot heading 0 (+Y), sensor faces -X (270°)
  int sensor_heading_deg = (int)(robot_heading_deg - 90.0f);
  sensor_heading_deg = (sensor_heading_deg % 360 + 360) % 360;

  float robot_y = current.y;
  if (135 < sensor_heading_deg && sensor_heading_deg <= 225) {
    // Sensor facing south wall (Y = -70)
    robot_y = -FIELD_HALF_SIZE + (dist_in - off_y_field);
  } else if (sensor_heading_deg <= 45 || sensor_heading_deg > 315) {
    // Sensor facing north wall (Y = +70)
    robot_y = FIELD_HALF_SIZE - (dist_in + off_y_field);
  } else {
    // Facing east/west: can't reset Y, don't update pose
    return false;
  }

  chassis.setPose(current.x, val * std::abs(robot_y), robot_heading_deg);
  return true;
}

/**
 * @brief Clone of resetposeskilly using distance sensor port 1 (dEast). Only
 * resets Y.
 *
 * East sensor faces right: sensor_heading = robot_heading + 90.
 * Y is set when sensor faces north or south wall; X and heading kept from
 * current pose.
 *
 * East sensor on the right often ha2s beam pointing the opposite way vs west;
 * we use swapped north/south formulas so "sensor facing north" = measure north
 * wall. If Y is still wrong, try un-swapping (use west-style formulas in both
 * branches).
 *
 * @return true if pose was updated, false if reading invalid or sensor not
 * facing north/south.
 */
bool resetposeskillyEast(int val) {
  const float dist_mm = dEast.get();
  if (dist_mm <= 0 || dist_mm >= 5000)
    return false;

  constexpr float MM_TO_IN = 1.0f / 25.4f;
  const float dist_in =
      (dist_mm * MM_TO_IN) / DISTANCE_SENSOR_CORRECTION_FACTOR;
  constexpr float FIELD_HALF_SIZE = 70.0f;

  lemlib::Pose current = chassis.getPose();
  const float robot_heading_deg = current.theta;
  const float heading_rad = robot_heading_deg * 3.14159265f / 180.0f;
  const float c = std::cos(heading_rad);
  const float s = std::sin(heading_rad);

  const float ex = EAST_SENSOR_X_OFFSET;
  const float ey = EAST_SENSOR_Y_OFFSET;
  // East sensor on right: field Y of sensor = robot_y + (c*ex + s*ey). Same as
  // west.
  const float off_y_field = c * ex + s * ey;

  // East sensor faces right: sensor_heading = robot_heading + 90
  int sensor_heading_deg = (int)(robot_heading_deg + 90.0f);
  sensor_heading_deg = (sensor_heading_deg % 360 + 360) % 360;

  float robot_y = current.y;
  // East sensor often mounted with beam opposite to west: when opening "faces
  // north" we measure south wall.
  const bool east_beam_opposite =
      true; // set false if east Y resets are mirrored/wrong
  if (135 < sensor_heading_deg && sensor_heading_deg <= 225) {
    if (east_beam_opposite)
      robot_y = FIELD_HALF_SIZE -
                (dist_in + off_y_field); // actually measuring north wall
    else
      robot_y = -FIELD_HALF_SIZE + (dist_in - off_y_field);
  } else if (sensor_heading_deg <= 45 || sensor_heading_deg > 315) {
    if (east_beam_opposite)
      robot_y = -FIELD_HALF_SIZE +
                (dist_in - off_y_field); // actually measuring south wall
    else
      robot_y = FIELD_HALF_SIZE - (dist_in + off_y_field);
  } else {
    return false;
  }

  chassis.setPose(current.x, val * std::abs(robot_y), robot_heading_deg);
  return true;
}

/**
 * @brief Reset X using dNorth (front) sensor. Only resets X when sensor faces
 * east/west wall.
 *
 * North sensor faces forward: sensor_heading = robot_heading.
 * Same pattern as resetposeskillx(): Y and heading kept from current pose.
 */
bool resetposeskillxNorth(int val) {
  const float dist_mm = dNorth.get();
  if (dist_mm <= 0 || dist_mm >= 5000)
    return false;

  constexpr float MM_TO_IN = 1.0f / 25.4f;
  const float dist_in =
      (dist_mm * MM_TO_IN) / DISTANCE_SENSOR_CORRECTION_FACTOR;
  constexpr float FIELD_HALF_SIZE = 70.0f;

  lemlib::Pose current = chassis.getPose();
  const float robot_heading_deg = current.theta;
  const float heading_rad = robot_heading_deg * 3.14159265f / 180.0f;
  const float c = std::cos(heading_rad);
  const float s = std::sin(heading_rad);

  const float nx = NORTH_SENSOR_X_OFFSET;
  const float ny = NORTH_SENSOR_Y_OFFSET;
  const float off_x_field = -s * nx + c * ny;

  // North sensor faces forward: sensor_heading = robot_heading
  int sensor_heading_deg = (int)(robot_heading_deg);
  sensor_heading_deg = (sensor_heading_deg % 360 + 360) % 360;

  float robot_x = current.x;
  if (45 < sensor_heading_deg && sensor_heading_deg <= 135) {
    robot_x = FIELD_HALF_SIZE - (dist_in + off_x_field);
  } else if (225 < sensor_heading_deg && sensor_heading_deg <= 315) {
    robot_x = -FIELD_HALF_SIZE + (dist_in - off_x_field);
  } else {
    return false;
  }

  chassis.setPose(val * std::abs(robot_x), current.y, robot_heading_deg);
  return true;
}

/**
 * @brief Reset X using south (back) sensor. Only resets X when sensor faces
 * east/west wall.
 *
 * South sensor faces backward: sensor_heading = robot_heading + 180.
 * X and heading kept from current pose; Y unchanged.
 *
 * @return true if pose was updated, false if reading invalid or sensor not
 * facing east/west.
 */
bool resetposeskillxSouth(int val) {
  const float dist_mm = dSouth.get();
  if (dist_mm <= 0 || dist_mm >= 5000)
    return false;

  constexpr float MM_TO_IN = 1.0f / 25.4f;
  const float dist_in =
      (dist_mm * MM_TO_IN) / DISTANCE_SENSOR_CORRECTION_FACTOR;
  constexpr float FIELD_HALF_SIZE = 70.0f;

  lemlib::Pose current = chassis.getPose();
  const float robot_heading_deg = current.theta;
  const float heading_rad = robot_heading_deg * 3.14159265f / 180.0f;
  const float c = std::cos(heading_rad);
  const float s = std::sin(heading_rad);

  const float sx = SOUTH_SENSOR_X_OFFSET;
  const float sy = SOUTH_SENSOR_Y_OFFSET;
  const float off_x_field = -s * sx + c * sy;

  // South sensor faces backward: when robot heading 0 (+Y), sensor faces -Y
  // (180°)
  int sensor_heading_deg = (int)(robot_heading_deg + 180.0f);
  sensor_heading_deg = (sensor_heading_deg % 360 + 360) % 360;

  float robot_x = current.x;
  if (45 < sensor_heading_deg && sensor_heading_deg <= 135) {
    robot_x = FIELD_HALF_SIZE - (dist_in + off_x_field);
  } else if (225 < sensor_heading_deg && sensor_heading_deg <= 315) {
    robot_x = -FIELD_HALF_SIZE + (dist_in - off_x_field);
  } else {
    return false;
  }

  chassis.setPose(val * std::abs(robot_x), current.y, robot_heading_deg);
  return true;
}

// Function to set stage (declared in main.cpp)
// extern void // setStage(int stageMode);

// ASSET declarations
ASSET(pathdescore1_txt);
ASSET(pathdescore2_txt);
ASSET(pathdescore3_txt);
ASSET(pathdescoreright_txt);
ASSET(pathdescorecbd_txt);

void awp() {
  // descoreLeft.set_value(true);
  // // setStage(2);
  intakeSpeed = 127;
  intakeMode = 1;
  chassis.setPose(48.5, 17.5, 0);
  chassis.moveToPoint(48.5, 47.5, 1300, {}, false);
  counterLoader.set_value(true);
  chassis.turnToHeading(90, 600, {}, true);
  resetposeskillxNorth(1);
  resetposeskilly(1);
  pros::delay(50);
  chassis.moveToPoint(56.5, 47.5, 900, {}, false);
  // pros::delay(400);
  chassis.moveToPoint(23, 47.5, 1000, {.forwards = false, .minSpeed = 100},
                      false);
  // pros::delay(150);
  // intakeMode = 4;
  // blockblock.set_value(false);
  // pros::delay(150);
  intakeMode = 2;
  pros::delay(1200);
  intakeMode = 1;
  chassis.turnToHeading(90, 900, {});
  resetposeskillxNorth(1);
  resetposeskilly(1);
  pros::delay(50);
  counterLoader.set_value(false);

  chassis.turnToHeading(220, 800, {}, false);
  chassis.moveToPose(23, -24, 180, 1900, {.minSpeed = 80}, true);
  pros::delay(1300);
  counterLoader.set_value(true);
  chassis.turnToHeading(135, 900, {}, true);
  // intakeMode = 4;
  // pros::delay(150);
  // intakeMode = 0;
  // chassis.moveToPose(5, -6.5, 135, 1500, {.forwards = false, .minSpeed = 45},
  // false);
  chassis.moveToPoint(6, -7.5, 800, {.forwards = false}, false);
  // pros::delay(550);
  intakeMode = 3;
  pros::delay(800);
  // counterLoader.set_value(true);

  // ang loader
  intakeMode = 1;
  chassis.moveToPoint(42, -50.5, 1300, {}, false);
  counterLoader.set_value(true);
  chassis.turnToHeading(90, 600, {}, false);
  resetposeskillyEast(-1);
  // resetposeskillxNorth(1);
  pros::delay(50);

  // chassis.moveToPoint(61, -47.25, 1350, {}, false);
  // pros::delay(700);
  // chassis.moveToPoint(23, -51.5, 1200, {.forwards = false, .minSpeed = 90});
  chassis.moveToPoint(21, -47.25, 1000, {.forwards = false, .minSpeed = 90},
                      false);
  // goStraight(100, 900);
  // intakeMode = 4;
  // pros::delay(150);
  intakeMode = 2;
  chassis.turnToHeading(90, 300, {});
}

// ========= LEFT DESCORERE 7 BLOCK =========
void leftdescore7bloc() {
  intakeSpeed = 127;
  // manualPneumaticOverride = true;
  // descoreLeft.set_value(true);
  // // setStage(2);
  intakeMode = 1;
  // intake.move(127);
  // conveyor.move(127);
  // chassis.setPose(48.65, -9.85, 249.5);
  chassis.setPose(45.4, -14.35, 270.0);
  chassis.moveToPoint(22.5, -19.55, 800, {}, true);
  // chassis.moveToPoint(22.5, -22.25, 800, {}, true);
  pros::delay(450);
  counterLoader.set_value(true);
  // chassis.turnToHeading(130, 700, {}, false);
  chassis.turnToPoint(43, -48.5, 500, {}, true);
  chassis.moveToPoint(43, -48.5, 1150, {}, true);

  // chassis.turnToPoint(59.75, -49, 500, {}, true);

  chassis.turnToHeading(90, 600, {}, false);
  // counterLoader.set_value(true);
  resetposeskillyEast(-1);
  pros::delay(50);
  // chassis.turnToPoint(62, chassis.getPose().y, 650, {}, true);

  // intakeMode = 4;
  // pros::delay(100);
  // intakeMode = 1;
  chassis.moveToPoint(59.75, -47.5, 720, {.maxSpeed = 60}, false);
  // chassis.moveToPose(59.75, -47.5, 90, 950, {.maxSpeed = 60}, false);
  // chassis.turnToHeading(90, 650);
  resetposeskillyEast(-1);
  pros::delay(50);
  chassis.cancelMotion();

  // pros::delay(400);

  chassis.turnToPoint(22.5, -47.5, 500, {.forwards = false}, true);
  // chassis.turnToHeading(90, 500, {}, true);
  // intakeMode = 0;
  chassis.moveToPoint(22.5, -47.25, 1150, {.forwards = false, .maxSpeed = 67});
  // chassis.moveToPose(22.5, -47.25, 90, 1250, {.forwards = false, .maxSpeed =
  // 80}, true); pros::delay(700); chassis.turnToHeading(90, 350, {});

  // intakeMode = 1;
  chassis.waitUntilDone();
  // chassis.turnToPoint(22.5, -47.35, 1000, {.forwards = false}, true);
  // pros::delay(700)
  // setStage(4);
  // intakeMode = 4;
  // pros::delay(150);
  intakeMode = 2;
  // resetposeskillxNorth(1);
  // resetposeskillyEast(-1);
  pros::delay(1300);
  // intakeMode = 0;
  chassis.turnToHeading(90, 450, {});
  resetposeskillyEast(-1);

  // chassis.setPose(29.5, -47.25, chassis.getPose().theta);
  counterLoader.set_value(false);

  // chassis.moveToPoint(37, -47.35, 800, {}, true);
  descoreLeft.set_value(false);
  descoreRight.set_value(false);

  // descore phai
  // chassis.turnToHeading(140, 600, {}, true);
  // chassis.moveToPoint(31, -36.95, 850, {.forwards = false}, true);
  chassis.moveToPose(35.55, -38.9, 90, 1200, {}, true);
  // chassis.moveToPose((chassis.getPose().x)+1, -36.5, 90, 1100, {}, true);

  chassis.turnToHeading(90, 400, {}, true);
  intakeMode = 0;
  chassis.moveToPoint(13.2, -39, 1500, {.forwards = false, .minSpeed = 80},
                      true);

  chassis.turnToHeading(90, 10000, {}, false);

  // chassis.turnToHeading(30, 400, {}, true);
  // chassis.moveToPoint(31, -57.75, 1100, {.forwards = false}, true);
  // chassis.turnToHeading(90, 500, {}, true);
  // chassis.moveToPoint(15, -57.7, 2000, {.forwards = false, .minSpeed = 80},
  // true); chassis.turnToHeading(90, 10000, {}, false);

  // chassis.follow(pathdescore2_txt, 12, 3800, false, false);
  // pros::delay(600);

  // pros::delay(2000);
  // chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y, 6000,
  // {.minSpeed = 120}, false); chassis.turnToHeading(80, 1000, {.minSpeed =
  // 120}, false); chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y,
  // 10000, {.minSpeed = 120}, false);
}

void leftfastdescore() {
  // setStage(3);
  intakeMode = 1;
  chassis.setPose(49.2, -10, 247.334);
  chassis.moveToPoint(25.2, -20, 700, {}, true);
  pros::delay(500);
  counterLoader.set_value(true);
  chassis.turnToHeading(135, 650, {}, false);
  chassis.moveToPoint(41, -45.5, 800, {}, false);
  chassis.turnToHeading(90, 250, {}, true);
  chassis.moveToPoint(23, -47.75, 2300, {.forwards = false, .minSpeed = 90});
  pros::delay(600);
  pros::delay(150);
  // setStage(2);
  intakeMode = 1;
  pros::delay(1500);

  chassis.setPose(29.5, -47.15, 90);

  chassis.moveToPoint(40, -47.15, 800, {}, false);
  chassis.turnToHeading(35, 500, {}, false);
  chassis.follow(pathdescore2_txt, 9, 3000, false, true);
  pros::delay(800);
  descoreLeft.set_value(false);

  pros::delay(1000);
  chassis.moveToPoint((chassis.getPose().x), chassis.getPose().y, 6000, {},
                      false);
  chassis.turnToHeading(60, 500, {.minSpeed = 120}, false);
}

// ========= LEFT CENTER DESCORERE =========
void leftcenterdescore() {
  descoreLeft.set_value(true);
  // setStage(2);
  intakeMode = 1;
  chassis.setPose(49.2, -10, 247.334);
  chassis.moveToPoint(25.2, -20, 700, {}, true);
  pros::delay(500);
  counterLoader.set_value(true);
  pros::delay(600);
  counterLoader.set_value(false);

  // chassis.moveToPose(7, -43, 206, 1700, {}, false);
  // chassis.moveToPoint(25.2, -20, 1500, {.forwards = false}, false);
  pros::delay(300);
  intakeMode = 0;
  pros::delay(200);
  // setStage(0);
  // chassis.moveToPose(2, -13, 135, 2800, {.forwards = false}, false);
  // chassis.moveToPose(0, -7, 135, 2000, {.forwards = false}, false);
  pros::delay(200);

  // chassis.moveToPose(5, -7, 135, 2000, {.forwards = false}, false);
  chassis.moveToPoint(25.75, -25.75, 1000, {.forwards = false}, false);
  chassis.turnToHeading(135, 900, {}, false);
  chassis.moveToPoint(9, -9, 1000, {.forwards = false}, false);
  // // setStage(0);
  intakeMode = 2;
  pros::delay(200);
  intakeMode = 1;
  pros::delay(900);
  // setStage(2);

  chassis.turnToHeading(145, 650, {}, false);

  pros::Task intakeSequenceTask([&]() {
    counterLoader.set_value(true);
    intakeMode = 2;
    pros::delay(200);
    // // setStage(3);
    pros::delay(250);
    intakeMode = 1;
  });

  chassis.moveToPoint(45, -47.5, 1100, {}, false);

  chassis.turnToHeading(90, 450, {}, true);
  chassis.moveToPoint(64, -47.5, 1100, {}, false);
  pros::delay(650);
  intakeMode = 0;
  // chassis.moveToPoint(23, -47.3, 2300, {.forwards = false, .minSpeed = 90});
  chassis.moveToPoint(23, -47.5, 2300, {.forwards = false, .minSpeed = 90});
  pros::delay(600);
  intakeMode = 2;
  pros::delay(200);
  // setStage(1);
  intakeMode = 1;
  pros::delay(2000);
  intakeMode = 0;

  chassis.setPose(29.25, -48, 90);
  counterLoader.set_value(false);
  // descoreLeft.set_value(true);

  chassis.moveToPoint(40, -48, 600, {}, false);
  chassis.turnToHeading(35, 300, {}, false);
  chassis.follow(pathdescore2_txt, 9, 3500, false, true);
  pros::delay(600);
  descoreLeft.set_value(false);

  pros::delay(2000);
  chassis.turnToHeading(80, 1000, {.minSpeed = 120}, false);
  chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y, 10000,
                      {.minSpeed = 120}, false);
}

void leftlongcenter2() {
  intakeSpeed = 127;
  intakeMode = 1;
  chassis.setPose(48.5, -17.5, 180);
  chassis.moveToPoint(48.5, -48.75, 1300, {}, true);

  counterLoader.set_value(true);
  chassis.turnToHeading(90, 700, {}, false);
  // chassis.turnToPoint(59.5, -47.5, 800, {}, false);
  //
  // resetposeskillxNorth(1);
  resetposeskillyEast(-1);
  pros::delay(50);

  chassis.moveToPoint(59.5, -47.25, 500, {}, false);
  // chassis.turnToHeading(90, 400, {}, false);
  // resetposeskillxNorth(1);
  resetposeskillyEast(-1);
  pros::delay(50);
  // pros::delay(400);

  // chassis.cancelMotion();

  chassis.turnToPoint(22.5, -47.5, 450, {.forwards = false}, true);
  chassis.moveToPoint(22.5, -47.5, 1200, {.forwards = false, .maxSpeed = 67});

  chassis.waitUntilDone();
  intakeMode = 2;
  // pros::delay(800);
  counterLoader.set_value(false);
  chassis.turnToHeading(90, 800, {}, false);
  resetposeskillyEast(-1);
  // resetposeskillxNorth(1);
  pros::delay(100);

  chassis.moveToPose(28.25, -25, 0, 1800, {.maxSpeed = 75});
  pros::delay(1200);
  counterLoader.set_value(true);
  intakeMode = 1;
  chassis.waitUntilDone();

  chassis.turnToPoint(8, -8, 1000, {.forwards = false}, false);

  // chassis.turnToHeading(135, 900, {}, false);
  counterLoader.set_value(false);
  // chassis.moveToPoint(13.25, -13.25, 1000, {.forwards = false});
  chassis.moveToPose(16, -11, 135, 1100, {.forwards = false});
  chassis.waitUntilDone();

  // intakeSpeed = 75;
  // chassis.cancelMotion();
  intakeMode = 3;
  pros::delay(1000);
  chassis.cancelMotion();
  // intakeSpeed = 127;
  // intakeMode = 1;

  // chassis.moveToPose(40, -36.2, 90, 1900);

  chassis.moveToPoint(40, -39, 1400, {}, true);
  chassis.turnToHeading(90, 600, {}, true);
  chassis.moveToPoint(19, -39.5, 1670, {.forwards = false}, true);
  descoreLeft.set_value(false);
  descoreRight.set_value(false);
  intakeMode = 4;
  chassis.turnToHeading(90, 9000, {}, false);
  intakeMode = 0;

  // intakeMode = 1;
  // chassis.moveToPoint(23, -25, 1300, {}, true);
  // pros::delay(900);true
  // counterLoader.set_value(true);
  // chassis.turnToHeading(135, 1000, {}, true);
  // chassis.moveToPose(5, -6.5, 135, 1400, {.forwards = false}, false);
  // // pros::delay(300);
  // intakeMode = 3;
  // pros::delay(1250);
  // counterLoader.set_value(false);
  // // stage.set_value(true);
  // intakeMode = 0;
  // descoreLeft.set_value(false);
  // chassis.swingToHeading(180, DriveSide::LEFT, 500);
  // chassis.moveToPoint(8.5, -38, 1500, {}, true);

  // chassis.turnToHeading(315, 1000, {}, false);

  // chassis.follow(pathdescorecen1_txt, 10, 3000);
}

// ========= RIGHT DESCORERE 7 BLOCK =========
void rightdescore7bloc() {
  intakeSpeed = 127;
  // descoreLeft.set_value(true);
  // // setStage(3);
  intakeMode = 1;
  chassis.setPose(45.4, 14.35, 270.0);
  chassis.moveToPoint(25.2, 19.55, 750, {}, true);
  pros::delay(450);
  counterLoader.set_value(true);
  chassis.turnToPoint(45, 47.75, 500, {}, true);
  chassis.moveToPoint(45, 47.75, 1200, {}, true);

  // chassis.turnToPoint(59.75, 47.75, 500, {}, true);
  chassis.turnToHeading(90, 500, {}, false);
  resetposeskilly(1);
  pros::delay(50);
  // intakeMode = 4;
  // pros::delay(150);
  // intakeMode = 1;
  chassis.moveToPoint(59.9, 46.75, 650, {.maxSpeed = 67}, false);
  // chassis.turnToHeading(90, 400, {}, false);
  // resetposeskillxNorth(1);
  resetposeskilly(1);
  pros::delay(50);
  // chassis.cancelMotion();

  // intakeMode = 0;0
  chassis.turnToPoint(20, 47, 450, {.forwards = false}, true);
  chassis.moveToPoint(20, 47, 1150, {.forwards = false, .maxSpeed = 85});
  // chassis.moveToPose(22.5, 47.25, 90, 1000, {.forwards = false}, false);
  // pros::delay(600);
  // chassis.waitUntilDone();

  // intakeMode = 1;
  chassis.waitUntilDone();
  // // setStage(2);
  // intakeMode = 4;
  // pros::delay(150);
  intakeMode = 2;
  pros::delay(1200);
  chassis.turnToHeading(90, 500, {});
  // resetposeskillxNorth(1);
  resetposeskilly(1);
  pros::delay(50);
  // intakeMode = 0;

  // chassis.setPose(29.25, 48, chassis.getPose().theta);
  counterLoader.set_value(false);
  descoreLeft.set_value(false);
  descoreRight.set_value(false);

  // chassis.moveToPose(39.5, 40, 90, 1400, {}, true);
  chassis.moveToPoint(30, 37.2, 1000, {}, true);

  // chassis.moveToPoint(40, 47.25, 700, {}, true);
  // chassis.turnToHeading(40, 300, {}, true);
  // chassis.moveToPoint(30, 36.5, 800, {.forwards = false}, true);
  // chassis.turnToHeading(100, 400, {}, true);
  chassis.turnToHeading(90, 700, {}, true);
  intakeMode = 0;
  // descoreLeft.set_value(false);
  chassis.moveToPoint(10, 37.25, 1670, {.forwards = false}, true);
  // chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y, 6000,
  // {.minSpeed = 120}, false);
  chassis.turnToHeading(90, 9000, {}, false);
}

void rightcenterdescore() {
  descoreLeft.set_value(true);
  descoreRight.set_value(true);
  intakeSpeed = 127;
  intakeMode = 1;
  chassis.setPose(48.5, 17.5, 0);

  chassis.moveToPoint(48.5, 48.75, 1300);
  // counterLoader.set_value(true);
  // chassis.turnToPoint(59.75, 47.25, 500, {}, true);
  chassis.turnToHeading(90, 650);
  counterLoader.set_value(true);
  chassis.moveToPoint(59.5, 50.25, 600, {.maxSpeed = 70}, false);

  resetposeskilly(1);
  pros::delay(50);
  // chassis.cancelMotion();

  chassis.turnToPoint(20, 47.15, 450, {.forwards = false}, true);
  chassis.moveToPoint(20, 47.15, 1150, {.forwards = false, .maxSpeed = 105});

  chassis.waitUntilDone();
  intakeMode = 2;
  pros::delay(800);
  counterLoader.set_value(false);
  chassis.turnToHeading(90, 650, {});
  resetposeskilly(1);
  pros::delay(50);

  // chassis.moveToPose(22, 22, 225, 1200);
  chassis.moveToPose(4.5, 3.75, 220, 2100, {.maxSpeed = 68.5});
  intakeMode = 1;
  pros::delay(1000);
  counterLoader.set_value(true);
  pros::delay(300);
  counterLoader.set_value(false);
  chassis.waitUntilDone();
  chassis.turnToPoint(4.5, 3.5, 650);
  // pros::delay(650);
  // intakeSpeed = 75;
  // chassis.cancelMotion();
  intakeMode = 4;
  pros::delay(1300);
  chassis.cancelMotion();
  // intakeSpeed = 127;
  // intakeMode = 1;

  // chassis.moveToPose(17, 37, 90, 1900);
  
  descoreLeft.set_value(false);
  descoreRight.set_value(false);
  chassis.moveToPose(22, 37.27, 90, 1200, {.forwards = false}, true);

  // chassis.moveToPoint(24.5, 41, 1400, {.forwards = false}, true);
  // pros::delay(300);
 
  chassis.turnToHeading(90, 400, {}, true);
  intakeMode = 0;
  chassis.moveToPoint(9.75, 37.27, 1600, {.forwards = false}, true);
  chassis.turnToHeading(90, 9000, {}, false);
}

void rightdescore9bloc() {
  intakeSpeed = 127;
  intakeMode = 1;
  chassis.setPose(48.5, 17.5, 0);

  chassis.moveToPoint(25.364, 20, 700, {}, true);
  pros::delay(500);
  counterLoader.set_value(true);
  pros::delay(300);
  counterLoader.set_value(false);
  chassis.moveToPose(7, 43, 334, 1800, {}, false);
  chassis.moveToPoint(21, 21, 1200, {.forwards = false}, true);

  chassis.turnToHeading(35, 650, {}, false);
  chassis.moveToPoint(41.5, 45.5, 800, {}, false);
  chassis.turnToHeading(90, 250, {}, true);
  chassis.moveToPoint(23, 47.75, 2800, {.forwards = false, .minSpeed = 90});
  pros::delay(800);
  // setStage(2);
  pros::delay(1800);
  counterLoader.set_value(true);
  intakeMode = 2;
  pros::delay(300);
  // setStage(3);
  pros::delay(500);
  intakeMode = 1;
  chassis.moveToPoint(57, 48, 1600, {}, false);
  pros::delay(200);
  chassis.moveToPoint(23, 47.75, 2300, {.forwards = false, .minSpeed = 90});
  // setStage(2);
  intakeMode = 1;
  pros::delay(2500);
  intakeMode = 0;

  chassis.setPose(29.25, 47.15, 90);
  chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y, 6000, {},
                      false);
}

void rightlower() {
  descoreLeft.set_value(true);
  // // setStage(2);
  intakeMode = 1;
  chassis.setPose(48, 15.5, 0);
  chassis.moveToPoint(48, 43.5, 800, {}, false);
  counterLoader.set_value(true);

  chassis.turnToHeading(90, 600, {}, true);
  chassis.moveToPoint(56.5, 47.5, 900, {}, false);
  // pros::delay(400);
  chassis.moveToPoint(23, 47.5, 1000, {.forwards = false, .minSpeed = 100},
                      false);
  // pros::delay(150);
  // intakeMode = 4;
  // blockblock.set_value(false);
  // pros::delay(150);
  intakeMode = 2;
  pros::delay(1200);
  intakeMode = 1;
  chassis.turnToHeading(90, 900, {});
  counterLoader.set_value(false);

  chassis.turnToHeading(235, 800, {}, false);

  chassis.moveToPoint(23, 25, 1300, {}, true);
  pros::delay(800);
  // counterLoader.set_value(true);
  pros::delay(300);
  // chassis.turnToHeading(135, 1000, {}, true);
  chassis.moveToPose(5, 6.5, 45, 1400, {}, false);
  // pros::delay(300);

  descoreRight.set_value(true);
  intakeMode = 4;
  pros::delay(1050);

  // stage.set_value(true);
  // intakeMode = 0;
  descoreLeft.set_value(false);
  descoreRight.set_value(false);
  chassis.swingToHeading(90, DriveSide::RIGHT, 500);
  chassis.moveToPoint(17, 38, 1500, {}, true);
  descoreLeft.set_value(false);

  chassis.turnToHeading(70, 1000);
  // chassis.swingToHeading(180, DriveSide::LEFT, 500);
}

// Helper functions
void goStraight(int speed, int duration) {
  unsigned long startTime = pros::millis();
  while (pros::millis() - startTime < duration) {
    chassis.tank(speed, speed, true);
    pros::delay(10);
  }
  chassis.tank(0, 0, true);
}

void reverse(int speed, int duration) {
  unsigned long startTime = pros::millis();
  while (pros::millis() - startTime < duration) {
    chassis.tank(speed, speed, true);
    pros::delay(10);
  }
  chassis.tank(0, 0, true);
}

void wiggle(int speed, int duration) {
  unsigned long startTime = pros::millis();
  bool wiggleLeft = true;
  while (pros::millis() - startTime < duration) {
    if (wiggleLeft) {
      chassis.tank(-speed, speed, true);
    } else {
      chassis.tank(speed, -speed, true);
    }
    wiggleLeft = !wiggleLeft;
    pros::delay(150);
  }
  chassis.tank(0, 0, true);
}

void chutrinhtrai() {
  descoreLeft.set_value(true);
  intakeMode = 1;
  chassis.setPose(-48.75, 10, 64.125);
  chassis.moveToPoint(-22.715, 22.5, 1000, {}, true);
  pros::delay(400);
  counterLoader.set_value(true);
  chassis.turnToHeading(315, 800, {}, true);

  // =============== centerrrrrr ===============
  chassis.moveToPose(-4.5, 4.5, 315, 1800, {.forwards = false}, true);
  pros::delay(1000);
  intakeMode = 3;
  counterLoader.set_value(false);

  // =============== loaderr 1 ===============
  chassis.moveToPoint(-47.25, 47.25, 2000, {}, true);
  chassis.turnToHeading(270, 1000, {}, true);
  chassis.moveToPoint(-53, 47.25, 1000, {}, true);
  intakeMode = 1;
  counterLoader.set_value(true);
  resetposeskillyEast(1);
  pros::delay(50);
  resetposeskillxNorth(-1);
  pros::delay(50);
  chassis.moveToPoint(-57.75, 47.25, 1350, {.maxSpeed = 100}, false);

  // =============== lui ra ===============
  chassis.moveToPoint(-47.25, 47.25, 1000, {.forwards = false}, true);
  chassis.turnToHeading(50, 1300, {}, true);
  intakeMode = 0;
  counterLoader.set_value(false);

  // =============== qua longg ===============
  chassis.moveToPoint(-33, 59.25, 900, {}, true);
  chassis.turnToHeading(90, 1000, {}, true);
  chassis.moveToPoint(25, 59.5, 2250, {}, true);
  chassis.turnToHeading(90, 300, {}, false);
  resetposeskilly(1);
  pros::delay(50);
  resetposeskillxNorth(1);
  chassis.turnToHeading(130, 1000, {}, true);
  chassis.moveToPoint(39, 47.25, 1500, {}, true);
  chassis.turnToHeading(90, 1000, {}, true);

  // =============== out 7 bloc 2 ===============
  chassis.moveToPoint(28, 47.25, 1000, {.forwards = false}, true);
  pros::delay(100);
  intakeMode = 4;
  pros::delay(150);
  intakeMode = 2;
  chassis.turnToHeading(90, 1900, {}, false);
  resetposeskillxNorth(1);
  pros::delay(50);
  resetposeskilly(1);
  pros::delay(50);

  // =============== loaderr 2 ===============
  chassis.moveToPoint(53, 47.25, 1000, {}, true);
  intakeMode = 1;
  counterLoader.set_value(true);
  resetposeskilly(1);
  pros::delay(50);
  resetposeskillxNorth(1);
  pros::delay(50);
  chassis.moveToPoint(57.75, 47.25, 1350, {.maxSpeed = 100}, false);

  // =============== out 7 bloc 3 ===============
  chassis.moveToPoint(28, 47.25, 1200, {.forwards = false}, true);
  pros::delay(200);
  counterLoader.set_value(false);
  pros::delay(100);
  intakeMode = 4;
  pros::delay(150);
  intakeMode = 2;
  chassis.turnToHeading(90, 1900, {}, false);
  // goStraight(42, 350);
  // goStraight(-60, 300);
}

void quapark2() {
  resetposeskillxNorth(1);
  pros::delay(50);
  resetposeskilly(1);
  pros::delay(50);
  chassis.swingToHeading(125.5, DriveSide::LEFT, 1000, {}, true);
  chassis.moveToPoint(63, 23, 1850, {}, true);
  odoLift.set_value(true);
  chassis.turnToHeading(165, 550, {}, false);
  intakeMode = 1;
  goStraight(79, 850);
  counterLoader.set_value(true);
  goStraight(74, 700);
  counterLoader.set_value(false);
  goStraight(75, 1050);
  odoLift.set_value(false);
  chassis.turnToHeading(90, 1000, {}, false);
  resetposeskillyEast(-1);
  pros::delay(50);
  resetposeskillxNorth(1);
  pros::delay(50);
  intakeSpeed = 90;
  intakeMode = 1;
  chassis.moveToPoint(24, -24, 1800, {.forwards = false}, true);
  chassis.moveToPose(3.5, -3.5, 135, 2200, {.forwards = false}, true);
  chassis.turnToHeading(135, 500, {}, true);
  pros::delay(200);
  intakeMode = 6;
  pros::delay(150);
  intakeMode = 3;
  chassis.moveToPose(3.2, -3.2, 135, 2000, {}, false);

  goStraight(30, 200);
  intakeSpeed = 127;
  reverse(-30, 350);
  goStraight(30, 200);
  reverse(-27.2, 350);
  chassis.moveToPoint(47.25, -47.25, 2000, {}, true);

  // ============================= chutrinhphai ================================

  // =============== loaderr 3 ===============
  chassis.turnToHeading(90, 1000, {}, true);
  chassis.moveToPoint(53, -47.25, 1000, {}, true);
  intakeMode = 1;
  counterLoader.set_value(true);
  resetposeskillyEast(-1);
  pros::delay(50);
  resetposeskillxNorth(1);
  pros::delay(50);
  chassis.moveToPoint(57.75, -47.25, 1350, {.maxSpeed = 100}, false);

  // =============== lui ra ===============
  chassis.moveToPoint(47.25, -47.25, 1000, {.forwards = false}, true);
  chassis.turnToHeading(230, 1300, {}, true);
  intakeMode = 0;
  counterLoader.set_value(false);

  // =============== qua longg ===============
  chassis.moveToPoint(33, -59.25, 900, {}, true);
  chassis.turnToHeading(270, 1000, {}, true);
  chassis.moveToPoint(-25, -59.5, 2250, {}, true);
  chassis.turnToHeading(270, 300, {}, false);
  resetposeskilly(-1);
  pros::delay(50);
  resetposeskillxNorth(-1);
  chassis.turnToHeading(310, 1000, {}, true);
  chassis.moveToPoint(-39, -47.25, 1500, {}, true);
  chassis.turnToHeading(270, 1000, {}, true);

  // =============== out 7 bloc 3 ===============
  chassis.moveToPoint(-28, -47.25, 1000, {.forwards = false}, true);
  pros::delay(100);
  intakeMode = 4;
  pros::delay(150);
  intakeMode = 2;
  chassis.turnToHeading(270, 1900, {}, false);
  resetposeskillxNorth(-1);
  pros::delay(50);
  resetposeskilly(-1);
  pros::delay(50);
}

void chutrinhphai() {
  // =============== out 7 bloc 2 ===============
  pros::delay(100);
  intakeMode = 4;
  pros::delay(150);
  intakeMode = 2;
  chassis.turnToHeading(270, 1900, {}, false);
  resetposeskillxNorth(-1);
  pros::delay(50);
  resetposeskilly(-1);
  pros::delay(50);

  // =============== loaderr 4 ===============
  chassis.turnToHeading(270, 1000, {}, true);
  chassis.moveToPoint(-53, -47.25, 1000, {}, true);
  intakeMode = 1;
  counterLoader.set_value(true);
  resetposeskilly(-1);
  pros::delay(50);
  resetposeskillxNorth(1);
  pros::delay(50);
  chassis.moveToPoint(-57.75, -47.25, 1350, {.maxSpeed = 100}, false);

  // =============== out 7 bloc 3 ===============
  chassis.moveToPoint(-28, -47.25, 1200, {.forwards = false}, true);
  pros::delay(200);
  counterLoader.set_value(false);
  pros::delay(100);
  intakeMode = 4;
  pros::delay(150);
  intakeMode = 2;
  chassis.turnToHeading(270, 1900, {}, false);
  resetposeskillxNorth(-1);
  pros::delay(50);
  resetposeskilly(-1);
  pros::delay(50);

  // goStraight(42, 350);
  // goStraight(-60, 300);

  // parkkkkk
  chassis.swingToHeading(310, DriveSide::LEFT, 1000, {}, true);
  chassis.moveToPoint(-63, -23, 1500, {}, false);
  odoLift.set_value(true);
  chassis.turnToHeading(340, 550, {}, false);
  intakeMode = 2;
  goStraight(86, 1240);
  intakeMode = 1;
  odoLift.set_value(false);
  goStraight(-60, 200);
}

// ASSET(pathqualong1_txt)
void skillz() {
  chutrinhtrai();
  // chassis.setPose(0, 0, 90);
  // resetposeskillxNorth(1);
  // resetposeskillyEast(-1);
  // pros::delay(50);
  quapark2();
  chutrinhphai();

  // chassis.setPose(0, 0, 270);
  // resetposeskillxNorth(-1);
  // resetposeskillxSouth(1);
  // resetposeskillyEast();
  // resetposeskilly(-1);
  // intakeMode = 1;
  // counterLoader.set_value(true);
  // Use encoder-based approach to hit 250mm from the wall
  // driveToNorthDistanceEncoder(240.0f, 40);
  // pros::delay(200);
}

void testodo11() {
  chassis.setPose(0, 0, 90);
  // pros::delay(100);
  resetposeskillyEast(-1);
  lemlib::Pose pose = chassis.getPose();
  controller.print(0, 0, "X:%.1f Y:%.1f", pose.x, pose.y);
  // resetposeskilly(-1);
  // resetposeskillxNorth(1);
  // resetposeskillxSouth(1);
  // pros::delay(100);

  // intakeMode = 1;
  // chassis.moveToPose()
  // chassis.setPose(0, 0, 0);
  // chassis.moveToPoint(0, 24, 10000);
  // chassis.turnToHeading(90, 10000);
}

// ========== PID TUNER ==========
// Adapted from gradient_descent's Desmos-based PID tuner
// https://www.desmos.com/calculator/g3wwb4d5l9 (Drive/Turn PID - FOPDT)
// https://www.desmos.com/calculator/zujd40ppxq (Turn PID - SOPDT, more
// accurate)
//
// HOW TO USE:
// 1. Set manualAutonFunction = &pidTuneDrive (or &pidTuneTurn) in main.cpp
// 2. Connect brain to laptop via USB
// 3. Open PROS terminal (pros terminal) to see serial output
// 4. Run the auton — robot will drive/turn with a voltage step and print data
// 5. Copy the LaTeX output (everything between the brackets)
// 6. Paste into the Desmos calculator's data list
// 7. Desmos will compute optimal kP, kI, kD values for you!
//
// NOTE: Make sure the robot has enough space to drive ~3-4 feet straight
//       or do a full rotation for turn tuning.

static double getAvgVelocityRPM() {
  double sum = 0;
  auto leftVels = leftMotors.get_actual_velocity_all();
  auto rightVels = rightMotors.get_actual_velocity_all();
  int count = 0;
  for (auto v : leftVels) {
    sum += v;
    count++;
  }
  for (auto v : rightVels) {
    sum += v;
    count++;
  }
  return (count > 0) ? (sum / count) : 0.0;
}

static double getAvgPositionInches() {
  auto pose = chassis.getPose();
  return sqrt(pose.x * pose.x + pose.y * pose.y);
}

void pidTuneDrive() {
  printf("\n\n===== DRIVE PID TUNER =====\n");
  printf("https://www.desmos.com/calculator/g3wwb4d5l9\n\n");

  chassis.setPose(0, 0, 0);
  pros::delay(200);

  int stepPower = 80; // Motor power step (0-127)
  int durationMs = 3000;
  int sampleIntervalMs = 10;
  int samples = durationMs / sampleIntervalMs;

  double initialX = chassis.getPose().x;
  double initialY = chassis.getPose().y;

  printf("\\left[");

  leftMotors.move(stepPower);
  rightMotors.move(stepPower);

  for (int i = 0; i < samples; i++) {
    auto pose = chassis.getPose();
    double dist = sqrt((pose.x - initialX) * (pose.x - initialX) +
                       (pose.y - initialY) * (pose.y - initialY));
    double timeSec = (double)i * sampleIntervalMs / 1000.0;

    printf("\\left(%.3f,%.4f\\right)", timeSec, dist);
    if (i < samples - 1)
      printf(",");

    // Flush periodically so terminal shows progress
    if (i % 50 == 0)
      fflush(stdout);

    pros::delay(sampleIntervalMs);
  }

  // Stop motors
  leftMotors.move(0);
  rightMotors.move(0);
  leftMotors.brake();
  rightMotors.brake();

  printf("\\right]\n\n");
  printf("Step input: %d (motor power, 0-127 scale)\n", stepPower);
  printf("===== DONE =====\n");
  fflush(stdout);
}

void pidTuneTurn() {
  printf("\n\n===== TURN PID TUNER =====\n");
  printf("https://www.desmos.com/calculator/zujd40ppxq\n\n");

  chassis.setPose(0, 0, 0);
  pros::delay(200);

  int stepPower = 60;
  int durationMs = 3000;
  int sampleIntervalMs = 10;
  int samples = durationMs / sampleIntervalMs;

  double initialHeading = chassis.getPose().theta;

  printf("\\left[");

  leftMotors.move(-stepPower);
  rightMotors.move(stepPower);

  for (int i = 0; i < samples; i++) {
    double heading = chassis.getPose().theta - initialHeading;
    double timeSec = (double)i * sampleIntervalMs / 1000.0;

    printf("\\left(%.3f,%.4f\\right)", timeSec, heading);
    if (i < samples - 1)
      printf(",");

    if (i % 50 == 0)
      fflush(stdout);

    pros::delay(sampleIntervalMs);
  }

  leftMotors.move(0);
  rightMotors.move(0);
  leftMotors.brake();
  rightMotors.brake();

  printf("\\right]\n\n");
  printf("Step input: %d (motor power, 0-127 scale)\n", stepPower);
  printf("===== DONE =====\n");
  fflush(stdout);
}

void pidTuneVelocity() {
  printf("\n\n===== VELOCITY vs MOTOR POWER DATA =====\n");
  printf(
      "Paste the output below into Desmos Velocity Controller calculator.\n");
  printf("https://www.desmos.com/calculator/7go1fxuufd\n\n");

  // Test motor powers from 0 to 127 in steps
  int powers[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 127};
  int numSteps = sizeof(powers) / sizeof(powers[0]);
  double outputs[14] = {0};

  float direction = 1.0;

  for (int s = 0; s < numSteps; s++) {
    if (powers[s] == 0)
      continue;

    int power = (int)(direction * powers[s]);
    leftMotors.move(power);
    rightMotors.move(power);
    pros::delay(1000); // Wait for steady state

    // Average velocity over 500 samples
    double vSum = 0;
    int n = 500;
    for (int i = 0; i < n; i++) {
      vSum += getAvgVelocityRPM();
    }
    outputs[s] = direction * (vSum / (double)n);

    // Slow down gracefully
    float v = (float)(powers[s]) * direction;
    while (fabs(v) > 5) {
      v *= 0.9;
      leftMotors.move((int)v);
      rightMotors.move((int)v);
      pros::delay(10);
    }
    leftMotors.move(0);
    rightMotors.move(0);

    // Brief pause and flip direction
    pros::delay(500);
    direction = -direction;
  }

  leftMotors.brake();
  rightMotors.brake();

  // Print results
  printf("\\left[");
  for (int s = 0; s < numSteps; s++) {
    printf("\\left(%d,%.4f\\right)", powers[s], outputs[s]);
    if (s < numSteps - 1)
      printf(",");
  }
  printf("\\right]\n\n");
  printf("===== DONE =====\n");
  fflush(stdout);
}
