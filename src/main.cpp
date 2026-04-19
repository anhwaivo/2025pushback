#include "main.h"
#include "auton.h"
#include "auton_selector.h"
// #include "monte.h"  // MCL disabled to save resources
#include "lemlib/api.hpp" 
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/apix.h"  
#include <future>
#include <cmath>
#include <cstdio>


// ========== GLOBAL STATE ==========
bool calibrate = false;
bool disableBrainScreen = true;  // Set true during match to save resources

// === Motors ===
pros::MotorGroup leftMotors({-12, -13, -14}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({17, 18, 19}, pros::MotorGearset::blue);
pros::Motor intake(-16, pros::MotorGearset::blue);
pros::Motor conveyor(9, pros::MotorGearset::blue);
pros::Motor outtake(8, pros::MotorGearset::blue);

// === Pneumatics ===
pros::adi::Pneumatics stage('A', true);
pros::adi::Pneumatics descoreRight('E', true); 
pros::adi::Pneumatics counterLoader('F', false);
pros::adi::Pneumatics descoreLeft('C', true);
pros::adi::Pneumatics odoLift('B', true);
pros::adi::Pneumatics blockblock('D', false);

// === Sensors ===
pros::Rotation horizontal_encoder(10);
pros::Rotation vertical_encoder(1);
pros::Imu imu(2);
pros::Optical colorSensor(4);

// === Distance Sensors for Monte Carlo Localization ===
#include "monte_config.h"
pros::Distance dNorth(MCL_SENSOR_NORTH_PORT);  // North-facing sensor
pros::Distance dSouth(MCL_SENSOR_SOUTH_PORT);  // South-facing sensor
pros::Distance dEast(MCL_SENSOR_EAST_PORT);    // East-facing sensor
pros::Distance dWest(MCL_SENSOR_WEST_PORT);     // West-facing sensor

// === Tracking Wheels ===
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.8); // nullpt
// === Odom sensors ===
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// === Drivetrain ===
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.26, lemlib::Omniwheel::NEW_325, 450, 2);

// === PID Controllers ===
// lemlib::ControllerSettings lateral_controller(14, 0, 91.5, 0, 0, 0, 0, 0, 0); // 8 41 | 14 87 | 14 91.5
// lemlib::ControllerSettings lateral_controller(15, 0, 82.5, 0, 0, 0, 0, 0, 0); // 8 41 | 14 87 | 14 91.5
lemlib::ControllerSettings lateral_controller(9, 0, 45.35, 0, 0, 0, 0, 0, 0); // 8 41 | 14 87 | 14 91.5
lemlib::ControllerSettings angular_controller(5, 0, 42, 0, 0, 0, 0, 0, 0);
// lemlib::ControllerSettings angular_controller(5, 0, 30, 0, 0, 0, 0, 0, 0);


// === Input Curves ===
// ExpoDriveCurve(deadband, minOutput, curve)
// - deadband: vùng input được coi là 0 (deadzone)
// - minOutput: output tối thiểu khi input vượt qua deadband
// - curve: độ cong (1.0 = linear, >1.0 = cong hơn, input nhỏ sẽ chậm hơn)
// Giá trị cao hơn (1.1-1.2) sẽ làm input nhỏ chậm hơn đáng kể
// lemlib::ExpoDriveCurve throttle_curve(3, 5, 1.12); // Tăng từ 1.019 lên 1.12 để chậm hơn ở input nhỏ
// lemlib::ExpoDriveCurve steer_curve(3, 5, 1.12); // Đồng bộ với throttle, tăng từ 1.05 lên 1.12

// === Chassis ===
// lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// === Controller ===
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ========== AUTON SELECTOR ==========
AutonSelector auton_selector;

// ========== MANUAL AUTON SELECTION ==========
// Set this to nullptr to use selector, or set to a function pointer to override selector
// Example: manualAutonFunction = &leftcenterdescore;
// Available autons:
// - &leftdescore7bloc
// - &leftfastdescore
// - &leftcenterdescore
// - &leftlongcenter2
// - &rightdescore7bloc
// - &rightcenterdescore
// - &rightdescore9bloc
// - &skillz
// - &testodo11
// - &pidTuneDrive
// - &pidTuneTurn
// - &pidTuneVelocity
// - &awp
AutonSelector::routine_action_t manualAutonFunction = nullptr; // nullptr = use selector, or set to function pointer

void disabled() {
    // Update selector while disabled
    while (pros::competition::is_disabled()) {
        auton_selector.update();
        pros::delay(20);
    }
}

void competition_initialize() {
    // Initialize selector for competition
    auton_selector.initialize();
    while (pros::competition::is_disabled()) {
        auton_selector.update();
        pros::delay(20);
    }
}


// ========== HELPER STATES ==========
// bool isSkillMode = false;  // rememberrr this
// bool boostMode = false;

bool stageState = false;
bool gripperState = false;
bool counterLoaderState = false;
bool punchGoalState = false;
bool descoreLeftState = true;
bool descoreRightState = true;
bool odoLiftState = false;
bool doubleParkState = false;
bool sequenceState = false;
bool colorFilterEnabled = false;
int colorToReject = 0;
bool loaderMode = false;
double speedAbove;
double speedUnder;
bool colorSortEnabled = false;
bool useManualColorSort = true;
AutonSelector::Color manualColorToSort = AutonSelector::Color::RED;
// Intake mode values:
// 0 = Stop motors (off)
// 1 = Blockblock mode (intake with blockblock engaged)
// 2 = Long goal mode (intake for long goal, blockblock off)
// 3 = Center goal mode (intake for center goal, blockblock on)
// 4 = Reverse/outtake mode (motorTrai reverse, motorPhai forward)
int intakeMode = 0;
int intakeSpeed = 127;  // 0-127, used by intake task when applying intakeMode (set from auton)
int current_auton = 0;
bool odoLifted = false;
int currentStageMode = -1;
bool manualPneumaticOverride = false; // When true, intakeTask won't auto-return pneumatics


// ========== HELPER FUNCTIONS ==========
bool checkBlockDetected() {
    int proximity = colorSensor.get_proximity();
    return (proximity > 200);
}

void reverseMotorsForDuration(int duration_ms) {
    int speed = 127;
    intake.move(-speed);
    conveyor.move(-speed);
    outtake.move(-speed);

    int delay_count = duration_ms / 25; 
    for (int i = 0; i < delay_count; i++) {
        pros::delay(25);
    }
}

// ========== TASKS ==========
void intakeTaskFn() { 
    static bool r1Toggle = false;  // R1 toggle state
    static bool lastR1State = false;
    static uint32_t lastR2HoldTime = 0;
    while (true) {
        int speed = 127;
        
        bool r2Held = false;
        bool yHeld = false;
        bool l2Held = false;
        bool xHeld = false;
        
        if (!pros::competition::is_autonomous()) {
            bool r1Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
            r2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
            yHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
            l2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
            xHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
            
            // R1: toggle on press (edge detection)
            if (r1Current && !lastR1State) {
                r1Toggle = !r1Toggle;
            }
            lastR1State = r1Current;
        }
        
        if (!pros::competition::is_autonomous()) {
            // Cancel R1 toggle if other intake buttons are pressed
            if (r2Held || yHeld || l2Held) {
                r1Toggle = false;
            }

            // L2 hold: reverse all motors (highest priority, swapped from Y)
            if (l2Held) {
                intake.move(-speed);
                conveyor.move(-speed);
                outtake.move(-speed);
            }
            // R2 hold or Y hold: spin all 3 motors (Y swapped with L1)
            else if (r2Held) {
                intake.move(speed);
                conveyor.move(speed);
                outtake.move(speed);
            }
            else if (yHeld) {
                intake.move(speed);
                conveyor.move(speed);
                outtake.move(speed);
            }
            // R1 toggled on: spin intake and conveyor only
            else if (r1Toggle) {
                intake.move(speed);
                conveyor.move(speed);
                // outtake.move(0);
            }
            // Nothing active: stop all
            else {
                intake.move(0);
                conveyor.move(0);
                outtake.move(0);
            }
            
            // Any intake button clears manual override
            if (yHeld || r2Held || xHeld || r1Toggle) {
                manualPneumaticOverride = false;
            }

            if (yHeld) {
                stage.set_value(false);
                blockblock.set_value(false);
            } else if (r2Held) {
                stage.set_value(true);
                blockblock.set_value(true);
                lastR2HoldTime = pros::millis();
            } else if (lastR2HoldTime > 0 && (pros::millis() - lastR2HoldTime < 500)) {
                blockblock.set_value(true);
            } else if (xHeld) {
                blockblock.set_value(false);
            } else if (r1Toggle) {
                blockblock.set_value(false);
                stage.set_value(true);
            } else if (!manualPneumaticOverride) {
                // Auto-return to defaults when no intake buttons active
                blockblock.set_value(false);
                stage.set_value(true);
            }
        } else {
            // Autonomous mode: use intakeMode
            if (intakeMode == 0) {
                intake.move(0);
                conveyor.move(0);
                outtake.move(0);
            } else if (intakeMode == 1) {
                // Intake + conveyor
                intake.move(intakeSpeed);
                conveyor.move(intakeSpeed);
                outtake.move(0);
                blockblock.set_value(false);
            } else if (intakeMode == 2) {
                // All 3 motors
                intake.move(intakeSpeed);
                conveyor.move(intakeSpeed);
                outtake.move(intakeSpeed);
                blockblock.set_value(true);
                stage.set_value(true);
            } else if (intakeMode == 3) {
                intake.move(intakeSpeed);
                conveyor.move(intakeSpeed);
                outtake.move(105);
                blockblock.set_value(false);
                stage.set_value(false);
            } else if (intakeMode == 4) {
                // Reverse all
                intake.move(-80);
                conveyor.move(-intakeSpeed);
                outtake.move(-intakeSpeed);
            }
        }
        
        pros::delay(25);
    }
}

// intakeTask started in opcontrol() to avoid global constructor race condition

void handleDrive() {
    // Đọc input từ controller
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    const int forwardDeadzone = 5;
    const int turnDeadzone = 5; 
    if (abs(leftY) < forwardDeadzone) leftY = 0;
    if (abs(rightX) < turnDeadzone) rightX = 0;
    // if (isSkillMode && boostMode) {
    //     leftY = (int)(leftY * 81.0 / 127.0);
    //     rightX = (int)(rightX * 81.0 / 127.0);
    // }
    
    // chassis.arcade(leftY, rightX, false, 0.15);
    chassis.curvature(leftY, rightX);
}

void handleManualPneumatics() {
    // L1: toggle both left and right descore
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        descoreLeftState = !descoreLeftState;
        descoreLeft.set_value(descoreLeftState);
        descoreRightState = !descoreRightState;
        descoreRight.set_value(descoreRightState);
    }
    
    // TOGGLE descore left and right independently by UP AND LEFT
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        descoreLeftState = !descoreLeftState;
        descoreLeft.set_value(descoreLeftState);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        descoreRightState = !descoreRightState;
        descoreRight.set_value(descoreRightState);
    }

    // B: set stage false and blockblock false, prevent auto-return
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        stage.set_value(false);
        blockblock.set_value(false);
        manualPneumaticOverride = true;
    }

    // Y is now used for motor reverse (hold) in intakeTaskFn

    // RIGHT: toggle counter loader
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        counterLoaderState = !counterLoaderState;
        counterLoader.set_value(counterLoaderState);
    }

    // DOWN: toggle odo lift
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        odoLiftState = !odoLiftState;
        odoLift.set_value(odoLiftState);
    }

}

// Telemetry update task for autonomous
void telemetryTaskFn(void* param) {
    while (true) {
        if (!disableBrainScreen) {
            auton_selector.update_telemetry();
            lv_timer_handler();
        }
        pros::delay(100);
    }
}

void autonomous() {
    if (!disableBrainScreen) {
        auton_selector.show_telemetry();
        pros::Task telemetryTask(telemetryTaskFn, nullptr, "TelemetryTask");
    }
   

    // isSkillMode = (auton_selector.get_selected_name() == "Skillz");

    if (manualAutonFunction != nullptr) {
        manualAutonFunction();
    } else {
        auton_selector.run_auton();
    }
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(90, 100000);
    // chassis.moveToPoint(0, 24, 10000);


// chassis.setPose(-46.5, 0, 90.0);

// chassis.swingToPoint(24.0, -24.0, DriveSide::LEFT, 647, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 40, .earlyExitRange = 2.85}, false);
// chassis.moveToPoint(-24.0, 24.0, 1900, {}, false);
// // ml_mech.set_value(true);
// // blockblock.set_value(true);
// intake.move(127);
// conveyor.move(127);
// chassis.turnToPoint(-42.0, 48.0, 790, {}, false);
// chassis.moveToPoint(-42.0, 48.0, 1320, {}, false);
// chassis.turnToPoint(-57.84, 48.0, 652, {}, false);
// chassis.moveToPoint(-57.84, 48.0, 928, {}, false);
// pros::delay(700);
// chassis.moveToPoint(-36.0, 48.0, 1095, {.forwards = false}, false);
// blockblock.set_value(true);
// intake.move(127);
// conveyor.move(127);
// outtake.move(127);
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0, 24, 10000);
    // chassis.turnToHeading(90, 10000);
    
}

void initialize() {
    chassis.calibrate();
    // colorSensor.set_led_pwm(100);

    auton_selector.initialize();

    // Start intake task early so it runs during both autonomous and opcontrol
    pros::Task intakeTask(intakeTaskFn);

    // chassis.setPose(45, 0, 270);
    // startMCL(chassis); // MCL disabled to save resources
}


// // Skill mode opcontrol: chạy từng chu trình, DOWN = ngắt
// static bool skillRunning = false;
// static pros::Task* skillTask = nullptr;
// static void skillTaskFnAll(void*) {
//     odoLift.set_value(false);
//     pros::delay(50);
//     chutrinhtrai();
//     quapark2();
//     chutrinhphai();
//     skillRunning = false;
// }
// static void skillTaskFnQuapark2(void*) {
//     odoLift.set_value(false);
//     pros::delay(50);
//     resetposeskillxNorth(1);
//     resetposeskilly(1);
//     quapark2();
//     skillRunning = false;
// }
// static void skillTaskFnChutrinhphai(void*) {
//     // chassis.setPose(-29.5, -47.25, chassis.getPose().theta);
//     // resetposeskilly(-1);
//     resetposeskillxNorth(-1);
//     resetposeskilly(-1);
//     pros::delay(50);
//     odoLift.set_value(false);
//     chutrinhphai();
//     skillRunning = false;
// }
// static void skillAbort() {
//     intakeMode = 0;
//     odoLift.set_value(true);
//     chassis.cancelMotion();
//     if (skillTask != nullptr) {
//         skillTask->remove();
//         skillTask = nullptr;
//     }
//     skillRunning = false;
// }
// static void skillStartAll() {
//     skillRunning = true;
//     skillTask = new pros::Task(skillTaskFnAll, nullptr, "skill_all");
// }
// static void skillStartQuapark2() {
//     skillRunning = true;
//     skillTask = new pros::Task(skillTaskFnQuapark2, nullptr, "skill_qp2");
// }
// static void skillStartChutrinhphai() {
//     skillRunning = true;
//     skillTask = new pros::Task(skillTaskFnChutrinhphai, nullptr, "skill_ctp");
// }

// ========== OP CONTROL ==========
void opcontrol() {   

    // chassis.setPose(45.4, 0, 270.0);
    odoLift.set_value(false);

    if (!disableBrainScreen) {
        auton_selector.show_telemetry();
    }
    intakeMode = 0;
    
    int telemetryCounter = 0;
    int controllerDisplayCounter = 0;

    while (true) {
        // if (isSkillMode) {
        //     // UP = chạy skill (cả 3 chu trình)
        //     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && !skillRunning) {
        //         skillStartAll();
        //     }
        //     // DOWN = chỉ ngắt chu trình đang chạy
        //     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) && skillRunning) {
        //         skillAbort();
        //     }
        //     // LEFT = chạy quapark2
        //     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) && !skillRunning) {
        //         skillStartQuapark2();
        //     }
        //     // RIGHT = chạy chutrinhphai
        //     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) && !skillRunning) {
        //         skillStartChutrinhphai();
        //     }
        // }
        
        // Handle manual pneumatics and drive
        handleManualPneumatics();
        handleDrive();
        
        // Update telemetry periodically (every 4 loops = ~100ms at 25ms delay)
        if (!disableBrainScreen && telemetryCounter % 4 == 0) {
            auton_selector.update_telemetry();
            lv_timer_handler();
        }
        telemetryCounter++;
        
        // Update controller display periodically (every 20 loops = ~500ms)
        // Controller serial is slow, reducing frequency saves CPU
        if (controllerDisplayCounter % 20 == 0 && !disableBrainScreen) {
            lemlib::Pose pose = chassis.getPose();
            controller.print(0, 0, "X:%.1f Y:%.1f", pose.x, pose.y);
        }
        controllerDisplayCounter++;
        
        pros::delay(25);
    }
}

// void opcontrol() {   
    
//     manualAutonFunction();
    
    
// }
