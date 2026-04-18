#include "main.h"
#include "lemlib/api.hpp" 
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <future>
#include <cmath>


// ========== GLOBAL STATE ==========
bool calibrate = false;

// === Motors ===
pros::MotorGroup leftMotors({-2, -4, -3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({7, 8, 9}, pros::MotorGearset::blue);
pros::Motor motorTrai(12, pros::MotorGearset::blue);
pros::Motor motorPhai(-13, pros::MotorGearset::blue);

// === Pneumatics ===
pros::adi::Pneumatics stage('A', false); // Stage (from up)
pros::adi::Pneumatics descoreCenter('B', false); // DescoreCenter
pros::adi::Pneumatics counterLoader('C', false);
pros::adi::Pneumatics descoreLeft('D', false);
pros::adi::Pneumatics doublePark('E', false);
pros::adi::Pneumatics odoLift('F', true);
pros::adi::Pneumatics gripper('G', true);

// === Sensors ===
pros::Rotation horizontal_encoder(-5);
pros::Rotation vertical_encoder(-6);
pros::Imu imu(10);
pros::Optical colorSensor(19);
pros::Distance distance_sensor(14); 

// === Tracking Wheels ===
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -1);
// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, 1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0);
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 1);

// === Odom sensors ===
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// === Drivetrain ===
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.8740157, lemlib::Omniwheel::NEW_325, 450, 2);

// === PID Controllers ===
lemlib::ControllerSettings lateral_controller(12, 0, 60, 0, 0, 0, 0, 0, 0); // D 45 D2 35
// lemlib::ControllerSettings lateral_controller(10, 0, 0, 0, 0, 0, 0, 0, 0); // D 45 D2 35
lemlib::ControllerSettings angular_controller(6, 0, 30, 0, 0, 0, 0, 0, 0); // ngon

// === Input Curves ===
// ExpoDriveCurve(deadband, minOutput, curve)
// - deadband: vùng input được coi là 0 (deadzone)
// - minOutput: output tối thiểu khi input vượt qua deadband
// - curve: độ cong (1.0 = linear, >1.0 = cong hơn, input nhỏ sẽ chậm hơn)
// Giá trị cao hơn (1.1-1.2) sẽ làm input nhỏ chậm hơn đáng kể
// lemlib::ExpoDriveCurve throttle_curve(3, 5, 1.12); // Tăng từ 1.019 lên 1.12 để chậm hơn ở input nhỏ
// lemlib::ExpoDriveCurve steer_curve(3, 5, 1.12); // Đồng bộ với throttle, tăng từ 1.05 lên 1.12

// old settings:
// 3, 10, 1.019
// 20, 5, 1.05

// === Chassis ===
// lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// === Controller ===oo
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// pros::Distance distance_sensor(15);

void disabled() {}
void competition_initialize() {}


const char* autonNames[] = {
    "Right WP",
    "Right Elim",
    "Right Solo WP",

    "Left WP",
    "Left Elim",
    "Left Solo WP",

    "Skills"
};

// ========== HELPER STATES ==========
bool isSkillMode = false; 
bool boostMode = false;

bool stageState = false;
bool gripperState = false;
bool counterLoaderState = false;
bool punchGoalState = false;
bool descoreLeftState = false;
bool descoreCenterState = false;
bool odoLiftState = false;
bool doubleParkState = false;
bool sequenceState = false;
bool colorFilterEnabled = false;
int colorToReject = 0;
bool loaderMode = false;
double speedAbove;
double speedUnder;
bool colorSortEnabled = false;
int intakeMode = 0;
int current_auton = 0;
bool odoLifted = false;
int currentStageMode = -1; // Lưu stage mode hiện tại (-1 = chưa set, 0-3 = các mode)

// ========== TASKS ==========
void intakeTaskFn() {
    colorSensor.set_led_pwm(100); 

    while (true) {
        int speed = 127; // Match mode: luôn max speed
        
        // Skill mode: stage 0 và boost mode = 79, các trường hợp khác = 127
        if (isSkillMode && (boostMode || currentStageMode == 0)) {
            speed = 85;
        }
        
        if (intakeMode == 1) { // ĐANG HÚT
            motorPhai.move(speed);
            motorTrai.move(speed);

        } else if (intakeMode == 2) { // ĐANG NHẢ ngc
            motorPhai.move(-speed);
            motorTrai.move(-speed);

        } else { // STOP
            motorPhai.move(0);
            motorTrai.move(0);
        }

        pros::delay(20);
    }
}

pros::Task intakeTask(intakeTaskFn); // Declare once


void handleDrive() {
    // Đọc input từ controller
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    
    // Deadzone để tránh drift nhỏ - deadzone nhỏ hơn cho turn để turn nhạy hơn
    const int forwardDeadzone = 5;
    const int turnDeadzone = 5; // Deadzone nhỏ hơn cho turn
    if (abs(leftY) < forwardDeadzone) leftY = 0;
    if (abs(rightX) < turnDeadzone) rightX = 0;
    
    // Skill mode + boost mode: giảm tốc độ drivetrain xuống 75%
    if (isSkillMode && boostMode) {
        leftY = (int)(leftY * 0.65);
        rightX = (int)(rightX * 0.65);
    }
    
    // Arcade drive với desaturateBias để điều chỉnh throttle/turn priority khi saturation
    // desaturateBias = 0.35: ưu tiên throttle hơn turn (0 = throttle priority, 1 = turn priority)
    // Giá trị thấp hơn (0.3-0.4) sẽ làm turn chậm hơn khi max throttle forward
    chassis.arcade(leftY, rightX, false, 0.35);
    
    // Hoặc dùng curvature drive (không có desaturateBias):
    // chassis.curvature(leftY, rightX);

}

void setStage(int stageMode) {
    // stageMode: 0 = center goal + descorecenter xuong (stage trên extend, descorecenter xuống)
    //            1 = long goal (cả stage trên và dưới thu lại - stage = false, descorecenter = false)
    //            2 = blockblock + descorecenter len (stage dưới extend, descorecenter lên)
    //            3 = blockblock + descorecenter xuong (stage trên extend, descorecenter lên)
    
    currentStageMode = stageMode; // Lưu stage mode hiện tại để intake task có thể sử dụng
    
    if (stageMode == 0) {
        // center goal + descorecenter xuong
        // stage trên extend (A = true), descorecenter xuống (B = false)
        stageState = true;
        descoreCenterState = false;
        stage.set_value(true);
        descoreCenter.set_value(false);
        
    } else if (stageMode == 1) {
        // long goal - cả stage trên và dưới thu lại
        // stage dưới extend/trên unextend (A = false), descorecenter thu lại (B = false)
        stageState = false;
        descoreCenterState = false;
        stage.set_value(false);
        descoreCenter.set_value(false);
    } else if (stageMode == 2) {
        // blockblock + descorecenter len
        // stage dưới extend (A = false), descorecenter lên (B = true)
        stageState = false;
        descoreCenterState = true;
        stage.set_value(false);
        descoreCenter.set_value(true);
    } else if (stageMode == 3) {
        // blockblock + descorecenter xuong
        // stage trên extend (A = true), descorecenter lên (B = true) - KHÁC với mode 0
        stageState = true;
        descoreCenterState = true;
        stage.set_value(true);
        descoreCenter.set_value(true);
    }
}

void handleManualPneumatics() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        odoLiftState = !odoLiftState;  
        odoLift.set_value(odoLiftState); 
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        gripperState = !gripperState;  
        gripper.set_value(gripperState); 
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        doubleParkState = !doubleParkState;  
        doublePark.set_value(doubleParkState);  
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        counterLoaderState = !counterLoaderState;
        counterLoader.set_value(counterLoaderState);
    }

    // R2: Skill mode = boost toggle, Match mode = descore toggle
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        if (isSkillMode) {
            // Skill mode: toggle boost
            boostMode = !boostMode;
            // Rumble khi toggle boost mode
            controller.rumble(".");
        } else {
            // Match mode: toggle descore
            descoreLeftState = !descoreLeftState;
            descoreLeft.set_value(descoreLeftState);
        }
    }
    
    // --- Stage and DescoreCenter controls ---
    // Mỗi nút một chức năng riêng:
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        // blockblock + descorecenter len (stage dưới extend, descorecenter lên)
        setStage(2);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        // blockblock + descorecenter xuong (stage trên extend, descorecenter xuống)
        setStage(3);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        // long goal (cả stage trên và dưới thu lại)
        setStage(1);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        // center goal + descorecenter xuong (stage trên extend, descorecenter xuống)
        setStage(0);
    }

}

// ========== AUTON =========

// void right_wp();
// void right_elim();
// void right_solo_wp();

// void left_wp();
// void left_elim();
// void left_solo_wp();

// void autoSkill();


// ========= LEFT DESCORERE 7 BLOCK =========
ASSET(pathdescore1_txt);
ASSET(pathdescore2_txt);
ASSET(pathdescore3_txt);
void leftdescore7bloc() {
    setStage(3); 
    // chassis.setPose(60.895, -17.116, 270);
    intakeMode = 1;
    chassis.setPose(48.411, -10.375, 247.334);
    chassis.moveToPoint(25.364, -20, 700, {}, true); // t 700 | y 23
    pros::delay(500);
    counterLoader.set_value(true);
    chassis.turnToHeading(135, 650, {}, false);
    chassis.moveToPoint(41.5, -45.5, 800, {}, false);
    chassis.turnToHeading(90, 250, {}, true);
    // chassis.moveToPose(54, -47, 90, 1000, {}, false);
    // pros::delay(300);
    // chassis.moveToPose(60, -48, 90, 1300, {}, false); // t 2000
    chassis.moveToPoint(58, -49, 1000, {}, false);
    // chassis.waitUntilDone();
    pros::delay(400);
    intakeMode = 0;
    chassis.moveToPoint(23, -47.75, 2300, {.forwards = false, .minSpeed = 90}); // xam l
    pros::delay(600);
    intakeMode = 2;
    pros::delay(150);
    setStage(2);
    // pros::delay(200);
    intakeMode = 1;
    pros::delay(2300);
    intakeMode = 0;

    chassis.setPose(29.5, -47.15, 90);

    // chassis.moveToPose(40, -47.5, 91, 2000, {}, false); 
    // chassis.moveToPoint((chassis.getPose().x - 5), chassis`.getPose().y, 1800, {}, false);
    // chassis.moveToPoint(38.5, -47.15, 1500, {}, false);
    // chassis.turnToHeading(0, 900, {}, false);
    // chassis.moveToPoint(38.5, -58.5, 1700, {.forwards = false}, false); 
    // chassis.turnToHeading(90, 900, {}, false);
    // chassis.moveToPoint(30, -58.5, 1700, {.forwards = false}, false); 
    
    // --------------------------------------------------------
    chassis.moveToPoint(40, -47.15, 800, {}, false); // x 49
    // chassis.turnToHeading(235, 700); // MAGIC
    chassis.turnToHeading(35, 500, {}, false);

    // // chassis.follow(pathdescore1_txt, 9, 1500, false, true);
    chassis.follow(pathdescore2_txt, 9, 3000, false, true);
    // chassis.follow(pathdescore3_txt, 9, 2000, false, true);

    // chassis.moveToPoint(38, -59.5, 1000, {.forwards = false}, false);
    pros::delay(1000);
    descoreLeft.set_value(false);
    // --------------------------------------------------------
    
    pros::delay(2000);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y, 6000, {}, false);
}


void leftfastdescore() {
    setStage(3); 
    // chassis.setPose(60.895, -17.116, 270);
    intakeMode = 1;
    chassis.setPose(48.411, -10.375, 247.334);
    chassis.moveToPoint(25.364, -20, 700, {}, true); // t 700 | y 23
    pros::delay(500);
    counterLoader.set_value(true);
    chassis.turnToHeading(135, 650, {}, false);
    chassis.moveToPoint(36, -45.5, 800, {}, false);
    chassis.turnToHeading(90, 250, {}, true);
    // chassis.moveToPose(54, -47, 90, 1000, {}, false);
    // pros::delay(300);
    // chassis.moveToPose(60, -48, 90, 1300, {}, false); // t 2000
    // chassis.moveToPoint(67, -49, 1000, {}, false);
    // // chassis.waitUntilDone();
    // pros::delay(400);
    // intakeMode = 0;
    chassis.moveToPoint(23, -47.75, 2300, {.forwards = false, .minSpeed = 90}); // xam l
    pros::delay(600);
    // intakeMode = 2;
    pros::delay(150);
    setStage(2);
    // pros::delay(200);
    intakeMode = 1;
    pros::delay(1500);
    // intakeMode = 0;

    chassis.setPose(29.5, -47.15, 90);

    // chassis.moveToPose(40, -47.5, 91, 2000, {}, false); 
    // chassis.moveToPoint((chassis.getPose().x - 5), chassis`.getPose().y, 1800, {}, false);
    // chassis.moveToPoint(38.5, -47.15, 1500, {}, false);
    // chassis.turnToHeading(0, 900, {}, false);
    // chassis.moveToPoint(38.5, -58.5, 1700, {.forwards = false}, false); 
    // chassis.turnToHeading(90, 900, {}, false);
    // chassis.moveToPoint(30, -58.5, 1700, {.forwards = false}, false); 
    
    // --------------------------------------------------------
    chassis.moveToPoint(40, -47.15, 800, {}, false); // x 49
    // chassis.turnToHeading(235, 700); // MAGIC
    chassis.turnToHeading(35, 500, {}, false);

    // // chassis.follow(pathdescore1_txt, 9, 1500, false, true);
    chassis.follow(pathdescore2_txt, 9, 3000, false, true);
    // chassis.follow(pathdescore3_txt, 9, 2000, false, true);

    // chassis.moveToPoint(38, -59.5, 1000, {.forwards = false}, false);
    pros::delay(800);
    descoreLeft.set_value(false);
    // --------------------------------------------------------

    
    pros::delay(1000);
    chassis.moveToPoint((chassis.getPose().x), chassis.getPose().y, 6000, {}, false);
    chassis.turnToHeading(60, 500, {.minSpeed = 120},false);
}

// ========= LEFT CENTER DESCORERE =========
void leftcenterdescore() {
    setStage(3); 
    // chassis.setPose(60.895, -17.116, 270);
    intakeMode = 1;
    chassis.setPose(48.411, -10.375, 247.334);
    chassis.moveToPoint(18.411, -25, 1400, {}, true); // t 700 | y 23
    // chassis.moveToPose(25.364, -20, 1500, {}, true); // t 700 | y 23
    pros::delay(500);
    counterLoader.set_value(true);
    // // chassis.turnToHeading(135, 650, {}, false);
    pros::delay(600);
    counterLoader.set_value(false);

    // --------------------------------------------------------
    // chassis.moveToPose(7, -43, 206, 1600, {}, false); // an long goal
    chassis.moveToPose(9, -40, 206, 1600, {}, false);
    chassis.moveToPoint(23, -25, 1200, {.forwards = false}, true);
    intakeMode = 0;
    setStage(0);
    chassis.moveToPose(7.5, -9, 135, 2000, {.forwards = false}, false);
    intakeMode = 1;
    pros::delay(1500);

    // --------------------------------------------------------
    pros::Task intakeSequenceTask([&]() {
        counterLoader.set_value(true);
        intakeMode = 2;
        pros::delay(200);
        setStage(3);
        pros::delay(250);
        intakeMode = 1;
    });
    
    // --------------------------------------------------------


    chassis.turnToHeading(135, 650, {}, false);
    chassis.moveToPoint(46, -47.5, 1000, {}, false);
    chassis.turnToHeading(90, 700, {}, false);
    // chassis.moveToPose(54, -47, 90, 1000, {}, false);
    // pros::delay(300);
    // chassis.moveToPose(60, -48, 90, 1300, {}, false); // t 2000
    chassis.moveToPoint(67, -47.5, 1200, {}, false);
    // chassis.waitUntilDone();
    pros::delay(400);
    intakeMode = 0;
    chassis.moveToPoint(23, -48, 2300, {.forwards = false, .minSpeed = 90}); // xam l
    pros::delay(600);
    intakeMode = 2;
    pros::delay(150);
    setStage(2);
    // pros::delay(200);
    intakeMode = 1;
    pros::delay(1900);
    intakeMode = 0;

    chassis.setPose(29.8, -47.15, 90);

    // chassis.moveToPose(40, -47.5, 91, 2000, {}, false); 
    // chassis.moveToPoint((chassis.getPose().x - 5), chassis`.getPose().y, 1800, {}, false);
    // chassis.moveToPoint(38.5, -47.15, 1500, {}, false);
    // chassis.turnToHeading(0, 900, {}, false);
    // chassis.moveToPoint(38.5, -58.5, 1700, {.forwards = false}, false); 
    // chassis.turnToHeading(90, 900, {}, false);
    // chassis.moveToPoint(30, -58.5, 1700, {.forwards = false}, false); 
    
    // --------------------------------------------------------
    // chassis.moveToPoint(40, -47.15, 800, {}, false); // x 49
    // chassis.turnToHeading(235, 700); // MAGIC
    // chassis.turnToHeading(35, 500, {}, false);

    // // chassis.follow(pathdescore1_txt, 9, 1500, false, true);
    // chassis.follow(pathdescore2_txt, 9, 3000, false, true);
    // chassis.follow(pathdescore3_txt, 9, 2000, false, true);

    // chassis.moveToPoint(38, -59.5, 1000, {.forwards = false}, false);
    // pros::delay(950);
    // descoreLeft.set_value(false);
    // --------------------------------------------------------

    // pros::delay(1900);
    // chassis.moveToPoint((chassis.getPose().x), chassis.getPose().y, 4000, {}, false);
    counterLoader.set_value(false);
    descoreLeft.set_value(true);
}

void leftlongcenter() {
    setStage(3); 
    // chassis.setPose(60.895, -17.116, 270);
    intakeMode = 1;
    chassis.setPose(48.411, -10.375, 247.334);
    chassis.moveToPoint(25.364, -20, 700, {}, true); // t 700 | y 23
    pros::delay(500);
    counterLoader.set_value(true);
    chassis.turnToHeading(135, 650, {}, false);
    chassis.moveToPoint(41.5, -45.5, 800, {}, false);
    chassis.turnToHeading(90, 250, {}, true);
    // chassis.moveToPose(54, -47, 90, 1000, {}, false);
    // pros::delay(300);
    // chassis.moveToPose(60, -48, 90, 1300, {}, false); // t 2000
    chassis.moveToPoint(58, -49, 1000, {}, false);
    // chassis.waitUntilDone();
    pros::delay(400);
    intakeMode = 0;
    chassis.moveToPoint(23, -47.75, 2300, {.forwards = false, .minSpeed = 90}); // xam l
    pros::delay(600);
    intakeMode = 2;
    pros::delay(150);
    setStage(2);
    // pros::delay(200);
    intakeMode = 1;
    pros::delay(500);
    intakeMode = 0;

    chassis.setPose(29.5, -47.15, 90);

    chassis.moveToPoint(47, -47.15, 800, {}, false); // x 49
    chassis.turnToHeading(140, 500, {}, false);

    intakeMode = 0;
    setStage(0);
    chassis.moveToPose(8.5, -8.5, 135, 2000, {.forwards = false}, false);
    intakeMode = 1;


}

// ========= RIGHT DESCORERE 7 BLOCK =========
ASSET(pathdescoreright_txt);
ASSET(pathdescorecbd_txt);
void rightdescore7bloc() {
    setStage(3); 
    // chassis.setPose(60.895, -17.116, 270);
    intakeMode = 1;
    chassis.setPose(48.411, 10.375, 292.667);
    chassis.moveToPoint(25.364, 20, 700, {}, true); // t 700 | y 23
    pros::delay(500);
    counterLoader.set_value(true);
    chassis.turnToHeading(35, 650, {}, false);
    chassis.moveToPoint(41.5, 45.5, 800, {}, false);
    chassis.turnToHeading(90, 250, {}, true);
    // chassis.moveToPose(54, -47, 90, 1000, {}, false);
    // pros::delay(300);
    // chassis.moveToPose(60, -48, 90, 1300, {}, false); // t 2000
    chassis.moveToPoint(57, 48, 1100, {}, false);
    // chassis.waitUntilDone();
    pros::delay(400);
    intakeMode = 0;
    chassis.moveToPoint(23, 47.75, 2300, {.forwards = false, .minSpeed = 90});
    pros::delay(600);
    intakeMode = 2;
    pros::delay(150);
    setStage(2);
    // pros::delay(200);
    intakeMode = 1;
    pros::delay(2500);
    intakeMode = 0;

    chassis.setPose(29.5, 47.15, 90);
    
    chassis.moveToPoint(40, 47.15, 800, {}, false); // x 49
    // chassis.turnToHeading(235, 700); // MAGIC
    chassis.turnToHeading(40, 500, {}, false);
    // chassis.follow(pathdescorecbd_txt, 9, 3000, false, true);
    chassis.moveToPoint(30, 35.5, 1000, {.forwards = false}, false); // x 49
    chassis.turnToHeading(90, 400, {}, true);
    descoreLeft.set_value(false);
    chassis.moveToPoint(12, 35.5, 1000, {.forwards = false}, false); // x 49
    // // chassis.follow(pathdescore1_txt, 9, 1500, false, true);
    // chassis.follow(pathdescoreright_txt, 9, 3000, false, true);
    // chassis.follow(pathdescore3_txt, 9, 2000, false, true);

    // chassis.moveToPoint(38, -59.5, 1000, {.forwards = false}, false);
    // pros::delay(1000);
    // descoreLeft.set_value(false);
    // --------------------------------------------------------
    
    // pros::delay(2000);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y, 6000, {}, false);

}

void rightdescore9bloc() {
    setStage(3); 
    // chassis.setPose(60.895, -17.116, 270);
    intakeMode = 1;
    chassis.setPose(48.411, 10.375, 292.667);
    chassis.moveToPoint(25.364, 20, 700, {}, true); // t 700 | y 23
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
    // intakeMode = 1;
    pros::delay(800);
    setStage(2);
    pros::delay(1800);
    counterLoader.set_value(true);
    intakeMode = 2;
    pros::delay(300);
    setStage(3);
    pros::delay(500);
    // chassis.moveToPose(54, -47, 90, 1000, {}, false);
    // pros::delay(300);
    // chassis.moveToPose(60, -48, 90, 1300, {}, false); // t 2000
    intakeMode = 1;
    chassis.moveToPoint(57, 48, 1600, {}, false);
    // chassis.waitUntilDone();
    pros::delay(200);
    // intakeMode = 0;
    chassis.moveToPoint(23, 47.75, 2300, {.forwards = false, .minSpeed = 90});
    // pros::delay(600);
    // intakeMode = 2;
    // pros::delay(150);
    setStage(2);
    // pros::delay(200);
    intakeMode = 1;
    pros::delay(2500);
    intakeMode = 0;

    chassis.setPose(29.5, 47.15, 90);
    
    // chassis.moveToPoint(40, 47.15, 800, {}, false); // x 49
    // // chassis.turnToHeading(235, 700); // MAGIC
    // chassis.turnToHeading(40, 500, {}, false);
    // // chassis.follow(pathdescorecbd_txt, 9, 3000, false, true);
    // chassis.moveToPoint(30, 35.5, 1000, {.forwards = false}, false); // x 49
    // chassis.turnToHeading(90, 400, {}, true);
    // descoreLeft.set_value(false);
    // chassis.moveToPoint(12, 35.5, 1000, {.forwards = false}, false); // x 49
    // // chassis.follow(pathdescore1_txt, 9, 1500, false, true);
    // chassis.follow(pathdescoreright_txt, 9, 3000, false, true);
    // chassis.follow(pathdescore3_txt, 9, 2000, false, true);

    // chassis.moveToPoint(38, -59.5, 1000, {.forwards = false}, false);
    // pros::delay(1000);
    // descoreLeft.set_value(false);
    // --------------------------------------------------------
    
    // pros::delay(2000);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y, 6000, {}, false);

}


// Hàm tiến thẳng
void goStraight(int speed, int duration) {
    unsigned long startTime = pros::millis();
    while (pros::millis() - startTime < duration) {
        chassis.tank(speed, speed, true);
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
    // pros::delay(50);
}

// Hàm lùi
void reverse(int speed, int duration) {
    unsigned long startTime = pros::millis();
    while (pros::millis() - startTime < duration) {
        chassis.tank(speed, speed, true);
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
    // pros::delay(50);
}

// Hàm lắc qua lại
void wiggle(int speed, int duration) {
    unsigned long startTime = pros::millis();
    bool wiggleLeft = true;
    while (pros::millis() - startTime < duration) {
        if (wiggleLeft) {
            chassis.tank(-speed, speed, true); // Quay trái
        } else {
            chassis.tank(speed, -speed, true); // Quay phải
        }
        wiggleLeft = !wiggleLeft;
        pros::delay(150); // Đổi hướng mỗi 100ms
    }
    chassis.tank(0, 0, true);
    // pros::delay(50); 
}


void skillz() {
    setStage(3); 
    chassis.setPose(-45, 0, 270);
    intakeMode = 1;

    // code chay qua park 
    // ------------------------------------------------------------------------------
    // odoLift.set_value(false);
    // counterLoader.set_value(true);
    
    // Go straight (forward)
    // goStraight(100, 800);
    
    // Wiggle
    // wiggle(50, 1500);

    // Tiến lại
    // goStraight(80, 800);

    // Lắc lại
    // wiggle(50, 1500);

    // odoLift.set_value(true);
    // counterLoader.set_value(true);
    // Reverse
    // reverse(-70, 1000);
    
    // counterLoader.set_value(false);
    // odoLift.set_value(true);
    // pros::delay(500);
    // chassis.turnToHeading(90, 1500, {}, false);
    // pros::delay(500);
    // counterLoader.set_value(false);
    // goStraight(20, 2000);
    odoLift.set_value(true);
    // pros::delay(1500);
    chassis.setPose(-45, 0, 270);
    // chassis.moveToPoint(-30, 0, 1500, {.forwards = false}, false);
    // brakeChassis();  // Tương đương chassis.tank(0, 0, true)
    // chassis.turnToHeading(0, 1000, {}, false);
    
    // intakeMode = 1;
    // chassis.moveToPoint(-30, 21, 1500, {.maxSpeed = 60}, false);
    // intakeMode = 0;
    // setStage(0);
    // chassis.moveToPose(-7.5, 7.5, -45, 2000, {.forwards = false}, false);
    // intakeMode = 1;

    chassis.moveToPoint(-41, 0, 1000, {.forwards = false}, true);
    // pros::delay(500);
    chassis.turnToHeading(0, 1000, {}, true);
    // pros::delay(500);
    // wallReset();
    // pros::delay(510);
    chassis.moveToPoint(-41, 43, 2000, {}, false);
    // pros::delay(2000);
    chassis.turnToHeading(135, 1000, {}, true);
    // intakeMode = 1;
    chassis.moveToPoint(-30, 30, 1000, {}, true);
    pros::delay(950);
    intakeMode = 0;
    // chassis.moveToPoint(-29, 29, 1000, {}, true);
    chassis.turnToHeading(315, 1000, {}, false);
    setStage(0);
    chassis.moveToPose(-7.6, 7.6, -45, 2000, {.forwards = false}, false);
    counterLoader.set_value(true);
    pros::delay(100);
    intakeMode = 2;
    pros::delay(200);
    intakeMode = 1;

    pros::delay(3500);


    setStage(3);
    counterLoader.set_value(true);
    chassis.moveToPoint(-45, 45.5, 1200, {}, false);
    chassis.turnToHeading(270, 500, {}, false);

    // chassis.moveToPoint(-59.35, 47, 2000, {.maxSpeed = 70}, false);
    chassis.moveToPose(-59.35, 47, 270, 2000, {}, false);
    // chassis.waitUntilDone();
    pros::delay(1000);
    intakeMode = 0;



    // ski
    chassis.moveToPoint(-45, 47, 1200, {.forwards = false}, false);
    counterLoader.set_value(false);
    // chassis.turnToHeading(220, 500, {}, false);
    // chassis.swingToHeading(240, DriveSide::LEFT, 1000);
    chassis.turnToHeading(235, 1000, {}, false);
    chassis.moveToPoint(-29, 59.54, 1000, {.forwards = false}, false);
    chassis.swingToHeading(275, DriveSide::LEFT, 1000);

    chassis.moveToPoint(45, 54.75 ,4000, {.forwards = false, .maxSpeed = 100}, false);
    chassis.moveToPose(25, 47, 90, 2300, {.forwards = false});
    intakeMode = 1;
}

void testodo11() {
    // chassis.setPose(48.411, -speed10.375, 247.334);
    // chassis.setPose(45.5, 0, 270);
    // chassis.setPose(0, 0, 270);
    // chassis.moveToPose(0, 10, 270, 4000);
    // chassis.turnToHeading(215, 700);
    // pros::delay(10000);
    // chassis.turnToHeading(90, 700);
    chassis.setPose(0, 0, 270);
    chassis.moveToPoint((chassis.getPose().x), chassis.getPose().y, 15000, {}, false);
}


void autonomous() {
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(90, 100000);
    // chassis.moveToPoint(0, 10, 5000);
    // leftdescore7bloc();
    // leftfastdescore();
    // leftlongcenter();
    leftcenterdescore();
    // rightdescore7bloc();
    // rightdescore9bloc();
    // testodo11();
    // skillz();
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    // pros::delay(500); // give time for sensors to boot
    chassis.calibrate();
    
    // Set brake mode for drivetrain
    // Options:
    // - pros::E_MOTOR_BRAKE_COAST: Motors coast when stopped (no resistance)
    // - pros::E_MOTOR_BRAKE_BRAKE: Motors brake when stopped (some resistance)
    // - pros::E_MOTOR_BRAKE_HOLD: Motors hold position when stopped (strongest, best for precision)
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); // Recommended for autonomous
    // chassis.setPose(45.5,0,270);
    // colorSensor.set_integration_time(25);
    // colorSensor.set_led_pwm(100);
    // pros::Task sorter(colorSortTask);

    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta); // heading from odometry
            pros::lcd::print(7, "IMU: %.2f", imu.get_heading()); // IMU heading directly
            
            // Battery stage
            double batteryLevel = pros::battery::get_capacity();
            pros::lcd::print(3, "Battery: %.1f%%", batteryLevel);
            
            // Motor temperatures - lấy nhiệt độ từ các motor đại diện
            pros::Motor tempMotor1(3); // Motor từ leftMotors (port 3)
            pros::Motor tempMotor2(7); // Motor từ rightMotors (port 7)
            double tempLeft = tempMotor1.get_temperature();
            double tempRight = tempMotor2.get_temperature();
            double tempAbove = motorTrai.get_temperature();
            double tempUnder = motorPhai.get_temperature();
            
            // Hiển thị nhiệt độ motor
            pros::lcd::print(4, "Motor L: %.1fC R: %.1fC", tempLeft, tempRight);
            pros::lcd::print(5, "Motor Up: %.1fC Dn: %.1fC", tempAbove, tempUnder);
            
            // Tìm nhiệt độ cao nhất để cảnh báo
            double maxTemp = std::max({tempLeft, tempRight, tempAbove, tempUnder});
            if (maxTemp > 50) {
                pros::lcd::print(6, "WARNING: Max %.1fC!", maxTemp);
            } else {
                pros::lcd::print(6, "Max Temp: %.1fC", maxTemp);
            }
            
            pros::delay(200); // Refresh every 2 seconds
        }
    });

    // controller.clear();
    // controller.set_text(0, 0, "AUTON SELECT");

    // while (pros::competition::is_disabled()) {
    //     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    //         current_auton = (current_auton == 0 ? 6 : current_auton - 1);
    //     }
    //     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    //         current_auton = (current_auton == 6 ? 0 : current_auton + 1);
    //     }

    //     controller.set_text(1, 0, autonNames[current_auton]);
    //     pros::delay(120);
    // }
}



// ========== OP CONTROL ==========
void opcontrol() {
    // Đảm bảo đèn cảm biến bật
    // colorSensor.set_led_pwm(100);
    // chassis.setPose(60.895, -17.116, 270);
    // Set IMU heading để đồng bộ với pose
    // imu.set_heading(270);
    // pros::delay(50); // Delay nhỏ để IMU cập nhật

    int displayCounter = 0; // Counter để update controller display không quá nhanh

    while (true) {
        handleDrive();
        handleManualPneumatics();

        // --- Intake toggle (L1 = hút, L2 = nhả) ---
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (intakeMode == 1) {
                intakeMode = 0; // Đang hút -> Tắt
            } else {
                intakeMode = 1; // Đang tắt/nhả -> Bật hút
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (intakeMode == 2) {
                intakeMode = 0; // Đang nhả -> Tắt
            } else {
                intakeMode = 2; // Đang tắt/hút -> Bật nhả
            }
        }
        
        // Update controller display mỗi 100ms (mỗi 4 lần loop với delay 25ms)
        // Controller text update rate chậm, không nên update quá nhanh
        // if (displayCounter % 4 == 0) {
        //     if (isSkillMode) {
        //         // Skill mode: hiển thị boost mode status
        //         if (boostMode) {
        //             controller.print(0, 0, "BOOST: ON");
        //         } else {
        //             controller.print(0, 0, "BOOST: OFF");
        //         }
        //     } else {
        //         // Match mode: clear hoặc hiển thị mode
        //         controller.print(0, 0, "MATCH MODE");
        //     }
        // }
        // displayCounter++;
        
        pros::delay(25);
    }
}