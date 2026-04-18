#include "auton_selector.h"
#include "monte.h"
#include "pros/apix.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include <string>
#include <cmath>
#include <cstdio>

// Initialize static instance pointer
AutonSelector* AutonSelector::instance = nullptr;

AutonSelector::AutonSelector() 
    : selected_color(Color::RED), selected_auton_index(0),
      screen(nullptr), main_label(nullptr), prev_btn(nullptr), 
      next_btn(nullptr), color_toggle_btn(nullptr),
      pose_label(nullptr), temp_label(nullptr) {
    instance = this;
    
    // Create routine list
    all_routines = {
        {"leftdescore7bloc", &leftdescore7bloc},
        {"leftlongcenter2", &leftlongcenter2},
        {"rightdescore7bloc", &rightdescore7bloc},
        {"rightcenterdescore", &rightcenterdescore},
        {"rightlower", &rightlower},
        {"awp", &awp},
        {"Skillz", &skillz},
        {"Test Odo", &testodo11},
        {"PID Drive", &pidTuneDrive},
        {"PID Turn", &pidTuneTurn},
        {"PID Velocity", &pidTuneVelocity}
    };
}

void AutonSelector::initialize() {
    // Create screen
    screen = lv_obj_create(NULL);
    lv_screen_load(screen);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
    
    // Create main label (shows current auton name)
    main_label = lv_label_create(screen);
    lv_obj_set_style_text_color(main_label, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(main_label, &lv_font_montserrat_24, 0);
    lv_obj_align(main_label, LV_ALIGN_CENTER, 0, -30);
    
    // Create navigation buttons
    prev_btn = lv_button_create(screen);
    lv_obj_set_size(prev_btn, 60, 200);
    lv_obj_align(prev_btn, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_add_event_cb(prev_btn, prev_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_opa(prev_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(prev_btn, LV_OPA_TRANSP, 0);
    lv_obj_t *prev_label = lv_label_create(prev_btn);
    lv_label_set_text(prev_label, "<");
    lv_obj_set_style_text_font(prev_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(prev_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(prev_label);
    
    next_btn = lv_button_create(screen);
    lv_obj_set_size(next_btn, 60, 200);
    lv_obj_align(next_btn, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_event_cb(next_btn, next_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_opa(next_btn, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(next_btn, LV_OPA_TRANSP, 0);
    lv_obj_t *next_label = lv_label_create(next_btn);
    lv_label_set_text(next_label, ">");
    lv_obj_set_style_text_font(next_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(next_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(next_label);
    
    // Create color toggle button
    color_toggle_btn = lv_button_create(screen);
    lv_obj_set_size(color_toggle_btn, 150, 50);
    lv_obj_align(color_toggle_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_add_event_cb(color_toggle_btn, color_toggle_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *toggle_label = lv_label_create(color_toggle_btn);
    lv_label_set_text(toggle_label, "RED");
    lv_obj_center(toggle_label);
    
    // Create telemetry labels (hidden initially)
    pose_label = lv_label_create(screen);
    lv_obj_set_style_text_color(pose_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(pose_label, &lv_font_montserrat_20, 0);
    lv_obj_align(pose_label, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_add_flag(pose_label, LV_OBJ_FLAG_HIDDEN);
    
    temp_label = lv_label_create(screen);
    lv_obj_set_style_text_color(temp_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(temp_label, &lv_font_montserrat_20, 0);
    lv_obj_align(temp_label, LV_ALIGN_TOP_LEFT, 10, 50);
    lv_obj_add_flag(temp_label, LV_OBJ_FLAG_HIDDEN);
    
    
    
    // Show initial display
    update_display();
}

void AutonSelector::update() {
    lv_timer_handler();
    pros::delay(5);
}

void AutonSelector::update_display() {
    if (all_routines.empty()) return;
    
    // Reset index if out of bounds
    if (selected_auton_index >= (int)all_routines.size()) {
        selected_auton_index = 0;
    }
    if (selected_auton_index < 0) {
        selected_auton_index = all_routines.size() - 1;
    }
    
    // Update main label
    lv_label_set_text(main_label, all_routines[selected_auton_index].name.c_str());
    
    // Update color toggle button
    const char* color_text = (selected_color == Color::RED) ? "RED" : "BLUE";
    lv_obj_t *toggle_label = lv_obj_get_child(color_toggle_btn, 0);
    if (toggle_label) {
        lv_label_set_text(toggle_label, color_text);
    }
    lv_obj_set_style_bg_color(color_toggle_btn, 
        (selected_color == Color::RED) ? lv_color_hex(0xFF0000) : lv_color_hex(0x0000FF), 0);
}

void AutonSelector::toggle_color() {
    selected_color = (selected_color == Color::RED) ? Color::BLUE : Color::RED;
    update_display();
}

void AutonSelector::next_auton(bool wrap_around) {
    if (all_routines.empty()) return;
    
    selected_auton_index++;
    if (selected_auton_index >= (int)all_routines.size()) {
        if (wrap_around) {
            selected_auton_index = 0;
        } else {
            selected_auton_index = all_routines.size() - 1;
        }
    }
    update_display();
}

void AutonSelector::prev_auton(bool wrap_around) {
    if (all_routines.empty()) return;
    
    selected_auton_index--;
    if (selected_auton_index < 0) {
        if (wrap_around) {
            selected_auton_index = all_routines.size() - 1;
        } else {
            selected_auton_index = 0;
        }
    }
    update_display();
}

void AutonSelector::run_auton() {
    if (selected_auton_index >= 0 && selected_auton_index < (int)all_routines.size()) {
        all_routines[selected_auton_index].action();
    }
}

std::string AutonSelector::get_selected_name() const {
    if (selected_auton_index >= 0 && selected_auton_index < (int)all_routines.size()) {
        return all_routines[selected_auton_index].name;
    }
    return "None";
}

void AutonSelector::show_telemetry() {
    // Hide selector UI
    if (main_label) {
        lv_obj_add_flag(main_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (prev_btn) {
        lv_obj_add_flag(prev_btn, LV_OBJ_FLAG_HIDDEN);
    }
    if (next_btn) {
        lv_obj_add_flag(next_btn, LV_OBJ_FLAG_HIDDEN);
    }
    if (color_toggle_btn) {
        lv_obj_add_flag(color_toggle_btn, LV_OBJ_FLAG_HIDDEN);
    }
    
    // Show only pose and temp labels
    if (pose_label) {
        lv_obj_remove_flag(pose_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (temp_label) {
        lv_obj_remove_flag(temp_label, LV_OBJ_FLAG_HIDDEN);
    }
    
    update_telemetry();
}

void AutonSelector::update_telemetry() {
    extern lemlib::Chassis chassis;
    extern pros::Motor intake;
    extern pros::Motor conveyor;
    extern pros::Motor outtake;
    extern pros::MotorGroup leftMotors;
    extern pros::MotorGroup rightMotors;
    
    // Update pose only
    if (pose_label && !lv_obj_has_flag(pose_label, LV_OBJ_FLAG_HIDDEN)) {
        lemlib::Pose pose = chassis.getPose();
        char buffer[80];
        snprintf(buffer, sizeof(buffer), "Pose: X:%.2f Y:%.2f T:%.2f", 
                 pose.x, pose.y, pose.theta);
        lv_label_set_text(pose_label, buffer);
    }    
    
    
    if (temp_label && !lv_obj_has_flag(temp_label, LV_OBJ_FLAG_HIDDEN)) {
        static uint32_t last_temp_update = 0;
        if (pros::millis() - last_temp_update >= 1000 || last_temp_update == 0) {
            last_temp_update = pros::millis();
            
            double tempLeft = leftMotors.get_temperature();
            double tempRight = rightMotors.get_temperature();
            double tempIntake = intake.get_temperature();
            double tempConveyor = conveyor.get_temperature();
            double tempOuttake = outtake.get_temperature();
            double maxTemp = std::max({tempLeft, tempRight, tempIntake, tempConveyor, tempOuttake});
            
            char buffer[256];
            if (maxTemp > 50) {
                snprintf(buffer, sizeof(buffer), "Drivetrain: L:%.1f R:%.1f\nIntake: I:%.1f C:%.1f O:%.1f\nMAX: %.1f!", 
                         tempLeft, tempRight, tempIntake, tempConveyor, tempOuttake, maxTemp);
                lv_obj_set_style_text_color(temp_label, lv_color_hex(0xFF0000), 0);
            } else {
                snprintf(buffer, sizeof(buffer), "Drivetrain: L:%.1f R:%.1f\nIntake: I:%.1f C:%.1f O:%.1f", 
                         tempLeft, tempRight, tempIntake, tempConveyor, tempOuttake);
                lv_obj_set_style_text_color(temp_label, lv_color_hex(0xFFFFFF), 0);
            }
            lv_label_set_text(temp_label, buffer);
        }
    }
}

// Callback functions
void AutonSelector::color_toggle_btn_cb(lv_event_t *event) {
    if (instance) {
        instance->toggle_color();
    }
}

void AutonSelector::prev_btn_cb(lv_event_t *event) {
    if (instance) {
        instance->prev_auton();
    }
}

void AutonSelector::next_btn_cb(lv_event_t *event) {
    if (instance) {
        instance->next_auton();
    }
}
