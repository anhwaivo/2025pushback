#pragma once
#include "lemlib/api.hpp"

// Declare all auton functions
void blue_right_wp();
void blue_right_elim();
void blue_right_solo_wp();

void blue_left_wp();
void blue_left_elim();
void blue_left_solo_wp();

void red_right_wp();
void red_right_elim();
void red_right_solo_wp();

void red_left_wp();
void red_left_elim();
void red_left_solo_wp();

void autoSkill();


// Function to run selected auton
void runAuton(int selection);
