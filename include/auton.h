#pragma once

// Autonomous routine declarations
void leftdescore7bloc();
void leftfastdescore();
void leftcenterdescore();
// void leftlongcenter();
void leftlongcenter2();
void awp();
void rightdescore7bloc();
void rightcenterdescore();
void rightdescore9bloc();
void rightlower();
void skillz();
void chutrinhtrai();
void quapark2();
void chutrinhphai();
void testodo11();

// PID Tuner routines (output to PROS terminal for Desmos)
void pidTuneDrive();     // Drive straight step response
void pidTuneTurn();      // Turn in place step response
void pidTuneVelocity();  // Velocity vs voltage characterization

// Helper functions
void goStraight(int speed, int duration);
void reverse(int speed, int duration);
void wiggle(int speed, int duration);

// Reset pose (distance sensors) – gọi từ main khi cần
bool resetposeskilly(int val = 1);
bool resetposeskillyEast(int val = -1);
bool resetposeskillxNorth(int val);
bool resetposeskillxSouth(int val);
