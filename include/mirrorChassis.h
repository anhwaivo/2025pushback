#pragma once
#include "main.h"
#include <cmath>

// ============================
// 🔧 MIRROR CHASSIS WRAPPERS
// ============================
//
// mode = 1 → bình thường
// mode = 2 → đối xứng qua trục X (y → -y, heading → 360.0 - heading)
// 
// Bạn có thể dùng tất cả hàm như:
// moveToPoint(x, y, 3000, 2); // tự đối xứng theo trục X
//

// ---- Hàm tiện ích lật toạ độ ----
inline void mirrorCoord(double &x, double &y, double &heading, int mode) {
    if (mode == 2) {
        y = -y;                          // lật qua trục X
        heading = fmod(360.0 - heading, 360.0); // đổi hướng đối xứng
    }
}

// ---- TURN TO HEADING ----
inline void turnToHeading(double heading, int timeout, int mode = 1,
                          lemlib::MoveOptions opts = {}, bool async = false) {
    double h = heading;
    if (mode == 2) h = fmod(360.0 - h, 360.0);
    chassis.turnToHeading(h, timeout, opts, async);
}

// ---- TURN TO POINT ----
inline void turnToPoint(double x, double y, int timeout, int mode = 1,
                        lemlib::MoveOptions opts = {}, bool async = false) {
    double mx = x, my = y;
    if (mode == 2) my = -my;
    chassis.turnToPoint(mx, my, timeout, opts, async);
}

// ---- SWING TO HEADING ----
inline void swingToHeading(double heading, int timeout, int mode = 1,
                           lemlib::MoveOptions opts = {}, bool async = false) {
    double h = heading;
    if (mode == 2) h = fmod(360.0 - h, 360.0);
    chassis.swingToHeading(h, timeout, opts, async);
}

// ---- SWING TO POINT ----
inline void swingToPoint(double x, double y, int timeout, int mode = 1,
                         lemlib::MoveOptions opts = {}, bool async = false) {
    double mx = x, my = y;
    if (mode == 2) my = -my;
    chassis.swingToPoint(mx, my, timeout, opts, async);
}

// ---- MOVE TO POINT ----
inline void moveToPoint(double x, double y, int timeout, int mode = 1,
                        lemlib::MoveOptions opts = {}, bool async = false) {
    double mx = x, my = y;
    if (mode == 2) my = -my;
    chassis.moveToPoint(mx, my, timeout, opts, async);
}

// ---- MOVE TO POSE ----
inline void moveToPose(double x, double y, double heading, int timeout, int mode = 1,
                       lemlib::MoveOptions opts = {}, bool async = false) {
    double mx = x, my = y, mh = heading;
    if (mode == 2) {
        my = -my;
        mh = fmod(360.0 - mh, 360.0);
    }
    chassis.moveToPose(mx, my, mh, timeout, opts, async);
}
