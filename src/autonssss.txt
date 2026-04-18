// ========= LEFT DESCORERE 7 BLOCK =========
ASSET(pathdescore1_txt);
ASSET(pathdescore2_txt);
ASSET(pathdescore3_txt);
void leftdescore7bloc() {
    setStage(3); 
    // chassis.setPose(60.895, -17.116, 270);
    intakeMode = 1;
    chassis.setPose(49.2, -10, 247.334);
    chassis.moveToPoint(25.2, -20, 700, {}, true); // t 700 | y 23
    pros::delay(500);
    counterLoader.set_value(true);
    chassis.turnToHeading(135, 550, {}, false);
    chassis.moveToPoint(41.5, -47.75 , 800, {}, false);
    chassis.turnToHeading(90, 500, {}, true);
    // chassis.moveToPose(54, -47, 90, 1000, {}, false);
    // pros::delay(300);
    // chassis.moveToPose(60, -48, 90, 1300, {}, false); // t 2000
    chassis.moveToPoint(60, -48.5, 1500, {}, false);
    // chassis.turnToHeading(90, 250, {}, true); // mhm
    // chassis.waitUntilDone();
    pros::delay(400);
    intakeMode = 0;
    chassis.moveToPoint(23, -48.25, 2300, {.forwards = false, .minSpeed = 100}); // xam l
    pros::delay(600);
    intakeMode = 2;
    pros::delay(200);
    setStage(2);
    // pros::delay(200);
    intakeMode = 1;
    pros::delay(2400);
    intakeMode = 0;

    chassis.setPose(29.25, -48, 90);
    counterLoader.set_value(false);

    // chassis.moveToPose(40, -47.5, 91, 2000, {}, false); 
    // chassis.moveToPoint((chassis.getPose().x - 5), chassis`.getPose().y, 1800, {}, false);
    // chassis.moveToPoint(38.5, -47.15, 1500, {}, false);
    // chassis.turnToHeading(0, 900, {}, false);
    // chassis.moveToPoint(38.5, -58.5, 1700, {.forwards = false}, false); 
    // chassis.turnToHeading(90, 900, {}, false);
    // chassis.moveToPoint(30, -58.5, 1700, {.forwards = false}, false); 
    
    // --------------------------------------------------------
    chassis.moveToPoint(40, -48, 600, {}, false); // x 49
    // chassis.turnToHeading(235, 700); // MAGIC
    chassis.turnToHeading(35, 300, {}, false);

    // // chassis.follow(pathdescore1_txt, 9, 1500, false, true);
    chassis.follow(pathdescore2_txt, 9, 3500, false, true);
    // chassis.follow(pathdescore3_txt, 9, 2000, false, true);

    // chassis.moveToPoint(38, -59.5, 1000, {.forwards = false}, false);
    pros::delay(600);
    descoreLeft.set_value(false);
    // --------------------------------------------------------
    
    pros::delay(2000);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y, 6000, {.minSpeed = 120}, false);
   
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
    chassis.setPose(49.2, -10, 247.334);
    chassis.moveToPoint(25.2, -20, 700, {}, true);
    // chassis.moveToPose(25.364, -20, 1500, {}, true); // t 700 | y 23
    pros::delay(500);
    counterLoader.set_value(true);
    // // chassis.turnToHeading(135, 650, {}, false);
    pros::delay(600);
    counterLoader.set_value(false);

    // --------------------------------------------------------
    chassis.moveToPose(7, -43, 206, 1700, {}, false); // an long goal
    // chassis.moveToPose(9, -40, 206, 1600, {}, false);
    // chassis.moveToPoint(23, -25, 1200, {.forwards = false}, false);
    chassis.moveToPoint(25.2, -20, 1500, {.forwards = false}, false);
    intakeMode = 0;
    setStage(0);
    chassis.moveToPose(7.5, -9, 135, 2000, {.forwards = false}, false);
    intakeMode = 1;
    pros::delay(1200);

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
    chassis.moveToPoint(67, -48, 1200, {}, false);
    // chassis.waitUntilDone();
    pros::delay(400);
    intakeMode = 0;
    chassis.moveToPoint(23, -48, 2300, {.forwards = false, .minSpeed = 100}); // xam l
    pros::delay(600);
    intakeMode = 2;
    pros::delay(150);
    setStage(2);
    // pros::delay(200);
    intakeMode = 1;
    pros::delay(2000);
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
    chassis.moveToPoint(41.5, 47.5, 800, {}, false);
    chassis.turnToHeading(90, 250, {}, true);
    // chassis.moveToPose(54, -47, 90, 1000, {}, false);
    // pros::delay(300);
    // chassis.moveToPose(60, -48, 90, 1300, {}, false); // t 2000
    chassis.moveToPoint(59, 48, 1100, {.minSpeed = 90}, false);

    chassis.turnToHeading(90, 250, {}, true); // mhm

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

    chassis.setPose(29.25, 47.15, 90);
    
    chassis.moveToPoint(40, 47.15, 800, {}, false); // x 49
    // chassis.turnToHeading(235, 700); // MAGIC
    chassis.turnToHeading(40, 900, {}, false);
    // chassis.follow(pathdescorecbd_txt, 9, 3000, false, true);
    chassis.moveToPoint(30, 34.5, 1500, {.forwards = false}, false); // x 49
    chassis.turnToHeading(90, 400, {}, true);
    descoreLeft.set_value(false);
    chassis.moveToPoint(12, 34.5, 1000, {.forwards = false}, false); // x 49
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

    chassis.setPose(29.25, 47.15, 90);
    
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
    chassis.setPose(0, 0, 0);
    // chassis.moveToPose(-24, 0, 270, 9000);
    // chassis.turnToHeading(90, 7000);
    // pros::delay(10000);
    // chassis.turnToHeading(90, 700);
    // chassis.setPose(0, 0, 270);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y, 15000, {}, false);
}