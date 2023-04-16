#include "main.h"
#include "okapi/api.hpp"
#include "selection.cpp"
#include "aerodynamiclunges/MAINCODE.cpp"

lv_obj_t * myLabel;

using namespace okapi;
using namespace std;



void initialize() {
    imu_sensor.reset();
    selector::init();
    expansion.set_value(false);
    boost.set_value(false);

}

void disabled() {}

void competition_initialize() {}

void autonomous() {

    if(selector::auton == 1){         
        turnPID(180);
        pros::delay(1000);
        turnPID(0);

    }


    
    if(selector::auton == -1){ 
        t->setMaxVelocity(125);
        t->isSettled();
        t->moveDistance(28_in);
        t->isSettled();
        roll(-4500);
        t->turnAngle(90_deg);
        t->isSettled();
        t->moveDistance(6_in);
        t->isSettled();
        pros::delay(300);
        stopRoll();
        t->moveDistance(-6_in);
        t->isSettled();
        t->turnAngle(-90_deg);
        t->isSettled();
        t->moveDistance(-72_in);
        t->isSettled();
        t->turnAngle(-87_deg);
        t->isSettled();
    }
}

void opcontrol() {
    opcontr();
}

