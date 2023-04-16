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
//Do Nothing
    if(selector::auton == 0 /* Nothing */){

    }

//Close
    if(selector::auton == 1 /* Full */){    
    }
    if(selector::auton == 2 /* Half */){
        t->setMaxVelocity(125);     
        t->turnAngle(-9_deg);
        t->waitUntilSettled();
        pros::delay(500);
        shoot();
        pros::delay(500);
        t->moveDistance(4_in);
        t->waitUntilSettled();
        pros::delay(500);
        roll(-4500);
        pros::delay(750);
        t->moveDistance(-6_in);
        t->waitUntilSettled();
        pros::delay(500);
        t->turnAngle(-122_deg);
        t->waitUntilSettled();
        t->setMaxVelocity(250);
        pros::delay(500);
        t->moveDistance(35_in);
        pros::delay(500);
        roll(12000);
        pros::delay(500);
        t->setMaxVelocity(125);
        t->moveDistance(30_in);
        t->waitUntilSettled();
        pros::delay(500);
        t->turnAngle(88_deg);
        pros::delay(500);
        shoot();
    }
    if(selector::auton == 3 /* Roller */){
        t->setMaxVelocity(125);     
        t->turnAngle(-9_deg);
        t->waitUntilSettled();
        pros::delay(500);
        shoot();
        pros::delay(500);
        t->moveDistance(4_in);
        t->waitUntilSettled();
        pros::delay(500);
        roll(-4500);
        pros::delay(750);
        t->moveDistance(-6_in);
        pros::delay(500);
        stopRoll();
        t->turnAngle(-82_deg);
        t->waitUntilSettled();
        t->moveDistance(67_in);
        t->waitUntilSettled();
        pros::delay(500);
        t->turnAngle(-86_deg);


    }


//Far
    if(selector::auton == -1 /* Full */){
    }
    if(selector::auton == -2 /* Half */){
    }
    if(selector::auton == -3 /* Roller */){
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

