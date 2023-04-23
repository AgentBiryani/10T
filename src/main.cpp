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

void competition_initialize() {
    imu_sensor.reset();
}

void autonomous() {
//Skills
    if(selector::auton == 0 /* Skills */){


    }

//Close
    if(selector::auton == 1 /* Nothing */){ 

        //jerk();
        turnPID(25);
        //turnPID(0);
    }
    if(selector::auton == 2 /* Half */){
        turnPID(-9);
        pros::delay(250);
        shoot();
        roll(-4000);
        pros::delay(130);
        movePID(6, 6, 1000);
        pros::delay(100);
        movePID(-6, -6, 1000);
        stopRoll();
        pros::delay(100);
        turnPID(-133.5);
        pros::delay(250);
        movePID(37, 37, 1000, 0.6);
        pros::delay(100);
        roll(12000);
        movePID(40, 40, 1000, 0.4);
        pros::delay(500);
        turnPID(-40);
        pros::delay(100);
        movePID(-3, -3, 1000);
        pros::delay(100);
        shoot();


    }
    if(selector::auton == 3 /* Roller */){
        turnPID(-9);
        pros::delay(250);
        shoot();
        roll(-4000);
        pros::delay(130);
        movePID(6, 6, 1000);
        pros::delay(100);
        movePID(-6, -6, 1000);
        stopRoll();
        pros::delay(100);
        turnPID(-90);
        pros::delay(250);
        movePID(68, 68, 2000, 0.6);
        turnPID(-180);

    }


//Far
    if(selector::auton == -1 /* Nothing */){

    }
    if(selector::auton == -2 /* Half */){
        movePID(28, 28, 300);
        pros::delay(300);
        turnPID(80);
        roll(-4500);
        movePID(10, 10, 750);
        pros::delay(300);
        movePID(-7.5, -7.5, 1000);
        stopRoll();
        pros::delay(250);
        turnPID(99);
        pros::delay(100);
        shoot();
        cataReset();
        turnPID(224);
        pros::delay(300);
        roll(12000);
        movePID(90, 90, 1750, .5);
        pros::delay(750);
        jerk();
        pros::delay(300);
        turnPID(135);
        pros::delay(300);
        stopRoll();
        movePID(7.5, 7.5, 950, .5);
        shoot();


    }
    if(selector::auton == -3 /* Roller */){
        movePID(28, 28, 300);
        pros::delay(300);
        turnPID(80);
        roll(-4500);
        movePID(10, 10, 750);
        pros::delay(300);
        movePID(-7.5, -7.5, 1000);
        stopRoll();
        pros::delay(250);
        turnPID(99);
        pros::delay(100);
        shoot();
        cataReset();
        turnPID(180);
        pros::delay(300);
        movePID(60, 60, 1250);
        turnPID(270);
        
    }

}

void opcontrol() {
    opcontr();
}

