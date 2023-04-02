#include "main.h"
#include "okapi/api.hpp"

using namespace okapi;
using namespace std;

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    // Creating controller object
    Controller contr;

    // Creating motor objects for intake and catapult
    Motor Intake(8, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    Motor Catapult(16, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

    // Creating object for rotation sensor - gets position of catapult motor
    RotationSensor rotation(18);
    
    // Defining buttons for controller actions
    ControllerButton intake(ControllerDigital::L1);
    ControllerButton outtake(ControllerDigital::L2);
    ControllerButton Shoot(ControllerDigital::R1);
    ControllerButton Speed(ControllerDigital::R2);

    // Defining motor ports for the left and right sides of the robot
    const int LEFT_FRONT_PORT = 12;
    const int LEFT_MIDDLE_PORT = 13;
    const int LEFT_REAR_PORT = 14;
    const int RIGHT_FRONT_PORT = 1;
    const int RIGHT_MIDDLE_PORT = 3;
    const int RIGHT_REAR_PORT = 5;

    // Defining motor objects for the left and right sides of the robot
    Motor leftFront(LEFT_FRONT_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    Motor leftMiddle(LEFT_MIDDLE_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    Motor leftRear(LEFT_REAR_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    Motor rightFront(RIGHT_FRONT_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    Motor rightMiddle(RIGHT_MIDDLE_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    Motor rightRear(RIGHT_REAR_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

    // Defining controller object
    Controller controller;

    // Defining constants for acceleration and deacceleration rates
    const double ACCELERATION_RATE = 0.55; // increase in power per millisecond
    const double DEACCELERATION_RATE = -2.5; // decrease in power per millisecond
    
    // Variables for catapult
    double cpost = rotation.get();
    float prime = 71;
    float cataspeed;

    // Variables for drive
    double targetPower = 550.0;
    double currentPower;
    double turnP;


    while (true){                
        // Get joystick inputs
        double forwardPower = controller.getAnalog(ControllerAnalog::rightX);
        double turnPower = controller.getAnalog(ControllerAnalog::leftY);
        if (controller.getAnalog(ControllerAnalog::rightX) < 0) {forwardPower *= -1;}
        if (controller.getAnalog(ControllerAnalog::leftY) < 0) {turnPower *= -1;}

        double cpost = rotation.get();
        double leftPower = (forwardPower * turnP) + turnPower;
        double rightPower = (forwardPower * turnP) - turnPower;
        Catapult.setBrakeMode(AbstractMotor::brakeMode::coast);


		forwardPower *= forwardPower;
		turnPower *= turnPower;
        // Limits the power to -5.0 to 5.0
        leftPower = std::clamp(leftPower, -5.0, 5.0);
        rightPower = std::clamp(rightPower, -5.0, 5.0);

        // Sets the power of each motor
        leftFront.moveVelocity(leftPower * currentPower);
        leftMiddle.moveVelocity(leftPower * currentPower);
        leftRear.moveVelocity(leftPower * currentPower);
        rightFront.moveVelocity(rightPower * currentPower);
        rightMiddle.moveVelocity(rightPower * currentPower);
        rightRear.moveVelocity(rightPower * currentPower);
		
		// Gradually accelerates or deaccelerates each motor to the target power
		if (currentPower < targetPower) {
            currentPower += ACCELERATION_RATE * pros::c::millis();
            currentPower = std::min(currentPower, targetPower);
        } else if (currentPower > targetPower) {
            currentPower += DEACCELERATION_RATE * pros::c::millis();
            currentPower = std::max(currentPower, targetPower);
        }

        // Turns on intake
        if (intake.isPressed()){
            Intake.moveVoltage(12000);
        } else if (outtake.isPressed()) {
        // Reverses intake
            Intake.moveVoltage(-12000);
        } else {
        // Brakes intake - coast brake mode
            Intake.moveVoltage(0);
        }

        if (prime - cpost < 10) {
            cataspeed = 35;
        } 
        else{
            cataspeed = 100;
        }
 
        if (cpost < prime || cpost > 300){
            Catapult.moveVelocity(cataspeed);
        } else if (Shoot.isPressed()) { 
            Catapult.moveVoltage(12000);
        } 
        else {
            Catapult.moveVoltage(0);
        }

        if (Speed.isPressed()){
            turnP = .075;
            
            leftFront.setBrakeMode(AbstractMotor::brakeMode::brake);
            leftMiddle.setBrakeMode(AbstractMotor::brakeMode::brake);
            leftRear.setBrakeMode(AbstractMotor::brakeMode::brake);
            rightFront.setBrakeMode(AbstractMotor::brakeMode::brake);
            rightMiddle.setBrakeMode(AbstractMotor::brakeMode::brake);
            rightRear.setBrakeMode(AbstractMotor::brakeMode::brake);
            
        }else {
            turnP = 1;

            leftFront.setBrakeMode(AbstractMotor::brakeMode::coast);
            leftMiddle.setBrakeMode(AbstractMotor::brakeMode::coast);
            leftRear.setBrakeMode(AbstractMotor::brakeMode::coast);
            rightFront.setBrakeMode(AbstractMotor::brakeMode::coast);
            rightMiddle.setBrakeMode(AbstractMotor::brakeMode::coast);
            rightRear.setBrakeMode(AbstractMotor::brakeMode::coast);
        }
    
        pros::delay(15);
	
    }
}
