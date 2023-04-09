#include "main.h"
#include "okapi/api.hpp"

using namespace okapi;
using namespace std;

// Creating motor objects for intake and catapult
Motor Intake(8, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor Catapult(16, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
pros::ADIDigitalOut expansion (2);
pros::ADIDigitalOut boost (1);

// Creating object for rotation sensor - gets position of catapult motor
RotationSensor rotation(18);

// Variables for catapult
double cpost = rotation.get();
float prime = 72;
float cataspeed;

std::shared_ptr<OdomChassisController> chassis =
        ChassisControllerBuilder()
        
            .withMotors({-12, -13, -14}, 
                        {1, 3, 5})
            .withGains(
                {0.00195, 0.000005, 0.0000105},
                {0.00195, 0.000005, 0.0000105},
                {0.00195, 0.000005, 0.0000105}
            )
            // green gearset, 4 inch wheel diameter, 11.5 inch wheel track
            .withDimensions({AbstractMotor::gearset::blue, (48.0 / 36.0)}, {{3.25_in, 16_in}, imev5BlueTPR})
            .withOdometry() // use the same scales as the chassis (above)
            .buildOdometry(); // build an odometry chassis

void initialize() {
    boost.set_value(false);
    expansion.set_value(false);
}

void disabled() {}

void competition_initialize() {}

void move(QLength distance) {
    chassis->moveDistance(distance);
}

void wait() {
    chassis->waitUntilSettled();
}

void turn(QAngle angle) {
    chassis->turnAngle(angle*0.75);
}

void roll(float Volts){
  Intake.moveVoltage(Volts);
}

void stopRoll(){
  Intake.moveVoltage(0);
}

void cataReset(){
    while (Catapult.getTargetVelocity() != 0) {
        float cpost = rotation.get();

        if (prime - cpost < 10){
            cataspeed = 35;
        }else{
            cataspeed = 100;
        }
    
        if (cpost < prime || cpost > 300){
            Catapult.moveVelocity(cataspeed);
        } else {
            Catapult.moveVoltage(0);
        }   
    }    
}

void shoot(){
    Catapult.moveVoltage(12000);
    cataReset();
}

void autonomous() {
    chassis->setMaxVelocity(300);
    chassis->setState({0_in, 0_in, 0_deg});



    move(-36_in);

}


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
    ControllerButton Expansion(ControllerDigital::Y);
    ControllerButton BoostUp(ControllerDigital::up);
    ControllerButton BoostDown(ControllerDigital::down);

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
    const double ACCELERATION_RATE = 7.55; // increase in power per millisecond
    const double DEACCELERATION_RATE = -11.65; // decrease in power per millisecond
    
    // Variables for catapult
    double cpost = rotation.get();
    float prime = 72;
    float cataspeed;

    // Variables for drive
    double targetPower = 475.0;
    double currentPower;
    double turnP;


    while (true){                
        // Get joystick inputs
        double forwardPower = controller.getAnalog(ControllerAnalog::rightX);
        double turnPower = controller.getAnalog(ControllerAnalog::leftY);
        forwardPower = (forwardPower * 21/13);
		turnPower = (turnPower * 17/11);
        double cpost = rotation.get();
        double leftPower = (forwardPower * turnP) + turnPower;
        double rightPower = (forwardPower * turnP) - turnPower;
        Catapult.setBrakeMode(AbstractMotor::brakeMode::brake);


		
        // Limits the power to -5.0 to 5.0
        leftPower = std::clamp(leftPower, -15.0, 15.0);
        rightPower = std::clamp(rightPower, -15.0, 15.0);

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
            pros::delay(250);
        } 
        else {
            Catapult.moveVoltage(0);
        }

        if (Expansion.isPressed()){
            expansion.set_value(true);
        }else{
            expansion.set_value(false);
        }
        
        
        if (BoostUp.isPressed()){
            boost.set_value(false);
        }
        else if (BoostDown.isPressed()) {
            boost.set_value(true);
        }

        if (Speed.isPressed()){
            turnP = .095;
            
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
