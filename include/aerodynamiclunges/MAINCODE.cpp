#include "main.h"
#include "okapi/api.hpp"
#include "aerodynamiclunges/declaration.cpp"


using namespace okapi;
using namespace std;

#pragma region Driver Controller 

void opcontr() {
    while (true){                
        // Get joystick inputs
        double forwardPower = controller.getAnalog(ControllerAnalog::rightX);
        double turnPower = controller.getAnalog(ControllerAnalog::leftY);
        forwardPower = (forwardPower);
		turnPower = (turnPower);
        double cpost = rotation.get();
        double leftPower = (forwardPower * turnP) + turnPower;
        double rightPower = (forwardPower * turnP) - turnPower;
        Catapult.setBrakeMode(AbstractMotor::brakeMode::brake);

		
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
#pragma endregion

#pragma region Building Autonomous  

std::shared_ptr<OdomChassisController> t =
  ChassisControllerBuilder()
    .withMotors({-12, -13, -14}, 
                {  1,   3,  11})
    .withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 16_in}, (imev5BlueTPR * (48/36))})
    .withOdometry({{3.25_in, 16_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
    .buildOdometry();


void roll(float Volts){
  Intake.moveVoltage(Volts);
}


const int TURN_SPEED = 100;  // Motor speed for turning
const int TURN_THRESHOLD = 3; // Stop turning when within 3 degrees of target

// Function to turn a specific angle (in degrees) using the inertial sensor
void turn(int angle) {
  imu.reset();
  int target = imu.get_rotation() + angle;
  int direction = (angle > 0) ? 1 : -1;
  while (abs(imu.get_rotation() - target) > TURN_THRESHOLD) {
    int error = target - imu.get_rotation();
    int speed = error * direction * TURN_SPEED / 180;
    
    leftFront.moveVelocity(-speed);
    leftMiddle.moveVelocity(-speed);
    leftRear.moveVelocity(-speed);
    rightFront.moveVelocity(speed);
    rightMiddle.moveVelocity(speed);
    rightRear.moveVelocity(speed);


    // Wait for a short amount of time to allow the robot to turn
    pros::delay(20);
  }
    leftFront.moveVelocity(0);
    leftMiddle.moveVelocity(0);
    leftRear.moveVelocity(0);
    rightFront.moveVelocity(0);
    rightMiddle.moveVelocity(0);
    rightRear.moveVelocity(0);
}

// Example usage of the turn function
void autonomous() {
  // Turn 90 degrees to the right
  turn(90);

  // Turn 45 degrees to the left
  turn(-45);
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

#pragma endregion

