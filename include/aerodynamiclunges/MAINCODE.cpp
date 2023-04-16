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


void turnPID(double angle){
	double threshold;
	if(angle <= 0.0){
		threshold = .3;
	}
	else{
		threshold = .3;
	}

	double error = angle - imu_sensor.get_rotation();
	double integral;
	double derivative;
	double prevError;
	double kp = 3;
	double ki = 0;
	double kd = 15;

    float start = imu_sensor.get_rotation();
    float target = start + angle;

	while(fabs(error) > threshold){

		error = angle - imu_sensor.get_rotation();
		integral  = integral + error;

		if(error == 0 || fabs(error) >= angle){
			integral = 0;
		}

		derivative = error - prevError;
		prevError = error;
		double p = error * kp;
		double i = integral * ki;
		double d = derivative * kd;

		double vel = p + i + d;

        leftFront.moveVelocity(vel);
        leftMiddle.moveVelocity(vel);
        leftRear.moveVelocity(vel);
        rightFront.moveVelocity(vel);
        rightMiddle.moveVelocity(vel);
        rightRear.moveVelocity(vel);

		pros::delay(15);
	}
    
    leftFront.moveVelocity(0);
    leftMiddle.moveVelocity(0);
    leftRear.moveVelocity(0);
    rightFront.moveVelocity(0);
    rightMiddle.moveVelocity(0);
    rightRear.moveVelocity(0);
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


#pragma endregion

