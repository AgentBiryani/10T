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
        }
        
        if (BoostDown.isPressed()) {
            boost.set_value(true);
        }
        
        
        if (BoostDown.isPressed()) {
            boost.set_value(true);
        }

        if (Speed.isPressed()){
            turnP = .0135;
            
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
    .withMotors(Left, Right)   
    .withSensors(Left.getEncoder(), Right.getEncoder())
    .withDimensions(AbstractMotor::gearset::blue * (48/36), {{3.25_in, 16_in}, (imev5BlueTPR * (48/36))})
    .withOdometry({{3.25_in, 16_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
    .buildOdometry();




double pGain = 0.0012;
double iGain = 0.00000;
double dGain = 0.000001;

auto model = t -> getModel();

void movePID(double distanceL, double distanceR, int ms, double maxV = 1)
{
	double degreesL = distanceL / (3.141 * 3.25) * 360;
	double degreesR = distanceR / (3.141 * 3.25) * 360;
	auto drivePIDL = okapi::IterativeControllerFactory::posPID(pGain, iGain, dGain);
	auto drivePIDR = okapi::IterativeControllerFactory::posPID(pGain, iGain, dGain);
	std::cout << drivePIDL.getSampleTime().convert(okapi::millisecond) << std::endl;
	model->resetSensors();

	int timer = 0;
	double errorL;
	double errorR;
	double powerL;
	double powerR;
	while (timer < ms)
	{
		errorL = degreesL + model->getSensorVals()[0] / 337.5 * 360;
		errorR = degreesR + model->getSensorVals()[1] / 337.5 * 360;
		powerL = drivePIDL.step(errorL);
		powerR = drivePIDR.step(errorR);
		model->tank(powerL * maxV, powerR * maxV);
		std::cout << "Left: " << model->getSensorVals()[0] << " Right: " << model->getSensorVals()[1] << std::endl;
		std::cout << errorL << " " << errorR << std::endl;

		pros::delay(10);
		timer += 10;
	}

	model->tank(0, 0);
}



void turnPID(double angle){
	double threshold;
	if(angle <= 0.0){
		threshold = 1.5;
	}
	else{
		threshold = 0.7;
	}

	double error = angle - imu_sensor.get_rotation();
	double integral;
	double derivative;
	double prevError;
	double kp = 2.4;
	double ki = 0;
	double kd = 4;

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

        float bcap = 30;
        if (abs(error) < 30)
        {
            bcap = 20;
        }
        if (abs(vel) <= bcap)
        {
            if (vel > 0)
            {
                vel = bcap;
            }
            else
            {
                vel = -bcap;
            }
        }

        //printf("%f\n", vel);

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


void jerk(){
    leftFront.moveVelocity(-600);
    leftMiddle.moveVelocity(-600);
    leftRear.moveVelocity(-600);
    rightFront.moveVelocity(600);
    rightMiddle.moveVelocity(600);
    rightRear.moveVelocity(600);

    pros::delay(100);
    
    leftFront.moveVelocity(600);
    leftMiddle.moveVelocity(600);
    leftRear.moveVelocity(600);
    rightFront.moveVelocity(-600);
    rightMiddle.moveVelocity(-600);
    rightRear.moveVelocity(-600);

    pros::delay(100);

    leftFront.moveVelocity(-600);
    leftMiddle.moveVelocity(-600);
    leftRear.moveVelocity(-600);
    rightFront.moveVelocity(600);
    rightMiddle.moveVelocity(600);
    rightRear.moveVelocity(600);

    pros::delay(100);
    
    leftFront.moveVelocity(600);
    leftMiddle.moveVelocity(600);
    leftRear.moveVelocity(600);
    rightFront.moveVelocity(-600);
    rightMiddle.moveVelocity(-600);
    rightRear.moveVelocity(-600);
    
    pros::delay(100);
    
    leftFront.moveVelocity(-600);
    leftMiddle.moveVelocity(-600);
    leftRear.moveVelocity(-600);
    rightFront.moveVelocity(600);
    rightMiddle.moveVelocity(600);
    rightRear.moveVelocity(600);

    pros::delay(100);
    
    leftFront.moveVelocity(600);
    leftMiddle.moveVelocity(600);
    leftRear.moveVelocity(600);
    rightFront.moveVelocity(-600);
    rightMiddle.moveVelocity(-600);
    rightRear.moveVelocity(-600);

    pros::delay(100);
    
    leftFront.moveVelocity(-600);
    leftMiddle.moveVelocity(-600);
    leftRear.moveVelocity(-600);
    rightFront.moveVelocity(600);
    rightMiddle.moveVelocity(600);
    rightRear.moveVelocity(600);

    pros::delay(100);
    
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
    pros::delay(250);
    cataReset();
}


#pragma endregion

