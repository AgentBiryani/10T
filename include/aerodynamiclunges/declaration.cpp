#include "main.h"
#include "okapi/api.hpp"


using namespace okapi;
using namespace std;


Controller controller;

ControllerButton intake(ControllerDigital::L1);
ControllerButton outtake(ControllerDigital::L2);
ControllerButton Shoot(ControllerDigital::R1);
ControllerButton Speed(ControllerDigital::R2);
ControllerButton Expansion(ControllerDigital::Y);
ControllerButton BoostUp(ControllerDigital::up);
ControllerButton BoostDown(ControllerDigital::down);

const int LEFT_FRONT_PORT = 12;
const int LEFT_MIDDLE_PORT = 13;
const int LEFT_REAR_PORT = 14;
const int RIGHT_FRONT_PORT = 1;
const int RIGHT_MIDDLE_PORT = 3;
const int RIGHT_REAR_PORT = 11;

Motor leftFront(LEFT_FRONT_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftMiddle(LEFT_MIDDLE_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftRear(LEFT_REAR_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightFront(RIGHT_FRONT_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightMiddle(RIGHT_MIDDLE_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightRear(RIGHT_REAR_PORT, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

MotorGroup Left({12, 13, 14});
MotorGroup Right({1, 3, 11});

Motor Intake(8, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor Catapult(16, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
pros::ADIDigitalOut expansion (2);
pros::ADIDigitalOut boost (1);
pros::Imu imu_sensor(7);
RotationSensor rotation(18);

#pragma region DRIVER CONSTANTS 
const double ACCELERATION_RATE = .55;
const double DEACCELERATION_RATE = -1.85;
double targetPower = 600.0;
double currentPower;
double turnP;

#pragma endregion
#pragma region CATAPULT CONSTANTS 
double cpost = rotation.get();
float prime = 70;
float cataspeed;
#pragma endregion

