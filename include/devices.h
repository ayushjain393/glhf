#include "api.h"
#include "lemlib/api.hpp"
using namespace pros; 
extern const int RF;
extern const int RMB;
extern const int RMT; 
extern const int RB; 
extern const int LF; 
extern const int LMB;
extern const int LMT;
extern const int LB;
extern const int IMUport;
extern MotorGroup left_drive;
extern MotorGroup right_drive;
extern Controller controller;
// DRIVETRAIN CONSTRUCTOR ONNLY!!!!
extern lemlib::Drivetrain Drive;
extern IMU intertial;
extern lemlib::OdomSensors odom_setup;
//USE CHASSIS NOT DRIVE FOR DRIVE MOVEMENTS!!
extern lemlib::Chassis chassis;
// linear PID constructor
extern lemlib::ControllerSettings lateral_controller;
//angular PID constructor 
extern lemlib::ControllerSettings angular_controller;