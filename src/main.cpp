#include "main.h"
#include "devices.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/llemu.hpp"


const int RF = 7;
const int RMB = -8;
const int RMT = 9; 
const int RB = 10; 
const int LF = -1; 
const int LMB = -2;
const int LMT = 3;
const int LB = -4;
const int IMUport = 21;

MotorGroup right_drive({RF,RMB,RMT,RB});
MotorGroup left_drive({LF,LMB,LMT,LB});
IMU intertial(21);
Controller controller(E_CONTROLLER_MASTER);

lemlib::OdomSensors odom_setup(
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&intertial
);

lemlib::Drivetrain Drive(
	&left_drive, 
	&right_drive, 
	11.375,       
	lemlib::Omniwheel::NEW_325,
	450,
	2
);
lemlib::ExpoDriveCurve linear(
	5,
	15, 
	1.005
);

lemlib::ExpoDriveCurve angular(
	0,
	0,
	0
);


// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              2, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.35, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              7, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              5 // maximum acceleration (slew)
);

lemlib::Chassis chassis(
	Drive,
	lateral_controller,
	angular_controller,
	odom_setup,
	&linear/*,
	&angular*/
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	chassis.calibrate();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
chassis.setPose(0,0,0);
chassis.moveToPoint(0, 48, 2000,{},false);
pros::lcd::print(1, "%f", chassis.getPose().y);
controller.rumble(".");
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	
		while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
		int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int curvature = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		if (abs(curvature)<5){
			curvature = 0;
		}
		chassis.curvature( forward/2, curvature/2);
		
		pros::delay(20); // Run for 20 ms then update
	}
}