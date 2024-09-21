#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
using namespace pros;
Motor Lf(20);
Motor Lb(18);
Motor Rf(-16);
Motor Rb(-17);
Motor Rt(15);
Motor Lt(-19);
Controller master(E_CONTROLLER_MASTER);
MotorGroup left_side_motors({-20, -18,19}); // Creates a motor group with forwards ports 1 & 3 and reversed port 2
MotorGroup right_side_motors({16,17, -15});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
MotorGroup intake_mg({8, 9});
Rotation odomVerticalPod (3);
Imu imu (1);
adi::Pneumatics piston('h', true);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
// Lemlib drivetrain  struct

lemlib::Drivetrain drivetrain{
    &left_side_motors,  // left drrivetrain motors
    &right_side_motors, // right train motors
    10,                 // track width in INCHES
    3.25,               // wheel diameter
    360,                // wheel rpm
    8                   // tune this value later : )

};

// define odom pod
lemlib::TrackingWheel vertical_tracking_wheel(&odomVerticalPod, 2.75, 4.3, 1);
// odom struct
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, nullptr, nullptr, &imu);

// forward/backward PID
lemlib::ControllerSettings lateralController{
    10,  // kP
    0,   // KI
    55,   // kD
    0,   // windup
    1,   // smallErrorRange
    100, // smallErrorTimeout
    3,   // largeErrorRange
    250, // largeErrorTimeout
    5    // slew rate
};

// turning PID
// lemlib::ControllerSettings angularController{

//     1.5, // kP (1.6)
//     0,
//     1, // kD (1)
//     0,
//     2,   // smallErrorRange
//     100, // smallErrorTimeout
//     3,   // largeErrorRange
//     250, // largeErrorTimeout
//     5    // slew rate
// };

lemlib::ControllerSettings angularController(0.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              60, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController,
                        sensors);
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
void initialize()
{pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });

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
	    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
    // // set position to x:0, y:0, heading:0
    // chassis.setPose(-147.711,-60.528,270);
    // // turn to face heading 90 with a very long timeout
    // //chassis.turnToHeading(270, 2000);
	// chassis.moveToPose(-60.945,-56.363,270, 4000,{.forwards=false});
}
	//left_mg.move(-70);
	/*right_side_.move(-70);
	delay(1200);
	piston.set_value(true);
	left_mg.move(0);
	right_mg.move(0);
	intake_mg.move(-100);
	piston.set_value(true);
	left_mg.move(20);
	right_mg.move(20);
	delay(1000);
*/





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

void opcontrol()
{
	bool pstate = false;
	while (true)
	{
		    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
	}
		// Arcade control scheme
		// int dir = master.get_analog(ANALOG_LEFT_Y);	  // Gets amount forward/backward from left joystick
		// int turn = .90 * master.get_analog(ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
		// left_side_motors.move(dir + turn);					  // Sets left motor voltage
		// right_side_motors.move(dir - turn);					  // Sets right motor voltage

		if (master.get_digital(DIGITAL_R1))
		{
			intake_mg.move(-100);
		}
		else if (master.get_digital(DIGITAL_R2))
		{
			intake_mg.move(100);
		}
		else
		{
			intake_mg.move(0);
		}

		if (master.get_digital_new_press(DIGITAL_A)){
			pstate = !pstate;
		}

		if (pstate == true){
			piston.extend();
		}
		else {
			piston.retract();
		}
		// if (master.get_digital(DIGITAL_A))
		// {
		// 	piston.extend();
		// }
		// if (master.get_digital(DIGITAL_B))
		// {
		// 	piston.retract();
		// }
		delay(20); // Run for 20 ms then update
	}
}