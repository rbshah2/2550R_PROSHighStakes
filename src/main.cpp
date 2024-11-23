#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-19, 3, -15},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(12);

pros::Motor intake1(-16);
pros::Motor intake2(17);
pros::MotorGroup intake({-16, 17});
pros::ADIDigitalOut clamp_piston_1('A');
pros::ADIDigitalOut blooper('C');
pros::ADIDigitalOut intake_lift('E');
pros::ADIDigitalOut climb('H');
pros::Optical intakeSensor(9);

// horizontal tracking wheel encoder. Rotation sensor, port 14, not reversed
pros::Rotation horizontalEnc(14);

// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::OLD_275_HALF, 2.398);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              30 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(19, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    intakeSensor.set_led_pwm(100);
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy


/*move to pose: 
    chassis.moveToPose():
    parameters:
    first value: x
    second value: y
    third value: target angle
    fourth value: timeout(in ms)
    {bracket values}:
    .forwards: if your movement is going with the front of your robot
    .maxSpeed: the max speed out of 127 ur robot can move at
    .lead: how "curvy" ur movement can be. The straighter you want your movement, the less out of 1 it will be(usually 0.1, 0.2). It has to be out of (0-1).
    .minSpeed: the least speed out of 127 ur robot can move at
    .earlyExitRange: DO NOT WORRY ABOUT THIS

    chassis.moveToPoint():
    parameters: 
    first value: x
    second value: y
    third value: timeout(in ms)
    {bracket values}:
    .forwards: if your movement is going with the front of your robot
    .maxSpeed: the max speed out of 127 your robot can move at

*/


void autonomous() {
    int dist;
    lemlib::Pose b = lemlib::Pose(0,0);
    chassis.moveToPose(0, 24, 90, 3000, {.forwards = true, .lead = 0.4});
    //IMPORTANT ONE BELOW
    chassis.moveToPoint(0,0, 3000, {.forwards = false, .maxSpeed = 127});
    
}

/**
 * Runs in driver control
 */
#define BLUE_HUE 100
#define RED_HUE 100
#define RUNINTAKE intake.move(127);
#define RUNOUTTAKE intake.move(-127);

void opcontrol() {
    // controller
    // loop to continuously update motors
    static bool r2Pressed;
    while (true) {
        //color sort
        if(intakeSensor.get_hue() == BLUE_HUE ){
            RUNOUTTAKE
            pros::delay(200); //tune this value
            RUNINTAKE
        } else{
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
                RUNINTAKE
            }
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
                RUNOUTTAKE
            }
        }
        //base control
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX, true, 0);

        //boolean switches value with each press

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            //if true, switch to false. If false switch to true
            r2Pressed = !r2Pressed;
            //if the button is pressed
            if(r2Pressed){
                clamp_piston_1.set_value(1);
            } 
            //otherwise
            else{
                clamp_piston_1.set_value(0);
            }
        }
        

        // delay to save resources
        pros::delay(10);
    }
}
