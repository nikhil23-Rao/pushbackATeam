#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cmath>
#include <algorithm>


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({1, -2, -9},

                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({10, -7, 4}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(21);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(0);

// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(10);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(
    2,    // kP
    0.0,    // kI
    8,   // kD
    3,      // anti windup
    0.15,    // small error range (in)
    100,    // small error timeout (ms)
    4.0,    // large error range (in)
    400,    // large error timeout (ms)
    10       // slew
);


// angular motion controller
lemlib::ControllerSettings angularController(1, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             0, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             50, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
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

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

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

void disabled() {}

/**got
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

pros::Motor firstStage(3, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor secondStage(6, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

double wrapAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

void topStageScore() {
  firstStage.move_voltage(12000);
  secondStage.move_voltage(-12000);
}

void middleScore() {
  firstStage.move_voltage(12000);
  secondStage.move_voltage(-10000);
}

void inventoryScore() {
    firstStage.move_voltage(12000);
}

void outtake() {
  firstStage.move_voltage(-12000);
  secondStage.move_voltage(12000);
}

pros::adi::DigitalOut trapdoor('A');
static bool toggle {false};
pros::adi::DigitalOut scraper('B');
static bool toggle2 {false};
pros::adi::DigitalOut descore('C');
static bool toggle3 {false};
void deployScraper() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        toggle2 = !toggle2;
    }
    scraper.set_value(toggle2);
}
void deployTrapdoor(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        toggle = !toggle;
    }
    trapdoor.set_value(toggle);
}
// Globals for R1 tracking
// assumes these exist globally
bool r2WasPressed = false;
bool descoreDeployed = false;

void updateIntakeAndDescore() {
    bool r1Pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool r2Pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    // --- BOTH pressed → OUTTAKE (highest priority) ---
    if (r1Pressed && r2Pressed) {
        outtake();
        return;
    }

    // --- R2 toggle on press (edge detect) ---
    if (r2Pressed && !r2WasPressed) {
        descoreDeployed = !descoreDeployed;
    }
    r2WasPressed = r2Pressed;

    // --- Handle Intake when R1 is pressed ---
    if (r1Pressed) {
        inventoryScore();
    } 
    else {
        // Other intake controls (UNCHANGED STRUCTURE)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && toggle) {
            middleScore();
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            topStageScore();
        } 
        else {
            // idle
            descore.set_value(descoreDeployed);
            firstStage.move_velocity(0);
            secondStage.move_velocity(0);
        }
    }
}

// void rightSide9Ball(){
//     chassis.moveToPoint(0, 24, 700);
//     pros::delay(750);
//     scraper.set_value(true);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(90, 750);
//     chassis.waitUntilDone();

//     chassis.setPose(0,0,0);
//     firstStage.move_velocity(600);
//     chassis.moveToPoint(0, 11, 1100, {.maxSpeed = 110});
//     chassis.moveToPoint(0, 3, 500,  {.forwards=false});
//     chassis.waitUntilDone();
//     scraper.set_value(false);
//     chassis.moveToPoint(0, -18, 1200, {.forwards = false, .maxSpeed = 60}); 
//     chassis.waitUntilDone();
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-600);
//     pros::delay(1500);
//     firstStage.move_velocity(0);
//     secondStage.move_velocity(0);

//     chassis.setPose(0,0,0);
//     chassis.moveToPoint(-11.4, 6, 700);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(0, 500);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(-11.4, -4, 500, {.forwards = false});
//     chassis.waitUntilDone();
// }

// void soloAWP(){
//     chassis.moveToPoint(0, 24, 700);
//     pros::delay(750);
//     scraper.set_value(true);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(90, 750);
//     chassis.waitUntilDone();

//     chassis.setPose(0,0,0);
//     firstStage.move_velocity(600);
//     chassis.moveToPoint(0, 11, 1100, {.maxSpeed = 110});
//     chassis.moveToPoint(0, 3, 500,  {.forwards=false});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(0, -18.5, 1200, {.forwards = false, .maxSpeed = 65}); 
//     chassis.waitUntilDone();
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-600);
//     pros::delay(1500);
//     firstStage.move_velocity(0);
//     secondStage.move_velocity(0);
//     scraper.set_value(false);

//     chassis.moveToPoint(0, -4, 500);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(129, 800);
//     chassis.waitUntilDone();
    
//     chassis.setPose(0,0,0);
//     firstStage.move_velocity(600);
//     chassis.moveToPoint(0, 32, 1000);
//     pros::delay(600);
//     scraper.set_value(true);
//     pros::delay(400);
//     scraper.set_value(false);
//     chassis.waitUntilDone();
//     // chassis.moveToPoint(0, 43, 500);
//     // chassis.waitUntilDone();
//     // firstStage.move_velocity(-600);
//     // pros::delay(750);
//     // chassis.moveToPoint(0, 47, 500);
//     // chassis.waitUntilDone();
//     // chassis.moveToPoint(0, 37, 500, {.forwards = false});
//     // chassis.waitUntilDone();
//     chassis.turnToHeading(-45, 700);
//     chassis.waitUntilDone();

//     chassis.setPose(0,0,0);
//     firstStage.move_velocity(600);
//     chassis.moveToPoint(0, 35, 800, {.maxSpeed = 90});
//     pros::delay(800);
//     scraper.set_value(true);
//     // pros::delay(400);
//     // scraper.set_value(false);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(-45, 700);
//     chassis.waitUntilDone();
//     trapdoor.set_value(true);
    
//     chassis.setPose(0,0,0);
//     chassis.moveToPoint(0, -13, 500, {.forwards = false, .maxSpeed = 70});
//     chassis.waitUntilDone();
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-600);
//     pros::delay(600);
//     // firstStage.move_velocity(0);
//     secondStage.move_velocity(0);
//     trapdoor.set_value(false);

//     chassis.moveToPoint(0, 41.75, 800);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(-45, 800);
//     chassis.waitUntilDone();

//     chassis.setPose(0,0,0);
//     chassis.moveToPoint(0, -13.5, 500, {.forwards = false}); 
//     chassis.waitUntilDone();
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-600);
//     pros::delay(1000);
//     firstStage.move_velocity(0);
//     secondStage.move_velocity(0);
// }

// void leftSide43(){
//     firstStage.move_velocity(600);
//     chassis.moveToPose(-14.7, 27.7, -60, 1400, {.maxSpeed = 127});
//     pros::delay(800);
//     scraper.set_value(true);
//     chassis.waitUntilDone();

//     scraper.set_value(false);
//     chassis.turnToHeading(-125, 800);
//     chassis.waitUntilDone();

//     firstStage.move_velocity(0);
//     trapdoor.set_value(true);
//     chassis.setPose(0,0,0);
//     chassis.moveToPoint(0, -13, 500, {.forwards = false, .maxSpeed = 70});
//     chassis.waitUntilDone();
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-600);
//     pros::delay(1400);
//     secondStage.move_velocity(0);
//     trapdoor.set_value(false);

//     chassis.moveToPoint(0, 18, 800);
//     chassis.waitUntilDone();
//     pros::delay(200);
//     chassis.turnToHeading(-53, 800);
//     chassis.waitUntilDone();
//     chassis.setPose(0,0,0);

//     scraper.set_value(true);
//     pros::delay(400);
//     firstStage.move_velocity(600);
//     chassis.moveToPoint(0, 19.5, 1300, {.maxSpeed = 85});
//     chassis.moveToPoint(0, 10, 500,  {.forwards=false});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(0, -22.5, 1100, {.forwards = false, .maxSpeed = 60}); 
//     chassis.waitUntilDone();
//     scraper.set_value(false);
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-600);
//     pros::delay(1500);
//     firstStage.move_velocity(0);
//     secondStage.move_velocity(0);
    
//     chassis.setPose(0,0,0);
//     chassis.moveToPoint(-11.4, 6, 700);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(0, 500);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(-11.4, -4, 500, {.forwards = false});
//     chassis.waitUntilDone();
// }

// void progSkills() {
//     chassis.moveToPoint(0, 24, 700);
//     pros::delay(750);
//     scraper.set_value(true);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(-90, 750);
//     chassis.waitUntilDone();

//     chassis.setPose(0,0,0);
//     firstStage.move_velocity(600);
//     chassis.moveToPoint(0, 11, 900, {.maxSpeed = 110});
//     chassis.moveToPoint(0, 3, 500,  {.forwards=false});
//     chassis.moveToPoint(0, 11, 900, {.maxSpeed = 110});
//     chassis.moveToPoint(0, 3, 500,  {.forwards=false});
//     chassis.moveToPoint(0, 11, 900, {.maxSpeed = 110});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(0, -2, 900, {.forwards = false, .maxSpeed = 60}); 
//     chassis.waitUntilDone();

//     chassis.turnToHeading(-80, 1000);
//     chassis.waitUntilDone();
//     chassis.setPose(0,0,0);
//     chassis.moveToPoint(0, 96.2, 3000);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(85, 800);
//     chassis.waitUntilDone();

//     chassis.setPose(0,0,0);
//     chassis.moveToPoint(0, -22, 900, {.forwards = false, .maxSpeed = 60}); 
//     chassis.waitUntilDone();
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-600);
//     pros::delay(2000);
//     secondStage.move_velocity(0);

//     chassis.moveToPoint(0, -2, 800, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(0, 13, 900, {.maxSpeed = 80});
//     chassis.moveToPoint(0, 5, 500,  {.forwards=false});
//     chassis.moveToPoint(0, 13, 900, {.maxSpeed = 110});
//     chassis.moveToPoint(0, 5, 500,  {.forwards=false});
//     chassis.moveToPoint(0, 13, 900, {.maxSpeed = 110});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(0, -22, 1000, {.forwards = false, .maxSpeed = 50}); 
//     chassis.waitUntilDone();
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-600);
//     pros::delay(2000);
//     firstStage.move_velocity(0);
//     secondStage.move_velocity(0);
//     scraper.set_value(false);
//     pros::delay(400);

//     chassis.moveToPoint(0, 0, 800);
//     chassis.waitUntilDone();  
//     chassis.turnToHeading(-135, 800);
//     chassis.waitUntilDone();
//     chassis.setPose(0,0,0);
//     chassis.moveToPose(-5, -10, 45, 800, {.forwards = false});

//     // chassis.moveToPoint(-11, -10, 700);
//     // chassis.waitUntilDone();
//     // chassis.turnToHeading(180, 800);
//     // chassis.waitUntilDone();
//     // chassis.moveToPose(-15, -20, 0, 1000, {.forwards = false});
//     // chassis.waitUntilDone();

//     // chassis.moveToPoint(0, 55, 1500);
//     // chassis.waitUntilDone();
//     // chassis.moveToPoint(-5, 57, 600);
//     // chassis.waitUntilDone();
//     // chassis.turnToHeading(0, 800);
//     // chassis.waitUntilDone();
//     // chassis.setPose(0,0,0);

//     // chassis.moveToPoint(-11, -20.5, 500, {.forwards = false});
//     // chassis.waitUntilDone();
// }

void rightSide4Rush(){
    chassis.setPose(0, 0, 0);
    firstStage.move_velocity(600);
    chassis.moveToPoint(10, 32.5, 1000, {.maxSpeed = 127});
    pros::delay(900);
    scraper.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(120, 800);
    pros::delay(810);

    chassis.setPose(10, 32.5, chassis.getPose().theta);

    chassis.moveToPoint(37, 10, 1000, {.maxSpeed = 127});
    // chassis.waitUntilDone();
    firstStage.move_velocity(0);
    chassis.turnToHeading(182, 750);
    pros::delay(750);

    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -7.5, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 100});
    scraper.set_value(false);
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(1100);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    chassis.setPose(0, 0, 0);

    chassis.moveToPoint(0, 2, 800, {.maxSpeed = 127, .minSpeed = 100});
    // chassis.waitUntilDone();
    chassis.turnToHeading(45, 800);
    // chassis.waitUntilDone();
    chassis.moveToPoint(-7, 0, 800, {.forwards = false, .maxSpeed = 127, .minSpeed =100});
    // chassis.waitUntilDone();
    chassis.turnToHeading(0, 800);
    pros::delay(800);

    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -1.5, 1000, {.forwards = false,.maxSpeed = 127, .minSpeed = 100});
    chassis.moveToPoint(0, -1.75, 1000, {.forwards = false,.maxSpeed = 127, .minSpeed = 50});
    chassis.waitUntil(1);
}

void leftSide43(){
    chassis.setPose(0,0,0);
    scraper.set_value(true);
    chassis.moveToPoint(0, 26, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 800);

}

void autonomous(){
    // rightSide4Rush();
    // rightSide7Push();
    // leftSide43();
    chassis.setPose({0.0,0});
    chassis.moveToPoint(0,24, 1000, {.maxSpeed=127});
    chassis.waitUntilDone();
}

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   // forward/backward
        // int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);     // turning

        // chassis.arcade(forward, turn);


        const double TURN_REDUCTION = 0.5;   // lower = smoother, higher = sharper
        const double TURN_BOOST = 0.5;       // lower = smoother, higher = more sensitive

        int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // normalize to [-1,1]
        double f = forward / 127.0;
        double t = turn / 127.0;

        // decrease forward when turning hard
        double turnScale = 1.0 - TURN_REDUCTION * fabs(t);
        f *= turnScale;

        // increase turn when going faster
        double speedBoost = 1.0 + TURN_BOOST * fabs(f);
        t *= speedBoost;

        // final motor outputs
        int left = (f + t) * 127;
        int right = (f - t) * 127;

        leftMotors.move(left);
        rightMotors.move(right);


        
        // // get joystick positions (tank)
        // int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        // chassis.tank(leftY, rightX);
        updateIntakeAndDescore();
        deployScraper();
        deployTrapdoor();
        
        // delay to save resources
        pros::delay(10);
    }
}