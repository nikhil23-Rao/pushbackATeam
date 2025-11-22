#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-2, 1, -9},

                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({10, -8, 4}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)


// Inertial Sensor on port 10
pros::Imu imu(21);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(16                                              );

// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
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
lemlib::ControllerSettings linearController(9, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            0, // derivative gain (kD)
                                             0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
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

void curveToPose(double x, double y, double finalHeading, int maxSpeed = 127, int timeoutMs = 4000) {
    int startTime = pros::millis();
    double prevF = 0;

    while (true) {
        auto pose = chassis.getPose();
        double dx = x - pose.x;
        double dy = y - pose.y;
        double dist = sqrt(dx*dx + dy*dy);

        // Stop condition: close enough or timeout
        if ((dist < 0.5 && fabs(wrapAngle(finalHeading - pose.theta)) < 2) || 
            (pros::millis() - startTime > timeoutMs)) break;

        // -------- Forward along vector to target --------
        double targetAngle = atan2(dy, dx) * 180 / M_PI;
        double headingToTarget = wrapAngle(targetAngle - pose.theta); // shortest turn to point at target
        double f = dist / 12.0;

        // Slow down when turning sharply
        double turnScale = 1.0 - 0.6 * fabs(headingToTarget / 90.0);
        if (turnScale < 0.0) turnScale = 0.0;
        f *= turnScale;

        // Acceleration / deceleration
        double accel = 0.05;
        if (f > prevF + accel) f = prevF + accel;
        if (f < prevF - accel) f = prevF - accel;
        prevF = f;

        // -------- Heading correction --------
        double headingError = wrapAngle(finalHeading - pose.theta);
        double t = headingError / 90.0; // proportional turn
        t *= 0.8;

        // -------- Convert to motor outputs --------
        int left = int((f + t) * maxSpeed);
        int right = int((f - t) * maxSpeed);

        // Cap outputs
        if (left > 127) left = 127; if (left < -127) left = -127;
        if (right > 127) right = 127; if (right < -127) right = -127;

        leftMotors.move(left);
        rightMotors.move(right);

        pros::delay(10);
    }

    // Stop motors
    leftMotors.move(0);
    rightMotors.move(0);
}



void topStageScore() {
  firstStage.move_voltage(12000);
  secondStage.move_voltage(-12000);
}

void middleScore() {
  firstStage.move_voltage(12000);
  secondStage.move_voltage(-12000);
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

void deployDescore(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        toggle3 = !toggle3;
    }
    descore.set_value(toggle3);
}


void updateIntake(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {          
        topStageScore();  
    } else {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            outtake();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            inventoryScore();
        } else {
            firstStage.move_velocity(0);
            secondStage.move_velocity(0);
        }   
    } 
}

void rightSide9Ball(){
    chassis.setPose(0,0,0);
    firstStage.move_velocity(600);
    chassis.moveToPose(16, 32, 60, 1600, {.maxSpeed = 127});
    pros::delay(1000);
    scraper.set_value(true);
    chassis.moveToPoint(17.5,33, 400, {.maxSpeed = 70});
    chassis.waitUntilDone();

    // chassis.turnToHeading(75, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(30, 39.5, 800);
    // pros::delay(700);
    // scraper.set_value(true);
    // // pros::delay(600);
    // //scraper.set_value(false);
    // chassis.waitUntilDone();

    // chassis.moveToPoint(20, 29.5, 1000, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(0, 800);
    // chassis.waitUntilDone();
    chassis.moveToPoint(29.5, 4, 1400, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);
    chassis.waitUntilDone();
    firstStage.move_velocity(0);

    chassis.setPose(0,0,0);
    chassis.moveToPoint(-0.2, -17, 1000, {.forwards = false});
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(1400);
    secondStage.move_velocity(0);

    secondStage.move_velocity(0);
    chassis.moveToPoint(3, 14, 1450, {.maxSpeed = 50});
    chassis.moveToPoint(3, 9, 500,  {.forwards=false});
    chassis.moveToPoint(3, 14, 500, {.maxSpeed = 80});
    chassis.moveToPoint(3, 9, 500,  {.forwards=false});
    chassis.moveToPoint(3, 14, 500, {.maxSpeed = 80});
    chassis.waitUntilDone();

    chassis.moveToPoint(0, -17, 1500, {.forwards=false, .maxSpeed = 60});
    scraper.set_value(false);
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(2000);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    chassis.moveToPoint(0, -8, 800);
    chassis.moveToPoint(0, -23, 500, {.forwards=false, .maxSpeed = 127});


    // chassis.moveToPoint(13, -3, 600);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(180, 600);
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.moveToPoint(2, 27, 900);

    


    // chassis.setPose(0,0,0);
    // descore.set_value(true);

    // // chassis.moveToPoint(16, 32, 1000);
    // firstStage.move_velocity(600);
    // chassis.moveToPose(19, 30.6, 53,1400, {.maxSpeed = 127});
    // chassis.waitUntilDone();
    // pros::delay(200);

    // chassis.moveToPoint(29, 38.5, 1100);
    // chassis.waitUntilDone();
    // // chassis.moveToPoint(23, 38.5, 1100);

    // // chassis.moveToPoint(28i23i 1000, {.forwards=false});
    // chassis.moveToPoint(20, 15, 1000,{.forwards = false});

    // chassis.waitUntilDone();
    // chassis.moveToPoint(38.5,21,1000,{.forwards=false, .maxSpeed=70});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(180,1000);
    // chassis.waitUntilDone();
    // topStageScore();
    // scraper.set_value(1);
    // pros::delay(2300);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);
    // chassis.waitUntilDone();
    // chassis.setPose(0,0,0);
    // chassis.waitUntilDone();
    // firstStage.move_velocity(600);
    // chassis.moveToPoint(-4,40,360);
    // chassis.waitUntilDone();
    // pros::delay(1700);
    // chassis.moveToPoint(-4,20,700);
    //  chassis.waitUntilDone();
    //  chassis.moveToPoint(-4,32,700);
    //  chassis.waitUntilDone();
    // pros::delay(900);
    // chassis.moveToPoint(0,0,700);
    //  chassis.waitUntilDone();
    //  topStageScore();
    // pros::delay(1700);


    // // firstStage.move_velocity(0);
    // // secondStage.move_velocity(0);
    // // // chassis.turnToHeading(0, 500);
    // // chassis.waitUntilDone();
    
    // // chassis.waitUntilDone();
    // // chassis.moveToPoint(27, 15, 500, {.forwards = false});
    // // chassis.moveToPoint(23, 30, 500, {.forwards = false});
}

void rightSide7Ball(){ 
    chassis.setPose(0,0,0);

    firstStage.move_velocity(600);
    chassis.moveToPose(20,35,25,1700); //pick up the first 3 balls
    chassis.waitUntilDone();
    chassis.moveToPoint(35, 10, 1000);
    chassis.waitUntilDone();
    firstStage.move_velocity(0);
    chassis.moveToPose(50.5, 0.2, 177.5,1700); // this needs to be 1700
    chassis.waitUntilDone();

    chassis.moveToPoint(46.2, 16.3, 500, {.forwards=false, .maxSpeed = 60});
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(1750);

    scraper.set_value(true);
    secondStage.move_velocity(0);
    chassis.moveToPoint(46.2, -20.5, 1400, {.maxSpeed = 60});
    chassis.moveToPoint(46.2, -12.5, 500,  {.forwards=false});
    chassis.moveToPoint(46.2, -20.5, 1200, {.maxSpeed = 80});
    chassis.moveToPoint(46.2, -12.5, 300,  {.forwards=false});
    chassis.waitUntilDone();

    chassis.moveToPoint(46.2, 16.3, 1500, {.forwards=false, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(1300);

    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    chassis.moveToPoint(46.2, 7.3, 500);
    chassis.moveToPoint(46.2, 16.3, 500, {.forwards=false});
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
}

void leftSide43Ball(){
    chassis.setPose(0,0,0);
    scraper.set_value(true);
    chassis.moveToPoint(0,28,750);//align with goal
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000);
    chassis.waitUntilDone();

    firstStage.move_velocity(600); //matchloading
    chassis.moveToPoint(-14.75, 21.5, 1000,{.maxSpeed=80});
    chassis.moveToPoint(-8, 21.5, 500, {.forwards=false});
    chassis.moveToPoint(-14, 21.5, 500,{.maxSpeed= 80});
    chassis.moveToPoint(-8, 21.5, 500, {.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();


    chassis.moveToPoint(18, 22.5, 1400, {.forwards=false,.maxSpeed = 60});
    chassis.waitUntilDone();
    secondStage.move_velocity(-600);
    pros::delay(2000);
    secondStage.move_velocity(0);
    firstStage.move_velocity(0);

    scraper.set_value(false);
    chassis.moveToPoint(-3, 17.3, 800);
    chassis.waitUntilDone();
    chassis.turnToHeading(133, 800);
    chassis.waitUntilDone();
    
    firstStage.move_velocity(600);
    chassis.moveToPoint(38, -13, 1600, {.maxSpeed = 70});
    pros::delay(600);
    scraper.set_value(true);
    chassis.waitUntilDone();

    chassis.turnToHeading(-45, 800);
    chassis.waitUntilDone();
    firstStage.move_velocity(0);

    // chassis.setPose(0,0,0);
    // chassis.moveToPoint(1.8, -20, 800, {.forwards = false});
    // chassis.waitUntilDone();
    trapdoor.set_value(true);
    chassis.moveToPoint(37.34, -13.3, 800, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(-44, 800);
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    


}

void leftSide43BallOLD(){
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,25.3,750);//align with goal
    pros::delay(550);
    scraper.set_value(true);
    chassis.turnToHeading(-90, 1000);
    chassis.waitUntilDone();

    firstStage.move_velocity(600); //matchloading
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 11, 1000,{.maxSpeed=80});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 4.25, 500, {.forwards=false});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 11, 500,{.maxSpeed= 80});
    chassis.waitUntilDone();
    // chassis.waitUntilDone();
    // pros::delay(175);

    chassis.moveToPoint(-0.70, -18.1, 1400, {.forwards=false,.maxSpeed = 60});
    chassis.waitUntilDone();
    secondStage.move_velocity(-600);
    pros::delay(2000);
    secondStage.move_velocity(0);
    firstStage.move_velocity(0);

    scraper.set_value(false);
    chassis.moveToPoint(-1.2, -4, 800);
    chassis.waitUntilDone();
    chassis.turnToHeading(-123, 800);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);
    
    firstStage.move_velocity(600);
    chassis.moveToPoint(0, 35, 1600, {.maxSpeed = 70});
    pros::delay(600);
    scraper.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(178, 900);
    chassis.waitUntilDone();
    firstStage.move_velocity(0);

    chassis.setPose(0,0,0);
    chassis.moveToPoint(1.8, -20, 800, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(1.8, -15, 500);
    chassis.waitUntilDone();
    trapdoor.set_value(true);
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    


}


void leftSide61Ball(){
    chassis.setPose(0,0,0);

    chassis.moveToPoint(0, 5, 500);
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    chassis.turnToHeading(-45, 800);
    chassis.waitUntilDone();

    chassis.moveToPoint(-20, 25, 1500, {.maxSpeed = 80});
    chassis.waitUntilDone();
    chassis.turnToHeading(-129.5, 1000);
    firstStage.move_velocity(0);
    chassis.waitUntilDone();

    chassis.moveToPoint(-9.5, 22.5, 600, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    secondStage.move_velocity(-600);
    pros::delay(300);
    secondStage.move_velocity(0);
    pros::delay(200);

    chassis.moveToPoint(-39, -5, 1200);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);
    chassis.waitUntilDone();

    firstStage.move_velocity(600);
    scraper.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(-36.5, -26, 1200, {.maxSpeed = 90});
    chassis.moveToPoint(-36.5, -19, 500,  {.forwards=false});
    chassis.moveToPoint(-36.5, -26, 1200, {.maxSpeed = 90});
    chassis.moveToPoint(-36.5, -19, 500,  {.forwards=false});
    chassis.waitUntilDone();

    chassis.moveToPoint(-35.5, 16.3, 1500, {.forwards=false, .maxSpeed = 60});
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
}

void soloAWP(){
    chassis.moveToPoint(5.2, -26, 750, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    secondStage.move_velocity(-350);
    pros::delay(300);
    secondStage.move_velocity(0);
    pros::delay(150);

    chassis.moveToPoint(0, -15, 750);
    chassis.waitUntilDone();
    chassis.turnToHeading(-129, 700);
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    chassis.moveToPoint(-18, -38, 1600, {.maxSpeed = 50});
    pros::delay(600);
    scraper.set_value(true);

    chassis.waitUntilDone();
    chassis.turnToHeading(180, 500);
    chassis.waitUntilDone();
    firstStage.move_velocity(0);
    scraper.set_value(false);
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-2, -45, 750);

    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    pros::delay(1000);
    firstStage.move_velocity(600);
    pros::delay(100);
    firstStage.move_velocity(0);

    chassis.moveToPoint(-39.2, -13, 900, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 750);
    chassis.waitUntilDone();

    scraper.set_value(true);
    firstStage.move_velocity(600);
    pros::delay(300);
    chassis.moveToPoint(-39, 18, 1200, {.maxSpeed = 90});
    chassis.moveToPoint(-39, 9, 500,  {.forwards=false});
    chassis.moveToPoint(-39, 18, 500, {.maxSpeed = 90});
    chassis.moveToPoint(-39, 9, 500,  {.forwards=false});
    chassis.waitUntilDone();

    chassis.moveToPoint(-43.4, -21.5, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
}

void moveForward(){
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 3, 1000);
}

void progSkills() {
    chassis.setPose(0,0,0);

    //move forward, rotate to first loader, move into loader, score in goal
    scraper.set_value(true);
    chassis.moveToPoint(0, 44, 1000, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    chassis.moveToPoint(24, 44, 1200, {.maxSpeed = 85});
    chassis.moveToPoint(20, 44, 500, {.forwards = false});
    chassis.moveToPoint(25, 44, 1200, {.maxSpeed = 85});
    chassis.moveToPoint(20, 44, 500, {.forwards = false});
    chassis.moveToPoint(25, 44, 1200, {.maxSpeed = 85});
    chassis.waitUntilDone();

    chassis.moveToPoint(8, 44, 500, {.forwards = false}); 
    chassis.waitUntilDone();
    chassis.turnToHeading(-135, 800);
    firstStage.move_velocity(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 24, 600);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 800);

    chassis.moveToPoint(-70, 25, 2000, {.maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(-45, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(-78, 45.5, 600);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();

    chassis.setPose(0,0,0);

    chassis.moveToPoint(0, -15, 800, {.forwards = false}); 
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(3000);

    secondStage.move_velocity(0);
    chassis.moveToPoint(-1, 19, 1200, {.maxSpeed = 75});
    chassis.moveToPoint(-1, 13, 500, {.forwards = false});
    chassis.moveToPoint(-1, 19, 1200, {.maxSpeed = 85});
    chassis.moveToPoint(-1, 13, 500, {.forwards = false});
    chassis.moveToPoint(-1, 19, 1200, {.maxSpeed = 85});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -15, 1500, {.forwards = false, .maxSpeed = 60}); 
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(3000);

    firstStage.move_velocity(0);
    secondStage.move_velocity(0);  
    chassis.moveToPoint(0, 0, 800); 
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 800);
    chassis.moveToPoint(-97 , -3, 3000, {.maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 800);
    chassis.waitUntilDone();

    chassis.setPose(0,0,0);
    firstStage.move_velocity(600);
    chassis.moveToPoint(0, 15, 1200, {.maxSpeed = 75});
    chassis.moveToPoint(0, 10, 500, {.forwards = false});
    chassis.moveToPoint(0, 15, 1200, {.maxSpeed = 85});
    chassis.moveToPoint(0, 10, 500, {.forwards = false});
    chassis.moveToPoint(0, 15, 1200, {.maxSpeed = 85});
    chassis.waitUntilDone();

    chassis.moveToPoint(0, 3, 500, {.forwards = false}); 
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 800);
    firstStage.move_velocity(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(25, -13, 600);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);

    chassis.moveToPoint(25, -80, 2000, {.maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(-135, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(2.95, -89, 800); //adjust x for aligning
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);
    chassis.waitUntilDone();
    chassis.setPose(0,0,0);

    chassis.moveToPoint(0, -11.5, 800, {.forwards = false}); 
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(3000);

    secondStage.move_velocity(0);
    firstStage.move_velocity(600);
    chassis.moveToPoint(-1, 25, 900, {.maxSpeed = 85});
    chassis.moveToPoint(-1, 20, 500, {.forwards = false});
    chassis.moveToPoint(-1, 25, 900, {.maxSpeed = 85});
    chassis.moveToPoint(-1, 20, 500, {.forwards = false});
    chassis.moveToPoint(-1, 25, 900, {.maxSpeed = 85});
    chassis.waitUntilDone();

    chassis.moveToPoint(0, -11.5, 1000, {.forwards = false, .maxSpeed = 60}); 
    scraper.set_value(false);
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(3000);

    chassis.moveToPoint(0, 0, 500);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    chassis.moveToPoint(-59, -2, 900);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 800);
    chassis.waitUntilDone();

    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 20, 1000, {.maxSpeed = 127});
}


void autonomous() {
    chassis.setPose(0,0,0);
    // progSkills();
    // rightSide7Ball();
    // leftSide61Ball();
    rightSide9Ball();
    // soloAWP();
    // moveForward();
 
    // leftSide43Ball();
}

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   // forward/backward
        // int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);     // turning

        // double turnFactor = 0.90;
        // turn = turn * (fabs(forward) / 127.0 * (1.0 - turnFactor) + turnFactor);


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
        updateIntake();
        deployScraper();
        deployTrapdoor();
        deployDescore();
        
        // delay to save resources
        pros::delay(10);
    }
}