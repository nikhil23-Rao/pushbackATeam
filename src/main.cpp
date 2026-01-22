#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({1 , -9, -5},

                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (rev  ersed)
pros::MotorGroup rightMotors({-7, 4,8}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)


// Inertial Sensor on port 10
pros::Imu imu(21);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(17);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-2);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325 * 300, -3.0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13, // 10-inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings linearController(
    8.3,    // kP
    0.0,    // kI
    5.5,   // kD
    3,      // anti windup
    1,    // small error range (in)
    100,    // small error timeout (ms)
    3,    // large error range (in)
    500,    // large error timeout (ms)
    20       // slew
);




// angular motion controller
lemlib::ControllerSettings angularController(1.6, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10,// derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100,// small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);




// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
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

    // pros::delay(300);

    // imu.reset();
    // while (imu.is_calibrating()) {
    //     pros::delay(10);
    // }

    // chassis.setPose(0, 0, 0);

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
  firstStage.move_voltage(-12000);
  secondStage.move_voltage(-12000);
}

void middleScore() {
  firstStage.move_voltage(-12000);
  secondStage.move_voltage(-8000);
}

void inventoryScore() {
    firstStage.move_voltage(-12000);
}

void outtake() {
  firstStage.move_voltage(12000);
  secondStage.move_voltage(12000);
}

// ================= INTAKE JAM SYSTEM =================

enum class IntakeState {
    IDLE,
    INTAKING,
    CLEARING_JAM
};

IntakeState intakeState = IntakeState::IDLE;

double lastIntakePos = 0;
uint32_t lastMovementTime = 0;
uint32_t jamStartTime = 0;

// ---- tuning ----
constexpr int INTAKE_SPEED = -600;
constexpr int OUTTAKE_SPEED = 600;

constexpr int JAM_VELOCITY_THRESHOLD = 12; // rpm
constexpr int JAM_TIME_MS = 300;            // stall time
constexpr int CLEAR_TIME_MS = 300;          // reverse time

void requestIntake(bool enable) {
    if (!enable) {
        intakeState = IntakeState::IDLE;
        firstStage.move_velocity(0);
        return;
    }

    if (intakeState == IntakeState::IDLE) {
        intakeState = IntakeState::INTAKING;
        lastIntakePos = firstStage.get_position();
        lastMovementTime = pros::millis();
    }
}

void updateIntakeJamSystem() {
    uint32_t now = pros::millis();

    switch (intakeState) {

        case IntakeState::IDLE:
            break;

        case IntakeState::INTAKING: {
            firstStage.move_velocity(INTAKE_SPEED);

            double currentPos = firstStage.get_position();
            double delta = fabs(currentPos - lastIntakePos);
            int velocity = abs(firstStage.get_actual_velocity());

            // jam detection
            if (delta < 1.0 && velocity < JAM_VELOCITY_THRESHOLD) {
                if (now - lastMovementTime > JAM_TIME_MS) {
                    intakeState = IntakeState::CLEARING_JAM;
                    jamStartTime = now;
                }
            } else {
                lastMovementTime = now;
                lastIntakePos = currentPos;
            }
            break;
        }

        case IntakeState::CLEARING_JAM:
            firstStage.move_velocity(OUTTAKE_SPEED);

            if (now - jamStartTime > CLEAR_TIME_MS) {
                intakeState = IntakeState::INTAKING;
                lastMovementTime = now;
                lastIntakePos = firstStage.get_position();
            }
            break;
    }
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
    if (r1Pressed && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
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
        // inventoryScore();
        requestIntake(true);

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
            // firstStage.move_velocity(0);
            requestIntake(false);
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

void rightSide7Push(){
    chassis.setPose(0, 0, 0);
    firstStage.move_velocity(-600);
    chassis.moveToPoint(10 - 4, 32.5 - 4, 700, {.maxSpeed = 127});
    chassis.moveToPoint(11.5, 33.5, 700, {.maxSpeed = 50});
    chassis.waitUntilDone();
    chassis.turnToHeading(125, 650);
    chassis.waitUntilDone();
    chassis.moveToPoint(30, 12, 700, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 650);
    scraper.set_value(true);
    chassis.waitUntilDone();

    chassis.setPose(30, 12, chassis.getPose().theta);
    chassis.moveToPoint(30, -16, 1200, {.maxSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.moveToPoint(30.5, 12 + 15, 1100, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    scraper.set_value(false);

    firstStage.move_velocity(-600);
    secondStage.move_velocity(-600);
    pros::delay(2000);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);

    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0, -11, 700, {.forwards = false, .maxSpeed = 127});
    // chassis.waitUntilDone();
    // firstStage.move_velocity(600);
    // secondStage.move_velocity(-600);
    // pros::delay(1500);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);
    // scraper.set_value(false);

    chassis.setPose(0, 0, chassis.getPose().theta);
    chassis.moveToPoint(-11, 8, 800, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(-11, -16, 1000, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
}

void rightSide4Rush(){
    chassis.setPose(0, 0, 0);
    firstStage.move_velocity(600);
    chassis.moveToPoint(-8, 26.5, 1000, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 800);
    scraper.set_value(true);
    chassis.waitUntilDone();

    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 17, 1000, {.maxSpeed = 100});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 0, 700, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -17, 700, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(1200);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    scraper.set_value(false);

    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(-11, 8, 700, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 700);
    chassis.waitUntilDone();
    chassis.moveToPoint(-11, -16, 1000, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
}

void leftSide43(){
    chassis.setPose(0, 0, 0);
    firstStage.move_velocity(600);
    chassis.moveToPoint(8, 26.5, 1000, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 800);
    scraper.set_value(true);
    chassis.waitUntilDone();

    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 17, 1100, {.maxSpeed = 100});
    chassis.waitUntilDone();
    // firstStage.move_velocity(0);
    chassis.moveToPoint(0.5, -17, 900, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(1500);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    scraper.set_value(false);
    chassis.waitUntilDone();

    chassis.moveToPoint(-8, 0, 900, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(-145, 800);
    firstStage.move_velocity(600);
    chassis.waitUntilDone();
    chassis.moveToPoint(-33, -27, 1000, {.maxSpeed = 127});
    pros::delay(450);
    scraper.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(50, 1000);
    chassis.waitUntilDone();
    firstStage.move_velocity(0);

    chassis.setPose(-33, -27, chassis.getPose().theta);
    trapdoor.set_value(true);
    chassis.moveToPoint(-43, -37, 700, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    // chassis.moveToPoint(0, -14, 700, {.maxSpeed = 127});
    // chassis.waitUntilDone();
    firstStage.move_velocity(600);
    secondStage.move_velocity(-600);
    pros::delay(1200);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    scraper.set_value(false);
    trapdoor.set_value(false);
    
    chassis.moveToPoint(-16, -18, 800, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 800);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -19.5, 1000, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();

}

void driveThroughParkingZone(
    int velocity,
    int timeMs,
    double targetHeading
) {
    uint32_t start = pros::millis();

    while (pros::millis() - start < timeMs) {
        double headingError =
            lemlib::angleError(targetHeading, chassis.getPose().theta);

        double turn = headingError * 2.0; // tune

        leftMotors.move_velocity(velocity - turn);
        rightMotors.move_velocity(velocity + turn);

        pros::delay(10);
    }

    // stop drive
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}

void skills(){
    //Pick up 5 balls, reset poistion
    chassis.setPose(0,0,0);
    firstStage.move_velocity(-600);
    chassis.moveToPoint(0, 35, 1000, {.maxSpeed = 127, .minSpeed = 110});
    chassis.moveToPoint(0, 64, 3500, {.maxSpeed = 75});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 15, 2000, {.forwards = false, .maxSpeed = 30}); 
    chassis.waitUntilDone();
    firstStage.move_velocity(0);

    //drive --> position next to mid goal
    int xDist = 28;
    int yDist = 17;
    chassis.setPose(0, 0, chassis.getPose().theta);
    chassis.moveToPoint(xDist, yDist, 800);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
    chassis.turnToHeading(-45, 800);
    chassis.waitUntilDone();

    //drive into mid goal, score 6 balls
    chassis.moveToPoint(xDist + 18, yDist - 18, 1000, {.forwards = false});
    trapdoor.set_value(true);
    chassis.moveToPoint(xDist + 23, yDist - 23, 800, {.forwards = false});
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-400);
    pros::delay(2000);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    trapdoor.set_value(false);
    chassis.moveToPoint(xDist - 15, yDist + 18, 800);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, chassis.getPose().theta);
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();
    scraper.set_value(true);

    //set pose at first loader, get balls
    chassis.setPose(0, 0, 0);
    firstStage.move_velocity(-600);
    chassis.moveToPoint(0, 24, 1100, {.maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(0, 16, 700, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 24, 1200, {.maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(0, 12, 700, {.forwards = false, .maxSpeed = 90});
    scraper.set_value(false);

    //angle into alley
    chassis.waitUntilDone();
    chassis.turnToHeading(-45, 800);
    chassis.waitUntilDone();
    firstStage.move_velocity(0);
    chassis.setPose(0, 0, chassis.getPose().theta);
    chassis.moveToPoint(0 + 11.5, 0 - 24, 800, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 800);
    chassis.waitUntilDone();

    //drive through alley, align to goal
    chassis.setPose(11.5, -24, chassis.getPose().theta);
    chassis.moveToPoint(11.5, -24 - 50, 2000, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(45, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(1.5, -24 - 40 - 15, 800, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);

    //reset pose, score 6 balls
    chassis.moveToPoint(0, -18, 700, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-600);
    pros::delay(1700);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    scraper.set_value(true);
    chassis.moveToPoint(0, 0, 700, {.maxSpeed = 127});
    chassis.waitUntilDone();

    //go into 2nd match loader --> score
    firstStage.move_velocity(-600);
    chassis.moveToPoint(0, 17, 1100, {.maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(0, 10, 700, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 17, 1200, {.maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(0, 0, 700, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -17, 700, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-600);
    pros::delay(1700);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    scraper.set_value(false);

    //align with blue parking zone, pick up at least 5 balls
    chassis.moveToPoint(0, 0, 700, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.setPose(0, 0, chassis.getPose().theta);
    chassis.moveToPoint(0 + 30, 0 + 22, 800, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();

    //Pick up 5 balls, reset poistion
    chassis.setPose(0,0,0);
    firstStage.move_velocity(-600);
    chassis.moveToPoint(0, 35, 1000, {.maxSpeed = 127, .minSpeed = 110});
    chassis.moveToPoint(0, 64, 3500, {.maxSpeed = 75});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 15, 2000, {.forwards = false, .maxSpeed = 30}); 
    chassis.waitUntilDone();
    firstStage.move_velocity(0);



    // //drive --> position next to mid goal
    // int midGoalDistX = 43.3;
    // int midGoalDistY = 39.4;
    // chassis.setPose(0, 0, chassis.getPose().theta);
    // firstStage.move_velocity(0);
    // chassis.moveToPoint(-9, 10, 800);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(43, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-9 - midGoalDistX, 10 - midGoalDistY, 3500, {.forwards = false, .maxSpeed = 80});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(135, 800);
    // chassis.waitUntilDone();
    // chassis.setPose(-9 - midGoalDistX, 10 - midGoalDistY, chassis.getPose().theta);

    // trapdoor.set_value(true);
    // chassis.moveToPoint(-9 - midGoalDistX - 6, 10 - midGoalDistY + 6, 800, {.forwards = false});
    // chassis.waitUntilDone();
    // firstStage.move_voltage(-12000);
    // secondStage.move_velocity(-300);
    // pros::delay(4000);
    // firstStage.move_voltage(0);
    // secondStage.move_velocity(0);
    // chassis.moveToPoint(-9 - midGoalDistX - 8, 10 - midGoalDistY + 8, 400, {.forwards = false});
    // chassis.moveToPoint(-9 - midGoalDistX - 3, 10 - midGoalDistY + 3, 400);
    // chassis.moveToPoint(-9 - midGoalDistX - 8, 10 - midGoalDistY + 8, 400, {.forwards = false});
    // chassis.moveToPoint(-9 - midGoalDistX + 7, 10 - midGoalDistY - 7, 800);
    // pros::delay(100);
    // trapdoor.set_value(false);
    // scraper.set_value(true);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-9 - midGoalDistX + 7 + 23, 10 - midGoalDistY - 7 - 26.5, 800);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 800);
    // chassis.waitUntilDone();
    // firstStage.move_voltage(0);
    
    // //drive into long goal, score 3
    // chassis.setPose(-9 - midGoalDistX + 7 + 23, 10 - midGoalDistY - 7 - 26.5, chassis.getPose().theta);
    // chassis.moveToPoint(-9 - midGoalDistX + 7 + 23 - 17, 10 - midGoalDistY - 7 - 26.5, 800, {.forwards = false});
    // chassis.waitUntilDone();
    // firstStage.move_voltage(-600);
    // secondStage.move_velocity(-600);
    // pros::delay(800);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);





    // //Drive through park zone
    // chassis.setPose(0,0,0);
    // scraper.set_value(true);
    // firstStage.move_velocity(600);
    // pros::delay(300);
    // chassis.moveToPoint(0, 30, 2000, {.maxSpeed = 127}); // get over initial barrier
    // pros::delay(300);
    // scraper.set_value(false);
    // chassis.moveToPoint(0, 65, 3000);
    // chassis.waitUntilDone();

    // //align to loader
    // chassis.setPose(0, 65, chassis.getPose().theta);
    // chassis.moveToPoint(17, 77, 800, {.maxSpeed = 127});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 800);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(-91.5, 800);
    // chassis.waitUntilDone();

    // //Go into Match Loader, Drive out from loader, angle in alley
    // chassis.setPose(0, 0, 0);
    // scraper.set_value(true);
    // firstStage.move_velocity(600);
    // chassis.moveToPoint(0, 20, 1100, {.maxSpeed = 70});
    // chassis.waitUntilDone();
    // pros::delay(400);
    // // chassis.moveToPoint(7, 84, 700, {.forwards = false, .maxSpeed = 90});
    // // chassis.waitUntilDone();
    // // chassis.moveToPoint(0, 84, 1200, {.maxSpeed = 70});
    // // chassis.waitUntilDone();
    // // pros::delay(500);
    // chassis.moveToPoint(0, 9, 700, {.forwards = false, .maxSpeed = 90});
    // scraper.set_value(false);
    // chassis.waitUntilDone();

    // chassis.turnToHeading(-135, 800);
    // chassis.waitUntilDone();
    // chassis.setPose(0, 9, chassis.getPose().theta);
    // chassis.moveToPoint(0 - 23, 9 - 23, 800);
    // chassis.waitUntilDone();
    // pros::delay(300);
    // chassis.turnToHeading(45, 800);
    // firstStage.move_velocity(0);
    // chassis.waitUntilDone();
    // trapdoor.set_value(true);
    // chassis.setPose(0 - 23, 9 - 23, chassis.getPose().theta);
    // chassis.moveToPoint(0 - 23 - 14, 9 - 23 - 14, 800, {.forwards = false});
    // chassis.waitUntilDone();
    // firstStage.move_velocity(600);
    // secondStage.move_velocity(-300);
    // pros::delay(4000);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);
    // chassis.moveToPoint(0 - 23 - 16, 9  - 23 - 16, 800, {.forwards = false});








    // chassis.turnToHeading(-45, 800);
    // chassis.waitUntilDone();
    // firstStage.move_velocity(0);
    // chassis.setPose(0, 0, chassis.getPose().theta);
    // chassis.moveToPoint(0 + 10, 0 - 24, 800, {.forwards = false, .maxSpeed = 127});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(0, 800);
    // chassis.waitUntilDone();
}  

// void skills(){
//     //drive and score 2 balls into middle, align with loader
//     chassis.setPose(0, 0, 0);
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(600);
//     chassis.moveToPoint(0, 36, 1300, {.maxSpeed = 100});
//     chassis.waitUntilDone();
//     chassis.turnToHeading(-135, 800);
//     firstStage.move_velocity(0);
//     chassis.waitUntilDone();
//     trapdoor.set_value(true);
//     chassis.moveToPoint(0 + 10, 36 + 10, 800, {.forwards = false, .maxSpeed = 100});
//     chassis.waitUntilDone();
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-400);
//     pros::delay(800);
//     firstStage.move_velocity(0);
//     secondStage.move_velocity(0);
//     scraper.set_value(true);
//     chassis.moveToPoint(0 - 25, 36 - 25, 1000, {.maxSpeed = 100});
//     chassis.waitUntilDone();
//     trapdoor.set_value(false);
//     chassis.turnToHeading(177, 800);
//     chassis.waitUntilDone();

//     //Go into Match Loader, Drive out from loader, angle in alley
//     chassis.setPose(0, 0, 0);
//     firstStage.move_velocity(600);
//     chassis.moveToPoint(0, 17, 1100, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     pros::delay(500);
//     chassis.moveToPoint(0, 10, 700, {.forwards = false, .maxSpeed = 90});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(0, 17, 1200, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     pros::delay(500);
//     chassis.moveToPoint(0, 0, 700, {.forwards = false, .maxSpeed = 90});
//     scraper.set_value(false);
//     chassis.waitUntilDone();
//     chassis.turnToHeading(-45, 800);
//     chassis.waitUntilDone();
//     firstStage.move_velocity(0);
//     chassis.setPose(0, 0, chassis.getPose().theta);
//     chassis.moveToPoint(0 + 10, 0 - 24, 800, {.forwards = false, .maxSpeed = 127});
//     chassis.waitUntilDone();
//     chassis.turnToHeading(0, 800);
//     chassis.waitUntilDone();

//     //drive through alley, align to goal
//     chassis.setPose(10, -24, chassis.getPose().theta);
//     chassis.moveToPoint(10, -24 - 38, 2000, {.forwards = false, .maxSpeed = 127});
//     chassis.waitUntilDone();
//     chassis.turnToHeading(45, 800);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(-3, -24 - 38 - 15, 800, {.forwards = false, .maxSpeed = 127});
//     chassis.waitUntilDone();
//     chassis.turnToHeading(180, 800);
//     chassis.waitUntilDone();
//     chassis.setPose(0, 0, 0);

//     //reset pose, score 6 balls
//     chassis.moveToPoint(0, -17, 700, {.forwards = false, .maxSpeed = 127});
//     chassis.waitUntilDone();
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-600);
//     pros::delay(1700);
//     firstStage.move_velocity(0);
//     secondStage.move_velocity(0);
//     scraper.set_value(true);
//     chassis.moveToPoint(0, 0, 700, {.maxSpeed = 127});
//     chassis.waitUntilDone();

//     //go into 2nd match loader --> score
//     firstStage.move_velocity(600);
//     chassis.moveToPoint(0, 17, 1100, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     pros::delay(500);
//     chassis.moveToPoint(0, 10, 700, {.forwards = false, .maxSpeed = 90});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(0, 17, 1200, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     pros::delay(500);
//     chassis.moveToPoint(0, 0, 700, {.forwards = false, .maxSpeed = 90});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(0, -17, 700, {.forwards = false, .maxSpeed = 127});
//     chassis.waitUntilDone();
//     firstStage.move_velocity(600);
//     secondStage.move_velocity(-600);
//     pros::delay(1700);
//     firstStage.move_velocity(0);
//     secondStage.move_velocity(0);
//     scraper.set_value(false);

//     //align with blue parking zone, pick up at least 5 balls
//     chassis.moveToPoint(0, 0, 700, {.maxSpeed = 127});
//     chassis.waitUntilDone();
//     chassis.setPose(0, 0, chassis.getPose().theta);
//     chassis.moveToPoint(0 + 29, 0 + 19, 800, {.maxSpeed = 127});
//     chassis.waitUntilDone();
//     chassis.turnToHeading(90, 800);
//     chassis.waitUntilDone();

//     chassis.setPose(0,0,0);
//     firstStage.move_velocity(600);
//     chassis.moveToPoint(0, 65, 5000, {.maxSpeed = 60});
//     pros::delay(50);
//     scraper.set_value(true);
//     pros::delay(650);
//     scraper.set_value(false);
//     chassis.waitUntilDone();

//     chassis.setPose(0, 65, chassis.getPose().theta);
//     chassis.moveToPoint(0 + 40, 65 + 5, 800, {.maxSpeed = 127});
//     chassis.waitUntilDone();
//     chassis.turnToHeading(90, 800);
//     chassis.waitUntilDone();
// }

void autonomous(){
    pros::Task intakeTask([] {
        while (true) {
            updateIntakeJamSystem();
            pros::delay(10);
        }
    });

    // rightSide7Push();
    // leftSide43();  
    skills();
}

void opcontrol() {
    // controller
    pros::Task intakeTask([] {
        while (true) {
            updateIntakeJamSystem();
            pros::delay(10);
        }
    });

    chassis.setPose(0,0,0);
    firstStage.move_velocity(-600);
    chassis.moveToPoint(0, 35, 1000, {.maxSpeed = 127, .minSpeed = 110});
    chassis.moveToPoint(0, 64, 3500, {.maxSpeed = 75});
    chassis.waitUntilDone();

    // loop to continuously update motors
    while (true) {
        int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   // forward/backward
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);     // turning

        // chassis.arcade(forward, turn);


        const double TURN_REDUCTION = 0.5;   // lower = smoother, higher = sharper
        const double TURN_BOOST = 0.5;       // lower = smoother, higher = more sensitive

        // int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

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

        lemlib::Pose pose = chassis.getPose();
        pros::lcd::print(4, "X: %.2f", pose.x);
        pros::lcd::print(5, "Y: %.2f", pose.y);
        pros::lcd::print(6, "H: %.2f", pose.theta);
        
        // // get joystick positions (tank)
        // int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        // chassis.tank(leftY, rightX);
        updateIntakeAndDescore();
        deployScraper();
        deployTrapdoor();

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            chassis.setPose(0,0,0);
            firstStage.move_velocity(-600);
            chassis.moveToPoint(0, 35, 1000, {.maxSpeed = 127, .minSpeed = 110});
            chassis.moveToPoint(0, 64, 3500, {.maxSpeed = 75});
            chassis.waitUntilDone();
        }
        
        // delay to save resources
        pros::delay(10);
    }
}