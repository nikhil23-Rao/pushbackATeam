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
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
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
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            outtake();
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

void soloAWP(){
    //move to first loader, turn towards it
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 17.5, 1000, {.minSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 800);
    scraper.set_value(true);
    chassis.waitUntilDone();
    int xDist = -8;
    int yDist = 17.5;
    
    //set pose at first loader, get balls
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    firstStage.move_velocity(-600);
    chassis.moveToPoint(xDist + 10, yDist, 500, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist + 22.5, yDist, 520, {.maxSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist, 700, {.forwards = false, .maxSpeed = 60});
    pros::delay(300);
    scraper.set_value(false);

    //score 4 balls
    chassis.moveToPoint(xDist - 10, yDist, 750, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-600);
    pros::delay(950);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);

    //pick up 6 balls across
    chassis.moveToPoint(xDist, yDist, 700);
    chassis.waitUntilDone();
    chassis.turnToHeading(-135, 800);
    firstStage.move_velocity(-600);
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist - 7, yDist - 24, 700);
    pros::delay(500);
    scraper.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 600);
    chassis.waitUntilDone();
    scraper.set_value(false);
    chassis.setPose(xDist - 7, yDist - 24, chassis.getPose().theta);
    chassis.moveToPoint(xDist-5, yDist - 20 - 38, 900);
    pros::delay(750);
    scraper.set_value(true);
    chassis.moveToPoint(xDist-5, yDist - 20 - 49, 400, {.maxSpeed = 45});
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 800);
}

void rightSide7Push(){
    //pick up stack of 3
    chassis.setPose(0, 0, 0);
    firstStage.move_velocity(-600);
    chassis.moveToPoint(13 - 5, 29 - 5, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(13, 29, 700, {.maxSpeed = 70});
    chassis.turnToHeading(135, 800);
    chassis.waitUntilDone();
    int xDist = 13;
    int yDist = 29;
    chassis.setPose(xDist, yDist, chassis.getPose().theta);

    //align to loader
    chassis.moveToPoint(xDist + 15, yDist - 15, 700, {.minSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);
    chassis.waitUntilDone();
    scraper.set_value(true);
    pros::delay(300);
    xDist = xDist + 15;
    yDist = yDist - 15;

    //set pose at first loader, get balls
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    firstStage.move_velocity(-600);
    chassis.moveToPoint(xDist, yDist - 10, 500, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist - 22.5, 700, {.maxSpeed = 90});
    pros::delay(500);
    // chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist - 5, 700, {.forwards = false, .maxSpeed = 60});
    pros::delay(300);
    scraper.set_value(false);

    //score 4 balls
    chassis.moveToPoint(xDist, yDist + 12, 1000, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-600);
    pros::delay(1200);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    yDist = yDist + 12;

    //descore arm
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    chassis.moveToPoint(xDist + 14, yDist - 5, 800);
    chassis.waitUntilDone();
    chassis.turnToHeading(175, 800); 
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist + 15, yDist + 3.5, 800, {.forwards = false, .minSpeed = 127});
    chassis.waitUntilDone();
}

void leftSide4Rush(){
    //move to first loader, turn towards it
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(8, 28.75, 1000, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 800);
    scraper.set_value(true);
    chassis.waitUntilDone();
    int xDist = 8;
    int yDist = 28.75;
    
    //set pose at first loader, get balls
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    firstStage.move_velocity(-600);
    chassis.moveToPoint(xDist - 10, yDist, 500, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist - 22.5, yDist, 700, {.maxSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist, 700, {.forwards = false, .maxSpeed = 127});
    pros::delay(300);
    scraper.set_value(false);

    //score 4 balls
    chassis.moveToPoint(xDist + 16, yDist, 1000, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-600);
    pros::delay(1000);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    xDist = xDist + 16;
    yDist = yDist + 1;

    //descore arm
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    chassis.moveToPoint(xDist - 7, yDist - 12, 800);
    chassis.waitUntilDone();
    chassis.turnToHeading(-95, 800); 
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist + 5, yDist - 12, 800, {.forwards = false, .minSpeed = 127});
    chassis.waitUntilDone();
}

void rightSide4Rush(){
    //move to first loader, turn towards it
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 32, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 800);
    scraper.set_value(true);
    chassis.waitUntilDone();
    int xDist = 0;
    int yDist = 32;
    
    //set pose at first loader, get balls
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    firstStage.move_velocity(-600);
    chassis.moveToPoint(xDist + 10, yDist - 1.5, 500, {.maxSpeed = 127});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist + 22.5, yDist - 1.5, 700, {.maxSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist - 1.5, 700, {.forwards = false, .maxSpeed = 60});
    pros::delay(300);
    scraper.set_value(false);
    yDist = yDist - 1.5;

    //score 4 balls
    chassis.moveToPoint(xDist - 13, yDist, 1000, {.forwards = false, .maxSpeed = 127});
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-600);
    pros::delay(1000);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    xDist = xDist - 10;

    //descore arm
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    chassis.moveToPoint(xDist + 6, yDist + 14, 800);
    chassis.waitUntilDone();
    chassis.turnToHeading(85, 800); 
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist - 1, yDist + 14, 800, {.forwards = false, .minSpeed = 110});
    chassis.waitUntilDone();
}

void leftSide43(){
    //Pick up 4 balls, reset poistion
    chassis.setPose(0,0,0);
    descore.set_value(true);
    firstStage.move_velocity(-600);
    chassis.moveToPose(-9, 18, -45, 600, {.minSpeed = 80});
    chassis.waitUntilDone();
    chassis.moveToPoint(-14, 23, 700, {.maxSpeed = 60});
    // pros::delay(250);
    // scraper.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(-135, 800);
    int xDist = -14;
    int yDist = 23;
    chassis.setPose(xDist, yDist, chassis.getPose().theta);

    //deploy scraper, score 5 ball into mid goal
    // firstStage.move_velocity(0);
    // chassis.moveToPoint(xDist + 10, yDist + 10, 700, {.forwards = false, .maxSpeed = 60});
    // chassis.waitUntilDone();
    // trapdoor.set_value(true);
    // chassis.moveToPoint(xDist + 15, yDist + 15, 700, {.forwards = false});
    // firstStage.move_velocity(-600);
    // secondStage.move_velocity(-400);
    // pros::delay(1200);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);


    

    // //move to first loader, turn towards it
    // chassis.setPose(0, 0, 0);
    // // firstStage.move_velocity(600);
    // chassis.moveToPoint(8, 28.75, 1000, {.maxSpeed = 127});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(-90, 800);
    // scraper.set_value(true);
    // chassis.waitUntilDone();
    // int xDist = 8;
    // int yDist = 28.75;
    
    // //set pose at first loader, get balls
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // firstStage.move_velocity(-600);
    // chassis.moveToPoint(xDist - 10, yDist, 500, {.maxSpeed = 70});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist - 22.5, yDist, 700, {.maxSpeed = 90});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist - 2, yDist, 700, {.forwards = false, .maxSpeed = 90});
    // pros::delay(300);
    // scraper.set_value(false);

    // //score 4 balls
    // chassis.moveToPoint(xDist + 16, yDist, 1000, {.forwards = false, .maxSpeed = 60});
    // chassis.waitUntilDone();
    // firstStage.move_velocity(-600);
    // secondStage.move_velocity(-600);
    // pros::delay(1000);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);

    // //angle towards mid goal, pickup three balls, align with midgoal
    // chassis.moveToPoint(xDist - 3.5, yDist, 700, {.maxSpeed = 90});
    // xDist = xDist - 3.5;
    // chassis.waitUntilDone();
    // chassis.turnToHeading(135, 800);
    // chassis.waitUntilDone();
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // firstStage.move_velocity(-600);
    // chassis.moveToPoint(xDist + 16, yDist - 16, 700, {.maxSpeed = 90});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist + 26, yDist - 26, 700, {.maxSpeed = 60});
    // chassis.waitUntilDone();
    // chassis.setPose(xDist + 26, yDist - 26, chassis.getPose().theta);
    // chassis.turnToHeading(-45, 800);
    // chassis.waitUntilDone();
    // trapdoor.set_value(true);
    // xDist = xDist + 26;
    // yDist = yDist - 26;
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // chassis.moveToPoint(xDist + 9, yDist - 10, 700, {.forwards = false, .maxSpeed = 90});
    // chassis.waitUntilDone();

    // firstStage.move_velocity(-600);
    // secondStage.move_velocity(-600);
    // pros::delay(1000);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);

    // xDist = xDist + 10;
    // yDist = yDist - 10;
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // //align with long goal, descore
    // chassis.moveToPoint(xDist - 24, yDist + 23.8, 800, {.maxSpeed = 90});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(-95, 800);
    // chassis.waitUntilDone();
    // xDist = xDist - 24;
    // yDist = yDist + 23.8;
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // chassis.moveToPoint(xDist + 22.7, yDist, 800, {.forwards = false, .maxSpeed = 127});
    // chassis.waitUntilDone();
}

void skills(){
    //Pick up 4 balls, reset poistion
    chassis.setPose(0,0,0);
    descore.set_value(true);
    firstStage.move_velocity(-600);
    chassis.moveToPose(-11, 18, -45, 600, {.minSpeed = 80});
    chassis.waitUntilDone();
    chassis.moveToPoint(-16, 23, 700, {.maxSpeed = 60});
    // pros::delay(250);
    // scraper.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(-135, 800);
    int xDist = -16;
    int yDist = 23;
    chassis.setPose(xDist, yDist, chassis.getPose().theta);

    //deploy scraper, score 5 ball into mid goal
    firstStage.move_velocity(0);
    chassis.moveToPoint(xDist + 10, yDist + 10, 700, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    trapdoor.set_value(true);
    chassis.moveToPoint(xDist + 15, yDist + 15, 700, {.forwards = false});
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-400);
    pros::delay(1200);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);

    //drive --> align with loader/long goal
    chassis.moveToPoint(xDist - 26, yDist - 17, 1000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);
    trapdoor.set_value(false);
    chassis.waitUntilDone();
    scraper.set_value(true);
    xDist = xDist - 26;
    yDist = yDist - 18;
    chassis.setPose(xDist, yDist, chassis.getPose().theta);

    //pick up 6 balls
    firstStage.move_velocity(-600);
    chassis.moveToPoint(xDist, yDist - 10, 800, {.maxSpeed = 80});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist - 20, 1000);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(xDist, yDist - 12, 800, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist - 20, 1000);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(xDist, yDist - 6, 800, {.forwards = false, .minSpeed = 50});
    chassis.waitUntilDone();

    //reset pose, angle, drive through alley way
    yDist = yDist - 6;
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    chassis.turnToHeading(135, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist - 11, yDist + 13, 800, {.forwards = false, .minSpeed = 50});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);
    chassis.waitUntilDone();
    firstStage.move_velocity(0);
    scraper.set_value(false);
    chassis.moveToPoint(xDist - 11, yDist + 13 + 58, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(-135, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist + 13 + 58 + 12.75, 800, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 800);
    chassis.waitUntilDone();

    //reset pose, drive into long goal, score 6 balls
    yDist = yDist + 13 + 58 + 12.75;
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    chassis.moveToPoint(xDist, yDist - 17, 800, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    scraper.set_value(true);
    firstStage.move_velocity(600);
    pros::delay(600);
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-600);
    pros::delay(2000);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);

    //drive into loader, pick up 6 balls
    chassis.moveToPoint(xDist, yDist, 800, {.maxSpeed = 70});
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    chassis.moveToPoint(xDist, yDist + 20, 800, {.maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(xDist, yDist + 12, 800, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist + 20, 1000);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(xDist, yDist + 12, 800, {.forwards = false});
    chassis.waitUntilDone();

    //drive into long goal, score 6 balls
    chassis.moveToPoint(xDist, yDist, 800, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist - 17, 800, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    firstStage.move_velocity(600);
    pros::delay(600);
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-600);
    pros::delay(2000);
    scraper.set_value(false);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);
    chassis.moveToPoint(xDist, yDist + 5, 800, {.maxSpeed = 70});
    chassis.waitUntilDone();
    yDist = yDist + 5;
    chassis.setPose(xDist, yDist, chassis.getPose().theta);

    //angle towards stack, pick up blocks
    chassis.turnToHeading(135, 800);
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    chassis.moveToPoint(xDist + 30, yDist - 23, 800, {.maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
    chassis.setPose(xDist + 30, yDist - 23, chassis.getPose().theta);
    chassis.moveToPoint(xDist + 30 + 45, yDist - 23, 2000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    xDist = xDist + 30 + 45;
    yDist = yDist - 23;

    //turn towards middle goal, score
    chassis.turnToHeading(45, 800);
    chassis.waitUntilDone();
    firstStage.move_velocity(0);
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    chassis.moveToPoint(xDist - 10, yDist - 10, 700, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    trapdoor.set_value(true);
    chassis.moveToPoint(xDist - 12, yDist - 12, 700, {.forwards = false});
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-400);
    pros::delay(1200);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);

    //drive --> align with loader/long goal
    chassis.moveToPoint(xDist + 26, yDist + 18, 1000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 800);
    trapdoor.set_value(false);
    chassis.waitUntilDone();
    scraper.set_value(true);
    xDist = xDist + 26;
    yDist = yDist + 18;
    chassis.setPose(xDist, yDist, chassis.getPose().theta);

    //score 4 balls
    chassis.moveToPoint(xDist, yDist - 17, 800, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    secondStage.move_velocity(-600);
    pros::delay(700);
    firstStage.move_velocity(0);
    secondStage.move_velocity(0);

    //drive into loader, pick up 6 balls
    chassis.moveToPoint(xDist, yDist, 800, {.maxSpeed = 70});
    chassis.waitUntilDone();
    firstStage.move_velocity(-600);
    chassis.moveToPoint(xDist, yDist + 20, 800, {.maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(xDist, yDist + 12, 800, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist + 20, 1000);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(xDist, yDist, 1200, {.forwards = false});
    chassis.waitUntilDone();

    //angle, drive through alley
    chassis.setPose(xDist, yDist, chassis.getPose().theta);
    chassis.turnToHeading(-45, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist + 11, yDist - 13, 800, {.forwards = false, .minSpeed = 50});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 800);
    chassis.waitUntilDone();
    firstStage.move_velocity(0);
    scraper.set_value(false);
    chassis.moveToPoint(xDist + 11, yDist - 13 - 58, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(45, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(xDist, yDist - 13 - 58 - 12.75, 800, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 800);
    chassis.waitUntilDone();




    // //Pick up 5 balls, reset poistion
    // chassis.setPose(0,0,0);
    // firstStage.move_velocity(-600);
    // chassis.moveToPoint(0, 35, 1000, {.maxSpeed = 127, .minSpeed = 110});
    // chassis.moveToPoint(0, 62, 3500, {.maxSpeed = 75});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0, 15, 2000, {.forwards = false, .maxSpeed = 30}); 
    // chassis.waitUntilDone();
    // firstStage.move_velocity(0);

    // //drive --> position next to mid goal
    // int xDist = 28;
    // int yDist = 17;
    // chassis.setPose(chassis.getPose().x, 0, chassis.getPose().theta);
    // chassis.moveToPoint(xDist, yDist, 800);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 800);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(-45, 800);
    // chassis.waitUntilDone();

    // //drive into mid goal, score 6 balls
    // chassis.moveToPoint(xDist + 18, yDist - 18, 1000, {.forwards = false});
    // trapdoor.set_value(true);
    // chassis.moveToPoint(xDist + 23, yDist - 23, 800, {.forwards = false});
    // chassis.waitUntilDone();
    // firstStage.move_velocity(-600);
    // secondStage.move_velocity(-400);
    // pros::delay(2000);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);
    // chassis.moveToPoint(xDist - 10.5, yDist + 10.5, 1000);
    // chassis.waitUntilDone();
    // trapdoor.set_value(false);
    // chassis.turnToHeading(-90, 800);
    // chassis.waitUntilDone();
    // chassis.setPose(xDist - 10.5, yDist + 10.5, chassis.getPose().theta);
    // scraper.set_value(true);
    // xDist = xDist - 10.5;
    // yDist = yDist + 10.5;

    // //set pose at first loader, get balls
    // firstStage.move_velocity(-600);
    // chassis.moveToPoint(xDist - 16, yDist, 1100, {.maxSpeed = 50});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist - 22.5, yDist, 700, {.maxSpeed = 90});
    // chassis.waitUntilDone();
    // pros::delay(800);
    // chassis.moveToPoint(xDist - 2, yDist, 700, {.forwards = false, .maxSpeed = 90});
    // pros::delay(300);
    // scraper.set_value(false);
    // xDist = xDist - 2;

    // //angle into alley
    // chassis.waitUntilDone();
    // chassis.turnToHeading(-145, 800);
    // chassis.waitUntilDone();
    // firstStage.move_velocity(0);
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // chassis.moveToPoint(xDist + 15, yDist + 12.75, 800, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(-90, 800);
    // chassis.waitUntilDone();
    // xDist = xDist + 15;
    // yDist = yDist + 12.75;

    // //drive through alley, align to goal
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // chassis.moveToPoint(xDist + 51, yDist, 2000, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(-45, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist + 51 + 13, yDist - 12, 800, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 800);
    // chassis.waitUntilDone();
    // xDist = xDist + 51 + 13;
    // yDist = yDist - 12;

    // //reset pose, score 6 balls
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // chassis.moveToPoint(xDist - 16, yDist, 700, {.forwards = false});
    // chassis.waitUntilDone();
    // firstStage.move_velocity(-600);
    // secondStage.move_velocity(-600);
    // pros::delay(1700);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);
    // scraper.set_value(true);

    // //center, drive into match loader
    // chassis.moveToPoint(xDist + 5, yDist, 700);
    // chassis.waitUntilDone();
    // firstStage.move_velocity(-600);
    // chassis.moveToPoint(xDist + 16, yDist, 1100, {.maxSpeed = 50});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist + 22.5, yDist, 700, {.maxSpeed = 90});
    // chassis.waitUntilDone();
    // pros::delay(800);
    // chassis.moveToPoint(xDist + 5, yDist, 700, {.forwards = false, .maxSpeed = 90});
    // pros::delay(300);
    // scraper.set_value(false);

    // //score 6 balls
    // chassis.moveToPoint(xDist - 16, yDist, 1300, {.forwards = false, .maxSpeed = 60});
    // chassis.waitUntilDone();
    // firstStage.move_velocity(-600);
    // secondStage.move_velocity(-600);
    // pros::delay(1700);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);

    // //align with blue parking zone
    // chassis.moveToPoint(xDist + 5, yDist, 700);
    // chassis.waitUntilDone();
    // chassis.setPose(xDist + 5, yDist, chassis.getPose().theta);
    // chassis.moveToPoint(xDist + 5 + 20, yDist - 30, 800, {.maxSpeed = 127});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(180, 800);
    // chassis.waitUntilDone();
    // pros::delay(500);
    // xDist = xDist + 5 + 20;
    // yDist = yDist - 30; 

    // //Pick up 5 balls, reset poistion
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // firstStage.move_velocity(-600);
    // chassis.moveToPoint(xDist, yDist - 35, 1000, {.maxSpeed = 127, .minSpeed = 110});
    // chassis.moveToPoint(xDist, yDist - 62, 3500, {.maxSpeed = 75});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist, yDist - 20, 2000, {.forwards = false, .maxSpeed = 30}); 
    // chassis.waitUntilDone();
    // firstStage.move_velocity(0);
    // yDist = yDist - 15;

    // //drive --> position next to mid goal
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // chassis.moveToPoint(xDist - 24, yDist - 33.5, 800);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 800);
    // chassis.waitUntilDone();
    // xDist = xDist - 24;
    // yDist = yDist - 33.5;

    // //reset pose, score 6 balls
    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // chassis.moveToPoint(xDist, yDist, 700, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist - 16, yDist, 700, {.forwards = false});
    // chassis.waitUntilDone();
    // firstStage.move_velocity(-600);
    // secondStage.move_velocity(-600);
    // pros::delay(1700);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);
    // scraper.set_value(true);
    // // yDist = yDist - 1;

    // //center, drive into match loader
    // chassis.moveToPoint(xDist + 5, yDist, 700);
    // chassis.waitUntilDone();
    // firstStage.move_velocity(-600);
    // chassis.moveToPoint(xDist + 16, yDist, 1100, {.maxSpeed = 50});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist + 22.5, yDist, 700, {.maxSpeed = 90});
    // chassis.waitUntilDone();
    // pros::delay(800);
    // chassis.moveToPoint(xDist + 5, yDist, 700, {.forwards = false, .maxSpeed = 90});
    // scraper.set_value(false);
    // xDist = xDist + 5;

    // //angle to mid goal, drive into mid goal, score 3 balls
    // chassis.turnToHeading(135, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist - 30, yDist + 30, 1000, {.forwards = false});
    // trapdoor.set_value(true);
    // chassis.moveToPoint(xDist - 37, yDist + 37, 800, {.forwards = false});
    // chassis.waitUntilDone();
    // firstStage.move_velocity(-600);
    // secondStage.move_velocity(-400);
    // pros::delay(1000);
    // firstStage.move_velocity(0);
    // secondStage.move_velocity(0);
    // xDist = xDist - 37;
    // yDist = yDist + 37;

    // //drive out from mid goal, move across field towards red side
    // chassis.moveToPoint(xDist + 8, yDist - 8, 1000);
    // chassis.waitUntilDone();
    // trapdoor.set_value(false);
    // chassis.turnToHeading(90, 800);
    // chassis.waitUntilDone();
    // xDist = xDist + 8;
    // yDist = yDist - 8;

    // chassis.setPose(xDist, yDist, chassis.getPose().theta);
    // chassis.moveToPoint(xDist - 51, yDist, 2000, {.forwards = false, .maxSpeed = 127});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(45, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(xDist - 51  - 13, yDist - 13, 800, {.forwards = false, .maxSpeed = 127});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(-90, 800);
    // chassis.waitUntilDone();
    // xDist = xDist + 51 + 13;
    // yDist = yDist - 13;
}  

void autonomous(){
    pros::Task intakeTask([] {
        while (true) {
            updateIntakeJamSystem();
            pros::delay(10);
        }
    });

    // soloAWP();
    // leftSide4Rush();
    // rightSide4Rush();
    // rightSide7Push();
    leftSide43();  
    // skills();
}

void opcontrol() {
    // controller
    pros::Task intakeTask([] {
        while (true) {
            updateIntakeJamSystem();
            pros::delay(10);
        }
    });

    // chassis.setPose(0,0,0);
    // firstStage.move_velocity(-600);
    // chassis.moveToPoint(0, 35, 1000, {.maxSpeed = 127, .minSpeed = 110});
    // chassis.moveToPoint(0, 64, 3500, {.maxSpeed = 75});
    // chassis.waitUntilDone();

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