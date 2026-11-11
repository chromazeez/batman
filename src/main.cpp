// motors, sensors, chassis, and the four functions PROS calls during a match
// routes -> autons.cpp, brain screen -> screen.cpp, anything shared -> include/robot.h

#include "robot.h"
#include <cmath>

// ---- devices ----
// global on purpose, PROS calls initialize/autonomous/opcontrol separately so shared stuff has
// to outlive all three

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain, 3 motors a side, minus sign = mounted backwards so it spins the other way
// we made the old back the new front, which swapped the sides
// so leftMotors is the physical RIGHT side of the robot now
pros::MotorGroup leftMotors({19, 17, 6}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-20, -18, -3}, pros::MotorGearset::blue);

pros::Motor claw(2, pros::MotorGears::green);
pros::Motor toggle(8, pros::MotorGears::green);
pros::MotorGroup cascade({-9, 10}, pros::MotorGearset::green); // two motors locked together, 9 faces the other way

pros::Imu imu(5);
pros::Rotation horizontalEnc(-4); // reversed for the same reason the drive is, front flipped
pros::Optical optical(16);

// ---- chassis ----
// plain LemLib setup, these are the numbers to change if the robot gets rebuilt

lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0); // 2" omni, 0 offset from tracking center

// 11.3" track width, 2.75" drive wheels, 450 rpm, 2 horizontal drift
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.3, lemlib::Omniwheel::NEW_275, 450, 2);

// kP, kI, kD, anti windup, small error, small timeout, large error, large timeout, slew
// exit conditions ported from EZ-Template's drive defaults: within 1" for 90ms, or 3" for 250ms
// the old 500ms large timeout was the pause after a drive that settles instead of chaining
lemlib::ControllerSettings linearController(7.5, 0, 10, 3, 1, 90, 3, 250, 20);
// exit conditions ported from EZ-Template's turn defaults: within 3 deg for 90ms, or 7 for 250ms
// 1 degree was below what the IMU can resolve, so the turn hunted trying to hit it
lemlib::ControllerSettings angularController(7, 0, 45, 3, 3, 90, 7, 250, 0);

// vertical 1, vertical 2, horizontal 1, horizontal 2, imu
lemlib::OdomSensors sensors(nullptr, nullptr, &horizontal, nullptr, &imu); // no vertical wheels, hence the nullptrs

// deadband, min output, expo gain
// curve softens small stick moves so the driver can line up slow without the robot jumping
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// ---- cascade ----

// the cascade has no PID of ours. move_absolute uses the motor's own position controller, which
// runs in motor firmware faster than this loop could and is the same every time

// ---- auto toggle ----
// driver lines up with a toggle, sensor sees something close, motor spins until the right color
// shows up, then stops. no button

constexpr int TOGGLE_MIN_PROXIMITY = 100; // 0-255, below this assume nothing is in front of the sensor

// hue = angle round the color wheel, 0-360, measured off the real toggles
// reading drifts with field lighting so anything within HUE_TOLERANCE counts
// to retune: watch the Hue line on the brain screen with a toggle in front of the sensor
constexpr double RED_HUE = 353;   // #eb001c
constexpr double BLUE_HUE = 195;  // #0085b3
constexpr double YELLOW_HUE = 51; // #efcd00
constexpr double HUE_TOLERANCE = 25;

constexpr double hueOf(ToggleColor color) {
    switch (color) {
        case ToggleColor::blue: return BLUE_HUE;
        case ToggleColor::yellow: return YELLOW_HUE;
        default: return RED_HUE;
    }
}

bool seesColor(ToggleColor color) {
    double diff = std::fabs(optical.get_hue() - hueOf(color));
    if (diff > 180) diff = 360 - diff; // hue wraps, 359 and 1 are 2 apart not 358
    return diff <= HUE_TOLERANCE;      // matters for us, red sits at 353 right next to the wrap
}

// one pass of the auto behaviour
// driver control calls it every loop, so it decides what the motor does right now and returns
void autoToggle(ToggleColor color) {
    const bool somethingClose = optical.get_proximity() > TOGGLE_MIN_PROXIMITY;

    if (somethingClose && !seesColor(color)) {
        toggle.move(TOGGLE_SPEED); // toggle is there, wrong color, keep spinning
    } else {
        toggle.move(0); // nothing there, or we already found our color
    }
}

// failsafe for a dead or lying optical sensor
// look what a broken sensor does to the code above: seesColor never comes back true, so the
// toggle spins at every toggle it meets and never stops
// A cuts the sensor out so that can't happen, driver spins it by hand instead
// stays latched until A again, even if driver control restarts
static bool toggleManual = false;

bool toggleManualMode() { return toggleManual; } // screen.cpp reads this to show the warning

// ---- program start ----

void initialize() {
    screenShowSplash(); // logo up now, pulses until calibration finishes

    // groups were built all blue, fix the middle motor of each side here
    // a MotorGroup only takes one gearset, which is why it couldn't be set up top
    leftMotors.set_gearing(pros::MotorGears::green, 1);
    rightMotors.set_gearing(pros::MotorGears::green, 1);

    cascade.set_brake_mode(pros::MotorBrake::coast); // so it can be pushed down by hand pre match
    cascade.tare_position();                         // call this zero, so the cascade MUST start fully down

    optical.set_led_pwm(100); // sensor light full blast, keeps hue off room lighting

    // how long the sensor gathers light per reading
    // default 100ms = a new color only 10x a second, too slow for a spinning toggle
    optical.set_integration_time(20);

    toggle.set_brake_mode(pros::MotorBrake::hold); // stop ON the color instead of coasting past
    claw.set_brake_mode(pros::MotorBrake::hold);   // so auton can brake() it and it stays put

    chassis.calibrate();  // couple of seconds, keep the robot still
    screenShowSelector(); // done calibrating, swap the logo for the selector
}

void disabled() {}

void competition_initialize() {}

// ---- autonomous ----

void autonomous() {
    cascade.set_brake_mode(pros::MotorBrake::hold); // route needs the cascade to stay where it stops

    const AutonRoute& slot = selectedRoute();      // whichever slot is picked on the brain screen
    if (slot.run != nullptr) slot.run(slot.color); // slots with no route yet are nullptr, so check first
}

// ---- driver control ----
//   left stick   drive, right stick turn
//   R1 / R2      claw one way / the other (hold)
//   L1 / L2      cascade up / down (hold), stops at top and bottom
//   L1 tap       cascade drives itself to scoring height, any hold cancels it
//   up / down    spin the toggle by hand (hold), ignores the optical sensor
//   A            failsafe latch, see above
//   B + down     run the selected auton, practice only. dead once a comp switch is plugged in
//   toggle       automatic the rest of the time

void opcontrol() {
    // hold, not coast, or the cascade sags the second the driver lets go of L1
    // initialize() leaves it on coast so it can be pushed down by hand before the match
    cascade.set_brake_mode(pros::MotorBrake::hold);

    // no tare here on purpose. auton can hand over with the cascade raised, and re-zeroing at
    // that height would drag CASCADE_TOP and CASCADE_BOTTOM up with it

    // press shorter than this = tap, longer = hold
    // L1 does nothing during the window, which is what keeps a tap from nudging the cascade
    // before the macro takes over. lower it if holding feels laggy
    constexpr std::uint32_t TAP_MS = 200;
    constexpr std::uint32_t MACRO_GIVE_UP_MS = 2000; // stop pushing if it hasn't got there by now

    std::uint32_t upPressedAt = 0; // when L1 last went down
    bool cascadeMacro = false;     // true while the motor is driving itself to CASCADE_SCORE

    while (true) {
        // hold B + DOWN to run the selected auton from here, same trick EZ-Template uses in
        // ez_template_extras(). saves driving to the field and re-plugging a comp switch to test
        // is_connected() is the safety: plug into a comp switch or field control and this whole
        // block goes dead, so it can't fire mid match
        // checked before anything else so DOWN doesn't also spin the toggle on the way in
        if (!pros::competition::is_connected() &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {

            const pros::MotorBrake before = cascade.get_brake_mode();
            autonomous();                   // blocks for the whole route
            cascade.set_brake_mode(before); // autonomous() leaves it on hold, put it back to coast

            // wait for both to come up, otherwise it just runs again the instant the route ends
            while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) ||
                   controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                pros::delay(10);
            }
            continue; // skip the rest of this pass, every reading is stale after a long route
        }

        // arcade: left stick speed, right stick turn
        chassis.arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        // claw
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            claw.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            claw.move(-63);
        } else {
            claw.move(0); // neither held
        }

        // new_press fires once on the press, not the whole time A is held
        // that's what makes A a switch instead of flipping 100x a second
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            toggleManual = !toggleManual;
            controller.rumble(toggleManual ? "-" : "."); // driver feels it without looking down
        }

        const bool spinFwd = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        const bool spinBack = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

        if (spinFwd) {
            toggle.move(TOGGLE_SPEED); // checked first, so a held arrow always beats the sensor
        } else if (spinBack) {
            toggle.move(-TOGGLE_SPEED);
        } else if (toggleManual) {
            toggle.move(0); // sensor cut out, toggle only moves when asked
        } else {
            autoToggle(allianceColor()); // normal case, aim for our alliance color
        }

        // cascade
        // tap L1 -> drives itself to CASCADE_SCORE, hold L1 -> manual up like normal
        // limits stop the driver winding the string past the top or unwinding past the bottom,
        // either one wraps it back on itself
        const double pos = cascade.get_position();
        const bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        const bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            upPressedAt = pros::millis(); // L1 just went down, start the clock
        }

        // let go fast enough to count as a tap, so hand the job to the motor's own position
        // controller. it drives there and holds, no PID loop of ours needed
        if (controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L1) &&
            pros::millis() - upPressedAt < TAP_MS) {
            cascade.move_absolute(CASCADE_SCORE, CASCADE_RPM);
            cascadeMacro = true;
        }

        // held past the tap window, so the driver means manual
        const bool holdingUp = up && pros::millis() - upPressedAt >= TAP_MS;

        if (holdingUp) {
            cascadeMacro = false; // manual always wins over the macro
            if (pos < CASCADE_TOP) {
                cascade.move(127);
            } else {
                cascade.brake();
            }
        } else if (down) {
            cascadeMacro = false;

            // full speed until the last stretch, then crawl, so it coasts to a stop above the
            // floor instead of overshooting it at -127
            const double room = pos - CASCADE_BOTTOM;
            if (room <= 0) {
                cascade.brake();
            } else if (room < CASCADE_SLOW_ZONE) {
                cascade.move(-CASCADE_SLOW_POWER);
            } else {
                cascade.move(-127);
            }
        } else if (cascadeMacro) {
            // the motor is driving itself, so there's nothing to do but let it
            // unless it's been too long, which means it can't get there and is just pushing
            if (pros::millis() - upPressedAt > MACRO_GIVE_UP_MS) {
                cascadeMacro = false;
                cascade.brake();
            }
        } else {
            cascade.brake(); // nothing held, no macro, so sit where it is
        }

        pros::delay(10); // let other tasks run, a loop with no delay starves the whole brain
    }
}
