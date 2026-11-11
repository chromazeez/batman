// autonomous routes
// to add one: write the function here, declare it in robot.h, add a line to AUTON_ROUTES at the
// bottom so it shows up on the selector

#include "robot.h"
#include <cmath>
#include <iterator> // std::size

// ---- building blocks ----
// both block until done, fine in auton since nothing else needs the controller for 15 seconds
// both take a timeout so a jammed mech or bad sensor reading can't stall the whole route

// spin the toggle until the sensor reads `color`, or the timeout runs out
void autoToggle(ToggleColor color, std::uint32_t timeout) {
    const std::uint32_t start = pros::millis();

    while (!seesColor(color) && pros::millis() - start < timeout) {
        toggle.move(TOGGLE_SPEED);
        pros::delay(10);
    }

    toggle.move(0);
}

constexpr double CASCADE_TOLERANCE = 10;     // degrees, close enough to call it arrived
constexpr double CASCADE_STALL_DEGREES = 3;  // moved less than this, for
constexpr std::uint32_t CASCADE_STALL_MS = 100; // this long = against a hard stop, quit

// drive the cascade to a position in motor degrees
// ignores CASCADE_TOP unlike driver control, so a route can be given any target
// leaves the motors braked where they stop, since autonomous() sets brake mode to hold
// rpm defaults to the same speed the L1 tap uses, pass a lower one to go gently
static void cascadeMoveTo(double target, std::uint32_t timeout, int rpm = CASCADE_RPM) {
    cascade.move_absolute(target, rpm);

    const std::uint32_t start = pros::millis();
    double lastPos = cascade.get_position();
    std::uint32_t lastMoved = start;

    // move_absolute doesn't block, so sit here until it arrives
    while (pros::millis() - start < timeout) {
        const double pos = cascade.get_position();
        if (std::fabs(target - pos) <= CASCADE_TOLERANCE) break; // close enough

        // stopped climbing, so it's against a hard stop. quit instead of cooking the motors
        if (std::fabs(pos - lastPos) > CASCADE_STALL_DEGREES) {
            lastPos = pos;
            lastMoved = pros::millis();
        } else if (pros::millis() - lastMoved > CASCADE_STALL_MS) {
            break;
        }

        pros::delay(10);
    }

    cascade.brake(); // cancels the move_absolute and holds where it is
}

// LemLib has no "drive forward 12 inches", every motion takes an absolute field point
// so work the point out from where the robot is now. heading 0 is +y, so forward is (sin, cos)
// negative dist backs up instead of spinning round to face the target
static void driveInches(float dist, int timeout, lemlib::MoveToPointParams params = {}) {
    const lemlib::Pose pose = chassis.getPose(true); // true = theta in radians
    const float x = pose.x + dist * std::sin(pose.theta);
    const float y = pose.y + dist * std::cos(pose.theta);

    params.forwards = dist >= 0;
    chassis.moveToPoint(x, y, timeout, params, false);
}

// turnToHeading is absolute, so add to the current heading to turn relative
// positive is right, negative is left
static void turnBy(float degrees, int timeout, lemlib::TurnToHeadingParams params = {}) {
    chassis.turnToHeading(chassis.getPose().theta + degrees, timeout, params, false);
}

// ---- routes ----
void skill_route(ToggleColor color) {
    chassis.setPose(0, 0, 0);

    autoToggle(color, 5000);

    chassis.moveToPoint(0, 7.25, 1000, {.minSpeed = 60, .earlyExitRange = 1});
    // async, so it climbs during the turn and the drive below instead of the robot waiting for it
    cascade.move_absolute(CASCADE_SCORE, CASCADE_RPM);
    // slower than the tuned gains would run it, the raised cascade makes the robot top heavy
    chassis.turnToHeading(90, 1500, {.maxSpeed = 70}, false);
    // no minSpeed here on purpose, it would switch off the slew set in linearController
    // slew ramps the power up instead of slamming it, which matters with the cascade raised
    driveInches(12.5, 2000, {.maxSpeed = 60});

    // move_absolute returns right away, so the cascade keeps lowering while the claw opens
    // 80 rpm instead of the usual CASCADE_RPM, gravity makes the way down too quick at full speed
    cascadeMoveTo(CASCADE_BOTTOM + 250, 2500, 80); // slow on the way down, blocks until it's there

    claw.move(-127); // open, same direction as R2 in driver control
    pros::delay(300);

    driveInches(-12, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    turnBy(-45, 1500);

    cascadeMoveTo(CASCADE_BOTTOM, 2500, 80); 

    claw.move(127); // intake, R1's direction. left running for the rest of the route
    driveInches(28, 3000, {.minSpeed = 60, .earlyExitRange = 1});
    driveInches(3, 1500, {.maxSpeed = 37});

    // zero the heading here so the 135 below is measured from a clean reference
    const lemlib::Pose here = chassis.getPose();
    chassis.setPose(here.x, here.y, 0);

    // 20% above the scoring height. async, so it keeps rising through the turn below
    cascade.move_absolute(CASCADE_SCORE * 1.6, CASCADE_RPM);

    claw.move(0); // voltage 0, so it coasts through the turn

    // absolute now that the heading is zeroed above
    chassis.turnToHeading(135, 3000, {.maxSpeed = 70}, false);

    claw.move(127); // back to intaking, you wanted it spinning after the 135
    pros::delay(20);
    driveInches(12, 2000, {.minSpeed = 60, .earlyExitRange = 2});
}

// ---- tuning routes, kept for the notebook ----
// commented out so they can't be picked on the selector by accident at a competition
// ran these with B + DOWN and read X/Y/Theta off the brain screen
//
// results 2026-11-10:
//   tune_direction  single 90 turn. landed 90.91, clean. X -2.66, Y 4.91
//   tune_turns      90 / -90 / 180 chained. 180 landed 179.04
//   tune_straight   24" target, drove 24.25" measured. confirms the 2.75" drive wheel
//
// void tune_direction(ToggleColor) {
//     chassis.setPose(0, 0, 0);
//     chassis.turnToHeading(90, 3000, {}, false);
// }
//
// void tune_turns(ToggleColor) {
//     chassis.setPose(0, 0, 0);
//     chassis.turnToHeading(90, 3000, {}, false);
//     pros::delay(75);
//     chassis.turnToHeading(-90, 3000, {}, false);
//     pros::delay(75);
//     chassis.turnToHeading(180, 3000, {}, false);
// }
//
// void tune_straight(ToggleColor) {
//     chassis.setPose(0, 0, 0);
//     chassis.moveToPoint(0, 24, 4000, {}, false);
// }

// ---- selector list ----
// what the brain screen cycles through, in order
// the color on each line is also the alliance driver control aims at, so a red and a blue slot
// can share one route function and still behave differently

const AutonRoute AUTON_ROUTES[] = {
    {"Skills", ToggleColor::red, skill_route},
    {"Red Alliance", ToggleColor::red, nullptr},   // TODO: write a match route, put it here
    {"Blue Alliance", ToggleColor::blue, nullptr}, // TODO: same for blue
};

const int AUTON_ROUTE_COUNT = std::size(AUTON_ROUTES); // counted from the array, so adding a line is the only edit
