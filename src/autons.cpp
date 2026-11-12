// autonomous routes
// to add one: write the function here, declare it in robot.h, add a line to AUTON_ROUTES at the
// bottom so it shows up on the selector

#include "robot.h"
#include <algorithm> // std::sort
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

// ---- wall resets ----
// a distance sensor reads straight out from where it's bolted. square the robot up to a field
// wall and that reading, plus how far the sensor sits from the tracking center, gives one axis
// of our position absolutely. that's how you undo odometry drift partway through a long route
//
// only call these when the robot is stopped and square to a wall. a reading taken at an angle
// measures the hypotenuse, not the distance to the wall, and comes out too long
//
// mounting, from 38535A's TitanReset guide:
//   square to the robot, no tilt, or the line of sight is off
//   high enough off the ground that game elements don't get read instead of the wall
//   nothing on the robot in front of the sensor
//
// call after moves that cause drift, not after every move. swings and pose moves drift most

constexpr double FRONT_SENSOR_OFFSET = 7; // TODO measure: tracking center out to the sensor face
constexpr double SIDE_SENSOR_OFFSET = 6;  // TODO measure: same for the side one

// thresholds from 38535A's TitanReset docs: "relatively close to the field < 40 inches" and
// "relatively square to the field ~10 deg tolerance". 7 degrees is a bit tighter than theirs
constexpr int WALL_MIN_CONFIDENCE = 60;     // out of 63
constexpr int WALL_MAX_MM = 1000;           // 40 inches, past that the reading gets unreliable
constexpr double WALL_SQUARE_TOLERANCE = 7; // degrees off square before we ignore the reading

constexpr int WALL_SAMPLES = 9;         // readings to take per reset
constexpr double WALL_MAX_SPREAD = 1.0; // inches between the lowest and highest before we bail

static bool wallReadingOk(pros::Distance& sensor) {
    const int mm = sensor.get_distance();
    return mm > 0 && mm < WALL_MAX_MM && sensor.get_confidence() >= WALL_MIN_CONFIDENCE;
}

// one reading is noisy, so take nine and use the middle one. a stray spike ends up at one end
// of the sorted list and can't drag the middle anywhere
// writes inches to `out`, returns false if the readings weren't good enough to use
static bool sampleWall(pros::Distance& sensor, double& out) {
    double s[WALL_SAMPLES];
    int n = 0;

    for (int i = 0; i < WALL_SAMPLES; i++) {
        if (wallReadingOk(sensor)) s[n++] = sensor.get_distance() / 25.4; // mm to inches
        pros::delay(10);
    }
    if (n < WALL_SAMPLES / 2) return false; // most of them were junk

    std::sort(s, s + n);
    if (s[n - 1] - s[0] > WALL_MAX_SPREAD) return false; // they disagree, something moved

    out = s[n / 2];
    return true;
}

// snap the Y part of our pose to a wall. squareHeading is the heading we should be at for this
// sensor to be pointing at that wall. returns false if it didn't trust the reading
static bool resetYFromWall(pros::Distance& sensor, double offset, double wallY, double squareHeading) {
    const lemlib::Pose pose = chassis.getPose();
    if (std::fabs(lemlib::angleError(squareHeading, pose.theta, false)) > WALL_SQUARE_TOLERANCE) return false;

    double reading;
    if (!sampleWall(sensor, reading)) return false;

    const double gap = reading + offset;
    const double fixed = (wallY > pose.y) ? wallY - gap : wallY + gap; // odom says which side we're on

    chassis.setPose(pose.x, fixed, pose.theta);
    return true;
}

static bool resetXFromWall(pros::Distance& sensor, double offset, double wallX, double squareHeading) {
    const lemlib::Pose pose = chassis.getPose();
    if (std::fabs(lemlib::angleError(squareHeading, pose.theta, false)) > WALL_SQUARE_TOLERANCE) return false;

    double reading;
    if (!sampleWall(sensor, reading)) return false;

    const double gap = reading + offset;
    const double fixed = (wallX > pose.x) ? wallX - gap : wallX + gap;

    chassis.setPose(fixed, pose.y, pose.theta);
    return true;
}

// lift heights, named for how many pins they clear
constexpr double CASCADE_2PIN = CASCADE_SCORE * 1.76;
constexpr double CASCADE_3PIN = CASCADE_SCORE * 2.5;

// ---- routes ----
void skill_route(ToggleColor color) {
    chassis.setPose(0, 0, 0);

    autoToggle(color, 5000);

    // minSpeed holds the power up, PID alone fades to nothing over 7 inches and lands short
    // blocking, then a delay to cover the coast, so the turn can't start while it's still rolling
    driveInches(7.5, 1500, {.minSpeed = 60, .earlyExitRange = 1});
    pros::delay(100);
    // async, so it climbs during the turn and the drive below instead of the robot waiting for it
    cascade.move_absolute(CASCADE_SCORE, CASCADE_RPM);
    // slower than the tuned gains would run it, the raised cascade makes the robot top heavy
    chassis.turnToHeading(90, 1500, {.maxSpeed = 40}, false);
    // no minSpeed here on purpose, it would switch off the slew set in linearController
    // slew ramps the power up instead of slamming it, which matters with the cascade raised
    driveInches(14, 2000, {.maxSpeed = 50});

    // move_absolute returns right away, so the cascade keeps lowering while the claw opens
    // 80 rpm instead of the usual CASCADE_RPM, gravity makes the way down too quick at full speed
    cascadeMoveTo(CASCADE_BOTTOM + 300, 2500, 80); // slow on the way down, blocks until it's there

    claw.move(-127); // open, same direction as R2 in driver control
    pros::delay(700);

    // the aligner just squared us to the goal, so this is the one place in the route where we
    // physically know where the robot is. call it zero and throw away the drift up to here
    // every heading below is measured from this, which is why they're all 90 less than before
    chassis.setPose(0, 0, 0);

    driveInches(-12.5, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    // -46, same turn the old absolute 44 was in the old frame
    chassis.turnToHeading(-46, 1500, {}, false);

    cascadeMoveTo(CASCADE_BOTTOM, 2500, 80); 

    claw.move(127); // intake, R1's direction. left running for the rest of the route
    driveInches(33.5, 4000, {.maxSpeed = 20});

    cascade.move_absolute(CASCADE_2PIN, CASCADE_RPM); // async, keeps rising through the turn

    // claw left running from the drive above, so it keeps intaking through the whole turn
    chassis.turnToHeading(90, 3000, {.maxSpeed = 56}, false); // -46 + 135, was 180 before the zero

    // no minSpeed, so the slew applies. same maxSpeed as the scoring drive
    driveInches(24, 2000, {.maxSpeed = 60});

    // ---- second cycle ----
    cascadeMoveTo(CASCADE_BOTTOM + 200, 2500, 80);

    driveInches(-24, 2000, {.minSpeed = 60, .earlyExitRange = 2});
    chassis.turnToHeading(45, 1500, {}, false); // 90 - 45, was 135

    cascadeMoveTo(CASCADE_BOTTOM, 2500, 80);

    claw.move(127);
    driveInches(27, 3000, {.minSpeed = 50, .earlyExitRange = 1});
    driveInches(4, 1500, {.maxSpeed = 25});

    pros::delay(10);
    claw.move(0);
    cascade.move_absolute(CASCADE_3PIN, CASCADE_RPM);
    // 45 + 135 is 180. slower than the 2pin turn, it's carrying more height
    chassis.turnToHeading(180, 3000, {.maxSpeed = 48}, false);
    claw.move(127);

    driveInches(12, 2000, {.maxSpeed = 60});
    cascadeMoveTo(CASCADE_BOTTOM + 250, 2500, 80);
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
