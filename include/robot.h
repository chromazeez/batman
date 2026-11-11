#pragma once

// everything the three .cpp files share
// each .cpp is compiled on its own and knows nothing about the others, so anything one file
// defines and another uses gets announced here first
// extern = this exists, it's just built somewhere else (comment says where)

#include "main.h"         // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep

// ---- devices + chassis, built in main.cpp ----

extern pros::Controller controller;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Motor claw;
extern pros::Motor toggle;
extern pros::MotorGroup cascade;
extern pros::Imu imu;
extern pros::Rotation horizontalEnc;
extern pros::Optical optical;
extern lemlib::Chassis chassis;

// ---- toggle ----

// toggle motor power out of 127
// deliberately slow, fast spinning shoots past the color before the sensor can react
constexpr int TOGGLE_SPEED = 60;

// which color the toggle mech is aiming for
// enum class not plain ints, so a color can't get mixed up with some other number
enum class ToggleColor { red, blue, yellow };

// color name for the brain screen
// lives in the header because main.cpp and screen.cpp both want it
// inline = ok to define in a header, linker won't complain about it twice
inline const char* nameOf(ToggleColor color) {
    switch (color) {
        case ToggleColor::blue: return "BLUE";
        case ToggleColor::yellow: return "YELLOW";
        default: return "RED";
    }
}

bool seesColor(ToggleColor color); // is the sensor looking at that color right now

// same name twice, C++ picks one by how many args you pass (overloading)
void autoToggle(ToggleColor color);                        // one driver control pass, returns right away
void autoToggle(ToggleColor color, std::uint32_t timeout); // blocks until it sees the color, for routes

bool toggleManualMode(); // true once the driver latched the sensor out with A

// ---- cascade ----
// position = motor degrees from wherever it sat at power on, so it has to start fully down for
// any of these numbers to mean anything

constexpr double CASCADE_TOP = 10000; // top of travel, driver control won't go past it or below 0

// scoring height, used by the skills route AND the L1 tap in driver control
// named instead of inlined on purpose, it lives in two files and they have to agree
constexpr double CASCADE_SCORE = 855;

// speed cap for move_absolute, rpm. green cartridge tops out at 200
// shared so the auton and the L1 tap raise the cascade at the same rate
constexpr int CASCADE_RPM = 200;

// the cascade starts with string already wound on, so 0 is not the real floor
// stop above it, and crawl the last stretch so momentum can't unwind that first wrap
constexpr double CASCADE_BOTTOM = 30;     // degrees above the tare point to stop at
constexpr double CASCADE_SLOW_ZONE = 200; // start easing off this far above the floor
constexpr int CASCADE_SLOW_POWER = 40;    // crawl speed inside that zone

// ---- routes, written in autons.cpp ----

void skill_route(ToggleColor color);

// tuning routes are commented out in autons.cpp, kept there for the notebook

// ---- auton selector ----

// a route is "function taking a color, returning nothing"
// naming that type with `using` keeps the struct readable
// without it the line reads void (*run)(ToggleColor), same thing written the hard way
using RouteFn = void (*)(ToggleColor);

// one slot in the selector
// storing the function itself means the selector runs whichever route is picked, no big if/else
struct AutonRoute {
    const char* name;  // what the screen shows
    ToggleColor color; // our alliance for that slot, used by the route AND driver control
    RouteFn run;       // route to call, or nullptr for a slot with no route yet
};

extern const AutonRoute AUTON_ROUTES[]; // the slot list, in autons.cpp
extern const int AUTON_ROUTE_COUNT;

// ---- brain screen, built in screen.cpp ----

void screenShowSplash();   // pulsing logo, for while the IMU calibrates
void screenShowSelector(); // swaps the splash for the selector

const AutonRoute& selectedRoute(); // whichever slot is picked right now
ToggleColor allianceColor();       // that slot's color, so driver control aims the toggle right
