// the brain screen
//
// PROS ships a text-only screen library called LLEMU, what most teams use
// we draw with LVGL underneath it instead, since LLEMU only does 8 lines of text and we wanted
// the logo in color
// two states: pulsing logo while the IMU calibrates, then the auton selector
//
// one rule holds this file together: everything touching an LVGL object runs inside LVGL's own
// task. buttons do because LVGL calls their handlers itself, text does because an lv_timer runs
// inside LVGL too. no other file touches the screen, so two tasks can never be drawing and
// editing the same object at once

#include "robot.h"
#include "liblvgl/lvgl.h" // IWYU pragma: keep
#include <algorithm>      // std::sort
#include <cmath>
#include <cstdio>
#include <iterator> // std::size

// unnamed namespace = everything inside is private to this file, same as marking each one static
namespace {

constexpr int SCREEN_W = 480; // V5 brain screen, in pixels
constexpr int SCREEN_H = 240;

// only the right half of the bat is written down, clockwise from the bottom of the body
// buildBat() mirrors it for the left half, so the two sides can't drift apart as numbers change
constexpr lv_point_t BAT_HALF[] = {
    {240, 171}, // bottom of the body, sits on the mirror line
    {251, 152}, {262, 142}, {271, 139},
    {279, 144}, // first point of the wing's trailing edge
    {291, 133}, {303, 128}, {315, 127},
    {327, 132}, // second point
    {343, 123}, {359, 115}, {373, 107},
    {387, 99},  // wingtip
    {359, 87},  {332, 82},  {307, 83},
    {282, 90},  // top of the wing, meets the ear
    {273, 66},  // ear tip
    {258, 93},  // inside of the ear
    {249, 93},
    {240, 105}, // middle of the head, back on the mirror line
};

constexpr int HALF_COUNT = std::size(BAT_HALF);
constexpr int BAT_COUNT = HALF_COUNT * 2 - 2; // minus 2, the end points already sit on the mirror line

constexpr float BAT_CX = 240.0f; // middle of the bat, so buildBat() scales about its own centre
constexpr float BAT_CY = 118.0f;

constexpr uint32_t YELLOW = 0xFFD800;
constexpr uint32_t BLACK = 0x000000;

constexpr int OVAL_CX = 240; // yellow ellipse behind the full size logo
constexpr int OVAL_CY = 120;
constexpr int OVAL_A = 190; // half the width
constexpr int OVAL_B = 78;  // half the height

constexpr float BADGE_SCALE = 0.65f; // small badge on the selector
constexpr int BADGE_CX = 102;
constexpr int BADGE_CY = 54;

// file scope because the timers and button handlers all need to reach these, and LVGL calls
// those whenever it likes rather than from one place we control
lv_obj_t* splashObj = nullptr;
lv_timer_t* flashTimer = nullptr;
uint8_t ovalOpa = LV_OPA_COVER; // how solid the ellipse draws, stepped by the pulse
int flashStep = 0;

lv_obj_t* selectorObj = nullptr;
lv_timer_t* uiTimer = nullptr;
lv_obj_t* autonLabel = nullptr;
lv_obj_t* slotLabel = nullptr;
lv_obj_t* lockLabel = nullptr;
lv_obj_t* failsafeLabel = nullptr;
lv_obj_t* telemetryLabel = nullptr;
lv_obj_t* lockBtnLabel = nullptr;

int autonIndex = 0;       // which slot of AUTON_ROUTES is picked
bool autonLocked = false; // true once the driver locked the choice in

uint32_t tintOf(ToggleColor color) {
    switch (color) {
        case ToggleColor::blue: return 0x0085B3;
        case ToggleColor::yellow: return 0xEFCD00;
        default: return 0xEB001C;
    }
}

// ---- filling shapes ----
//
// LVGL draws rectangles but has no "fill this outline" call, so we do it a row of pixels at a
// time. per row: find every x where an edge crosses it, sort them left to right, fill between
// 1st and 2nd, 3rd and 4th, and so on
//
// filling in pairs works because crossing the outline takes you from outside the shape to inside,
// crossing again takes you back out. a row through the bat's ears gives 4 crossings and 2 filled
// runs, leaving the gap between the ears alone
//
// nothing stored but the point list, which is why the logo costs ~1KB instead of the tens an
// image would

void drawSpan(lv_layer_t* layer, lv_draw_rect_dsc_t* dsc, int x1, int x2, int y) {
    if (x2 < x1) return;

    lv_area_t area;
    area.x1 = x1;
    area.y1 = y;
    area.x2 = x2;
    area.y2 = y; // same row top and bottom, so the rect is 1 pixel tall
    lv_draw_rect(layer, dsc, &area);
}

void initFill(lv_draw_rect_dsc_t* dsc, uint32_t color, lv_opa_t opa) {
    lv_draw_rect_dsc_init(dsc); // LVGL fills in its defaults first
    dsc->bg_color = lv_color_hex(color);
    dsc->bg_opa = opa;
    dsc->radius = 0;       // no rounded corners
    dsc->border_width = 0; // no border either, just a solid fill
}

void fillPolygon(lv_layer_t* layer, const lv_point_t* pts, int count, uint32_t color, lv_opa_t opa) {
    lv_draw_rect_dsc_t dsc;
    initFill(&dsc, color, opa);

    int top = pts[0].y; // only bother with rows the shape actually reaches
    int bottom = pts[0].y;
    for (int i = 1; i < count; i++) {
        if (pts[i].y < top) top = pts[i].y;
        if (pts[i].y > bottom) bottom = pts[i].y;
    }

    for (int y = top; y <= bottom; y++) {
        int crossings[16]; // bat never needs more than 4, plenty of room
        int found = 0;

        for (int i = 0; i < count; i++) {
            const lv_point_t& a = pts[i];
            const lv_point_t& b = pts[(i + 1) % count]; // % wraps the last point back to the first

            // one end above the row, one below = this edge crosses it
            // the <= on one side and > on the other counts a shared corner once instead of twice,
            // which would otherwise flip inside/outside the wrong way round
            const bool crosses = (a.y <= y && b.y > y) || (b.y <= y && a.y > y);
            if (!crosses || found >= 16) continue;

            const float howFar = static_cast<float>(y - a.y) / static_cast<float>(b.y - a.y); // 0 at a, 1 at b
            crossings[found++] = a.x + static_cast<int>(howFar * (b.x - a.x));
        }

        std::sort(crossings, crossings + found);

        for (int k = 0; k + 1 < found; k += 2) { // fill 0 to 1, then 2 to 3, etc
            drawSpan(layer, &dsc, crossings[k], crossings[k + 1], y);
        }
    }
}

// same row by row idea, but edges come from the ellipse equation so there's no hunting
void fillOval(lv_layer_t* layer, lv_opa_t opa) {
    lv_draw_rect_dsc_t dsc;
    initFill(&dsc, YELLOW, opa);

    for (int y = -OVAL_B; y <= OVAL_B; y++) {
        const float k = static_cast<float>(y) / OVAL_B;
        const int dx = static_cast<int>(OVAL_A * std::sqrt(1.0f - k * k)); // half the width at this row
        drawSpan(layer, &dsc, OVAL_CX - dx, OVAL_CX + dx, OVAL_CY + y);
    }
}

// stored half scaled about the bat's centre, then the same offsets subtracted instead of added
// for the other half
// rounding once and using that one value both ways is what keeps the halves exactly equal
void buildBat(lv_point_t* out, float scale, int cx, int cy) {
    for (int i = 0; i < HALF_COUNT; i++) {
        out[i].x = cx + std::lroundf((BAT_HALF[i].x - BAT_CX) * scale);
        out[i].y = cy + std::lroundf((BAT_HALF[i].y - BAT_CY) * scale);
    }

    for (int i = HALF_COUNT - 2, k = HALF_COUNT; i >= 1; i--, k++) { // backwards, skipping both ends
        out[k].x = cx - std::lroundf((BAT_HALF[i].x - BAT_CX) * scale);
        out[k].y = cy + std::lroundf((BAT_HALF[i].y - BAT_CY) * scale);
    }
}

// ---- splash ----

void drawSplash(lv_event_t* e) {               // LVGL calls this whenever it repaints the splash
    lv_layer_t* layer = lv_event_get_layer(e); // where LVGL wants the drawing to go

    fillOval(layer, ovalOpa);

    lv_point_t bat[BAT_COUNT];
    buildBat(bat, 1.0f, 240, 118); // full size, centred where the stored points already are
    fillPolygon(layer, bat, BAT_COUNT, BLACK, LV_OPA_COVER);
}

// pulse the ellipse while the IMU calibrates, shows the robot is busy and should be left alone
// bat stays solid black on top, so the logo dims as a whole
void flashCb(lv_timer_t*) {
    constexpr int STEPS = 40; // 40 steps x 40ms = 1.6s per cycle

    flashStep = (flashStep + 1) % STEPS;
    const int tri = flashStep < STEPS / 2 ? flashStep : STEPS - flashStep; // climbs to 20, falls back

    ovalOpa = static_cast<uint8_t>(60 + (tri * (255 - 60)) / (STEPS / 2)); // 60 dimmest, 255 brightest
    lv_obj_invalidate(splashObj);                                          // tell LVGL to repaint it
}

// ---- selector ----

void drawSelector(lv_event_t* e) {
    lv_layer_t* layer = lv_event_get_layer(e);

    lv_point_t bat[BAT_COUNT];
    buildBat(bat, BADGE_SCALE, BADGE_CX, BADGE_CY); // same shape, smaller, over on the left
    fillPolygon(layer, bat, BAT_COUNT, tintOf(AUTON_ROUTES[autonIndex].color), LV_OPA_COVER);
}

void onPrev(lv_event_t*) {
    if (!autonLocked) autonIndex = (autonIndex + AUTON_ROUTE_COUNT - 1) % AUTON_ROUTE_COUNT; // wraps to the last slot
}

void onNext(lv_event_t*) {
    if (!autonLocked) autonIndex = (autonIndex + 1) % AUTON_ROUTE_COUNT; // wraps back to the first
}

// locking matters because the V5 screen is a touchscreen
// without it a palm on the brain while placing the robot could change the route, nobody notices
void onLock(lv_event_t*) { autonLocked = !autonLocked; }

void uiCb(lv_timer_t*) { // LVGL runs this on its own timer, so editing objects here is safe
    char buf[96];
    const AutonRoute& slot = AUTON_ROUTES[autonIndex];

    lv_label_set_text(autonLabel, slot.name);

    snprintf(buf, sizeof(buf), "slot %d/%d   %s", autonIndex + 1, AUTON_ROUTE_COUNT, nameOf(slot.color));
    lv_label_set_text(slotLabel, buf);

    lv_label_set_text(lockLabel, autonLocked ? "LOCKED" : "");
    lv_label_set_text(lockBtnLabel, autonLocked ? "UNLOCK" : "LOCK");
    lv_label_set_text(failsafeLabel, toggleManualMode() ? "TOGGLE: MANUAL" : "");

    const lemlib::Pose pose = chassis.getPose();
    snprintf(buf, sizeof(buf), "X %.2f    Y %.2f\nTheta %.2f    Casc %.0f\nHue %.0f    Prox %d",
             pose.x, pose.y, pose.theta, cascade.get_position(), optical.get_hue(),
             optical.get_proximity());
    lv_label_set_text(telemetryLabel, buf);

    lv_obj_invalidate(selectorObj); // repaint the badge in case the alliance color changed
}

// ---- building the objects ----

lv_obj_t* makeLabel(lv_obj_t* parent, int x, int y, const lv_font_t* font, uint32_t color) {
    lv_obj_t* label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(color), LV_PART_MAIN);
    lv_label_set_text(label, ""); // uiCb fills it in a moment later
    return label;
}

// labelOut hands back the button's text object
// LOCK needs it since its wording changes, the other two pass nullptr
void makeButton(lv_obj_t* parent, int x, const char* text, lv_event_cb_t cb, lv_obj_t** labelOut) {
    lv_obj_t* btn = lv_button_create(parent);
    lv_obj_set_size(btn, 152, 40); // big enough to hit with a finger
    lv_obj_set_pos(btn, x, 194);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x202024), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, 6, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x55555C), LV_PART_MAIN);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr); // call cb when this button is tapped

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_center(label);

    if (labelOut != nullptr) *labelOut = label;
}

// plain black rectangle covering the screen, drawCb paints on top of it
lv_obj_t* makeFullScreen(lv_event_cb_t drawCb) {
    lv_obj_t* obj = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(obj); // drop LVGL's default border, padding, rounded corners
    lv_obj_set_size(obj, SCREEN_W, SCREEN_H);
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_style_bg_color(obj, lv_color_hex(BLACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_event_cb(obj, drawCb, LV_EVENT_DRAW_MAIN, nullptr); // our drawing, on every repaint
    return obj;
}

} // namespace

// ---- what the rest of the robot calls ----

void screenShowSplash() {
    if (splashObj != nullptr) return; // already up

    splashObj = makeFullScreen(drawSplash);
    flashTimer = lv_timer_create(flashCb, 40, nullptr); // run flashCb every 40ms
}

void screenShowSelector() {
    if (flashTimer != nullptr) { // clear the splash away first
        lv_timer_delete(flashTimer);
        flashTimer = nullptr;
    }
    if (splashObj != nullptr) {
        lv_obj_delete(splashObj); // deleting an object deletes its children too
        splashObj = nullptr;
    }

    if (selectorObj != nullptr) return;

    selectorObj = makeFullScreen(drawSelector);

    // fixed width + centred text puts 393B in the middle of the badge whatever width LVGL decides
    // the text is, steadier than guessing an offset off the font size
    lv_obj_t* team = makeLabel(selectorObj, BADGE_CX - 80, BADGE_CY - 13, &lv_font_montserrat_24, 0xFFFFFF);
    lv_obj_set_width(team, 160);
    lv_obj_set_style_text_align(team, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(team, "393B");

    autonLabel = makeLabel(selectorObj, 212, 12, &lv_font_montserrat_20, 0xFFFFFF);
    slotLabel = makeLabel(selectorObj, 212, 42, &lv_font_montserrat_16, 0x9A9AA0);
    lockLabel = makeLabel(selectorObj, 212, 66, &lv_font_montserrat_16, YELLOW);
    failsafeLabel = makeLabel(selectorObj, 212, 88, &lv_font_montserrat_16, 0xEB001C);
    telemetryLabel = makeLabel(selectorObj, 12, 112, &lv_font_montserrat_16, 0x9A9AA0);

    makeButton(selectorObj, 8, "< PREV", onPrev, nullptr);
    makeButton(selectorObj, 164, "LOCK", onLock, &lockBtnLabel);
    makeButton(selectorObj, 320, "NEXT >", onNext, nullptr);

    uiTimer = lv_timer_create(uiCb, 50, nullptr); // refresh the text 20x a second
    uiCb(nullptr);                                // and once right now, so it's never blank
}

const AutonRoute& selectedRoute() { return AUTON_ROUTES[autonIndex]; }

ToggleColor allianceColor() { return AUTON_ROUTES[autonIndex].color; }
