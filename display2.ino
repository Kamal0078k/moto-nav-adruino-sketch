#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <NimBLEDevice.h>
#include <math.h>

#include "display_protocol.h"

// ESP32-C3 -> 1.8-inch ST7735 module (hardware SPI):
// GPIO 4 -> SCK, GPIO 6 -> SDA/MOSI, GPIO 7 -> CS,
// GPIO 2 -> A0/DC, GPIO 3 -> RESET. Connect GND to GND and power VCC/LED
// according to the module markings (3.3 V is safe for this ESP32-C3 setup).
#ifndef TFT_WIDTH
#define TFT_WIDTH 128
#endif
#ifndef TFT_HEIGHT
#define TFT_HEIGHT 160
#endif
#ifndef TFT_SCLK_PIN
#define TFT_SCLK_PIN 4
#endif
#ifndef TFT_MOSI_PIN
#define TFT_MOSI_PIN 6
#endif
#ifndef TFT_CS_PIN
#define TFT_CS_PIN 7
#endif
#ifndef TFT_DC_PIN
#define TFT_DC_PIN 2
#endif
#ifndef TFT_RST_PIN
#define TFT_RST_PIN 3
#endif
#ifndef TFT_ROTATION
#define TFT_ROTATION 0
#endif
#ifndef TFT_INIT_OPTION
#define TFT_INIT_OPTION INITR_BLACKTAB
#endif

#ifndef START_IN_BLE_MODE
#define START_IN_BLE_MODE 1
#endif

#if TFT_ROTATION != 0 && TFT_ROTATION != 2
#error "display2 must use portrait rotation 0 or 2"
#endif

const uint16_t TFT_BACKGROUND = ST7735_BLACK;
const uint16_t TFT_FOREGROUND = ST7735_WHITE;

Adafruit_ST7735 tft(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);

// The old SSD1306 assembled each screen in RAM before transferring it. Keep that
// behaviour on the TFT so animations do not expose partially drawn frames.
class BufferedTftDisplay : public GFXcanvas16 {
 public:
  BufferedTftDisplay() : GFXcanvas16(TFT_WIDTH, TFT_HEIGHT) {}

  void clearDisplay() {
    fillScreen(TFT_BACKGROUND);
  }

  void display() {
    if (getBuffer()) {
      tft.drawRGBBitmap(0, 0, getBuffer(), TFT_WIDTH, TFT_HEIGHT);
    }
  }
};

BufferedTftDisplay display;

struct Point { float x; float y; };

enum ManeuverType : uint8_t {
  MANEUVER_STRAIGHT, MANEUVER_LEFT, MANEUVER_RIGHT, MANEUVER_ARRIVE
};

enum NavigationState : uint8_t {
  STATE_DISCONNECTED, STATE_ACQUIRING_GPS, STATE_NAVIGATING,
  STATE_REROUTING, STATE_ARRIVED, STATE_DIAGNOSTIC
};

struct ManeuverEvent {
  size_t waypointIndex;
  ManeuverType type;
};

struct RouteScenario {
  const char* name;
  const Point* route;
  size_t routeCount;
  const Point* positions;
  size_t positionCount;
  const ManeuverEvent* maneuvers;
  size_t maneuverCount;
};

struct RouteProgress {
  size_t segment;
  float segmentFraction;
  Point nearest;
};

// Starts 1.2 km before a right turn to test km and metre formatting.
const Point RIGHT_ROUTE[] = {
  {0, 0}, {0, 400}, {0, 800}, {0, 1200}, {60, 1200}, {120, 1200}
};
const Point RIGHT_POSITIONS[] = {
  {0, 0}, {0, 200}, {0, 400}, {0, 600}, {0, 800}, {0, 1000},
  {0, 1100}, {0, 1180}, {20, 1200}, {60, 1200}, {100, 1200}, {120, 1200}
};
const ManeuverEvent RIGHT_MANEUVERS[] = {
  {3, MANEUVER_RIGHT}, {5, MANEUVER_ARRIVE}
};

const Point LEFT_ROUTE[] = {
  {0, 0}, {0, 60}, {-50, 60}, {-100, 60}
};
const Point LEFT_POSITIONS[] = {
  {0, 0}, {0, 20}, {0, 40}, {0, 55}, {-15, 60}, {-35, 60},
  {-55, 60}, {-75, 60}, {-95, 60}, {-100, 60}
};
const ManeuverEvent LEFT_MANEUVERS[] = {
  {1, MANEUVER_LEFT}, {3, MANEUVER_ARRIVE}
};

const Point CONSECUTIVE_ROUTE[] = {
  {0, 0}, {0, 70}, {55, 70}, {55, 125}, {10, 125}
};
const Point CONSECUTIVE_POSITIONS[] = {
  {0, 0}, {0, 25}, {0, 50}, {0, 68}, {20, 70}, {45, 70},
  {55, 78}, {55, 100}, {55, 123}, {35, 125}, {12, 125}, {10, 125}
};
const ManeuverEvent CONSECUTIVE_MANEUVERS[] = {
  {1, MANEUVER_RIGHT}, {2, MANEUVER_LEFT},
  {3, MANEUVER_LEFT}, {4, MANEUVER_ARRIVE}
};

const RouteScenario SCENARIOS[] = {
  {
    "RIGHT TURN",
    RIGHT_ROUTE, sizeof(RIGHT_ROUTE) / sizeof(RIGHT_ROUTE[0]),
    RIGHT_POSITIONS, sizeof(RIGHT_POSITIONS) / sizeof(RIGHT_POSITIONS[0]),
    RIGHT_MANEUVERS, sizeof(RIGHT_MANEUVERS) / sizeof(RIGHT_MANEUVERS[0])
  },
  {
    "LEFT TURN",
    LEFT_ROUTE, sizeof(LEFT_ROUTE) / sizeof(LEFT_ROUTE[0]),
    LEFT_POSITIONS, sizeof(LEFT_POSITIONS) / sizeof(LEFT_POSITIONS[0]),
    LEFT_MANEUVERS, sizeof(LEFT_MANEUVERS) / sizeof(LEFT_MANEUVERS[0])
  },
  {
    "CONSECUTIVE",
    CONSECUTIVE_ROUTE, sizeof(CONSECUTIVE_ROUTE) / sizeof(CONSECUTIVE_ROUTE[0]),
    CONSECUTIVE_POSITIONS,
    sizeof(CONSECUTIVE_POSITIONS) / sizeof(CONSECUTIVE_POSITIONS[0]),
    CONSECUTIVE_MANEUVERS,
    sizeof(CONSECUTIVE_MANEUVERS) / sizeof(CONSECUTIVE_MANEUVERS[0])
  }
};

const size_t SCENARIO_COUNT = sizeof(SCENARIOS) / sizeof(SCENARIOS[0]);
const unsigned long FRAME_INTERVAL_MS = 500;
const float METRES_PER_PIXEL = 3.0f;
// Portrait layout, 128 x 160, tuned for a glance from the handlebar:
//   0-51    hero         turn arrow tile + distance to the turn
//   55-125  route map    rider marker near the bottom, road ahead above it
//   128-159 footer       distance remaining | arrival time, or a caller pill during a call
// Screens that are not the route (calls outside a ride, rerouting...) still use the
// 20 px status header below for the call indicator.
const int HEADER_CONTENT_Y = 2;
const int HEADER_BOTTOM = 19;
const int HERO_TOP = 0;
const int HERO_BOTTOM = 51;
const int ROUTE_TOP = HERO_BOTTOM + 4;
const int FOOTER_TOP = 127;
const int ROUTE_BOTTOM = FOOTER_TOP - 2;
const int MARKER_X = TFT_WIDTH / 2;
const int MARKER_Y = ROUTE_TOP + (ROUTE_BOTTOM - ROUTE_TOP) * 5 / 6;
const int BLE_MARKER_Y = MARKER_Y;
const int MARKER_HALF_WIDTH = 8;
const int MARKER_TIP_OFFSET = 11;
const int MARKER_BASE_OFFSET = 6;
// The turn arrow is knocked out (black) of a white rounded tile; no divider lines anywhere.
const int HERO_TILE_SIZE = 48;
const int HERO_TILE_X = 3;
const int HERO_TILE_Y = HERO_TOP + (HERO_BOTTOM - HERO_TOP + 1 - HERO_TILE_SIZE) / 2;
const int HERO_ICON_X = HERO_TILE_X + 4;
const int HERO_ICON_Y = HERO_TILE_Y + 4;
const int HERO_TEXT_LEFT = HERO_TILE_X + HERO_TILE_SIZE + 5;
const int HERO_TEXT_WIDTH = TFT_WIDTH - HERO_TEXT_LEFT - 2;
const int CALL_PILL_X = 4;
const int CALL_PILL_Y = FOOTER_TOP + 1;
const int CALL_PILL_WIDTH = TFT_WIDTH - 2 * CALL_PILL_X;
const int CALL_PILL_HEIGHT = TFT_HEIGHT - CALL_PILL_Y - 2;

// Advertise the TFT's real portrait geometry to the mobile application. Route
// points remain normalized, so the protocol wire format does not change.
const int PROTOCOL_DISPLAY_WIDTH = TFT_WIDTH;
const int PROTOCOL_DISPLAY_HEIGHT = TFT_HEIGHT;

const uint8_t PROGMEM ICON_STRAIGHT[] = {
  0x18, 0x3C, 0x7E, 0x18, 0x18, 0x18, 0x18, 0x18
};
const uint8_t PROGMEM ICON_LEFT[] = {
  0x20, 0x60, 0xE0, 0x60, 0x3E, 0x02, 0x02, 0x02
};
const uint8_t PROGMEM ICON_RIGHT[] = {
  0x04, 0x06, 0x07, 0x06, 0x7C, 0x40, 0x40, 0x40
};
const uint8_t PROGMEM ICON_ARRIVE[] = {
  0x60, 0x5C, 0x6A, 0x5C, 0x60, 0x40, 0x40, 0x40
};
const uint8_t PROGMEM ICON_MUSIC[] = {
  0x0C, 0x0E, 0x0B, 0x09, 0x08, 0x78, 0xF8, 0x70
};
const uint8_t PROGMEM ICON_PHONE_SMALL[] = {
  0xC0, 0xE0, 0x60, 0x60, 0x30, 0x1C, 0x0F, 0x06
};

size_t currentScenarioIndex = 0;
size_t currentFrame = 0;
NavigationState navigationState = STATE_NAVIGATING;
bool simulationPaused = false;
unsigned long lastFrameAt = 0;

enum ApplicationMode : uint8_t { MODE_DEMO, MODE_BLE_RECEIVER };

const char* BLE_DEVICE_NAME = "MotoNav Display";
const char* BLE_SERVICE_UUID = "7d2ea28a-f7bd-485a-bd9d-92ad6ecfe93e";
const char* BLE_FRAME_UUID = "7d2ea28b-f7bd-485a-bd9d-92ad6ecfe93e";
const char* BLE_CAPABILITY_UUID = "7d2ea28c-f7bd-485a-bd9d-92ad6ecfe93e";
const unsigned long STALE_FRAME_MS = 5000;

ApplicationMode applicationMode = START_IN_BLE_MODE ? MODE_BLE_RECEIVER : MODE_DEMO;
navproto::FragmentAssembler fragmentAssembler;
navproto::DisplayFrame pendingBleFrame = {};
portMUX_TYPE assemblerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE pendingFrameMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool pendingBleFrameReady = false;
volatile uint8_t pendingBleFeedback = 0;
volatile uint16_t pendingBleReceivedBytes = 0;
volatile uint16_t pendingBleExpectedBytes = 0;
volatile bool bleConnected = false;
volatile uint8_t bleConnectionEvent = 0;
bool bleInitialized = false;
bool staleFrameShown = false;
unsigned long lastValidBleFrameAt = 0;
navproto::DisplayFrame lastBleFrame = {};
bool hasLastBleFrame = false;
unsigned long activeCallStartedAt = 0;
unsigned long lastDrawnCallSecond = 0xffffffffUL;

float distanceBetween(Point a, Point b) {
  return hypotf(b.x - a.x, b.y - a.y);
}

RouteProgress findRouteProgress(const RouteScenario& scenario, Point current) {
  RouteProgress best = {0, 0.0f, scenario.route[0]};
  float bestDistance = 1000000.0f;

  for (size_t i = 0; i + 1 < scenario.routeCount; i++) {
    Point a = scenario.route[i];
    Point b = scenario.route[i + 1];
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float lengthSquared = dx * dx + dy * dy;
    float t = 0.0f;

    if (lengthSquared > 0.0f) {
      t = ((current.x - a.x) * dx + (current.y - a.y) * dy) /
          lengthSquared;
      t = constrain(t, 0.0f, 1.0f);
    }

    Point nearest = {a.x + t * dx, a.y + t * dy};
    float candidateDistance = distanceBetween(current, nearest);
    if (candidateDistance < bestDistance) {
      bestDistance = candidateDistance;
      best = {i, t, nearest};
    }
  }
  return best;
}

float distanceToWaypoint(
  const RouteScenario& scenario,
  const RouteProgress& progress,
  size_t waypointIndex
) {
  if (waypointIndex <= progress.segment) return 0.0f;

  float distance = distanceBetween(
    progress.nearest, scenario.route[progress.segment + 1]
  );
  for (size_t i = progress.segment + 1; i < waypointIndex; i++) {
    distance += distanceBetween(scenario.route[i], scenario.route[i + 1]);
  }
  return distance;
}

const ManeuverEvent& nextManeuver(
  const RouteScenario& scenario,
  const RouteProgress& progress
) {
  for (size_t i = 0; i < scenario.maneuverCount; i++) {
    if (scenario.maneuvers[i].waypointIndex > progress.segment) {
      return scenario.maneuvers[i];
    }
  }
  return scenario.maneuvers[scenario.maneuverCount - 1];
}

Point toScreen(Point world, Point current, float headingX, float headingY) {
  float dx = world.x - current.x;
  float dy = world.y - current.y;
  float forward = dx * headingX + dy * headingY;
  float right = dx * headingY - dy * headingX;
  return {
    MARKER_X + right / METRES_PER_PIXEL,
    MARKER_Y - forward / METRES_PER_PIXEL
  };
}

uint8_t clipCode(float x, float y) {
  uint8_t code = 0;
  if (x < 0) code |= 1;
  else if (x > TFT_WIDTH - 1) code |= 2;
  if (y < ROUTE_TOP) code |= 4;
  else if (y > ROUTE_BOTTOM) code |= 8;
  return code;
}

bool clipToRouteArea(Point& start, Point& end) {
  while (true) {
    uint8_t startCode = clipCode(start.x, start.y);
    uint8_t endCode = clipCode(end.x, end.y);
    if ((startCode | endCode) == 0) return true;
    if ((startCode & endCode) != 0) return false;

    uint8_t outside = startCode != 0 ? startCode : endCode;
    float x;
    float y;
    if (outside & 4) {
      y = ROUTE_TOP;
      x = start.x + (end.x - start.x) * (y - start.y) / (end.y - start.y);
    } else if (outside & 8) {
      y = ROUTE_BOTTOM;
      x = start.x + (end.x - start.x) * (y - start.y) / (end.y - start.y);
    } else if (outside & 2) {
      x = TFT_WIDTH - 1;
      y = start.y + (end.y - start.y) * (x - start.x) / (end.x - start.x);
    } else {
      x = 0;
      y = start.y + (end.y - start.y) * (x - start.x) / (end.x - start.x);
    }

    if (outside == startCode) start = {x, y};
    else end = {x, y};
  }
}

void drawClippedRouteLine(Point start, Point end) {
  if (!clipToRouteArea(start, end)) return;
  int x0 = (int)roundf(start.x);
  int y0 = (int)roundf(start.y);
  int x1 = (int)roundf(end.x);
  int y1 = (int)roundf(end.y);
  // Five centred strokes keep the route prominent without crowding the guidance panel.
  display.drawLine(x0, y0, x1, y1, TFT_FOREGROUND);
  if (x0 > 0 && x1 > 0) {
    display.drawLine(x0 - 1, y0, x1 - 1, y1, TFT_FOREGROUND);
  }
  if (x0 > 1 && x1 > 1) {
    display.drawLine(x0 - 2, y0, x1 - 2, y1, TFT_FOREGROUND);
  }
  if (x0 < TFT_WIDTH - 1 && x1 < TFT_WIDTH - 1) {
    display.drawLine(x0 + 1, y0, x1 + 1, y1, TFT_FOREGROUND);
  }
  if (x0 < TFT_WIDTH - 2 && x1 < TFT_WIDTH - 2) {
    display.drawLine(x0 + 2, y0, x1 + 2, y1, TFT_FOREGROUND);
  }
}

void drawScaledIcon(const uint8_t* bitmap, int x, int y, int scale, uint16_t color = TFT_FOREGROUND) {
  for (int row = 0; row < 8; row++) {
    uint8_t bits = pgm_read_byte(bitmap + row);
    for (int column = 0; column < 8; column++) {
      if (bits & (0x80 >> column)) {
        display.fillRect(x + column * scale, y + row * scale, scale, scale, color);
      }
    }
  }
}

// 40 x 40 turn arrow built from filled shapes so it stays sharp at handlebar distance.
// Drawn in colour [c] so it can be knocked out of a filled tile.
void drawHeroManeuverIcon(ManeuverType maneuver, int x, int y, uint16_t c) {
  if (maneuver == MANEUVER_LEFT) {
    display.fillRect(x + 23, y + 15, 9, 25, c);
    display.fillRect(x + 11, y + 15, 21, 9, c);
    display.fillTriangle(x, y + 19, x + 14, y + 4, x + 14, y + 34, c);
  } else if (maneuver == MANEUVER_RIGHT) {
    display.fillRect(x + 8, y + 15, 9, 25, c);
    display.fillRect(x + 8, y + 15, 21, 9, c);
    display.fillTriangle(x + 39, y + 19, x + 25, y + 4, x + 25, y + 34, c);
  } else if (maneuver == MANEUVER_ARRIVE) {
    display.fillRect(x + 8, y, 4, 40, c);
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 6; col++) {
        display.drawRect(x + 12 + col * 4, y + row * 4, 5, 5, c);
        if ((row + col) % 2 == 0) display.fillRect(x + 12 + col * 4, y + row * 4, 4, 4, c);
      }
    }
  } else {
    display.fillRect(x + 15, y + 17, 10, 23, c);
    display.fillTriangle(x + 20, y, x + 3, y + 21, x + 37, y + 21, c);
  }
}

const uint8_t* iconFor(ManeuverType maneuver) {
  switch (maneuver) {
    case MANEUVER_LEFT: return ICON_LEFT;
    case MANEUVER_RIGHT: return ICON_RIGHT;
    case MANEUVER_ARRIVE: return ICON_ARRIVE;
    default: return ICON_STRAIGHT;
  }
}

void formatDistance(uint32_t metres, char* output, size_t outputSize) {
  if (metres < 1000) {
    snprintf(output, outputSize, "%lu m", (unsigned long)metres);
    return;
  }
  if (metres >= 10000) {
    snprintf(output, outputSize, "%lu km", (unsigned long)((metres + 500) / 1000));
    return;
  }
  uint32_t tenths = (metres + 50) / 100;
  snprintf(
    output, outputSize, "%lu.%lu km",
    (unsigned long)(tenths / 10), (unsigned long)(tenths % 10)
  );
}

// Wall clock synced from the phone with every frame, then kept ticking locally with millis().
bool clockKnown = false;
uint32_t clockSecondsAtSync = 0;
unsigned long clockSyncedAt = 0;
bool clock24Hour = false;
int lastDrawnClockMinute = -1;

void syncClock(const navproto::DisplayFrame& frame) {
  if (!frame.hasClock) return;
  clockSecondsAtSync = frame.clockHour * 3600UL + frame.clockMinute * 60UL + frame.clockSecond;
  clockSyncedAt = millis();
  clock24Hour = (frame.clockOptions & navproto::CLOCK_OPTION_24_HOUR) != 0;
  clockKnown = true;
}

int clockMinuteOfDay() {
  uint32_t seconds = clockSecondsAtSync + (millis() - clockSyncedAt) / 1000;
  return (int)((seconds / 60) % 1440);
}

// "16:05" (24-hour) or "4:05" (12-hour, like a phone status bar). Empty until the phone syncs.
void formatClock(char* output, size_t outputSize) {
  output[0] = '\0';
  if (!clockKnown) return;
  int minuteOfDay = clockMinuteOfDay();
  int hour = minuteOfDay / 60;
  int minute = minuteOfDay % 60;
  if (!clock24Hour) {
    hour %= 12;
    if (hour == 0) hour = 12;
    snprintf(output, outputSize, "%d:%02d", hour, minute);
  } else {
    snprintf(output, outputSize, "%02d:%02d", hour, minute);
  }
}

void formatCompactDistance(uint32_t metres, char* output, size_t outputSize) {
  if (metres < 1000) {
    snprintf(output, outputSize, "%lum", (unsigned long)metres);
  } else if (metres < 10000) {
    uint32_t tenths = (metres + 50) / 100;
    snprintf(output, outputSize, "%lu.%luk", (unsigned long)(tenths / 10), (unsigned long)(tenths % 10));
  } else if (metres < 100000) {
    snprintf(output, outputSize, "%lukm", (unsigned long)((metres + 500) / 1000));
  } else {
    snprintf(output, outputSize, "%luk", (unsigned long)((metres + 500) / 1000));
  }
}

void formatRemainingDistance(uint32_t metres, char* output, size_t outputSize) {
  if (metres < 1000) {
    snprintf(output, outputSize, "%lum", (unsigned long)metres);
  } else if (metres < 10000) {
    uint32_t tenths = (metres + 50) / 100;
    snprintf(output, outputSize, "%lu.%lukm", (unsigned long)(tenths / 10), (unsigned long)(tenths % 10));
  } else if (metres < 100000) {
    snprintf(output, outputSize, "%lukm", (unsigned long)((metres + 500) / 1000));
  } else {
    snprintf(output, outputSize, "%luk", (unsigned long)((metres + 500) / 1000));
  }
}

void formatEta(uint16_t remainingMinutes, char* output, size_t outputSize) {
  if (!clockKnown) {
    snprintf(output, outputSize, "--");
    return;
  }
  int etaMinute = (clockMinuteOfDay() + remainingMinutes) % 1440;
  int hour = etaMinute / 60;
  int minute = etaMinute % 60;
  if (clock24Hour) {
    snprintf(output, outputSize, "%02d:%02d", hour, minute);
  } else {
    hour %= 12;
    if (hour == 0) hour = 12;
    snprintf(output, outputSize, "%d:%02d", hour, minute);
  }
}

// "450" + "m", "1.2" + "km", "12" + "km".
void splitHeroDistance(uint32_t metres, char* number, size_t numberSize, const char** unit) {
  if (metres < 1000) {
    snprintf(number, numberSize, "%lu", (unsigned long)metres);
    *unit = "m";
  } else if (metres < 10000) {
    uint32_t tenths = (metres + 50) / 100;
    snprintf(number, numberSize, "%lu.%lu", (unsigned long)(tenths / 10), (unsigned long)(tenths % 10));
    *unit = "km";
  } else {
    snprintf(number, numberSize, "%lu", (unsigned long)((metres + 500) / 1000));
    *unit = "km";
  }
}

void drawFooterCell(int left, int width, const char* value, const char* label) {
  display.setTextSize(2);
  display.setCursor(left + (width - (int)strlen(value) * 12) / 2, FOOTER_TOP + 5);
  display.print(value);
  display.setTextSize(1);
  display.setCursor(left + (width - (int)strlen(label) * 6) / 2, FOOTER_TOP + 23);
  display.print(label);
}

void drawNavigationGuidance(
  ManeuverType maneuver,
  uint32_t metres,
  bool hasRemainingDistance = false,
  uint32_t remainingDistance = 0,
  bool hasRemainingDuration = false,
  uint16_t remainingDurationMinutes = 0
) {
  display.fillRect(0, HERO_TOP, TFT_WIDTH, HERO_BOTTOM - HERO_TOP + 1, TFT_BACKGROUND);
  display.fillRect(0, FOOTER_TOP, TFT_WIDTH, TFT_HEIGHT - FOOTER_TOP, TFT_BACKGROUND);
  display.setTextColor(TFT_FOREGROUND);

  // Hero: turn arrow on the left, distance to the turn on the right.
  display.fillRoundRect(HERO_TILE_X, HERO_TILE_Y, HERO_TILE_SIZE, HERO_TILE_SIZE, 9, TFT_FOREGROUND);
  drawHeroManeuverIcon(maneuver, HERO_ICON_X, HERO_ICON_Y, TFT_BACKGROUND);
  char number[8];
  const char* unit;
  splitHeroDistance(metres, number, sizeof(number), &unit);
  const int numberWidth = (int)strlen(number) * 18;
  int unitSize = 2;
  if (numberWidth + 3 + (int)strlen(unit) * 12 > HERO_TEXT_WIDTH) unitSize = 1;
  const int unitWidth = (int)strlen(unit) * 6 * unitSize;
  const int textX = HERO_TEXT_LEFT + max(0, (HERO_TEXT_WIDTH - numberWidth - 3 - unitWidth) / 2);
  const int numberY = HERO_TOP + (HERO_BOTTOM - HERO_TOP + 1 - 21) / 2;
  display.setTextSize(3);
  display.setCursor(textX, numberY);
  display.print(number);
  display.setTextSize(unitSize);
  display.setCursor(textX + numberWidth + 3, numberY + 21 - 7 * unitSize);
  display.print(unit);
  display.setTextSize(1);

  // Footer: distance left to the destination and the arrival time.
  char remaining[12] = "--";
  char eta[12] = "--";
  if (hasRemainingDistance) formatRemainingDistance(remainingDistance, remaining, sizeof(remaining));
  if (hasRemainingDuration) formatEta(remainingDurationMinutes, eta, sizeof(eta));
  drawFooterCell(0, TFT_WIDTH / 2, remaining, "TO GO");
  drawFooterCell(TFT_WIDTH / 2, TFT_WIDTH / 2, eta, "ARRIVE");
}

void drawPositionMarker(int markerY = MARKER_Y) {
  const int x[3] = {
    MARKER_X, MARKER_X - MARKER_HALF_WIDTH, MARKER_X + MARKER_HALF_WIDTH
  };
  const int y[3] = {
    markerY - MARKER_TIP_OFFSET, markerY + MARKER_BASE_OFFSET,
    markerY + MARKER_BASE_OFFSET
  };
  display.fillTriangle(
    x[0], y[0], x[1], y[1], x[2], y[2],
    TFT_BACKGROUND
  );
  display.drawTriangle(
    x[0], y[0], x[1], y[1], x[2], y[2],
    TFT_FOREGROUND
  );
}

void drawDestinationFlag(Point destination) {
  int anchorX = (int)roundf(destination.x);
  int anchorY = (int)roundf(destination.y);

  // The pole is anchored at the actual final route point. Wait until the
  // complete flag is inside the route viewport before drawing it.
  if (anchorX < 1 || anchorX >= TFT_WIDTH - 1 ||
      anchorY < ROUTE_TOP + 10 || anchorY > ROUTE_BOTTOM) {
    return;
  }

  const int flagTop = anchorY - 10;
  const bool drawLeft = anchorX > TFT_WIDTH - 9;
  const int flagX = drawLeft ? anchorX - 7 : anchorX;

  // A small black backing keeps the flag distinct from the white route.
  int backingX = max(0, flagX - 1);
  int backingWidth = min(TFT_WIDTH - backingX, 9);
  display.fillRect(backingX, flagTop - 1, backingWidth, 12, TFT_BACKGROUND);

  display.drawLine(anchorX, anchorY, anchorX, flagTop, TFT_FOREGROUND);
  display.drawRect(flagX, flagTop, 7, 6, TFT_FOREGROUND);

  // Two white squares make the 7x6 flag read as a chequered destination flag.
  display.fillRect(flagX + 1, flagTop + 1, 2, 2, TFT_FOREGROUND);
  display.fillRect(flagX + 3, flagTop + 3, 2, 2, TFT_FOREGROUND);
  display.fillCircle(anchorX, anchorY, 2, TFT_FOREGROUND);
}

void drawCentredText(const char* firstLine, const char* secondLine = nullptr);
void drawCentredLine(const char* text, int y, uint8_t size = 1);

Point normalizedToScreen(const navproto::RoutePoint& point) {
  const float routeHeight = ROUTE_BOTTOM - ROUTE_TOP;
  return {
    point.x * (TFT_WIDTH - 1) / 65535.0f,
    ROUTE_TOP + point.y * routeHeight / 65535.0f
  };
}

void drawCentredLine(const char* text, int y, uint8_t size) {
  int16_t x1;
  int16_t y1;
  uint16_t width;
  uint16_t height;
  display.setTextSize(size);
  display.getTextBounds(text, 0, 0, &x1, &y1, &width, &height);
  display.setCursor(max(0, (TFT_WIDTH - (int)width) / 2), y);
  display.print(text);
  display.setTextSize(1);
}

// Shared look for every non-route screen: white rounded pills and tiles with dark text knocked
// out of them, big type, and no divider lines.
const int PILL_X = 4;
const int PILL_WIDTH = TFT_WIDTH - 2 * PILL_X;

void drawPillText(int y, int height, const char* text, uint8_t size) {
  display.fillRoundRect(PILL_X, y, PILL_WIDTH, height, min(9, height / 2), TFT_FOREGROUND);
  display.setTextColor(TFT_BACKGROUND);
  drawCentredLine(text, y + (height - 7 * size) / 2, size);
  display.setTextColor(TFT_FOREGROUND);
}

// A 48 x 48 white tile with an 8 x 8 icon knocked out at 4x.
void drawIconTile(const uint8_t* bitmap, int y) {
  const int x = (TFT_WIDTH - 48) / 2;
  display.fillRoundRect(x, y, 48, 48, 9, TFT_FOREGROUND);
  drawScaledIcon(bitmap, x + 8, y + 8, 4, TFT_BACKGROUND);
}

// A rider-facing message: the title (9 characters at most) in a white pill, with up to two short
// lines of guidance underneath and the time above once the phone has synced it.
void drawMessageScreen(const char* title, const char* line1, const char* line2 = nullptr) {
  display.clearDisplay();
  display.setTextColor(TFT_FOREGROUND);
  char clockText[8];
  formatClock(clockText, sizeof(clockText));
  if (clockText[0]) drawCentredLine(clockText, 14, 2);
  drawPillText(46, 40, title, 2);
  if (line1) drawCentredLine(line1, 102);
  if (line2) drawCentredLine(line2, 120);
  display.display();
}

// Song line layout: a note icon, a small gap, then the title. A title wider than the screen
// scrolls right-to-left as a loop, pausing briefly at the start of each pass.
const int MUSIC_LINE_Y = 118;
const int NAV_MUSIC_LINE_Y = HEADER_CONTENT_Y;
const int MUSIC_ICON_WIDTH = 8;
const int MUSIC_TEXT_GAP = 4;
const int MARQUEE_SPACING_PX = 24;
const unsigned long MARQUEE_PAUSE_MS = 1500;
const unsigned long MARQUEE_STEP_MS = 40;  // 1 px per step, about 25 px/s
unsigned long marqueeStartedAt = 0;

bool titleNeedsScrolling(const char* title) {
  return (int)strlen(title) * 6 > TFT_WIDTH - MUSIC_ICON_WIDTH - MUSIC_TEXT_GAP;
}

bool navigationTitleNeedsScrolling(const char* title) {
  return (int)strlen(title) * 12 > TFT_WIDTH - MUSIC_ICON_WIDTH * 2 - MUSIC_TEXT_GAP;
}

void drawMusicLineAt(const char* title, int lineY, uint8_t textSize = 1) {
  const int iconWidth = MUSIC_ICON_WIDTH * textSize;
  const int iconHeight = 8 * textSize;
  const int textWidth = (int)strlen(title) * 6 * textSize;
  display.setTextSize(textSize);
  display.setTextColor(TFT_FOREGROUND);
  const bool needsScrolling = textWidth > TFT_WIDTH - iconWidth - MUSIC_TEXT_GAP;
  if (!needsScrolling) {
    const int totalWidth = iconWidth + MUSIC_TEXT_GAP + textWidth;
    const int x = (TFT_WIDTH - totalWidth) / 2;
    drawScaledIcon(ICON_MUSIC, x, lineY, textSize);
    display.setCursor(x + iconWidth + MUSIC_TEXT_GAP, lineY);
    display.print(title);
    display.setTextSize(1);
    return;
  }
  const int textLeft = iconWidth + MUSIC_TEXT_GAP;
  const unsigned long cycleWidth = textWidth + MARQUEE_SPACING_PX * textSize;
  const unsigned long elapsed = millis() - marqueeStartedAt;
  const unsigned long cycleMs = MARQUEE_PAUSE_MS + cycleWidth * MARQUEE_STEP_MS;
  const unsigned long phase = elapsed % cycleMs;
  const int offset = phase < MARQUEE_PAUSE_MS ? 0 : (int)((phase - MARQUEE_PAUSE_MS) / MARQUEE_STEP_MS);
  // Two copies make the loop seamless; the icon column is repainted on top as a clip edge.
  display.setTextWrap(false);
  display.setCursor(textLeft - offset, lineY);
  display.print(title);
  display.setCursor(textLeft - offset + (int)cycleWidth, lineY);
  display.print(title);
  display.setTextWrap(true);
  display.fillRect(0, lineY, textLeft, iconHeight, TFT_BACKGROUND);
  drawScaledIcon(ICON_MUSIC, 0, lineY, textSize);
  display.setTextSize(1);
}

void drawMusicLine(const char* title) {
  drawMusicLineAt(title, MUSIC_LINE_Y);
}

bool plainTextNeedsScrolling(const char* text, uint8_t textSize) {
  return text && (int)strlen(text) * 6 * textSize > TFT_WIDTH;
}

void drawScrollingCentredLine(const char* text, int lineY, uint8_t textSize) {
  if (!text || !text[0]) return;
  const int textWidth = (int)strlen(text) * 6 * textSize;
  display.setTextSize(textSize);
  display.setTextColor(TFT_FOREGROUND);
  if (textWidth <= TFT_WIDTH) {
    display.setCursor((TFT_WIDTH - textWidth) / 2, lineY);
    display.print(text);
    display.setTextSize(1);
    return;
  }
  const unsigned long cycleWidth = textWidth + MARQUEE_SPACING_PX * textSize;
  const unsigned long elapsed = millis() - marqueeStartedAt;
  const unsigned long cycleMs = MARQUEE_PAUSE_MS + cycleWidth * MARQUEE_STEP_MS;
  const unsigned long phase = elapsed % cycleMs;
  const int offset = phase < MARQUEE_PAUSE_MS ? 0 : (int)((phase - MARQUEE_PAUSE_MS) / MARQUEE_STEP_MS);
  display.setTextWrap(false);
  display.setCursor(-offset, lineY);
  display.print(text);
  display.setCursor(-offset + (int)cycleWidth, lineY);
  display.print(text);
  display.setTextWrap(true);
  display.setTextSize(1);
}

void drawNowPlayingScreen(const char* title, const char* artist) {
  display.clearDisplay();
  display.setTextColor(TFT_FOREGROUND);
  drawIconTile(ICON_MUSIC, 12);
  drawCentredLine("NOW PLAYING", 72);
  drawScrollingCentredLine(title, 90, 2);
  if (artist && artist[0]) drawScrollingCentredLine(artist, 118, 1);
  display.display();
}

// ---------------------------------------------------------------- Idle face
// Ported from ESP32_OLED_EYE.ino ("Fluid ROBOT Eye Motion", Sayan Electronics): two rounded eyes
// that blink every 2 s and drift between three positions with the same easing and timing.
// Changes: a shorter blink (120 ms instead of 400 ms) and the eyes sit higher when the time and
// song are shown underneath.

const int EYE_WIDTH = 25;
const int EYE_HEIGHT = 30;
const int EYE_Y_WITH_CLOCK = 32;  // eyes above the time and song
const int EYE_Y_ALONE = 64;       // vertically centred when the phone is absent
const unsigned long FACE_FRAME_MS = 50;  // the original loop's delay(50)

int leftEyeX = 40;
int rightEyeX = 80;
int targetLeftEyeX = leftEyeX;
int targetRightEyeX = rightEyeX;
const int EYE_MOVE_SPEED = 2;
int blinkState = 0;               // 0 = open, 1 = closed
const unsigned long BLINK_DELAY_MS = 2000;
// The original held the eyes shut for 400 ms, which reads as "eyes closed"; a real blink is ~120 ms.
const unsigned long BLINK_CLOSED_MS = 120;
unsigned long lastBlinkTime = 0;
unsigned long eyeMoveTime = 0;
unsigned long lastFaceFrameAt = 0;

// What the idle screen shows when no frame is driving it: before any phone, or just linked.
enum IdleMode : uint8_t { IDLE_NONE, IDLE_NO_PHONE, IDLE_LINKED };
IdleMode idleMode = IDLE_NO_PHONE;

// One step of the original loop(): blink timing, random moves and smoothing.
void updateFace(unsigned long currentTime) {
  if (currentTime - lastBlinkTime > BLINK_DELAY_MS && blinkState == 0) {
    blinkState = 1;
    lastBlinkTime = currentTime;
  } else if (currentTime - lastBlinkTime > BLINK_CLOSED_MS && blinkState == 1) {
    blinkState = 0;
    lastBlinkTime = currentTime;
  }

  if (currentTime - eyeMoveTime > (unsigned long)random(2000, 5000) && blinkState == 0) {
    int eyeMovement = random(0, 3);
    if (eyeMovement == 1) {
      targetLeftEyeX = 30;
      targetRightEyeX = 60;
    } else if (eyeMovement == 2) {
      targetLeftEyeX = 50;
      targetRightEyeX = 80;
    } else {
      targetLeftEyeX = 40;
      targetRightEyeX = 70;
    }
    eyeMoveTime = currentTime;
  }

  if (leftEyeX != targetLeftEyeX) leftEyeX += (targetLeftEyeX - leftEyeX) / EYE_MOVE_SPEED;
  if (rightEyeX != targetRightEyeX) rightEyeX += (targetRightEyeX - rightEyeX) / EYE_MOVE_SPEED;
}

// Neutral eyes: the original's expression overlays drew white on white, so they are left out.
void drawFace(int eyeY) {
  if (blinkState == 0) {
    display.fillRoundRect(leftEyeX, eyeY, EYE_WIDTH, EYE_HEIGHT, 5, TFT_FOREGROUND);
    display.fillRoundRect(rightEyeX, eyeY, EYE_WIDTH, EYE_HEIGHT, 5, TFT_FOREGROUND);
  } else {
    display.fillRect(leftEyeX, eyeY + EYE_HEIGHT / 2 - 2, EYE_WIDTH, 4, TFT_FOREGROUND);
    display.fillRect(rightEyeX, eyeY + EYE_HEIGHT / 2 - 2, EYE_WIDTH, 4, TFT_FOREGROUND);
  }
}

// Idle screen. No phone: just the eyes. Phone linked: eyes on top, the time below, then the
// playing song when there is one.
void drawIdleScreen(bool phoneLinked, const char* title, const char* artist = nullptr) {
  const unsigned long now = millis();
  if (phoneLinked && title && title[0]) {
    drawNowPlayingScreen(title, artist);
    lastFaceFrameAt = now;
    return;
  }
  updateFace(now);
  display.clearDisplay();
  display.setTextColor(TFT_FOREGROUND);
  drawFace(phoneLinked ? EYE_Y_WITH_CLOCK : EYE_Y_ALONE);
  if (phoneLinked) {
    char clockText[8];
    formatClock(clockText, sizeof(clockText));
    drawCentredLine(clockText[0] ? clockText : "MotoNav", 82, clockText[0] ? 3 : 2);
    if (title && title[0]) drawMusicLine(title);
  }
  display.display();
  lastFaceFrameAt = now;
}

void drawReadyScreen(const char* title = nullptr, const char* artist = nullptr) {
  drawIdleScreen(true, title, artist);
}

void showWaitingForPhone() {
  idleMode = IDLE_NO_PHONE;
  drawIdleScreen(false, nullptr);
}

void showWelcome() {
  idleMode = IDLE_LINKED;
  drawIdleScreen(true, nullptr);
}

// Splits a destination over two lines of at most 10 characters (size 2 fits 10 across),
// preferring to break at a space.
void splitDestination(const char* text, char* line1, char* line2) {
  const size_t maxChars = 10;
  const size_t length = strlen(text);
  line1[0] = line2[0] = '\0';
  if (length <= maxChars) {
    snprintf(line1, maxChars + 1, "%s", text);
    return;
  }
  size_t cut = maxChars;
  for (size_t i = maxChars; i > 0; i--) {
    if (text[i] == ' ') { cut = i; break; }
  }
  snprintf(line1, cut + 1, "%s", text);
  const char* rest = text + cut;
  while (*rest == ' ') rest++;
  snprintf(line2, maxChars + 1, "%s", rest);
}

// Route ready: where you start, the destination in big type, and the ride distance in a pill.
// The display has no touch input, so the ride is started from the phone.
void drawRoutePreview(const navproto::DisplayFrame& frame) {
  display.clearDisplay();
  display.setTextColor(TFT_FOREGROUND);
  char clockText[8];
  formatClock(clockText, sizeof(clockText));
  drawCentredLine(clockText[0] ? clockText : "ROUTE READY", clockText[0] ? 6 : 10, clockText[0] ? 2 : 1);

  char place[19];
  snprintf(place, sizeof(place), "%s", frame.hasText && frame.primaryText[0] ? frame.primaryText : "Start");
  display.drawCircle(9, 36, 4, TFT_FOREGROUND);
  display.setCursor(20, 32);
  display.print(place);

  drawCentredLine("GOING TO", 52);
  char line1[11];
  char line2[11];
  splitDestination(
    frame.hasText && frame.secondaryText[0] ? frame.secondaryText : "Destination", line1, line2
  );
  if (line2[0]) {
    drawCentredLine(line1, 66, 2);
    drawCentredLine(line2, 84, 2);
  } else {
    drawCentredLine(line1, 75, 2);
  }

  if (frame.hasRemainingDistance) {
    char number[8];
    const char* unit;
    splitHeroDistance(frame.remainingDistanceMetres, number, sizeof(number), &unit);
    const int numberWidth = (int)strlen(number) * 18;
    const int unitWidth = (int)strlen(unit) * 12;
    const int pillY = 112;
    const int pillHeight = 40;
    display.fillRoundRect(PILL_X, pillY, PILL_WIDTH, pillHeight, 12, TFT_FOREGROUND);
    display.setTextColor(TFT_BACKGROUND);
    const int x = (TFT_WIDTH - numberWidth - 3 - unitWidth) / 2;
    const int y = pillY + (pillHeight - 21) / 2;
    display.setTextSize(3);
    display.setCursor(x, y);
    display.print(number);
    display.setTextSize(2);
    display.setCursor(x + numberWidth + 3, y + 21 - 14);
    display.print(unit);
    display.setTextSize(1);
    display.setTextColor(TFT_FOREGROUND);
  }
  display.display();
}

void drawStateMessage(const navproto::DisplayFrame& frame) {
  const char* destination = frame.hasText && frame.primaryText[0] ? frame.primaryText : nullptr;
  switch (frame.state) {
    case navproto::READY: drawReadyScreen(
      frame.hasText ? frame.primaryText : nullptr,
      frame.hasText ? frame.secondaryText : nullptr
    ); break;
    case navproto::DISCONNECTED: drawMessageScreen("Ride over", "Pick a route in", "the app to ride"); break;
    case navproto::ACQUIRING_GPS: drawMessageScreen("Starting", "Finding GPS signal", "Ride starts soon"); break;
    case navproto::REROUTING: drawMessageScreen("Rerouting", "Finding a new way", "Keep riding safely"); break;
    case navproto::ARRIVED: drawMessageScreen("Arrived!", destination ? destination : "You made it", "Ride safe"); break;
    case navproto::GPS_DEGRADED: drawMessageScreen("GPS weak", "Signal is patchy", "Follow road signs"); break;
    case navproto::WAITING_PERMISSION: drawMessageScreen("Location?", "Allow precise", "location in the app"); break;
    case navproto::LOCATION_DISABLED: drawMessageScreen("GPS off", "Turn on phone", "location to ride"); break;
    default: return;
  }
}

void formatCallDuration(char* output, size_t outputSize) {
  unsigned long seconds = activeCallStartedAt == 0 ? 0 : (millis() - activeCallStartedAt) / 1000;
  if (seconds < 3600) {
    snprintf(output, outputSize, "%02lu:%02lu", seconds / 60, seconds % 60);
  } else {
    snprintf(output, outputSize, "%lu:%02lu:%02lu", seconds / 3600, (seconds / 60) % 60, seconds % 60);
  }
}

void drawCallScreen(const navproto::DisplayFrame& frame) {
  display.clearDisplay();
  display.setTextColor(TFT_FOREGROUND);
  const char* heading = frame.callPhase == navproto::CALL_RINGING ? "INCOMING CALL" :
      frame.callPhase == navproto::CALL_MISSED ? "MISSED CALL" : "IN CALL";
  drawCentredLine(heading, 8);
  drawIconTile(ICON_PHONE_SMALL, 24);
  // Long names scroll at size 2 instead of shrinking to an unreadable size 1.
  drawScrollingCentredLine(frame.callerText[0] ? frame.callerText : "Unknown", 86, 2);
  char status[14];
  if (frame.callPhase == navproto::CALL_ACTIVE) formatCallDuration(status, sizeof(status));
  else snprintf(status, sizeof(status), "%s", frame.callPhase == navproto::CALL_MISSED ? "MISSED" : "RINGING");
  drawPillText(TFT_HEIGHT - 42, 32, status, 2);
  display.display();
}

const size_t CALLER_HEADER_CHARS = 9;
const size_t MAX_TEXT_DIGITS = navproto::MAX_TEXT_LENGTH;

// During a ride the footer's distance / ETA cells give way to the caller. A name wider than the
// screen scrolls at the same pace as the old song marquee; a status line sits underneath.
void drawCallFooter(const navproto::DisplayFrame& frame) {
  display.fillRect(0, FOOTER_TOP, TFT_WIDTH, TFT_HEIGHT - FOOTER_TOP, TFT_BACKGROUND);
  display.fillRoundRect(CALL_PILL_X, CALL_PILL_Y, CALL_PILL_WIDTH, CALL_PILL_HEIGHT, 9, TFT_FOREGROUND);
  const char* name = frame.callerText[0] ? frame.callerText : "Unknown";
  const int textWidth = (int)strlen(name) * 12;
  const int textLeft = CALL_PILL_X + 6;
  const int textArea = CALL_PILL_WIDTH - 12;
  const int nameY = CALL_PILL_Y + 5;
  // Dark text on the white pill; anything scrolled past the pill edge lands on black and vanishes.
  display.setTextColor(TFT_BACKGROUND);
  display.setTextSize(2);
  if (textWidth <= textArea) {
    display.setCursor(textLeft + (textArea - textWidth) / 2, nameY);
    display.print(name);
  } else {
    const unsigned long cycleWidth = textWidth + MARQUEE_SPACING_PX * 2;
    const unsigned long elapsed = millis() - marqueeStartedAt;
    const unsigned long phase = elapsed % (MARQUEE_PAUSE_MS + cycleWidth * MARQUEE_STEP_MS);
    const int offset = phase < MARQUEE_PAUSE_MS ? 0 : (int)((phase - MARQUEE_PAUSE_MS) / MARQUEE_STEP_MS);
    display.setTextWrap(false);
    display.setCursor(textLeft - offset, nameY);
    display.print(name);
    display.setCursor(textLeft - offset + (int)cycleWidth, nameY);
    display.print(name);
    display.setTextWrap(true);
    // Square off the scroll window so text does not run under the pill's rounded ends.
    display.fillRect(0, CALL_PILL_Y, CALL_PILL_X + 6, CALL_PILL_HEIGHT, TFT_BACKGROUND);
    display.fillRect(CALL_PILL_X + CALL_PILL_WIDTH - 6, CALL_PILL_Y, CALL_PILL_X + 6, CALL_PILL_HEIGHT, TFT_BACKGROUND);
  }
  display.setTextSize(1);
  char status[20];
  if (frame.callPhase == navproto::CALL_ACTIVE) {
    char duration[12];
    formatCallDuration(duration, sizeof(duration));
    snprintf(status, sizeof(status), "IN CALL %s", duration);
  } else if (frame.callPhase == navproto::CALL_MISSED) {
    snprintf(status, sizeof(status), "MISSED CALL");
  } else {
    snprintf(status, sizeof(status), "INCOMING CALL");
  }
  display.setCursor((TFT_WIDTH - (int)strlen(status) * 6) / 2, CALL_PILL_Y + 21);
  display.print(status);
  display.setTextColor(TFT_FOREGROUND);
}

// First name only ("Ravi Kumar" -> "Ravi"); an unknown number shows its last five digits.
void headerCallerName(const char* caller, char* output, size_t outputSize) {
  output[0] = '\0';
  if (caller[0] == '+' || (caller[0] >= '0' && caller[0] <= '9')) {
    char digits[MAX_TEXT_DIGITS + 1];
    size_t count = 0;
    for (size_t i = 0; caller[i] && count < MAX_TEXT_DIGITS; i++) {
      if (caller[i] >= '0' && caller[i] <= '9') digits[count++] = caller[i];
    }
    digits[count] = '\0';
    snprintf(output, outputSize, "%s", count > 5 ? digits + count - 5 : digits);
    return;
  }
  size_t length = 0;
  while (caller[length] && caller[length] != ' ' && length < CALLER_HEADER_CHARS && length + 1 < outputSize) {
    output[length] = caller[length];
    length++;
  }
  output[length] = '\0';
}

// Top-right call status, drawn over whatever screen is showing. On the route screen it adds the
// name: ringing reads "Ravi... [handset]" (dots = incoming), on a call "[handset] Ravi", and a
// missed call "Missed [handset]". Screens with a big title only get the handset.
void drawCallHeader(const navproto::DisplayFrame& frame, bool withName) {
  char name[CALLER_HEADER_CHARS + 1] = "";
  if (withName && frame.callPhase == navproto::CALL_MISSED) {
    snprintf(name, sizeof(name), "Missed");
  } else if (withName && frame.callerText[0]) {
    headerCallerName(frame.callerText, name, sizeof(name));
  }
  const int iconWidth = 16;
  const int gap = 4;
  const int textCellWidth = 12;
  display.setTextSize(2);
  display.setTextColor(TFT_FOREGROUND);

  if ((frame.callPhase == navproto::CALL_RINGING || frame.callPhase == navproto::CALL_MISSED) && name[0]) {
    char label[CALLER_HEADER_CHARS + 1];
    if (frame.callPhase == navproto::CALL_RINGING) {
      // Keep room for the dots at the larger status-bar text size.
      name[min((size_t)4, strlen(name))] = '\0';
      snprintf(label, sizeof(label), "%s...", name);
    } else {
      snprintf(label, sizeof(label), "%s", name);
    }
    const int iconX = TFT_WIDTH - iconWidth - 1;
    const int textX = iconX - gap - (int)strlen(label) * textCellWidth;
    display.fillRect(textX - 1, 0, TFT_WIDTH - textX + 1, HEADER_BOTTOM, TFT_BACKGROUND);
    display.setCursor(textX, HEADER_CONTENT_Y);
    display.print(label);
    drawScaledIcon(ICON_PHONE_SMALL, iconX, HEADER_CONTENT_Y, 2);
  } else if (frame.callPhase == navproto::CALL_ACTIVE) {
    char duration[12];
    formatCallDuration(duration, sizeof(duration));
    const int durationWidth = (int)strlen(duration) * textCellWidth;
    const int totalWidth = iconWidth + gap + durationWidth;
    const int iconX = (TFT_WIDTH - totalWidth) / 2;
    const int textX = iconX + iconWidth + gap;
    display.fillRect(0, 0, TFT_WIDTH, HEADER_BOTTOM, TFT_BACKGROUND);
    drawScaledIcon(ICON_PHONE_SMALL, iconX, HEADER_CONTENT_Y, 2);
    display.setCursor(textX, HEADER_CONTENT_Y);
    display.print(duration);
  } else {
    display.fillRect(TFT_WIDTH - 18, 0, 18, HEADER_BOTTOM, TFT_BACKGROUND);
    drawScaledIcon(ICON_PHONE_SMALL, TFT_WIDTH - 17, HEADER_CONTENT_Y, 2);
  }
  display.setTextSize(1);
  display.display();
}

void drawBleFrame(const navproto::DisplayFrame& frame);

// During a ride calls stay in the status bar so the road is never covered. Outside a ride, the
// caller gets a dedicated screen because no navigation information is being displaced.
bool rideActive(const navproto::DisplayFrame& frame) {
  return frame.state != navproto::READY && frame.state != navproto::ROUTE_PREVIEW &&
      frame.state != navproto::DISCONNECTED;
}

bool fullCallScreenShown(const navproto::DisplayFrame& frame) {
  return !rideActive(frame) && frame.callPhase != navproto::CALL_NONE;
}

void renderLastBleFrame() {
  if (!hasLastBleFrame) return;
  const navproto::DisplayFrame& frame = lastBleFrame;
  const bool onRoute = frame.state == navproto::NAVIGATING;
  if (fullCallScreenShown(frame)) {
    drawCallScreen(frame);
    return;
  }
  drawBleFrame(frame);
  // On the route screen the call lives in the footer; other screens keep the header icon.
  if (frame.callPhase != navproto::CALL_NONE && !onRoute) drawCallHeader(frame, false);
}

bool scrollingTitleShown(const navproto::DisplayFrame& frame) {
  if (fullCallScreenShown(frame)) return plainTextNeedsScrolling(frame.callerText, 2);
  if (frame.state == navproto::NAVIGATING) {
    return frame.callPhase != navproto::CALL_NONE && (int)strlen(frame.callerText) * 12 > CALL_PILL_WIDTH - 12;
  }
  if (!frame.hasText || !frame.primaryText[0]) return false;
  if (frame.state == navproto::READY) {
    return plainTextNeedsScrolling(frame.primaryText, 2) ||
        (frame.secondaryText[0] && plainTextNeedsScrolling(frame.secondaryText, 1));
  }
  return false;
}

void acceptBleFrame(const navproto::DisplayFrame& frame) {
  idleMode = IDLE_NONE;  // frames from the phone drive the screen from here on
  Serial.print("Rendering BLE frame, sequence=");
  Serial.print(frame.sequence);
  Serial.print(" state=");
  Serial.println(frame.state);
  syncClock(frame);
  const bool sameTitle = hasLastBleFrame && lastBleFrame.state == frame.state &&
      strcmp(lastBleFrame.primaryText, frame.primaryText) == 0 &&
      strcmp(lastBleFrame.callerText, frame.callerText) == 0;
  if (!sameTitle) marqueeStartedAt = millis();  // each new song starts from its beginning
  lastDrawnClockMinute = clockKnown ? clockMinuteOfDay() : -1;
  const navproto::CallPhase previousPhase = hasLastBleFrame ? lastBleFrame.callPhase : navproto::CALL_NONE;
  if (frame.callPhase == navproto::CALL_ACTIVE && previousPhase != navproto::CALL_ACTIVE) {
    activeCallStartedAt = millis();
    if (activeCallStartedAt == 0) activeCallStartedAt = 1;
    lastDrawnCallSecond = 0xffffffffUL;
  } else if (frame.callPhase != navproto::CALL_ACTIVE) {
    activeCallStartedAt = 0;
    lastDrawnCallSecond = 0xffffffffUL;
  }
  lastBleFrame = frame;
  hasLastBleFrame = true;
  renderLastBleFrame();
  Serial.println("TFT frame rendered");
}

// Keeps the eyes (and a scrolling title) moving on both idle screens.
void updateIdleAnimation() {
  const unsigned long now = millis();
  if (now - lastFaceFrameAt < FACE_FRAME_MS) return;
  if (idleMode == IDLE_NO_PHONE) {
    drawIdleScreen(false, nullptr);
  } else if (idleMode == IDLE_LINKED) {
    drawIdleScreen(true, nullptr);
  } else if (hasLastBleFrame && !staleFrameShown) {
    if (lastBleFrame.state == navproto::READY) {
      if (fullCallScreenShown(lastBleFrame)) {
        if (scrollingTitleShown(lastBleFrame)) {
          renderLastBleFrame();
          lastFaceFrameAt = now;
        }
        return;
      }
      renderLastBleFrame();
    } else if (scrollingTitleShown(lastBleFrame)) {
      renderLastBleFrame();
      lastFaceFrameAt = now;
    }
  }
}

// Keeps the time current on screen even if the phone briefly stops sending frames.
void updateClock() {
  if (!clockKnown || !hasLastBleFrame) return;
  int minute = clockMinuteOfDay();
  if (minute == lastDrawnClockMinute) return;
  lastDrawnClockMinute = minute;
  renderLastBleFrame();
}

// The phone reports the call phase; the display owns the lightweight duration timer locally.
void updateActiveCallDuration() {
  if (!hasLastBleFrame || lastBleFrame.callPhase != navproto::CALL_ACTIVE || activeCallStartedAt == 0) return;
  unsigned long second = (millis() - activeCallStartedAt) / 1000;
  if (second == lastDrawnCallSecond) return;
  lastDrawnCallSecond = second;
  renderLastBleFrame();
}

void drawBleFrame(const navproto::DisplayFrame& frame) {
  if (frame.state == navproto::ROUTE_PREVIEW) {
    drawRoutePreview(frame);
    return;
  }
  if (frame.state != navproto::NAVIGATING) {
    drawStateMessage(frame);
    return;
  }

  display.clearDisplay();
  bool hasPrevious = false;
  Point previous = {0, 0};
  bool hasDestination = false;
  Point destination = {0, 0};
  for (size_t i = 0; i < frame.pointCount; i++) {
    Point current = normalizedToScreen(frame.points[i]);
    if ((frame.points[i].flags & navproto::POINT_FLAG_START) != 0) {
      hasPrevious = true;
      previous = current;
    } else if (hasPrevious) {
      drawClippedRouteLine(previous, current);
      previous = current;
    }

    if ((frame.points[i].flags & navproto::POINT_FLAG_DESTINATION) != 0) {
      hasDestination = true;
      destination = current;
    }
  }

  if (hasDestination) drawDestinationFlag(destination);
  drawPositionMarker(BLE_MARKER_Y);
  drawNavigationGuidance(
    (ManeuverType)frame.maneuver,
    frame.nextTurnMetres,
    frame.hasRemainingDistance,
    frame.remainingDistanceMetres,
    frame.hasRemainingDuration,
    frame.remainingDurationMinutes
  );
  if (frame.callPhase != navproto::CALL_NONE) drawCallFooter(frame);
  display.display();
}

void drawNavigationFrame() {
  const RouteScenario& scenario = SCENARIOS[currentScenarioIndex];
  Point current = scenario.positions[currentFrame];
  RouteProgress progress = findRouteProgress(scenario, current);
  Point routeStart = scenario.route[progress.segment];
  Point routeEnd = scenario.route[progress.segment + 1];
  float headingX = routeEnd.x - routeStart.x;
  float headingY = routeEnd.y - routeStart.y;
  float headingLength = hypotf(headingX, headingY);
  if (headingLength < 0.01f) return;
  headingX /= headingLength;
  headingY /= headingLength;

  const ManeuverEvent& maneuver = nextManeuver(scenario, progress);
  uint32_t maneuverDistance = (uint32_t)roundf(
    distanceToWaypoint(scenario, progress, maneuver.waypointIndex)
  );

  // Draw in the memory buffer and transfer one finished frame.
  display.clearDisplay();
  Point previous = toScreen(current, current, headingX, headingY);
  for (size_t i = progress.segment + 1; i < scenario.routeCount; i++) {
    Point next = toScreen(scenario.route[i], current, headingX, headingY);
    drawClippedRouteLine(previous, next);
    previous = next;
  }
  Point destination = toScreen(
    scenario.route[scenario.routeCount - 1], current, headingX, headingY
  );
  drawDestinationFlag(destination);
  drawPositionMarker();
  drawNavigationGuidance(maneuver.type, maneuverDistance);
  display.display();

  Serial.print("Scenario ");
  Serial.print(currentScenarioIndex + 1);
  Serial.print(" (");
  Serial.print(scenario.name);
  Serial.print(") frame ");
  Serial.print(currentFrame + 1);
  Serial.print("/");
  Serial.print(scenario.positionCount);
  Serial.print(" next=");
  Serial.print(maneuverDistance);
  Serial.println("m");
}

void drawCentredText(const char* firstLine, const char* secondLine) {
  display.clearDisplay();
  display.setTextColor(TFT_FOREGROUND);
  display.setTextSize(1);
  int16_t x1;
  int16_t y1;
  uint16_t width;
  uint16_t height;
  display.getTextBounds(firstLine, 0, 0, &x1, &y1, &width, &height);
  display.setCursor((TFT_WIDTH - width) / 2, secondLine ? 66 : 76);
  display.print(firstLine);
  if (secondLine) {
    display.getTextBounds(secondLine, 0, 0, &x1, &y1, &width, &height);
    display.setCursor((TFT_WIDTH - width) / 2, 84);
    display.print(secondLine);
  }
  display.display();
}

class DisplayServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& connectionInfo) override {
    portENTER_CRITICAL(&assemblerMux);
    fragmentAssembler.resetSession();
    portEXIT_CRITICAL(&assemblerMux);
    bleConnected = true;
    bleConnectionEvent = 1;
    Serial.print("BLE client connected: ");
    Serial.println(connectionInfo.getAddress().toString().c_str());
  }

  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    portENTER_CRITICAL(&assemblerMux);
    fragmentAssembler.resetSession();
    portEXIT_CRITICAL(&assemblerMux);
    portENTER_CRITICAL(&pendingFrameMux);
    pendingBleFrameReady = false;
    pendingBleFeedback = 0;
    portEXIT_CRITICAL(&pendingFrameMux);
    bleConnected = false;
    bleConnectionEvent = 2;
    Serial.print("BLE client disconnected, reason=");
    Serial.println(reason);
    NimBLEDevice::startAdvertising();
  }
};

class FrameCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(
    NimBLECharacteristic* characteristic,
    NimBLEConnInfo&
  ) override {
    const NimBLEAttValue& value = characteristic->getValue();
    navproto::DisplayFrame decoded = {};
    navproto::FragmentResult result;
    navproto::DecodeStatus decodeStatus;
    size_t receivedCount;
    size_t expectedLength;

    Serial.print("BLE write received, bytes=");
    Serial.print(value.size());
    Serial.print(" data=");
    for (size_t i = 0; i < value.size(); i++) {
      if (value[i] < 0x10) Serial.print('0');
      Serial.print(value[i], HEX);
      if (i + 1 < value.size()) Serial.print(' ');
    }
    Serial.println();

    portENTER_CRITICAL(&assemblerMux);
    result = fragmentAssembler.acceptFragment(
      value.data(), value.size(), millis(), decoded
    );
    decodeStatus = fragmentAssembler.lastDecodeStatus();
    receivedCount = fragmentAssembler.receivedCount();
    expectedLength = fragmentAssembler.expectedLength();
    portEXIT_CRITICAL(&assemblerMux);

    if (result == navproto::FRAGMENT_COMPLETE) {
      portENTER_CRITICAL(&pendingFrameMux);
      pendingBleFrame = decoded;
      pendingBleFrameReady = true;
      pendingBleFeedback = 0;
      portEXIT_CRITICAL(&pendingFrameMux);
      Serial.print("BLE frame accepted, sequence=");
      Serial.print(decoded.sequence);
      Serial.print(" points=");
      Serial.println(decoded.pointCount);
    } else if (result == navproto::FRAGMENT_IN_PROGRESS) {
      portENTER_CRITICAL(&pendingFrameMux);
      pendingBleReceivedBytes = receivedCount;
      pendingBleExpectedBytes = expectedLength;
      pendingBleFeedback = 1;
      portEXIT_CRITICAL(&pendingFrameMux);
      Serial.print("BLE fragment accepted: ");
      Serial.print(receivedCount);
      Serial.print('/');
      Serial.print(expectedLength);
      Serial.println(" frame bytes");
    } else if (result == navproto::FRAGMENT_FRAME_INVALID) {
      portENTER_CRITICAL(&pendingFrameMux);
      pendingBleFeedback = 2;
      portEXIT_CRITICAL(&pendingFrameMux);
      Serial.print("BLE frame rejected: ");
      Serial.println(navproto::decodeStatusName(decodeStatus));
    } else if (result == navproto::FRAGMENT_INVALID) {
      portENTER_CRITICAL(&pendingFrameMux);
      pendingBleFeedback = 2;
      portEXIT_CRITICAL(&pendingFrameMux);
      Serial.println("BLE fragment rejected: invalid header or bounds");
    } else if (result == navproto::FRAGMENT_STALE) {
      portENTER_CRITICAL(&pendingFrameMux);
      pendingBleFeedback = 2;
      portEXIT_CRITICAL(&pendingFrameMux);
      Serial.println("BLE fragment ignored: stale sequence");
    }
  }
};

DisplayServerCallbacks displayServerCallbacks;
FrameCharacteristicCallbacks frameCharacteristicCallbacks;

void initializeBle() {
  if (bleInitialized) return;

  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(&displayServerCallbacks);
  NimBLEService* service = server->createService(BLE_SERVICE_UUID);

  NimBLECharacteristic* frameCharacteristic = service->createCharacteristic(
    BLE_FRAME_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR,
    navproto::MAX_FRAME_SIZE
  );
  frameCharacteristic->setCallbacks(&frameCharacteristicCallbacks);

  NimBLECharacteristic* capabilityCharacteristic = service->createCharacteristic(
    BLE_CAPABILITY_UUID,
    NIMBLE_PROPERTY::READ,
    10
  );
  // Byte 9 extra flags: bit 0 = wall clock, bit 1 = remaining duration / ETA.
  uint8_t capabilities[10] = {
    navproto::PROTOCOL_VERSION,
    (uint8_t)(PROTOCOL_DISPLAY_WIDTH & 0xFF), (uint8_t)(PROTOCOL_DISPLAY_WIDTH >> 8),
    (uint8_t)(PROTOCOL_DISPLAY_HEIGHT & 0xFF), (uint8_t)(PROTOCOL_DISPLAY_HEIGHT >> 8),
    (uint8_t)navproto::MAX_POINTS,
    (uint8_t)(navproto::MAX_FRAME_SIZE & 0xFF),
    (uint8_t)(navproto::MAX_FRAME_SIZE >> 8),
    0xF7,
    0x03
  };
  capabilityCharacteristic->setValue(capabilities, sizeof(capabilities));

  service->start();
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->setName(BLE_DEVICE_NAME);
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->enableScanResponse(true);
  advertising->start();
  bleInitialized = true;
  Serial.println("BLE receiver advertising as MotoNav Display");
}

void enterBleReceiverMode() {
  initializeBle();
  applicationMode = MODE_BLE_RECEIVER;
  staleFrameShown = false;
  lastValidBleFrameAt = 0;
  if (bleConnected) showWelcome();
  else showWaitingForPhone();
  Serial.println("BLE receiver mode active");
}

void processBleReceiver() {
  uint8_t connectionEvent = bleConnectionEvent;
  if (connectionEvent != 0) {
    bleConnectionEvent = 0;
    staleFrameShown = false;
    lastValidBleFrameAt = 0;
    hasLastBleFrame = false;
    activeCallStartedAt = 0;
    lastDrawnCallSecond = 0xffffffffUL;
    if (connectionEvent == 1) showWelcome();
    else showWaitingForPhone();
  }

  navproto::DisplayFrame frame = {};
  bool hasFrame = false;
  uint8_t feedback = 0;
  uint16_t receivedBytes = 0;
  uint16_t expectedBytes = 0;
  portENTER_CRITICAL(&pendingFrameMux);
  if (pendingBleFrameReady) {
    frame = pendingBleFrame;
    pendingBleFrameReady = false;
    hasFrame = true;
  }
  feedback = pendingBleFeedback;
  receivedBytes = pendingBleReceivedBytes;
  expectedBytes = pendingBleExpectedBytes;
  pendingBleFeedback = 0;
  portEXIT_CRITICAL(&pendingFrameMux);

  if (hasFrame) {
    acceptBleFrame(frame);
    lastValidBleFrameAt = millis();
    staleFrameShown = false;
  }
  // Partial or rejected frames are logged over serial; the rider keeps the last good screen.
  (void)feedback;
  (void)receivedBytes;
  (void)expectedBytes;

  if (!staleFrameShown) {
    updateActiveCallDuration();
    updateClock();
  }
  updateIdleAnimation();

  bool assemblyExpired;
  portENTER_CRITICAL(&assemblerMux);
  assemblyExpired = fragmentAssembler.expire(millis());
  portEXIT_CRITICAL(&assemblerMux);
  if (assemblyExpired) Serial.println("BLE incomplete frame discarded: timeout");

  if (bleConnected && lastValidBleFrameAt != 0 && !staleFrameShown &&
      millis() - lastValidBleFrameAt > STALE_FRAME_MS) {
    drawMessageScreen("Hold on", "Waiting for phone", "to send an update");
    staleFrameShown = true;
    idleMode = IDLE_NONE;
    hasLastBleFrame = false;
  }
}

void drawStateScreen() {
  switch (navigationState) {
    case STATE_DISCONNECTED: drawCentredText("DISCONNECTED"); break;
    case STATE_ACQUIRING_GPS: drawCentredText("ACQUIRING", "GPS..."); break;
    case STATE_REROUTING: drawCentredText("REROUTING..."); break;
    case STATE_ARRIVED: drawCentredText("ARRIVED"); break;
    case STATE_DIAGNOSTIC: break;
    case STATE_NAVIGATING: drawNavigationFrame(); break;
  }
}

void drawDiagnostic() {
  navigationState = STATE_DIAGNOSTIC;
  display.clearDisplay();
  display.drawRect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_FOREGROUND);
  display.drawLine(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1, TFT_FOREGROUND);
  display.drawLine(TFT_WIDTH - 1, 0, 0, TFT_HEIGHT - 1, TFT_FOREGROUND);
  display.fillCircle(TFT_WIDTH / 2, TFT_HEIGHT / 2, 3, TFT_BACKGROUND);
  display.drawCircle(TFT_WIDTH / 2, TFT_HEIGHT / 2, 3, TFT_FOREGROUND);
  display.setTextColor(TFT_FOREGROUND);
  display.setTextSize(1);
  display.setCursor(4, 4);
  display.print(TFT_WIDTH);
  display.print('x');
  display.print(TFT_HEIGHT);
  display.setCursor(4, TFT_HEIGHT - 20);
  display.print("ST7735 portrait");
  display.setCursor(4, TFT_HEIGHT - 10);
  display.print("SPI SCK/MOSI ");
  display.print(TFT_SCLK_PIN);
  display.print('/');
  display.print(TFT_MOSI_PIN);
  display.display();
}

void selectScenario(size_t index) {
  if (index >= SCENARIO_COUNT) return;
  applicationMode = MODE_DEMO;
  currentScenarioIndex = index;
  currentFrame = 0;
  navigationState = STATE_NAVIGATING;
  simulationPaused = false;
  drawNavigationFrame();
  lastFrameAt = millis();
}

void printControls() {
  Serial.println();
  Serial.println("Motorcycle display demo controls:");
  Serial.println("  1 right turn / km-to-m transition");
  Serial.println("  2 left turn");
  Serial.println("  3 consecutive junctions");
  Serial.println("  p pause/resume, f advance one frame");
  Serial.println("  d disconnected, g acquiring GPS");
  Serial.println("  n navigating, r rerouting, a arrived");
  Serial.println("  i display/SPI diagnostic, h help");
  Serial.println("  b BLE receiver mode");
}

void handleCommand(char command) {
  switch (command) {
    case '1': selectScenario(0); break;
    case '2': selectScenario(1); break;
    case '3': selectScenario(2); break;
    case 'p':
    case 'P':
      simulationPaused = !simulationPaused;
      Serial.println(simulationPaused ? "Simulation paused" : "Simulation resumed");
      break;
    case 'f':
    case 'F': {
      const RouteScenario& scenario = SCENARIOS[currentScenarioIndex];
      navigationState = STATE_NAVIGATING;
      simulationPaused = true;
      currentFrame = (currentFrame + 1) % scenario.positionCount;
      drawNavigationFrame();
      break;
    }
    case 'd':
    case 'D': navigationState = STATE_DISCONNECTED; drawStateScreen(); break;
    case 'g':
    case 'G': navigationState = STATE_ACQUIRING_GPS; drawStateScreen(); break;
    case 'n':
    case 'N': navigationState = STATE_NAVIGATING; drawStateScreen(); break;
    case 'r':
    case 'R': navigationState = STATE_REROUTING; drawStateScreen(); break;
    case 'a':
    case 'A': navigationState = STATE_ARRIVED; drawStateScreen(); break;
    case 'i':
    case 'I': drawDiagnostic(); break;
    case 'b':
    case 'B': enterBleReceiverMode(); break;
    case 'h':
    case 'H': printControls(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  SPI.begin(TFT_SCLK_PIN, -1, TFT_MOSI_PIN, TFT_CS_PIN);
  tft.initR(TFT_INIT_OPTION);
  tft.setRotation(TFT_ROTATION);

  if (!display.getBuffer() || tft.width() != TFT_WIDTH || tft.height() != TFT_HEIGHT) {
    Serial.println("TFT initialization failed: portrait framebuffer unavailable or dimensions mismatch");
    while (true) delay(1000);
  }

  display.clearDisplay();
  display.display();
  Serial.print("ST7735 TFT configured: ");
  Serial.print(TFT_WIDTH);
  Serial.print('x');
  Serial.print(TFT_HEIGHT);
  Serial.print(" portrait rotation=");
  Serial.print(TFT_ROTATION);
  Serial.print(" SCK=");
  Serial.print(TFT_SCLK_PIN);
  Serial.print(" MOSI=");
  Serial.print(TFT_MOSI_PIN);
  Serial.print(" CS=");
  Serial.print(TFT_CS_PIN);
  Serial.print(" DC=");
  Serial.print(TFT_DC_PIN);
  Serial.print(" RST=");
  Serial.println(TFT_RST_PIN);
  printControls();
  initializeBle();
  if (START_IN_BLE_MODE) enterBleReceiverMode();
  else selectScenario(0);
}

void loop() {
  while (Serial.available() > 0) handleCommand((char)Serial.read());
  if (applicationMode == MODE_BLE_RECEIVER) {
    processBleReceiver();
    return;
  }
  if (navigationState != STATE_NAVIGATING || simulationPaused) return;

  unsigned long now = millis();
  if (now - lastFrameAt >= FRAME_INTERVAL_MS) {
    lastFrameAt = now;
    const RouteScenario& scenario = SCENARIOS[currentScenarioIndex];
    currentFrame = (currentFrame + 1) % scenario.positionCount;
    drawNavigationFrame();
  }
}
