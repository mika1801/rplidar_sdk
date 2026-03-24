/**
 * BasicObstacleAvoidance.ino
 *
 * Minimal example: read RPLidar A1M8 data and print obstacle alerts to the
 * USB serial monitor.
 *
 * Target board: Arduino Giga R1 WiFi
 *
 * Wiring
 * ──────
 *   RPLidar pin   →  Arduino Giga R1 WiFi pin
 *   ──────────────────────────────────────────
 *   TX            →  Serial1 RX  (D0)
 *   RX            →  Serial1 TX  (D1)
 *   VCC (5 V)     →  5 V
 *   GND           →  GND
 *   MOTO_CTRL     →  D2  (motor enable, HIGH = spinning)
 *
 * In the Arduino IDE:
 *   Board   : Arduino Giga R1 WiFi
 *   Port    : the correct COM / /dev/ttyACM* port
 */

#include <RPLidarObstacleAvoidance.h>

// ---- Pins & parameters -----------------------------------------------------

static const int  MOTOR_PIN       = 2;     // MOTO_CTRL pin  (-1 = not used)
static const float DANGER_DIST_MM = 500.0f; // 50 cm danger zone

// ---- Library instance -------------------------------------------------------
//   Serial1 is the hardware UART on pins D0/D1 of the Giga R1 WiFi.
RPLidarObstacleAvoidance lidar(Serial1, MOTOR_PIN);

// ---- Setup ------------------------------------------------------------------

void setup()
{
    // USB debug output
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait up to 3 s for USB */ }

    Serial.println("RPLidar A1M8 – Basic Obstacle Avoidance");
    Serial.println("Initialising… (takes ~3 seconds)");

    // Sector defaults (can be changed with setSector*() before begin()):
    //   Front  0° ±30°    Left  90° ±30°
    //   Back 180° ±30°    Right 270° ±30°
    lidar.begin();

    Serial.println("Scanning started.");
    Serial.println("─────────────────────────────────────────");
}

// ---- Loop -------------------------------------------------------------------

void loop()
{
    // Pump the serial RX buffer – call as often as possible
    lidar.update();

    // Process once a complete 360° sweep is available
    if (!lidar.isScanComplete()) return;

    // ---- Obstacle checks ----
    bool front = lidar.isObstacleFront(DANGER_DIST_MM);
    bool left  = lidar.isObstacleLeft (DANGER_DIST_MM);
    bool right = lidar.isObstacleRight(DANGER_DIST_MM);
    bool back  = lidar.isObstacleBack (DANGER_DIST_MM);

    // ---- High-level recommendation ----
    ObstacleDirection dir = lidar.getRecommendedDirection(DANGER_DIST_MM);

    // ---- Print distances in each sector ----
    float dFront = lidar.getMinDistance(330.0f, 30.0f);   // 330°-360° + 0°-30°
    float dLeft  = lidar.getMinDistance( 60.0f, 120.0f);
    float dRight = lidar.getMinDistance(240.0f, 300.0f);
    float dBack  = lidar.getMinDistance(150.0f, 210.0f);

    Serial.print("Scan #");
    Serial.print(lidar.getScanCount());
    Serial.print("  F:");
    printDist(dFront, front);
    Serial.print("  L:");
    printDist(dLeft,  left);
    Serial.print("  R:");
    printDist(dRight, right);
    Serial.print("  B:");
    printDist(dBack,  back);
    Serial.print("  → ");
    Serial.println(directionName(dir));
}

// ---- Helpers ----------------------------------------------------------------

static void printDist(float mm, bool blocked)
{
    if (mm < 0) {
        Serial.print("---");
    } else {
        Serial.print(static_cast<int>(mm));
        Serial.print("mm");
    }
    Serial.print(blocked ? "!" : " ");
}

static const char* directionName(ObstacleDirection d)
{
    switch (d) {
        case DIRECTION_CLEAR:    return "CLEAR";
        case DIRECTION_FORWARD:  return "FORWARD";
        case DIRECTION_LEFT:     return "TURN LEFT";
        case DIRECTION_RIGHT:    return "TURN RIGHT";
        case DIRECTION_BACKWARD: return "BACK UP";
        case DIRECTION_BLOCKED:  return "BLOCKED";
        default:                 return "UNKNOWN";
    }
}
