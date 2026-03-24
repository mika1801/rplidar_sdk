/**
 * NanoRobotNavigation.ino
 *
 * RPLidar A1M8 obstacle avoidance on Arduino Nano 33 IoT
 * with Arduino Nano Motor Carrier and Pololu 4751 gearmotors.
 *
 * Key advantage over the Giga R1 / Motor Shield Rev3 version:
 *   The 64 CPR encoders on the Pololu 4751 motors give 4 864 ticks/wheel-rev
 *   after the 19:1 gearbox.  This allows:
 *     • Precise 90° turns regardless of battery voltage or floor friction
 *     • Exact backup distances (e.g. always retreat exactly 20 cm)
 *     • Differential speed correction to keep straight lines
 *     • Stall detection (wheel blocked → instant stop)
 *
 * Hardware setup
 * ──────────────
 *   Nano 33 IoT stacked on Arduino Nano Motor Carrier.
 *
 *   RPLidar A1M8
 *     TX        →  Nano pin 0  (Serial1 RX)
 *     RX        →  Nano pin 1  (Serial1 TX)
 *     5 V       →  5 V
 *     GND       →  GND
 *     MOTO_CTRL →  D7          (HIGH = lidar motor ON)
 *
 *   Pololu 4751 left  motor  →  Carrier M1  (encoder → ENC_A1 / ENC_B1)
 *   Pololu 4751 right motor  →  Carrier M2  (encoder → ENC_A2 / ENC_B2)
 *   12 V supply              →  Carrier VIN / GND
 *
 * !! IMPORTANT: measure your robot !!
 *   Set WHEEL_DIAM_MM and WHEEL_BASE_MM below to match your actual build.
 *   Wrong values will cause inaccurate turns and distances.
 *
 * Angle convention:
 *   0° = front   90° = left   180° = back   270° = right
 */

#include <RPLidarObstacleAvoidanceNano.h>

// ─── Robot geometry – adjust to your build ───────────────────────────────────
static const float WHEEL_DIAM_MM = 65.0f;   // outer diameter of your drive wheels
static const float WHEEL_BASE_MM = 150.0f;  // centre-to-centre distance between wheels

// ─── Safety distances ─────────────────────────────────────────────────────────
static const float STOP_DIST_MM   = 300.0f;  // hard-stop threshold  (30 cm)
static const float SLOW_DIST_MM   = 600.0f;  // slow-down threshold  (60 cm)
static const float BACKUP_DIST_MM = 180.0f;  // how far to back up   (18 cm)
static const float TURN_ANGLE_DEG =  90.0f;  // how much to turn     (90°)

// ─── Lidar motor pin ──────────────────────────────────────────────────────────
static const int LIDAR_MOTOR_PIN = 7;

// ─── Library instance ─────────────────────────────────────────────────────────
RPLidarObstacleAvoidanceNano::Config robotCfg;

RPLidarObstacleAvoidanceNano lidar(Serial1, LIDAR_MOTOR_PIN, robotCfg);

// ─── Robot state ──────────────────────────────────────────────────────────────
enum RobotState {
    STATE_FORWARD,          // driving ahead, checking for obstacles
    STATE_AVOIDING,         // encoder manoeuvre in progress (turn / backup)
    STATE_STOPPED,          // completely blocked or stalled
};

static RobotState robotState = STATE_STOPPED;

// ─── Forward declarations ─────────────────────────────────────────────────────
static void     startAvoidance(ObstacleDirection dir);
static void     printStatus(ObstacleDirection dir, float frontDist);
static const char* dirName(ObstacleDirection d);

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("RPLidar A1M8 – Nano Motor Carrier – Encoder Obstacle Avoidance");
    Serial.println("Configuring robot geometry…");

    // ── Geometry ──────────────────────────────────────────────────────────────
    robotCfg.wheelDiameterMm = WHEEL_DIAM_MM;
    robotCfg.wheelBaseMm     = WHEEL_BASE_MM;

    // ── Pololu 4751 encoder: 64 CPR × 4× quadrature × 19:1 = 4 864 ticks/rev ──
    robotCfg.ticksPerRev = 4864;

    // ── Navigation duty cycles (0-100 %) ─────────────────────────────────────
    robotCfg.dutyForward  = 60;
    robotCfg.dutyTurn     = 50;
    robotCfg.dutyBackward = 45;

    // ── Stall detection ───────────────────────────────────────────────────────
    robotCfg.stallWindowMs = 400;   // declare stall if no movement for 400 ms
    robotCfg.stallMinTicks = 8;     // minimum ticks to count as "moving"

    // ── Lidar danger sectors ──────────────────────────────────────────────────
    lidar.setFrontSector(  0.0f, 60.0f);   // 330°–30°
    lidar.setLeftSector  ( 90.0f, 80.0f);  // 50°–130°
    lidar.setRightSector (270.0f, 80.0f);  // 230°–310°
    lidar.setBackSector  (180.0f, 60.0f);  // 150°–210°
    lidar.setMinQuality(8);

    Serial.println("Initialising lidar (~3 s)…");

    if (!lidar.begin()) {
        Serial.println("ERROR: Motor Carrier not found on I2C! Check connections.");
        while (true) {}
    }

    Serial.println("Ready – starting forward navigation.");
    Serial.println("──────────────────────────────────────────────────────────");

    robotState = STATE_FORWARD;
    lidar.driveForward(robotCfg.dutyForward);
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop()
{
    // Always pump the lidar serial buffer first
    lidar.update();

    // ── Lidar health check ────────────────────────────────────────────────────
    if (!lidar.isConnected()) {
        Serial.println("LIDAR TIMEOUT – emergency stop!");
        lidar.abortManeuver();
        lidar.motorStop();
        robotState = STATE_STOPPED;
        delay(2000);
        return;
    }

    // ── Stall detection ───────────────────────────────────────────────────────
    if (robotState == STATE_FORWARD &&
        (lidar.isLeftMotorStalled() || lidar.isRightMotorStalled())) {
        Serial.println("STALL DETECTED – initiating emergency backup!");
        startAvoidance(DIRECTION_BACKWARD);
        return;
    }

    // ── Wait for active manoeuvre to finish ───────────────────────────────────
    if (robotState == STATE_AVOIDING) {
        if (lidar.isManeuverComplete()) {
            Serial.println("Manoeuvre complete – resuming forward.");
            robotState = STATE_FORWARD;
            lidar.driveForward(robotCfg.dutyForward);
        }
        return;
    }

    // ── Process completed lidar scan ──────────────────────────────────────────
    if (!lidar.isScanComplete()) return;

    ObstacleDirection dir      = lidar.getRecommendedDirection(STOP_DIST_MM);
    float             frontDist = lidar.getMinDistance(330.0f, 30.0f);

    printStatus(dir, frontDist);

    // ── Slow down if obstacle is in SLOW zone but not STOP zone ───────────────
    if (robotState == STATE_FORWARD && dir == DIRECTION_FORWARD &&
        frontDist >= STOP_DIST_MM && frontDist < SLOW_DIST_MM) {
        // Reduce speed proportionally: 30-60% duty based on distance
        uint8_t slowDuty = static_cast<uint8_t>(
            30 + (frontDist - STOP_DIST_MM) / (SLOW_DIST_MM - STOP_DIST_MM) * 30);
        lidar.driveForward(slowDuty);
        return;
    }

    // ── Normal obstacle avoidance decision ────────────────────────────────────
    switch (dir) {
        case DIRECTION_CLEAR:
        case DIRECTION_FORWARD:
            // Resume full speed if we were slowing down
            if (robotState == STATE_FORWARD) {
                lidar.driveForward(robotCfg.dutyForward);
            }
            break;

        default:
            startAvoidance(dir);
            break;
    }
}

// ─── Avoidance action launcher ────────────────────────────────────────────────

static void startAvoidance(ObstacleDirection dir)
{
    switch (dir) {
        case DIRECTION_LEFT:
            Serial.print("OBSTACLE → turning left ");
            Serial.print(TURN_ANGLE_DEG, 0); Serial.println("°");
            lidar.startTurnLeft(TURN_ANGLE_DEG);
            break;

        case DIRECTION_RIGHT:
            Serial.print("OBSTACLE → turning right ");
            Serial.print(TURN_ANGLE_DEG, 0); Serial.println("°");
            lidar.startTurnRight(TURN_ANGLE_DEG);
            break;

        case DIRECTION_BACKWARD:
            Serial.print("OBSTACLE → backing up ");
            Serial.print(BACKUP_DIST_MM, 0); Serial.println(" mm then turning");
            lidar.startDriveBackward(BACKUP_DIST_MM);
            break;

        case DIRECTION_BLOCKED:
            Serial.println("BLOCKED on all sides – stopping.");
            lidar.motorStop();
            robotState = STATE_STOPPED;
            return;

        default:
            break;
    }
    robotState = STATE_AVOIDING;
}

// ─── Status printing ──────────────────────────────────────────────────────────

static void printStatus(ObstacleDirection dir, float frontDist)
{
    Serial.print("Scan #"); Serial.print(lidar.getScanCount());
    Serial.print("  Front: ");
    if (frontDist >= 0) {
        Serial.print(static_cast<int>(frontDist)); Serial.print(" mm");
    } else {
        Serial.print("---");
    }
    Serial.print("  L-RPM: "); Serial.print(lidar.getLeftRPM(),  1);
    Serial.print("  R-RPM: "); Serial.print(lidar.getRightRPM(), 1);
    Serial.print("  → "); Serial.println(dirName(dir));
}

static const char* dirName(ObstacleDirection d)
{
    switch (d) {
        case DIRECTION_CLEAR:    return "CLEAR";
        case DIRECTION_FORWARD:  return "FORWARD (slow)";
        case DIRECTION_LEFT:     return "TURN LEFT";
        case DIRECTION_RIGHT:    return "TURN RIGHT";
        case DIRECTION_BACKWARD: return "BACK UP";
        case DIRECTION_BLOCKED:  return "BLOCKED";
        default:                 return "?";
    }
}
