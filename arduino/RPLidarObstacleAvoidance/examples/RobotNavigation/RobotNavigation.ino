/**
 * RobotNavigation.ino
 *
 * Advanced example: RPLidar A1M8 obstacle avoidance integrated with a
 * differential-drive robot on the Arduino Giga R1 WiFi.
 *
 * Two DC motors are driven via an L298N (or compatible) H-bridge module.
 * Motor commands are sent via PWM + direction pins.
 *
 * Wiring – RPLidar
 * ────────────────
 *   RPLidar TX    →  Serial1 RX  (D0)
 *   RPLidar RX    →  Serial1 TX  (D1)
 *   RPLidar 5 V   →  5 V
 *   RPLidar GND   →  GND
 *   MOTO_CTRL     →  D2  (HIGH = motor spinning)
 *
 * Wiring – L298N H-bridge
 * ───────────────────────
 *   ENA (left PWM)   →  D5
 *   IN1 (left fwd)   →  D6
 *   IN2 (left rev)   →  D7
 *   ENB (right PWM)  →  D8
 *   IN3 (right fwd)  →  D9
 *   IN4 (right rev)  →  D10
 *
 * Coordinate convention:
 *   The RPLidar sits facing forward on the robot.
 *   0°  = front, 90° = left, 180° = rear, 270° = right.
 */

#include <RPLidarObstacleAvoidance.h>

// ─── Lidar configuration ────────────────────────────────────────────────────
static const int   LIDAR_MOTOR_PIN = 2;
static const float STOP_DIST_MM    = 300.0f;  // Stop when closer than 30 cm
static const float SLOW_DIST_MM    = 600.0f;  // Slow down when closer than 60 cm

RPLidarObstacleAvoidance lidar(Serial1, LIDAR_MOTOR_PIN);

// ─── Motor pins ─────────────────────────────────────────────────────────────
static const int PIN_ENA = 5;   // Left  motor PWM
static const int PIN_IN1 = 6;   // Left  motor forward
static const int PIN_IN2 = 7;   // Left  motor reverse
static const int PIN_ENB = 8;   // Right motor PWM
static const int PIN_IN3 = 9;   // Right motor forward
static const int PIN_IN4 = 10;  // Right motor reverse

static const int SPEED_FULL  = 200;  // 0-255 PWM
static const int SPEED_SLOW  = 120;
static const int SPEED_TURN  = 160;

// ─── Robot state ─────────────────────────────────────────────────────────────
enum RobotState {
    ROBOT_FORWARD,
    ROBOT_TURN_LEFT,
    ROBOT_TURN_RIGHT,
    ROBOT_BACKWARD,
    ROBOT_STOP,
};

static RobotState currentState = ROBOT_STOP;
static uint32_t   stateStartMs = 0;
static const uint32_t TURN_DURATION_MS   = 600;   // how long to turn
static const uint32_t BACKUP_DURATION_MS = 400;   // how long to back up

// ─── Forward declarations ────────────────────────────────────────────────────
static void setMotors(int leftPwm, int rightPwm);
static void driveForward(int speed);
static void driveBackward(int speed);
static void turnLeft(int speed);
static void turnRight(int speed);
static void motorStop();
static void applyLidarDecision(ObstacleDirection dir, float frontDist);

// ─── Setup ──────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // Motor pins
    pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
    motorStop();

    Serial.println("RPLidar A1M8 – Robot Navigation Example");
    Serial.println("Initialising lidar (~3 s)…");

    // Fine-tune the sectors to match robot geometry if needed.
    // Default: front = 0° ±30°, left = 90° ±30°, right = 270° ±30°, back = 180° ±30°
    lidar.setFrontSector(  0.0f, 60.0f);  // ±30°
    lidar.setLeftSector  ( 90.0f, 80.0f); // ±40°
    lidar.setRightSector (270.0f, 80.0f); // ±40°
    lidar.setBackSector  (180.0f, 60.0f); // ±30°
    lidar.setMinQuality(8);               // accept slightly lower quality readings

    lidar.begin();

    Serial.println("Lidar ready – starting navigation.");
    currentState = ROBOT_FORWARD;
    stateStartMs = millis();
}

// ─── Loop ───────────────────────────────────────────────────────────────────

void loop()
{
    // Always pump the lidar RX buffer first
    lidar.update();

    // Check lidar health
    if (!lidar.isConnected()) {
        Serial.println("LIDAR DISCONNECTED – stopping robot!");
        motorStop();
        delay(1000);
        return;
    }

    // Act only when a fresh full scan is available
    if (!lidar.isScanComplete()) {
        // While waiting for the next scan, continue the timed manoeuvre
        applyCurrentState();
        return;
    }

    // ── Obstacle assessment ──────────────────────────────────────────────
    ObstacleDirection dir      = lidar.getRecommendedDirection(STOP_DIST_MM);
    float             frontDist = lidar.getMinDistance(330.0f, 30.0f); // 330°-30°

    // Log current state to USB serial
    Serial.print("Scan #"); Serial.print(lidar.getScanCount());
    Serial.print("  Front: ");
    if (frontDist >= 0) { Serial.print((int)frontDist); Serial.print(" mm"); }
    else Serial.print("---");
    Serial.print("  Dir: ");
    Serial.println(dirToString(dir));

    // ── Apply decision ───────────────────────────────────────────────────
    applyLidarDecision(dir, frontDist);
}

// ─── Navigation logic ───────────────────────────────────────────────────────

static void applyLidarDecision(ObstacleDirection dir, float frontDist)
{
    // If a timed manoeuvre (turn / backup) is still running, don't interrupt
    uint32_t elapsed = millis() - stateStartMs;
    if ((currentState == ROBOT_TURN_LEFT  || currentState == ROBOT_TURN_RIGHT) &&
         elapsed < TURN_DURATION_MS)  return;
    if (currentState == ROBOT_BACKWARD && elapsed < BACKUP_DURATION_MS) return;

    RobotState newState = currentState;

    switch (dir) {
        case DIRECTION_CLEAR:
        case DIRECTION_FORWARD:
            newState = ROBOT_FORWARD;
            break;

        case DIRECTION_LEFT:
            // Start a left turn
            newState = ROBOT_TURN_LEFT;
            break;

        case DIRECTION_RIGHT:
            newState = ROBOT_TURN_RIGHT;
            break;

        case DIRECTION_BACKWARD:
            newState = ROBOT_BACKWARD;
            break;

        case DIRECTION_BLOCKED:
            newState = ROBOT_STOP;
            break;
    }

    if (newState != currentState) {
        currentState = newState;
        stateStartMs = millis();
    }

    applyCurrentState();
}

// Execute the current robot state (called every loop iteration)
static void applyCurrentState()
{
    switch (currentState) {
        case ROBOT_FORWARD:
            driveForward(SPEED_FULL);
            break;
        case ROBOT_TURN_LEFT:
            turnLeft(SPEED_TURN);
            break;
        case ROBOT_TURN_RIGHT:
            turnRight(SPEED_TURN);
            break;
        case ROBOT_BACKWARD:
            driveBackward(SPEED_SLOW);
            break;
        case ROBOT_STOP:
        default:
            motorStop();
            break;
    }
}

// ─── Motor helpers ───────────────────────────────────────────────────────────

static void setMotors(int leftPwm, int rightPwm)
{
    // Left motor
    if (leftPwm >= 0) {
        digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, leftPwm);
    } else {
        digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
        analogWrite(PIN_ENA, -leftPwm);
    }

    // Right motor
    if (rightPwm >= 0) {
        digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, rightPwm);
    } else {
        digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, HIGH);
        analogWrite(PIN_ENB, -rightPwm);
    }
}

static void driveForward (int speed) { setMotors( speed,  speed); }
static void driveBackward(int speed) { setMotors(-speed, -speed); }
static void turnLeft     (int speed) { setMotors(-speed,  speed); }
static void turnRight    (int speed) { setMotors( speed, -speed); }
static void motorStop()
{
    analogWrite(PIN_ENA, 0); analogWrite(PIN_ENB, 0);
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

// ─── Utility ─────────────────────────────────────────────────────────────────

static const char* dirToString(ObstacleDirection d)
{
    switch (d) {
        case DIRECTION_CLEAR:    return "CLEAR";
        case DIRECTION_FORWARD:  return "FORWARD";
        case DIRECTION_LEFT:     return "TURN LEFT";
        case DIRECTION_RIGHT:    return "TURN RIGHT";
        case DIRECTION_BACKWARD: return "BACKWARD";
        case DIRECTION_BLOCKED:  return "BLOCKED";
        default:                 return "?";
    }
}
