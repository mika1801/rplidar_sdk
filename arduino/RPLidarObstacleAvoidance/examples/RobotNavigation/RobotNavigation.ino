/**
 * RobotNavigation.ino
 *
 * Advanced example: RPLidar A1M8 obstacle avoidance integrated with a
 * differential-drive robot on the Arduino Giga R1 WiFi.
 *
 * Two DC motors are driven via the Arduino Motor Shield Rev3 (L298P).
 * The shield stacks directly on top of the Giga R1 WiFi.
 *
 * Wiring – RPLidar
 * ────────────────
 *   RPLidar TX    →  Serial1 RX  (D0)
 *   RPLidar RX    →  Serial1 TX  (D1)
 *   RPLidar 5 V   →  5 V
 *   RPLidar GND   →  GND
 *   MOTO_CTRL     →  D2  (HIGH = motor spinning)
 *
 * Arduino Motor Shield Rev3 – fixed pin mapping (no wiring needed)
 * ────────────────────────────────────────────────────────────────
 *   Channel A  (left  motor): DIR=D12  PWM=D3   BRAKE=D9   SENSE=A0
 *   Channel B  (right motor): DIR=D13  PWM=D11  BRAKE=D8   SENSE=A1
 *
 *   DIR  HIGH → forward,  LOW → reverse
 *   BRAKE HIGH → hard brake (coasts when LOW)
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

// ─── Motor Shield Rev3 – fixed pin mapping ──────────────────────────────────
// Channel A = left motor, Channel B = right motor
static const int PIN_DIR_A   = 12;  // Channel A direction (HIGH=fwd, LOW=rev)
static const int PIN_PWM_A   =  3;  // Channel A speed (PWM)
static const int PIN_BRAKE_A =  9;  // Channel A brake  (HIGH=brake, LOW=coast)
static const int PIN_DIR_B   = 13;  // Channel B direction
static const int PIN_PWM_B   = 11;  // Channel B speed (PWM)
static const int PIN_BRAKE_B =  8;  // Channel B brake

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

    // Motor Shield Rev3 pins
    pinMode(PIN_DIR_A,   OUTPUT); pinMode(PIN_PWM_A,   OUTPUT); pinMode(PIN_BRAKE_A, OUTPUT);
    pinMode(PIN_DIR_B,   OUTPUT); pinMode(PIN_PWM_B,   OUTPUT); pinMode(PIN_BRAKE_B, OUTPUT);
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

// ─── Motor helpers (Arduino Motor Shield Rev3) ──────────────────────────────
//
// Motor Shield Rev3 interface per channel:
//   DIR   HIGH = forward,  LOW = reverse
//   BRAKE HIGH = hard brake (motor short-circuits internally)
//   PWM   analogWrite speed 0-255

static void setChannel(int dirPin, int pwmPin, int brakePin, int pwm)
{
    // Release brake before changing speed/direction
    digitalWrite(brakePin, LOW);
    if (pwm >= 0) {
        digitalWrite(dirPin, HIGH);
        analogWrite(pwmPin, pwm);
    } else {
        digitalWrite(dirPin, LOW);
        analogWrite(pwmPin, -pwm);
    }
}

static void setMotors(int leftPwm, int rightPwm)
{
    setChannel(PIN_DIR_A, PIN_PWM_A, PIN_BRAKE_A, leftPwm);
    setChannel(PIN_DIR_B, PIN_PWM_B, PIN_BRAKE_B, rightPwm);
}

static void driveForward (int speed) { setMotors( speed,  speed); }
static void driveBackward(int speed) { setMotors(-speed, -speed); }
static void turnLeft     (int speed) { setMotors(-speed,  speed); }
static void turnRight    (int speed) { setMotors( speed, -speed); }
static void motorStop()
{
    // Apply hard brake on both channels
    analogWrite(PIN_PWM_A, 0); digitalWrite(PIN_BRAKE_A, HIGH);
    analogWrite(PIN_PWM_B, 0); digitalWrite(PIN_BRAKE_B, HIGH);
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
