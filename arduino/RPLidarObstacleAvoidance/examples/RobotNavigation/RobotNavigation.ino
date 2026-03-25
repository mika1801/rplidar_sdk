/**
 * RobotNavigation.ino
 *
 * RPLidar A1M8 obstacle avoidance + quadrature encoder feedback on an
 * Arduino Giga R1 WiFi with Arduino Motor Shield Rev3 (L298P).
 *
 * ── Why no GPIO interrupts for the encoders? ─────────────────────────────────
 * The Giga R1 WiFi (STM32H747, Mbed OS) can crash / lock up when four
 * external-interrupt lines toggle at high frequency simultaneously.
 * At maximum motor speed (530 RPM) the Pololu 4751 encoder (64 CPR) produces
 *   530 RPM × 64 counts/rev ÷ 60 s ≈ 565 Hz per channel edge
 *   × 2 edges (rising + falling) ≈ 1 130 Hz per channel
 * Four channels together reach ~4 500 ISR calls/s – enough to destabilise
 * the Mbed OS interrupt controller.
 *
 * Solution: a single mbed::Ticker fires every 200 µs (5 kHz).  It reads all
 * four pins and runs a standard 16-entry quadrature lookup table.
 *   • One ISR instead of four  → predictable, low-jitter timing
 *   • 5 kHz >> 2 × 1 130 Hz Nyquist limit  → no pulses missed at max speed
 *   • CPU cost ≈ 4 × digitalRead ≈ 8 µs / 200 µs window ≈ 4 %
 *
 * ── Wiring – RPLidar ─────────────────────────────────────────────────────────
 *   RPLidar TX    →  Serial1 RX  (D0)
 *   RPLidar RX    →  Serial1 TX  (D1)
 *   RPLidar 5 V   →  5 V
 *   RPLidar GND   →  GND
 *   MOTO_CTRL     →  D2  (HIGH = motor spinning)
 *
 * ── Arduino Motor Shield Rev3 – fixed pin mapping ────────────────────────────
 *   Channel A  (left  motor): DIR=D12  PWM=D3   BRAKE=D9   SENSE=A0
 *   Channel B  (right motor): DIR=D13  PWM=D11  BRAKE=D8   SENSE=A1
 *
 *   DIR HIGH → forward,  LOW → reverse
 *   BRAKE HIGH → hard brake
 *
 * ── Encoder wiring – Pololu 4751 (19:1 37D 12V, 64 CPR quadrature) ───────────
 *   Encoder Vcc  Blue   →  5 V  (spec: 3.5 – 20 V, max 10 mA)
 *   Encoder GND  Green  →  GND
 *
 *   Left  motor  Enc-A  Yellow  →  D4
 *   Left  motor  Enc-B  White   →  D5
 *   Right motor  Enc-A  Yellow  →  D6
 *   Right motor  Enc-B  White   →  D7
 *
 * ── Differential drive convention ────────────────────────────────────────────
 *   FORWARD: left wheel CCW, right wheel CW  (viewed from outside)
 *   DIR_A HIGH → left  motor forward (CCW)
 *   DIR_B HIGH → right motor forward (CW)
 *
 *   Because the two motors face in opposite directions, the right encoder
 *   naturally counts in the opposite direction during forward motion.
 *   ENC_R_INVERT = true corrects this so both counters increase when the
 *   robot drives forward.  If the robot moves backward but a counter still
 *   increases, swap the matching INVERT flag.
 *
 * ── Encoder maths ────────────────────────────────────────────────────────────
 *   64 CPR × 19 gear ratio = 1 216 counts / revolution of the output shaft
 *   Example wheel (⌀ 65 mm): π × 65 mm ≈ 204 mm / rev
 *   → 1 216 / 204 ≈ 5.96 counts / mm of travel
 *
 * ── Coordinate convention (Lidar) ────────────────────────────────────────────
 *   0° = front, 90° = left, 180° = rear, 270° = right
 */

#include <Arduino.h>
#include <mbed.h>
#include <RPLidarObstacleAvoidance.h>

// ─── Lidar configuration ─────────────────────────────────────────────────────
static const int   LIDAR_MOTOR_PIN = 2;
static const float STOP_DIST_MM   = 300.0f;   // hard stop  < 30 cm
static const float SLOW_DIST_MM   = 600.0f;   // slow down  < 60 cm

RPLidarObstacleAvoidance lidar(Serial1, LIDAR_MOTOR_PIN);

// ─── Motor Shield Rev3 – pin mapping ─────────────────────────────────────────
static const int PIN_DIR_A   = 12;
static const int PIN_PWM_A   =  3;
static const int PIN_BRAKE_A =  9;
static const int PIN_DIR_B   = 13;
static const int PIN_PWM_B   = 11;
static const int PIN_BRAKE_B =  8;

static const int SPEED_FULL = 200;   // 0–255 PWM
static const int SPEED_SLOW = 120;
static const int SPEED_TURN = 160;

// ─── Encoder pins ─────────────────────────────────────────────────────────────
static const int ENC_L_A = 4;   // Left  motor – channel A (Yellow lead)
static const int ENC_L_B = 5;   // Left  motor – channel B (White  lead)
static const int ENC_R_A = 6;   // Right motor – channel A (Yellow lead)
static const int ENC_R_B = 7;   // Right motor – channel B (White  lead)

// Invert a motor's encoder count direction so that forward motion always
// produces a positive increment on both sides.
// Default: right motor inverted because it spins CW vs the left motor's CCW.
// If counts go the wrong way after testing, flip the relevant constant.
static const bool ENC_L_INVERT = false;
static const bool ENC_R_INVERT = true;

// ─── Encoder state ────────────────────────────────────────────────────────────
//
// Standard quadrature lookup table.
// Index = (prevAB << 2) | currAB  where AB = (A_bit << 1) | B_bit
//   +1 → forward step, −1 → backward step, 0 → no change / error
//
static const int8_t QEM[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static volatile int32_t _encLeft  = 0;
static volatile int32_t _encRight = 0;

// Previous encoder states (not volatile – only touched inside the ISR)
static uint8_t _prevL = 0;
static uint8_t _prevR = 0;

static mbed::Ticker _encTicker;

/**
 * Ticker ISR – called every 200 µs (5 kHz).
 * Samples all four encoder pins and updates the quadrature counters.
 * Keep this function short; no Serial, no malloc, no blocking calls.
 */
static void encSample()
{
    // Read current pin states
    uint8_t currL = ((uint8_t)digitalRead(ENC_L_A) << 1) | (uint8_t)digitalRead(ENC_L_B);
    uint8_t currR = ((uint8_t)digitalRead(ENC_R_A) << 1) | (uint8_t)digitalRead(ENC_R_B);

    // Quadrature decode
    int8_t dL = QEM[(_prevL << 2) | currL];
    int8_t dR = QEM[(_prevR << 2) | currR];

    // Apply direction correction and accumulate
    _encLeft  += ENC_L_INVERT ? -dL : dL;
    _encRight += ENC_R_INVERT ? -dR : dR;

    _prevL = currL;
    _prevR = currR;
}

// ─── Encoder public API ───────────────────────────────────────────────────────

/** Read left encoder count (positive = forward). Thread-safe snapshot. */
int32_t getEncLeft()
{
    noInterrupts();
    int32_t v = _encLeft;
    interrupts();
    return v;
}

/** Read right encoder count (positive = forward). Thread-safe snapshot. */
int32_t getEncRight()
{
    noInterrupts();
    int32_t v = _encRight;
    interrupts();
    return v;
}

/** Zero both counters atomically. */
void resetEncoders()
{
    noInterrupts();
    _encLeft = _encRight = 0;
    interrupts();
}

// ─── Robot state ──────────────────────────────────────────────────────────────
enum RobotState {
    ROBOT_FORWARD,
    ROBOT_TURN_LEFT,
    ROBOT_TURN_RIGHT,
    ROBOT_BACKWARD,
    ROBOT_STOP,
};

static RobotState currentState = ROBOT_STOP;
static uint32_t   stateStartMs = 0;
static const uint32_t TURN_DURATION_MS   = 600;
static const uint32_t BACKUP_DURATION_MS = 400;

// ─── Forward declarations ─────────────────────────────────────────────────────
static void setChannel(int dirPin, int pwmPin, int brakePin, int pwm);
static void setMotors(int leftPwm, int rightPwm);
static void driveForward(int speed);
static void driveBackward(int speed);
static void turnLeft(int speed);
static void turnRight(int speed);
static void motorStop();
static void applyLidarDecision(ObstacleDirection dir, float frontDist);
static void applyCurrentState();
static const char* dirToString(ObstacleDirection d);

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // Motor Shield Rev3
    pinMode(PIN_DIR_A,   OUTPUT); pinMode(PIN_PWM_A,   OUTPUT); pinMode(PIN_BRAKE_A, OUTPUT);
    pinMode(PIN_DIR_B,   OUTPUT); pinMode(PIN_PWM_B,   OUTPUT); pinMode(PIN_BRAKE_B, OUTPUT);
    motorStop();

    // Encoder inputs – use internal pull-ups; encoder outputs are push-pull
    // (3.3 V / 5 V active) so pull-up only matters when disconnected
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    // Prime the previous-state variables with the actual pin levels so the
    // very first Ticker call does not generate a spurious step
    _prevL = ((uint8_t)digitalRead(ENC_L_A) << 1) | (uint8_t)digitalRead(ENC_L_B);
    _prevR = ((uint8_t)digitalRead(ENC_R_A) << 1) | (uint8_t)digitalRead(ENC_R_B);

    // Start the 5 kHz encoder sampler
    // attach_us(callback, period_in_microseconds)
    _encTicker.attach_us(encSample, 200);

    Serial.println("RPLidar A1M8 – Robot Navigation with Encoder Feedback");
    Serial.println("Encoder sampler: 5 kHz ticker (no GPIO interrupts)");
    Serial.println("Initialising lidar (~3 s)…");

    lidar.setFrontSector(  0.0f, 60.0f);
    lidar.setLeftSector  ( 90.0f, 80.0f);
    lidar.setRightSector (270.0f, 80.0f);
    lidar.setBackSector  (180.0f, 60.0f);
    lidar.setMinQuality(8);
    lidar.begin();

    Serial.println("Lidar ready – starting navigation.");
    resetEncoders();
    currentState = ROBOT_FORWARD;
    stateStartMs = millis();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop()
{
    // Always pump the lidar RX buffer first
    lidar.update();

    if (!lidar.isConnected()) {
        Serial.println("LIDAR DISCONNECTED – stopping robot!");
        motorStop();
        delay(1000);
        return;
    }

    // Print encoder telemetry every 500 ms
    static uint32_t lastEncPrint = 0;
    if (millis() - lastEncPrint >= 500) {
        lastEncPrint = millis();
        Serial.print("  EncL="); Serial.print(getEncLeft());
        Serial.print("  EncR="); Serial.println(getEncRight());
    }

    if (!lidar.isScanComplete()) {
        applyCurrentState();
        return;
    }

    ObstacleDirection dir       = lidar.getRecommendedDirection(STOP_DIST_MM);
    float             frontDist = lidar.getMinDistance(330.0f, 30.0f);

    Serial.print("Scan #"); Serial.print(lidar.getScanCount());
    Serial.print("  Front: ");
    if (frontDist >= 0) { Serial.print((int)frontDist); Serial.print(" mm"); }
    else Serial.print("---");
    Serial.print("  Dir: "); Serial.println(dirToString(dir));

    applyLidarDecision(dir, frontDist);
}

// ─── Navigation logic ─────────────────────────────────────────────────────────
static void applyLidarDecision(ObstacleDirection dir, float frontDist)
{
    uint32_t elapsed = millis() - stateStartMs;
    if ((currentState == ROBOT_TURN_LEFT || currentState == ROBOT_TURN_RIGHT) &&
         elapsed < TURN_DURATION_MS)  return;
    if (currentState == ROBOT_BACKWARD && elapsed < BACKUP_DURATION_MS) return;

    RobotState newState = currentState;
    switch (dir) {
        case DIRECTION_CLEAR:
        case DIRECTION_FORWARD:  newState = ROBOT_FORWARD;    break;
        case DIRECTION_LEFT:     newState = ROBOT_TURN_LEFT;  break;
        case DIRECTION_RIGHT:    newState = ROBOT_TURN_RIGHT; break;
        case DIRECTION_BACKWARD: newState = ROBOT_BACKWARD;   break;
        case DIRECTION_BLOCKED:  newState = ROBOT_STOP;       break;
    }

    if (newState != currentState) {
        currentState = newState;
        stateStartMs = millis();
    }
    applyCurrentState();
}

static void applyCurrentState()
{
    switch (currentState) {
        case ROBOT_FORWARD:    driveForward(SPEED_FULL);  break;
        case ROBOT_TURN_LEFT:  turnLeft(SPEED_TURN);      break;
        case ROBOT_TURN_RIGHT: turnRight(SPEED_TURN);     break;
        case ROBOT_BACKWARD:   driveBackward(SPEED_SLOW); break;
        case ROBOT_STOP:
        default:               motorStop();               break;
    }
}

// ─── Motor helpers (Arduino Motor Shield Rev3, L298P) ─────────────────────────
//
// setMotors(leftPwm, rightPwm):
//   +speed → forward direction for that side
//   -speed → reverse direction for that side
//
// Forward motion:
//   Left  motor – DIR_A HIGH (CCW, viewed from left  side of robot)
//   Right motor – DIR_B HIGH (CW,  viewed from right side of robot)

static void setChannel(int dirPin, int pwmPin, int brakePin, int pwm)
{
    digitalWrite(brakePin, LOW);   // release brake before changing direction
    if (pwm >= 0) {
        digitalWrite(dirPin, HIGH);
        analogWrite(pwmPin,  pwm);
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
    analogWrite(PIN_PWM_A, 0); digitalWrite(PIN_BRAKE_A, HIGH);
    analogWrite(PIN_PWM_B, 0); digitalWrite(PIN_BRAKE_B, HIGH);
}

// ─── Utility ──────────────────────────────────────────────────────────────────
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
