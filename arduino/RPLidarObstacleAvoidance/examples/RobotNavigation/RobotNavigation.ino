/**
 * RobotNavigation.ino  – Autonomous Exploration, 45° Encoder Turns
 *
 * RPLidar A1M8 obstacle avoidance + autonomous room exploration on an
 * Arduino Giga R1 WiFi with Arduino Motor Shield Rev3 (L298P).
 * Motor: Pololu 4751 – 19:1 37D 12V with 64 CPR quadrature encoder.
 *
 * ── Exploration strategy ──────────────────────────────────────────────────────
 *   • Drive forward as long as the path is clear.
 *   • When an obstacle is detected: reverse a fixed distance, then evaluate
 *     seven candidate headings in 45° steps (±45°, ±90°, ±135°, 180°).
 *   • Turn encoder-controlled to the heading with the greatest clearance.
 *   • A periodic direction bias (alternates every 6 s) breaks symmetry and
 *     prevents the robot from cycling through the same loop repeatedly.
 *
 * ── Why encoder-based turns? ─────────────────────────────────────────────────
 *   Time-based turns drift with battery voltage and floor surface.
 *   Encoder turns count actual wheel rotations → consistent angle regardless
 *   of motor load or supply voltage.
 *
 *   counts_for_angle = (WHEEL_BASE_MM / 2 × angle_rad) / (π × WHEEL_DIAM_MM)
 *                      × COUNTS_OUTPUT_REV
 *   Example (200 mm base, 65 mm wheel, 1216 CPR_output):
 *     ≈ 10.4 counts / degree  →  468 counts per 45°
 *
 *   Set WHEEL_BASE_MM to the centre-to-centre track width of your robot.
 *
 * ── Why no GPIO interrupts for encoders? ─────────────────────────────────────
 *   Four simultaneous high-frequency interrupt lines crash the Giga R1 WiFi.
 *   A single mbed::Ticker at 5 kHz polls all four pins instead.
 *   At max speed (530 RPM, 64 CPR) the encoder produces ≈ 1 130 Hz/channel –
 *   well within the 2.5 kHz Nyquist limit of a 5 kHz sampler.
 *
 * ── Wiring – RPLidar ──────────────────────────────────────────────────────────
 *   RPLidar TX    →  Serial1 RX  (D0)
 *   RPLidar RX    →  Serial1 TX  (D1)
 *   RPLidar 5 V   →  5 V
 *   RPLidar GND   →  GND
 *   MOTO_CTRL     →  D2  (HIGH = motor spinning)
 *
 * ── Motor Shield Rev3 – fixed pin mapping ────────────────────────────────────
 *   Channel A  (left  motor): DIR=D12  PWM=D3   BRAKE=D9   SENSE=A0
 *   Channel B  (right motor): DIR=D13  PWM=D11  BRAKE=D8   SENSE=A1
 *
 * ── Encoder wiring – Pololu 4751 ─────────────────────────────────────────────
 *   Encoder Vcc  Blue   →  5 V
 *   Encoder GND  Green  →  GND
 *   Left  motor  Enc-A  Yellow  →  D4
 *   Left  motor  Enc-B  White   →  D5
 *   Right motor  Enc-A  Yellow  →  D6
 *   Right motor  Enc-B  White   →  D7
 *
 * ── Differential drive convention ────────────────────────────────────────────
 *   FORWARD: left wheel CCW, right wheel CW (viewed from outside)
 *   DIR_A HIGH → left  motor forward (CCW)
 *   DIR_B HIGH → right motor forward (CW)
 *   ENC_R_INVERT = true normalises both counters: positive = forward.
 *
 * ── Lidar angle convention ────────────────────────────────────────────────────
 *   0° = front,  90° = left,  180° = rear,  270° = right
 */

#include <Arduino.h>
#include <mbed.h>
#include <RPLidarObstacleAvoidance.h>

// ─── Lidar ────────────────────────────────────────────────────────────────────
static const int   LIDAR_MOTOR_PIN = 2;
static const float DANGER_MM       = 350.0f;   // trigger avoidance below this distance

RPLidarObstacleAvoidance lidar(Serial1, LIDAR_MOTOR_PIN);

// ─── Motor Shield Rev3 – pins ─────────────────────────────────────────────────
static const int PIN_DIR_A   = 12;
static const int PIN_PWM_A   =  3;
static const int PIN_BRAKE_A =  9;
static const int PIN_DIR_B   = 13;
static const int PIN_PWM_B   = 11;
static const int PIN_BRAKE_B =  8;

static const int SPEED_DRIVE = 180;   // 0–255 PWM when driving forward
static const int SPEED_BACK  = 120;   // speed during reverse
static const int SPEED_TURN  = 140;   // speed during in-place rotation

// ─── Robot geometry – ADJUST TO YOUR ROBOT ────────────────────────────────────
static const float WHEEL_BASE_MM     = 200.0f;  // centre-to-centre track width [mm]
static const float WHEEL_DIAM_MM     =  65.0f;  // wheel outer diameter [mm]
static const int   COUNTS_OUTPUT_REV = 1216;    // 64 CPR × 19:1 gearbox
static const float BACKUP_MM         = 150.0f;  // how far to reverse before turning

// Pre-computed conversion factors
// counts/mm  =  COUNTS_OUTPUT_REV / (π × WHEEL_DIAM_MM)
static const float COUNTS_PER_MM =
    (float)COUNTS_OUTPUT_REV / (PI * WHEEL_DIAM_MM);

// counts/degree of in-place rotation  (one wheel each direction)
//   arc per wheel = (WHEEL_BASE/2) × (π/180) per degree
//   counts = arc / (π × WHEEL_DIAM) × COUNTS_OUTPUT_REV
//   = WHEEL_BASE / (360 × WHEEL_DIAM) × COUNTS_OUTPUT_REV
static const float COUNTS_PER_DEG =
    WHEEL_BASE_MM / (360.0f * WHEEL_DIAM_MM) * (float)COUNTS_OUTPUT_REV;
//  e.g. 200 / (360 × 65) × 1216 ≈ 10.4  →  ~468 counts per 45°

// ─── Encoder – 5 kHz ticker sampling (avoids Giga R1 interrupt crashes) ───────
static const int  ENC_L_A = 4;
static const int  ENC_L_B = 5;
static const int  ENC_R_A = 6;
static const int  ENC_R_B = 7;

// Right motor spins CW for forward → flip so positive always means forward
static const bool ENC_L_INVERT = false;
static const bool ENC_R_INVERT = true;

// Quadrature lookup: index = (prevAB << 2) | currAB, AB = (A << 1) | B
static const int8_t QEM[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static volatile int32_t _encLeft  = 0;
static volatile int32_t _encRight = 0;
static uint8_t _prevL = 0, _prevR = 0;
static mbed::Ticker _encTicker;

static void encSample()
{
    uint8_t cL = ((uint8_t)digitalRead(ENC_L_A) << 1) | (uint8_t)digitalRead(ENC_L_B);
    uint8_t cR = ((uint8_t)digitalRead(ENC_R_A) << 1) | (uint8_t)digitalRead(ENC_R_B);
    int8_t  dL = QEM[(_prevL << 2) | cL];
    int8_t  dR = QEM[(_prevR << 2) | cR];
    _encLeft  += ENC_L_INVERT ? -dL : dL;
    _encRight += ENC_R_INVERT ? -dR : dR;
    _prevL = cL;
    _prevR = cR;
}

static int32_t getEncLeft()  { noInterrupts(); int32_t v = _encLeft;  interrupts(); return v; }
static int32_t getEncRight() { noInterrupts(); int32_t v = _encRight; interrupts(); return v; }

// ─── Exploration state machine ────────────────────────────────────────────────
enum ExploreState {
    EX_FORWARD,    // driving straight ahead
    EX_BACKING,    // reversing before a turn
    EX_TURNING,    // encoder-controlled in-place rotation
    EX_STOP,       // lidar disconnected / error
};
static ExploreState exState = EX_STOP;

// Encoder snapshots captured at the start of each maneuver
static int32_t _mStartL      = 0;
static int32_t _mStartR      = 0;
static int32_t _mTargetCounts = 0;
static int     _turnDir      = +1;   // +1 = left (CCW), −1 = right (CW)

// ─── Maneuver helpers ─────────────────────────────────────────────────────────

/** Begin reversing BACKUP_MM. */
static void startBackup()
{
    _mStartL       = getEncLeft();
    _mStartR       = getEncRight();
    _mTargetCounts = (int32_t)(BACKUP_MM * COUNTS_PER_MM);
    exState        = EX_BACKING;
}

/** True once both wheels have reversed BACKUP_MM. */
static bool backupDone()
{
    // Backing up → encoder counts decrease
    int32_t dL  = _mStartL - getEncLeft();
    int32_t dR  = _mStartR - getEncRight();
    return ((dL + dR) / 2) >= _mTargetCounts;
}

/**
 * Begin an encoder-controlled in-place turn.
 * @param degrees  Positive → turn left (CCW), negative → turn right (CW)
 */
static void startTurn(float degrees)
{
    _mStartL       = getEncLeft();
    _mStartR       = getEncRight();
    _mTargetCounts = (int32_t)(fabsf(degrees) * COUNTS_PER_DEG);
    _turnDir       = (degrees >= 0.0f) ? +1 : -1;
    exState        = EX_TURNING;
}

/** True once the rotation has reached the target angle. */
static bool turnDone()
{
    int32_t dL, dR;
    if (_turnDir > 0) {
        // Left turn: left wheel backward (counts ↓), right wheel forward (counts ↑)
        dL = _mStartL - getEncLeft();
        dR = getEncRight() - _mStartR;
    } else {
        // Right turn: left wheel forward (counts ↑), right wheel backward (counts ↓)
        dL = getEncLeft()  - _mStartL;
        dR = _mStartR - getEncRight();
    }
    return ((dL + dR) / 2) >= _mTargetCounts;
}

// ─── Direction selection – 45° granularity ───────────────────────────────────
/**
 * Evaluate seven candidate headings at ±45°, ±90°, ±135°, and 180° relative
 * to the current robot heading, using the most recent complete lidar scan.
 * Returns the turn delta in degrees:
 *   positive → turn left,  negative → turn right.
 *
 * Tie-break: if two headings have equal clearance, the one requiring less
 * rotation is preferred.  A time-based bias (flips every 6 s) breaks
 * remaining ties and prevents the robot from looping in symmetric corridors.
 *
 * A scan point that returns no reading (open space / out of lidar range) is
 * treated as maximum clearance (9 999 mm) to encourage the robot to explore
 * uncharted territory.
 */
static float chooseTurnAngle()
{
    // Candidate lidar angles (0° = front is intentionally excluded – it was blocked)
    // and their corresponding turn deltas (positive = left, negative = right)
    static const float CAND_ANGLE[7] = {  45,  90,  135,  180,  225, 270,  315 };
    static const float CAND_DELTA[7] = { +45, +90, +135, +180, -135, -90,  -45 };
    static const float HALF_WIN      = 22.5f;   // ±22.5° window around each heading
    static const int   N             = 7;

    // Time-based tie-break bias: alternates every 6 s to randomise left/right preference
    bool biasLeft = ((millis() / 6000UL) % 2) == 0;

    float bestClearance = -1.0f;
    float bestDelta     = biasLeft ? +90.0f : -90.0f;   // fallback

    for (int i = 0; i < N; i++) {
        float dist = lidar.getMinDistance(CAND_ANGLE[i] - HALF_WIN,
                                          CAND_ANGLE[i] + HALF_WIN);
        // No valid reading → treat as open/unexplored space
        float clearance = (dist < 0.0f) ? 9999.0f : dist;

        bool better;
        if (clearance > bestClearance) {
            better = true;
        } else if (clearance == bestClearance) {
            // Equal clearance: prefer smaller rotation; use bias to break last tie
            float absNew  = fabsf(CAND_DELTA[i]);
            float absBest = fabsf(bestDelta);
            better = (absNew < absBest) ||
                     (absNew == absBest && (biasLeft ? CAND_DELTA[i] > 0
                                                     : CAND_DELTA[i] < 0));
        } else {
            better = false;
        }

        if (better) {
            bestClearance = clearance;
            bestDelta     = CAND_DELTA[i];
        }
    }

    Serial.print("  → turn "); Serial.print(bestDelta, 0);
    Serial.print("°   clearance ");
    if (bestClearance >= 9998.0f) Serial.println(">range");
    else { Serial.print((int)bestClearance); Serial.println(" mm"); }

    return bestDelta;
}

// ─── Motor helpers ────────────────────────────────────────────────────────────
static void setChannel(int dirPin, int pwmPin, int brakePin, int pwm)
{
    digitalWrite(brakePin, LOW);
    if (pwm >= 0) { digitalWrite(dirPin, HIGH); analogWrite(pwmPin,  pwm); }
    else          { digitalWrite(dirPin, LOW);  analogWrite(pwmPin, -pwm); }
}
static void setMotors(int l, int r)
{
    setChannel(PIN_DIR_A, PIN_PWM_A, PIN_BRAKE_A, l);
    setChannel(PIN_DIR_B, PIN_PWM_B, PIN_BRAKE_B, r);
}
// Forward: left CCW, right CW → both positive in normalised encoder convention
static void driveForward()  { setMotors( SPEED_DRIVE,  SPEED_DRIVE); }
static void driveBackward() { setMotors(-SPEED_BACK,  -SPEED_BACK);  }
// Left turn:  left backward, right forward
static void motorTurnLeft()  { setMotors(-SPEED_TURN,  SPEED_TURN); }
// Right turn: left forward,  right backward
static void motorTurnRight() { setMotors( SPEED_TURN, -SPEED_TURN); }
static void motorStop()
{
    analogWrite(PIN_PWM_A, 0); digitalWrite(PIN_BRAKE_A, HIGH);
    analogWrite(PIN_PWM_B, 0); digitalWrite(PIN_BRAKE_B, HIGH);
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // Motor Shield
    pinMode(PIN_DIR_A, OUTPUT); pinMode(PIN_PWM_A, OUTPUT); pinMode(PIN_BRAKE_A, OUTPUT);
    pinMode(PIN_DIR_B, OUTPUT); pinMode(PIN_PWM_B, OUTPUT); pinMode(PIN_BRAKE_B, OUTPUT);
    motorStop();

    // Encoder inputs
    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
    _prevL = ((uint8_t)digitalRead(ENC_L_A) << 1) | (uint8_t)digitalRead(ENC_L_B);
    _prevR = ((uint8_t)digitalRead(ENC_R_A) << 1) | (uint8_t)digitalRead(ENC_R_B);
    _encTicker.attach_us(encSample, 200);   // 5 kHz sampler

    Serial.println("=== Autonomous Exploration – 45° Encoder Turns ===");
    Serial.print("Counts/deg : "); Serial.println(COUNTS_PER_DEG,  2);
    Serial.print("Counts/45° : "); Serial.println(45.0f * COUNTS_PER_DEG, 0);
    Serial.print("Counts/mm  : "); Serial.println(COUNTS_PER_MM,   2);
    Serial.println("Initialising lidar (~3 s)…");

    lidar.setFrontSector(  0.0f, 60.0f);
    lidar.setLeftSector  ( 90.0f, 80.0f);
    lidar.setRightSector (270.0f, 80.0f);
    lidar.setBackSector  (180.0f, 60.0f);
    lidar.setMinQuality(8);
    lidar.begin();

    Serial.println("Lidar ready – starting exploration.");
    exState = EX_FORWARD;
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop()
{
    lidar.update();

    if (!lidar.isConnected()) {
        motorStop();
        exState = EX_STOP;
        Serial.println("LIDAR DISCONNECTED – halted.");
        delay(1000);
        return;
    }

    switch (exState) {

        // ── Forward ───────────────────────────────────────────────────────────
        case EX_FORWARD:
            driveForward();
            if (lidar.isScanComplete()) {
                float front = lidar.getMinDistance(330.0f, 30.0f);
                bool  blocked = lidar.isObstacleFront(DANGER_MM);
                Serial.print("FWD  encL="); Serial.print(getEncLeft());
                Serial.print("  encR=");    Serial.print(getEncRight());
                Serial.print("  front=");
                if (front >= 0) { Serial.print((int)front); Serial.print("mm"); }
                else Serial.print("---");
                Serial.println(blocked ? "  BLOCKED" : "  clear");

                if (blocked) startBackup();
            }
            break;

        // ── Reversing ─────────────────────────────────────────────────────────
        case EX_BACKING:
            driveBackward();
            if (backupDone()) {
                motorStop();
                delay(80);   // brief settle before choosing direction
                startTurn(chooseTurnAngle());
            }
            break;

        // ── Turning ───────────────────────────────────────────────────────────
        case EX_TURNING:
            (_turnDir > 0) ? motorTurnLeft() : motorTurnRight();
            if (turnDone()) {
                motorStop();
                delay(80);   // brief settle before driving
                exState = EX_FORWARD;
            }
            break;

        // ── Stopped / error ───────────────────────────────────────────────────
        case EX_STOP:
        default:
            motorStop();
            break;
    }
}
