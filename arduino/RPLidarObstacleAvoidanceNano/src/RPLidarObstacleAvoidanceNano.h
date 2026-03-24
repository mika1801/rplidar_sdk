/**
 * RPLidarObstacleAvoidanceNano.h
 *
 * Arduino library for the RPLidar A1M8 + Arduino Nano Motor Carrier on the
 * Arduino Nano 33 IoT.  Compared to the Giga R1 variant, this library adds
 * encoder-based closed-loop manoeuvres using the quadrature encoders on the
 * Pololu 4751 gearmotors (19:1, 64 CPR, 12 V, 530 RPM).
 *
 * Motor specs for Pololu 4751
 * ───────────────────────────
 *   Gear ratio  : 19:1
 *   Encoder CPR : 64 (cycles per revolution of motor shaft)
 *   Quadrature  : 4× decoding → 256 pulses/motor-rev
 *   Output shaft: 256 × 19 = 4 864 ticks/wheel-revolution
 *
 * Wiring
 * ──────
 *   RPLidar TX   →  Serial1 RX  (Nano 33 IoT pin 0)
 *   RPLidar RX   →  Serial1 TX  (Nano 33 IoT pin 1)
 *   RPLidar 5 V  →  5 V
 *   RPLidar GND  →  GND
 *   MOTO_CTRL    →  any free GPIO, e.g. D7  (HIGH = spinning)
 *
 *   Left  motor  →  Carrier M1   (encoder wires → ENC_A1 / ENC_B1)
 *   Right motor  →  Carrier M2   (encoder wires → ENC_A2 / ENC_B2)
 *   Motor 12 V supply connected to carrier VIN / GND
 *
 * Angle convention (configurable):
 *   0° = forward, 90° = left, 180° = back, 270° = right
 *
 * Encoder-based manoeuvres (non-blocking, driven by update()):
 *   startDriveForward(200)   // drive exactly 200 mm forward
 *   startTurnLeft(90)        // turn exactly 90° left in place
 *   startDriveBackward(150)  // back up 150 mm
 *   while (!lidar.isManeuverComplete()) lidar.update();
 *
 * Requires: Arduino_MotorCarrier library
 *   (Sketch → Include Library → Manage Libraries → "Arduino_MotorCarrier")
 */

#pragma once

#include <Arduino.h>
#include <Arduino_MotorCarrier.h>
#include <stdint.h>
#include <string.h>

// ── Pololu 4751 encoder defaults ─────────────────────────────────────────────
// 64 CPR × 4 (quadrature) × 19 (gear ratio) = 4 864 ticks per output revolution
#define NANO_TICKS_PER_REV_DEFAULT   4864
#define NANO_WHEEL_DIAM_MM_DEFAULT   65.0f   // set to your actual wheel diameter
#define NANO_WHEELBASE_MM_DEFAULT   150.0f   // set to wheel-centre to wheel-centre

// ── RPLidar A1M8 protocol constants ──────────────────────────────────────────
#define RPLIDAR_CMD_SYNC_BYTE        0xA5u
#define RPLIDAR_ANS_SYNC_BYTE1       0xA5u
#define RPLIDAR_ANS_SYNC_BYTE2       0x5Au
#define RPLIDAR_CMD_STOP             0x25u
#define RPLIDAR_CMD_SCAN             0x20u
#define RPLIDAR_CMD_RESET            0x40u
#define RPLIDAR_ANS_TYPE_MEASUREMENT 0x81u
#define RPLIDAR_ANS_HEADER_TAIL_N    5       // bytes after the two sync bytes
#define RPLIDAR_DEFAULT_BAUD_N       115200u
#define RPLIDAR_SCAN_BUCKETS_N       360     // 1 bucket per degree
#define RPLIDAR_MIN_QUALITY_N        10
#define RPLIDAR_TIMEOUT_MS_N         5000UL

// ── Public types ──────────────────────────────────────────────────────────────

/** Recommended robot action. */
enum ObstacleDirection {
    DIRECTION_CLEAR    = 0,
    DIRECTION_FORWARD  = 1,
    DIRECTION_LEFT     = 2,
    DIRECTION_RIGHT    = 3,
    DIRECTION_BACKWARD = 4,
    DIRECTION_BLOCKED  = 5,
};

/** One degree of scan data (uint16_t to keep RAM footprint small on Nano). */
struct ScanPoint {
    uint16_t distanceMm; ///< 0 = no valid reading
    uint8_t  quality;    ///< 0-63
} __attribute__((packed));

// ── Main class ────────────────────────────────────────────────────────────────

class RPLidarObstacleAvoidanceNano {
public:

    /**
     * Robot geometry & tuning parameters.
     * Adjust these to match your physical robot before calling begin().
     */
    struct Config {
        // ── Geometry ─────────────────────────────────────────────────────────
        float   wheelDiameterMm = NANO_WHEEL_DIAM_MM_DEFAULT;
        float   wheelBaseMm     = NANO_WHEELBASE_MM_DEFAULT;
        int32_t ticksPerRev     = NANO_TICKS_PER_REV_DEFAULT;

        // ── Navigation speeds (Motor Carrier duty, 0-100 %) ───────────────
        uint8_t dutyForward  = 60;
        uint8_t dutyTurn     = 50;
        uint8_t dutyBackward = 45;

        // ── Stall detection ───────────────────────────────────────────────
        // A motor is declared stalled if its encoder does not advance by at
        // least stallMinTicks within stallWindowMs while receiving a command.
        uint32_t stallWindowMs  = 300;
        int32_t  stallMinTicks  = 5;

        // ── Encoder polarity ──────────────────────────────────────────────
        // Set to -1 if a motor's encoder counts backwards for forward motion.
        int8_t leftEncoderDir  = 1;   // +1 or -1
        int8_t rightEncoderDir = 1;
    };

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /**
     * @param serial        HardwareSerial connected to RPLidar (Serial1)
     * @param lidarMotorPin GPIO pin for RPLidar MOTO_CTRL (-1 = not used)
     * @param cfg           Robot geometry and tuning
     */
    explicit RPLidarObstacleAvoidanceNano(HardwareSerial& serial,
                                          int lidarMotorPin = -1,
                                          const Config& cfg = Config());

    /**
     * Initialise Motor Carrier, reset RPLidar, start scan.
     * Returns false if the Motor Carrier cannot be found on I2C.
     */
    bool begin(uint32_t baudrate = RPLIDAR_DEFAULT_BAUD_N);

    /** Stop scanning, spin down RPLidar motor, coast robot motors. */
    void stop();

    /**
     * Process incoming RPLidar bytes, advance encoder manoeuvre, update
     * stall detection.  Call as often as possible inside loop().
     * @return Number of serial bytes consumed.
     */
    int update();

    // ── Scan status ───────────────────────────────────────────────────────────

    /** Returns true (once) when a fresh complete 360° scan has arrived. */
    bool isScanComplete();

    /** True while scanning and fresh data arrived within RPLIDAR_TIMEOUT_MS_N ms. */
    bool isConnected() const;

    // ── Obstacle detection ────────────────────────────────────────────────────

    /**
     * True if any valid reading inside [angleStart, angleEnd] is closer than
     * maxDistMm.  Zones that wrap across 0°/360° are handled automatically.
     */
    bool isObstacleInZone(float angleStart, float angleEnd, float maxDistMm) const;

    bool isObstacleFront(float maxDistMm) const;
    bool isObstacleLeft (float maxDistMm) const;
    bool isObstacleRight(float maxDistMm) const;
    bool isObstacleBack (float maxDistMm) const;

    /**
     * Minimum distance (mm) in the given angular range.
     * Returns -1 if no valid readings in that zone.
     */
    float getMinDistance(float angleStart, float angleEnd) const;

    /** Distance at the nearest 1° bucket. 0 = no valid reading. */
    float getDistanceAt(float angle) const;

    // ── High-level avoidance decision ─────────────────────────────────────────

    /**
     * Evaluates front/left/right/back sectors and returns the recommended
     * action.  Uses the obstacle distances from the latest completed scan.
     * @param dangerDistMm  Obstacle threshold in mm (default 500 mm)
     */
    ObstacleDirection getRecommendedDirection(float dangerDistMm = 500.0f) const;

    // ── Encoder-based precise manoeuvres (non-blocking) ───────────────────────

    /**
     * Start a manoeuvre.  Returns immediately; call update() in loop() until
     * isManeuverComplete() returns true.
     *
     * @param distanceMm / angleDeg  Target distance or rotation
     * @param duty                   0 = use Config default
     */
    void startDriveForward (float distanceMm, uint8_t duty = 0);
    void startDriveBackward(float distanceMm, uint8_t duty = 0);
    void startTurnLeft     (float angleDeg,   uint8_t duty = 0);
    void startTurnRight    (float angleDeg,   uint8_t duty = 0);

    /** True when no encoder manoeuvre is running. */
    bool isManeuverComplete() const { return _maneuver == MAN_NONE; }

    /** Immediately stop the active manoeuvre and coast to a halt. */
    void abortManeuver();

    // ── Direct (open-loop) motor commands ─────────────────────────────────────

    void driveForward (uint8_t duty);
    void driveBackward(uint8_t duty);
    void turnLeft     (uint8_t duty);
    void turnRight    (uint8_t duty);
    void motorStop();

    // ── Encoder diagnostics ───────────────────────────────────────────────────

    float   getLeftRPM()     const { return encoder1.getRPM(); }
    float   getRightRPM()    const { return encoder2.getRPM(); }
    int32_t getLeftTicks()   const { return encoder1.getPosition() * _cfg.leftEncoderDir;  }
    int32_t getRightTicks()  const { return encoder2.getPosition() * _cfg.rightEncoderDir; }

    bool isLeftMotorStalled () const { return _leftStalled;  }
    bool isRightMotorStalled() const { return _rightStalled; }

    // ── Sector configuration ──────────────────────────────────────────────────

    void setFrontSector(float center, float width = 60.0f);
    void setLeftSector (float center, float width = 60.0f);
    void setRightSector(float center, float width = 60.0f);
    void setBackSector (float center, float width = 60.0f);

    // ── Raw data & statistics ─────────────────────────────────────────────────

    const ScanPoint* getScanData()   const { return _scan; }
    void     setMinQuality(uint8_t q)      { _minQuality = q; }
    uint32_t getScanCount()          const { return _scanCount;  }
    uint32_t getErrorCount()         const { return _errorCount; }
    Config&  config()                      { return _cfg; }

private:
    // ── Hardware ──────────────────────────────────────────────────────────────
    HardwareSerial& _serial;
    int             _lidarMotorPin;
    Config          _cfg;
    bool            _scanning;
    uint8_t         _minQuality;
    float           _distPerTick; ///< mm per encoder tick (computed in begin())

    // ── Scan double-buffer ────────────────────────────────────────────────────
    ScanPoint _scan[RPLIDAR_SCAN_BUCKETS_N]; // stable copy for callers
    ScanPoint _buf [RPLIDAR_SCAN_BUCKETS_N]; // accumulation buffer
    bool      _scanComplete;
    bool      _firstScanDone;
    uint32_t  _scanCount;
    uint32_t  _errorCount;
    uint32_t  _lastDataMs;

    // ── Direction sectors (center + half-width) ───────────────────────────────
    float _frontCenter, _frontHalf;
    float _leftCenter,  _leftHalf;
    float _rightCenter, _rightHalf;
    float _backCenter,  _backHalf;

    // ── Encoder manoeuvre state machine ───────────────────────────────────────
    enum ManeuverType : uint8_t {
        MAN_NONE,
        MAN_FORWARD,
        MAN_BACKWARD,
        MAN_TURN_LEFT,
        MAN_TURN_RIGHT,
    };
    ManeuverType _maneuver;
    int32_t      _leftStart,  _rightStart;   // encoder snapshot at manoeuvre start
    int32_t      _leftTarget, _rightTarget;  // |ticks| each wheel must travel
    uint8_t      _maneuverDuty;

    // ── Stall detection ───────────────────────────────────────────────────────
    bool     _leftStalled,  _rightStalled;
    int32_t  _leftTickLast, _rightTickLast;
    uint32_t _leftStallTimer, _rightStallTimer;

    // ── Protocol parser ───────────────────────────────────────────────────────
    enum ParserState : uint8_t {
        PS_SYNC1, PS_SYNC2, PS_HEADER, PS_SCANNING
    };
    ParserState _ps;
    uint8_t     _hdr[RPLIDAR_ANS_HEADER_TAIL_N];
    int         _hdrIdx;
    uint8_t     _node[5];
    int         _nodeIdx;

    // ── Private helpers ───────────────────────────────────────────────────────
    void    sendCommand(uint8_t cmd);
    void    parseByte(uint8_t b);
    void    commitNode(const uint8_t* raw);
    void    updateManeuver();
    void    updateStall();
    int32_t distToTicks(float mm)       const;
    int32_t turnToTicks(float angleDeg) const;
    float   normalise(float angle)      const;
    void    lidarMotorOn();
    void    lidarMotorOff();

    // Apply signed duty (-100..100) directly to Motor Carrier channels
    void applyDuties(int leftDuty, int rightDuty);
};
