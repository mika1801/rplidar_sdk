/**
 * RPLidarObstacleAvoidance.h
 *
 * Arduino library for the RPLidar A1M8 with obstacle avoidance functionality.
 * Designed for the Arduino Giga R1 WiFi (STM32H747) but compatible with any
 * Arduino board that provides a HardwareSerial port.
 *
 * Wiring:
 *   RPLidar TX  -> Arduino RX pin (e.g. Serial1 RX on Giga R1: pin 0)
 *   RPLidar RX  -> Arduino TX pin (e.g. Serial1 TX on Giga R1: pin 1)
 *   RPLidar VCC -> 5V
 *   RPLidar GND -> GND
 *   RPLidar MOTO_CTRL -> motorPin (optional, HIGH = motor on)
 *
 * Angle convention (configurable):
 *   0°   = forward
 *   90°  = left
 *   180° = backward
 *   270° = right
 *
 * Usage:
 *   RPLidarObstacleAvoidance lidar(Serial1, MOTOR_PIN);
 *   lidar.begin();
 *   // In loop():
 *   lidar.update();
 *   if (lidar.isObstacleFront(500)) { ... }
 *   ObstacleDirection dir = lidar.getRecommendedDirection(500);
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

// ---- Library defaults -------------------------------------------------------

#define RPLIDAR_DEFAULT_BAUDRATE     115200u
#define RPLIDAR_SCAN_BUCKETS         360       // one bucket per degree
#define RPLIDAR_MIN_QUALITY          10        // discard readings below this
#define RPLIDAR_TIMEOUT_MS           5000UL    // declare disconnected after this many ms without data

// ---- Protocol constants (RPLidar serial protocol) ---------------------------

#define RPLIDAR_CMD_SYNC_BYTE        0xA5u
#define RPLIDAR_ANS_SYNC_BYTE1       0xA5u
#define RPLIDAR_ANS_SYNC_BYTE2       0x5Au
#define RPLIDAR_CMD_STOP             0x25u
#define RPLIDAR_CMD_SCAN             0x20u
#define RPLIDAR_CMD_RESET            0x40u
#define RPLIDAR_ANS_TYPE_MEASUREMENT 0x81u
// Response header layout:  [A5][5A][4-byte size_q30_subtype][type]  = 7 bytes total
// After the two sync bytes we read 5 more bytes (indices 0-4), type is at index 4.
#define RPLIDAR_ANS_HEADER_TAIL      5

// ---- Public types -----------------------------------------------------------

/**
 * Recommended robot action returned by getRecommendedDirection().
 */
enum ObstacleDirection {
    DIRECTION_CLEAR    = 0,  ///< No obstacles within danger distance
    DIRECTION_FORWARD  = 1,  ///< Forward is clear  – continue forward
    DIRECTION_LEFT     = 2,  ///< Turn left
    DIRECTION_RIGHT    = 3,  ///< Turn right
    DIRECTION_BACKWARD = 4,  ///< Back up
    DIRECTION_BLOCKED  = 5,  ///< Surrounded – stop
};

/** A single distance measurement for one degree bucket. */
struct ScanPoint {
    float   distance; ///< Distance in mm, 0 = no valid reading
    uint8_t quality;  ///< Measurement quality 0-63
};

// ---- Main class -------------------------------------------------------------

class RPLidarObstacleAvoidance {
public:
    /**
     * @param serial   Hardware serial port connected to the RPLidar
     *                 (e.g. Serial1 on Arduino Giga R1 WiFi)
     * @param motorPin GPIO pin wired to MOTO_CTRL on the RPLidar.
     *                 HIGH enables motor, LOW disables.
     *                 Pass -1 if motor control is handled externally.
     */
    explicit RPLidarObstacleAvoidance(HardwareSerial& serial, int motorPin = -1);

    // ---- Lifecycle ----------------------------------------------------------

    /**
     * Initialise serial port, reset the lidar, start the motor, and begin
     * streaming scan data.
     * @param baudrate Serial baud rate (default 115200 for A1M8)
     * @return true always (hardware errors surface through isConnected())
     */
    bool begin(uint32_t baudrate = RPLIDAR_DEFAULT_BAUDRATE);

    /**
     * Stop scanning and spin down the motor.
     * Call begin() again to restart.
     */
    void stop();

    /**
     * Process incoming serial bytes. Call this as frequently as possible
     * inside loop() to avoid RX buffer overflow.
     * @return Number of bytes consumed this call.
     */
    int update();

    // ---- Scan status --------------------------------------------------------

    /**
     * Returns true (once) when a fresh, complete 360° scan has arrived since
     * the last call.  Clears the flag automatically.
     */
    bool isScanComplete();

    /**
     * True while scanning is active and fresh data has arrived within
     * RPLIDAR_TIMEOUT_MS milliseconds.
     */
    bool isConnected() const;

    // ---- Obstacle detection -------------------------------------------------

    /**
     * Check whether any valid reading within [angleStart, angleEnd] is closer
     * than maxDistance (mm).
     *
     * Angles wrap naturally: angleStart=350, angleEnd=10 covers 350°-360°+0°-10°.
     */
    bool isObstacleInZone(float angleStart, float angleEnd, float maxDistance) const;

    /** Convenience wrappers using the configured directional sectors. */
    bool isObstacleFront(float maxDistance) const;
    bool isObstacleLeft (float maxDistance) const;
    bool isObstacleRight(float maxDistance) const;
    bool isObstacleBack (float maxDistance) const;

    /**
     * Minimum distance (mm) of any valid reading in [angleStart, angleEnd].
     * Returns -1 if no valid readings exist in that zone.
     */
    float getMinDistance(float angleStart, float angleEnd) const;

    /**
     * Distance (mm) at the given angle (nearest 1° bucket).
     * Returns 0 if no valid reading at that angle.
     */
    float getDistanceAt(float angle) const;

    /**
     * High-level obstacle avoidance decision.
     *
     * Evaluates the front, left, right, and back sectors and returns the
     * recommended robot action.
     *
     * @param dangerDistance Obstacle threshold in mm (default 500 mm = 50 cm)
     */
    ObstacleDirection getRecommendedDirection(float dangerDistance = 500.0f) const;

    // ---- Sector configuration -----------------------------------------------

    /**
     * Configure the angular sector treated as "forward".
     * @param center    Center angle in degrees (default 0)
     * @param width     Total sector width in degrees (default 60)
     */
    void setFrontSector(float center, float width = 60.0f);
    void setLeftSector  (float center, float width = 60.0f);
    void setRightSector (float center, float width = 60.0f);
    void setBackSector  (float center, float width = 60.0f);

    // ---- Raw data access ----------------------------------------------------

    /** Pointer to the current 360-element scan array (one ScanPoint per degree). */
    const ScanPoint* getScanData() const { return _scan; }

    // ---- Quality / diagnostics ----------------------------------------------

    /** Set minimum quality threshold for accepted readings (0-63, default 10). */
    void setMinQuality(uint8_t q) { _minQuality = q; }

    /** Total number of completed 360° scans since begin(). */
    uint32_t getScanCount()  const { return _scanCount; }

    /** Total number of discarded/malformed nodes since begin(). */
    uint32_t getErrorCount() const { return _errorCount; }

private:
    // ---- Hardware -----------------------------------------------------------
    HardwareSerial& _serial;
    int             _motorPin;

    // ---- Scan storage -------------------------------------------------------
    ScanPoint _scan  [RPLIDAR_SCAN_BUCKETS]; // stable copy for callers
    ScanPoint _buf   [RPLIDAR_SCAN_BUCKETS]; // accumulation buffer for current scan
    bool      _scanComplete;
    bool      _firstScanDone;
    uint32_t  _scanCount;
    uint32_t  _errorCount;
    uint32_t  _lastDataMs;

    // ---- Settings -----------------------------------------------------------
    bool    _scanning;
    uint8_t _minQuality;

    // Sector definitions stored as (center, halfWidth) pairs
    float _frontCenter, _frontHalf;
    float _leftCenter,  _leftHalf;
    float _rightCenter, _rightHalf;
    float _backCenter,  _backHalf;

    // ---- Parser state machine -----------------------------------------------
    enum ParserState : uint8_t {
        STATE_WAIT_SYNC1,  // looking for 0xA5
        STATE_WAIT_SYNC2,  // looking for 0x5A
        STATE_HEADER,      // consuming 5 header tail bytes
        STATE_SCANNING,    // consuming 5-byte measurement nodes
    };

    ParserState _state;
    uint8_t     _hdr[RPLIDAR_ANS_HEADER_TAIL];
    int         _hdrIdx;
    uint8_t     _node[5];
    int         _nodeIdx;

    // ---- Private helpers ----------------------------------------------------
    void  sendCommand(uint8_t cmd);
    void  parseByte(uint8_t b);
    void  commitNode(const uint8_t* raw);
    float normalise(float angle) const;
    bool  inZone(int bucket, float center, float half) const;
    void  motorOn();
    void  motorOff();
};
