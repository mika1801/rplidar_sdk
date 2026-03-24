/**
 * RPLidarObstacleAvoidance.cpp
 *
 * Implementation of the RPLidar A1M8 obstacle-avoidance Arduino library.
 *
 * Protocol reference – standard scan mode (command 0x20):
 *
 *  Request packet:  [0xA5][0x20]
 *  Response header: [0xA5][0x5A][4 bytes size_q30_subtype][0x81]
 *  Then a continuous stream of 5-byte measurement nodes:
 *
 *  Byte 0  sync_quality   bit0=sync, bit1=!sync, bit2..7=quality(0-63)
 *  Byte 1  angle_low      LSB of angle_q6_checkbit
 *  Byte 2  angle_high     MSB of angle_q6_checkbit   (bit0=checkbit, bit1..15=angle_q6)
 *  Byte 3  dist_low       LSB of distance_q2
 *  Byte 4  dist_high      MSB of distance_q2
 *
 *  angle_degrees = (uint16(byte2, byte1) >> 1) / 64.0
 *  distance_mm   = uint16(byte4, byte3) / 4.0
 *  sync_bit      = byte0 & 0x01  → 1 signals the start of a new 360° sweep
 *  quality       = byte0 >> 2
 *  check_bit     = byte1 & 0x01  → must be 1, else discard node
 */

#include "RPLidarObstacleAvoidance.h"

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

RPLidarObstacleAvoidance::RPLidarObstacleAvoidance(HardwareSerial& serial,
                                                   int motorPin)
    : _serial(serial)
    , _motorPin(motorPin)
    , _scanComplete(false)
    , _firstScanDone(false)
    , _scanCount(0)
    , _errorCount(0)
    , _lastDataMs(0)
    , _scanning(false)
    , _minQuality(RPLIDAR_MIN_QUALITY)
    , _frontCenter(0.0f),   _frontHalf(30.0f)
    , _leftCenter(90.0f),   _leftHalf(30.0f)
    , _rightCenter(270.0f), _rightHalf(30.0f)
    , _backCenter(180.0f),  _backHalf(30.0f)
    , _state(STATE_WAIT_SYNC1)
    , _hdrIdx(0)
    , _nodeIdx(0)
{
    memset(_scan, 0, sizeof(_scan));
    memset(_buf,  0, sizeof(_buf));
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool RPLidarObstacleAvoidance::begin(uint32_t baudrate)
{
    _serial.begin(baudrate);

    if (_motorPin >= 0) {
        pinMode(_motorPin, OUTPUT);
        motorOff();
    }

    delay(100);

    // Graceful shutdown of any previous session
    sendCommand(RPLIDAR_CMD_STOP);
    delay(20);

    // Hardware reset – the lidar needs ~2 s to boot
    sendCommand(RPLIDAR_CMD_RESET);
    delay(2000);

    // Discard any boot messages
    while (_serial.available()) {
        _serial.read();
    }

    // Start the spinning motor
    motorOn();
    delay(500); // allow motor to reach speed

    // Reset parser and counters
    _state         = STATE_WAIT_SYNC1;
    _hdrIdx        = 0;
    _nodeIdx       = 0;
    _scanComplete  = false;
    _firstScanDone = false;
    _scanCount     = 0;
    _errorCount    = 0;
    _lastDataMs    = millis();

    memset(_scan, 0, sizeof(_scan));
    memset(_buf,  0, sizeof(_buf));

    // Issue the scan command
    sendCommand(RPLIDAR_CMD_SCAN);
    _scanning = true;

    return true;
}

void RPLidarObstacleAvoidance::stop()
{
    sendCommand(RPLIDAR_CMD_STOP);
    delay(20);
    motorOff();
    _scanning = false;
    _state    = STATE_WAIT_SYNC1;
}

// ---------------------------------------------------------------------------
// update() – call from loop() as often as possible
// ---------------------------------------------------------------------------

int RPLidarObstacleAvoidance::update()
{
    if (!_scanning) return 0;

    int available = _serial.available();
    if (available <= 0) return 0;

    // Cap to 512 bytes per call so we don't monopolise the CPU
    int limit = (available < 512) ? available : 512;

    for (int i = 0; i < limit; i++) {
        parseByte(static_cast<uint8_t>(_serial.read()));
    }

    return limit;
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

bool RPLidarObstacleAvoidance::isScanComplete()
{
    if (_scanComplete) {
        _scanComplete = false;
        return true;
    }
    return false;
}

bool RPLidarObstacleAvoidance::isConnected() const
{
    return _scanning && ((millis() - _lastDataMs) < RPLIDAR_TIMEOUT_MS);
}

// ---------------------------------------------------------------------------
// Obstacle detection
// ---------------------------------------------------------------------------

bool RPLidarObstacleAvoidance::isObstacleInZone(float angleStart,
                                                 float angleEnd,
                                                 float maxDistance) const
{
    angleStart = normalise(angleStart);
    angleEnd   = normalise(angleEnd);

    for (int i = 0; i < RPLIDAR_SCAN_BUCKETS; i++) {
        if (_scan[i].distance <= 0.0f) continue;
        if (_scan[i].distance > maxDistance) continue;

        float a = static_cast<float>(i);
        bool inside;
        if (angleStart <= angleEnd) {
            inside = (a >= angleStart && a <= angleEnd);
        } else {
            // Zone wraps around 0°/360°
            inside = (a >= angleStart || a <= angleEnd);
        }
        if (inside) return true;
    }
    return false;
}

bool RPLidarObstacleAvoidance::isObstacleFront(float maxDistance) const
{
    return isObstacleInZone(normalise(_frontCenter - _frontHalf),
                            normalise(_frontCenter + _frontHalf),
                            maxDistance);
}

bool RPLidarObstacleAvoidance::isObstacleLeft(float maxDistance) const
{
    return isObstacleInZone(normalise(_leftCenter - _leftHalf),
                            normalise(_leftCenter + _leftHalf),
                            maxDistance);
}

bool RPLidarObstacleAvoidance::isObstacleRight(float maxDistance) const
{
    return isObstacleInZone(normalise(_rightCenter - _rightHalf),
                            normalise(_rightCenter + _rightHalf),
                            maxDistance);
}

bool RPLidarObstacleAvoidance::isObstacleBack(float maxDistance) const
{
    return isObstacleInZone(normalise(_backCenter - _backHalf),
                            normalise(_backCenter + _backHalf),
                            maxDistance);
}

float RPLidarObstacleAvoidance::getMinDistance(float angleStart, float angleEnd) const
{
    angleStart = normalise(angleStart);
    angleEnd   = normalise(angleEnd);

    float minDist = -1.0f;

    for (int i = 0; i < RPLIDAR_SCAN_BUCKETS; i++) {
        if (_scan[i].distance <= 0.0f) continue;

        float a = static_cast<float>(i);
        bool inside;
        if (angleStart <= angleEnd) {
            inside = (a >= angleStart && a <= angleEnd);
        } else {
            inside = (a >= angleStart || a <= angleEnd);
        }

        if (inside) {
            if (minDist < 0.0f || _scan[i].distance < minDist) {
                minDist = _scan[i].distance;
            }
        }
    }
    return minDist;
}

float RPLidarObstacleAvoidance::getDistanceAt(float angle) const
{
    int bucket = static_cast<int>(normalise(angle)) % RPLIDAR_SCAN_BUCKETS;
    return _scan[bucket].distance;
}

// ---------------------------------------------------------------------------
// High-level avoidance decision
// ---------------------------------------------------------------------------

ObstacleDirection RPLidarObstacleAvoidance::getRecommendedDirection(float dangerDistance) const
{
    bool front = isObstacleFront(dangerDistance);
    bool left  = isObstacleLeft (dangerDistance);
    bool right = isObstacleRight(dangerDistance);
    bool back  = isObstacleBack (dangerDistance);

    // Nothing in the way
    if (!front && !left && !right && !back) return DIRECTION_CLEAR;

    // Completely surrounded
    if (front && left && right && back) return DIRECTION_BLOCKED;

    // Forward is clear – best option
    if (!front) return DIRECTION_FORWARD;

    // Front blocked – choose side with more clearance
    if (!left && !right) {
        float ld = getMinDistance(normalise(_leftCenter  - _leftHalf),
                                  normalise(_leftCenter  + _leftHalf));
        float rd = getMinDistance(normalise(_rightCenter - _rightHalf),
                                  normalise(_rightCenter + _rightHalf));
        // Prefer the side with more space (larger min-distance)
        return (rd > 0.0f && (ld < 0.0f || rd > ld)) ? DIRECTION_RIGHT : DIRECTION_LEFT;
    }

    if (!left)  return DIRECTION_LEFT;
    if (!right) return DIRECTION_RIGHT;
    if (!back)  return DIRECTION_BACKWARD;

    return DIRECTION_BLOCKED;
}

// ---------------------------------------------------------------------------
// Sector configuration
// ---------------------------------------------------------------------------

void RPLidarObstacleAvoidance::setFrontSector(float center, float width)
{
    _frontCenter = normalise(center);
    _frontHalf   = width * 0.5f;
}

void RPLidarObstacleAvoidance::setLeftSector(float center, float width)
{
    _leftCenter = normalise(center);
    _leftHalf   = width * 0.5f;
}

void RPLidarObstacleAvoidance::setRightSector(float center, float width)
{
    _rightCenter = normalise(center);
    _rightHalf   = width * 0.5f;
}

void RPLidarObstacleAvoidance::setBackSector(float center, float width)
{
    _backCenter = normalise(center);
    _backHalf   = width * 0.5f;
}

// ---------------------------------------------------------------------------
// Private – protocol helpers
// ---------------------------------------------------------------------------

void RPLidarObstacleAvoidance::sendCommand(uint8_t cmd)
{
    uint8_t pkt[2] = { RPLIDAR_CMD_SYNC_BYTE, cmd };
    _serial.write(pkt, 2);
}

// State machine – one byte at a time
void RPLidarObstacleAvoidance::parseByte(uint8_t b)
{
    _lastDataMs = millis();

    switch (_state) {

        case STATE_WAIT_SYNC1:
            if (b == RPLIDAR_ANS_SYNC_BYTE1) {
                _state = STATE_WAIT_SYNC2;
            }
            break;

        case STATE_WAIT_SYNC2:
            if (b == RPLIDAR_ANS_SYNC_BYTE2) {
                _hdrIdx = 0;
                _state  = STATE_HEADER;
            } else if (b == RPLIDAR_ANS_SYNC_BYTE1) {
                // stay in SYNC2 – could be first byte of next header
            } else {
                _state = STATE_WAIT_SYNC1;
            }
            break;

        case STATE_HEADER:
            _hdr[_hdrIdx++] = b;
            if (_hdrIdx >= RPLIDAR_ANS_HEADER_TAIL) {
                // _hdr[4] is the response type byte
                if (_hdr[4] == RPLIDAR_ANS_TYPE_MEASUREMENT) {
                    _nodeIdx = 0;
                    _state   = STATE_SCANNING;
                } else {
                    _errorCount++;
                    _state = STATE_WAIT_SYNC1;
                }
            }
            break;

        case STATE_SCANNING:
            _node[_nodeIdx++] = b;
            if (_nodeIdx >= 5) {
                commitNode(_node);
                _nodeIdx = 0;
            }
            break;
    }
}

// Decode a complete 5-byte node and store it in the accumulation buffer
void RPLidarObstacleAvoidance::commitNode(const uint8_t* raw)
{
    uint8_t  syncQuality = raw[0];
    uint16_t angleWord   = (static_cast<uint16_t>(raw[2]) << 8) | raw[1];
    uint16_t distWord    = (static_cast<uint16_t>(raw[4]) << 8) | raw[3];

    // The check bit (LSB of angleWord) must be 1
    if (!(angleWord & 0x01u)) {
        _errorCount++;
        return;
    }

    bool    syncBit  = (syncQuality & 0x01u) != 0u;
    uint8_t quality  = syncQuality >> 2u;
    float   angleDeg = static_cast<float>(angleWord >> 1u) / 64.0f;
    float   distMm   = static_cast<float>(distWord) / 4.0f;

    // sync bit = true  →  this node is the first point of a new 360° rotation.
    // Publish the completed buffer and reset for the new sweep.
    if (syncBit && _firstScanDone) {
        memcpy(_scan, _buf, sizeof(_scan));
        memset(_buf, 0, sizeof(_buf));
        _scanComplete = true;
        _scanCount++;
    }
    _firstScanDone = true;

    // Filter: discard low-quality or zero-distance readings
    if (quality < _minQuality || distMm <= 0.0f) return;

    // Map angle to a 1°-wide bucket, keeping best quality per bucket
    int bucket = static_cast<int>(angleDeg) % RPLIDAR_SCAN_BUCKETS;
    if (bucket < 0 || bucket >= RPLIDAR_SCAN_BUCKETS) return;

    if (_buf[bucket].quality < quality || _buf[bucket].distance == 0.0f) {
        _buf[bucket].distance = distMm;
        _buf[bucket].quality  = quality;
    }
}

// ---------------------------------------------------------------------------
// Private – utilities
// ---------------------------------------------------------------------------

float RPLidarObstacleAvoidance::normalise(float angle) const
{
    while (angle <    0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

bool RPLidarObstacleAvoidance::inZone(int bucket, float center, float half) const
{
    float a     = static_cast<float>(bucket);
    float start = normalise(center - half);
    float end   = normalise(center + half);

    if (start <= end) {
        return a >= start && a <= end;
    }
    return a >= start || a <= end;
}

void RPLidarObstacleAvoidance::motorOn()
{
    if (_motorPin >= 0) digitalWrite(_motorPin, HIGH);
}

void RPLidarObstacleAvoidance::motorOff()
{
    if (_motorPin >= 0) digitalWrite(_motorPin, LOW);
}
