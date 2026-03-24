/**
 * RPLidarObstacleAvoidanceNano.cpp
 *
 * ── Encoder maths ────────────────────────────────────────────────────────────
 *
 * Pololu 4751 motor (19:1 gearbox, 64 CPR encoder on motor shaft, 4× quadrature):
 *
 *   ticksPerRev = CPR × 4 × gearRatio = 64 × 4 × 19 = 4 864
 *
 *   distPerTick (mm) = (π × wheelDiameterMm) / ticksPerRev
 *
 *   ticksForDist  = distanceMm / distPerTick
 *
 *   ticksForTurn  = ((wheelBaseMm × π × angleDeg) / 360) / distPerTick
 *     → each wheel travels half the circumference of the rotation circle
 *       (robot pivots about its own centre, so radius = wheelBase / 2,
 *        but both wheels travel the same arc in opposite directions)
 *
 * ── RPLidar A1M8 standard scan protocol ─────────────────────────────────────
 *
 *  Tx: [0xA5][0x20]
 *  Rx: [0xA5][0x5A][4-byte size_q30_subtype][0x81]  (7-byte header)
 *      then continuous 5-byte nodes:
 *
 *   raw[0]  sync_quality   bit0=sync bit1=!sync bit2..7=quality
 *   raw[1]  angle_low      LSB of angle_q6_checkbit (uint16)
 *   raw[2]  angle_high     MSB – bit0=checkbit, bit1..15=angle_q6
 *   raw[3]  dist_low       LSB of distance_q2 (uint16)
 *   raw[4]  dist_high      MSB
 *
 *   angle_deg  = (uint16(raw[2], raw[1]) >> 1) / 64.0
 *   dist_mm    = uint16(raw[4], raw[3]) / 4.0
 *   sync       = raw[0] & 0x01  →  1 = first node of a new 360° sweep
 *   quality    = raw[0] >> 2
 *   checkbit   = raw[1] & 0x01  →  must be 1
 */

#include "RPLidarObstacleAvoidanceNano.h"

// ── Constructor ───────────────────────────────────────────────────────────────

RPLidarObstacleAvoidanceNano::RPLidarObstacleAvoidanceNano(
        HardwareSerial& serial, int lidarMotorPin, const Config& cfg)
    : _serial(serial)
    , _lidarMotorPin(lidarMotorPin)
    , _cfg(cfg)
    , _scanning(false)
    , _minQuality(RPLIDAR_MIN_QUALITY_N)
    , _distPerTick(0.0f)
    , _scanComplete(false)
    , _firstScanDone(false)
    , _scanCount(0)
    , _errorCount(0)
    , _lastDataMs(0)
    , _frontCenter(0.0f),   _frontHalf(30.0f)
    , _leftCenter(90.0f),   _leftHalf(30.0f)
    , _rightCenter(270.0f), _rightHalf(30.0f)
    , _backCenter(180.0f),  _backHalf(30.0f)
    , _maneuver(MAN_NONE)
    , _leftStart(0),  _rightStart(0)
    , _leftTarget(0), _rightTarget(0)
    , _maneuverDuty(0)
    , _leftStalled(false),  _rightStalled(false)
    , _leftTickLast(0),     _rightTickLast(0)
    , _leftStallTimer(0),   _rightStallTimer(0)
    , _ps(PS_SYNC1)
    , _hdrIdx(0)
    , _nodeIdx(0)
{
    memset(_scan, 0, sizeof(_scan));
    memset(_buf,  0, sizeof(_buf));
}

// ── begin() ───────────────────────────────────────────────────────────────────

bool RPLidarObstacleAvoidanceNano::begin(uint32_t baudrate)
{
    // ── 1. Compute encoder geometry ──────────────────────────────────────────
    _distPerTick = (PI * _cfg.wheelDiameterMm) / static_cast<float>(_cfg.ticksPerRev);

    // ── 2. Motor Carrier ─────────────────────────────────────────────────────
    if (!controller.begin()) {
        return false;  // I2C device not found
    }
    motorStop();  // ensure motors are off

    // ── 3. RPLidar motor pin ─────────────────────────────────────────────────
    if (_lidarMotorPin >= 0) {
        pinMode(_lidarMotorPin, OUTPUT);
        lidarMotorOff();
    }

    // ── 4. Serial port ───────────────────────────────────────────────────────
    _serial.begin(baudrate);
    delay(100);

    // ── 5. Reset lidar ───────────────────────────────────────────────────────
    sendCommand(RPLIDAR_CMD_STOP);
    delay(20);
    sendCommand(RPLIDAR_CMD_RESET);
    delay(2000);  // boot time

    while (_serial.available()) _serial.read();  // drain boot messages

    // ── 6. Spin up lidar motor and start scan ────────────────────────────────
    lidarMotorOn();
    delay(500);

    _ps           = PS_SYNC1;
    _hdrIdx       = 0;
    _nodeIdx      = 0;
    _scanComplete = false;
    _firstScanDone = false;
    _scanCount    = 0;
    _errorCount   = 0;
    _lastDataMs   = millis();

    memset(_scan, 0, sizeof(_scan));
    memset(_buf,  0, sizeof(_buf));

    sendCommand(RPLIDAR_CMD_SCAN);
    _scanning = true;
    return true;
}

// ── stop() ────────────────────────────────────────────────────────────────────

void RPLidarObstacleAvoidanceNano::stop()
{
    abortManeuver();
    motorStop();
    sendCommand(RPLIDAR_CMD_STOP);
    delay(20);
    lidarMotorOff();
    _scanning = false;
    _ps = PS_SYNC1;
}

// ── update() ─────────────────────────────────────────────────────────────────

int RPLidarObstacleAvoidanceNano::update()
{
    if (!_scanning) return 0;

    // 1. Process incoming lidar bytes (capped to avoid monopolising CPU)
    int avail = _serial.available();
    int limit = (avail < 256) ? avail : 256;
    for (int i = 0; i < limit; i++) {
        parseByte(static_cast<uint8_t>(_serial.read()));
    }

    // 2. Advance active manoeuvre
    if (_maneuver != MAN_NONE) updateManeuver();

    // 3. Stall detection (only meaningful when a motor is running)
    updateStall();

    return limit;
}

// ── isScanComplete() / isConnected() ─────────────────────────────────────────

bool RPLidarObstacleAvoidanceNano::isScanComplete()
{
    if (_scanComplete) { _scanComplete = false; return true; }
    return false;
}

bool RPLidarObstacleAvoidanceNano::isConnected() const
{
    return _scanning && (millis() - _lastDataMs < RPLIDAR_TIMEOUT_MS_N);
}

// ── Obstacle detection ────────────────────────────────────────────────────────

bool RPLidarObstacleAvoidanceNano::isObstacleInZone(
        float angleStart, float angleEnd, float maxDistMm) const
{
    angleStart = normalise(angleStart);
    angleEnd   = normalise(angleEnd);

    for (int i = 0; i < RPLIDAR_SCAN_BUCKETS_N; i++) {
        if (_scan[i].distanceMm == 0) continue;
        if (_scan[i].distanceMm > static_cast<uint16_t>(maxDistMm)) continue;

        float a = static_cast<float>(i);
        bool inside = (angleStart <= angleEnd)
                      ? (a >= angleStart && a <= angleEnd)
                      : (a >= angleStart || a <= angleEnd);
        if (inside) return true;
    }
    return false;
}

bool RPLidarObstacleAvoidanceNano::isObstacleFront(float d) const {
    return isObstacleInZone(normalise(_frontCenter-_frontHalf), normalise(_frontCenter+_frontHalf), d);
}
bool RPLidarObstacleAvoidanceNano::isObstacleLeft(float d) const {
    return isObstacleInZone(normalise(_leftCenter-_leftHalf), normalise(_leftCenter+_leftHalf), d);
}
bool RPLidarObstacleAvoidanceNano::isObstacleRight(float d) const {
    return isObstacleInZone(normalise(_rightCenter-_rightHalf), normalise(_rightCenter+_rightHalf), d);
}
bool RPLidarObstacleAvoidanceNano::isObstacleBack(float d) const {
    return isObstacleInZone(normalise(_backCenter-_backHalf), normalise(_backCenter+_backHalf), d);
}

float RPLidarObstacleAvoidanceNano::getMinDistance(float angleStart, float angleEnd) const
{
    angleStart = normalise(angleStart);
    angleEnd   = normalise(angleEnd);
    float minD = -1.0f;

    for (int i = 0; i < RPLIDAR_SCAN_BUCKETS_N; i++) {
        if (_scan[i].distanceMm == 0) continue;
        float a = static_cast<float>(i);
        bool inside = (angleStart <= angleEnd)
                      ? (a >= angleStart && a <= angleEnd)
                      : (a >= angleStart || a <= angleEnd);
        if (inside) {
            float d = static_cast<float>(_scan[i].distanceMm);
            if (minD < 0.0f || d < minD) minD = d;
        }
    }
    return minD;
}

float RPLidarObstacleAvoidanceNano::getDistanceAt(float angle) const
{
    int b = static_cast<int>(normalise(angle)) % RPLIDAR_SCAN_BUCKETS_N;
    return static_cast<float>(_scan[b].distanceMm);
}

// ── getRecommendedDirection() ─────────────────────────────────────────────────

ObstacleDirection RPLidarObstacleAvoidanceNano::getRecommendedDirection(float danger) const
{
    bool front = isObstacleFront(danger);
    bool left  = isObstacleLeft (danger);
    bool right = isObstacleRight(danger);
    bool back  = isObstacleBack (danger);

    if (!front && !left && !right && !back) return DIRECTION_CLEAR;
    if ( front &&  left &&  right &&  back) return DIRECTION_BLOCKED;
    if (!front)                             return DIRECTION_FORWARD;

    // Front blocked – pick the side with more clearance
    if (!left && !right) {
        float ld = getMinDistance(normalise(_leftCenter  - _leftHalf),  normalise(_leftCenter  + _leftHalf));
        float rd = getMinDistance(normalise(_rightCenter - _rightHalf), normalise(_rightCenter + _rightHalf));
        return (rd > 0.0f && (ld < 0.0f || rd > ld)) ? DIRECTION_RIGHT : DIRECTION_LEFT;
    }
    if (!left)  return DIRECTION_LEFT;
    if (!right) return DIRECTION_RIGHT;
    if (!back)  return DIRECTION_BACKWARD;

    return DIRECTION_BLOCKED;
}

// ── Encoder manoeuvres ────────────────────────────────────────────────────────

void RPLidarObstacleAvoidanceNano::startDriveForward(float distanceMm, uint8_t duty)
{
    encoder1.resetEncoder();
    encoder2.resetEncoder();
    _leftStart  = 0; _rightStart  = 0;
    _leftTarget = distToTicks(distanceMm);
    _rightTarget = _leftTarget;
    _maneuverDuty = (duty > 0) ? duty : _cfg.dutyForward;
    _maneuver = MAN_FORWARD;
    applyDuties(_maneuverDuty, _maneuverDuty);
}

void RPLidarObstacleAvoidanceNano::startDriveBackward(float distanceMm, uint8_t duty)
{
    encoder1.resetEncoder();
    encoder2.resetEncoder();
    _leftStart  = 0; _rightStart  = 0;
    _leftTarget = distToTicks(distanceMm);
    _rightTarget = _leftTarget;
    _maneuverDuty = (duty > 0) ? duty : _cfg.dutyBackward;
    _maneuver = MAN_BACKWARD;
    applyDuties(-static_cast<int>(_maneuverDuty), -static_cast<int>(_maneuverDuty));
}

void RPLidarObstacleAvoidanceNano::startTurnLeft(float angleDeg, uint8_t duty)
{
    encoder1.resetEncoder();
    encoder2.resetEncoder();
    _leftStart  = 0; _rightStart  = 0;
    _leftTarget = turnToTicks(angleDeg);
    _rightTarget = _leftTarget;
    _maneuverDuty = (duty > 0) ? duty : _cfg.dutyTurn;
    _maneuver = MAN_TURN_LEFT;
    // Left wheel backward, right wheel forward
    applyDuties(-static_cast<int>(_maneuverDuty), _maneuverDuty);
}

void RPLidarObstacleAvoidanceNano::startTurnRight(float angleDeg, uint8_t duty)
{
    encoder1.resetEncoder();
    encoder2.resetEncoder();
    _leftStart  = 0; _rightStart  = 0;
    _leftTarget = turnToTicks(angleDeg);
    _rightTarget = _leftTarget;
    _maneuverDuty = (duty > 0) ? duty : _cfg.dutyTurn;
    _maneuver = MAN_TURN_RIGHT;
    // Left wheel forward, right wheel backward
    applyDuties(_maneuverDuty, -static_cast<int>(_maneuverDuty));
}

void RPLidarObstacleAvoidanceNano::abortManeuver()
{
    _maneuver = MAN_NONE;
    motorStop();
}

// ── updateManeuver() – called from update() ───────────────────────────────────

void RPLidarObstacleAvoidanceNano::updateManeuver()
{
    // Read signed encoder positions and apply configured polarity
    int32_t leftNow  = encoder1.getPosition() * _cfg.leftEncoderDir;
    int32_t rightNow = encoder2.getPosition() * _cfg.rightEncoderDir;

    // Progress = absolute ticks travelled since manoeuvre start
    int32_t leftDone  = abs(leftNow  - _leftStart);
    int32_t rightDone = abs(rightNow - _rightStart);

    bool leftReached  = (leftDone  >= _leftTarget);
    bool rightReached = (rightDone >= _rightTarget);

    if (leftReached && rightReached) {
        // Both wheels at target → manoeuvre complete
        motorStop();
        _maneuver = MAN_NONE;
        return;
    }

    // ── Differential correction for straight drives ───────────────────────
    // If one wheel is significantly ahead of the other during a straight
    // drive, slow it down slightly so both arrive together.
    if (_maneuver == MAN_FORWARD || _maneuver == MAN_BACKWARD) {
        int32_t diff = leftDone - rightDone;           // >0 means left is ahead
        int correction = static_cast<int>(diff / 5);   // gentle correction
        int sign = (_maneuver == MAN_FORWARD) ? 1 : -1;

        int leftDuty  = sign * (static_cast<int>(_maneuverDuty) - (diff > 0 ?  correction : 0));
        int rightDuty = sign * (static_cast<int>(_maneuverDuty) - (diff < 0 ? -correction : 0));

        // Clamp individual duties (keep them positive to avoid reversals)
        leftDuty  = constrain(leftDuty,  sign * 20, sign * 100);
        rightDuty = constrain(rightDuty, sign * 20, sign * 100);

        if (leftReached)  leftDuty  = 0;
        if (rightReached) rightDuty = 0;

        applyDuties(leftDuty, rightDuty);

    } else {
        // Turn manoeuvre: no differential correction needed, just stop done wheels
        int d = static_cast<int>(_maneuverDuty);
        int leftDuty  = 0, rightDuty = 0;

        if (!leftReached) {
            leftDuty  = (_maneuver == MAN_TURN_LEFT) ? -d :  d;
        }
        if (!rightReached) {
            rightDuty = (_maneuver == MAN_TURN_LEFT) ?  d : -d;
        }
        applyDuties(leftDuty, rightDuty);
    }
}

// ── Direct motor commands ─────────────────────────────────────────────────────

void RPLidarObstacleAvoidanceNano::driveForward (uint8_t d) {
    _maneuver = MAN_NONE;
    applyDuties( d,  d);
}
void RPLidarObstacleAvoidanceNano::driveBackward(uint8_t d) {
    _maneuver = MAN_NONE;
    applyDuties(-static_cast<int>(d), -static_cast<int>(d));
}
void RPLidarObstacleAvoidanceNano::turnLeft (uint8_t d) {
    _maneuver = MAN_NONE;
    applyDuties(-static_cast<int>(d),  d);
}
void RPLidarObstacleAvoidanceNano::turnRight(uint8_t d) {
    _maneuver = MAN_NONE;
    applyDuties( d, -static_cast<int>(d));
}
void RPLidarObstacleAvoidanceNano::motorStop()
{
    M1.setDuty(0);
    M2.setDuty(0);
}

// ── Sector configuration ──────────────────────────────────────────────────────

void RPLidarObstacleAvoidanceNano::setFrontSector(float c, float w) { _frontCenter = normalise(c); _frontHalf = w * 0.5f; }
void RPLidarObstacleAvoidanceNano::setLeftSector (float c, float w) { _leftCenter  = normalise(c); _leftHalf  = w * 0.5f; }
void RPLidarObstacleAvoidanceNano::setRightSector(float c, float w) { _rightCenter = normalise(c); _rightHalf = w * 0.5f; }
void RPLidarObstacleAvoidanceNano::setBackSector (float c, float w) { _backCenter  = normalise(c); _backHalf  = w * 0.5f; }

// ── Stall detection ───────────────────────────────────────────────────────────

void RPLidarObstacleAvoidanceNano::updateStall()
{
    uint32_t now = millis();

    int32_t leftNow  = encoder1.getPosition() * _cfg.leftEncoderDir;
    int32_t rightNow = encoder2.getPosition() * _cfg.rightEncoderDir;

    // ── Left motor ────────────────────────────────────────────────────────────
    if (abs(leftNow - _leftTickLast) >= _cfg.stallMinTicks) {
        _leftStalled    = false;
        _leftStallTimer = now;
    } else if ((now - _leftStallTimer) > _cfg.stallWindowMs) {
        _leftStalled = true;  // no movement within window
    }
    _leftTickLast = leftNow;

    // ── Right motor ───────────────────────────────────────────────────────────
    if (abs(rightNow - _rightTickLast) >= _cfg.stallMinTicks) {
        _rightStalled    = false;
        _rightStallTimer = now;
    } else if ((now - _rightStallTimer) > _cfg.stallWindowMs) {
        _rightStalled = true;
    }
    _rightTickLast = rightNow;
}

// ── Protocol parser ───────────────────────────────────────────────────────────

void RPLidarObstacleAvoidanceNano::sendCommand(uint8_t cmd)
{
    uint8_t pkt[2] = { RPLIDAR_CMD_SYNC_BYTE, cmd };
    _serial.write(pkt, 2);
}

void RPLidarObstacleAvoidanceNano::parseByte(uint8_t b)
{
    _lastDataMs = millis();

    switch (_ps) {
        case PS_SYNC1:
            if (b == RPLIDAR_ANS_SYNC_BYTE1) _ps = PS_SYNC2;
            break;

        case PS_SYNC2:
            if      (b == RPLIDAR_ANS_SYNC_BYTE2) { _hdrIdx = 0; _ps = PS_HEADER; }
            else if (b == RPLIDAR_ANS_SYNC_BYTE1) { /* stay */ }
            else                                   _ps = PS_SYNC1;
            break;

        case PS_HEADER:
            _hdr[_hdrIdx++] = b;
            if (_hdrIdx >= RPLIDAR_ANS_HEADER_TAIL_N) {
                if (_hdr[4] == RPLIDAR_ANS_TYPE_MEASUREMENT) { _nodeIdx = 0; _ps = PS_SCANNING; }
                else { _errorCount++; _ps = PS_SYNC1; }
            }
            break;

        case PS_SCANNING:
            _node[_nodeIdx++] = b;
            if (_nodeIdx >= 5) { commitNode(_node); _nodeIdx = 0; }
            break;
    }
}

void RPLidarObstacleAvoidanceNano::commitNode(const uint8_t* raw)
{
    uint8_t  syncQuality = raw[0];
    uint16_t angleWord   = (static_cast<uint16_t>(raw[2]) << 8) | raw[1];
    uint16_t distWord    = (static_cast<uint16_t>(raw[4]) << 8) | raw[3];

    if (!(angleWord & 0x01u)) { _errorCount++; return; }

    bool    sync    = (syncQuality & 0x01u) != 0u;
    uint8_t quality = syncQuality >> 2u;
    float   angle   = static_cast<float>(angleWord >> 1u) / 64.0f;
    float   distF   = static_cast<float>(distWord) / 4.0f;

    // sync = true → first node of a new sweep → publish completed buffer
    if (sync && _firstScanDone) {
        memcpy(_scan, _buf, sizeof(_scan));
        memset(_buf,  0,    sizeof(_buf));
        _scanComplete = true;
        _scanCount++;
    }
    _firstScanDone = true;

    if (quality < _minQuality || distF <= 0.0f) return;

    // Saturate to uint16_t range (max ~65 m, well beyond A1M8's 12 m limit)
    uint16_t distMm = (distF > 65535.0f) ? 65535u : static_cast<uint16_t>(distF);

    int bucket = static_cast<int>(angle) % RPLIDAR_SCAN_BUCKETS_N;
    if (bucket < 0 || bucket >= RPLIDAR_SCAN_BUCKETS_N) return;

    if (_buf[bucket].quality < quality || _buf[bucket].distanceMm == 0) {
        _buf[bucket].distanceMm = distMm;
        _buf[bucket].quality    = quality;
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

int32_t RPLidarObstacleAvoidanceNano::distToTicks(float mm) const
{
    if (_distPerTick <= 0.0f) return 0;
    return static_cast<int32_t>(mm / _distPerTick + 0.5f);
}

int32_t RPLidarObstacleAvoidanceNano::turnToTicks(float angleDeg) const
{
    // Each wheel travels arc = (wheelBase/2) * angleRad
    //   = wheelBaseMm * PI * angleDeg / 360
    float arcMm = _cfg.wheelBaseMm * PI * angleDeg / 360.0f;
    return distToTicks(arcMm);
}

float RPLidarObstacleAvoidanceNano::normalise(float angle) const
{
    while (angle <    0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

void RPLidarObstacleAvoidanceNano::applyDuties(int leftDuty, int rightDuty)
{
    // Motor Carrier accepts -100..100
    leftDuty  = constrain(leftDuty,  -100, 100);
    rightDuty = constrain(rightDuty, -100, 100);
    M1.setDuty(leftDuty);
    M2.setDuty(rightDuty);
}

void RPLidarObstacleAvoidanceNano::lidarMotorOn()  { if (_lidarMotorPin >= 0) digitalWrite(_lidarMotorPin, HIGH); }
void RPLidarObstacleAvoidanceNano::lidarMotorOff() { if (_lidarMotorPin >= 0) digitalWrite(_lidarMotorPin, LOW);  }
