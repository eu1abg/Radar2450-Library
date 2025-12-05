#include "LD2450radar.h"

LD2450radar::LD2450radar(HardwareSerial &serial, int rxPin, int txPin, unsigned long baud,
                         int maxTargets, float alpha, float maxDistM)
: _serial(serial), _rxPin(rxPin), _txPin(txPin), _baud(baud),
  _maxTargets(maxTargets), _alpha(alpha), _maxDistM(maxDistM)
{
    _targets = new Target[_maxTargets];
    for (int i=0;i<_maxTargets;i++) {
        _targets[i].id = -1;
        _targets[i].lostCounter = 100;
        _targets[i].valid = false;
    }
}

bool LD2450radar::begin(bool configureMultiTarget) {
    // init serial used for radar
    _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    delay(50);
    _radar.begin(_serial, false);
    if (configureMultiTarget) {
        _radar.beginConfigurationSession();
        _radar.setMultiTargetTracking();
        _radar.endConfigurationSession();
    }
    delay(150);
    reset();
    return true;
}

int LD2450radar::update() {
    if (!_radar.update()) return 0;
    int nr = _radar.getNrValidTargets();

    // increment lost counters
    for (int i=0;i<_maxTargets;i++) _targets[i].lostCounter++;

    for (int i=0;i<nr;i++) {
        auto t = _radar.getTarget(i);
        if (!t.valid) continue;

        float x_m = float(t.x) / 1000.0f;
        float y_m = float(t.y) / 1000.0f;
        float dist_m = sqrtf(x_m*x_m + y_m*y_m);
        float angle = atan2f(x_m, y_m) * 180.0f / M_PI;

        // clamp
        if (dist_m < 0.0f) dist_m = 0.0f;
        if (dist_m > _maxDistM) dist_m = _maxDistM;
        if (angle < -180.0f) angle = -180.0f;
        if (angle > 180.0f) angle = 180.0f;

        int slot = -1;
        // find by id
        for (int s=0;s<_maxTargets;s++) {
            if (_targets[s].id == t.id) { slot = s; break; }
        }
        // else find free or expired
        if (slot == -1) {
            for (int s=0;s<_maxTargets;s++) {
                if (_targets[s].id == -1 || _targets[s].lostCounter > 10) { slot = s; break; }
            }
        }

        if (slot != -1) {
            float oldDist = _targets[slot].distance;

            _targets[slot].id = t.id;
            _targets[slot].x = _alpha * x_m + (1.0f - _alpha) * _targets[slot].x;
            _targets[slot].y = _alpha * y_m + (1.0f - _alpha) * _targets[slot].y;
            _targets[slot].distance = sqrtf(_targets[slot].x*_targets[slot].x + _targets[slot].y*_targets[slot].y);
            _targets[slot].angle = atan2f(_targets[slot].x, _targets[slot].y) * 180.0f / M_PI;
            _targets[slot].lostCounter = 0;
            _targets[slot].valid = true;

            // speed estimate (assumes ~50 ms between updates)
            if (oldDist > 0.0f) {
                _targets[slot].speed = (_targets[slot].distance - oldDist) / 0.05f;
            } else {
                _targets[slot].speed = 0.0f;
            }
        }
    }

    return nr;
}

int LD2450radar::getActiveCount() const {
    int c=0;
    for (int i=0;i<_maxTargets;i++) if (_targets[i].lostCounter==0 && _targets[i].valid) c++;
    return c;
}

int LD2450radar::getID(int n) const {
    int cnt=0;
    for (int i=0;i<_maxTargets;i++) {
        if (_targets[i].lostCounter==0 && _targets[i].valid) {
            cnt++;
            if (cnt==n) return _targets[i].id;
        }
    }
    return -1;
}

float LD2450radar::getAngle(int n) const {
    int cnt=0;
    for (int i=0;i<_maxTargets;i++) {
        if (_targets[i].lostCounter==0 && _targets[i].valid) {
            cnt++;
            if (cnt==n) return _targets[i].angle;
        }
    }
    return 0.0f;
}

float LD2450radar::getDistance(int n) const {
    int cnt=0;
    for (int i=0;i<_maxTargets;i++) {
        if (_targets[i].lostCounter==0 && _targets[i].valid) {
            cnt++;
            if (cnt==n) return _targets[i].distance;
        }
    }
    return 0.0f;
}

float LD2450radar::getX(int n) const {
    int cnt=0;
    for (int i=0;i<_maxTargets;i++) {
        if (_targets[i].lostCounter==0 && _targets[i].valid) {
            cnt++;
            if (cnt==n) return _targets[i].x;
        }
    }
    return 0.0f;
}

float LD2450radar::getY(int n) const {
    int cnt=0;
    for (int i=0;i<_maxTargets;i++) {
        if (_targets[i].lostCounter==0 && _targets[i].valid) {
            cnt++;
            if (cnt==n) return _targets[i].y;
        }
    }
    return 0.0f;
}

float LD2450radar::getSpeed(int n) const {
    int cnt=0;
    for (int i=0;i<_maxTargets;i++) {
        if (_targets[i].lostCounter==0 && _targets[i].valid) {
            cnt++;
            if (cnt==n) return _targets[i].speed;
        }
    }
    return 0.0f;
}

// Raw indexed access
int LD2450radar::getIDRaw(int idx) const { if (idx<1||idx>_maxTargets) return -1; return _targets[idx-1].id; }
float LD2450radar::getAngleRaw(int idx) const { if (idx<1||idx>_maxTargets) return 0.0f; return _targets[idx-1].angle; }
float LD2450radar::getDistanceRaw(int idx) const { if (idx<1||idx>_maxTargets) return 0.0f; return _targets[idx-1].distance; }
float LD2450radar::getXRaw(int idx) const { if (idx<1||idx>_maxTargets) return 0.0f; return _targets[idx-1].x; }
float LD2450radar::getYRaw(int idx) const { if (idx<1||idx>_maxTargets) return 0.0f; return _targets[idx-1].y; }
float LD2450radar::getSpeedRaw(int idx) const { if (idx<1||idx>_maxTargets) return 0.0f; return _targets[idx-1].speed; }

void LD2450radar::reset() {
    for (int i=0;i<_maxTargets;i++) {
        _targets[i].id = -1;
        _targets[i].distance = 0.0f;
        _targets[i].x = 0.0f;
        _targets[i].y = 0.0f;
        _targets[i].angle = 0.0f;
        _targets[i].speed = 0.0f;
        _targets[i].lostCounter = 100;
        _targets[i].valid = false;
    }
}
