#include "LD2450radar.h"

LD2450radar::LD2450radar(HardwareSerial &serial, int rxPin, int txPin, unsigned long baud,
                         int maxTargets, float alpha, float maxDistM)
: _serial(serial), _rxPin(rxPin), _txPin(txPin), _baud(baud),
  _maxTargets(maxTargets), _alpha(alpha), _maxDistM(maxDistM),
  _activeCount(0), _lastUpdateTime(0), _activeListDirty(true)
{
    // Protection against incorrect values
    if (_maxTargets <= 0) _maxTargets = 10;
    if (_baud == 0) _baud = 256000;
    if (_alpha < 0.0f || _alpha > 1.0f) _alpha = 0.1f;
    if (_maxDistM <= 0.0f) _maxDistM = 10.0f;
    
    _targets = new Target[_maxTargets];
    _activeTargets = new Target*[_maxTargets];
    
    // Initialize targets
    for (int i=0; i<_maxTargets; i++) {
        _targets[i].id = -1;
        _targets[i].lostCounter = 100;
        _targets[i].valid = false;
    }
}

LD2450radar::~LD2450radar() {
    if (_targets) {
        delete[] _targets;
        _targets = nullptr;
    }
    if (_activeTargets) {
        delete[] _activeTargets;
        _activeTargets = nullptr;
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
    _lastUpdateTime = millis();
    return true;
}

void LD2450radar::rebuildActiveList() {
    _activeCount = 0;
    for (int i = 0; i < _maxTargets; i++) {
        if (_targets[i].lostCounter == 0 && _targets[i].valid) {
            _activeTargets[_activeCount++] = &_targets[i];
        }
    }
    _activeListDirty = false;
}

int LD2450radar::update() {
    if (!_radar.update()) return 0;
    
    // Calculate real time between updates
    unsigned long now = millis();
    float dt = 0.05f; // default value
    
    if (_lastUpdateTime > 0) {
        dt = (now - _lastUpdateTime) / 1000.0f; // in seconds
        if (dt <= 0.0f) dt = 0.001f; // minimum interval
    }
    _lastUpdateTime = now;
    
    int nr = _radar.getNrValidTargets();

    // increment lost counters
    for (int i=0; i<_maxTargets; i++) _targets[i].lostCounter++;

    // Flag to track if active list needs rebuilding
    bool targetsChanged = false;

    for (int i=0; i<nr; i++) {
        auto t = _radar.getTarget(i);
        if (!t.valid) continue;

        float x_m = float(t.x) / 1000.0f;
        float y_m = float(t.y) / 1000.0f;
        float dist_m = sqrtf(x_m*x_m + y_m*y_m);
        
        // ORIGINAL ANGLE CALCULATION: atan2f(x_m, y_m) - gives 0° center, -60° to +60°
        float angle = atan2f(x_m, y_m) * 180.0f / M_PI;

        // clamp
        if (dist_m < 0.0f) dist_m = 0.0f;
        if (dist_m > _maxDistM) dist_m = _maxDistM;
        if (angle < -59.0f) angle = -60.0f;
        if (angle > 59.0f) angle = 60.0f;

        int slot = -1;
        // find by id
        for (int s=0; s<_maxTargets; s++) {
            if (_targets[s].id == t.id) { slot = s; break; }
        }
        // else find free or expired
        if (slot == -1) {
            for (int s=0; s<_maxTargets; s++) {
                if (_targets[s].id == -1 || _targets[s].lostCounter > 10) { slot = s; break; }
            }
        }

        if (slot != -1) {
            float oldDist = _targets[slot].distance;

            _targets[slot].id = t.id;
            _targets[slot].x = _alpha * x_m + (1.0f - _alpha) * _targets[slot].x;
            _targets[slot].y = _alpha * y_m + (1.0f - _alpha) * _targets[slot].y;
            _targets[slot].distance = sqrtf(_targets[slot].x*_targets[slot].x + _targets[slot].y*_targets[slot].y);
            
            // ORIGINAL ANGLE CALCULATION for smoothed values
            _targets[slot].angle = atan2f(_targets[slot].x, _targets[slot].y) * 180.0f / M_PI;
            
            _targets[slot].lostCounter = 0;
            _targets[slot].valid = true;

            // speed estimate with real time calculation
            if (oldDist > 0.0f && dt > 0.0f) {
                _targets[slot].speed = (_targets[slot].distance - oldDist) / dt;
            } else {
                _targets[slot].speed = 0.0f;
            }
            
            targetsChanged = true;
        }
    }
    
    // Rebuild active list only if there were changes
    if (targetsChanged || nr > 0) {
        _activeListDirty = true;
    }
    
    // Rebuild active list if needed
    if (_activeListDirty) {
        rebuildActiveList();
    }

    return nr;
}

// Raw indexed access
int LD2450radar::getIDRaw(int idx) const { 
    if (idx<1||idx>_maxTargets) return -1; 
    return _targets[idx-1].id; 
}

float LD2450radar::getAngleRaw(int idx) const { 
    if (idx<1||idx>_maxTargets) return 0.0f; 
    return _targets[idx-1].angle; 
}

float LD2450radar::getDistanceRaw(int idx) const { 
    if (idx<1||idx>_maxTargets) return 0.0f; 
    return _targets[idx-1].distance; 
}

float LD2450radar::getXRaw(int idx) const { 
    if (idx<1||idx>_maxTargets) return 0.0f; 
    return _targets[idx-1].x; 
}

float LD2450radar::getYRaw(int idx) const { 
    if (idx<1||idx>_maxTargets) return 0.0f; 
    return _targets[idx-1].y; 
}

float LD2450radar::getSpeedRaw(int idx) const { 
    if (idx<1||idx>_maxTargets) return 0.0f; 
    return _targets[idx-1].speed; 
}

void LD2450radar::reset() {
    for (int i=0; i<_maxTargets; i++) {
        _targets[i].id = -1;
        _targets[i].distance = 0.0f;
        _targets[i].x = 0.0f;
        _targets[i].y = 0.0f;
        _targets[i].angle = 0.0f;
        _targets[i].speed = 0.0f;
        _targets[i].lostCounter = 100;
        _targets[i].valid = false;
    }
    _activeCount = 0;
    _activeListDirty = true;
}