#pragma once
#ifndef LD2450RADAR_H
#define LD2450RADAR_H

#include <Arduino.h>
#include <LD2450.hpp> // HLK-LD245X driver required
#include <math.h>

using namespace esphome::ld245x;

class LD2450radar {
public:
    struct Target {
        int id = -1;
        float distance = 0.0f; // meters
        float x = 0.0f; // meters
        float y = 0.0f; // meters
        float angle = 0.0f; // degrees
        float speed = 0.0f; // m/s
        int lostCounter = 0;
        bool valid = false;
    };

    // Constructor
    LD2450radar(HardwareSerial &serial, int rxPin, int txPin, unsigned long baud = 256000,
                int maxTargets = 10, float alpha = 0.1f, float maxDistM = 10.0f);

    // Destructor
    ~LD2450radar();

    // Initialize low-level serial + radar. Call in setup()
    bool begin(bool configureMultiTarget = true);

    // Update: read radar frames and process; returns number of raw targets read
    int update();

    // Active target helpers (1-based index over active targets) - O(1) access
    int getActiveCount() const { return _activeCount; }
    int getID(int n) const { return (n>=1 && n<=_activeCount) ? _activeTargets[n-1]->id : -1; }
    float getAngle(int n) const { return (n>=1 && n<=_activeCount) ? _activeTargets[n-1]->angle : 0.0f; }
    float getDistance(int n) const { return (n>=1 && n<=_activeCount) ? _activeTargets[n-1]->distance : 0.0f; }
    float getX(int n) const { return (n>=1 && n<=_activeCount) ? _activeTargets[n-1]->x : 0.0f; }
    float getY(int n) const { return (n>=1 && n<=_activeCount) ? _activeTargets[n-1]->y : 0.0f; }
    float getSpeed(int n) const { return (n>=1 && n<=_activeCount) ? _activeTargets[n-1]->speed : 0.0f; }

    // Fast direct access to target objects
    const Target* getActiveTarget(int n) const { return (n>=1 && n<=_activeCount) ? _activeTargets[n-1] : nullptr; }
    
    // Fast access to all active targets array
    const Target* const* getActiveTargets() const { return _activeTargets; }

    // Raw indexed access (1..capacity())
    int getIDRaw(int idx) const;
    float getAngleRaw(int idx) const;
    float getDistanceRaw(int idx) const;
    float getXRaw(int idx) const;
    float getYRaw(int idx) const;
    float getSpeedRaw(int idx) const;

    // Capacity and reset
    int capacity() const { return _maxTargets; }
    void reset();

private:
    void rebuildActiveList(); // Rebuild active targets list
    
    HardwareSerial &_serial;
    LD2450 _radar;
    int _rxPin, _txPin;
    unsigned long _baud;
    int _maxTargets;
    float _alpha;
    float _maxDistM;
    Target *_targets; // dynamic array
    Target **_activeTargets; // array of pointers to active targets
    int _activeCount; // number of active targets
    unsigned long _lastUpdateTime;
    bool _activeListDirty; // flag to indicate active list needs rebuilding
};

#endif // LD2450RADAR_H