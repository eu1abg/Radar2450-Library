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

    // Constructor:
    // serial - reference to existing HardwareSerial (Serial1/Serial2)
    // rxPin, txPin - pins for that serial port (ESP32: use proper pins)
    // baud - serial speed (default 256000)
    // maxTargets - internal capacity (default 10)
    // alpha - EMA smoothing (0..1)
    // maxDistM - clamp distances in meters
    LD2450radar(HardwareSerial &serial, int rxPin, int txPin, unsigned long baud = 256000,
                int maxTargets = 10, float alpha = 0.1f, float maxDistM = 10.0f);

    // Initialize low-level serial + radar. Call in setup()
    bool begin(bool configureMultiTarget = true);

    // Update: read radar frames and process; returns number of raw targets read
    int update();

    // Active target helpers (1-based index over active targets)
    int getActiveCount() const;
    int getID(int n) const;
    float getAngle(int n) const;
    float getDistance(int n) const;
    float getX(int n) const;
    float getY(int n) const;
    float getSpeed(int n) const;

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
    HardwareSerial &_serial;
    LD2450 _radar;
    int _rxPin, _txPin;
    unsigned long _baud;
    int _maxTargets;
    float _alpha;
    float _maxDistM;
    Target *_targets; // dynamic array
};

#endif // LD2450RADAR_H
