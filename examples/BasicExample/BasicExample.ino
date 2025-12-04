/*
  Radar2450 - Basic Example
  =========================
  
  Connections for ESP32:
  LD2450 TX -> GPIO16
  LD2450 RX -> GPIO17
  LD2450 3.3V -> 3.3V
  LD2450 GND -> GND
*/

#include <Radar2450.h>

#define RADAR_RX 16
#define RADAR_TX 17

Radar2450 radar;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("==================================");
    Serial.println("Radar2450 - Basic Example");
    Serial.println("==================================");
    
    // Initialize radar
    if (radar.begin(RADAR_RX, RADAR_TX)) {
        Serial.println("✓ Radar initialized successfully!");
        
        // Optional configuration
        radar.setMaxDistance(6.0);        // Max detection distance: 6m
        radar.setSmoothingFactor(0.2);    // Smoothing factor: 0.2
        radar.setMaxTargets(5);           // Track up to 5 targets
        radar.setAngleLimits(-45, 45);    // Field of view: -45° to 45°
        
        Serial.println("Configuration applied:");
        Serial.println("  Max Distance: 6.0m");
        Serial.println("  Smoothing: 0.2");
        Serial.println("  Max Targets: 5");
        Serial.println("  FOV: -45° to 45°");
        Serial.println("\nReady to detect targets!");
    } else {
        Serial.println("✗ Failed to initialize radar!");
        Serial.println("Check wiring and restart.");
        while(1);
    }
}

void loop() {
    // Update radar data
    int detected = radar.update();
    
    // Get number of active (tracked) targets
    int activeTargets = radar.getActiveTargetCount();
    
    if (activeTargets > 0) {
        Serial.printf("\n[%lu ms] Active Targets: %d\n", millis(), activeTargets);
        
        // Display information for each active target
        for (int i = 1; i <= activeTargets; i++) {
            Serial.printf("Target #%d:\n", i);
            Serial.printf("  ID: %d\n", radar.getTargetID(i));
            Serial.printf("  Distance: %.2f m\n", radar.getTargetDistance(i));
            Serial.printf("  Angle: %.1f°\n", radar.getTargetAngle(i));
            Serial.printf("  Position: X=%.2f m, Y=%.2f m\n", 
                         radar.getTargetX(i), radar.getTargetY(i));
            Serial.printf("  Speed: %.2f m/s (%.1f km/h)\n",
                         radar.getTargetSpeed(i), radar.getTargetSpeedKmh(i));
            Serial.printf("  Moving: %s\n", 
                         radar.getTargetMoving(i) ? "YES" : "NO");
        }
    }
    
    // Print debug info every 5 seconds
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 5000) {
        Serial.println("\n--- Debug Information ---");
        radar.printDebugInfo();
        Serial.printf("Update Rate: %.1f Hz\n", radar.getUpdateRateHz());
        lastDebug = millis();
    }
    
    delay(50); // ~20Hz update rate
}