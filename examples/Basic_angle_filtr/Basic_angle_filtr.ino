#include <LD2450radar.h>

LD2450radar radar(Serial1, 16, 17);

// Настройка углового сектора
float minAngle = -45.0f;  // Левая граница
float maxAngle = 45.0f;   // Правая граница

void setup() {
    Serial.begin(115200);
    radar.begin();
}

void loop() {
    radar.update();
    
    // Проверяем ВСЕ цели (сырой доступ)
    for (int i = 1; i <= radar.capacity(); i++) {
        int targetID = radar.getIDRaw(i);
        
        if (targetID != -1) { // Если цель существует
            float angle = radar.getAngleRaw(i);
            
            // Фильтрация по углу
            if (angle >= minAngle && angle <= maxAngle) {
                // Цель в нужном секторе
                float distance = radar.getDistanceRaw(i);
                Serial.printf("Цель %d: угол=%.1f°, дистанция=%.2fм\n", 
                             targetID, angle, distance);
                
                // Ваша логика обработки...
            }
        }
    }
    
    delay(50);
}