// 1. ИНИЦИАЛИЗАЦИЯ
Radar2450 radar;
radar.begin(16, 17);  // RX=16, TX=17

// 2. ОБНОВЛЕНИЕ ДАННЫХ
int detected = radar.update();

// 3. ОСНОВНЫЕ ДАННЫЕ
int count = radar.getActiveTargetCount();     // Количество целей
int id = radar.getTargetID(1);               // ID первой цели
float dist = radar.getTargetDistance(1);     // Дистанция до цели
float angle = radar.getTargetAngle(1);       // Угол цели
float x = radar.getTargetX(1);               // Координата X
float y = radar.getTargetY(1);               // Координата Y
float speed = radar.getTargetSpeed(1);       // Скорость (м/с)
float speedKmh = radar.getTargetSpeedKmh(1); // Скорость (км/ч)
bool moving = radar.getTargetMoving(1);      // Движется ли цель

// 4. КОНФИГУРАЦИЯ (опционально)
radar.setMaxDistance(8.0);      // Макс. дистанция
radar.setSmoothingFactor(0.2);  // Сглаживание
radar.setMaxTargets(5);         // Макс. целей
radar.setAngleLimits(-60, 60);  // Угол обзора

// 5. ОТЛАДКА
radar.printDebugInfo();        // Вывод информации
float rate = radar.getUpdateRateHz(); // Частота обновления
radar.reset();                 // Сброс целей