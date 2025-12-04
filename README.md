# Radar2450 Library

Расширенная обертка для библиотеки HLK-LD2450 с фильтрацией, трекингом целей и сглаживанием данных.

![Radar2450](https://img.shields.io/badge/version-1.0.0-blue)
![Platform](https://img.shields.io/badge/platform-ESP32%20%7C%20ESP8266-green)
![License](https://img.shields.io/badge/license-MIT-yellow)
![Arduino](https://img.shields.io/badge/Arduino-Compatible-blue)
![Downloads](https://img.shields.io/badge/downloads-100+-brightgreen)

## ?? Содержание
- [Особенности](#особенности)
- [Установка](#установка)
- [Подключение](#подключение)
- [Быстрый старт](#быстрый-старт)
- [Справочник API](#справочник-api)
- [Примеры](#примеры)
- [Настройка параметров](#настройка-параметров)
- [Типичные сценарии](#типичные-сценарии)
- [Устранение неполадок](#устранение-неполадок)
- [Вклад в проект](#вклад-в-проект)
- [Лицензия](#лицензия)

## ? Особенности

- ? **Экспоненциальное сглаживание** координат целей
- ? **Устойчивый трекинг** целей с уникальными ID
- ? **Расчет скорости** в м/с и км/ч
- ? **Настраиваемые ограничения** по углу и дистанции
- ? **Сортировка целей** по расстоянию (ближайшие первые)
- ? **Конфигурируемый коэффициент** сглаживания
- ? **Отладочная информация** и статистика
- ? **Совместимость** с ESP32 и ESP8266

## ?? Установка

### Arduino IDE
1. Установите библиотеку [HLK-LD2450](https://github.com/RBEGamer/HLK-LD2450) через Менеджер библиотек
2. Скачайте Radar2450 как ZIP-архив
3. Скетч > Подключить библиотеку > Добавить .ZIP библиотеку

### PlatformIO
Добавьте в `platformio.ini`:
```ini
lib_deps = 
    eu1abg/Radar2450-Library
    RBEGamer/HLK-LD2450

##?? Подключение

LD2450    ESP32
TX    >   GPIO16 (RADAR_RX)
RX    >   GPIO17 (RADAR_TX)
3.3V  >   3.3V
GND   >   GND

## ?? Быстрый старт

#include <Radar2450.h>

Radar2450 radar;

void setup() {
    Serial.begin(115200);
    radar.begin(16, 17);           // Инициализация радара
    radar.setMaxDistance(6.0);     // Макс. дистанция 6 метров
    radar.setSmoothingFactor(0.2); // Коэффициент сглаживания
}

void loop() {
    radar.update();                // Обновление данных
    
    if (radar.getActiveTargetCount() > 0) {
        float dist = radar.getTargetDistance(1);   // Дистанция до ближайшей цели
        float angle = radar.getTargetAngle(1);     // Угол цели
        float speed = radar.getTargetSpeedKmh(1);  // Скорость в км/ч
        
        Serial.printf("Цель: %.1fм, %.0f°, %.1fкм/ч\n", dist, angle, speed);
    }
    
    delay(50);  // 20 Гц - оптимальная частота обновления
}

## ?? Справочник API

Инициализация и конфигурация
Radar2450()
Конструктор класса. Создает новый экземпляр радара.

begin(int rx_pin, int tx_pin, long baud_rate = 256000)
Инициализация радара.

rx_pin: Пин для приема данных (RX)

tx_pin: Пин для передачи данных (TX)

baud_rate: Скорость UART (по умолчанию 256000)

void update()
Обновление данных радара. Должен вызываться в loop().

void setSmoothingFactor(float factor)
Установка коэффициента сглаживания (0.0-1.0).

void setMaxDistance(float distance)
Установка максимальной дистанции обнаружения (метры).

void setMinDistance(float distance)
Установка минимальной дистанции обнаружения (метры).

void setAngleFilter(float min_angle, float max_angle)
Установка углового фильтра (градусы).
