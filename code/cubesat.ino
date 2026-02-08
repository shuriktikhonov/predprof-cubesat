// Код для модуля системы наведения (Cubesat)

// Подключение необходимых библиотек
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// Пины радиомодуля nRF24L01+
const uint8_t PIN_CE = 9;
const uint8_t PIN_CSN = 10;

// Пины сервоприводов
const uint8_t PIN_X = 3;
const uint8_t PIN_Y = 4;

// Пин лазера
const uint8_t PIN_LASER = 5;

// Ограничения углов сервоприводов
const int16_t MIN = 0;
const int16_t MAX = 180;

// Точка (0; 0) сервоприводов
const int16_t CENTER_X = 90;
const int16_t CENTER_Y = 85;

// Маркер пакета, по которому будем отличать помехи по радиоканалу от наших пакетов
const uint16_t MAGIC = 0xBEEF;

// Подключение радиоканала и задача адреса соединения с наземной станцией
RF24 radio(PIN_CE, PIN_CSN);
const byte PIPE[6] = "AIM01";

// Пакет данных, получаемый от назменой станции
struct Data {
    uint16_t magic;
    int16_t x;
    int16_t y;
    uint8_t flag; // bit0: лазер, bit1: авто, bit2: E-STOP
    uint8_t count;
};

// Обозначение сервоприводов
Servo servoX;
Servo servoY;

// Переменные для проверки стабильности связи
uint32_t lastMs = 0;
bool link = false;

// Текущие значения координат (на старте прировняем к точке (0; 0))
int16_t x = CENTER_X;
int16_t y = CENTER_Y;

// Флаги режимов из пакета данных
bool laser = false;
bool eStop = false;

void setup() {
    // Настройка лазера
    pinMode(PIN_LASER, OUTPUT);
    digitalWrite(PIN_LASER, LOW);

    // Инициализация сервоприводов и назначение их в точку (0; 0)
    servoX.attach(PIN_X);
    servoY.attach(PIN_Y);
    servoX.write(x);
    servoY.write(y);

    // Инициализация радиомодуля
    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(112);
    radio.openReadingPipe(1, PIPE);
    radio.startListening();
}

void loop() {
    // Принимаем пакет
    if (radio.available()) {
        Data packet;
        while (radio.available()) {
            radio.read(&packet, sizeof(packet)); // читаем пакет и постоянно обновляем его
        }
        if (packet.magic == MAGIC) { // если пакет от наземной станции, можем выполнять с ним действия
            // отмечаем, что принят пакет
            lastMs = millis(); 
            link = true; 

            // разбираем флаги от пакета
            eStop = (packet.flag & 0x04); 
            laser = (packet.flag & 0x01);

            // если нет eStop, то можем управлять сервоприводами
            if (!eStop) {
                x = constrain(packet.x, MIN, MAX);
                y = constrain(packet.y, MIN, MAX);
                servoX.write(x);
                servoY.write(y);
            }
        }
    }

    // Таймаут связи
    if (millis() - lastMs > 700) {
        link = false;
    }

    // Если связи нет, или eStop => выключаем лазер
    if (!link || eStop) {
        digitalWrite(PIN_LASER, LOW);
    }
    // Иначе, смотрим в пакет запрошено ли включение лазера
    else {
        digitalWrite(PIN_LASER, laser ? HIGH : LOW);
    }
}