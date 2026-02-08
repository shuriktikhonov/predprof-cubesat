// Код для наземной станции

// Подключение необходимых библиотек
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Подключение OLED-дисплея
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Пины модуля nRF24L01+
const uint8_t PIN_CE = 9;
const uint8_t PIN_CSN = 10;

// Пины энкодера (A/B/кнопка)
const uint8_t PIN_ENC_A = 2;
const uint8_t PIN_ENC_B = 3;
const uint8_t PIN_ENC_BTN = 4;

// Кнопки: аварийная остановка, режим, центр
const uint8_t PIN_ESTOP = 5;
const uint8_t PIN_MODE = 6;
const uint8_t PIN_CENTER = 7;

// Ограничения углов сервоприводов
const int16_t X_MIN = 0;
const int16_t X_MAX = 180;
const int16_t Y_MIN = 0;
const int16_t Y_MAX = 180;

// Центр и параметры автосканирования (диапазон от центра)
const int16_t CENTER_X = 90;
const int16_t CENTER_Y = 85;
const int16_t AUTO_RANGE = 40;
const int16_t AUTO_MIN_X = CENTER_X - AUTO_RANGE;
const int16_t AUTO_MAX_X = CENTER_X + AUTO_RANGE;
const int16_t AUTO_MIN_Y = CENTER_Y - AUTO_RANGE;
const int16_t AUTO_MAX_Y = CENTER_Y + AUTO_RANGE;
const uint16_t STEP_MS = 3000; // задержка между шагами авто

// Коррекция авто: место для своих координат движения.
// Если значение = USE_SPEC, то движение будет по координатам ТЗ.
// Можно заменить на любые реальные координаты сервоприводов (0..180).
const int16_t USE_SPEC = -10000;
const int16_t AUTO_X[4][9] = {
  // Фаза 0: горизонталь 
  {USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC},
  // Фаза 1: вертикаль 
  {USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC},
  // Фаза 2: диагональ 1
  {USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC},
  // Фаза 3: диагональ 2
  {USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC}
};
const int16_t AUTO_Y[4][9] = {
  // Фаза 0: горизонталь 
  {USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC},
  // Фаза 1: вертикаль 
  {USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC},
  // Фаза 2: диагональ 1
  {USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC},
  // Фаза 3: диагональ 2
  {USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC, USE_SPEC}
};

// Маркер пакета для защиты от мусора
const uint16_t MAGIC = 0xBEEF;

// Радиоканал на модуль наведения
RF24 radio(PIN_CE, PIN_CSN);
const byte PIPE[6] = "AIM01";

// Пакет управления для модуля наведения
struct Data {
  uint16_t magic;
  int16_t x;
  int16_t y;
  uint8_t flag; // bit0: лазер, bit1: авто, bit2: E-STOP
  uint8_t count;
};

// Антидребезг для кнопок
struct Button {
  uint8_t pin;
  bool stable;
  bool last;
  uint32_t lastChange;
  bool pressed;
  bool released;

  void begin() {
    pinMode(pin, INPUT_PULLUP);
    bool r = (digitalRead(pin) == LOW);
    stable = r;
    last = r;
    lastChange = 0;
    pressed = false;
    released = false;
  }

  void update() {
    pressed = false;
    released = false;
    bool r = (digitalRead(pin) == LOW);
    if (r != last) {
      lastChange = millis();
      last = r;
    }
    if ((millis() - lastChange) > 25) {
      if (r != stable) {
        stable = r;
        if (stable) pressed = true;
        else released = true;
      }
    }
  }

  bool isPressed() const { return stable; }
};

// Кнопки
Button btnEStop { PIN_ESTOP };
Button btnMode { PIN_MODE };
Button btnCenter { PIN_CENTER };
Button btnEnc { PIN_ENC_BTN };

// Состояние энкодера (ISR обновляет дельту)
volatile int8_t encDelta = 0;
volatile uint8_t lastA = 0;

// ISR энкодера: собираем шаги вращения
void encIsr() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  if (a == lastA) return;
  lastA = a;
  if (a == b) encDelta++;
  else encDelta--;
}

// Текущие и последние отправленные цели
int16_t x = CENTER_X;
int16_t y = CENTER_Y;
int16_t sentX = CENTER_X;
int16_t sentY = CENTER_Y;
// Реальные координаты движения в авто (с учетом коррекции)
int16_t autoX = CENTER_X;
int16_t autoY = CENTER_Y;

// Флаги режима
bool mode = false;
bool laser = false;
bool eStop = false;

// Ручной режим: выбранная ось и счетчик пакетов
uint8_t axis = 0; // 0 = X, 1 = Y
uint8_t count = 0;

// Есть ли новые ручные данные для отправки
bool manual = false;

// Таймеры
uint32_t sendMs = 0;
uint32_t autoStepMs = 0;
uint32_t displayMs = 0;

// Таймеры удержания кнопок
uint32_t encTimer = 0;
uint32_t modeTimer = 0;

// Состояние автосканирования
bool link = false;
uint8_t phase = 0; // 0: горизонталь, 1: вертикаль, 2: диагональ 1, 3: диагональ 2
uint8_t index = 0; // 0..8 соответствует 40..-40 шаг 10
uint8_t state = 0; // 0: ожидание, 1: выполнение, 2: удержание последней точки, 3: удержание центра
uint32_t holdStartMs = 0; // таймер удержания (последняя точка / центр)

// Помощники для вычисления смещений авто
int16_t autoOffsetPos(uint8_t index) {
  return 40 - (int16_t)index * 10;
}

int16_t autoOffsetNeg(uint8_t index) {
  return -40 + (int16_t)index * 10;
}

// Автосканирование
void autoMode() {
  int16_t offA = autoOffsetPos(index); // 40..-40
  int16_t offB = autoOffsetNeg(index); // -40..40
  uint8_t stepPhase = phase;
  uint8_t stepIndex = index;

  switch (phase) {
    case 0: // Горизонтальный скан: X 40..-40, Y = центр
      x = CENTER_X + offA;
      y = CENTER_Y;
      break;
    case 1: // Вертикальный скан: Y 40..-40, X = центр
      x = CENTER_X;
      y = CENTER_Y + offA;
      break;
    case 2: // Диагональ 1: X 40..-40, Y 40..-40
      x = CENTER_X + offA;
      y = CENTER_Y + offA;
      break;
    case 3: // Диагональ 2: X -40..40, Y 40..-40
      x = CENTER_X + offB;
      y = CENTER_Y + offA;
      break;
  }

  // Защита по диапазону сервоприводов
  x = constrain(x, X_MIN, X_MAX);
  y = constrain(y, Y_MIN, Y_MAX);

  // Вычисляем реальные координаты движения в авто
  int16_t mx = AUTO_X[stepPhase][stepIndex];
  int16_t my = AUTO_Y[stepPhase][stepIndex];
  if (mx == USE_SPEC) mx = x;
  if (my == USE_SPEC) my = y;
  autoX = constrain(mx, X_MIN, X_MAX);
  autoY = constrain(my, Y_MIN, Y_MAX);

  // Последний шаг последней диагонали
  if (phase == 3 && index == 8) {
    state = 2;
    holdStartMs = 0;
    index = 0;
    phase = 0;
    return;
  }

  // Переход к следующему шагу/фазе
  index++;
  if (index >= 9) {
    index = 0;
    phase = (phase + 1) % 4;
  }
}

// Отрисовка экрана
void renderDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("MODE: ");
  display.println(mode ? "AUTO" : "MANUAL");

  display.setCursor(0, 20);
  display.print("Axis: ");
  display.println(axis == 0 ? "X" : "Y");

  display.setCursor(0, 32);
  display.print("X: ");
  display.print(-(x - CENTER_X));
  display.print("  Y: ");
  display.println(y - CENTER_Y);

  display.setCursor(0, 44);
  display.print("Laser: ");
  display.println(laser ? "ON" : "OFF");

  display.setCursor(0, 56);
  if (eStop) {
    display.print("E-STOP ACTIVE");
  } else {
    display.print("Radio: ");
    display.println(link ? "OK" : "NO");
  }
  display.display();
}

void setup() {
  // Пины энкодера + прерывание
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encIsr, CHANGE);

  // Кнопки
  btnEStop.begin();
  btnMode.begin();
  btnCenter.begin();
  btnEnc.begin();

  // OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  // Радио
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(112);
  radio.openWritingPipe(PIPE);
  radio.stopListening();
}

void loop() {
  // Чтение кнопок
  btnEStop.update();
  btnMode.update();
  btnCenter.update();
  btnEnc.update();

  // Красная кнопка: аварийная остановка (защелка)
  if (btnEStop.pressed) {
    eStop = true;
  }

  // Синяя кнопка: возврат в центр и немедленная отправка
  if (btnCenter.pressed) {
    x = CENTER_X;
    y = CENTER_Y;
    sentX = x;
    sentY = y;
    autoX = x;
    autoY = y;
    manual = false;
    phase = 0;
    index = 0;
    autoStepMs = millis();

    Data packet;
    packet.magic = MAGIC;
    packet.x = sentX;
    packet.y = sentY;
    packet.flag = 0;
    if (laser) packet.flag |= 0x01;
    if (mode) packet.flag |= 0x02;
    if (eStop) packet.flag |= 0x04;
    packet.count = count++;
    link = radio.write(&packet, sizeof(packet));
    sendMs = millis();
  }

  // Желтая кнопка: переключение режима и снятие E-STOP
  if (btnMode.pressed) modeTimer = millis();
  if (btnMode.released) {
    uint32_t dur = millis() - modeTimer;
    if (eStop && dur >= 1500) {
      // Долгое нажатие: снять E-STOP и прервать автоцикл
      eStop = false;
      mode = false;
      state = 0;
      phase = 0;
      index = 0;
      manual = false;
      holdStartMs = 0;
      x = CENTER_X;
      y = CENTER_Y;
      sentX = x;
      sentY = y;
      autoX = x;
      autoY = y;

      Data packet;
      packet.magic = MAGIC;
      packet.x = sentX;
      packet.y = sentY;
      packet.flag = 0;
      if (laser) packet.flag |= 0x01;
      packet.count = count++;
      link = radio.write(&packet, sizeof(packet));
      sendMs = millis();
    } else if (!eStop && dur < 1500) {
      // Короткое нажатие: переключить авто/ручной
      mode = !mode;
      if (mode) {
      state = 1;
      phase = 0;
      index = 0;
      manual = false;
      autoMode();
      autoStepMs = millis();
      holdStartMs = 0;
    } else {
      state = 0;
    }
    }
  }

  // Кнопка энкодера: коротко — смена оси, долго — лазер
  if (btnEnc.pressed) encTimer = millis();
  if (btnEnc.released) {
    uint32_t dur = millis() - encTimer;
    if (dur >= 800) {
      laser = !laser;
    } else {
      axis = (axis == 0) ? 1 : 0;
    }
  }

  // Забираем дельту от энкодера (из ISR)
  int8_t d;
  noInterrupts();
  d = encDelta;
  encDelta = 0;
  interrupts();

  // Ручное управление энкодером
  if (!mode && d != 0) {
    int16_t step = 1;
    if (axis == 0) {
      x = constrain(x + d * step, X_MIN, X_MAX);
    } else {
      y = constrain(y + d * step, Y_MIN, Y_MAX);
    }
    manual = true;
  }

  // Автосканирование по шагам
  if (mode && !eStop) {
    uint32_t now = millis();
    if (state == 1 && now - autoStepMs >= STEP_MS) {
      autoStepMs = now;
      autoMode();
    }
    if (state == 2) {
      // Удерживаем последнюю точку 
      if (holdStartMs == 0) holdStartMs = now;
      if (now - holdStartMs >= STEP_MS) {
        state = 3;
        holdStartMs = now;
        x = CENTER_X;
        y = CENTER_Y;
        autoX = x;
        autoY = y;
      }
    } else if (state == 3) {
      // Удерживаем центр, затем выходим из авто
      x = CENTER_X;
      y = CENTER_Y;
      autoX = x;
      autoY = y;
      if (holdStartMs == 0) holdStartMs = now;
      if (now - holdStartMs >= STEP_MS) {
        state = 0;
        mode = false;
        holdStartMs = 0;
      }
    }
  }

  // Решаем, когда слать пакет
  bool Send = false;
  uint32_t now = millis();

  if (eStop) {
    // При E-STOP периодически шлем флаг
    if (now - sendMs >= 150) Send = true;
  } else if (mode) {
    // В авто режиме отправляем чаще
    if (now - sendMs >= 50) Send = true;
    sentX = autoX;
    sentY = autoY;
  } else {
    // В ручном режиме: сразу при изменении, + постоянно обновляем
    if (manual) {
      sentX = x;
      sentY = y;
      Send = true;
      manual = false;
    } else if (now - sendMs >= 300) {
      Send = true;
    }
  }

  // Передача пакета по радио
  if (Send) {
    Data packet;
    packet.magic = MAGIC;
    packet.x = sentX;
    packet.y = sentY;
    packet.flag = 0;
    if (laser) packet.flag |= 0x01;
    if (mode) packet.flag |= 0x02;
    if (eStop) packet.flag |= 0x04;
    packet.count = count++;

    link = radio.write(&packet, sizeof(packet));
    sendMs = now;
  }

  // Обновление OLED
  if (now - displayMs >= 100) {
    displayMs = now;
    renderDisplay();
  }
}