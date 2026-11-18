#include <Arduino.h>

// Cabezal Delta-SC-02 suelto (la placa de la Roomba ya no existe).
// El C3 no habla con el lidar: no tiene RX. Solo lo alimenta, lo hace girar
// y reenvía TX1 por el USB nativo. La historia está en ../README.md.
//
//   TX1 -- 1 kΩ -- GPIO20 -- 2 kΩ -- GND     (5 V de TX → ~3,3 V)
//   GPIO21 suelto
//   GPIO4  PWM al lado bajo del motor. M+ queda en los 5 V del buck-boost.
//   El motor es de 3,5 V, así que el duty no pasa de 70 % (0,70 × 5 V).
//   GPIO18/19 son D−/D+ del USB: no usarlos.
//
// Los bytes de TX1 salen por USB sin modificar. En paralelo se busca una
// trama AA .. 61 AD|AE con checksum de suma, y el byte de offset 8 (÷ SCALE,
// 20) es la realimentación del PI. Si 800 ms no hay trama válida, el duty
// vuelve a START en vez de quedarse pegado al máximo.

static const int LIDAR_RX = 20;
static const int LIDAR_TX = 21;
static const int PWM_PIN = 4;
static const int PWM_CH = 0;

static const uint8_t BUF = 128;
static uint8_t buf[BUF];
static uint8_t rxb[512];
static int rxn = 0;

static char cmd[48];
static uint8_t cmdLen = 0;

static float targetHz = 6.0f;
static float measHz = 0.0f;
static float scale = 20.0f;
static float duty = 0.0f;
static float kp = 8.0f;
static float ki = 5.0f;
static float kd = 0.0f;
static float integ = 0.0f;
static float prevErr = 0.0f;
static bool autoLoop = true;
static bool haveFrame = false;
static int dutyMax = 70;
static int dutyMin = 0;
static int startPct = 50;
static uint32_t pwmFreq = 1000;
static uint32_t lastCtrl = 0;
static uint32_t lastFrame = 0;
static uint32_t lastStatus = 0;
static bool statusEverySec = false;

static int clampPct(float value) {
  if (value < dutyMin) {
    return dutyMin;
  }
  if (value > dutyMax) {
    return dutyMax;
  }
  return static_cast<int>(value + 0.5f);
}

static void applyDuty() {
  const int pct = clampPct(duty);
  duty = static_cast<float>(pct);
  const uint32_t ticks = (255u * static_cast<uint32_t>(pct)) / 100u;
  ledcWrite(PWM_CH, ticks);
}

static void setupPwm() {
  if (pwmFreq < 50) {
    pwmFreq = 50;
  }
  if (pwmFreq > 20000) {
    pwmFreq = 20000;
  }
  ledcSetup(PWM_CH, pwmFreq, 8);
  ledcAttachPin(PWM_PIN, PWM_CH);
  applyDuty();
}

static void printStatus() {
  Serial.printf(
      "\n## HZ %.2f TGT %.2f PWM %d AUTO %d FREQ %lu MAX %d\n",
      measHz,
      targetHz,
      clampPct(duty),
      autoLoop ? 1 : 0,
      static_cast<unsigned long>(pwmFreq),
      dutyMax);
}

static void noteSpeed(uint8_t raw) {
  const float z = static_cast<float>(raw) / scale;
  if (!haveFrame) {
    measHz = z;
    haveFrame = true;
  } else {
    measHz += 0.35f * (z - measHz);
  }
  lastFrame = millis();
}

static uint16_t u16at(int i) {
  return (static_cast<uint16_t>(rxb[i]) << 8) | rxb[i + 1];
}

static void parseRx() {
  int i = 0;
  while (i + 11 <= rxn) {
    if (rxb[i] != 0xAA) {
      i++;
      continue;
    }
    const int length = u16at(i + 1);
    if (length < 9 || length > 250) {
      i++;
      continue;
    }
    if (i + length + 2 > rxn) {
      break;
    }
    if (rxb[i + 4] != 0x61 || (rxb[i + 5] != 0xAD && rxb[i + 5] != 0xAE)) {
      i++;
      continue;
    }
    uint32_t sum = 0;
    for (int k = 0; k < length; k++) {
      sum += rxb[i + k];
    }
    if ((sum & 0xFFFF) != u16at(i + length)) {
      i++;
      continue;
    }
    noteSpeed(rxb[i + 8]);
    i += length + 2;
  }
  if (i > 0) {
    memmove(rxb, rxb + i, rxn - i);
    rxn -= i;
  }
}

static void handleLine(char* s) {
  while (*s == ' ') {
    s++;
  }
  if (*s == 0) {
    return;
  }
  for (char* p = s; *p; p++) {
    if (*p >= 'a' && *p <= 'z') {
      *p = static_cast<char>(*p - 32);
    }
  }
  if (!strncmp(s, "HZ ", 3)) {
    targetHz = atof(s + 3);
    if (targetHz < 0.0f) {
      targetHz = 0.0f;
    }
    if (targetHz > 20.0f) {
      targetHz = 20.0f;
    }
    autoLoop = true;
    integ = 0.0f;
    prevErr = targetHz - measHz;
  } else if (!strncmp(s, "PWM ", 4)) {
    autoLoop = false;
    duty = atof(s + 4);
    applyDuty();
  } else if (!strncmp(s, "AUTO ", 5)) {
    autoLoop = atoi(s + 5) != 0;
    integ = 0.0f;
  } else if (!strncmp(s, "MAX ", 4)) {
    dutyMax = atoi(s + 4);
    if (dutyMax < 0) {
      dutyMax = 0;
    }
    if (dutyMax > 100) {
      dutyMax = 100;
    }
    if (dutyMin > dutyMax) {
      dutyMin = dutyMax;
    }
  } else if (!strncmp(s, "MIN ", 4)) {
    dutyMin = atoi(s + 4);
    if (dutyMin < 0) {
      dutyMin = 0;
    }
    if (dutyMin > dutyMax) {
      dutyMin = dutyMax;
    }
  } else if (!strncmp(s, "START ", 6)) {
    startPct = atoi(s + 6);
    if (startPct < dutyMin) {
      startPct = dutyMin;
    }
    if (startPct > dutyMax) {
      startPct = dutyMax;
    }
  } else if (!strncmp(s, "FREQ ", 5)) {
    pwmFreq = static_cast<uint32_t>(atoi(s + 5));
    setupPwm();
  } else if (!strncmp(s, "SCALE ", 6)) {
    scale = atof(s + 6);
    if (scale < 1.0f) {
      scale = 1.0f;
    }
  } else if (!strncmp(s, "KP ", 3)) {
    kp = atof(s + 3);
  } else if (!strncmp(s, "KI ", 3)) {
    ki = atof(s + 3);
  } else if (!strncmp(s, "KD ", 3)) {
    kd = atof(s + 3);
  } else if (!strcmp(s, "STATUS")) {
    printStatus();
    return;
  } else if (!strncmp(s, "TICK ", 5)) {
    statusEverySec = atoi(s + 5) != 0;
  }
  printStatus();
}

static void handleHost(uint8_t b) {
  if (b == '\r') {
    return;
  }
  if (b != '\n') {
    if (cmdLen < sizeof(cmd) - 1) {
      cmd[cmdLen++] = static_cast<char>(b);
    }
    return;
  }
  cmd[cmdLen] = 0;
  cmdLen = 0;
  handleLine(cmd);
}

static void control(uint32_t now) {
  if (!autoLoop || now - lastCtrl < 50) {
    return;
  }
  const float dt = (now - lastCtrl) / 1000.0f;
  lastCtrl = now;
  if (!haveFrame || now - lastFrame > 800) {
    integ = 0.0f;
    prevErr = 0.0f;
    duty = static_cast<float>(startPct);
    applyDuty();
    return;
  }
  const float err = targetHz - measHz;
  const bool atMax = duty >= dutyMax - 0.1f && err > 0.0f;
  const bool atMin = duty <= dutyMin + 0.1f && err < 0.0f;
  if (!atMax && !atMin) {
    integ += err * dt;
  }
  const float denom = fabsf(ki) > 0.05f ? fabsf(ki) : 0.05f;
  const float iCap = static_cast<float>(dutyMax) / denom;
  if (integ > iCap) {
    integ = iCap;
  }
  if (integ < -iCap) {
    integ = -iCap;
  }
  const float deriv = dt > 0.0f ? (err - prevErr) / dt : 0.0f;
  prevErr = err;
  duty = static_cast<float>(startPct) + kp * err + ki * integ + kd * deriv;
  applyDuty();
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  duty = 0.0f;
  setupPwm();

  Serial.setRxBufferSize(4096);
  Serial0.setRxBufferSize(4096);
  Serial.begin(115200);
  Serial0.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  Serial.setTimeout(0);
  Serial0.setTimeout(0);

  const uint32_t wait = millis();
  while (!Serial && millis() - wait < 1200) {
    delay(10);
  }
  Serial.println("SC02-C3 RX GPIO20 PWM GPIO4 USB");
  printStatus();
  lastCtrl = millis();
  lastStatus = lastCtrl;
}

void loop() {
  const int n = Serial0.read(buf, BUF);
  if (n > 0) {
    Serial.write(buf, n);
    const int room = static_cast<int>(sizeof(rxb)) - rxn;
    int take = n;
    const uint8_t* src = buf;
    if (take > room) {
      const int drop = take - room;
      if (drop >= rxn) {
        rxn = 0;
      } else {
        memmove(rxb, rxb + drop, rxn - drop);
        rxn -= drop;
      }
      take = static_cast<int>(sizeof(rxb)) - rxn;
      src = buf + (n - take);
    }
    memcpy(rxb + rxn, src, take);
    rxn += take;
    parseRx();
  }

  while (Serial.available() > 0) {
    handleHost(static_cast<uint8_t>(Serial.read()));
  }

  const uint32_t now = millis();
  control(now);
  if (statusEverySec && now - lastStatus >= 1000) {
    printStatus();
    lastStatus = now;
  }
}
