#include <SPI.h>
#include "Arduino_GigaDisplay_GFX.h"

// ============================================================================
// CONFIGURACIÓN DE PINES: ARDUINO GIGA R1 <-> LAC <-> ADS1256
// ============================================================================
// Salidas PWM hacia X6.VC de cada tarjeta LAC (3.3V PWM)
const int cmdPins[6] = {2, 3, 4, 5, 6, 7};

// Entradas analógicas de respaldo (si no se detecta el ADS1256)
const int potPins[6] = {A0, A1, A2, A3, A4, A5};

// Invierte por software los ejes que estén cableados al revés (true = invertir 100 - pct)
const bool invertAxis[6] = {true, false, false, false, false, false};

// Pines del módulo ADS1256 conectados al bus SPI5 (Pines D10-D13 del Arduino GIGA R1)
// NOTA IMPORTANTE EN ARDUINO GIGA R1:
// - El objeto 'SPI' usa el conector ICSP de 6 pines (D89/D90/D91).
// - El objeto 'SPI1' usa los pines laterales D11 (COPI/MOSI), D12 (CIPO/MISO), D13 (SCK).
#define ADS_SPI       SPI1
const int ADS_CS_PIN   = 10; // Conectar a CS del ADS1256
const int ADS_DRDY_PIN = 9;  // Conectar a DRDY del ADS1256
const int ADS_RST_PIN  = 8;  // Conectar a RST del ADS1256

// Comandos y registros del ADS1256
#define ADS_CMD_WAKEUP  0x00
#define ADS_CMD_RDATA   0x01
#define ADS_CMD_RREG    0x10
#define ADS_CMD_WREG    0x50
#define ADS_CMD_SELFCAL 0xF0
#define ADS_CMD_SYNC    0xFC
#define ADS_CMD_STANDBY 0xFD
#define ADS_CMD_RESET   0xFE

#define ADS_REG_STATUS  0x00
#define ADS_REG_MUX     0x01
#define ADS_REG_ADCON   0x02
#define ADS_REG_DRATE   0x03

// Voltaje máximo esperado desde X4.P de la tarjeta LAC (P+ = 3.3V)
const float LAC_VREF = 3.30f;

// ============================================================================
// VARIABLES DE ESTADO Y CONTROL
// ============================================================================
int currentPos[6]  = {0, 0, 0, 0, 0, 0};
int setpointPos[6] = {0, 0, 0, 0, 0, 0};
int actualPos[6]   = {0, 0, 0, 0, 0, 0};
const int HOME_PERCENT = 0;

bool ads_available = false;
unsigned long lastAdsRead = 0;
unsigned long lastTelemetrySend = 0;

GigaDisplay_GFX display;

#define BLACK     0x0000
#define WHITE     0xFFFF
#define GREEN     0x07E0
#define CYAN      0x07FF
#define YELLOW    0xFFE0
#define ORANGE    0xFD20
#define GRAY      0x8410
#define DARKGRAY  0x4208
#define PURPLE    0x780F
#define RED       0xF800

bool is_serial_connected = false;
unsigned long last_command_time = 0;
const unsigned long SERIAL_TIMEOUT_MS = 8000;

SPISettings adsSpiSettings(1250000, MSBFIRST, SPI_MODE1); // 1.25 MHz, CPOL=0, CPHA=1

// ============================================================================
// FUNCIONES DEL DRIVER ADS1256
// ============================================================================
bool waitAdsDrdy(unsigned long timeout_us = 15000) {
  unsigned long start = micros();
  while (digitalRead(ADS_DRDY_PIN) == HIGH) {
    if (micros() - start > timeout_us) {
      return false; // Timeout: ADS1256 no responde en DRDY
    }
  }
  return true;
}

void writeAdsReg(uint8_t reg, uint8_t val) {
  ADS_SPI.beginTransaction(adsSpiSettings);
  digitalWrite(ADS_CS_PIN, LOW);
  ADS_SPI.transfer(ADS_CMD_WREG | (reg & 0x0F));
  ADS_SPI.transfer(0x00); // Escribir 1 registro (N - 1 = 0)
  ADS_SPI.transfer(val);
  delayMicroseconds(2);
  digitalWrite(ADS_CS_PIN, HIGH);
  ADS_SPI.endTransaction();
}

uint8_t readAdsReg(uint8_t reg) {
  ADS_SPI.beginTransaction(adsSpiSettings);
  digitalWrite(ADS_CS_PIN, LOW);
  ADS_SPI.transfer(ADS_CMD_RREG | (reg & 0x0F));
  ADS_SPI.transfer(0x00); // Leer 1 registro
  delayMicroseconds(10);  // t6 delay >= 6.5 us requerido por datasheet
  uint8_t val = ADS_SPI.transfer(0x00);
  delayMicroseconds(2);
  digitalWrite(ADS_CS_PIN, HIGH);
  ADS_SPI.endTransaction();
  return val;
}

bool initADS1256() {
  pinMode(ADS_CS_PIN, OUTPUT);
  digitalWrite(ADS_CS_PIN, HIGH);
  pinMode(ADS_DRDY_PIN, INPUT_PULLUP);
  pinMode(ADS_RST_PIN, OUTPUT);

  // Reset por hardware del ADS1256
  digitalWrite(ADS_RST_PIN, LOW);
  delay(5);
  digitalWrite(ADS_RST_PIN, HIGH);
  delay(20);

  ADS_SPI.begin();

  if (!waitAdsDrdy(50000)) {
    return false;
  }

  // Reset por comando SPI
  ADS_SPI.beginTransaction(adsSpiSettings);
  digitalWrite(ADS_CS_PIN, LOW);
  ADS_SPI.transfer(ADS_CMD_RESET);
  delayMicroseconds(10);
  digitalWrite(ADS_CS_PIN, HIGH);
  ADS_SPI.endTransaction();
  delay(10);

  if (!waitAdsDrdy(50000)) {
    return false;
  }

  // Configurar registros:
  // STATUS = 0x00 (MSB first, ACAL off, BUFEN = 0 para permitir lectura de 0 a 3.3V sin saturar a 3.0V)
  writeAdsReg(ADS_REG_STATUS, 0x00);
  // ADCON = 0x00 (CLKOUT off, Sensor Detect off, PGA = 1 -> rango +-5V)
  writeAdsReg(ADS_REG_ADCON, 0x00);
  // DRATE = 0x92 (500 SPS) o 0xA1 (1000 SPS)
  writeAdsReg(ADS_REG_DRATE, 0x92);

  // Verificar comunicación leyendo el registro DRATE
  uint8_t drate_readback = readAdsReg(ADS_REG_DRATE);
  if (drate_readback != 0x92) {
    return false;
  }

  // Autocalibración interna (SELFCAL)
  ADS_SPI.beginTransaction(adsSpiSettings);
  digitalWrite(ADS_CS_PIN, LOW);
  ADS_SPI.transfer(ADS_CMD_SELFCAL);
  delayMicroseconds(10);
  digitalWrite(ADS_CS_PIN, HIGH);
  ADS_SPI.endTransaction();

  waitAdsDrdy(100000);
  return true;
}

int readAdsChannelPercent(int channel) {
  if (!waitAdsDrdy(10000)) {
    return actualPos[channel]; // Mantener último valor válido si hay timeout
  }

  uint8_t mux = ((channel & 0x07) << 4) | 0x08; // Positivo = AINx, Negativo = AINCOM (0x08)

  ADS_SPI.beginTransaction(adsSpiSettings);
  digitalWrite(ADS_CS_PIN, LOW);

  // 1. Seleccionar canal en registro MUX
  ADS_SPI.transfer(ADS_CMD_WREG | ADS_REG_MUX);
  ADS_SPI.transfer(0x00);
  ADS_SPI.transfer(mux);
  delayMicroseconds(4);

  // 2. Sincronizar y despertar para iniciar conversión del nuevo canal
  ADS_SPI.transfer(ADS_CMD_SYNC);
  delayMicroseconds(5);
  ADS_SPI.transfer(ADS_CMD_WAKEUP);
  digitalWrite(ADS_CS_PIN, HIGH);
  ADS_SPI.endTransaction();

  // 3. Esperar a que termine la conversión del canal seleccionado
  if (!waitAdsDrdy(10000)) {
    return actualPos[channel];
  }

  // 4. Leer los 24 bits de la conversión
  ADS_SPI.beginTransaction(adsSpiSettings);
  digitalWrite(ADS_CS_PIN, LOW);
  ADS_SPI.transfer(ADS_CMD_RDATA);
  delayMicroseconds(10); // t6 delay >= 6.5 us

  uint8_t b0 = ADS_SPI.transfer(0x00);
  uint8_t b1 = ADS_SPI.transfer(0x00);
  uint8_t b2 = ADS_SPI.transfer(0x00);
  digitalWrite(ADS_CS_PIN, HIGH);
  ADS_SPI.endTransaction();

  // Extensión de signo de 24 bits a 32 bits
  int32_t raw = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | (int32_t)b2;
  if (raw & 0x800000) {
    raw |= 0xFF000000;
  }

  // Con Vref = 2.5V y PGA = 1, Full Scale = +5.0V = 8388607 (0x7FFFFF)
  float voltage = (float)raw * (5.0f / 8388607.0f);
  int pct = (int)roundf((voltage / LAC_VREF) * 100.0f);
  pct = constrain(pct, 0, 100);

  if (invertAxis[channel]) {
    pct = 100 - pct;
  }
  return pct;
}

// ============================================================================
// LECTURA DE SENSORES Y ESCRITURA A TARJETAS LAC
// ============================================================================
void writeLac(int index) {
  int duty = map(setpointPos[index], 0, 100, 0, 255);
  analogWrite(cmdPins[index], duty);
}

int readPotFallback(int index) {
  long acc = 0;
  for (int k = 0; k < 8; k++) acc += analogRead(potPins[index]);
  int pct = (int)((acc / 8) * 100L / 65535L);
  pct = constrain(pct, 0, 100);
  if (invertAxis[index]) pct = 100 - pct;
  return pct;
}

void readActualPositions() {
  for (int i = 0; i < 6; i++) {
    if (ads_available) {
      actualPos[i] = readAdsChannelPercent(i);
    } else {
      actualPos[i] = readPotFallback(i);
    }
  }
}

void sendTelemetryPacket() {
  Serial.print("TEL ");
  for (int i = 0; i < 6; i++) {
    Serial.print(actualPos[i]);
    if (i < 5) Serial.print(",");
  }
  Serial.println();
}

// ============================================================================
// SETUP Y LOOP PRINCIPAL
// ============================================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(16);

  display.begin();
  display.setRotation(1);
  display.fillScreen(BLACK);

  // Intentar inicializar ADS1256; si no responde, usa ADC interno A0..A5 como respaldo
  ads_available = initADS1256();

  readActualPositions();
  for (int i = 0; i < 6; i++) {
    setpointPos[i] = actualPos[i];
    currentPos[i] = actualPos[i];
    pinMode(cmdPins[i], OUTPUT);
    writeLac(i);
  }

  drawStaticUI();
  updateAllBars();
  drawConnectionLed(false);

  Serial.println("Sistema listo");
  Serial.println(ads_available ? "ADC: ADS1256 (24-bit SPI1 activo)" : "ADC: Interno GIGA A0-A5 (ADS1256 no detectado)");
  Serial.println("Comandos: a1 30 | all 60 | home | ping | pos ... | getpos");
}

void loop() {
  while (Serial.available()) {
    String cmdLine = Serial.readStringUntil('\n');
    cmdLine.trim();
    if (cmdLine.length() > 0) {
      processCommand(cmdLine);
      setConnectionStatus(true);
      last_command_time = millis();
    }
  }

  if (is_serial_connected && (millis() - last_command_time > SERIAL_TIMEOUT_MS)) {
    setConnectionStatus(false);
  }

  // Lectura de sensores y envío de telemetría a la PC cada 100 ms (10 Hz)
  if (millis() - lastAdsRead >= 100) {
    lastAdsRead = millis();
    readActualPositions();
    for (int i = 0; i < 6; i++) {
      if (abs(currentPos[i] - actualPos[i]) >= 1) {
        currentPos[i] = actualPos[i];
        updateCurrentBar(i);
      }
    }

    // Enviar telemetría continua a la PC mientras la sesión serial esté activa
    if (is_serial_connected && (millis() - lastTelemetrySend >= 100)) {
      lastTelemetrySend = millis();
      sendTelemetryPacket();
    }
  }
}

// ============================================================================
// PROCESAMIENTO DE COMANDOS SERIALES
// ============================================================================
void applySetpoint(int index, int percent, bool verbose) {
  int newPos = constrain(percent, 0, 100);
  if (setpointPos[index] != newPos) {
    setpointPos[index] = newPos;
    writeLac(index);
    updateSetpointBar(index);
  }
  if (verbose) {
    Serial.print("S");
    Serial.print(index + 1);
    Serial.print("=");
    Serial.println(setpointPos[index]);
  }
}

void processCommand(String cmdLine) {
  cmdLine.toLowerCase();

  if (cmdLine == "ping") {
    Serial.println("pong");
    return;
  }

  if (cmdLine == "getpos") {
    readActualPositions();
    Serial.print("pos ");
    for (int i = 0; i < 6; i++) {
      Serial.print(actualPos[i]);
      if (i < 5) Serial.print(",");
    }
    Serial.println();
    return;
  }

  if (cmdLine.startsWith("pos ")) {
    parseMultiplePositions(cmdLine.substring(4));
  } else if (cmdLine.startsWith("a") && cmdLine.length() >= 4 && cmdLine.charAt(1) >= '1' && cmdLine.charAt(1) <= '6') {
    int actuator = cmdLine.substring(1, 2).toInt() - 1;
    int space = cmdLine.indexOf(' ');
    if (space > 0 && actuator >= 0 && actuator < 6) {
      applySetpoint(actuator, cmdLine.substring(space + 1).toInt(), true);
    }
  } else if (cmdLine.startsWith("all ")) {
    int value = cmdLine.substring(4).toInt();
    for (int i = 0; i < 6; i++) applySetpoint(i, value, false);
    Serial.println("ok all");
  } else if (cmdLine == "home") {
    for (int i = 0; i < 6; i++) applySetpoint(i, HOME_PERCENT, false);
    Serial.println("ok home");
  } else if (cmdLine == "estop") {
    readActualPositions();
    for (int i = 0; i < 6; i++) {
      applySetpoint(i, actualPos[i], false);
    }
    Serial.println("ok estop");
  } else if (cmdLine == "reset_safe") {
    Serial.println("ok reset_safe");
  } else if (cmdLine == "adsinit") {
    ads_available = initADS1256();
    Serial.println(ads_available ? "ok adsinit (ADS1256 24-bit activo)" : "WARN: ADS1256 no detectado (usando A0-A5)");
  } else if (cmdLine == "calibrate") {
    ads_available = initADS1256();
    Serial.println(ads_available ? "ok calibrate (ADS1256 OK)" : "ok calibrate (ADC interno)");
  } else if (cmdLine == "zero") {
    Serial.println("ok zero");
  }
}

void parseMultiplePositions(String data) {
  data.trim();
  int values[6] = {0};
  int index = 0;
  int start = 0;

  for (int i = 0; i <= (int)data.length() && index < 6; i++) {
    if (i == (int)data.length() || data.charAt(i) == ',') {
      String token = data.substring(start, i);
      token.trim();
      values[index] = constrain(token.toInt(), 0, 100);
      index++;
      start = i + 1;
    }
  }

  if (index == 6) {
    for (int i = 0; i < 6; i++) applySetpoint(i, values[i], false);
    Serial.print("ok pos ");
    for (int i = 0; i < 6; i++) {
      Serial.print(setpointPos[i]);
      if (i < 5) Serial.print(",");
    }
    Serial.println();
  } else {
    Serial.print("ERROR: llegaron ");
    Serial.print(index);
    Serial.println(" valores (se esperaban 6)");
  }
}

// ============================================================================
// INTERFAZ GRÁFICA EN GIGA DISPLAY SHIELD
// ============================================================================
void setConnectionStatus(bool status) {
  if (is_serial_connected != status) {
    is_serial_connected = status;
    drawConnectionLed(is_serial_connected);
    Serial.println(status ? "LED: CONECTADO" : "LED: DESCONECTADO");
  }
}

void drawConnectionLed(bool connected) {
  display.fillCircle(25, 22, 10, BLACK);
  display.fillCircle(25, 22, 8, connected ? GREEN : RED);
  display.drawCircle(25, 22, 8, WHITE);
}

void drawStaticUI() {
  display.setTextColor(CYAN);
  display.setTextSize(3);
  display.setCursor(50, 12);
  display.print("GOUGH-STEWART PLATFORM");

  display.drawFastHLine(10, 45, 780, GRAY);
  display.drawFastVLine(400, 45, 390, GRAY);

  display.setTextSize(2);
  display.setTextColor(YELLOW);
  display.setCursor(120, 55);
  display.print("ACTUAL POS");
  display.setCursor(530, 55);
  display.print("SETPOINTS");

  display.drawFastHLine(10, 80, 780, GRAY);

  display.setTextSize(2);
  display.setTextColor(WHITE);
  for (int i = 0; i < 6; i++) {
    int y = 95 + i * 55;
    display.setCursor(10, y + 8);
    display.print("A");
    display.print(i + 1);
    display.setCursor(410, y + 8);
    display.print("S");
    display.print(i + 1);
  }

  display.drawFastHLine(10, 430, 780, GRAY);
  display.setTextColor(YELLOW);
  display.setTextSize(1);
  display.setCursor(10, 450);
  display.print(ads_available ? "ADC: ADS1256 24-bit SPI1 | " : "ADC: INTERNO A0-A5 | ");
  display.print("Cmds: a1 30 | all 60 | home | ping | pos | getpos");
}

void updateAllBars() {
  for (int i = 0; i < 6; i++) {
    updateCurrentBar(i);
    updateSetpointBar(i);
  }
}

void updateCurrentBar(int index) {
  int y = 95 + index * 55;
  int barX = 50;
  int barWidth = 260;
  int barHeight = 28;

  display.fillRoundRect(barX, y, barWidth, barHeight, 4, DARKGRAY);
  int fillWidth = map(currentPos[index], 0, 100, 0, barWidth - 4);

  uint16_t color;
  if (currentPos[index] <= 15 || currentPos[index] >= 85) color = ORANGE;
  else if (currentPos[index] >= 40 && currentPos[index] <= 60) color = GREEN;
  else color = CYAN;

  if (fillWidth > 0) {
    display.fillRoundRect(barX + 2, y + 2, fillWidth, barHeight - 4, 3, color);
  }

  display.fillRect(320, y, 70, 30, BLACK);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(325, y + 6);
  if (currentPos[index] < 10) display.print("  ");
  else if (currentPos[index] < 100) display.print(" ");
  display.print(currentPos[index]);
  display.print("%");
}

void updateSetpointBar(int index) {
  int y = 95 + index * 55;
  int barX = 450;
  int barWidth = 260;
  int barHeight = 28;

  display.fillRoundRect(barX, y, barWidth, barHeight, 4, DARKGRAY);
  int fillWidth = map(setpointPos[index], 0, 100, 0, barWidth - 4);

  if (fillWidth > 0) {
    display.fillRoundRect(barX + 2, y + 2, fillWidth, barHeight - 4, 3, PURPLE);
  }

  display.fillRect(720, y, 70, 30, BLACK);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(725, y + 6);
  if (setpointPos[index] < 10) display.print("  ");
  else if (setpointPos[index] < 100) display.print(" ");
  display.print(setpointPos[index]);
  display.print("%");
}
