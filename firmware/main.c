/*
  =====================================================================
   CanSat Flight Software  -  Team 2026-INSPACe-ROCKETRY-037
  =====================================================================
   Board   : Arduino Nano (ATmega328P, CH340 clone, NEW bootloader)
             Arduino IDE -> Board "Arduino Nano", Processor "ATmega328P"
   Sensors : BMP280 (I2C 0x76)  - pressure, temperature, altitude
             MPU6050 (I2C 0x68) - 3-axis accel (+/-16 g), gyro (+/-2000 dps)
   Storage : microSD (SPI, CS = D10) - 10 Hz log, from power-up
   Radio   : RYLR998 LoRa (SoftwareSerial D2/D3 @ 9600) - 1 Hz telemetry,
             only after the ground station sends CX,ON  (guideline 5.2)
   Outputs : Buzzer D5 (via BC547), status LED D6
   Battery : A0 via 20k/10k divider

   Pin map
     D2  <- RYLR998 TXD        D3  -> 10k/20k divider -> RYLR998 RXD
     D5  -> buzzer transistor  D6  -> status LED
     D10 SD CS  D11 MOSI  D12 MISO  D13 SCK
     A4  SDA    A5  SCL     A0  battery sense

   Ground commands (send over LoRa to address 1, or type in Serial Monitor
   at 115200 baud, line ending "Newline"):
     CX,ON    start telemetry          CX,OFF   stop telemetry
     CAL      re-zero altitude (only on the pad)
     RST      start a NEW mission: time/packets to 0, new SD file,
              re-zero altitude (only on the pad or after landing)
     PING     reply with current state
   Every command gets "ACK,<cmd>" or "NAK,<cmd>".

   Launch-site procedure
     1. Power on -> listen for the startup beep (codes below)
     2. Send RST  (clears any old mission kept in EEPROM from bench tests)
     3. After jury clearance send CX,ON -> telemetry starts at 1 Hz
     4. After landing the buzzer + LED beep every 2 s to help recovery

   Startup beep codes
     1 long beep            = all modules OK
     short beeps, repeated per failed module:
       2 = BMP280   3 = MPU6050   4 = SD card   5 = LoRa radio

   Flight software states (guideline 4.5)
     0 BOOT  2 LAUNCH_PAD  3 ASCENT  4 CAN-7USAT_DEPLOY  5 DESCENT  7 IMPACT

   Reset recovery (guidelines 4.5, 5.10)
     State, mission clock, packet count, ground pressure, max altitude
     and SD file number are saved in EEPROM every 5 s and on every state
     change (rotating through ~40 EEPROM slots to avoid wear). After a
     processor reset the CanSat resumes where it was.
     A hardware watchdog resets the board if the code ever hangs.
  =====================================================================
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <math.h>

// ---------------------------------------------------------------------
//  Configuration
// ---------------------------------------------------------------------
const char TEAM_ID[] PROGMEM = "2026-INSPACe-ROCKETRY-037";

// LoRa (allocation: 866.00 MHz main / 867.75 MHz backup, SF8)
#define LORA_BAUD        9600
#define LORA_MY_ADDR     1            // CanSat
#define LORA_GS_ADDR     2            // ground station
#define LORA_NETWORK_ID  7            // RYLR998 accepts only 3-15 or 18
#define LORA_BAND_HZ     866000000UL  // backup: 867750000UL
#define LORA_PARAMS      "8,7,1,12"   // SF8, BW 125 kHz, CR 4/5, preamble 12
#define LORA_POWER_DBM   22

// Timing
#define SAMPLE_MS        100          // 10 Hz sensors + SD log
#define TELEM_MS         1000         // 1 Hz telemetry
#define SAVE_MS          5000         // EEPROM checkpoint
#define FLUSH_MS         1000         // SD flush
#define RADIO_GUARD_MS   700          // wait between LoRa transmissions

// Flight logic
#define LAUNCH_ALT_M     20.0         // above this altitude -> launched
#define LAUNCH_ACC_G     3.0          // or above this acceleration
#define LAUNCH_CONFIRM   5            // consecutive samples (0.5 s)
#define APOGEE_DROP_M    8.0          // this far below max -> deployed
#define APOGEE_CONFIRM   5
#define DEPLOY_HOLD_MS   2000         // DEPLOY state -> DESCENT after this
#define LAND_VSPEED_MS   1.0          // |vertical speed| below this ...
#define LAND_TIME_MS     5000         // ... for this long -> landed
#define MIN_DESCENT_MS   10000        // never "land" sooner than this

// Battery sense
#define VBAT_DIVIDER     3.0          // (20k + 10k) / 10k
#define VBAT_CAL         1.00         // adjust after checking with a multimeter

// Pins
#define PIN_LORA_RX      2
#define PIN_LORA_TX      3
#define PIN_BUZZER       5
#define PIN_LED          6
#define PIN_SD_CS        10
#define PIN_VBAT         A0

#define BMP_ADDR         0x76
#define MPU_ADDR         0x68
#define PERSIST_MAGIC    0xCA37

// ---------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------
enum : uint8_t {
  ST_BOOT = 0, ST_TEST = 1, ST_PAD = 2, ST_ASCENT = 3,
  ST_DEPLOY = 4, ST_DESCENT = 5, ST_AEROBRAKE = 6, ST_IMPACT = 7
};

struct Persist {                // saved in EEPROM
  uint16_t magic;
  uint8_t  state;
  uint8_t  txOn;
  uint32_t missionMs;
  uint32_t packets;
  float    groundPa;
  float    maxAlt;
  uint8_t  fileNo;
  uint32_t seq;                 // save counter (wear levelling)
  uint8_t  check;               // checksum, detects a half-written save
};
Persist P;

SoftwareSerial lora(PIN_LORA_RX, PIN_LORA_TX);
File logFile;

bool okBmp, okMpu, okSd, okLora;

uint32_t missionBase;           // mission time = missionBase + millis()
uint32_t tSample, tTelem, tSave, tFlush, lastTxMs, lastAltMs;
uint32_t stateSinceMs, stillSinceMs;
uint8_t  confirmCnt;
bool     altSeeded;

// Sensor values
float pressPa, tempC, altM, altFilt, vSpeed, vbat;
float ax, ay, az, gx, gy, gz, roll, pitch;

// BMP280 calibration
uint16_t dT1, dP1;
int16_t  dT2, dT3, dP2, dP3, dP4, dP5, dP6, dP7, dP8, dP9;

// Buffers
char buf[160];
uint8_t bn;
char rxLine[64];
uint8_t rxN;
char serLine[24];
uint8_t serN;

inline uint32_t missionTime() { return missionBase + millis(); }

// ---------------------------------------------------------------------
//  Line builder (keeps RAM use low; no String objects)
// ---------------------------------------------------------------------
void bClear() { bn = 0; buf[0] = 0; }
void bStr(const char* s) { while (*s && bn < sizeof(buf) - 1) buf[bn++] = *s++; buf[bn] = 0; }
void bStrP(PGM_P s) { char c; while ((c = pgm_read_byte(s++)) && bn < sizeof(buf) - 1) buf[bn++] = c; buf[bn] = 0; }
void bComma() { bStr(","); }
void bInt(long v) { char t[12]; ltoa(v, t, 10); bStr(t); }
void bFloat(float v, uint8_t dec) { char t[16]; dtostrf(v, 1, dec, t); bStr(t); }

// ---------------------------------------------------------------------
//  I2C helpers
// ---------------------------------------------------------------------
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) out[i] = Wire.read();
  return true;
}

// Copy a flash string to d, return pointer to its terminating 0
char* catP(char* d, PGM_P s) { strcpy_P(d, s); return d + strlen(d); }

inline uint16_t le16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

// ---------------------------------------------------------------------
//  BMP280 (own driver, Bosch datasheet integer compensation)
// ---------------------------------------------------------------------
bool bmpBegin() {
  uint8_t id;
  if (!i2cRead(BMP_ADDR, 0xD0, &id, 1)) return false;
  if (id != 0x58 && id != 0x57 && id != 0x56 && id != 0x60) return false;  // 0x60 = BME280

  uint8_t c[24];
  if (!i2cRead(BMP_ADDR, 0x88, c, 24)) return false;
  dT1 = le16(c + 0);            dT2 = (int16_t)le16(c + 2);  dT3 = (int16_t)le16(c + 4);
  dP1 = le16(c + 6);            dP2 = (int16_t)le16(c + 8);  dP3 = (int16_t)le16(c + 10);
  dP4 = (int16_t)le16(c + 12);  dP5 = (int16_t)le16(c + 14); dP6 = (int16_t)le16(c + 16);
  dP7 = (int16_t)le16(c + 18);  dP8 = (int16_t)le16(c + 20); dP9 = (int16_t)le16(c + 22);

  if (!i2cWrite(BMP_ADDR, 0xF5, 0x08)) return false;  // standby 0.5 ms, IIR filter x4
  if (!i2cWrite(BMP_ADDR, 0xF4, 0x53)) return false;  // temp x2, pressure x8, normal mode
  delay(60);
  return true;
}

bool bmpRead() {
  uint8_t d[6];
  if (!i2cRead(BMP_ADDR, 0xF7, d, 6)) return false;
  int32_t adcP = ((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4);
  int32_t adcT = ((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4);
  if (adcP == 0x80000L || adcT == 0x80000L) return false;   // no measurement yet

  // Temperature
  int32_t v1 = ((((adcT >> 3) - ((int32_t)dT1 << 1))) * ((int32_t)dT2)) >> 11;
  int32_t v2 = (((((adcT >> 4) - ((int32_t)dT1)) * ((adcT >> 4) - ((int32_t)dT1))) >> 12) * ((int32_t)dT3)) >> 14;
  int32_t tFine = v1 + v2;
  tempC = ((tFine * 5 + 128) >> 8) / 100.0f;

  // Pressure (Pa)
  v1 = (tFine >> 1) - 64000L;
  v2 = (((v1 >> 2) * (v1 >> 2)) >> 11) * (int32_t)dP6;
  v2 = v2 + ((v1 * (int32_t)dP5) << 1);
  v2 = (v2 >> 2) + ((int32_t)dP4 << 16);
  v1 = ((((int32_t)dP3 * (((v1 >> 2) * (v1 >> 2)) >> 13)) >> 3) + (((int32_t)dP2 * v1) >> 1)) >> 18;
  v1 = ((32768L + v1) * (int32_t)dP1) >> 15;
  if (v1 == 0) return false;
  uint32_t p = ((uint32_t)(1048576L - adcP) - (v2 >> 12)) * 3125UL;
  if (p < 0x80000000UL) p = (p << 1) / (uint32_t)v1;
  else                  p = (p / (uint32_t)v1) * 2;
  v1 = ((int32_t)dP9 * (int32_t)(((p >> 3) * (p >> 3)) >> 13)) >> 12;
  v2 = ((int32_t)(p >> 2) * (int32_t)dP8) >> 13;
  p = (uint32_t)((int32_t)p + ((v1 + v2 + dP7) >> 4));

  if (p < 30000UL || p > 110000UL) return false;          // sanity check
  pressPa = (float)p;
  return true;
}

// ---------------------------------------------------------------------
//  MPU6050 (own driver)
// ---------------------------------------------------------------------
bool mpuBegin() {
  if (!i2cWrite(MPU_ADDR, 0x6B, 0x01)) return false;  // wake up, gyro clock
  delay(10);
  bool ok = true;
  ok &= i2cWrite(MPU_ADDR, 0x1A, 0x03);  // DLPF ~44 Hz
  ok &= i2cWrite(MPU_ADDR, 0x1B, 0x18);  // gyro  +/-2000 dps (16.4 LSB/dps)
  ok &= i2cWrite(MPU_ADDR, 0x1C, 0x18);  // accel +/-16 g   (2048 LSB/g)
  return ok;
}

bool mpuRead() {
  uint8_t d[14];
  if (!i2cRead(MPU_ADDR, 0x3B, d, 14)) return false;
  const float A = 9.80665f / 2048.0f;
  const float G = 1.0f / 16.4f;
  ax = (int16_t)((d[0] << 8) | d[1]) * A;
  ay = (int16_t)((d[2] << 8) | d[3]) * A;
  az = (int16_t)((d[4] << 8) | d[5]) * A;
  gx = (int16_t)((d[8] << 8) | d[9]) * G;
  gy = (int16_t)((d[10] << 8) | d[11]) * G;
  gz = (int16_t)((d[12] << 8) | d[13]) * G;
  roll  = atan2(ay, az) * 57.29578f;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.29578f;
  return true;
}

// ---------------------------------------------------------------------
//  LoRa (RYLR998)
// ---------------------------------------------------------------------
bool loraWaitOk(uint16_t timeoutMs) {
  char line[24];
  uint8_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (lora.available()) {
      char c = lora.read();
      if (c == '\n') {
        line[n] = 0;
        if (strncmp_P(line, PSTR("+OK"), 3) == 0) return true;
        if (strncmp_P(line, PSTR("+ERR"), 4) == 0) return false;
        n = 0;
      } else if (c != '\r' && n < sizeof(line) - 1) {
        line[n++] = c;
      }
    }
  }
  return false;
}

bool loraCmd(const char* cmd) {
  while (lora.available()) lora.read();
  lora.print(cmd);
  lora.print(F("\r\n"));
  return loraWaitOk(1000);
}

bool loraCmdP(PGM_P cmd) {
  char b[32];
  strncpy_P(b, cmd, sizeof(b) - 1);
  b[sizeof(b) - 1] = 0;
  return loraCmd(b);
}

bool loraBegin() {
  lora.begin(LORA_BAUD);
  delay(200);
  bool ok = false;
  for (uint8_t i = 0; i < 3 && !ok; i++) ok = loraCmdP(PSTR("AT"));
  if (!ok) return false;

  char cmd[32];
  ultoa(LORA_MY_ADDR, catP(cmd, PSTR("AT+ADDRESS=")), 10);      ok &= loraCmd(cmd);
  ultoa(LORA_NETWORK_ID, catP(cmd, PSTR("AT+NETWORKID=")), 10); ok &= loraCmd(cmd);
  ultoa(LORA_BAND_HZ, catP(cmd, PSTR("AT+BAND=")), 10);         ok &= loraCmd(cmd);
  ok &= loraCmdP(PSTR("AT+PARAMETER=" LORA_PARAMS));
  ultoa(LORA_POWER_DBM, catP(cmd, PSTR("AT+CRFOP=")), 10);      ok &= loraCmd(cmd);
  return ok;
}

inline bool radioBusy() { return millis() - lastTxMs < RADIO_GUARD_MS; }

void loraSend(const char* data, uint8_t len) {
  lora.print(F("AT+SEND="));
  lora.print(LORA_GS_ADDR);
  lora.print(',');
  lora.print(len);
  lora.print(',');
  lora.write((const uint8_t*)data, len);
  lora.print(F("\r\n"));
  lastTxMs = millis();
}

// ---------------------------------------------------------------------
//  EEPROM persistence
// ---------------------------------------------------------------------
// Saves rotate through N_SLOTS EEPROM slots so no single cell wears out
// (EEPROM cells survive ~100,000 writes).
#define N_SLOTS (E2END / sizeof(Persist))

uint8_t persistSum(const Persist& p) {
  const uint8_t* b = (const uint8_t*)&p;
  uint8_t s = 0xA5;
  for (uint8_t i = 0; i < sizeof(Persist) - 1; i++) s = (s << 1 | s >> 7) ^ b[i];
  return s;
}

void savePersist() {
  P.missionMs = missionTime();
  P.seq++;
  P.check = persistSum(P);
  EEPROM.put((P.seq % N_SLOTS) * sizeof(Persist), P);
}

bool loadPersist() {            // newest valid slot, false if none
  Persist t;
  bool found = false;
  for (uint16_t i = 0; i < N_SLOTS; i++) {
    EEPROM.get(i * sizeof(Persist), t);
    if (t.magic == PERSIST_MAGIC && t.check == persistSum(t) && (!found || t.seq > P.seq)) {
      P = t;
      found = true;
    }
  }
  return found;
}

void setState(uint8_t s) {
  P.state = s;
  stateSinceMs = millis();
  confirmCnt = 0;
  stillSinceMs = 0;
  savePersist();
  if (okSd) logFile.flush();
}

// ---------------------------------------------------------------------
//  SD card
// ---------------------------------------------------------------------
void makeFileName(char* name, uint8_t no) {
  no %= 100;
  char* e = catP(name, PSTR("CAN37_"));
  *e++ = '0' + no / 10;
  *e++ = '0' + no % 10;
  strcpy_P(e, PSTR(".CSV"));
}

void openLogFile() {
  if (!okSd) return;
  if (logFile) logFile.close();
  char name[13];
  makeFileName(name, P.fileNo);
  logFile = SD.open(name, FILE_WRITE);        // appends if it exists
  if (!logFile) { okSd = false; return; }
  if (logFile.size() == 0) {
    logFile.println(F("TIME_S,STATE,ALT_M,VSPEED_MS,PRESSURE_PA,TEMP_C,VBAT_V,"
                      "ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z,ROLL,PITCH,PACKETS"));
    logFile.flush();
  }
}

void logToSd() {
  if (!okSd) return;
  bClear();
  bFloat(missionTime() / 1000.0f, 1); bComma();
  bInt(P.state);                       bComma();
  bFloat(altFilt, 1);                  bComma();
  bFloat(vSpeed, 1);                   bComma();
  bInt((long)(pressPa + 0.5f));        bComma();
  bFloat(tempC, 1);                    bComma();
  bFloat(vbat, 2);                     bComma();
  bFloat(ax, 2); bComma(); bFloat(ay, 2); bComma(); bFloat(az, 2); bComma();
  bFloat(gx, 1); bComma(); bFloat(gy, 1); bComma(); bFloat(gz, 1); bComma();
  bFloat(roll, 1); bComma(); bFloat(pitch, 1); bComma();
  bInt((long)P.packets);
  if (logFile.println(buf) == 0) okSd = false;
}

// ---------------------------------------------------------------------
//  Ground calibration & missions
// ---------------------------------------------------------------------
void calibrateGround() {
  if (okBmp) {
    float sum = 0;
    uint8_t n = 0;
    for (uint8_t i = 0; i < 20; i++) {
      wdt_reset();
      if (bmpRead()) { sum += pressPa; n++; }
      delay(50);
    }
    if (n) P.groundPa = sum / n;
  }
  if (P.groundPa < 30000 || P.groundPa > 110000) P.groundPa = 101325.0f;
  P.maxAlt = 0;
  altSeeded = false;
  vSpeed = 0;
}

void newMission() {
  missionBase = 0UL - millis();       // mission clock restarts at 0
  P.magic = PERSIST_MAGIC;
  P.packets = 0;
  if (okSd) {                         // next unused file number
    char name[13];
    for (uint8_t i = 0; i < 100; i++) {
      P.fileNo = (P.fileNo + 1) % 100;
      makeFileName(name, P.fileNo);
      if (!SD.exists(name)) break;
    }
  }
  calibrateGround();
  openLogFile();
  setState(ST_PAD);
}

// ---------------------------------------------------------------------
//  Commands (from LoRa or USB serial)
// ---------------------------------------------------------------------
void reply(const char* cmd, bool ok, bool viaRadio) {
  char a[40];
  char* e = catP(a, ok ? PSTR("ACK,") : PSTR("NAK,"));
  strncpy(e, cmd, 24); e[24] = 0; e += strlen(e);
  e = catP(e, PSTR(",ST"));
  utoa(P.state, e, 10);
  Serial.println(a);
  if (viaRadio && okLora) {
    while (radioBusy()) wdt_reset();  // don't collide with a telemetry packet
    loraSend(a, strlen(a));
  }
}

void handleCommand(char* cmd, bool viaRadio) {
  bool onPad = (P.state == ST_PAD || P.state == ST_BOOT);
  bool ok = true;

  if      (strcmp_P(cmd, PSTR("CX,ON")) == 0)  { P.txOn = 1; savePersist(); }
  else if (strcmp_P(cmd, PSTR("CX,OFF")) == 0) { P.txOn = 0; savePersist(); }
  else if (strcmp_P(cmd, PSTR("CAL")) == 0)    { if (onPad) { calibrateGround(); savePersist(); } else ok = false; }
  else if (strcmp_P(cmd, PSTR("RST")) == 0)    { if (onPad || P.state == ST_IMPACT) newMission(); else ok = false; }
  else if (strcmp_P(cmd, PSTR("PING")) == 0)   { }
  else ok = false;

  reply(cmd, ok, viaRadio);
}

// +RCV=<addr>,<len>,<data>,<rssi>,<snr>
void handleRadioLine(char* s) {
  if (strncmp_P(s, PSTR("+RCV="), 5) != 0) return;   // ignore +OK / +ERR
  char* c1 = strchr(s, ',');       if (!c1) return;
  char* c2 = strchr(c1 + 1, ',');  if (!c2) return;
  uint8_t len = atoi(c1 + 1);
  char* data = c2 + 1;
  if (strlen(data) < len) return;
  data[len] = 0;
  handleCommand(data, true);
}

void pollRadio() {
  while (lora.available()) {
    char c = lora.read();
    if (c == '\n') { rxLine[rxN] = 0; handleRadioLine(rxLine); rxN = 0; }
    else if (c != '\r' && rxN < sizeof(rxLine) - 1) rxLine[rxN++] = c;
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serN) { serLine[serN] = 0; handleCommand(serLine, false); serN = 0; }
    } else if (serN < sizeof(serLine) - 1) {
      serLine[serN++] = c;
    }
  }
}

// ---------------------------------------------------------------------
//  Sensing & flight logic
// ---------------------------------------------------------------------
void sampleSensors() {
  uint32_t now = millis();
  if (okBmp && bmpRead()) {
    altM = 44330.0f * (1.0f - pow(pressPa / P.groundPa, 0.190295f));
    if (!altSeeded) {                 // start the filter at the real altitude
      altFilt = altM;
      vSpeed = 0;
      lastAltMs = now;
      altSeeded = true;
    } else {
      float prev = altFilt;
      altFilt += 0.3f * (altM - altFilt);
      float dt = (now - lastAltMs) / 1000.0f;
      if (dt > 0.01f) vSpeed += 0.2f * ((altFilt - prev) / dt - vSpeed);
      lastAltMs = now;
    }
  }
  if (okMpu) mpuRead();
  vbat = analogRead(PIN_VBAT) * (5.0f / 1023.0f) * VBAT_DIVIDER * VBAT_CAL;
}

void updateState() {
  uint32_t now = millis();
  float accG = sqrt(ax * ax + ay * ay + az * az) / 9.80665f;

  switch (P.state) {
    case ST_PAD:
      if (altFilt > LAUNCH_ALT_M || (okMpu && accG > LAUNCH_ACC_G)) {
        if (++confirmCnt >= LAUNCH_CONFIRM) setState(ST_ASCENT);
      } else confirmCnt = 0;
      break;

    case ST_ASCENT:
      if (altFilt > P.maxAlt) P.maxAlt = altFilt;
      if (altFilt < P.maxAlt - APOGEE_DROP_M) {
        if (++confirmCnt >= APOGEE_CONFIRM) setState(ST_DEPLOY);
      } else confirmCnt = 0;
      break;

    case ST_DEPLOY:
      if (now - stateSinceMs > DEPLOY_HOLD_MS) setState(ST_DESCENT);
      break;

    case ST_DESCENT:
      if (fabs(vSpeed) < LAND_VSPEED_MS) {
        if (stillSinceMs == 0) stillSinceMs = now;
        else if (now - stillSinceMs > LAND_TIME_MS && now - stateSinceMs > MIN_DESCENT_MS)
          setState(ST_IMPACT);
      } else stillSinceMs = 0;
      break;

    default:
      break;
  }
}

// ---------------------------------------------------------------------
//  Telemetry (Annexure-2 field order)
// ---------------------------------------------------------------------
//  TEAM_ID, TIME_S, PACKET, ALT_M, PRESSURE_PA, TEMP_C, VOLTAGE_V,
//  GNSS_TIME, GNSS_LAT, GNSS_LON, GNSS_ALT, GNSS_SATS   (0 - no GNSS on CanSat)
//  ACC_X, ACC_Y, ACC_Z (m/s2), ROLL, PITCH (deg),
//  GYRO_X, GYRO_Y, GYRO_Z (deg/s), STATE, VSPEED_MS (optional: descent rate)
void sendTelemetry() {
  P.packets++;
  bClear();
  bStrP(TEAM_ID);                      bComma();
  bFloat(missionTime() / 1000.0f, 1);  bComma();
  bInt((long)P.packets);               bComma();
  bFloat(altFilt, 1);                  bComma();
  bInt((long)(pressPa + 0.5f));        bComma();
  bFloat(tempC, 1);                    bComma();
  bFloat(vbat, 2);                     bComma();
  bStr("0,0.0000,0.0000,0.0,0");       bComma();
  bFloat(ax, 2); bComma(); bFloat(ay, 2); bComma(); bFloat(az, 2); bComma();
  bFloat(roll, 1); bComma(); bFloat(pitch, 1); bComma();
  bFloat(gx, 1); bComma(); bFloat(gy, 1); bComma(); bFloat(gz, 1); bComma();
  bInt(P.state);                       bComma();
  bFloat(vSpeed, 1);

  if (okLora) loraSend(buf, bn);
  Serial.println(buf);
}

// ---------------------------------------------------------------------
//  Buzzer & LED
// ---------------------------------------------------------------------
void beep(uint16_t onMs, uint16_t offMs) {
  digitalWrite(PIN_BUZZER, HIGH); digitalWrite(PIN_LED, HIGH);
  delay(onMs);
  digitalWrite(PIN_BUZZER, LOW);  digitalWrite(PIN_LED, LOW);
  delay(offMs);
}

void startupBeeps() {
  if (okBmp && okMpu && okSd && okLora) { beep(600, 200); return; }
  const bool ok[4] = { okBmp, okMpu, okSd, okLora };
  for (uint8_t m = 0; m < 4; m++) {
    if (ok[m]) continue;
    for (uint8_t i = 0; i < m + 2; i++) beep(120, 180);   // 2..5 short beeps
    delay(700);
  }
}

void updateIndicators() {
  uint32_t now = millis();
  bool led, buzz = false;
  if (P.state == ST_IMPACT) {                 // recovery beacon
    led = buzz = (now % 2000) < 300;
  } else if (P.state == ST_ASCENT || P.state == ST_DEPLOY || P.state == ST_DESCENT) {
    led = (now % 250) < 30;                   // fast flash in flight
  } else {
    led = (now % 1000) < 50;                  // short heartbeat on the pad
  }
  digitalWrite(PIN_LED, led);
  digitalWrite(PIN_BUZZER, buzz);
}

// ---------------------------------------------------------------------
//  Setup & loop
// ---------------------------------------------------------------------
void setup() {
  MCUSR = 0;
  wdt_disable();

  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_LED, OUTPUT);    digitalWrite(PIN_LED, LOW);
  pinMode(PIN_SD_CS, OUTPUT);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);            // never hang on a bad I2C bus

  okBmp  = bmpBegin();
  okMpu  = mpuBegin();
  okSd   = SD.begin(PIN_SD_CS);
  okLora = loraBegin();

  Serial.print(F("BMP280 ")); Serial.println(okBmp  ? F("OK") : F("FAIL"));
  Serial.print(F("MPU6050 "));Serial.println(okMpu  ? F("OK") : F("FAIL"));
  Serial.print(F("SD "));     Serial.println(okSd   ? F("OK") : F("FAIL"));
  Serial.print(F("LoRa "));   Serial.println(okLora ? F("OK") : F("FAIL"));

  if (!loadPersist()) {                       // very first power-up
    memset(&P, 0, sizeof(P));
    newMission();
    Serial.println(F("New mission"));
  } else {                                    // resume after power cycle / reset
    missionBase = P.missionMs - millis();
    if (P.state == ST_BOOT || P.state > ST_IMPACT) P.state = ST_PAD;
    if (P.groundPa < 30000 || P.groundPa > 110000) calibrateGround();
    openLogFile();
    stateSinceMs = millis();
    Serial.print(F("Resumed, state ")); Serial.println(P.state);
  }

  startupBeeps();

  uint32_t now = millis();
  tSample = tTelem = tSave = tFlush = now;
  lastTxMs = now - RADIO_GUARD_MS;

  wdt_enable(WDTO_4S);
}

void loop() {
  wdt_reset();
  pollRadio();
  pollSerial();

  uint32_t now = millis();

  if (now - tSample >= SAMPLE_MS) {
    tSample += SAMPLE_MS;
    if (now - tSample > 5UL * SAMPLE_MS) tSample = now;   // don't burst after a stall
    sampleSensors();
    updateState();
    logToSd();
  }

  if (P.txOn && now - tTelem >= TELEM_MS && !radioBusy()) {
    tTelem += TELEM_MS;
    if (now - tTelem > TELEM_MS) tTelem = now;
    sendTelemetry();
  }

  if (now - tFlush >= FLUSH_MS) { tFlush = now; if (okSd) logFile.flush(); }
  if (now - tSave  >= SAVE_MS)  { tSave  = now; savePersist(); }

  updateIndicators();
}
