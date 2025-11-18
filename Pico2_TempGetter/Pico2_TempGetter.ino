// Raspberry Pi Pico 2 + MCP7940N + BME280 + 4x ADS1115 + 2x 128x64 OLEDs
// Right OLED: ADS1115 data (J3-J18), 8 jacks per page, cycling every 2 seconds
// Left OLED: BME280 environment data
// Based on Kevin Darrah's MCP7940N demo, adapted for RP2040

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include <time.h>

// -------------------- Pins --------------------
const uint8_t RTC_MFP_PIN = 2;    // optional: MCP7940N MFP -> GP2

// -------------------- OLED Config (2x 128x64) --------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET   -1
#define OLED1_ADDR   0x3C        // right
#define OLED2_ADDR   0x3D        // left

Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -------------------- BME280 Config --------------------
#define BME_ADDR 0x77
Adafruit_BME280 bme;
bool bme_ok = false;

// -------------------- ADS1115 Config (4 devices) --------------------
Adafruit_ADS1115 ads0;
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
Adafruit_ADS1115 ads3;
bool ads_ok[4] = {false, false, false, false};

// -------------------- MCP7940N Address & Registers --------------------
#define RTCADDR 0x6F  // MCP7940N

#define RTCSEC    0x00
#define RTCMIN    0x01
#define RTCHOUR   0x02
#define RTCWKDAY  0x03
#define RTCDATE   0x04
#define RTCMTH    0x05
#define RTCYEAR   0x06
#define CONTROL   0x07
#define OSCTRIM   0x08
#define ALM0SEC   0x0A
#define ALM0MIN   0x0B
#define ALM0HOUR  0x0C
#define ALM0WKDAY 0x0D
#define ALM0DATE  0x0E
#define ALM0MTH   0x0F

// -------------------- Timekeeping Variables --------------------
byte rtcSeconds, rtcMinutes, rtcHours;
byte rtcWeekDay, rtcDay, rtcMonth, rtcYear;
boolean rtc12hrMode, rtcPM, rtcOscRunning, rtcPowerFail, rtcVbatEn;
String weekDayStr[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
boolean mfpPinTriggered = false;

// --------------- Page Flipping (right OLED) --------------------
const uint8_t TOTAL_JACKS    = 16;   // J3..J18
const uint8_t JACKS_PER_PAGE = 8;    // show 8 per screen
const unsigned long PAGE_MS  = 2000; // 2 seconds per page

unsigned long lastPageSwitch = 0;
uint8_t currentPage          = 0;    // 0 or 1

// ---------------- ADC Conversion (ADS1115, GAIN_TWOTHIRDS) ----------------
// ---------------- ADC + Thermistor conversion ----------------
// ADS1115 @ GAIN_TWOTHIRDS: 0.1875 mV per bit, ±6.144V
float countsToVolts(int16_t counts) {
  return counts * 0.1875f / 1000.0f;   // -> volts
}

// 10k NTC, Beta = 3435K, 10k fixed resistor, 3.3V supply
const float V_SUPPLY   = 3.3f;
const float R_FIXED    = 10000.0f;   // 10k series resistor
const float TH_BETA    = 3435.0f;    // 3435K thermistor
const float TH_R0      = 10000.0f;   // 10k at 25°C
const float TH_T0_K    = 298.15f;    // 25°C in Kelvin

float countsToTempC(int16_t counts) {
  float v = countsToVolts(counts);
  if (v <= 0.0f || v >= V_SUPPLY) {
    return NAN;  // out of range or not connected
  }

  // Voltage divider: Vout = V_SUPPLY * (R_therm / (R_FIXED + R_therm))
  // => R_therm = R_FIXED * Vout / (V_SUPPLY - Vout)
  float rTherm = R_FIXED * v / (V_SUPPLY - v);

  // Beta equation
  float invT = 1.0f / TH_T0_K + (1.0f / TH_BETA) * log(rTherm / TH_R0);
  float tempK = 1.0f / invT;
  return tempK - 273.15f;   // Kelvin -> °C
}

// -------------------- Function Prototypes --------------------
void rtcInit();
void rtcGetTime();
void checkSerial();
void checkAlarm();
void rtcMFP();
void updateDisplays(float tempC, float humidity, float pressure_hPa,
                    float adcVals[16]);
uint32_t rtcToEpoch();

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();           // default I2C pins for Pico 2 board def
  Wire.setClock(100000);

  Serial.println("Pico 2 MCP7940N + BME280 + ADS1115 + OLEDs");

  // MCP7940N interrupt pin (optional)
  pinMode(RTC_MFP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_MFP_PIN), rtcMFP, FALLING);

  // Initialize OLEDs
  if (!display1.begin(SSD1306_SWITCHCAPVCC, OLED1_ADDR)) {
    Serial.println("SSD1306 #1 (right) allocation failed");
  } else {
    display1.clearDisplay();
    display1.setTextSize(1);
    display1.setTextColor(SSD1306_WHITE);
    display1.setCursor(0, 0);
    display1.println("RIGHT OLED OK");
    display1.display();
  }

  if (!display2.begin(SSD1306_SWITCHCAPVCC, OLED2_ADDR)) {
    Serial.println("SSD1306 #2 (left) allocation failed");
  } else {
    display2.clearDisplay();
    display2.setTextSize(1);
    display2.setTextColor(SSD1306_WHITE);
    display2.setCursor(0, 0);
    display2.println("LEFT OLED OK");
    display2.display();
  }

  // Initialize BME280
  if (bme.begin(BME_ADDR)) {
    bme_ok = true;
    Serial.println("BME280 OK");
  } else {
    Serial.println("BME280 NOT FOUND");
  }

  // Initialize ADS1115s
  ads_ok[0] = ads0.begin(0x48);
  ads_ok[1] = ads1.begin(0x49);
  ads_ok[2] = ads2.begin(0x4A);
  ads_ok[3] = ads3.begin(0x4B);

  for (int i = 0; i < 4; i++) {
    if (ads_ok[i]) {
      Serial.print("ADS1115 #");
      Serial.print(i);
      Serial.println(" OK");
    } else {
      Serial.print("ADS1115 #");
      Serial.print(i);
      Serial.println(" NOT FOUND");
    }
  }

  // Gain settings (all the same here, adjust if you need different ranges)
  if (ads_ok[0]) ads0.setGain(GAIN_TWOTHIRDS);
  if (ads_ok[1]) ads1.setGain(GAIN_TWOTHIRDS);
  if (ads_ok[2]) ads2.setGain(GAIN_TWOTHIRDS);
  if (ads_ok[3]) ads3.setGain(GAIN_TWOTHIRDS);

  // Init RTC
  rtcInit();

  delay(500);
}

// ============================================================
// RTC → Epoch
// ============================================================
uint32_t rtcToEpoch() {
  struct tm t;

  t.tm_year = 2000 + rtcYear - 1900;  // struct tm year = years since 1900
  t.tm_mon  = rtcMonth - 1;           // struct tm month = 0–11
  t.tm_mday = rtcDay;
  t.tm_hour = rtcHours;
  t.tm_min  = rtcMinutes;
  t.tm_sec  = rtcSeconds;
  t.tm_isdst = 0;                     // no DST

  time_t epoch = mktime(&t);          // convert to epoch seconds
  return (uint32_t)epoch;
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // Get time from MCP7940N
  rtcGetTime();

  // Read BME280
  float tempC = NAN;
  float humidity = NAN;
  float pressure_hPa = NAN;
  if (bme_ok) {
    tempC = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure_hPa = bme.readPressure() / 100.0F; // Pa → hPa
  }

  // Read ALL ADS1115 channels (4 devices × 4 channels = 16)
  float adcVals[16];
  for (int i = 0; i < 16; i++) {
    adcVals[i] = NAN;  // default: not available
  }

  // ADS0 -> J3..J6
if (ads_ok[0]) {
  for (int ch = 0; ch < 4; ch++) {
    int16_t raw = ads0.readADC_SingleEnded(ch);
    adcVals[ch] = countsToTempC(raw);        // °C
  }
}

// ADS1 -> J7..J10
if (ads_ok[1]) {
  for (int ch = 0; ch < 4; ch++) {
    int16_t raw = ads1.readADC_SingleEnded(ch);
    adcVals[4 + ch] = countsToTempC(raw);    // °C
  }
}

// ADS2 -> J11..J14
if (ads_ok[2]) {
  for (int ch = 0; ch < 4; ch++) {
    int16_t raw = ads2.readADC_SingleEnded(ch);
    adcVals[8 + ch] = countsToTempC(raw);    // °C
  }
}

// ADS3 -> J15..J18
if (ads_ok[3]) {
  for (int ch = 0; ch < 4; ch++) {
    int16_t raw = ads3.readADC_SingleEnded(ch);
    adcVals[12 + ch] = countsToTempC(raw);   // °C
  }
}


  // Update both OLEDs (right paging, left env)
  updateDisplays(tempC, humidity, pressure_hPa, adcVals);

  // Handle serial commands ("T..." to set time, etc.)
  checkSerial();

  // Check for alarm interrupt (if using MCP alarms)
  checkAlarm();

  delay(200);  // small delay; page timing uses millis(), not this delay
}

// ============================================================
// RTC INIT
// ============================================================
void rtcInit() {
  // Clear CONTROL register
  Wire.beginTransmission(RTCADDR);
  Wire.write(CONTROL);
  Wire.write(B00000000);
  Wire.endTransmission();

  // Enable battery backup (VBATEN bit in weekday register)
  Wire.beginTransmission(RTCADDR);
  Wire.write(RTCWKDAY);
  Wire.endTransmission();
  Wire.requestFrom(RTCADDR, 1);
  delay(1);
  byte rtcWeekdayRegister = Wire.read();
  rtcWeekdayRegister |= 0x08;  // enable VBAT
  Wire.beginTransmission(RTCADDR);
  Wire.write(RTCWKDAY);
  Wire.write(rtcWeekdayRegister);
  Wire.endTransmission();

  // Start oscillator if not started
  Wire.beginTransmission(RTCADDR);
  Wire.write(RTCSEC);
  Wire.endTransmission();
  Wire.requestFrom(RTCADDR, 1);
  delay(1);
  byte rtcSecondRegister = Wire.read();
  rtcSecondRegister |= 0x80;  // ST bit
  Wire.beginTransmission(RTCADDR);
  Wire.write(RTCSEC);
  Wire.write(rtcSecondRegister);
  Wire.endTransmission();
}

// ============================================================
// RTC GET TIME
// ============================================================
void rtcGetTime() {
  Wire.beginTransmission(RTCADDR);
  Wire.write(RTCSEC);
  Wire.endTransmission();
  Wire.requestFrom(RTCADDR, 7);
  delay(1);

  rtcSeconds = Wire.read() & 0x7F;
  rtcMinutes = Wire.read() & 0x7F;
  rtcHours   = Wire.read() & 0x7F;
  rtcWeekDay = Wire.read() & 0x3F;
  rtcDay     = Wire.read() & 0x3F;
  rtcMonth   = Wire.read() & 0x3F;
  rtcYear    = Wire.read();

  // BCD → decimal
  rtcSeconds = (rtcSeconds >> 4) * 10 + (rtcSeconds & 0x0F);
  rtcMinutes = (rtcMinutes >> 4) * 10 + (rtcMinutes & 0x0F);

  if ((rtcHours >> 6) == 1)
    rtc12hrMode = true;
  else
    rtc12hrMode = false;

  if (rtc12hrMode) {
    if ((rtcHours >> 5) & 0x01 == 1)
      rtcPM = true;
    else
      rtcPM = false;
    rtcHours = ((rtcHours >> 4) & 0x01) * 10 + (rtcHours & 0x0F);
  } else {
    rtcPM = false;
    rtcHours = ((rtcHours >> 4) & 0x03) * 10 + (rtcHours & 0x0F);
  }

  // weekday flags
  if ((rtcWeekDay >> 5) & 0x01 == 1)
    rtcOscRunning = true;
  else
    rtcOscRunning = false;

  if ((rtcWeekDay >> 4) & 0x01 == 1)
    rtcPowerFail = true;
  else
    rtcPowerFail = false;

  if ((rtcWeekDay >> 3) & 0x01 == 1)
    rtcVbatEn = true;
  else
    rtcVbatEn = false;

  rtcWeekDay = rtcWeekDay & 0x07;

  // Date
  rtcDay   = (rtcDay >> 4) * 10 + (rtcDay & 0x0F);
  rtcMonth = ((rtcMonth >> 4) & 0x01) * 10 + (rtcMonth & 0x0F);
  rtcYear  = (rtcYear >> 4) * 10 + (rtcYear & 0x0F);

  // Print to Serial (optional)
  Serial.print(rtcHours);
  Serial.print(":");
  Serial.print(rtcMinutes);
  Serial.print(":");
  Serial.print(rtcSeconds);

  if (rtc12hrMode == true && rtcPM == true)
    Serial.print(" PM ");
  else if (rtc12hrMode == true && rtcPM == false)
    Serial.print(" AM ");

  if (rtc12hrMode == false)
    Serial.print(" 24hr ");

  Serial.print(weekDayStr[rtcWeekDay - 1]);
  Serial.print(" ");
  Serial.print(rtcMonth);
  Serial.print("/");
  Serial.print(rtcDay);
  Serial.print("/");
  Serial.print(rtcYear);
  Serial.println("");
}

// ============================================================
// SERIAL COMMAND HANDLER (time set + alarm – same as original style)
// ============================================================
void checkSerial() {
  byte rtcNewHour, rtcNewMinute, rtcNewSecond;
  byte rtcNewMonth, rtcNewDay, rtcNewYear, rtcNewWeekDay;
  byte rtcNew12hrMode, rtcNewPM;
  byte rtcAlarm0Sec, rtcAlarm0Min, rtcAlarm0Hour, rtcAlarm0mask;
  byte rtcAlarmWeekday, rtcAlarmDay, rtcAlarmMonth;

  if (Serial.available()) {
    String fromSerial = "Thh:mm:22 ab mm/dd/yr w";
    fromSerial = Serial.readStringUntil('\n');
    Serial.println(fromSerial);

    // ----- SET TIME -----
    if (fromSerial[0] == 'T' && fromSerial.length() == 23) {
      rtcNewHour   = (((fromSerial[1] - 48) << 4) & 0xF0) + ((fromSerial[2] - 48) & 0x0F);
      rtcNewMinute = (((fromSerial[4] - 48) << 4) & 0xF0) + ((fromSerial[5] - 48) & 0x0F);
      rtcNewSecond = (((fromSerial[7] - 48) << 4) & 0xF0) + ((fromSerial[8] - 48) & 0x0F);
      rtcNew12hrMode = fromSerial[10] - 48;
      rtcNewHour |= (rtcNew12hrMode << 6);
      rtcNewPM = fromSerial[11] - 48;
      rtcNewHour |= (rtcNewPM << 5);
      rtcNewMonth = (((fromSerial[13] - 48) << 4) & 0xF0) + ((fromSerial[14] - 48) & 0x0F);
      rtcNewDay   = (((fromSerial[16] - 48) << 4) & 0xF0) + ((fromSerial[17] - 48) & 0x0F);
      rtcNewYear  = (((fromSerial[19] - 48) << 4) & 0xF0) + ((fromSerial[20] - 48) & 0x0F);
      rtcNewWeekDay = (fromSerial[22] - 48) | (1 << 3); // keep VBAT enabled

      // Stop clock
      Wire.beginTransmission(RTCADDR);
      Wire.write(RTCSEC);
      Wire.write(0x00);
      Wire.endTransmission();

      rtcGetTime(); // verify osc stopped

      if (rtcOscRunning == false) {
        Serial.println("RTC has stopped - Changing Time");
        Wire.beginTransmission(RTCADDR);
        Wire.write(RTCSEC);
        Wire.write(rtcNewSecond);
        Wire.write(rtcNewMinute);
        Wire.write(rtcNewHour);
        Wire.write(rtcNewWeekDay);
        Wire.write(rtcNewDay);
        Wire.write(rtcNewMonth);
        Wire.write(rtcNewYear);
        Wire.endTransmission();

        // Restart oscillator
        Wire.beginTransmission(RTCADDR);
        Wire.write(RTCSEC);
        Wire.endTransmission();
        Wire.requestFrom(RTCADDR, 1);
        delay(1);
        byte rtcSecondRegister = Wire.read();
        rtcSecondRegister |= 0x80;
        Wire.beginTransmission(RTCADDR);
        Wire.write(RTCSEC);
        Wire.write(rtcSecondRegister);
        Wire.endTransmission();
      } else {
        return;
      }
    }

    // ----- SET ALARM 0 -----
    if (fromSerial[0] == 'A' && fromSerial[1] == '0' && fromSerial.length() == 24) {
      Serial.println("Setting Alarm");
      rtcAlarm0Hour = ((fromSerial[15] - 48) << 5) +
                      (((fromSerial[2] - 48) << 4) & 0xF0) +
                      ((fromSerial[3] - 48) & 0x0F);
      rtcAlarm0Min  = (((fromSerial[5] - 48) << 4) & 0xF0) + ((fromSerial[6] - 48) & 0x0F);
      rtcAlarm0Sec  = (((fromSerial[8] - 48) << 4) & 0xF0) + ((fromSerial[9] - 48) & 0x0F);
      rtcAlarm0mask = ((fromSerial[11] - 48) << 6) +
                      ((fromSerial[12] - 48) << 5) +
                      ((fromSerial[13] - 48) << 4);
      rtcAlarmMonth = (((fromSerial[17] - 48) << 4) & 0xF0) + ((fromSerial[18] - 48) & 0x0F);
      rtcAlarmDay   = (((fromSerial[20] - 48) << 4) & 0xF0) + ((fromSerial[21] - 48) & 0x0F);
      rtcAlarmWeekday = (fromSerial[23] - 48) + rtcAlarm0mask;

      Wire.beginTransmission(RTCADDR);
      Wire.write(ALM0SEC);
      Wire.write(rtcAlarm0Sec);
      Wire.write(rtcAlarm0Min);
      Wire.write(rtcAlarm0Hour);
      Wire.write(rtcAlarmWeekday);
      Wire.write(rtcAlarmDay);
      Wire.write(rtcAlarmMonth);
      Wire.endTransmission();

      Wire.beginTransmission(RTCADDR);
      Wire.write(CONTROL);
      Wire.endTransmission();
      Wire.requestFrom(RTCADDR, 1);
      delay(1);
      byte rtcControlRegister = Wire.read();
      rtcControlRegister |= 0x10; // enable alm0
      Wire.beginTransmission(RTCADDR);
      Wire.write(CONTROL);
      Wire.write(rtcControlRegister);
      Wire.endTransmission();
    }
  }
}

// ============================================================
// ALARM CHECK
// ============================================================
void checkAlarm() {
  if (mfpPinTriggered == true) {
    Serial.println("MFP Triggered");
    mfpPinTriggered = false;

    Wire.beginTransmission(RTCADDR);
    Wire.write(ALM0WKDAY);
    Wire.endTransmission();
    Wire.requestFrom(RTCADDR, 1);
    delay(1);
    byte alarm0Check = Wire.read();
    Serial.println(alarm0Check, BIN);
    if (((alarm0Check >> 3) & 0x01) == 1)
      Serial.println("Alarm0 Triggered");
    else
      Serial.println("Alarm0 False Alarm");
  }
}

void rtcMFP() {
  mfpPinTriggered = true;
}

// ============================================================
// OLED DISPLAY UPDATE
// ============================================================
void updateDisplays(float tempC, float humidity, float pressure_hPa,
                    float adcVals[16]) {
  // ---------- RIGHT OLED (display1): ADS1115 J3..J18, all on one page ----------
  if (display1.width() > 0) {
    display1.clearDisplay();
    display1.setTextSize(1);

    // 8 rows, 2 columns
    // Left column:  indices 0..7  -> J3..J10
    // Right column: indices 8..15 -> J11..J18
    for (int row = 0; row < 8; row++) {
      int y = row * 8;   // 0,8,16,...,56

      // ---- Left column ----
      int leftIdx  = row;         // 0..7
      int leftJack = leftIdx + 3; // J3..J10

      display1.setCursor(0, y);
      display1.print("J");
      display1.print(leftJack);
      display1.print(":");
      if (!isnan(adcVals[leftIdx])) {
        display1.print(adcVals[leftIdx], 1);   // e.g. 27.5
      } else {
        display1.print("---");
      }

      // ---- Right column ----
      int rightIdx  = row + 8;         // 8..15
      int rightJack = rightIdx + 3;    // J11..J18

      display1.setCursor(64, y);
      display1.print("J");
      display1.print(rightJack);
      display1.print(":");
      if (!isnan(adcVals[rightIdx])) {
        display1.print(adcVals[rightIdx], 1);
      } else {
        display1.print("---");
      }
    }

    display1.display();
  }

  // ---------- LEFT OLED (display2): Env data ----------
  if (display2.width() > 0) {
    display2.clearDisplay();
    display2.setTextSize(1);
    display2.setCursor(0, 0);

    if (rtcToEpoch() > 1) {
      display2.println(rtcToEpoch());
    } else {
      display2.print("RTC ERR");
    }
   

    if(!isnan(tempC)) {
       display2.setCursor(0, 8);
      display2.print("T:");
      display2.print(tempC, 1);
      display2.print("C ");
    } else {
       display2.setCursor(0, 8);
      display2.print("T--.-C");
    }

    if (!isnan(humidity)) {

       // display2.setCursor(0, 20);
      display2.print("H:");
      display2.print(humidity, 0);
      display2.println("%");
    } else {
       //display2.setCursor(0, 20);
      display2.println("H:--%");
    }

    //display2.setCursor(0, 40);
    if (!isnan(pressure_hPa)) {
      display2.print("P:");
      display2.print(pressure_hPa, 0);
      display2.print("hPa");
    } else {
      display2.print("P:----hPa");
    }

    // display2.setCursor(0, 20);
    // display2.print("E:");
    // display2.print(rtcToEpoch());

    display2.display();
  }
}
