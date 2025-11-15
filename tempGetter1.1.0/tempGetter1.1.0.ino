#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <math.h>  
// -----------------------------------------------------------------------------
// I2C ADDRESSES
// -----------------------------------------------------------------------------
const uint8_t LCD_LEFT_ADDR  = 0x3D;
const uint8_t LCD_RIGHT_ADDR = 0x3C;

// const uint8_t ADC0_ADDR = 0x48;   // ADS1115 – disabled for now to save flash
const uint8_t RTC_ADDR  = 0x6F;     // MCP7940N
const uint8_t BME_ADDR  = 0x77;

// -----------------------------------------------------------------------------
// OLED SETUP
// -----------------------------------------------------------------------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32   // you said 32px high

Adafruit_SSD1306 displayLeft(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_SSD1306 displayRight(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const char FW_VERSION[] = "1.0.0";   // firmware version string
float jackTemps[16];   // index 0 -> J3, index 15 -> J18

// -----------------------------------------------------------------------------
// MCP7940 REGISTER DEFINES
// -----------------------------------------------------------------------------
#define RTCSEC     0x00
#define RTCMIN     0x01
#define RTCHOUR    0x02
#define RTCWKDAY   0x03
#define RTCDATE    0x04
#define RTCMTH     0x05
#define RTCYEAR    0x06
#define CONTROL    0x07

// -----------------------------------------------------------------------------
// FORWARD DECLARATIONS
// -----------------------------------------------------------------------------
void rtcInit();
void rtcGetTime();
void checkAlarm();
void rtcMFP();
void updateDisplays();

// -----------------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------------
byte rtcSeconds, rtcMinutes, rtcHours;
byte rtcWeekDay, rtcDay, rtcMonth, rtcYear;
unsigned long currentEpoch = 0;

bool mfpPinTriggered = false;

Adafruit_BME280 bme;
bool bmeOk = false;

// -----------------------------------------------------------------------------
// EPOCH HELPERS
// -----------------------------------------------------------------------------

const uint8_t daysInMonth[12] =
{
  31, 28, 31, 30, 31, 30,
  31, 31, 30, 31, 30, 31
};

bool isLeapYear(int year) {
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

unsigned long toEpoch(int year, int month, int day,
                      int hour, int minute, int second)
{
  unsigned long days = 0;

  for (int y = 1970; y < year; y++) {
    days += isLeapYear(y) ? 366UL : 365UL;
  }

  for (int m = 1; m < month; m++) {
    days += daysInMonth[m - 1];
    if (m == 2 && isLeapYear(year)) {
      days += 1;
    }
  }

  days += (unsigned long)(day - 1);

  unsigned long epoch =
      days * 86400UL +
      (unsigned long)hour   * 3600UL +
      (unsigned long)minute * 60UL   +
      (unsigned long)second;

  return epoch;
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {

  for (int i = 0; i < 16; i++) {
  jackTemps[i] = NAN;   // will show as '---' until you fill them
}

  Serial.begin(9600);

  Wire.setClock(100000);
  Wire.begin();

  rtcInit();

  // BME280 init
  bmeOk = bme.begin(BME_ADDR);
  if (!bmeOk) {
    Serial.println(F("BME280 not found"));
  } else {
    Serial.println(F("BME280 OK"));
  }

  // Left OLED
  if (!displayLeft.begin(SSD1306_SWITCHCAPVCC, LCD_LEFT_ADDR)) {
    Serial.println(F("Left OLED init failed"));
  } else {
    displayLeft.clearDisplay();
    displayLeft.setTextSize(1);
    displayLeft.setTextColor(SSD1306_WHITE);
    displayLeft.setCursor(0, 0);
    displayLeft.println(F("Status Ready"));
    displayLeft.display();
  }

  // Right OLED
  if (!displayRight.begin(SSD1306_SWITCHCAPVCC, LCD_RIGHT_ADDR)) {
    Serial.println(F("Right OLED init failed"));
  } else {
    displayRight.clearDisplay();
    displayRight.setTextSize(1);
    displayRight.setTextColor(SSD1306_WHITE);
    displayRight.setCursor(0, 0);
    displayRight.println(F("Status Ready"));
    displayRight.display();
  }
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------
void loop() {
  rtcGetTime();      // updates currentEpoch and prints it to Serial
  updateDisplays();  // status page on both OLEDs
  checkAlarm();
  delay(1000);
}

// -----------------------------------------------------------------------------
// RTC INITIALIZATION
// -----------------------------------------------------------------------------
void rtcInit() {
  // Clear control register
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(CONTROL);
  Wire.write(0x00);
  Wire.endTransmission();

  // Start oscillator: set ST bit in seconds register
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(RTCSEC);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)1);
  delay(1);
  byte rtcSecondRegister = Wire.read() | 0x80;

  Wire.beginTransmission(RTC_ADDR);
  Wire.write(RTCSEC);
  Wire.write(rtcSecondRegister);
  Wire.endTransmission();
}

// -----------------------------------------------------------------------------
// READ CURRENT TIME, UPDATE currentEpoch, PRINT ONLY EPOCH TO SERIAL
// -----------------------------------------------------------------------------
void rtcGetTime() {

  Wire.beginTransmission(RTC_ADDR);
  Wire.write(RTCSEC);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)7);
  delay(1);

  rtcSeconds = Wire.read() & 0x7F;
  rtcMinutes = Wire.read() & 0x7F;
  rtcHours   = Wire.read() & 0x7F;
  rtcWeekDay = Wire.read() & 0x3F;
  rtcDay     = Wire.read() & 0x3F;
  rtcMonth   = Wire.read() & 0x3F;
  rtcYear    = Wire.read();

  rtcSeconds = (rtcSeconds >> 4) * 10 + (rtcSeconds & 0x0F);
  rtcMinutes = (rtcMinutes >> 4) * 10 + (rtcMinutes & 0x0F);
  rtcHours   = ((rtcHours >> 4) & 0x03) * 10 + (rtcHours & 0x0F);

  rtcDay   = (rtcDay >> 4) * 10 + (rtcDay & 0x0F);
  rtcMonth = ((rtcMonth >> 4) & 0x01) * 10 + (rtcMonth & 0x0F);
  rtcYear  = (rtcYear >> 4) * 10 + (rtcYear & 0x0F);

  int fullYear = 2000 + rtcYear;

  currentEpoch = toEpoch(fullYear, rtcMonth, rtcDay,
                         rtcHours, rtcMinutes, rtcSeconds);

  Serial.println(currentEpoch);
}

// -----------------------------------------------------------------------------
// STATUS PAGE ON BOTH OLEDs
// -----------------------------------------------------------------------------
void updateDisplays() {
  // --------- Read BME280 (if present) ----------
  float tempC = NAN;
  float hum = NAN;
  float press_hPa = NAN;

  if (bmeOk) {
    tempC = bme.readTemperature();
    hum   = bme.readHumidity();
    press_hPa = bme.readPressure() / 100.0f;  // Pa -> hPa
  }

  // --------- LEFT DISPLAY: Epoch + Temp/Hum + Pressure ----------
  displayLeft.clearDisplay();
  displayLeft.setTextSize(1);
  displayLeft.setTextColor(SSD1306_WHITE);

  // Line 1: Epoch
  displayLeft.setCursor(0, 0);
  displayLeft.print(F("E:"));
  displayLeft.println(currentEpoch);

  // Line 2: Temp + Humidity
  displayLeft.setCursor(0, 12);
  if (bmeOk) {
    int humInt = (int)round(hum);    // needs <math.h>

    char tStr[10];
    dtostrf(tempC, 0, 1, tStr);      // e.g. "23.4" no padding

    displayLeft.print(F("T:"));
    displayLeft.print(tStr);
    displayLeft.print(F("C"));

    displayLeft.print(F(" H:"));
    displayLeft.print(humInt);
    displayLeft.print(F("%"));
  } else {
    displayLeft.print(F("BME ERR"));
  }

  // Line 3: Pressure
  displayLeft.setCursor(0, 24);
  if (bmeOk) {
    char pStr[10];
    dtostrf(press_hPa, 0, 0, pStr);  // whole number hPa
    displayLeft.print(F("P:"));
    displayLeft.print(pStr);
    displayLeft.print(F("hPa"));
  } else {
    displayLeft.print(F("P: ---"));
  }

  displayLeft.display();

  // --------- RIGHT DISPLAY: J3–J18 matrix ----------
  displayRight.clearDisplay();
  displayRight.setTextSize(1);
  displayRight.setTextColor(SSD1306_WHITE);

  // Two pages:
  //  - page 0: J3–J10  (indices 0..7)
  //  - page 1: J11–J18 (indices 8..15)
  uint8_t page = (millis() / 2000) % 2;   // flip every 2s
  uint8_t startIndex = page * 8;          // 0 or 8

  // 4 rows, 2 columns per row -> 8 jacks per page
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t idxL = startIndex + row * 2;
    uint8_t idxR = idxL + 1;

    int16_t y = row * 8;      // 0, 8, 16, 24 (32px tall screen)
    int16_t xLeft = 0;
    int16_t xRight = 64;

    drawJackTemp(idxL, xLeft,  y);
    drawJackTemp(idxR, xRight, y);
  }

  displayRight.display();
}

// -----------------------------------------------------------------------------
// ALARM CHECK – no real logic yet, kept small
// -----------------------------------------------------------------------------
void checkAlarm() {
  if (!mfpPinTriggered) return;
  mfpPinTriggered = false;
}

// -----------------------------------------------------------------------------
// MFP INTERRUPT HANDLER
// -----------------------------------------------------------------------------
void rtcMFP() {
  mfpPinTriggered = true;
}

void drawJackTemp(uint8_t index, int16_t x, int16_t y) {
  if (index >= 16) return;

  uint8_t jackNum = 3 + index;   // index 0 -> J3, 1 -> J4, ..., 15 -> J18
  float t = jackTemps[index];

  displayRight.setCursor(x, y);
  displayRight.print('J');
  displayRight.print(jackNum);
  displayRight.print(':');

  if (isnan(t)) {
    displayRight.print(F("---"));
  } else {
    char buf[8];
    dtostrf(t, 0, 1, buf);  // e.g. "35.6"
    displayRight.print(buf);
  }
}
