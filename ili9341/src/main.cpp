/*
 * Precision Ohmmeter for M5Stack Cardputer v1.1
 * Version: 1.5.1-ILI9341
 *
 * External 2.8" ILI9341 display (320x240) version.
 * Uses ADS1115 16-bit ADC with auto-PGA ranging.
 * Measures resistance from ~22 Ohm to ~2 MOhm.
 * High-precision VDD calibration at startup + periodic recalibration.
 * Adaptive averaging + EMA smoothing for stable readings.
 * Continuity test mode with audible beep.
 *
 * Circuit:
 *   VDD (5V via Grove) --- R_known (10.04k) --- Junction --- R_unknown --- GND
 *                                           |
 *                                        ADS1115 A0
 *
 * Connection:
 *   Grove Port A: SDA=GPIO2, SCL=GPIO1 (Cardputer v1.1)
 *   ADS1115 address: 0x48 (ADDR->GND)
 *   ILI9341: SPI via EXT pins (SCK=40, MOSI=14, CS=5, DC=6, RST=3)
 *
 * Andy + AI, May 2026
 */

#include <M5Cardputer.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include "external_display/LGFX_ILI9341.h"

// SPI bus (shared, same pins as in ZX Spectrum project)
#define SD_SPI_SCK_PIN  40
#define SD_SPI_MISO_PIN 39
#define SD_SPI_MOSI_PIN 14
#define SD_CS_PIN       12
SPIClass extSPI(HSPI);

// --- Hardware constants ---
const float R_KNOWN = 10040.0;
const float VDD_INITIAL = 5.0;
const uint8_t ADS_ADDR = 0x48;
const int SDA_PIN = 2;
const int SCL_PIN = 1;

// --- Measurement settings ---
const int NUM_SAMPLES_MIN = 16;
const int NUM_SAMPLES_MAX = 128;
const int MEASURE_INTERVAL_MS = 200;
const float OPEN_THRESHOLD = 0.999;
const float EMA_FAST = 0.4;
const float EMA_SLOW = 0.1;
const int OPEN_CONFIRM_COUNT = 3;

// --- Continuity (buzzer) ---
const float CONTINUITY_THRESHOLD = 50.0;
const int BEEP_FREQ = 2000;

// --- External display (320x240) ---
#define EXT_W 320
#define EXT_H 240

// Colors
#define COL_BG       0x0000
#define COL_PANEL    0x18E3
#define COL_GREEN    0x07E0
#define COL_YELLOW   0xFFE0
#define COL_RED      0xF800
#define COL_CYAN     0x07FF
#define COL_WHITE    0xFFFF
#define COL_GRAY     0x7BEF
#define COL_ORANGE   0xFD20
#define COL_DKGREEN  0x03E0

// --- PGA gain table ---
struct GainEntry {
    adsGain_t gain;
    float fsRange;
    float lsb;
    const char* label;
};

const GainEntry gainTable[] = {
    { GAIN_TWOTHIRDS, 6.144, 0.0001875,  "6.144V" },
    { GAIN_ONE,       4.096, 0.000125,   "4.096V" },
    { GAIN_TWO,       2.048, 0.0000625,  "2.048V" },
    { GAIN_FOUR,      1.024, 0.00003125, "1.024V" },
    { GAIN_EIGHT,     0.512, 0.000015625,"0.512V" },
    { GAIN_SIXTEEN,   0.256, 0.0000078125,"0.256V" },
};
const int GAIN_COUNT = sizeof(gainTable) / sizeof(gainTable[0]);

// --- State ---
Adafruit_ADS1115 ads;
LGFX_ILI9341 extLcd;
bool adsFound = false;
int currentGainIdx = 0;
float lastResistance = -1;
float lastVoltage = 0;
float displayResistance = -1;
bool probeOpen = true;
float vdd = VDD_INITIAL;
bool vddCalibrated = false;
int currentSamples = NUM_SAMPLES_MIN;
int openCounter = 0;
bool continuityBeep = false;
bool continuityMode = false;
bool toneActive = false;

// --- Find optimal PGA gain ---
int findBestGain(float voltage) {
    float absV = fabs(voltage);
    for (int i = GAIN_COUNT - 1; i >= 0; i--) {
        if (absV < gainTable[i].fsRange * 0.80) {
            if (i >= 4 && lastResistance > 50000) continue;
            return i;
        }
    }
    return 0;
}

// --- Choose sample count ---
int chooseSamples(float resistance) {
    if (resistance < 0) return NUM_SAMPLES_MIN;
    if (resistance < 10000) return NUM_SAMPLES_MIN;
    if (resistance < 50000) return 32;
    if (resistance < 200000) return 64;
    return NUM_SAMPLES_MAX;
}

// --- Read ADS1115 with averaging ---
float readVoltageAvg(int gainIdx, int numSamples) {
    ads.setGain(gainTable[gainIdx].gain);
    float sum = 0;
    for (int i = 0; i < numSamples; i++) {
        int16_t raw = ads.readADC_SingleEnded(0);
        sum += raw * gainTable[gainIdx].lsb;
    }
    return sum / numSamples;
}

// --- Calculate resistance ---
float calcResistance(float vMeasured) {
    if (vMeasured <= 0.001) return 0;
    if (vMeasured >= vdd * OPEN_THRESHOLD) return -1;
    return R_KNOWN * vMeasured / (vdd - vMeasured);
}

// --- Format resistance with auto-units ---
void formatResistance(float ohms, char* buf, char* unitBuf) {
    if (ohms < 0) {
        strcpy(buf, "OPEN");
        strcpy(unitBuf, "");
        return;
    }
    if (ohms < 1.0) {
        sprintf(buf, "%.2f", ohms);
        strcpy(unitBuf, "Ohm");
    } else if (ohms < 1000.0) {
        sprintf(buf, "%.1f", ohms);
        strcpy(unitBuf, "Ohm");
    } else if (ohms < 10000.0) {
        sprintf(buf, "%.3f", ohms / 1000.0);
        strcpy(unitBuf, "kOhm");
    } else if (ohms < 100000.0) {
        sprintf(buf, "%.2f", ohms / 1000.0);
        strcpy(unitBuf, "kOhm");
    } else if (ohms < 1000000.0) {
        sprintf(buf, "%.1f", ohms / 1000.0);
        strcpy(unitBuf, "kOhm");
    } else {
        sprintf(buf, "%.3f", ohms / 1000000.0);
        strcpy(unitBuf, "MOhm");
    }
}

// --- Draw bar graph for resistance (log scale) ---
void drawBarGraph(int x, int y, int w, int h, float resistance) {
    extLcd.fillRect(x, y, w, h, COL_PANEL);
    if (resistance <= 0) return;

    float logMin = log10(10.0);
    float logMax = log10(2000000.0);
    float logR = log10(resistance);
    float ratio = (logR - logMin) / (logMax - logMin);
    if (ratio < 0) ratio = 0;
    if (ratio > 1.0) ratio = 1.0;

    int barW = (int)(ratio * (w - 4));
    uint16_t barColor = COL_GREEN;
    if (resistance > 500000) barColor = COL_ORANGE;
    if (resistance > 1000000) barColor = COL_RED;

    extLcd.fillRect(x + 2, y + 2, barW, h - 4, barColor);
    // Clear rest of bar area
    if (barW < w - 4) {
        extLcd.fillRect(x + 2 + barW, y + 2, w - 4 - barW, h - 4, COL_PANEL);
    }

    extLcd.setFont(&fonts::Font0);
    extLcd.setTextSize(1);
    extLcd.setTextColor(COL_GRAY, COL_BG);
    extLcd.drawString("10", x + 2, y + h + 2);
    extLcd.drawString("1k", x + w / 3 - 4, y + h + 2);
    extLcd.drawString("100k", x + 2 * w / 3 - 8, y + h + 2);
    extLcd.drawString("2M", x + w - 16, y + h + 2);
}

// --- Draw screen on external 320x240 display (direct, no sprite) ---
void drawScreen(float resistance, float voltage, int gainIdx) {
    extLcd.startWrite();

    // === Title bar (top 28px) ===
    extLcd.fillRect(0, 0, EXT_W, 28, COL_PANEL);
    extLcd.setFont(&fonts::Font2);
    extLcd.setTextSize(1);
    extLcd.setTextColor(COL_CYAN, COL_PANEL);
    extLcd.drawString("PRECISION OHMMETER v1.5", 8, 5);

    if (!adsFound) {
        extLcd.setTextColor(COL_RED, COL_PANEL);
        extLcd.drawString("ADS1115 ERROR", 210, 5);
    } else if (continuityMode) {
        extLcd.setTextColor(COL_GREEN, COL_PANEL);
        extLcd.drawString("[S] BEEP ON ", 218, 5);
    } else {
        extLcd.setTextColor(COL_GRAY, COL_PANEL);
        extLcd.drawString("[S] beep off", 218, 5);
    }

    // === Main area: clear ===
    extLcd.fillRect(0, 28, EXT_W, 122, COL_BG);

    char valBuf[20];
    char unitBuf[10];
    formatResistance(resistance, valBuf, unitBuf);
    bool isOpen = (resistance < 0);

    if (isOpen) {
        extLcd.setFont(&fonts::Font7);
        extLcd.setTextSize(2);
        extLcd.setTextColor(COL_YELLOW, COL_BG);
        extLcd.drawCenterString("OPEN", EXT_W / 2, 45);
    } else {
        extLcd.setFont(&fonts::Font7);
        extLcd.setTextColor(COL_GREEN, COL_BG);
        int len = strlen(valBuf);
        if (len <= 3) {
            extLcd.setTextSize(2);
            extLcd.drawCenterString(valBuf, EXT_W / 2, 40);
        } else {
            extLcd.setTextSize(1);
            extLcd.drawCenterString(valBuf, EXT_W / 2, 55);
        }
    }

    // === Unit label or BEEP indicator ===
    if (!isOpen) {
        if (continuityBeep) {
            extLcd.fillRect(0, 120, EXT_W, 30, COL_GREEN);
            extLcd.setFont(&fonts::Font4);
            extLcd.setTextSize(1);
            extLcd.setTextColor(COL_BG, COL_GREEN);
            extLcd.drawCenterString("CONTINUITY", EXT_W / 2, 124);
        } else {
            extLcd.fillRect(0, 120, EXT_W, 30, COL_BG);
            extLcd.setFont(&fonts::Font4);
            extLcd.setTextSize(1);
            extLcd.setTextColor(COL_CYAN, COL_BG);
            extLcd.drawCenterString(unitBuf, EXT_W / 2, 124);
        }
    }

    // === Bar graph (log scale) ===
    if (!isOpen && resistance > 0) {
        drawBarGraph(10, 158, EXT_W - 20, 16, resistance);
    } else {
        extLcd.fillRect(10, 158, EXT_W - 20, 26, COL_BG);
    }

    // === Info panel (bottom 42px) ===
    extLcd.fillRect(0, EXT_H - 42, EXT_W, 42, COL_PANEL);
    extLcd.setFont(&fonts::Font2);
    extLcd.setTextSize(1);

    extLcd.setTextColor(COL_YELLOW, COL_PANEL);
    char vBuf[30];
    sprintf(vBuf, "V: %.4fV", voltage);
    extLcd.drawString(vBuf, 10, EXT_H - 39);

    extLcd.setTextColor(COL_GRAY, COL_PANEL);
    char pgaBuf[20];
    sprintf(pgaBuf, "PGA: %s", gainTable[gainIdx].label);
    extLcd.drawString(pgaBuf, 180, EXT_H - 39);

    extLcd.setTextColor(vddCalibrated ? COL_GREEN : COL_ORANGE, COL_PANEL);
    char vddB[20];
    sprintf(vddB, "VDD: %.3fV", vdd);
    extLcd.drawString(vddB, 10, EXT_H - 22);

    extLcd.setTextColor(COL_GRAY, COL_PANEL);
    char refBuf[20];
    sprintf(refBuf, "Rref=%.2fk", R_KNOWN / 1000.0);
    extLcd.drawString(refBuf, 130, EXT_H - 22);

    char sampBuf[20];
    sprintf(sampBuf, "N=%d  ", currentSamples);
    extLcd.drawString(sampBuf, 260, EXT_H - 22);

    extLcd.endWrite();
}

// --- Setup ---
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg);

    M5Cardputer.Speaker.begin();
    M5Cardputer.Speaker.setVolume(100);
    M5Cardputer.Display.setBrightness(0);  // Disable built-in display

    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== Precision Ohmmeter v1.5.1-ILI9341 ===");
    Serial.printf("R_known = %.1f Ohm\n", R_KNOWN);

    // Init SPI bus FIRST (required before display init)
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);  // Deselect SD card
    extSPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_CS_PIN);
    Serial.println("SPI bus initialized");

    // Init external ILI9341 display
    if (!extLcd.init()) {
        Serial.println("ERROR: External display init FAILED!");
    }
    extLcd.setRotation(3);  // Landscape 320x240 (rotation 3 like ZX Spectrum)
    extLcd.setColorDepth(16);
    delay(50);
    extLcd.fillScreen(COL_BG);
    extLcd.setTextColor(COL_WHITE);

    Serial.printf("External ILI9341 display ready: %dx%d\n", extLcd.width(), extLcd.height());

    // Init I2C on Grove Port A
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Serial.printf("I2C init: SDA=%d, SCL=%d\n", SDA_PIN, SCL_PIN);

    // Init ADS1115
    if (ads.begin(ADS_ADDR, &Wire)) {
        adsFound = true;
        ads.setGain(gainTable[currentGainIdx].gain);
        ads.setDataRate(RATE_ADS1115_128SPS);
        Serial.println("ADS1115 found at 0x48");

        int16_t testRaw = ads.readADC_SingleEnded(0);
        float testV = testRaw * gainTable[currentGainIdx].lsb;
        Serial.printf("Test read: raw=%d, V=%.4f\n", testRaw, testV);
    } else {
        adsFound = false;
        Serial.println("ERROR: ADS1115 not found!");
    }

    // Startup screen on external display
    extLcd.setFont(&fonts::Font4);
    extLcd.drawCenterString("Precision Ohmmeter", EXT_W / 2, 30);
    extLcd.setFont(&fonts::Font2);
    extLcd.drawCenterString("v1.5.1 - ILI9341 320x240", EXT_W / 2, 70);
    extLcd.drawCenterString("ADS1115 16-bit ADC", EXT_W / 2, 95);

    if (adsFound) {
        extLcd.setTextColor(COL_GREEN);
        extLcd.drawCenterString("ADS1115 OK!", EXT_W / 2, 125);
    } else {
        extLcd.setTextColor(COL_RED);
        extLcd.drawCenterString("ADS1115 NOT FOUND!", EXT_W / 2, 125);
    }

    // High-precision VDD calibration at startup
    if (adsFound) {
        ads.setGain(GAIN_TWOTHIRDS);
        const int VDD_CAL_SAMPLES = 128;
        float vCalib = 0;
        for (int i = 0; i < VDD_CAL_SAMPLES; i++) {
            int16_t raw = ads.readADC_SingleEnded(0);
            vCalib += raw * 0.0001875;
        }
        vCalib /= (float)VDD_CAL_SAMPLES;
        if (vCalib > 3.0) {
            vdd = vCalib;
            vddCalibrated = true;
            Serial.printf("Startup VDD calibration: %.5fV (%d samples)\n", vdd, VDD_CAL_SAMPLES);
        }
        extLcd.setTextColor(COL_CYAN);
        char vddBuf[30];
        sprintf(vddBuf, "VDD = %.3fV", vdd);
        extLcd.drawCenterString(vddBuf, EXT_W / 2, 155);
    }

    extLcd.setTextColor(COL_GRAY);
    extLcd.drawCenterString("Leave probes open for calibration", EXT_W / 2, 185);
    extLcd.drawCenterString("Press [S] for continuity buzzer", EXT_W / 2, 210);
    delay(2500);

    // Clear startup screen before entering main loop
    extLcd.fillScreen(COL_BG);
}

// --- Main Loop ---
void loop() {
    M5Cardputer.update();

    if (!adsFound) {
        drawScreen(-1, 0, currentGainIdx);
        delay(1000);
        return;
    }

    currentSamples = chooseSamples(lastResistance);
    float voltage = readVoltageAvg(currentGainIdx, currentSamples);

    int bestGain = findBestGain(voltage);
    if (bestGain != currentGainIdx) {
        currentGainIdx = bestGain;
        voltage = readVoltageAvg(currentGainIdx, currentSamples);
        Serial.printf("PGA switched to %s\n", gainTable[currentGainIdx].label);
    }

    float resistance = calcResistance(voltage);
    lastVoltage = voltage;

    // OPEN confirmation
    if (resistance < 0) {
        openCounter++;
        if (openCounter < OPEN_CONFIRM_COUNT) {
            if (lastResistance > 0) resistance = lastResistance;
        }
    } else {
        openCounter = 0;
    }

    // EMA smoothing
    if (resistance < 0 && openCounter >= OPEN_CONFIRM_COUNT) {
        if (voltage > 3.0) {
            vdd = vdd * 0.95 + voltage * 0.05;
        }
        displayResistance = -1;
        probeOpen = true;
    } else if (resistance > 0 && (probeOpen || displayResistance < 0)) {
        displayResistance = resistance;
        probeOpen = false;
    } else {
        float ratio = resistance / displayResistance;
        if (ratio > 1.3 || ratio < 0.7) {
            displayResistance = resistance;
        } else {
            float alpha = (resistance > 100000) ? EMA_SLOW : EMA_FAST;
            displayResistance = displayResistance * (1.0 - alpha) + resistance * alpha;
        }
    }
    if (resistance > 0) lastResistance = resistance;

    // Keyboard — 'S' toggles continuity mode
    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
        Keyboard_Class::KeysState keys = M5Cardputer.Keyboard.keysState();
        for (auto key : keys.word) {
            if (key == 's' || key == 'S') {
                continuityMode = !continuityMode;
                if (!continuityMode && toneActive) {
                    M5Cardputer.Speaker.stop();
                    toneActive = false;
                }
                Serial.printf("Continuity mode: %s\n", continuityMode ? "ON" : "OFF");
            }
        }
    }

    // Continuity buzzer
    continuityBeep = continuityMode && displayResistance >= 0 && displayResistance < CONTINUITY_THRESHOLD;
    if (continuityBeep && !toneActive) {
        M5Cardputer.Speaker.tone(BEEP_FREQ);
        toneActive = true;
    } else if (!continuityBeep && toneActive) {
        M5Cardputer.Speaker.stop();
        toneActive = false;
    }

    // Serial output
    if (resistance < 0) {
        Serial.printf("OPEN  (V=%.4f, PGA=%s)\n", voltage, gainTable[currentGainIdx].label);
    } else {
        char valBuf[20], unitBuf[10];
        formatResistance(resistance, valBuf, unitBuf);
        Serial.printf("R=%s %s  (V=%.4f, PGA=%s, N=%d)\n",
                       valBuf, unitBuf, voltage, gainTable[currentGainIdx].label, currentSamples);
    }

    // Draw on external display
    drawScreen(displayResistance, voltage, currentGainIdx);

    delay(MEASURE_INTERVAL_MS);
}
