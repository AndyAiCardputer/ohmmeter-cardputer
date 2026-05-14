/*
 * Precision Ohmmeter for M5Stack Cardputer v1.1
 * Version: 1.5.1
 *
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
 *
 * Formula:
 *   R_unknown = R_known * V_measured / (VDD - V_measured)
 *
 * Andy + AI, May 2026
 */

#include <M5Cardputer.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// --- Hardware constants ---
const float R_KNOWN = 10040.0;       // Reference resistor, measured with multimeter (Ohm)
const float VDD_INITIAL = 5.0;       // Initial estimate (Grove Port = 5V, calibrated from OPEN readings)
const uint8_t ADS_ADDR = 0x48;       // ADS1115 I2C address
const int SDA_PIN = 2;               // Grove Port A on Cardputer v1.1
const int SCL_PIN = 1;

// --- Measurement settings ---
const int NUM_SAMPLES_MIN = 16;      // Min samples (low R, good signal)
const int NUM_SAMPLES_MAX = 128;     // Max samples (high R, noisy signal)
const int MEASURE_INTERVAL_MS = 200; // Update display ~5 times per second
const float OPEN_THRESHOLD = 0.999;  // If V > VDD * 0.999, consider probes open
const float EMA_FAST = 0.4;          // EMA alpha for new resistor (fast response)
const float EMA_SLOW = 0.1;          // EMA alpha for stable reading (smooth)
const int OPEN_CONFIRM_COUNT = 3;    // Require N consecutive OPEN reads before declaring OPEN

// --- Continuity (buzzer) ---
const float CONTINUITY_THRESHOLD = 50.0;  // Beep when R < 50 Ohm
const int BEEP_FREQ = 2000;               // 2 kHz tone

// --- Display ---
#define SCREEN_W 240
#define SCREEN_H 135

#define COL_BG       0x0000   // Black
#define COL_PANEL    0x18E3   // Dark gray
#define COL_GREEN    0x07E0
#define COL_YELLOW   0xFFE0
#define COL_RED      0xF800
#define COL_CYAN     0x07FF
#define COL_WHITE    0xFFFF
#define COL_GRAY     0x7BEF
#define COL_ORANGE   0xFD20

// --- PGA gain table ---
struct GainEntry {
    adsGain_t gain;
    float fsRange;       // Full-scale range (V)
    float lsb;           // LSB size (V)
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
M5Canvas canvas(&M5Cardputer.Display);
bool adsFound = false;
int currentGainIdx = 0;  // Start with GAIN_TWOTHIRDS (±6.144V) to read full 5V
float lastResistance = -1;
float lastVoltage = 0;
float displayResistance = -1;      // EMA-smoothed value for display
bool probeOpen = true;
float vdd = VDD_INITIAL;
bool vddCalibrated = false;
int currentSamples = NUM_SAMPLES_MIN;
int openCounter = 0;                  // Consecutive OPEN readings counter
bool continuityBeep = false;          // True when continuity detected this cycle
bool continuityMode = false;          // Toggled by 'S' key
bool toneActive = false;              // Track if tone is currently playing

// --- Find optimal PGA gain for measured voltage ---
int findBestGain(float voltage) {
    float absV = fabs(voltage);

    // From highest gain (most sensitive) to lowest
    // Use 80% of FSR as threshold to avoid clipping
    for (int i = GAIN_COUNT - 1; i >= 0; i--) {
        if (absV < gainTable[i].fsRange * 0.80) {
            // But don't use gains where input impedance drops too low
            // ±0.256V and ±0.512V have only 100kOhm input impedance
            // which affects high-R measurements
            if (i >= 4 && lastResistance > 50000) {
                continue;  // Skip high gains for high-R measurements
            }
            return i;
        }
    }
    return 0;  // Fallback: widest range
}

// --- Choose number of samples based on resistance range ---
int chooseSamples(float resistance) {
    if (resistance < 0) return NUM_SAMPLES_MIN;       // OPEN
    if (resistance < 10000) return NUM_SAMPLES_MIN;    // <10k: 16 samples
    if (resistance < 50000) return 32;                 // 10k-50k: 32 samples
    if (resistance < 200000) return 64;                // 50k-200k: 64 samples
    return NUM_SAMPLES_MAX;                            // >200k: 128 samples
}

// --- Read ADS1115 with adaptive averaging ---
float readVoltageAvg(int gainIdx, int numSamples) {
    ads.setGain(gainTable[gainIdx].gain);

    float sum = 0;
    for (int i = 0; i < numSamples; i++) {
        int16_t raw = ads.readADC_SingleEnded(0);
        sum += raw * gainTable[gainIdx].lsb;
    }
    return sum / numSamples;
}

// --- Calculate resistance from voltage divider ---
float calcResistance(float vMeasured) {
    if (vMeasured <= 0.001) {
        return 0;
    }
    if (vMeasured >= vdd * OPEN_THRESHOLD) {
        return -1;  // Open circuit candidate
    }
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

// --- Draw the main screen ---
void drawScreen(float resistance, float voltage, int gainIdx) {
    canvas.createSprite(SCREEN_W, SCREEN_H);
    canvas.fillSprite(COL_BG);

    // Title bar
    canvas.fillRect(0, 0, SCREEN_W, 20, COL_PANEL);
    canvas.setTextColor(COL_CYAN);
    canvas.setTextSize(1);
    canvas.setFont(&fonts::Font2);
    canvas.drawString("OHMMETER v1.5", 6, 2);

    // Status in title bar
    if (!adsFound) {
        canvas.setTextColor(COL_RED);
        canvas.drawString("ADS1115 NOT FOUND!", 120, 2);
    } else if (continuityMode) {
        canvas.setTextColor(COL_GREEN);
        canvas.drawString("[S] BEEP ON", 150, 2);
    } else {
        canvas.setTextColor(COL_GRAY);
        canvas.drawString("[S] beep off", 150, 2);
    }

    // Main resistance value
    char valBuf[20];
    char unitBuf[10];
    formatResistance(resistance, valBuf, unitBuf);

    bool isOpen = (resistance < 0);

    // Big digits
    canvas.setFont(&fonts::Font7);
    if (isOpen) {
        canvas.setTextColor(COL_YELLOW);
        canvas.setTextSize(1);
        canvas.drawCenterString("OPEN", SCREEN_W / 2, 30);
    } else {
        canvas.setTextColor(COL_GREEN);
        // Adjust font size based on value length
        int len = strlen(valBuf);
        if (len <= 5) {
            canvas.setTextSize(1);
        } else {
            canvas.setTextSize(1);
        }
        canvas.drawCenterString(valBuf, SCREEN_W / 2, 28);
    }

    // Unit label or continuity indicator
    if (!isOpen) {
        if (continuityBeep) {
            canvas.fillRect(0, 78, SCREEN_W, 24, COL_GREEN);
            canvas.setFont(&fonts::Font4);
            canvas.setTextSize(1);
            canvas.setTextColor(COL_BG);
            canvas.drawCenterString("BEEP", SCREEN_W / 2, 80);
        } else {
            canvas.setFont(&fonts::Font4);
            canvas.setTextSize(1);
            canvas.setTextColor(COL_CYAN);
            canvas.drawCenterString(unitBuf, SCREEN_W / 2, 82);
        }
    }

    // Info bar at bottom
    canvas.fillRect(0, SCREEN_H - 28, SCREEN_W, 28, COL_PANEL);
    canvas.setFont(&fonts::Font2);
    canvas.setTextSize(1);

    // Voltage reading
    canvas.setTextColor(COL_YELLOW);
    char vBuf[30];
    sprintf(vBuf, "V: %.4fV", voltage);
    canvas.drawString(vBuf, 6, SCREEN_H - 25);

    // PGA range
    canvas.setTextColor(COL_GRAY);
    char pgaBuf[20];
    sprintf(pgaBuf, "PGA: %s", gainTable[gainIdx].label);
    canvas.drawString(pgaBuf, 140, SCREEN_H - 25);

    // VDD info
    canvas.setTextColor(vddCalibrated ? COL_GREEN : COL_ORANGE);
    char rBuf[30];
    sprintf(rBuf, "VDD:%.2fV", vdd);
    canvas.drawString(rBuf, 6, SCREEN_H - 12);

    // Samples info
    canvas.setTextColor(COL_GRAY);
    char sBuf[30];
    sprintf(sBuf, "Rref=%.2fk", R_KNOWN / 1000.0);
    canvas.drawString(sBuf, 140, SCREEN_H - 12);

    canvas.pushSprite(0, 0);
    canvas.deleteSprite();
}

// --- Setup ---
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg);

    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.fillScreen(COL_BG);
    M5Cardputer.Display.setTextColor(COL_WHITE);

    M5Cardputer.Speaker.begin();
    M5Cardputer.Speaker.setVolume(200);

    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== Precision Ohmmeter v1.5.1 ===");
    Serial.printf("R_known = %.1f Ohm\n", R_KNOWN);
    Serial.println("VDD auto-calibration: leave probes OPEN at startup");

    // Init I2C on Grove Port A (Cardputer v1.1: SDA=G2, SCL=G1)
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);  // 400kHz Fast mode
    Serial.printf("I2C init: SDA=%d, SCL=%d\n", SDA_PIN, SCL_PIN);

    // Init ADS1115
    if (ads.begin(ADS_ADDR, &Wire)) {
        adsFound = true;
        ads.setGain(gainTable[currentGainIdx].gain);
        ads.setDataRate(RATE_ADS1115_128SPS);
        Serial.println("ADS1115 found at 0x48");

        // Test read
        int16_t testRaw = ads.readADC_SingleEnded(0);
        float testV = testRaw * gainTable[currentGainIdx].lsb;
        Serial.printf("Test read: raw=%d, V=%.4f\n", testRaw, testV);
    } else {
        adsFound = false;
        Serial.println("ERROR: ADS1115 not found!");
    }

    // Startup screen
    M5Cardputer.Display.setFont(&fonts::Font2);
    M5Cardputer.Display.drawCenterString("Ohmmeter v1.5.1", SCREEN_W / 2, 20);
    M5Cardputer.Display.drawCenterString("ADS1115 16-bit ADC", SCREEN_W / 2, 45);
    if (adsFound) {
        M5Cardputer.Display.setTextColor(COL_GREEN);
        M5Cardputer.Display.drawCenterString("ADS1115 OK!", SCREEN_W / 2, 70);
    } else {
        M5Cardputer.Display.setTextColor(COL_RED);
        M5Cardputer.Display.drawCenterString("ADS1115 NOT FOUND!", SCREEN_W / 2, 70);
    }
    // High-precision VDD calibration at startup (probes must be open)
    if (adsFound) {
        ads.setGain(GAIN_TWOTHIRDS);  // ±6.144V to read full 5V range
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
        M5Cardputer.Display.setTextColor(COL_CYAN);
        char vddBuf[30];
        sprintf(vddBuf, "VDD = %.3fV", vdd);
        M5Cardputer.Display.drawCenterString(vddBuf, SCREEN_W / 2, 95);
    }

    M5Cardputer.Display.setTextColor(COL_GRAY);
    M5Cardputer.Display.drawCenterString("Leave probes open for calibration", SCREEN_W / 2, 115);
    delay(2000);
}

// --- Main Loop ---
void loop() {
    M5Cardputer.update();

    if (!adsFound) {
        drawScreen(-1, 0, currentGainIdx);
        delay(1000);
        return;
    }

    // Step 1: Adaptive sample count based on last measurement
    currentSamples = chooseSamples(lastResistance);

    // Step 2: Read voltage with adaptive averaging
    float voltage = readVoltageAvg(currentGainIdx, currentSamples);

    // Step 3: Auto-range PGA
    int bestGain = findBestGain(voltage);
    if (bestGain != currentGainIdx) {
        currentGainIdx = bestGain;
        voltage = readVoltageAvg(currentGainIdx, currentSamples);
        Serial.printf("PGA switched to %s\n", gainTable[currentGainIdx].label);
    }

    // Step 4: Calculate resistance
    float resistance = calcResistance(voltage);
    lastVoltage = voltage;

    // Step 5: OPEN confirmation (need N consecutive OPEN reads)
    if (resistance < 0) {
        openCounter++;
        if (openCounter < OPEN_CONFIRM_COUNT) {
            // Not confirmed yet — keep last valid reading
            if (lastResistance > 0) {
                resistance = lastResistance;
            }
        }
    } else {
        openCounter = 0;
    }

    // Step 6: EMA smoothing for display
    if (resistance < 0 && openCounter >= OPEN_CONFIRM_COUNT) {
        // Confirmed OPEN — gently recalibrate VDD (tracks battery/USB changes)
        if (voltage > 3.0) {
            vdd = vdd * 0.95 + voltage * 0.05;
        }
        displayResistance = -1;
        probeOpen = true;
    } else if (resistance > 0 && (probeOpen || displayResistance < 0)) {
        // Just connected — snap to new value immediately
        displayResistance = resistance;
        probeOpen = false;
    } else {
        // Check if resistor changed significantly (>30% jump = new resistor)
        float ratio = resistance / displayResistance;
        if (ratio > 1.3 || ratio < 0.7) {
            displayResistance = resistance;  // Snap to new value
        } else {
            // Smooth with EMA
            float alpha = (resistance > 100000) ? EMA_SLOW : EMA_FAST;
            displayResistance = displayResistance * (1.0 - alpha) + resistance * alpha;
        }
    }
    if (resistance > 0) lastResistance = resistance;

    // Step 7: Handle keyboard — 'S' toggles continuity mode
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

    // Step 8: Continuity test — continuous tone while R < threshold
    continuityBeep = continuityMode && displayResistance >= 0 && displayResistance < CONTINUITY_THRESHOLD;
    if (continuityBeep && !toneActive) {
        M5Cardputer.Speaker.tone(BEEP_FREQ);
        toneActive = true;
    } else if (!continuityBeep && toneActive) {
        M5Cardputer.Speaker.stop();
        toneActive = false;
    }

    // Step 9: Serial output (raw, not smoothed)
    if (resistance < 0) {
        Serial.printf("OPEN  (V=%.4f, PGA=%s)\n", voltage, gainTable[currentGainIdx].label);
    } else {
        char valBuf[20], unitBuf[10];
        formatResistance(resistance, valBuf, unitBuf);
        Serial.printf("R=%s %s  (V=%.4f, PGA=%s, N=%d)\n",
                       valBuf, unitBuf, voltage, gainTable[currentGainIdx].label, currentSamples);
    }

    // Step 10: Draw display with smoothed value
    drawScreen(displayResistance, voltage, currentGainIdx);

    delay(MEASURE_INTERVAL_MS);
}
