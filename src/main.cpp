// Bench I/O test firmware
//
// Raw ADC counts for APPS1/APPS2/brake sensors printed at 10 Hz, plus:
//   Brake pedal pressure                         -> brake light on (pin 6)
//   AUX1 button (pin 7, ESP32_TX socket) held    -> RTD speaker on (pin 9)
//   RTD button  (pin 2) held 3s                  -> toggle PRECHARGE_EN (pin 3)
//   AUX2 button (pin 8, ESP32_RX socket) held 3s -> same toggle (stand-in,
//   kept while the RTD harness is being verified)
// Buttons wire to GND, no board pull-ups: INPUT_PULLUP, pressed = LOW.
// RTD button now wired same style as AUX buttons (active-low).
// Status row includes the raw RTD pin level to debug wiring directly.
// All events reported over USB serial at 115200.

#include <Arduino.h>

// Analog inputs (through 10k/20k divider + 1k RC on breakout board)
#define APPS1_PIN       A0
#define APPS2_PIN       A1
#define BRAKE_FRONT_PIN A2
#define BRAKE_REAR_PIN  A3

// Buttons (active low)
#define AUX1_BUTTON_PIN 7
#define AUX2_BUTTON_PIN 8
#define RTD_BUTTON_PIN  2

// Outputs
#define BRAKE_LIGHT_PIN  6  // Q3 MOSFET gate, HIGH = light on
#define RTD_SPEAKER_PIN  9  // Q1 MOSFET gate, HIGH = speaker on
#define PRECHARGE_EN_PIN 3  // to precharge circuit, HIGH = enable

#define PRINT_INTERVAL_MS 100   // 10 Hz sensor prints
#define PRECHARGE_HOLD_MS 3000
#define DEBOUNCE_MS       50

// Brake light thresholds, raw ADC counts (rest ~400, light tap ~457/421).
// ON above ~3 psi, hysteresis gap prevents flicker at the threshold.
#define BRAKE_LIGHT_ON_COUNTS  435
#define BRAKE_LIGHT_OFF_COUNTS 415

struct DebouncedButton {
  uint8_t pin;
  bool activeLow;    // true: pressed = pin LOW (button to GND, INPUT_PULLUP)
  bool pressed;      // debounced logical state
  bool lastRaw;
  uint32_t lastEdge;
};

DebouncedButton aux1 = {AUX1_BUTTON_PIN, true,  false, false, 0};
DebouncedButton aux2 = {AUX2_BUTTON_PIN, true,  false, false, 0};
DebouncedButton rtd  = {RTD_BUTTON_PIN,  true,  false, false, 0};

bool prechargeEnabled = false;

// Per-button hold-to-toggle state
struct HoldTracker {
  const char *name;
  uint32_t holdStart;
  bool consumed;      // require release before next toggle
  uint32_t lastPrint;
};

HoldTracker rtdHold  = {"RTD",  0, false, 0};
HoldTracker aux2Hold = {"AUX2", 0, false, 0};

// Returns true when the debounced state changed this call.
static bool updateButton(DebouncedButton &b) {
  bool raw = b.activeLow ? digitalRead(b.pin) == LOW
                         : digitalRead(b.pin) == HIGH;
  if (raw != b.lastRaw) {
    b.lastEdge = millis();
    b.lastRaw = raw;
  }
  if (raw != b.pressed && millis() - b.lastEdge > DEBOUNCE_MS) {
    b.pressed = raw;
    return true;
  }
  return false;
}

// RTD pin diagnostic: sample pin 2 under pull-up, pull-down and floating.
// What drives the pin (or doesn't) shows in the pattern:
//   PU:H PD:L FL:L  -> pin sees nothing (open circuit at R18/J12/wrong cavity)
//   PU:L PD:L       -> pin pulled to GND (button to GND working)
//   PU:H PD:H       -> pin driven high externally (button wired to 3.3V/5V?)
static void rtdPinDiagnostic() {
  char result[3];
  const uint8_t modes[] = {INPUT_PULLUP, INPUT_PULLDOWN, INPUT};
  for (int i = 0; i < 3; i++) {
    pinMode(RTD_BUTTON_PIN, modes[i]);
    delay(50);  // C10 (0.1uF) settle through weak pulls
    result[i] = digitalRead(RTD_BUTTON_PIN) == HIGH ? 'H' : 'L';
  }
  pinMode(RTD_BUTTON_PIN, INPUT_PULLUP);  // restore normal mode
  delay(50);

  char msg[48];
  snprintf(msg, sizeof(msg), ">> RTDdiag PU:%c PD:%c FL:%c",
           result[0], result[1], result[2]);
  Serial.println(msg);
}

// Hold-to-toggle: after `button` is held PRECHARGE_HOLD_MS, toggle
// PRECHARGE_EN once; requires release before it can toggle again.
static void holdToTogglePrecharge(DebouncedButton &button, HoldTracker &h,
                                  bool stateChanged) {
  if (stateChanged) {
    if (button.pressed) {
      h.holdStart = millis();
      Serial.print(">> ");
      Serial.print(h.name);
      Serial.println(" PRESSED - hold 3s to toggle precharge");
    } else {
      if (!h.consumed && h.holdStart != 0) {
        Serial.print(">> ");
        Serial.print(h.name);
        Serial.println(" RELEASED EARLY - precharge unchanged");
      }
      h.holdStart = 0;
      h.consumed = false;
    }
  }

  if (!button.pressed || h.consumed || h.holdStart == 0) return;

  uint32_t held = millis() - h.holdStart;
  if (held >= PRECHARGE_HOLD_MS) {
    prechargeEnabled = !prechargeEnabled;
    digitalWrite(PRECHARGE_EN_PIN, prechargeEnabled ? HIGH : LOW);
    Serial.println(prechargeEnabled ? ">> PRECHARGE_EN HIGH - precharge ENABLED"
                                    : ">> PRECHARGE_EN LOW - precharge DISABLED");
    h.consumed = true;  // no repeat until released
  } else if (millis() - h.lastPrint > 500) {
    h.lastPrint = millis();
    char msg[48];
    snprintf(msg, sizeof(msg), ">> %s holding %lu.%lus / 3.0s",
             h.name, held / 1000, (held % 1000) / 100);
    Serial.println(msg);
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogReadAveraging(4);

  pinMode(AUX1_BUTTON_PIN, INPUT_PULLUP);
  pinMode(AUX2_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RTD_BUTTON_PIN, INPUT_PULLUP);

  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(RTD_SPEAKER_PIN, OUTPUT);
  pinMode(PRECHARGE_EN_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
  digitalWrite(RTD_SPEAKER_PIN, LOW);
  digitalWrite(PRECHARGE_EN_PIN, LOW);

  Serial.println("=== BENCH I/O TEST ===");
  Serial.println("Brake pedal = brake light | AUX1 hold = speaker | RTD or AUX2 hold 3s = toggle precharge");
  Serial.println("RTDdiag legend: PU:H PD:L FL:L = pin open | PU:L PD:L = grounded | PU:H PD:H = driven high");
  Serial.println("Hold RTD button down through a diag cycle and compare to unpressed.");
}

void loop() {
  // --- Brake light follows brake pedal pressure ---
  static bool brakeLightOn = false;
  int brakeF = analogRead(BRAKE_FRONT_PIN);
  int brakeR = analogRead(BRAKE_REAR_PIN);
  if (!brakeLightOn &&
      (brakeF > BRAKE_LIGHT_ON_COUNTS || brakeR > BRAKE_LIGHT_ON_COUNTS)) {
    brakeLightOn = true;
    digitalWrite(BRAKE_LIGHT_PIN, HIGH);
    Serial.println(">> BRAKE LIGHT ON");
  } else if (brakeLightOn &&
             brakeF < BRAKE_LIGHT_OFF_COUNTS && brakeR < BRAKE_LIGHT_OFF_COUNTS) {
    brakeLightOn = false;
    digitalWrite(BRAKE_LIGHT_PIN, LOW);
    Serial.println(">> BRAKE LIGHT OFF");
  }

  // --- Speaker follows AUX1 ---
  if (updateButton(aux1)) {
    digitalWrite(RTD_SPEAKER_PIN, aux1.pressed ? HIGH : LOW);
    Serial.println(aux1.pressed ? ">> RTD SPEAKER ON" : ">> RTD SPEAKER OFF");
  }

  // --- RTD pin diagnostic every 2 s ---
  static uint32_t lastDiag = 0;
  if (millis() - lastDiag >= 2000) {
    lastDiag = millis();
    rtdPinDiagnostic();
    rtd.lastEdge = millis();  // let debounce re-settle after mode switching
    rtd.lastRaw = digitalRead(RTD_BUTTON_PIN) == LOW;
  }

  // --- RTD button: hold 3 s to toggle precharge ---
  holdToTogglePrecharge(rtd, rtdHold, updateButton(rtd));

  // --- AUX2 stand-in: same behaviour, kept while RTD harness is verified ---
  holdToTogglePrecharge(aux2, aux2Hold, updateButton(aux2));

  // --- Raw sensor prints at 10 Hz ---
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();
    char line[144];
    snprintf(line, sizeof(line),
             "APPS1:%4d  APPS2:%4d  BRAKE_F:%4d  BRAKE_R:%4d  | AUX1:%s AUX2:%s RTD:%s(pin%s) | PRECHARGE:%s",
             analogRead(APPS1_PIN), analogRead(APPS2_PIN),
             analogRead(BRAKE_FRONT_PIN), analogRead(BRAKE_REAR_PIN),
             aux1.pressed ? "PRESSED" : "released",
             aux2.pressed ? "PRESSED" : "released",
             rtd.pressed ? "PRESSED" : "released",
             digitalRead(RTD_BUTTON_PIN) == LOW ? "LOW" : "HIGH",
             prechargeEnabled ? "ON" : "off");
    Serial.println(line);
  }
}
