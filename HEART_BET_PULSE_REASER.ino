#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int PULSE_PIN  = A0;
const int BUZZER_PIN = 8;
const int LED_PIN    = 7;

/* ---------- heartbeat variables ---------- */
volatile int  BPM;
volatile int  Signal;
volatile int  IBI = 600;
volatile bool Pulse = false;
volatile bool QS    = false;
volatile bool lostPulse = false;

volatile int  rate[10];
volatile unsigned long sampleCounter = 0, lastBeatTime = 0;
volatile int  P = 512, T = 512, thresh = 525, amp = 100;
volatile bool firstBeat = true, secondBeat = false;

/* ---------- state machine ---------- */
enum CounterState { WAITING, QUALIFYING, MONITORING, SHOW_RESULT };
CounterState state = WAITING;

int  consecutiveBeats = 0;
int  beatsInMinute    = 0;
unsigned long windowStart   = 0;
unsigned long lastSecondRef = 0;
unsigned long resultShownAt = 0;

/* ---------- helpers ---------- */
void printPadded(int value, int width) {
  lcd.print(value);
  int len = (value == 0) ? 1 : (int)log10(value) + 1;
  for (int i = len; i < width; i++) lcd.print(' ');
}

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Place finger...");
  Serial.println(F("State: WAITING – place finger on sensor"));

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  interruptSetup();
}

void loop() {

  /* ----- lost pulse reset ----- */
  if (lostPulse) {
    lostPulse = false;
    state = WAITING;
    consecutiveBeats = beatsInMinute = 0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Place finger...");
    Serial.println(F("Pulse lost → RESET. State: WAITING"));
  }

  /* ----- beat detected ----- */
  if (QS) {
    QS = false;

    /* visual + audible flash */
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(20);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    switch (state) {

      case WAITING:
        state = QUALIFYING;
        consecutiveBeats = 1;
        lcd.clear();
        lcd.print("Qualify: 1/10");
        Serial.println(F("State: QUALIFYING – beat 1/10"));
        break;

      case QUALIFYING:
        consecutiveBeats++;
        lcd.setCursor(9,0);
        printPadded(consecutiveBeats, 2);
        Serial.print(F("QUALIFYING – beat "));
        Serial.print(consecutiveBeats);
        Serial.println(F("/10"));
        if (consecutiveBeats >= 10) {
          state = MONITORING;
          beatsInMinute = 1;
          windowStart   = millis();
          lastSecondRef = millis();
          lcd.clear();
          lcd.print("Beats: 1   ");
          lcd.setCursor(0,1);
          lcd.print("Sec: 0   ");
          Serial.println(F("State: MONITORING – counting for 60 s"));
        }
        break;

      case MONITORING:
        beatsInMinute++;
        lcd.setCursor(7,0);
        printPadded(beatsInMinute, 3);
        Serial.print(F("MONITORING – beat "));
        Serial.print(beatsInMinute);
        Serial.print(F(", time "));
        Serial.print((millis() - windowStart)/1000);
        Serial.println(F(" s"));
        break;

      default:
        break;
    }
  }

  /* ----- 1 Hz second counter on LCD + serial ----- */
  if (state == MONITORING && millis() - lastSecondRef >= 1000UL) {
    lastSecondRef += 1000UL;
    int secPassed = (millis() - windowStart) / 1000;
    lcd.setCursor(5,1);
    printPadded(secPassed, 2);

    Serial.print(F("Elapsed: "));
    Serial.print(secPassed);
    Serial.println(F(" s"));
  }

  /* ----- 60‑second window finished ----- */
  if (state == MONITORING && millis() - windowStart >= 60000UL) {
    state = SHOW_RESULT;
    resultShownAt = millis();
    lcd.clear();
    lcd.print("Beats/min:");
    lcd.setCursor(0,1);
    lcd.print(beatsInMinute);

    Serial.print(F("RESULT – Beats in one minute: "));
    Serial.println(beatsInMinute);
  }

  /* ----- result shown for 5 s, then reset ----- */
  if (state == SHOW_RESULT && millis() - resultShownAt >= 5000UL) {
    state = WAITING;
    consecutiveBeats = beatsInMinute = 0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Place finger...");
    Serial.println(F("State: WAITING – ready for next reading"));
  }

  delay(20);
}

/* ---------- Timer2 interrupt every 2 ms ---------- */
ISR(TIMER2_COMPA_vect) {
  cli();
  Signal = analogRead(PULSE_PIN);
  sampleCounter += 2;
  int N = sampleCounter - lastBeatTime;

  if (Signal < thresh && N > (IBI / 5) * 3) { if (Signal < T) T = Signal; }
  if (Signal > thresh && Signal > P) { P = Signal; }

  if (N > 250) {
    if ((Signal > thresh) && !Pulse && (N > (IBI / 5) * 3)) {
      Pulse = true;
      IBI = sampleCounter - lastBeatTime;
      lastBeatTime = sampleCounter;

      if (secondBeat) {
        secondBeat = false;
        for (int i=0; i<10; i++) rate[i] = IBI;
      }
      if (firstBeat) {
        firstBeat  = false;
        secondBeat = true;
        sei();
        return;
      }

      long runTot = 0;
      for (int i=0; i<9; i++) { rate[i] = rate[i+1]; runTot += rate[i]; }
      rate[9]  = IBI;  runTot += rate[9];
      BPM = 60000 / (runTot / 10);
      QS  = true;
    }
  }

  if (Signal < thresh && Pulse) {
    Pulse = false;
    amp   = P - T;
    thresh = amp/2 + T;
    P = thresh;  T = thresh;
  }

  if (N > 2500) {
    thresh = P = T = 512;
    lastBeatTime = sampleCounter;
    firstBeat = true;  secondBeat = false;
    lostPulse = true;
  }
  sei();
}

/* ---------- Timer2 config for 2 ms period ---------- */
void interruptSetup() {
  TCCR2A = 0x02;            // CTC
  TCCR2B = 0x06;            // prescaler 256
  OCR2A  = 0x7C;            // 124 → 2 ms at 16 MHz
  TIMSK2 = 0x02;            // enable compare‑A interrupt
  sei();
}
