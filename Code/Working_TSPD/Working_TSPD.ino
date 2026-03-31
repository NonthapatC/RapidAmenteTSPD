#include <Preferences.h>
Preferences prefs;

#define PWM_TIMEOUT_US 50000
#define PWM_TOl 1

#define MEDIAN_SIZE 25 // Use an odd number for a "true" middle element

#define interpolRes 21 //number of points used for interpolation and calibration
#define calTol 0.5 //PWM duty tolerance that gets average in during calibration
#define calSample 50 //number of samples needed per interpolation point
#define calPointTO 60000 //timeout for point calibration
#define calMinMaxTO 120000

//**NVS data
struct __attribute__((packed)) dataPackage { 
  float pwmMin;
  float pwmMax;
  int TPS1Min;
  int TPS1Max;
  int TPS2Min;
  int TPS2Max;

  float TPS1_Table[interpolRes];
  float TPS2_Table[interpolRes];
  bool calValid[interpolRes];
};
//**NVS data

dataPackage calData;

const int PWM_PIN = 27;
const int TPS1_PIN = 34; //TPS1 is actually on pin 32 but switch due to sketchy behavior
//const int TPS2_PIN = 34;

const int RLED = 16;
const int YLED = 18;
const int GLED = 17;

const int button_1 = 19;
const int button_2 = 21;

const int relay = 25;

const bool INVERTED_PWM = true; // Set to true if the signal is inverted (Low = Active), false if standard (High = Active)

int ledState = HIGH; //for blink
unsigned long previousMillis = 0; //for blink

//relay trip settings and var
float tripPoint = 10.0f;
bool allowTrip = true;

const int durationToTrip = 1000;
int tripPrevMillis = 0;

//button debounce
int lastSteadyState = LOW;       // The "confirmed" state
int lastFlickerState = LOW;      // The last state we saw
unsigned long lastDebounceTime = 0; // The last time the state flickered

volatile uint32_t lastRise = 0;
volatile uint32_t highTime = 0;
volatile uint32_t period = 0;
volatile uint32_t lastEdgeTime = 0;
volatile bool newSample = false;

float freq = 0;
float duty = 0;

template <typename T>
struct MedianFilter {
  T samples[MEDIAN_SIZE];
  int bufIndex = 0;
  bool filled = false;

  // Add this part!
  void reset() {
    bufIndex = 0;
    filled = false;
    for (int i = 0; i < MEDIAN_SIZE; i++) {
      samples[i] = 0;
    }
  }

  T update(T newValue) {
    samples[bufIndex] = newValue;
    bufIndex = (bufIndex + 1) % MEDIAN_SIZE;
    if (bufIndex == 0) filled = true;

    int n = filled ? MEDIAN_SIZE : bufIndex;
    T temp[MEDIAN_SIZE];

    for (int i = 0; i < n; i++) temp[i] = samples[i];

    for (int i = 1; i < n; i++) {
      T key = temp[i];
      int j = i - 1;
      while (j >= 0 && temp[j] > key) {
        temp[j + 1] = temp[j];
        j--;
      }
      temp[j + 1] = key;
    }

    if (n % 2 != 0) {
      return temp[n / 2];
    } else {
      return (temp[n / 2] + temp[(n / 2) - 1]) / 2.0; // 2.0 ensures float math
    }
  }
};

MedianFilter<float> PWMduty;
MedianFilter<int> TPS1;
//MedianFilter<int> TPS2;

void IRAM_ATTR pwmISR() {
  uint32_t now = micros();
  bool level = digitalRead(PWM_PIN);

  if (INVERTED_PWM) level = !level;

  lastEdgeTime = now;

  if (level) {
    // Rising edge
    period = now - lastRise;
    lastRise = now;
  } else {
    // Falling edge
    highTime = now - lastRise;
    newSample = true;
  }
}

float measurePWM() {
  uint32_t now = micros();
  uint32_t p, h, t;
  bool sample;

  noInterrupts();
  p = period;
  h = highTime;
  t = lastEdgeTime;
  sample = newSample;
  newSample = false;
  interrupts();
  // ---------- Normal PWM case ----------
  if (sample && p > 0) {
    freq = 1e6f / p;
    duty = (h * 100.0f) / p;
  }

  // ---------- 0% or 100% duty fallback ----------
  else if (now - t > PWM_TIMEOUT_US) {
    bool level = digitalRead(PWM_PIN);
    
    if (INVERTED_PWM) {
      duty = level ? 0.0f : 100.0f;
    } else {
      duty = level ? 100.0f : 0.0f;
    }
  }
  if (duty > 100.5f || duty < 00.0f) {
    Serial.print("**ERROR** PWM out of range ");
    Serial.println(duty);
    return NAN;
  }else{
    return duty;
  }
}

bool calibratePoint(int indx, float *result) {
  float target = (calData.pwmMax-calData.pwmMin)*indx/(interpolRes-1) + calData.pwmMin;
  float TPS1sum = 0;
  float count = 0;

  unsigned long startTime = millis();
  while (millis() - startTime < calPointTO) {
    float rawPWM = measurePWM();

    if (isnan(rawPWM)){
      continue;
    }

    float filteredPWM = PWMduty.update(rawPWM);
    int filteredTPS1 = TPS1.update(analogRead(TPS1_PIN));
    

    Serial.print("Duty: ");
    Serial.print(filteredPWM, 2);
    Serial.print(" % | ");
    Serial.print("TPS1: ");
    Serial.println(filteredTPS1);

    if(filteredPWM < target - calTol){
      digitalWrite(RLED, HIGH);
      digitalWrite(YLED, LOW);
      digitalWrite(GLED, HIGH);
    }
    if(filteredPWM > target + calTol){
      digitalWrite(YLED, HIGH);
      digitalWrite(RLED, LOW);
      digitalWrite(GLED, HIGH);
    }

    if (abs(filteredPWM-target) <= calTol) {
      PWMduty.reset();
      TPS1.reset();

      for(int i = 0; i < MEDIAN_SIZE; i++) {
          float p = measurePWM();
          if(isnan(p)) { i--; continue; }
          PWMduty.update(p);
          TPS1.update(analogRead(TPS1_PIN));
      }
      TPS1sum += filteredTPS1;
      count++;

      digitalWrite(RLED, HIGH);
      digitalWrite(YLED, HIGH);
      digitalWrite(GLED, LOW);

      if (count >= calSample) {
        *result = TPS1sum/count;
        return true;
      }
    }
  }
  return false;
}

bool calibrateAll() {
  Serial.println("getting PWM min max");

  float initialPWM = measurePWM();

  while(isnan(initialPWM)){
    initialPWM = measurePWM(); // Wait for first valid signal
    if(millis() > calPointTO) return false; // Safety timeout
  }

  float filteredPWM = PWMduty.update(initialPWM);
  int filteredTPS1 = TPS1.update(analogRead(TPS1_PIN));

  calData.pwmMin = filteredPWM;
  calData.pwmMax = filteredPWM;

  calData.TPS1Min = filteredTPS1;
  calData.TPS1Max = filteredTPS1;

  float candPwmMin = filteredPWM, candPwmMax = filteredPWM;
  int candTpsMin = filteredTPS1, candTpsMax = filteredTPS1;
  
  int minCount = 0;
  int maxCount = 0;
  const int STABILITY_THRESHOLD = 15;

  bool hitES = false;
  int countES = 0;

  unsigned long startTime = millis();
  while(countES <= 5){
    float rawPWM = measurePWM();
    if(isnan(rawPWM)){
      continue;
    }
    
    filteredPWM = PWMduty.update(rawPWM);
    filteredTPS1 = TPS1.update(analogRead(TPS1_PIN));

    if(millis() - startTime > calMinMaxTO){
      Serial.println("Min max timed out");
      delay(5000);
      return false;
    }

    float range = calData.pwmMax - calData.pwmMin;
    float lowThreshold = calData.pwmMin + (range * 0.1f);  // Bottom 10%
    float highThreshold = calData.pwmMax - (range * 0.1f); // Top 10%
    float middleZoneHigh = calData.pwmMax - (range * 0.3f);
    float middleZoneLow = calData.pwmMin + (range * 0.3f);

    // Trigger count when hitting the LOW end
    if(filteredPWM <= lowThreshold && !hitES) {
      countES++;
      hitES = true;
      Serial.printf(">>> HIT LOW (Stroke %d)\n", countES);
    } 
    // Trigger count when hitting the HIGH end
    else if(filteredPWM >= highThreshold && !hitES) {
      countES++;
      hitES = true;
      Serial.printf(">>> HIT HIGH (Stroke %d)\n", countES);
    }
    // RESET the flag when in the middle 40% of the travel
    else if(filteredPWM > middleZoneLow && filteredPWM < middleZoneHigh) {
      if(hitES) {
        Serial.println(">>> Back in Middle Zone (Ready for next stroke)");
        hitES = false;
      }
    }

    // --- PROTECTED MINIMUMS (PWM & TPS) ---
    if (filteredPWM < calData.pwmMin) {
      // Check if both PWM and TPS are stable at this new low
      if (abs(filteredPWM - candPwmMin) < calTol && abs(filteredTPS1 - candTpsMin) < 5) {
        minCount++;
      } else {
        candPwmMin = filteredPWM;
        candTpsMin = filteredTPS1;
        minCount = 0;
      }

      if (minCount >= STABILITY_THRESHOLD) {
        calData.pwmMin = candPwmMin;
        calData.TPS1Min = candTpsMin;
        minCount = 0;
        Serial.printf("NEW MIN: PWM %.2f | TPS %d\n", calData.pwmMin, calData.TPS1Min);
      }
    } else {
      minCount = 0;
    }

    // --- PROTECTED MAXIMUMS (PWM & TPS) ---
    if (filteredPWM > calData.pwmMax) {
      if (abs(filteredPWM - candPwmMax) < calTol && abs(filteredTPS1 - candTpsMax) < 5) {
        maxCount++;
      } else {
        candPwmMax = filteredPWM;
        candTpsMax = filteredTPS1;
        maxCount = 0;
      }

      if (maxCount >= STABILITY_THRESHOLD) {
        calData.pwmMax = candPwmMax;
        calData.TPS1Max = candTpsMax;
        maxCount = 0;
        Serial.printf("NEW MAX: PWM %.2f | TPS %d\n", calData.pwmMax, calData.TPS1Max);
      }
    } else {
      maxCount = 0;
    }

    Serial.print(countES);
    Serial.print(" Duty: ");
    Serial.print(filteredPWM, 2);
    Serial.print(" pwmMin: ");
    Serial.print(calData.pwmMin);
    Serial.print(" pwmMax: ");
    Serial.println(calData.pwmMax);

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 1000 / (2 * countES)) {
      // Save the last time the LED blinked
      previousMillis = currentMillis;

      // Toggle the LED state
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }

      digitalWrite(GLED, ledState);
    }
  }

  Serial.println("Min max cal completed");
  Serial.print("pwmMin: ");
  Serial.print(calData.pwmMin);
  Serial.print(" | pwmMax: ");
  Serial.print(calData.pwmMax);

  Serial.print(" | TPS1Min: ");
  Serial.print(calData.TPS1Min);
  Serial.print(" | TPS1Max: ");
  Serial.println(calData.TPS1Max);
  delay(5000);

  PWMduty.reset();
  TPS1.reset();

  for(int i=0; i < interpolRes; i++){
    Serial.print("Calibrating indx: ");
    Serial.println(i);

    if(calibratePoint(i, &calData.TPS1_Table[i])){
      calData.calValid[i] = true;
      Serial.print("Indx: ");
      Serial.print(i);
      Serial.println(" complete");
      delay(500);
    }else{
      calData.calValid[i] = false;
      Serial.print("Indx: ");
      Serial.print(i);
      Serial.println(" failed");
      delay(5000);
    }
    Serial.print("Saved: ");
    Serial.println(calData.TPS1_Table[i]);
  }

  prefs.begin("appData", false);
  prefs.putBytes("calSetting", &calData, sizeof(calData));
  prefs.end();

  for(int i=0; i < interpolRes; i++)
  {
    Serial.print(calData.TPS1_Table[i]);
    Serial.print(" | ");
  }

  digitalWrite(GLED, LOW);

  delay(20000);

  digitalWrite(RLED, HIGH);
  digitalWrite(GLED, HIGH);
  digitalWrite(YLED, HIGH);
  return true;
}

float interpolate(float pwm) {
  float STEP_PERCENT = (calData.pwmMax-calData.pwmMin)/(interpolRes-1);
  pwm = constrain(pwm, 0.0, 100.0);

  int idx = int(pwm - calData.pwmMin) / STEP_PERCENT;
  if (idx >= interpolRes - 1)
      return calData.TPS1_Table[interpolRes - 1];

  if (!calData.calValid[idx] || !calData.calValid[idx + 1])
      return NAN;  // don’t lie with bad data

  float x0 = idx * STEP_PERCENT + calData.pwmMin;

  float y0 = calData.TPS1_Table[idx];
  float y1 = calData.TPS1_Table[idx + 1];

  return y0 + (pwm - x0) * (y1 - y0) / STEP_PERCENT;
}

void printSavedData() {
  Serial.println("\n--- NVS CALIBRATION DATA DUMP ---");
  
  Serial.printf("PWM Range:  %.2f%% to %.2f%%\n", calData.pwmMin, calData.pwmMax);
  Serial.printf("TPS1 Range: %d to %d\n", calData.TPS1Min, calData.TPS1Max);
  Serial.printf("TPS2 Range: %d to %d\n", calData.TPS2Min, calData.TPS2Max);
  
  Serial.println("\nInterpolation Table (TPS1):");
  Serial.println("Idx | Valid | PWM Value | TPS1 Value");
  Serial.println("------------------------------------");

  float step = (calData.pwmMax - calData.pwmMin) / (interpolRes - 1);

  for (int i = 0; i < interpolRes; i++) {
    float currentPWM = calData.pwmMin + (i * step);
    
    Serial.print(i);
    Serial.print("   | ");
    Serial.print(calData.calValid[i] ? "YES" : "NO ");
    Serial.print("   | ");
    Serial.print(currentPWM, 2);
    Serial.print("%    | ");
    Serial.println(calData.TPS1_Table[i]);
  }
  
  Serial.println("------------------------------------");
  Serial.printf("Struct Size: %u bytes\n", sizeof(calData));
  Serial.println("--- END DUMP ---\n");
}

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, INPUT);
  pinMode(TPS1_PIN, INPUT);

  pinMode(relay, OUTPUT);

  digitalWrite(relay, LOW);

  pinMode(RLED, OUTPUT);
  pinMode(YLED, OUTPUT);
  pinMode(GLED, OUTPUT);

  pinMode(button_1, INPUT);
  pinMode(button_2, INPUT);

  digitalWrite(RLED, HIGH);
  digitalWrite(YLED, HIGH);
  digitalWrite(GLED, HIGH);

  attachInterrupt(digitalPinToInterrupt(PWM_PIN), pwmISR, CHANGE);
  
  bool savedValid = 0;

  delay(1000);

  prefs.begin("appData", true);

  if (prefs.isKey("calSetting")) {
    // 2. Check if the saved size matches our current struct size
    if (prefs.getType("calSetting") == PT_BLOB && prefs.getBytesLength("calSetting") == sizeof(calData)) {
      prefs.getBytes("calSetting", &calData, sizeof(calData));
      savedValid = 1;
      Serial.println("Saved config loaded succesfully");
    }else {
      Serial.println("Saved config size mismatch");
    }
  }else {
    Serial.println("No saved config found");
  }
  prefs.end();

  int b1_counter = 0;
  int b2_counter = 0;

  bool userInitCal = 0;

  unsigned long windowStart = millis();
  Serial.println("\n--- 5 SECOND SETUP WINDOW ---");
  Serial.println("Hold B1: Toggle Force Calibration");
  Serial.println("Hold B2: Toggle Safe Mode (Trip Disable)");

  while (millis() - windowStart < 2500) {
    int b1_raw = digitalRead(button_1);
    int b2_raw = digitalRead(button_2);
    
    if (b1_raw == HIGH){
      b1_counter++;
    }else{
      b1_counter = 0;
    }

    if (b1_counter > 50) {
      userInitCal = 1;
      digitalWrite(RLED, (millis() / 100) % 2); // Visual confirmation
      b1_counter = 0;
      Serial.println("user init calc");
    }

    if (b2_raw == HIGH){
      b2_counter++;
    }else{
      b2_counter = 0;
    }

    if (b2_counter > 50) {
      allowTrip = false;
      digitalWrite(GLED, LOW);
      b2_counter = 0;
      Serial.println("safe mode on");
    }

    digitalWrite(YLED, (millis() / 250) % 2);

    if(userInitCal){
      digitalWrite(RLED, (millis() / 100) % 2); // Visual confirmation
    }

    delay(1);
  }

  digitalWrite(RLED, HIGH);
  digitalWrite(YLED, HIGH);

  if(savedValid){
    printSavedData();
    //delay(15000);
  }

  if(!savedValid){
    Serial.println("save invalid initating calibration");
    //delay(1000);
    calibrateAll();
  }else if(userInitCal){
    Serial.println("User initated calibration");
    //delay(1000);
    calibrateAll();
  }

  digitalWrite(RLED, HIGH);
  digitalWrite(YLED, HIGH);

  if(allowTrip){
    digitalWrite(GLED, HIGH);
  }else{
    digitalWrite(GLED, LOW);
  }

  digitalWrite(YLED, LOW);
}

void loop() {
  float rawPWM = measurePWM();
  float filteredPWM = PWMduty.update(rawPWM);
  int rawTPS1 = analogRead(TPS1_PIN);
  int filteredTPS1 = TPS1.update(rawTPS1);

  Serial.print("Duty: ");
  Serial.print(rawPWM, 2);
  Serial.print(" % | ");
  Serial.print("TPS1: ");
  Serial.print(filteredTPS1);
  // Serial.print(" | TPS2: ");
  // Serial.println(filteredTPS2);
  Serial.print(" Target TPS1: ");
  float interpolatedTPS1 = interpolate(rawPWM);
  Serial.print(interpolatedTPS1);
  Serial.print(" Delta: ");
  float delta = abs(float(rawTPS1)-interpolatedTPS1) * 100/(float(calData.TPS1Max)-float(calData.TPS1Min));
  Serial.println(delta);

  if (!isnan(delta) && (delta > tripPoint) && allowTrip){
    if (tripPrevMillis == 0) {
      tripPrevMillis = millis();
      Serial.println("violation detected");
    }else if (millis() - tripPrevMillis > durationToTrip){
      digitalWrite(relay, HIGH);
      Serial.println("TSPD Tripped");
      while(1){
        digitalWrite(RLED, (millis() / 250) % 2);
      }
    }
  }else if (!isnan(delta) && (delta <= tripPoint)) {
    tripPrevMillis = 0;
    Serial.println("timer cleared");
  }

  // Button 2
  int currentState = digitalRead(button_2);

  // 1. Debounce logic
  if (currentState != lastFlickerState) {
    lastDebounceTime = millis();
    lastFlickerState = currentState;
  }

  // 2. Check if signal is steady
  if ((millis() - lastDebounceTime) > 100) {
    
    // 3. Detect the "Press" event (Rising Edge)
    if (lastSteadyState == LOW && currentState == HIGH) {
      
      // --- NEW COMBO CHECK ---
      if (digitalRead(button_1) == HIGH) {
        Serial.println("safeMode");
        allowTrip = !allowTrip;
        if(allowTrip){
          digitalWrite(GLED, HIGH);
        } else{
          digitalWrite(GLED, LOW);
        }
      } 
      // --- NORMAL LOGIC ---
      else {
        if(tripPoint == 10.0f){
          tripPoint = 15.0f;
          digitalWrite(YLED, HIGH);
          digitalWrite(RLED, LOW);
          Serial.println("Trip point: 15%");
        } else if(tripPoint == 15.0f){
          tripPoint = 20.0f;
          digitalWrite(YLED, LOW);
          digitalWrite(RLED, LOW);
          Serial.println("Trip point: 20%");
        } else if(tripPoint == 20.0f){
          tripPoint = 10.0f;
          digitalWrite(YLED, LOW);
          digitalWrite(RLED, HIGH);
          Serial.println("Trip point: 10%");
        }
      }
    }
    lastSteadyState = currentState;
  }
}
