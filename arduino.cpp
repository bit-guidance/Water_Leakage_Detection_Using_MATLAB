
const int P1_PIN = A0;
const int P2_PIN = A1;

const unsigned long SAMPLE_INTERVAL_US = 5000; // 5 ms

unsigned long lastSampleTime = 0;
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);

  pinMode(P1_PIN, INPUT);
  pinMode(P2_PIN, INPUT);

  startTime = micros();
  lastSampleTime = micros();

  // CSV header for the serial terminal / MATLAB
  Serial.println("time_us,P1,P2");
}

void loop() {
  unsigned long now = micros();

  if ((unsigned long)(now - lastSampleTime) >= SAMPLE_INTERVAL_US) {
    lastSampleTime = now;

    // Read both sensors in the same acquisition cycle
    int p1Value = analogRead(P1_PIN);
    int p2Value = analogRead(P2_PIN);

    unsigned long elapsedUs = now - startTime;

    // One complete line per sample
    Serial.print(elapsedUs);
    Serial.print(",");
    Serial.print(p1Value);
    Serial.print(",");
    Serial.println(p2Value);
  }
}
