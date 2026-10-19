const int P1_PIN = A0;
const int P2_PIN = A1;

#define RELAY1 7

const unsigned long SAMPLE_INTERVAL_US = 5000;

unsigned long lastSampleTime = 0;
unsigned long startTime = 0;

bool relayState = false;

void setup() {
  Serial.begin(115200);

  pinMode(RELAY1, OUTPUT);
  pinMode(P1_PIN, INPUT);
  pinMode(P2_PIN, INPUT);

  digitalWrite(RELAY1, LOW);

  startTime = micros();
  lastSampleTime = micros();
  Serial.println("time_us,P1,P2");
}

void loop() {

  // Read command from Serial Monitor
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command == "ON") {
      relayState = true;
      digitalWrite(RELAY1, HIGH);
    }
    else if (command == "OFF") {
      relayState = false;
      digitalWrite(RELAY1, LOW);
    }
  }

  // Sensor data sampling
  unsigned long now = micros();

  if ((unsigned long)(now - lastSampleTime)
      >= SAMPLE_INTERVAL_US) {

    lastSampleTime = now;

    int p1Value = analogRead(P1_PIN);
    int p2Value = analogRead(P2_PIN);

    unsigned long elapsedUs = now - startTime;

    Serial.print(elapsedUs);
    Serial.print(",");
    Serial.print(p1Value);
    Serial.print(",");
    Serial.println(p2Value);
  }
}