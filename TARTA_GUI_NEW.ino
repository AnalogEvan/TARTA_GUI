#include <SoftwareSerial.h>

// Pin Definitions
#define PUMP_PIN 4        // Pin for controlling the pump
#define RELAY_PIN 8       // Pin for controlling the relay (spark ignition)
#define BOOSTC_PIN 9      // Pin for controlling the boost circuit
// PM_SENSOR_PIN is unused as we use Serial1

// Timing Constants
const unsigned long SPARK_DURATION           = 5200;   // Total duration of a single spark (ms)
const unsigned long BOOST_DELAY              = 200;    // Delay before activating boost (ms)
const unsigned long INTER_SPARK_DELAY        = 1000;   // Delay between sparks (ms)
const unsigned long STATUS_INTERVAL          = 1000;   // Status update interval (ms)
const unsigned long PM_SAMPLE_ACCUM_INTERVAL = 5000; // Interval to grab a sample for the 1-min average
const unsigned long PM_MINUTE_INTERVAL       = 60000;  // 1-minute averaging window (ms)

// State Variables
bool scanning       = false;
bool cleaning       = false;
bool pmMonitoring   = false;
bool pmSparkPhase   = false;
bool pumpActive     = false;
bool sparkActive    = false;

unsigned long samplingDuration   = 0; // Scan pump run duration (ms)
unsigned long pumpStartTime      = 0;
unsigned long lastStatusTime     = 0;
unsigned long nextSparkTime      = 0;
unsigned long sparkStartTime     = 0;
unsigned long lastPMAccumTime    = 0; // Timer for accumulating samples
unsigned long pmMinuteStartTime  = 0;

int sparksPerCycle   = 0;
int totalCycles      = 0;
int cyclesRemaining  = 0;
int cleanSparks      = 0;
int totalSparks      = 0;
int currentSpark     = 0;
int pmThreshold      = 0;
float pmType         = 2.5; // Default to 2.5

// PM Sensor raw values - updated continuously
volatile uint16_t pm1_0_standard = 0;
volatile uint16_t pm2_5_standard = 0;
volatile uint16_t pm10_standard  = 0;

// Serial command buffer
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

// PM Monitoring accumulators - MODIFIED for cumulative totals
unsigned long pmMinuteSum   = 0;     // Sum for current 1-minute window
unsigned int pmMinuteCount  = 0;     // Count for current 1-minute window
float pmCumulativeTotal     = 0.0;   // Running total of all 1-minute averages

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // PM sensor serial
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BOOSTC_PIN, OUTPUT);
  // PM_SENSOR_PIN is not used for serial communication
  safetyShutoff();
  Serial.println("Arduino Ready");
}

/**
 * @brief Continuously processes incoming bytes from the PM sensor.
 * This should be called in every loop() to prevent the serial buffer from overflowing
 * and to ensure data is processed in real-time.
 */
void processPMSensorData() {
  static uint8_t buffer[32];
  static uint8_t frameLen = 0;

  while (Serial1.available()) {
    uint8_t c = Serial1.read();

    // 1. Look for the first start byte
    if (frameLen == 0 && c == 0x42) {
      buffer[0] = c;
      frameLen = 1;
      continue;
    }
    
    // 2. Look for the second start byte
    if (frameLen == 1) {
      if (c == 0x4D) {
        buffer[1] = c;
        frameLen = 2;
      } else {
        frameLen = 0; // Not a valid frame, reset
      }
      continue;
    }
    
    // 3. Fill the rest of the buffer
    if (frameLen > 1 && frameLen < 32) {
      buffer[frameLen++] = c;
      
      // 4. Once full, validate and parse the frame
      if (frameLen == 32) {
        // Checksum validation
        uint16_t sum = 0;
        for (int i = 0; i < 30; i++) sum += buffer[i];
        uint16_t recv_sum = (buffer[30] << 8) | buffer[31];

        if (sum == recv_sum) {
          // Checksum is good, update global PM values
          pm1_0_standard = (buffer[10] << 8) | buffer[11];
          pm2_5_standard = (buffer[12] << 8) | buffer[13];
          pm10_standard  = (buffer[14] << 8) | buffer[15];
        }
        frameLen = 0; // Reset for the next frame
      }
    }
  }
}


void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            if (ndx < numChars - 1) {
                receivedChars[ndx] = rc;
                ndx++;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // Null-terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void loop() {
  // 1. Always process incoming sensor data to keep globals updated
  processPMSensorData();

  // 2. Check for and process incoming commands from the host
  recvWithEndMarker();
  if (newData == true) {
    handleCommand(receivedChars);
    newData = false;
  }

  // 3. Periodic status updates for active modes
  if ((scanning || cleaning || pmMonitoring) && (millis() - lastStatusTime >= STATUS_INTERVAL)) {
    sendStatusUpdate();
    lastStatusTime = millis();
  }

  // 4. Run the state machine for the current mode
  if (scanning) {
    handleScanning();
  } else if (cleaning) {
    handleCleaning();
  } else if (pmMonitoring) {
    handlePMSensor();
  }
}

void handleCommand(char* command) {
  if (scanning || cleaning || pmMonitoring) {
    if (strcmp(command, "CANCEL") != 0) {
      Serial.println("ERROR: Operation already in progress");
      return;
    }
  }

  char* strtokIndex = strtok(command, ",");

  if (strcmp(strtokIndex, "SCAN") == 0) {
    int duration = atoi(strtok(NULL, ","));
    int sparks = atoi(strtok(NULL, ","));
    int cycles = atoi(strtok(NULL, ","));
    if (sparks > 0 && cycles > 0) {
      samplingDuration = (unsigned long)duration * 1000UL;
      sparksPerCycle = sparks;
      totalCycles = cycles;
      cyclesRemaining = cycles;
      startScan();
    } else {
      Serial.println("ERROR: Invalid SCAN parameters");
    }
  }
  else if (strcmp(strtokIndex, "CLEAN") == 0) {
    int sparks = atoi(strtok(NULL, ","));
    if (sparks > 0) {
      cleanSparks = sparks;
      startClean();
    } else {
      Serial.println("ERROR: Invalid CLEAN parameters");
    }
  }
  else if (strcmp(strtokIndex, "PM") == 0) {
    int sparks = atoi(strtok(NULL, ","));
    int threshold = atoi(strtok(NULL, ","));
    float pmsize_f = atof(strtok(NULL, ","));
    if (sparks > 0 && threshold > 0) {
      totalSparks = sparks;
      pmThreshold = threshold;
      pmType = pmsize_f;
      startPMMonitoring();
    } else {
      Serial.println("ERROR: Invalid PM parameters");
    }
  }
  else if (strcmp(command, "CANCEL") == 0) {
    emergencyStop("User cancel");
  }
  else {
    Serial.print("ERROR: Unknown command '");
    Serial.print(command);
    Serial.println("'");
  }
}

// ----- SCAN MODE -----
void startScan() {
  safetyShutoff();
  scanning = true;
  lastStatusTime = millis();
  Serial.println("STARTING SCAN");
  Serial.print("CYCLE,"); Serial.println(1);
  startPump();
}

void handleScanning() {
  unsigned long now = millis();
  if (pumpActive && now - pumpStartTime >= samplingDuration) {
    stopPump();
    nextSparkTime = now + INTER_SPARK_DELAY;
    currentSpark = 0;
    sparkActive = false;
    return;
  }
  if (!pumpActive && !sparkActive && now >= nextSparkTime) {
    executeSpark();
  } else if (sparkActive) {
    updateSparkState();
  }
}

void executeSpark() {
  if (!sparkActive) {
    Serial.print("SPARK,"); Serial.println(currentSpark + 1);
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(BOOSTC_PIN, LOW);
    sparkStartTime = millis();
    sparkActive = true;
  }
}

void updateSparkState() {
  unsigned long now = millis();
  unsigned long elapsed = now - sparkStartTime;
  
  if (elapsed >= BOOST_DELAY && elapsed < SPARK_DURATION) {
    digitalWrite(BOOSTC_PIN, HIGH);
  } else if (elapsed >= SPARK_DURATION) {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(BOOSTC_PIN, LOW);
    sparkActive = false;
    currentSpark++;

    // Check which mode we are in to decide what to do next
    if (scanning) {
        if (currentSpark < sparksPerCycle) {
            nextSparkTime = now + INTER_SPARK_DELAY;
        } else {
            cyclesRemaining--;
            if (cyclesRemaining > 0) {
                Serial.print("CYCLE,"); Serial.println(totalCycles - cyclesRemaining + 1);
                startPump();
                currentSpark = 0;
            } else {
                scanning = false;
                Serial.println("SCAN COMPLETE");
                Serial.println("DONE");
            }
        }
    } else if (pmSparkPhase) { // Check for PM spark phase
        if (currentSpark < totalSparks) {
            nextSparkTime = now + INTER_SPARK_DELAY;
        } else {
            // All sparks for this cycle are complete. Return to monitoring.
            pmSparkPhase = false;
            startPump();
            pmMinuteStartTime = millis(); 
            lastPMAccumTime = millis();
            Serial.println("PM SPARKS COMPLETE");
        }
    }
  }
}

// ----- CLEAN MODE -----
void startClean() {
  safetyShutoff();
  cleaning      = true;
  currentSpark  = 0;
  lastStatusTime= millis();
  Serial.println("STARTING CLEAN");
  Serial.print("CYCLE,"); Serial.println(1);
  nextSparkTime = millis(); // fire immediately
}

void handleCleaning() {
  unsigned long now = millis();
  if (currentSpark < cleanSparks) {
    if (!sparkActive && now >= nextSparkTime) {
      Serial.print("SPARK,"); Serial.println(currentSpark + 1);
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(BOOSTC_PIN, LOW);
      sparkStartTime = now;
      sparkActive    = true;
    }
    if (sparkActive) {
      unsigned long elapsed = now - sparkStartTime;
      if (elapsed >= BOOST_DELAY && elapsed < SPARK_DURATION) {
        digitalWrite(BOOSTC_PIN, HIGH);
      }
      else if (elapsed >= SPARK_DURATION) {
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(BOOSTC_PIN, LOW);
        sparkActive = false;
        currentSpark++;
        nextSparkTime = now + INTER_SPARK_DELAY;
      }
    }
  }
  else if (cleaning) {
    cleaning = false;
    Serial.println("CLEAN COMPLETE");
    Serial.println("DONE");
  }
}

// ----- PM MONITORING MODE -----
void startPMMonitoring() {
  safetyShutoff();
  pmMonitoring      = true;
  pmSparkPhase      = false;
  pmMinuteSum       = 0;
  pmMinuteCount     = 0;
  pmCumulativeTotal = 0.0;  // Reset cumulative total when starting new PM monitoring
  lastPMAccumTime   = millis();
  lastStatusTime    = millis();
  pmMinuteStartTime = millis();
  Serial.println("STARTING PM MONITORING");
  Serial.print("CYCLE,"); Serial.println(1);
  startPump();
}

/**
 * @brief Returns the latest parsed PM value for the configured type.
 * Relies on processPMSensorData() being called continuously.
 */
int getLatestPMValue(float type) {
  if (type == 1.0) return pm1_0_standard;
  if (type == 2.5) return pm2_5_standard;
  if (type == 10.0) return pm10_standard;
  return 0; // Default
}

void handlePMSensor() {
  unsigned long now = millis();

  // --- Monitoring/Sampling Phase ---
  if (!pmSparkPhase) {
    // Accumulate a sample for the average every 5 seconds
    if (now - lastPMAccumTime >= PM_SAMPLE_ACCUM_INTERVAL) {
      int val = getLatestPMValue(pmType);
      pmMinuteSum  += val;
      pmMinuteCount++;
      lastPMAccumTime = now;
    }

    // After 1 minute, calculate the average and add it to cumulative total
    if (now - pmMinuteStartTime >= PM_MINUTE_INTERVAL) {
      float avg = 0;
      if (pmMinuteCount > 0) {
        avg = pmMinuteSum / (float)pmMinuteCount;
      }
      
      // Add this 1-minute average to the cumulative total
      pmCumulativeTotal += avg;
      
      // Send the cumulative total to Python instead of just the average
      Serial.print("PM_VALUE,"); 
      Serial.println(pmCumulativeTotal);

      // Reset for the next 1-minute window (but keep cumulative total)
      pmMinuteSum       = 0;
      pmMinuteCount     = 0;
      pmMinuteStartTime = now;

      // Check if the cumulative total exceeds the threshold to trigger a spark cycle
      if (pmCumulativeTotal >= pmThreshold) {
        stopPump();
        pmSparkPhase = true;
        currentSpark = 0;
        nextSparkTime = now + INTER_SPARK_DELAY;
        Serial.println("PM THRESHOLD REACHED");
        // Reset cumulative total after triggering sparks
        pmCumulativeTotal = 0.0;
      }
    }
  }
  // --- Sparking Phase ---
  else {
    // Only try to start a new spark if one isn't already active and it's time
    if (!sparkActive && now >= nextSparkTime) {
      executePMSpark(); // This will set sparkActive to true
    }
    // If a spark is active, let the update function handle it
    else if (sparkActive) {
      updateSparkState(); // Use the existing robust spark state handler
    }
  }
}

void executePMSpark() {
  if (!sparkActive) {
    Serial.print("SPARK,"); Serial.println(currentSpark + 1);
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(BOOSTC_PIN, LOW);
    sparkStartTime = millis();
    sparkActive = true;
  }
}

// ----- UTILITY FUNCTIONS -----
void startPump() {
  analogWrite(PUMP_PIN, 255);
  pumpActive = true;
  pumpStartTime = millis();
}

void stopPump() {
  analogWrite(PUMP_PIN, 0);
  pumpActive = false;
}

void sendStatusUpdate() {
  if (scanning && pumpActive) {
    unsigned long rem = samplingDuration - (millis() - pumpStartTime);
    Serial.print("TIME_LEFT,"); Serial.println(rem);
  }
}

void safetyShutoff() {
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BOOSTC_PIN, LOW);
  scanning = cleaning = pmMonitoring = pmSparkPhase = false;
  sparkActive = pumpActive = false;
}

void emergencyStop(const char* reason) {
  safetyShutoff();
  Serial.print("EMERGENCY STOP: "); Serial.println(reason);
  Serial.println("DONE"); // Signal GUI that operation is over
}