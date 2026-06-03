#include <Arduino.h>

/*
 * Sensor Glove - ESP32 + CD74HC4067 Multiplexer
 *
 * GUI command protocol, 115200 baud:
 *   CAL_FLEX <sensor_id> <angle>
 *                     Captures flex ADC for angle 0, 45, or 90
 *   SET_FLEX <sensor_id> <angle> <adc>
 *                     Loads a saved flex calibration point from the GUI
 *   CLEAR_FLEX        Clears flex calibration points
 *   STREAM_RAW        Streams MP raw, PIP raw, FSR raw
 *   STREAM_MIXED      Streams MP raw, flex angle if calibrated, FSR raw
 *   STREAM_FLEX       Streams flex angle if calibrated
 *   STREAM_FSR        Streams FSR raw ADC
 *   STOP              Stops streaming
 */

// === Mux select pins ===
const int S0  = 26;
const int S1  = 25;
const int S2  = 33;
const int S3  = 32;
const int SIG = 34;

// === Mux channel assignments ===
const int CH_PRESSURE = 8;
const int CH_FLEX_DIP = 7;
const int CH_FLEX_MP  = 6;

// === Settings ===
const int OVERSAMPLE    = 10;
const int FILTER_SIZE   = 5;
const int LOOP_DELAY_MS = 50;
const int LED_PIN       = 2;

// === Flex angle calibration ===
const int FLEX_CALIB_POINTS = 3;
const int FLEX_SENSOR_COUNT = 6;
const int flexAngles[FLEX_CALIB_POINTS] = {0, 45, 90};
const char* flexSensorIds[FLEX_SENSOR_COUNT] = {
  "MIDDLE_DIP",
  "MIDDLE_MP",
  "INDEX_DIP",
  "INDEX_MP",
  "RING_DIP",
  "RING_MP"
};
const int flexSensorChannels[FLEX_SENSOR_COUNT] = {
  CH_FLEX_DIP,
  CH_FLEX_MP,
  -1,
  -1,
  -1,
  -1
};
int flexCalibADC[FLEX_SENSOR_COUNT][FLEX_CALIB_POINTS] = {0};
bool flexCalibSet[FLEX_SENSOR_COUNT][FLEX_CALIB_POINTS] = {false};
bool flexCalibrated[FLEX_SENSOR_COUNT] = {false};

// === Moving average buffers ===
int flexBuffer[FLEX_SENSOR_COUNT][FILTER_SIZE] = {0};
int flexBufIdx[FLEX_SENSOR_COUNT] = {0};

enum StreamMode {
  STREAM_NONE,
  STREAM_RAW,
  STREAM_MIXED,
  STREAM_FLEX,
  STREAM_FSR,
  STREAM_MULTI_RAW
};

StreamMode streamMode = STREAM_NONE;
String commandBuffer;
unsigned long lastStreamMs = 0;

void handleCommand(String command);
void captureFlexCalibration(int sensorIndex, int angle);
void loadFlexCalibration(int sensorIndex, int angle, int adc);
void clearFlexCalibration();
void updateFlexCalibrationState(int sensorIndex);
int findFlexSensorIndex(String sensorId);
int findFlexAngleIndex(int angle);
int defaultFlexSensorIndex();
void streamCurrentData();
void streamRaw();
void streamMixed();
void streamFlex();
void streamFsr();
int readMux(uint8_t channel);
int readMuxAveraged(int channel);
int bufferAverage(int buf[]);
float interpolateFlexAngle(int sensorIndex, int adc);

void setup() {
  Serial.begin(115200);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(300);
  Serial.println(F("READY"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      commandBuffer.trim();
      if (commandBuffer.length() > 0) {
        handleCommand(commandBuffer);
      }
      commandBuffer = "";
    } else {
      commandBuffer += c;
    }
  }

  if (streamMode != STREAM_NONE && millis() - lastStreamMs >= LOOP_DELAY_MS) {
    lastStreamMs = millis();
    streamCurrentData();
  }
}

void handleCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command.startsWith("CAL_FLEX ")) {
    String payload = command.substring(9);
    int splitAt = payload.indexOf(' ');
    if (splitAt < 0) {
      int angle = payload.toInt();
      captureFlexCalibration(defaultFlexSensorIndex(), angle);
      return;
    }
    int sensorIndex = findFlexSensorIndex(payload.substring(0, splitAt));
    int angle = payload.substring(splitAt + 1).toInt();
    captureFlexCalibration(sensorIndex, angle);
  } else if (command.startsWith("SET_FLEX ")) {
    String payload = command.substring(9);
    int firstSplit = payload.indexOf(' ');
    int secondSplit = payload.indexOf(' ', firstSplit + 1);
    if (firstSplit < 0 || secondSplit < 0) {
      Serial.println(F("ERR,SET_FLEX_FORMAT"));
      return;
    }
    int sensorIndex = findFlexSensorIndex(payload.substring(0, firstSplit));
    int angle = payload.substring(firstSplit + 1, secondSplit).toInt();
    int adc = payload.substring(secondSplit + 1).toInt();
    loadFlexCalibration(sensorIndex, angle, adc);
  } else if (command == "CLEAR_FLEX") {
    clearFlexCalibration();
  } else if (command == "STREAM_RAW") {
    streamMode = STREAM_RAW;
    Serial.println(F("ACK,STREAM_RAW"));
  } else if (command == "STREAM_MIXED") {
    streamMode = STREAM_MIXED;
    Serial.println(F("ACK,STREAM_MIXED"));
  } else if (command == "STREAM_FLEX") {
    streamMode = STREAM_FLEX;
    Serial.println(F("ACK,STREAM_FLEX"));
  } else if (command == "STREAM_FSR") {
    streamMode = STREAM_FSR;
    Serial.println(F("ACK,STREAM_FSR"));
  } else if (command == "STREAM_MULTI_RAW") {
    streamMode = STREAM_MULTI_RAW;
    Serial.println(F("ACK,STREAM_MULTI_RAW"));
  } else if (command == "STOP") {
    streamMode = STREAM_NONE;
    Serial.println(F("ACK,STOP"));
  } else {
    Serial.print(F("ERR,UNKNOWN_COMMAND,"));
    Serial.println(command);
  }
}

void captureFlexCalibration(int sensorIndex, int angle) {
  if (sensorIndex < 0) {
    Serial.println(F("ERR,FLEX_SENSOR"));
    return;
  }
  int channel = flexSensorChannels[sensorIndex];
  if (channel < 0) {
    Serial.print(F("ERR,FLEX_SENSOR_NOT_CONNECTED,"));
    Serial.println(flexSensorIds[sensorIndex]);
    return;
  }

  int index = findFlexAngleIndex(angle);
  if (index < 0) {
    Serial.print(F("ERR,FLEX_ANGLE,"));
    Serial.println(angle);
    return;
  }

  digitalWrite(LED_PIN, HIGH);
  int adc = readMuxAveraged(channel);
  delay(120);
  digitalWrite(LED_PIN, LOW);

  flexCalibADC[sensorIndex][index] = adc;
  flexCalibSet[sensorIndex][index] = true;
  updateFlexCalibrationState(sensorIndex);

  Serial.print(F("FLEX_CAL,"));
  Serial.print(flexSensorIds[sensorIndex]);
  Serial.print(F(","));
  Serial.print(angle);
  Serial.print(F(","));
  Serial.print(adc);
  Serial.print(F(","));
  Serial.println(flexCalibrated[sensorIndex] ? F("COMPLETE") : F("INCOMPLETE"));
}

void loadFlexCalibration(int sensorIndex, int angle, int adc) {
  if (sensorIndex < 0) {
    Serial.println(F("ERR,FLEX_SENSOR"));
    return;
  }
  int index = findFlexAngleIndex(angle);
  if (index < 0) {
    Serial.print(F("ERR,FLEX_ANGLE,"));
    Serial.println(angle);
    return;
  }
  if (adc < 0 || adc > 4095) {
    Serial.print(F("ERR,FLEX_ADC,"));
    Serial.println(adc);
    return;
  }

  flexCalibADC[sensorIndex][index] = adc;
  flexCalibSet[sensorIndex][index] = true;
  updateFlexCalibrationState(sensorIndex);

  Serial.print(F("ACK,SET_FLEX,"));
  Serial.print(flexSensorIds[sensorIndex]);
  Serial.print(F(","));
  Serial.print(angle);
  Serial.print(F(","));
  Serial.print(adc);
  Serial.print(F(","));
  Serial.println(flexCalibrated[sensorIndex] ? F("COMPLETE") : F("INCOMPLETE"));
}

void clearFlexCalibration() {
  for (int sensor = 0; sensor < FLEX_SENSOR_COUNT; sensor++) {
    for (int i = 0; i < FLEX_CALIB_POINTS; i++) {
      flexCalibADC[sensor][i] = 0;
      flexCalibSet[sensor][i] = false;
    }
    flexCalibrated[sensor] = false;
  }
  Serial.println(F("ACK,CLEAR_FLEX"));
}

void updateFlexCalibrationState(int sensorIndex) {
  flexCalibrated[sensorIndex] = true;
  for (int i = 0; i < FLEX_CALIB_POINTS; i++) {
    if (!flexCalibSet[sensorIndex][i]) {
      flexCalibrated[sensorIndex] = false;
      break;
    }
  }
}

int findFlexSensorIndex(String sensorId) {
  sensorId.trim();
  sensorId.toUpperCase();
  for (int i = 0; i < FLEX_SENSOR_COUNT; i++) {
    if (sensorId == flexSensorIds[i]) {
      return i;
    }
  }
  return -1;
}

int findFlexAngleIndex(int angle) {
  for (int i = 0; i < FLEX_CALIB_POINTS; i++) {
    if (flexAngles[i] == angle) {
      return i;
    }
  }
  return -1;
}

int defaultFlexSensorIndex() {
  return findFlexSensorIndex("MIDDLE_DIP");
}

void streamCurrentData() {
  switch (streamMode) {
    case STREAM_RAW:   streamRaw();   break;
    case STREAM_MIXED: streamMixed(); break;
    case STREAM_FLEX:  streamFlex();  break;
    case STREAM_FSR:   streamFsr();   break;
    case STREAM_MULTI_RAW: streamMultiRaw(); break;
    case STREAM_NONE:  break;
  }
}

void streamRaw() {
  Serial.print(readMuxAveraged(CH_FLEX_MP));
  Serial.print(F(","));
  Serial.print(readMuxAveraged(CH_FLEX_DIP));
  Serial.print(F(","));
  Serial.println(readMuxAveraged(CH_PRESSURE));
}

void streamMixed() {
  int mp = readMuxAveraged(CH_FLEX_MP);
  int sensorIndex = defaultFlexSensorIndex();
  int dip = readMuxAveraged(CH_FLEX_DIP);
  int fsr = readMuxAveraged(CH_PRESSURE);

  flexBuffer[sensorIndex][flexBufIdx[sensorIndex]] = dip;
  flexBufIdx[sensorIndex] = (flexBufIdx[sensorIndex] + 1) % FILTER_SIZE;
  int filtered = bufferAverage(flexBuffer[sensorIndex]);

  Serial.print(mp);
  Serial.print(F(","));
  Serial.print(flexCalibrated[sensorIndex] ? interpolateFlexAngle(sensorIndex, filtered) : (float)filtered, 1);
  Serial.print(F(","));
  Serial.print(fsr);
  Serial.println(F(",0,4095"));
}

void streamFlex() {
  int sensorIndex = defaultFlexSensorIndex();
  if (!flexCalibrated[sensorIndex]) {
    Serial.println(F("ERR,FLEX_NOT_CALIBRATED"));
    return;
  }

  int raw = readMuxAveraged(CH_FLEX_DIP);
  flexBuffer[sensorIndex][flexBufIdx[sensorIndex]] = raw;
  flexBufIdx[sensorIndex] = (flexBufIdx[sensorIndex] + 1) % FILTER_SIZE;
  int filtered = bufferAverage(flexBuffer[sensorIndex]);

  Serial.print(interpolateFlexAngle(sensorIndex, filtered), 1);
  Serial.println(F(",0,90"));
}

void streamFsr() {
  Serial.print(readMuxAveraged(CH_PRESSURE));
  Serial.println(F(",0,4095"));
}

void streamMultiRaw() {
  // Order must match what the GUI expects.
  // For each finger, DIP then MP.
  // Example: MIDDLE_DIP, MIDDLE_MP, INDEX_DIP, INDEX_MP, RING_DIP, RING_MP
  for (int i = 0; i < FLEX_SENSOR_COUNT; i++) {
    if (flexSensorChannels[i] >= 0) {
      Serial.print(readMuxAveraged(flexSensorChannels[i]));
    } else {
      Serial.print(0);  // placeholder for missing sensor
    }
    if (i < FLEX_SENSOR_COUNT - 1) Serial.print(',');
  }
  Serial.println();
}

int readMux(uint8_t channel) {
  digitalWrite(S0,  channel & 0x01);
  digitalWrite(S1, (channel >> 1) & 0x01);
  digitalWrite(S2, (channel >> 2) & 0x01);
  digitalWrite(S3, (channel >> 3) & 0x01);
  delayMicroseconds(5);
  return analogRead(SIG);
}

int readMuxAveraged(int channel) {
  long sum = 0;
  for (int i = 0; i < OVERSAMPLE; i++) {
    sum += readMux(channel);
    delayMicroseconds(100);
  }
  return (int)(sum / OVERSAMPLE);
}

int bufferAverage(int buf[]) {
  long sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += buf[i];
  }
  return (int)(sum / FILTER_SIZE);
}

float interpolateFlexAngle(int sensorIndex, int adc) {
  bool increasing = (
    flexCalibADC[sensorIndex][FLEX_CALIB_POINTS - 1] > flexCalibADC[sensorIndex][0]
  );

  if (increasing) {
    if (adc <= flexCalibADC[sensorIndex][0]) return (float)flexAngles[0];
    if (adc >= flexCalibADC[sensorIndex][FLEX_CALIB_POINTS - 1]) return (float)flexAngles[FLEX_CALIB_POINTS - 1];
    for (int i = 0; i < FLEX_CALIB_POINTS - 1; i++) {
      if (adc >= flexCalibADC[sensorIndex][i] && adc <= flexCalibADC[sensorIndex][i + 1]) {
        float t = (float)(adc - flexCalibADC[sensorIndex][i]) / (float)(flexCalibADC[sensorIndex][i + 1] - flexCalibADC[sensorIndex][i]);
        return flexAngles[i] + t * (flexAngles[i + 1] - flexAngles[i]);
      }
    }
  } else {
    if (adc >= flexCalibADC[sensorIndex][0]) return (float)flexAngles[0];
    if (adc <= flexCalibADC[sensorIndex][FLEX_CALIB_POINTS - 1]) return (float)flexAngles[FLEX_CALIB_POINTS - 1];
    for (int i = 0; i < FLEX_CALIB_POINTS - 1; i++) {
      if (adc <= flexCalibADC[sensorIndex][i] && adc >= flexCalibADC[sensorIndex][i + 1]) {
        float t = (float)(flexCalibADC[sensorIndex][i] - adc) / (float)(flexCalibADC[sensorIndex][i] - flexCalibADC[sensorIndex][i + 1]);
        return flexAngles[i] + t * (flexAngles[i + 1] - flexAngles[i]);
      }
    }
  }

  return 0.0;
}
