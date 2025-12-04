#include <Wire.h>

// MPU6050 I2C
#define MPU_ADDR 0x68
#define MPU_REG_ACC_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT 0x6B

// HMC5883L
#define HMC_ADDR 0x1E

// Sensitivity scales
#define SCALE_ACC 16384.0
#define SCALE_MAG 0.92          //0.92 microTesla

// Calibration offset
double accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
double magMinX = 0, magMaxX = 0, magMinY = 0, magMaxY = 0, magMinZ = 0, magMaxZ = 0;

int graphReferenceZR = 0, graphReferenceFF = 45, graphReferenceNT = 90, graphReferenceOE = 180, graphReferenceNFF = -45, graphReferenceNNT = -90, graphReferenceNOE = -180;

const int MA_WINDOW = 10;   // Fenstergröße (10 = weiche Glättung)

float pitchBuf[MA_WINDOW], rollBuf[MA_WINDOW], yawBuf[MA_WINDOW];
int maIndex = 0;
bool maFilled = false;

float movingAverage(float buf[], float newVal, float threshold) {

  // 1) Letzten gültigen Messwert aus dem Buffer holen
  float lastVal;
  if (maFilled || maIndex > 0)
    lastVal = buf[(maIndex - 1 + MA_WINDOW) % MA_WINDOW];
  else
    lastVal = newVal;  // beim ersten Durchlauf kein Vergleich möglich

  // 2) Prüfen, ob der neue Wert unrealistisch stark abweicht
  bool outlier = abs(newVal - lastVal) > threshold;

  // 3) Falls Ausreißer → ersetzen durch aktuellen Durchschnitt
  if (outlier) {
    float sum = 0;
    int count = maFilled ? MA_WINDOW : maIndex;

    for (int i = 0; i < count; i++)
      sum += buf[i];

    if (count > 0)
      newVal = sum / count;
    else
      newVal = lastVal;  // ganz am Anfang
  }

  // 4) Neuen (ggf. korrigierten) Wert in den Buffer schreiben
  buf[maIndex] = newVal;

  // 5) Durchschnitt neu berechnen
  float sum = 0;
  int count = maFilled ? MA_WINDOW : maIndex + 1;

  for (int i = 0; i < count; i++)
    sum += buf[i];

  return sum / count;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  MPU_init();
  HMC_init();

  MPU_calibrate();
  HMC_calibrate();
}

void loop() {
  double ax, ay, az;
  double mx, my, mz;
  double pitch, roll, yaw;

  double MA_pitch, MA_roll, MA_yaw;

  // Read MPU6050 accelerometer data
  MPU_read(ax, ay, az);

  // Calculate pitch and roll
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  pitch = pitch * 2;

  roll = atan2(ay, az) * 180.0 / PI;
  roll = roll * 2;

  // Read HMC5883L magnetometer data
  HMC_read(mx, my, mz);

  // Calculate yaw from magnetometer data
  yaw = atan2(my, mx);

  // Adjust yaw using declination angle (replace with your location's value)
  float declinationAngle = 0.06925;  // Declination in radians for -3° 58' 
  yaw += declinationAngle;

  // Normalize yaw to 0-360 degrees
  if (yaw < 0) yaw += 2 * PI;
  if (yaw > 2 * PI) yaw -= 2 * PI;

  yaw = yaw * 180.0 / PI; // Convert to degrees

  MA_pitch = movingAverage(pitchBuf, pitch, 50.0);
  MA_roll  = movingAverage(rollBuf, roll, 50.0);
  MA_yaw   = movingAverage(yawBuf, yaw, 50.0);

  // advance ring index
  maIndex++;
  if (maIndex >= MA_WINDOW) {
    maIndex = 0;
    maFilled = true;
  }

  // Print results
  Serial.print("Pitch:"); 
  Serial.print(pitch); 
  Serial.print(",");
  Serial.print("Roll:"); 
  Serial.print(roll); 
  Serial.print(",");
  Serial.print("Yaw:");
  Serial.print(yaw); 
  Serial.print(",");

  Serial.print("MA.Pitch:"); 
  Serial.print(MA_pitch); 
  Serial.print(",");
  Serial.print("MA.Roll:"); 
  Serial.print(MA_roll); 
  Serial.print(",");
  Serial.print("MA.Yaw:");
  Serial.print(MA_yaw); 
  Serial.print(",");

  Serial.print("0°:");
  Serial.print(graphReferenceZR); 
  Serial.print(",");
  Serial.print("45°:");
  Serial.print(graphReferenceFF); 
  Serial.print(",");
  Serial.print("90°:");
  Serial.print(graphReferenceNT); 
  Serial.print(",");
  Serial.print("180°:");
  Serial.print(graphReferenceOE); 
  Serial.print(",");
  Serial.print("-45°:");
  Serial.print(graphReferenceNFF); 
  Serial.print(",");
  Serial.print("-90°:");
  Serial.print(graphReferenceNNT); 
  Serial.print(",");
  Serial.print("-180°:");
  Serial.println(graphReferenceNOE); 

  delay(10);
}

void MPU_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_PWR_MGMT);
  Wire.write(0);
  Wire.endTransmission(true);
}

void MPU_calibrate() {
  Serial.println("Calibrating MPU6050...");
  double ax, ay, az;
  int numSamples = 100;

  for(int i = 0; i < numSamples; i++) {
    MPU_read(ax, ay, az);
    accOffsetX += ax;
    accOffsetY += ay;
    accOffsetZ += az;
    delay(10);
  }

  accOffsetX /= numSamples;
  accOffsetY /= numSamples;
  accOffsetZ /= numSamples;

  accOffsetZ -= 1.0;
}

void MPU_read(double &ax, double &ay, double &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACC_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  ax = (Wire.read() << 8 | Wire.read()) / SCALE_ACC - accOffsetX;
  ay = (Wire.read() << 8 | Wire.read()) / SCALE_ACC - accOffsetY;
  az = (Wire.read() << 8 | Wire.read()) / SCALE_ACC - accOffsetZ;
}

void HMC_init() {
  //Configuration register A
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();

  //Configuration Register B
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x01);
  Wire.write(0x20);
  Wire.endTransmission();

  //Continuous measurement mode
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

void HMC_calibrate() {
  Serial.println("Calibrating HMC5883L...");
  double mx, my, mz;

  magMinX = magMinY = magMinZ = 1e6;
  magMaxX = magMaxY = magMaxZ = -1e6;

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) { // 10 seconds for calibration
    HMC_read(mx, my, mz);

    if (mx < magMinX) magMinX = mx;
    if (mx > magMaxX) magMaxX = mx;
    if (my < magMinY) magMinY = my;
    if (my > magMaxY) magMaxY = my;
    if (mz < magMinZ) magMinZ = mz;
    if (mz > magMaxZ) magMaxZ = mz;

    delay(100);
  }

  Serial.println("HMC5883L Calibration Complete.");
}

void HMC_read(double &mx, double &my, double &mz) {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x03); // Starting register for magnetometer readings
  Wire.endTransmission(false);
  Wire.requestFrom(HMC_ADDR, 6, true);

  int16_t x = Wire.read() << 8 | Wire.read();
  int16_t z = Wire.read() << 8 | Wire.read();
  int16_t y = Wire.read() << 8 | Wire.read();

  // Apply calibration offsets and scaling
  mx = (x - (magMinX + magMaxX) / 2) * SCALE_MAG;
  my = (y - (magMinY + magMaxY) / 2) * SCALE_MAG;
  mz = (z - (magMinZ + magMaxZ) / 2) * SCALE_MAG;
}
