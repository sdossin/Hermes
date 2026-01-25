#include <Arduino.h>
#include <Wire.h>

static const uint8_t AS5600_ADDR = 0x36;
static const uint8_t ANGLE_REG_MSB = 0x0E; // RAW ANGLE (MSB), then 0x0F

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Ping device
  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("AS5600 not found at 0x36");
    while (1) delay(1000);
  }
  Serial.println("AS5600 OK");
  int start_up_delay = 3;
  for(int i = 0; i < start_up_delay; i++) {
    Serial.print("Loop starting in ");
    Serial.println(start_up_delay - i);
    delay(1000);
  }
}

bool readAS5600Raw(uint16_t &raw12) {
  // Set register pointer to 0x0E
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(ANGLE_REG_MSB);
  if (Wire.endTransmission(false) != 0) return false; // repeated start

  // Read 2 bytes
  uint8_t n = Wire.requestFrom((int)AS5600_ADDR, 2);
  if (n != 2) return false;

  int hi = Wire.read();
  int lo = Wire.read();
  if (hi < 0 || lo < 0) return false;

  uint16_t v = ((uint16_t)hi << 8) | (uint16_t)lo;
  raw12 = v & 0x0FFF; // AS5600 raw angle is 12-bit
  return true;
}

void loop() {
  uint16_t raw;
  if (readAS5600Raw(raw)) {
    float deg = raw * (360.0f / 4096.0f);
    Serial.println(deg, 2);
  } else {
    Serial.println("read failed");
  }
  delay(20);
}
