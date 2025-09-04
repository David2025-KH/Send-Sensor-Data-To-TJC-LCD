#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_DPS310.h>

// --- LCD serial port ---
#define LCDSerial Serial2

// ========== Sensors ==========
Adafruit_AHTX0 aht20;
Adafruit_DPS310 dps;

// --- LCD helpers with Serial mirror ---
void sendValue(uint8_t id, float val, uint8_t decimals = 0) {
  char buffer[16];
  if (decimals > 0) snprintf(buffer, sizeof(buffer), "%.*f", decimals, val);
  else snprintf(buffer, sizeof(buffer), "%d", (int)val);

  String cmd = "t" + String(id) + ".txt=\"" + String(buffer) + "\"";

  // Send to LCD
  LCDSerial.print(cmd);
  LCDSerial.write(0xFF); LCDSerial.write(0xFF); LCDSerial.write(0xFF);

  // Mirror to Serial Monitor
  Serial.println("[LCD] " + cmd);
}

void sendText(uint8_t id, const char* status) {
  String cmd = "t" + String(id) + ".txt=\"" + String(status) + "\"";

  // Send to LCD
  LCDSerial.print(cmd);
  LCDSerial.write(0xFF); LCDSerial.write(0xFF); LCDSerial.write(0xFF);

  // Mirror to Serial Monitor
  Serial.println("[LCD] " + cmd);
}

// --- Comment functions ---
const char* getTempComment(float t) {
  if (t < 18) return "Cold";
  else if (t < 27) return "Comfort";
  else return "Hot";
}

const char* getHumComment(float h) {
  if (h < 30) return "Dry";
  else if (h < 60) return "Normal";
  else return "Humid";
}

const char* getPressureComment(float p) {
  if (p < 100000) return "Low";
  else if (p < 102000) return "Normal";
  else return "High";
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  LCDSerial.begin(9600, SERIAL_8N1, 16, 17); // TX=17, RX=16
  delay(200);

  Serial.println("Starting AHT20 + DPS310 + LCD test...");

  // --- AHT20 ---
  if (!aht20.begin()) Serial.println("AHT20 not found!");
  else Serial.println("AHT20 OK!");

  // --- DPS310 ---
  if (!dps.begin_I2C()) Serial.println("DPS310 not found!");
  else {
    Serial.println("DPS310 OK!");
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  }
}

// ========== Loop ==========
void loop() {
  // --- AHT20 ---
  sensors_event_t aht_temp, aht_humidity;
  aht20.getEvent(&aht_humidity, &aht_temp);

  // --- DPS310 ---
  sensors_event_t dps_temp_event, dps_pressure_event;
  float pressurePa = 0;
  float dpsTemp = 0;

  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    dps.getEvents(&dps_temp_event, &dps_pressure_event);
    dpsTemp = dps_temp_event.temperature;
    pressurePa = dps_pressure_event.pressure * 100.0; // convert hPa → Pa
  }

  // --- Update LCD ---
  // Page 0: Temperature
  sendValue(0, aht_temp.temperature, 2);
  sendText(1, getTempComment(aht_temp.temperature));

  // Page 1: Humidity
  sendValue(2, aht_humidity.relative_humidity, 2);
  sendText(3, getHumComment(aht_humidity.relative_humidity));

  // Page 2: Pressure
  sendValue(4, pressurePa, 0);
  sendText(5, getPressureComment(pressurePa));

  // --- Serial debug ---
  Serial.print("[AHT20] Temp: "); Serial.print(aht_temp.temperature); 
  Serial.print(" °C, Hum: "); Serial.println(aht_humidity.relative_humidity);

  Serial.print("[DPS310] Temp: "); Serial.print(dpsTemp); 
  Serial.print(" °C, Pressure: "); Serial.println(pressurePa);

  Serial.println("---------------------------");

  delay(1000);
}
