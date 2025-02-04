#include <Wire.h>
#include <inttypes.h>

uint8_t AHTX0_I2CADDR_DEFAULT = 0x38;  // Default I2C address
uint8_t AHT10_CMD_CALIBRATE[3] = {0xE1, 0x08, 0x00};  // Calibration command for AHT10 sensor
uint8_t AHTX0_CMD_TRIGGER[3] = {0xAC, 0x33, 0x00};  // Trigger reading command
uint8_t AHTX0_CMD_SOFTRESET = 0xBA;  // Soft reset command
uint8_t AHTX0_STATUS_BUSY = 0x80;  // Status bit for busy
uint8_t AHTX0_STATUS_CALIBRATED = 0x08;  // Status bit for calibrated


void transmit(uint8_t address, uint8_t buffer[], uint8_t buffer_size) {
  Wire.beginTransmission(address);
  uint8_t write_result = Wire.write(buffer, buffer_size);
  if (write_result != buffer_size) {
    Serial.write("Failed to write reset command\n");
  }
  uint8_t transmission_result = Wire.endTransmission(true);
  if (transmission_result != 0) {
    Serial.write("Wire transmission failed, error code:");
    Serial.print(transmission_result);
    Serial.write(".\n");
  }
}
uint8_t receive(uint8_t address, uint8_t out_buffer[], uint8_t buffer_size) {
  Wire.requestFrom(address, (uint8_t)buffer_size);
  for (int i = 0; i < buffer_size; ++i) {
    if (Wire.available()) {
      out_buffer[i] = Wire.read();
      /*
      Serial.write("Read ");
      Serial.print(out_buffer[i]);
      Serial.write("\n");
      // */
    } else {
      //Serial.write("Too few bytesp available. Read ");
      //Serial.print(i);
      //Serial.write("\n");
      return i;
    }
  }
  if (Wire.available()) {
    Serial.print("Discarding extra bytes: ");
    while (Wire.available()) {
      // Discard extra bytes
      Wire.read();
      Serial.print(".");
    }
    Serial.print("\n");
  }
  return buffer_size;
}

uint8_t get_status(uint8_t address, bool print) {
    uint8_t buffer[1];
    receive(address, buffer, 1);
    uint8_t status = buffer[0];
    if (print) {
      Serial.print("Status: 0x");
      Serial.print(status, 16);
      Serial.print(", Busy: ");
      Serial.print((uint8_t)((status & AHTX0_STATUS_BUSY) > 0));
      Serial.print(", Calibrated: ");
      Serial.print((uint8_t)((status & AHTX0_STATUS_CALIBRATED) > 0));
      Serial.print("\n");
    }
    return status;
}
void send_reset(uint8_t address) {
  transmit(address, &AHTX0_CMD_SOFTRESET, 1);
  delay(20);
}
void calibrate(uint8_t address) {
  transmit(address, AHT10_CMD_CALIBRATE, 3);
  Serial.print("Calibrating.\n");
  uint8_t status = get_status(address, true);

  int attempts = 0;
  while ((status & AHTX0_STATUS_BUSY) != 0 && attempts < 100) {
    delay(20);
    status = get_status(address, false);
  }
  if ((status & AHTX0_STATUS_BUSY) != 0) {
    Serial.print("Failed to calibrate.\n");
  } else {
    Serial.print("Calibrated\n");
  }
}
void read_data(uint8_t address, float &out_humidity, float &out_temperature) {
  transmit(address, AHTX0_CMD_TRIGGER, 3);
  
  int attempts = 0;
  uint8_t status = get_status(address, true);
  while ((status & AHTX0_STATUS_BUSY) != 0 && attempts < 100) {
    delay(20);
    status = get_status(address, false);
  }
  if ((status & AHTX0_STATUS_BUSY) != 0) {
    Serial.print("Failed to generate new data.\n");
    return;
  }
  const uint8_t buffer_length = 6;
  uint8_t buffer[buffer_length];
  uint8_t received_length = receive(address, buffer, buffer_length);
  if (received_length != buffer_length) {
    Serial.print("Failed to read all bytes when reading data. Read ");
    Serial.print(received_length);
    Serial.print("/");
    Serial.print(buffer_length);
    Serial.print(" bytes.\n");
  }
  Serial.print("Raw data: ");
  for (int i = 0; i < buffer_length; ++i) {
    Serial.print((uint8_t)buffer[i], 16);
  }
  Serial.print("\n");
  uint32_t raw_humidity = (((uint32_t)buffer[1]) << 12) |
    (((uint32_t)buffer[2]) << 4) |
    (((uint32_t)buffer[3]) >> 4);
  uint32_t raw_temperature = ((((uint32_t)buffer[3]) & 0xF) << 16) |
    (((uint32_t)buffer[4]) << 8) |
    ((uint32_t)buffer[5]);
  Serial.print("Raw humidity: ");
  Serial.print(raw_humidity, 16);
  Serial.print(", raw temperature: ");
  Serial.print(raw_temperature, 16);
  Serial.print("\n");

  out_humidity = (((float)raw_humidity) * 100) / 0x100000;
  out_temperature = ((((float)raw_temperature) * 200) / 0x100000) - 50;
}

void setup() {
  //digitalWrite(SCL, 1);
  delay(20);
  Serial.begin(9600);
  Serial.write("--- Restart ---\n");
  Wire.begin();
  uint8_t address = AHTX0_I2CADDR_DEFAULT;
  Wire.begin(address);
  delay(20);
  uint8_t status = get_status(address, true);
  while (status == 0xFF) {
    get_status(address, true);
    delay(2000);
  }
  calibrate(address);
  float humidity = 0.0f;
  float temperature = 0.0f;
  read_data(address, humidity, temperature);
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%, temperature: ");
  Serial.print(temperature);
  Serial.print(" degrees Celcius\n");


  //send_reset(address);
  /*
  uint8_t status = get_status(address);
  Serial.print("Status is: 0x");
  Serial.print(status, 16);
  Serial.println();*/
  /*
  if (status != 0xFF) {
    Serial.write("Waiting for temperature and humidity sensor (AHT10) to start\n");
    int attempts = 0;
    while (status != 0xFF && attempts < 1000) {
      attempts += 1;
      delay(20);
      status = get_status(address);
    }
    if (status != 0xFF) {
      Serial.write("AHT10 failed to start.\n");
    } else {
      Serial.write("AHT10 started!\n");
    }
  } 
  calibrate(address);
*/
}

void loop() {
  // put your main code here, to run repeatedly:

}
