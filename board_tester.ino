#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <stdlib.h>

#include <Bounce2.h>

#include <Wire.h>
#include <inttypes.h>
#include "FastIMU.h"

enum class Status {
    Unknown,
    NotResponding,
    StoppedResponding,
    Ok,
};

#define IMU_ADDRESS 0x69  //Change to the address of the IMU
BMI160 IMU;               //Change to the name of any supported IMU!
calData calib = {0};    //Calibration data
AccelData accelData;      //Sensor data
#define CIRCLE_RADIUS 3
Status BMI160_status = Status::Unknown;

uint8_t AHTX0_I2CADDR_DEFAULT = 0x38;                   // Default I2C address
uint8_t AHT10_CMD_CALIBRATE[3] = {0xE1, 0x08, 0x00};  // Calibration command for AHT10 sensor
uint8_t AHTX0_CMD_TRIGGER[3] = {0xAC, 0x33, 0x00};    // Trigger reading command
uint8_t AHTX0_CMD_SOFTRESET = 0xBA;                     // Soft reset command
uint8_t AHTX0_STATUS_BUSY = 0x80;                       // Status bit for busy
uint8_t AHTX0_STATUS_CALIBRATED = 0x08;                 // Status bit for calibrated
Status AHT10_status = Status::Unknown;


#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16
Status SSD1306_status = Status::Unknown;

Bounce button1 = Bounce();
Bounce button2 = Bounce();

bool transmit(uint8_t address, uint8_t buffer[], uint8_t buffer_size) {
    delay(10);  // This device needs some space after other I2C devices are done talking
    Wire.beginTransmission(address);
    uint8_t write_result = Wire.write(buffer, buffer_size);
    if (write_result != buffer_size) {
        Serial.write("Failed to write I2C message\n");
        return false;
    }
    uint8_t transmission_result = Wire.endTransmission(true);
    if (transmission_result != 0) {
        Serial.write("Wire transmission failed, error code:");
        Serial.print(transmission_result);
        Serial.write(".\n");
        return false;
    }
    return true;
}
uint8_t receive(uint8_t address, uint8_t out_buffer[], uint8_t buffer_size) {
    for (int i = 0; i < buffer_size; ++i) {
        out_buffer[i] = 0xFF;
    }
    Wire.requestFrom(address, (uint8_t) buffer_size);
    for (int i = 0; i < buffer_size; ++i) {
        if (Wire.available()) {
            out_buffer[i] = Wire.read();
        } else {
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
    uint8_t received_length = receive(address, buffer, 1);
    uint8_t status = buffer[0];
    if (received_length != 1) {
        status = 0xFF;
    }
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
bool send_reset(uint8_t address) {
    if (!transmit(address, &AHTX0_CMD_SOFTRESET, 1)) {
        return false;
    }
    delay(20);
    return true;
}
Status calibrate(uint8_t address) {
    if (!transmit(address, AHT10_CMD_CALIBRATE, 3)) {
        return Status::StoppedResponding;
    }
    Serial.print("Calibrating.\n");
    uint8_t status = get_status(address, true);

    int attempts = 0;
    while ((status & AHTX0_STATUS_BUSY) != 0 && attempts < 100) {
        delay(20);
        status = get_status(address, false);
    }
    if ((status & AHTX0_STATUS_BUSY) != 0) {
        Serial.print("Failed to calibrate.\n");
        return Status::StoppedResponding;
    } else {
        Serial.print("Calibrated\n");
        return Status::Ok;
    }
}

Status initialize() {
    uint8_t address = AHTX0_I2CADDR_DEFAULT;
    Wire.begin(address);
    delay(20);
    if (!send_reset(address)) {
        // Let reset fail, it's optional, and instead have the repeating status be
        // the more reliable sign that the circuit is broken
        //return Status::NotResponding;
    }
    delay(20);
    uint8_t status = get_status(address, true);
    int attempts = 0;
    while (status == 0xFF && attempts < 100) {
        get_status(address, false);
        attempts += 1;
        delay(20);
    }
    if (status == 0xFF) {
        Serial.print("AHT10 didn't respond.");
        return Status::NotResponding;
    }
    return calibrate(address);
}

// Returns success
bool read_data(uint8_t address, float& out_humidity, float& out_temperature, bool print) {
    bool transmitted = transmit(address, AHTX0_CMD_TRIGGER, 3);
    if (!transmitted) {
        return false;
    }

    int attempts = 0;
    uint8_t status = get_status(address, false);
    while ((status & AHTX0_STATUS_BUSY) != 0 && attempts < 100) {
        delay(2000);
        status = get_status(address, print);
    }
    if ((status & AHTX0_STATUS_BUSY) != 0) {
        if (print) {
            Serial.print("Failed to generate new data.\n");
        }
        return false;
    }
    const uint8_t buffer_size = 6;
    uint8_t buffer[buffer_size];
    uint8_t received_length = receive(address, buffer, buffer_size);
    if (received_length != buffer_size) {
        Serial.print("Failed to read all bytes when reading data. Read ");
        Serial.print(received_length);
        Serial.print("/");
        Serial.print(buffer_size);
        Serial.print(" bytes.\n");
        return false;
    }
    if (print) {
        Serial.print("Raw data: ");
        for (int i = 0; i < buffer_size; ++i) {
            Serial.print((uint8_t) buffer[i], 16);
        }
        Serial.print(", ");
    }
    uint32_t raw_humidity = (((uint32_t) buffer[1]) << 12) | (((uint32_t) buffer[2]) << 4) | (((uint32_t) buffer[3]) >> 4);
    uint32_t raw_temperature = ((((uint32_t) buffer[3]) & 0xF) << 16) | (((uint32_t) buffer[4]) << 8) | ((uint32_t) buffer[5]);
    if (print) {
        Serial.print("raw humidity: ");
        Serial.print(raw_humidity, 16);
        Serial.print(", raw temperature: ");
        Serial.print(raw_temperature, 16);
        Serial.print("\n");
    }

    out_humidity = (((float) raw_humidity) * 100) / 0x100000;
    out_temperature = ((((float) raw_temperature) * 200) / 0x100000) - 50;
    return true;
}

void writeString(char* text) {
    char* c = text;
    while (*c) {
        display.write(*c);
        c++;
    }
}

void writeFloatString(float n) {
    char buffer[32];
#if BOARD_VENDORID == 0x2e8a && BOARD_PRODUCTID == 0x00c0
    sprintf(buffer, "%.1f", n);
#else
    dtostrf(n, 4, 1, buffer);
#endif
    writeString(buffer);
}
void writeIntString(uint32_t n) {
    char buffer[32];
#if BOARD_VENDORID == 0x2e8a && BOARD_PRODUCTID == 0x00c0
    sprintf(buffer, "%d", n);
#else
    itoa(n, buffer, 10);
#endif
    writeString(buffer);
}


void setup() {
    Serial.begin(9600);
    delay(20);
    Serial.write("--- Restart ---\n");
    Serial.write("Wire begin\n");

    Wire.begin();
    Serial.write("IMU init\n");
    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) {
        Serial.print("Error initializing IMU: ");
        Serial.println(err);
        BMI160_status = Status::NotResponding;
    } else {
        BMI160_status = Status::Ok;
    }
    Serial.write("AHT10 init\n");
    AHT10_status = initialize();

    Serial.write("Display init\n");
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
    }

#if BOARD_VENDORID == 0x2e8a && BOARD_PRODUCTID == 0x00c0
    // If we're on raspberry pi pico
    unsigned button1_pin = 7;
    unsigned button2_pin = 8;
#else
    unsigned button1_pin = 5;
    unsigned button2_pin = 4;
#endif

    pinMode(button1_pin,INPUT_PULLUP);
    button1.attach(button1_pin);
    button1.interval(10);// interval in ms
    pinMode(button2_pin,INPUT_PULLUP);
    button2.attach(button2_pin);
    button2.interval(10); // interval in ms

    display.setTextSize(1);       // Normal 1:1 pixel scale
    display.setTextColor(WHITE);  // Draw white text
    display.cp437(true);          // Use full 256 char 'Code Page 437' font
    display.clearDisplay();
    display.display();
}

void loop() {
    Serial.println("Loop begin");

    display.clearDisplay();
    display.setCursor(0, 0);      // Start at top-left corner

    if (AHT10_status == Status::Unknown) {
        writeString("AHT10: Unknown\n");
    } else if (AHT10_status == Status::NotResponding) {
        writeString("AHT10: No response\n");
    } else if (AHT10_status == Status::StoppedResponding) {
        writeString("AHT10: Stopped responding\n");
    } else if (AHT10_status == Status::Ok) {
        float humidity = 0.0f;
        float temperature = 0.0f;
        bool success = read_data(AHTX0_I2CADDR_DEFAULT, humidity, temperature, false);
        //writeString("AHT10: H22 T26\n");
        writeString("AHT10: H");
        writeIntString((uint32_t) humidity);
        writeString("% T");
        //writeIntString((uint32_t) temperature);
        writeFloatString(temperature);
        writeString("C\n");
    }
    if (BMI160_status == Status::Unknown) {
        writeString("BMI160: Unknown\n");
    } else if (BMI160_status == Status::NotResponding) {
        writeString("BMI160: No response\n");
    } else if (BMI160_status == Status::StoppedResponding) {
        writeString("BMI160: Stopped responding\n");
    } else if (BMI160_status == Status::Ok) {
        IMU.update();
        IMU.getAccel(&accelData);
        float temperature = IMU.getTemp();
        float accelY = accelData.accelY;
        float accelX = accelData.accelX;
        float factor = 2.5;
        int16_t x = CIRCLE_RADIUS + ((accelY * factor + 1) / 2) * (SCREEN_WIDTH - CIRCLE_RADIUS);
        int16_t y = CIRCLE_RADIUS + ((accelX * factor + 1) / 2) * (SCREEN_HEIGHT - CIRCLE_RADIUS);

        //writeString("BMI160: X0Y0 T26\n");
        writeString("BMI160: X");
        writeIntString((uint32_t) x);
        writeString("Y");
        writeIntString((uint32_t) y);
        writeString(" T");
        //writeIntString((uint32_t) temperature);
        writeFloatString(temperature);
        writeString("C\n");
//        writeString("BMI160: X0Y0 T26\n");
    }

    //writeString("B1: UP  , B2: DOWN");
    writeString("B1: ");
    button1.update();
    if (button1.read() == LOW) {
        writeString("DOWN");
    } else {
        writeString("UP  ");
    }
    writeString(", B2: ");
    button2.update();
    if (button2.read() == LOW) {
        writeString("DOWN");
    } else {
        writeString("UP  ");
    }
    writeString("\n");
    display.display();
    return;


//  display.drawRect(110, 0, 18, 18, WHITE);
//  display.display();
//  //    while (true) {
//  Serial.print("");
//  if (AHT10_functioning) {
//
//    uint8_t address = AHTX0_I2CADDR_DEFAULT;
//    bool success = read_data(address, humidity, temperature, false);
//    if (!success) {
//      initialize();
//    }
//    Serial.print("Humidity: ");
//    Serial.print(humidity);
//    Serial.print("%, temperature: ");
//    Serial.print(temperature);
//    Serial.print(" degrees Celcius\n");
//    // */
//
//    display.clearDisplay();
//    display.setCursor(0, 0);
//    writeString("\nTEMPERATURE = ");
//    writeFloatString(temperature);
//    writeString(", HUMIDITY = ");
//    writeFloatString(humidity);
//    writeString("\n");
//  } else {
//    writeString("AHT10 IS NOT RESPONDING\n");
//  }
//  IMU.update();
//  IMU.getAccel(&accelData);
//  float factor = 2.5;
//  int16_t x = CIRCLE_RADIUS + ((accelData.accelY * factor + 1) / 2) * (SCREEN_WIDTH - CIRCLE_RADIUS);
//  int16_t y = CIRCLE_RADIUS + ((accelData.accelX * factor + 1) / 2) * (SCREEN_HEIGHT - CIRCLE_RADIUS);
//  display.drawCircle(x, y, CIRCLE_RADIUS, SSD1306_WHITE);
//  display.display();
//
//  delay(200);
    //    }
}
