#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <stdlib.h>

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
float AHT10_humidity = 0.0f;
float AHT10_temperature = 0.0f;


#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
#define BOX_SIDE 16  // Size in pixels of the bounds of the gyro testing square

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
uint8_t display_buffer[SCREEN_WIDTH * ((SCREEN_HEIGHT + 7) / 8)];
#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16
bool SSD1306_ok = false;

#if BOARD_VENDORID == 0x2e8a && BOARD_PRODUCTID == 0x00c0
// If we're on raspberry pi pico
#define BUTTON1_PIN 7
#define BUTTON2_PIN 8
#else
#define BUTTON1_PIN 5
#define BUTTON2_PIN 4
#endif

class Adafruit_BufferEdit : public Adafruit_SSD1306 {
public:
    void setBuffer(uint8_t *new_buffer) { buffer = new_buffer; }
};

bool AHT10_transmit(uint8_t address, uint8_t buffer[], uint8_t buffer_size) {
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
uint8_t AHT10_receive(uint8_t address, uint8_t out_buffer[], uint8_t buffer_size) {
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
    uint8_t received_length = AHT10_receive(address, buffer, 1);
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
Status calibrate(uint8_t address) {
    if (!AHT10_transmit(address, AHT10_CMD_CALIBRATE, 3)) {
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

Status AHT10_initialize() {
    uint8_t address = AHTX0_I2CADDR_DEFAULT;
    Wire.begin(address);
    delay(20);
    if (!AHT10_transmit(address, &AHTX0_CMD_SOFTRESET, 1)) {
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
bool AHT10_read_data_start(uint8_t address) {
    return AHT10_transmit(address, AHTX0_CMD_TRIGGER, 3);
}
bool AHT10_read_data_result(uint8_t address, uint8_t status, float& out_humidity, float& out_temperature, bool print) {
    if ((status & AHTX0_STATUS_BUSY) != 0) {
        Serial.println("read_data_result called with busy status");
        return false;
    }
    const uint8_t buffer_size = 6;
    uint8_t buffer[buffer_size];
    uint8_t received_length = AHT10_receive(address, buffer, buffer_size);
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
    if (SSD1306_ok) {
        char* c = text;
        while (*c) {
            display.write(*c);
            c++;
        }
    } else {
        Serial.print(text);
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
    AHT10_status = AHT10_initialize();

    Serial.write("Display init\n");
    static_cast<Adafruit_BufferEdit*>(&display)->setBuffer(display_buffer);
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
        SSD1306_ok = true;

        display.setTextSize(1);       // Normal 1:1 pixel scale
        display.setTextColor(WHITE);  // Draw white text
        display.cp437(true);          // Use full 256 char 'Code Page 437' font
        display.clearDisplay();
        display.display();
    } else {
        SSD1306_ok = false;
        Serial.println("SSD1306 allocation failed");
    }

#if BOARD_VENDORID == 0x2e8a && BOARD_PRODUCTID == 0x00c0
    // If we're on raspberry pi pico
    unsigned button1_pin = 7;
    unsigned button2_pin = 8;
#else
    unsigned button1_pin = 5;
    unsigned button2_pin = 4;
#endif

    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
}

void loop() {
    if (SSD1306_ok) {
        display.clearDisplay();
        display.setCursor(0, 0);      // Start at top-left corner
    }

    if (AHT10_status == Status::Unknown) {
        writeString("AHT10: Unknown\n");
    } else if (AHT10_status == Status::NotResponding) {
        writeString("AHT10: No response\n");
    } else if (AHT10_status == Status::StoppedResponding) {
        writeString("AHT10: Stopped responding\n");
    } else if (AHT10_status == Status::Ok) {
        uint8_t address = AHTX0_I2CADDR_DEFAULT;
        if (!AHT10_read_data_start(address)) {
            AHT10_status = Status::StoppedResponding;
        }
        int attempts = 0;
        uint8_t status = get_status(address, false);
        while ((status & AHTX0_STATUS_BUSY) != 0 && attempts < 100) {
            delay(2000);
            status = get_status(address, false);
        }
        if ((status & AHTX0_STATUS_BUSY) != 0) {
            Serial.print("Failed to generate new data.\n");
            AHT10_status = Status::StoppedResponding;
        } else {
            bool success = AHT10_read_data_result(address, status, AHT10_humidity, AHT10_temperature, false);
            //writeString("AHT10: H22 T26\n");
            writeString("AHT10: H");
            writeIntString((uint32_t) AHT10_humidity);
            writeString("% T");
            //writeIntString((uint32_t) temperature);
            writeFloatString(AHT10_temperature);
            writeString("C\n");
        }
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
        int16_t center_x = SCREEN_WIDTH - (BOX_SIDE / 2);
        int16_t center_y = SCREEN_HEIGHT - (BOX_SIDE / 2);
        float factor = 50.0f;
        int16_t x = accelData.accelY * factor;
        int16_t y = accelData.accelX * factor;

        int16_t extents = BOX_SIDE / 2 - 4;
        if (x > extents) {
            x = extents;
        } else if (x < -extents) {
            x = -extents;
        };
        if (y > extents) {
            y = extents;
        } else if (y < -extents) {
            y = -extents;
        };
        if (SSD1306_ok) {
            display.drawRect(SCREEN_WIDTH - BOX_SIDE, SCREEN_HEIGHT - BOX_SIDE, BOX_SIDE, BOX_SIDE, WHITE);
            display.drawCircle(x + center_x, y + center_y, CIRCLE_RADIUS, SSD1306_WHITE);
        }

        //writeString("BMI160: X0Y0 T26\n");
        writeString("BMI160: X");
        writeIntString((uint32_t) (accelData.accelX * 100.0f));
        writeString("Y");
        writeIntString((uint32_t) (accelData.accelY * 100.0f));
        writeString(" T");
        writeIntString((uint32_t) temperature);
//        writeFloatString(temperature);
        writeString("C\n");
//        writeString("BMI160: X0Y0 T26\n");
    }

    //writeString("B1: UP  , B2: DOWN");
    writeString("B1: ");
    if (digitalRead(BUTTON1_PIN) == LOW) {
        writeString("DOWN");
    } else {
        writeString("UP  ");
    }
    writeString(", B2: ");
    if (digitalRead(BUTTON2_PIN) == LOW) {
        writeString("DOWN");
    } else {
        writeString("UP  ");
    }
    writeString("\n");
    if (SSD1306_ok) {
        display.display();
    } else {
        // Give ourselves a chance to read the serial log
        delay(2000);
    }
}
