// This software is a tester for MindRoad Mind Challenge circuit boards,
// for revision 1 through 3, where rev 3 can be identified as the first
// board vision that can take either an Arduino Nano or a Raspberry Pi Pico

// The software displays data about the state of the connected hardware
// in order to allow for figuring out if your solder job was good or not.

// Usage instructions, (windows):
// To use it, install Arduino IDE. I use version 2.3.4. https://www.arduino.cc/en/software
// Then open this .ino file in Arduino IDE.
// Try compiling with check mark in the top left.
// It will most likely fail due to missing dependencies.
// To address this, open library manager (pile of books icon in the left sidebar)
// From there, search for each of these dependencies:
// Adafruit SSD1306 by Adafruit (I use version 2.5.13)
// FastIMU by LiquidCGS (I use version 1.2.8)
// Then try compiling again.

// If you're using Arduino Nano, the next step is to plug in your board.
// From the Tools dropdown menu, select
// Board: Arduino Nano
// Port: Whichever port disappears when you disconnect your board.
// Programmer: ATmega328P, or ATmega328P (Old bootloader), try one then the other

// If you're using Raspberry Pi Pico, you need to install the board support package
// for Raspberry Pi Pico. Plug it in, and select Raspberry Pi Pico under the tools -> board menu
// and the IDE should offer to install the board support package.
// If it doesn't, check the boards manager (in the left bar, second icon down)
// and install Arduino Mbed OS RP2040 Boards by Arduino (I use version 4.2.1)
// At this point you can try compiling and uploading the software, and it might
// upload okay, but nothing will display on the screen.
// This is because the Arduino SDK does not support changing I2C pins,
// at least in the version I use. The only fix I've found for this is to
// change the default pins in the board support package.
// For me, the relevant file can be found at
// C:\Users\Gabriel\AppData\Local\Arduino15\packages\arduino\hardware\mbed_rp2040\4.2.1\variants\RASPBERRY_PI_PICO\pins_arduino.h
// And I change these lines
//#define PIN_WIRE_SDA        (4u)
//#define PIN_WIRE_SCL        (5u)
// to this
//#define PIN_WIRE_SDA        (20u)
//#define PIN_WIRE_SCL        (21u)
// This will change everything you compile for Raspberry Pi Pico to use those pins,
// so remember to change back if you use an board which is wired with default I2C pins.
// After this, put the Raspberry Pi Pico in bootloader mode
// by holding the BOOTSEL physical button on the actual board while inserting the USB cable into the computer.
// A windows explorer window not unlike when you insert a USB flash drive should appear.
// Then use the Upload button in Arduino IDE (Arrow icon in the top left)

// Once the software is running on the board, you will have a set of values appear
// on the small attached screen if it's soldered correctly.
// The values on the screen from top to bottom are:
// One row for the AHT10 temperature and humidity sensor (dark blue 4 pins small board).
//   It shows the current ambient humidity in % and current temperature in Celcius
//   Or an error message to indicate the AHT10 did not respond.
// One row for the BMI160 inertial measurement unit/accelerometer (purple 4 pins small board)
//   It shows the current x and y axis rotations from the accelerometer,
//   as well as the temperature in celcius as measured by the BMI160 (it, too, has a thermometer).
//   Or an error message.
// One row for buttons, which display "DOWN" if they're currently being pressed
// Optionally one row for potentiometers (knobs to turn) displaying the 12-bit ADC (analog to digital converter) value,
//   meaning a value from 0 to 1023 that stretches from 0 V to 3.3 V.
//   This row is only relevant for Rev3 boards, so it only appears if you're using Raspberry Pi Pico,
//   or have set the ARDUINO_ON_REV3 macro below to 1.
// Finally, a circle bounded by a square in the bottom right which
//   graphically conveys the current measurement value of the BMI160.
//   This only appears if the BMI160 is responding.
// If one of the fields don't show the values you expect, check your
// solders again for connection, especially on the SCL and SDA pins.

// Have fun with your board!

// Author Gabriel Nilsson

#include <stdlib.h>
#include <inttypes.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FastIMU.h>

// If we have a new board, which can contain both a raspberry and an arduino,
// and we have an arduino mounted, set this to 1.
#define ARDUINO_ON_REV3 1

#if BOARD_VENDORID == 0x2e8a && BOARD_PRODUCTID == 0x00c0
#define RASPBERRY_PI_PICO 1
#else
#define RASPBERRY_PI_PICO 0
#endif

#if ARDUINO_ON_REV3 || RASPBERRY_PI_PICO
#define REV3_BOARD 1
#else
// Rev3 false implies that it's rev2 or rev 1, which both look the same
#define REV3_BOARD 0
#endif

// The status reflects our understanding of the current state of each sensor
enum class Status {
    Unknown,
    NotResponding,
    StoppedResponding,
    Ok,
    Measuring,
};

// IMU is short for Inertial Measurement Unit
// Our particular IMU is the BMI160 from Bosch.
#define IMU_ADDRESS 0x69  //Change to the address of the IMU
BMI160 IMU;               //Change to the name of any supported IMU!
calData calib = {0};    //Calibration data
AccelData accelData;      //Sensor data
Status BMI160_status = Status::Unknown;

// The AHT10 is a temperature and humidity sensor.
// The driver is a little wonky so we implement our own
// In addition to that, the device itself is also a little wonky -
// it tends to interfere with other I2C things on the same bus
// Therefore, we try some mitigations that nevertheless are not fully functioning.
// If your card freezes unexplainable, consider if your design really needs the AHT10.
uint8_t AHTX0_I2CADDR_DEFAULT = 0x38;                   // Default I2C address
uint8_t AHT10_CMD_CALIBRATE[3] = {0xE1, 0x08, 0x00};  // Calibration command for AHT10 sensor
uint8_t AHTX0_CMD_TRIGGER[3] = {0xAC, 0x33, 0x00};    // Trigger reading command
uint8_t AHTX0_CMD_SOFTRESET = 0xBA;                     // Soft reset command
uint8_t AHTX0_STATUS_BUSY = 0x80;                       // Status bit for busy
uint8_t AHTX0_STATUS_CALIBRATED = 0x08;                 // Status bit for calibrated
// Status of the AHT10.
Status AHT10_status = Status::Unknown;
// Remaining attempts to get a status,
// or remaining iterations until we do another measurement,
// depending on status Measuring or Ok
uint16_t AHT10_counter = 0;
// Last measured values
float AHT10_humidity = 0.0f;
float AHT10_temperature = 0.0f;


// The SSD1306 is our display.
// We don't print anything to the display unless we believe it was properly initalized (== ok is true)
bool SSD1306_ok = false;
// Pixel coordinate-related constants
// Don't change the screen width and height, it should match the physical screen
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
#define BOX_SIDE 16  // Size in pixels of the bounds of the gyro testing square
#define CIRCLE_RADIUS 3  // IMU rendering circle
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint16_t led_timing = 0;
uint16_t print_timing = 0;

// The driver uses malloc by default, and it fails without much reason.
// We statically allocate it instead, and assign it despite it being protected
// via a subclass hack
// We skip this on raspberry pi though
#if !RASPBERRY_PI_PICO
uint8_t display_buffer[SCREEN_WIDTH * ((SCREEN_HEIGHT + 7) / 8)];
class Adafruit_BufferEdit : public Adafruit_SSD1306 {
public:
    void setBuffer(uint8_t *new_buffer) { buffer = new_buffer; }
};
#endif

// Buttons
// Pins for buttons vary depending on hardware
#if RASPBERRY_PI_PICO
// If we're on raspberry pi pico
#define BUTTON1_PIN 7
#define BUTTON2_PIN 8
#define POTENTIOMETER1_PIN 26
#define POTENTIOMETER2_PIN 27
#define ONBOARD_LED_PIN 25
#define LED1_PIN 3
#define LED2_PIN 6
#define LED3_PIN 9
#define LED4_PIN 10

#else
#if ARDUINO_ON_REV3 == 1
#define BUTTON1_PIN 7
#define BUTTON2_PIN 8
#define POTENTIOMETER1_PIN A0
#define POTENTIOMETER2_PIN A1
#define ONBOARD_LED_PIN LED_BUILTIN
#define LED1_PIN 3
#define LED2_PIN 6
#define LED3_PIN 9
#define LED4_PIN 10
#else

#define BUTTON1_PIN 5
#define BUTTON2_PIN 4
#define POTENTIOMETER1_PIN A0
#define POTENTIOMETER2_PIN 
#define ONBOARD_LED_PIN LED_BUILTIN

#endif
#endif

////////////////////////////////////
// Here begins our AHT10 driver.
// The device has very few functions. Checking Soft reset, calibration, measurement and checking status.
// The specific byte values can be read from its data sheet or by looking at someone else's driver
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

uint8_t AHT10_get_status(uint8_t address, bool print) {
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
Status AHT10_calibrate(uint8_t address) {
    if (!AHT10_transmit(address, AHT10_CMD_CALIBRATE, 3)) {
        return Status::StoppedResponding;
    }
    Serial.print("Calibrating.\n");
    uint8_t status = AHT10_get_status(address, true);

    int attempts = 0;
    while ((status & AHTX0_STATUS_BUSY) != 0 && attempts < 100) {
        delay(20);
        status = AHT10_get_status(address, false);
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
    delay(20);
    if (!AHT10_transmit(address, &AHTX0_CMD_SOFTRESET, 1)) {
        // Let reset fail, it's optional, and instead have the repeating status be
        // the more reliable sign that the circuit is broken
        //return Status::NotResponding;
    }
    delay(20);
    uint8_t status = AHT10_get_status(address, true);
    int attempts = 0;
    while (status == 0xFF && attempts < 100) {
        status = AHT10_get_status(address, false);
        attempts += 1;
        delay(20);
    }
    if (status == 0xFF) {
        Serial.print("AHT10 didn't respond.");
        return Status::NotResponding;
    }
    return AHT10_calibrate(address);
}
// Start measurement
bool AHT10_read_data_start(uint8_t address) {
    return AHT10_transmit(address, AHTX0_CMD_TRIGGER, 3);
}
// When status no longer shows busy, we can read the values and convert it using their provided math
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
// Here ends the AHT10 driver
//////////////////////////////////////

// Helper functions for printing to the display
void writeString(char* text) {
    char* c = text;
    while (*c) {
        display.write(*c);
        c++;
    }
    if (print_timing == 5){
        Serial.print(text);
    }
}

void writeFloatString(float n) {
    char buffer[32];
#if RASPBERRY_PI_PICO
    sprintf(buffer, "%.1f", n);
#else
    dtostrf(n, 4, 1, buffer);
#endif
    writeString(buffer);
}
void writeIntString(uint32_t n) {
    char buffer[32];
#if RASPBERRY_PI_PICO
    sprintf(buffer, "%d", n);
#else
    itoa(n, buffer, 10);
#endif
    writeString(buffer);
}


// Start of our program proper
void setup() {
    // Set up the serial port that sends data via USB.
    // We could wait until (Serial == true) but that means that we wait until USB connection is established on
    // the raspberry pi pico. This would mean that the firmware does nothing unless a computer is connected and listening to the USB serial port
    Serial.begin(9600);
    delay(20);
    Serial.write("--- Restart ---\n");

    Serial.write("Wire begin\n");
    Wire.begin();
    Wire.setClock(10000); //400khz clock

    // Initialize the IMU. The IMU can be expected to work right away if it's connected properly, and keep working if so.
    Serial.write("IMU init\n");
    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) {
        Serial.print("Error initializing IMU: ");
        Serial.println(err);
        BMI160_status = Status::NotResponding;
    } else {
        BMI160_status = Status::Ok;
    }

    // Use our driver's init function to check for a response, soft reset, and calibrate
    Serial.write("AHT10 init\n");
    AHT10_status = AHT10_initialize();

    // Set up our display using the hack we wrote above.
    // But not if we're on raspberry pi
    Serial.write("Display init\n");
#if !RASPBERRY_PI_PICO
    static_cast<Adafruit_BufferEdit*>(&display)->setBuffer(display_buffer);
#endif
    // If this fails and ok is set to false, we write nothing to the display,
    // and instead print to the usb serial bus so we can diagnose the other devices
    // We don't always print to the usb serial bus because a refresh rate that seems
    // reasonable on the display is high enough to spam usb serial, and that would hide more rare errors.
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

    // Initialize our buttons with an internal pullup.
    // This means that we don't need any external resistors that connect the input line to voltage
    // in order to detect that the button has been connected to ground (been pressed)
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);

    pinMode(ONBOARD_LED_PIN, OUTPUT);
#if REV3_BOARD
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);
    pinMode(LED4_PIN, OUTPUT);

#endif
}

void loop() {
    if (led_timing < 10) {
        digitalWrite(ONBOARD_LED_PIN, 1);
#if REV3_BOARD
        digitalWrite(LED1_PIN, 1);
        digitalWrite(LED2_PIN, 0);
        digitalWrite(LED3_PIN, 0);
        digitalWrite(LED4_PIN, 0);
#endif
    } else if (led_timing < 20) {
        digitalWrite(ONBOARD_LED_PIN, 0);
#if REV3_BOARD
        digitalWrite(LED1_PIN, 0);
        digitalWrite(LED2_PIN, 1);
        digitalWrite(LED3_PIN, 0);
        digitalWrite(LED4_PIN, 0);
#endif
    } else if (led_timing < 30) {
        digitalWrite(ONBOARD_LED_PIN, 1);
#if REV3_BOARD
        digitalWrite(LED1_PIN, 0);
        digitalWrite(LED2_PIN, 0);
        digitalWrite(LED3_PIN, 1);
        digitalWrite(LED4_PIN, 0);
#endif
    } else if (led_timing < 40) {
        digitalWrite(ONBOARD_LED_PIN, 0);
#if REV3_BOARD
        digitalWrite(LED1_PIN, 0);
        digitalWrite(LED2_PIN, 0);
        digitalWrite(LED3_PIN, 0);
        digitalWrite(LED4_PIN, 1);
#endif
    } else {
        led_timing = 1;
    }
    led_timing++;
    if (print_timing > 40) {
        print_timing = 0;
        Serial.print("\n\n--\n");
    } else {
        print_timing ++;
    }



    // We must always set the cursor to the stop of the screen each time we start over
    // because otherwise we'll write way outside the screen after the first couple loops
    // Most calls to the display driver doesn't actually communicate with the screen.
    // Instead we manipulate an internal buffer, the one we statically allocate above,
    // and then send it in bulk one we call display.display().
    if (SSD1306_ok) {
        display.clearDisplay();
        display.setCursor(0, 0);      // Start at top-left corner
    }

    // If we already determined the AHT10 is not working correctly, we just print a static message and move on.
    // Each writeString or other write* function updates the internal buffer of the screen but doesn't affect the actual screen right away.
    if (AHT10_status == Status::Unknown) {
        writeString("AHT10: Unknown\n");
    } else if (AHT10_status == Status::NotResponding) {
        writeString("AHT10: No response\n");
    } else if (AHT10_status == Status::StoppedResponding) {
        writeString("AHT10: Stopped resp\n");
    } else if (AHT10_status == Status::Ok || AHT10_status == Status::Measuring) {
        uint8_t address = AHTX0_I2CADDR_DEFAULT;
        // The "Ok" status is reinterpreted as "Ok, and not currently measuring"
        if (AHT10_status == Status::Ok) {
            // We measure less often as another mitigation for the freezing behaviour described below
            if (AHT10_counter > 0) {
                AHT10_counter -= 1;
            } else {
                delay(50);
                if (AHT10_read_data_start(address)) {
                    AHT10_status = Status::Measuring;
                    AHT10_counter = 0;
                    delay(50);
                } else {
                    AHT10_status = Status::StoppedResponding;
                }
            }
        } else /* if (AHT10_status == Status::Measuring( */ {
            // This sensor freezes the arduino after a random amount of time for some reason.
            // That amount of time is larger if it gets some space between its communication and other sensor's
            delay(50);
            uint8_t status = AHT10_get_status(address, false);
            if ((status & AHTX0_STATUS_BUSY) != 0) {
                // Busy
                AHT10_counter += 1;
                if (AHT10_counter > 10000) {
                    AHT10_status = Status::StoppedResponding;
                    Serial.print("Failed to generate new data.\n");
                }
            } else {
                // Not busy, measurement done! Update our cached values by reference
                bool success = AHT10_read_data_result(address, status, AHT10_humidity, AHT10_temperature, false);
                if (success) {
                    AHT10_status = Status::Ok;
                    // We wait 100 iterations until we try to get temperature or humidity again,
                    // to not interact with the sensor more than we need to,
                    // since interaction seems related to the arduino freezing.
                    AHT10_counter = 100;
                } else {
                    AHT10_status = Status::StoppedResponding;
                }
            }
            delay(50);
        }

        // Print the latest cached values. If these are zero on the screen,
        // it's likely the measurement hasn't succeeded even once.
        // This may be a software bug or the device having gotten an electric shock.
        writeString("AHT10: H");
        writeIntString((uint32_t) AHT10_humidity);
        writeString("% T");
        writeFloatString(AHT10_temperature);
        writeString("C\n");
    }

    // Updating the inertial measurement unit, model number BMI160
    if (BMI160_status == Status::Unknown) {
        writeString("BMI160: Unknown\n");
    } else if (BMI160_status == Status::NotResponding) {
        writeString("BMI160: No response\n");
    } else if (BMI160_status == Status::StoppedResponding) {
        writeString("BMI160: Stopped resp\n");
    } else if (BMI160_status == Status::Ok) {
        // IMU.update() actually gets new data from the sensor.
        // The getAccel() and getTemp() calls get it from the storage in the class
        IMU.update();
        IMU.getAccel(&accelData);
        float temperature = IMU.getTemp();
        // To visualize the inertial measurement,
        // we visualize a little box in the bottom right corner,
        // with a circle that moves based on the angle of the board.
        // center_x and center_y is the center of the little box
        int16_t center_x = SCREEN_WIDTH - (BOX_SIDE / 2);
        int16_t center_y = SCREEN_HEIGHT - (BOX_SIDE / 2);
        float factor = 50.0f;

        // Screen coordinate for the circle
#if REV3_BOARD
        // On the new board raspberry pi board, the orientation of the accelerometer
        // is not the same as the old board, so we do this transformation
        int16_t x = -accelData.accelX * factor;
        int16_t y = accelData.accelY * factor;
#else
        // Note that y is rotation around the y axis, which means it maps to the x coordinate on screen, and vice versa.
        int16_t x = accelData.accelY * factor;
        int16_t y = accelData.accelX * factor;
#endif

        // Clamp the movement of the circle within the box
        int16_t extents = BOX_SIDE / 2 - 4;
        if (x > extents) {
            x = extents;
        } else if (x < -extents - 1) {
            x = -extents - 1;
        };
        if (y > extents) {
            y = extents;
        } else if (y < -extents - 1) {
            y = -extents - 1;
        };
        // If the display is on, draw the circle and its surrounding box
        if (SSD1306_ok) {
            display.drawRect(SCREEN_WIDTH - BOX_SIDE, SCREEN_HEIGHT - BOX_SIDE, BOX_SIDE, BOX_SIDE, WHITE);
            display.drawCircle(x + center_x, y + center_y, CIRCLE_RADIUS, SSD1306_WHITE);
        }

        // Also draw raw values, but scaled up since they are below 1 by default
        writeString("BMI160: X");
        writeIntString((uint32_t) (accelData.accelX * 100.0f));
        writeString("Y");
        writeIntString((uint32_t) (accelData.accelY * 100.0f));
        writeString(" T");
        writeIntString((uint32_t) temperature);
        writeString("C\n");
    }

    // Print UP or DOWN for each button, where DOWN is displayed when the button is pressed.
    // If the button doesn't change state when pressed, it's not connected correctly,
    // or the software has the wrong pins for the buttons
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

#if REV3_BOARD
    writeString("P1: ");
    writeIntString(analogRead(POTENTIOMETER1_PIN));
    writeString(", P2: ");
    writeIntString(analogRead(POTENTIOMETER2_PIN));
#endif
    // Finally we print our internal buffer.
    // You must remember this step in your own application if you use the display,
    // otherwise nothing will render at all.
    if (SSD1306_ok) {
        display.display();
    } else {
        // If the screen fails, we instead print the data to usb serial.
        // If so, we pause to give ourselves a chance to read the serial log
        delay(2000);
    }
}
