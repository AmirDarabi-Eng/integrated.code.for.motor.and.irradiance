#include <Wire.h>
#include <RTClib.h>

// Pin settings and motor control setup
const int re_not = 7; // [not]_Read_Enable
const int de = 8;     // Data_Enable
String inputString;
boolean stringComplete = false;
double error;



// Pyranometer pin
#define PYRANOMETER_PIN A0 // Ensure your pyranometer is connected to a free analog pin

float previousTilt;
float previousAzimuth;

const float mvc = .8058608059;  // mv per count. STATIONARY PV INPUT
const float mvc1 = .8058608059; // mv per count. TRACKING PV INPUT

// Stationary PV INPUT
float counts = 0;
float mv = 0;

// Tracking PV INPUT
float counts1 = 0;
float mv1 = 0;

RTC_DS3231 rtc;

// Motor control functions
void send_command_2(int motor_id, char command, int parameter)
{
    Serial1.print('#');
    Serial1.print(motor_id);
    Serial1.print(command);
    Serial1.print(parameter);
    Serial1.print('\r');
    delay(100);
}

void send_command_1(int motor_id, char command)
{
    Serial1.print('#');
    Serial1.print(motor_id);
    Serial1.print(command);
    Serial1.print('\r');
    delay(100);
}

void move(int motor_id, double degrees)
{
    if (degrees < 0.0)
    {
        // Change direction
        send_command_2(motor_id, 'd', 0);
        degrees *= -1;
    }
    else
    {
        send_command_2(motor_id, 'd', 1);
    }
    double steps_floating = (((degrees * 10.0 * (double)16.0 * (double)(1.0 / 1.8))));
    int steps = abs(rint(steps_floating));

    // Error correction (due to rounding)
    error += steps_floating - (double)steps;
    steps += (int)(error);
    error -= (int)(error);

    send_command_2(motor_id, 's', steps);
    send_command_1(motor_id, 'A');

    flush_serialport();
}

int get_position(int motor_id)
{
    char rx[50];
    flush_serialport();
    send_command_1(motor_id, 'I');
    read_port(rx);
    *rx = ' ';
    *(rx + 1) = ' ';
    
    return atoi(rx);
}

void init_motors()
{
    // Set read and data enable to active
    digitalWrite(re_not, LOW);
    digitalWrite(de, HIGH);
    
    // Settings motor for elevation (motor id: 1):
    send_command_2(1, 'U', 1);      // Error correction: after travel
    send_command_2(1, 'F', 1);      // Automatic correction: on
    send_command_2(1, 'r', 50);     // Idle phase current: 50%
    send_command_2(1, 'i', 90);     // Phase current: 90%
    send_command_2(1, 'o', 1000);   // Max frequency 1: 400Hz
    send_command_2(1, 'n', 1000);   // Max frequency 2: 1000Hz
    send_command_2(1, 'b', 8956);   // Acceleration ramp frequency: 20Hz/ms, corresponds to 8965 (refer to programming reference, Nanotec)
    
    // Settings motor for azimuth (motor id: 3):
    send_command_2(3, 'U', 1);     
    send_command_2(3, 'F', 1);
    send_command_2(3, 'r', 50);
    send_command_2(3, 'i', 90);
    send_command_2(3, 'o', 1000);
    send_command_2(3, 'n', 1000);
    send_command_2(3, 'b', 8956);
}

void send_user_instructions()
{
    Serial.println();
    Serial.println("Available command list:");
    Serial.println("SETUP-TILT - prompts user to align tracker tilt to zero (horizontal) as reference start position");
    Serial.println("SETUP-AZ - prompts user to align tracker azimuth with sun azimuth as reference start position");
    Serial.println("MEASURE - only reads analog inputs");
    Serial.println("Timestamp in the format 2032-04-17 14:31:25 - calls SPA, motor position update, and reads analog inputs");
}

void setup()
{
    // Set pins for RS485 transceiver
    pinMode(re_not, OUTPUT);
    pinMode(de, OUTPUT);

    // Configure two serial ports (one to the PC, one to the motor controllers)
    Serial.begin(9600);  // Initialize serial communication with computer at 9600 baud
    Serial1.begin(9600); // tx, rx UART pins (see https://docs.arduino.cc/tutorials/nano-33-iot/uart)

    // Configure analog inputs on the Arduino to read at 12-bit resolution
    analogReadResolution(12);

    // Initialize motors
    init_motors();

    // Initialize RTC
    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
        while (1);
    }

    if (rtc.lostPower())
    {
        Serial.println("RTC lost power, setting the time!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    // Print a list of possible user commands to the PC serial stream
    send_user_instructions();
}

void serialEvent()
{
    while (Serial.available())
    {
        char inChar = (char)Serial.read();
        if (inChar == '\n')
        {
            stringComplete = true;
            return;
        }
        inputString += inChar;
    }
}

void read_port(char *DataString)
{
    char rx;
    int i = 0;
    while (Serial1.available() > 0)
    {
        rx = Serial1.read();
        DataString[i] = rx;
        i++;
    }
}

void flush_serialport()
{
    char rx;
    while (Serial1.available() > 0)
    {
        rx = Serial1.read();
        rx = rx;
    }
}

/// AZIMUTH ///
double Azimuth_Setup_Previous = 0;
double Azimuth_Setup = 0;
/////////////////////////////////////////////////////////////////////
// Tilt Angle Motor Movement
double Tilt_Previous = 0;
double Tilt = 0;
// Azimuth Angle Motor Movement
double Azimuth_Previous = 0;
double Azimuth = 0;

void loop()
{
    char rx[50];
    double error;
    int motor_id; // This method can have slight installation, positioning, and timing errors.
    double degrees;
    error = 0;
    int SampleCount1;
    float SampleBuffer1;
    int SampleCount;
    float SampleBuffer;

    String data = Serial.readStringUntil('\n'); // Read the incoming data string until a newline character is received
    int switch_control = 0;
    if (data == "SETUP-TILT")
    {
        switch_control = 1;
    }
    if (data == "SETUP-AZ")
    {
        switch_control = 2;
    }
    if (data == "")
    {
        switch_control = 99;
    }

    switch (switch_control)
    {
    case 99: // No data received
        Serial.println("No command received, waiting for new input...");
        delay(1000);
        break;

    case 1: // SETUP-TILT
        Serial.println();
        Serial.println("Enter tilt movement in degrees to place in horizontal (tilt = 0):");
        while (!Serial.available())
        {
            degrees = Serial.parseFloat(); // Read input degrees from Serial Monitor}
        }

        move(1, degrees); // Tilt motor controller has ID = 1

        while (get_position(1) != get_position(1))
        {
            // Print the message with the input degrees
            Serial.print("Changing tilt angle by ");
            Serial.print(degrees);
            Serial.println(" degrees...");
        }
        if (get_position(1) == get_position(1))
        {
            Serial.println("...Movement complete");
        }
        break;

    case 2: // SETUP-AZ
        Serial.println();
        Serial.println("Enter azimuth movement in degrees to place inline with current sun azimuth:");
        while (!Serial.available())
        {
            degrees = Serial.parseFloat(); // Read input degrees from Serial Monitor}
        }

        Serial.println();
        Serial.print("Changing azimuth angle by ");
        Serial.print(degrees);
        Serial.println(" degrees...");

        move(3, degrees); // Azimuth motor controller has ID = 3

        while (get_position(3) != get_position(3))
        {
            Serial.print("Changing tilt angle by ");
            Serial.print(degrees);
            Serial.println(" degrees...");
        }

        if (get_position(3) == get_position(3))
        {
            Serial.println("...Movement complete");
        }

        break;
    } // End switch statement for input command

    // Measure and print irradiance data
    measureIrradiance();
} // End main loop

void measureIrradiance()
{
    // Read pyranometer value
    int pyranometerValue = analogRead(PYRANOMETER_PIN);

    // Convert to millivolts
    float voltage = pyranometerValue * (3.3 / 4095.0); // Adjust for 3.3V reference and 12-bit resolution

    // Assuming calibration factor is 1 mV per W/m^2 for simplicity
    float irradiance = voltage * 1000.0;

    // Get current time and date from RTC
    DateTime now = rtc.now();

    // Print data to Serial Monitor
    Serial.print("Timestamp: ");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(" - Irradiance: ");
    Serial.print(irradiance);
    Serial.println(" W/m^2");

    delay(1000); // Delay for 1 second before the next measurement
}
