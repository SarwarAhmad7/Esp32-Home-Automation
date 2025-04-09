Home Automation System with ESP32 and Blynk
Overview

This project implements a home automation system using an ESP32 microcontroller, various sensors, and the Blynk IoT platform. The system allows for remote monitoring and control of home appliances, environmental sensing, and security features.
Features

    Environmental Monitoring:

        Temperature and humidity sensing (DHT11)

        Gas/smoke detection (MQ2 sensor)

    Security Features:

        Motion detection (PIR sensor)

        Proximity sensing (Ultrasonic sensor)

    Appliance Control:

        4 relay outputs for controlling devices (LED, fan, door lock, water motor)

        Buzzer for alarms

    Remote Control:

        Blynk mobile app integration for remote monitoring and control

        Real-time sensor data visualization
        
  Hardware Components

    ESP32 microcontroller

    DHT11 temperature and humidity sensor

    MQ2 gas/smoke sensor

    PIR motion sensor

    HC-SR04 ultrasonic sensor

    4-channel relay module

    Buzzer

    Breadboard and jumper wires

    Power supply

Pin Configuration
Component	ESP32 GPIO Pin
DHT11	4
Relay 1 (LED)	23
Relay 2 (FAN)	22
Relay 3 (Door)	21
Relay 4 (Motor)	19
Buzzer	32
MQ2 (Analog)	34
PIR	27
Ultrasonic Trig	5
Ultrasonic Echo	18

Blynk Configuration

    Create a new project in Blynk app

    Set Template ID: BLYNK_TEMPLATE_ID

    Set Template Name: "Home Automation"

    Add the following virtual pins:

        V0: Temperature (Value Display)

        V1: Humidity (Value Display)

        V2: Fan Status (LED)

        V3: Buzzer Status (LED)

        V5: Gas Concentration (Gauge)

        V7: Distance (Value Display)

        V8: Relay 1 Control (Switch)

        V9: Relay 2 Control (Switch)

        V10: Relay 3 Control (Switch)

        V11: Relay 4 Control (Switch)

Setup Instructions
    Install required libraries:

        Blynk by Volodymyr Shymanskyy

        DHT sensor library by Adafruit

    Update the following in the code:

        BLYNK_TEMPLATE_ID

        BLYNK_AUTH_TOKEN

        WIFI_SSID

        WIFI_PASSWORD

    Upload the code to ESP32

    Connect all hardware components as per pin configuration

Automatic Controls

    Fan Control: Turns on when temperature exceeds 45°C

    Buzzer: Activates when gas concentration exceeds threshold (1500)

    Light: Turns on when motion is detected (PIR sensor)

    Water Motor: Activates when distance exceeds 5cm (ultrasonic sensor)

Manual Controls

All relays can be manually controlled through the Blynk app:

    V8: Relay 1 (LED)

    V9: Relay 2 (Fan)
    
    V10: Relay 3 (Door Lock)

    V11: Relay 4 (Water Motor)

Serial Monitoring

The system outputs all sensor readings and relay states to the serial monitor at 115200 baud rate for debugging purposes.
License

This project is open-source and available for modification and distribution.
