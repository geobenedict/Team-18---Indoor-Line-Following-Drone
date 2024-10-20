#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

// Define the camera model
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Define Wi-Fi credentials
const char *ssid = "ESP32cam";
const char *password = "123456789";

// Create a web server on port 80
WebServer server(80);

// Initialize UART for communication with flight controller (FC)
HardwareSerial fcSerial(1);  // Use Serial1 for FC communication (TX: GPIO 17, RX: GPIO 16)

// MSP command IDs
#define MSP_SET_RAW_RC 200

// Function to calculate MSP checksum
uint8_t calculateChecksum(uint8_t *data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Function to send MSP commands to the flight controller
void sendMSPCommand(uint8_t command, uint8_t *payload, uint8_t payloadSize) {
    // MSP message structure
    fcSerial.write('$');
    fcSerial.write('M');
    fcSerial.write('<');
    fcSerial.write(payloadSize);  // Payload size
    fcSerial.write(command);      // Command ID
    
    // Send the payload
    for (int i = 0; i < payloadSize; i++) {
        fcSerial.write(payload[i]);
    }
    
    // Calculate and send checksum
    uint8_t checksum = payloadSize ^ command;
    for (int i = 0; i < payloadSize; i++) {
        checksum ^= payload[i];
    }
    fcSerial.write(checksum);
}

// Function to handle incoming POST requests
void handlePost() {
    if (server.hasArg("plain")) {  // Check if there is data in the body
        String body = server.arg("plain");  // Get the body content
        Serial.print("Received body: ");
        Serial.println(body);  // Print received data

        DynamicJsonDocument doc(1024);  // Create a JSON document to parse the incoming data
        DeserializationError error = deserializeJson(doc, body);
        if (error) {
            Serial.println("Failed to parse JSON");
            server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }

        // Extract lr and curve values
        int lr = doc["lr"];
        int curve = doc["curve"];

        Serial.print("Received lr: ");
        Serial.println(lr);
        Serial.print("Received curve: ");
        Serial.println(curve);

        // Prepare the MSP payload (channel data)
        uint16_t rcData[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};  // Default RC values (neutral)
        rcData[0] = 1500 + lr;  // Adjust roll (left/right) based on `lr`
        rcData[1] = 1500 + curve;  // Adjust pitch based on `curve`

        // Create MSP payload with raw RC data
        uint8_t payload[16];
        for (int i = 0; i < 8; i++) {
            payload[2 * i] = rcData[i] & 0xFF;
            payload[2 * i + 1] = (rcData[i] >> 8) & 0xFF;
        }

        // Send the MSP command to the flight controller
        sendMSPCommand(MSP_SET_RAW_RC, payload, 16);

        server.send(200, "application/json", "{\"status\":\"success\"}");  // Send success response
    } else {
        Serial.println("No data received in POST request");
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
    }
}

void startCameraServer();
void setupLedFlash(int pin);

// Initialize the camera and WiFi
void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();

    // Initialize UART for FC communication
    fcSerial.begin(115200, SERIAL_8N1, 16, 17);  // Serial1, TX (GPIO 17), RX (GPIO 16)

    // Camera configuration
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_UXGA;
    config.pixel_format = PIXFORMAT_JPEG;  // for streaming
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    // Initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    WiFi.softAP(ssid, password);  // Start WiFi Access Point

    startCameraServer();  // Start the camera server

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");

    // Define the POST route for receiving commands
    server.on("/commands", HTTP_POST, handlePost);
    server.begin();  // Start the server
    Serial.println("Server started and listening for commands");
}

void loop() {
    // Keep the loop running for handling POST requests
    delay(10000);
}
