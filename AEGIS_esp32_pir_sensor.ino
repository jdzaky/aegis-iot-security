/**
 * @file main.cpp
 * @brief ESP32-CAM AI Motion Alert System with Telegram and Google Sheets Logging
 * @version 2.0
 * @date 2025-10-20
 *
 * @details This project uses an ESP32-CAM to detect motion, optionally classify
 * the image using an Edge Impulse model, and send alerts via Telegram. It also
 * logs environmental data from a DHT22 sensor and system stats to Google Sheets.
 *
 * The architecture is built on FreeRTOS for non-blocking, stable operation.
 * Key features include:
 * - Multi-tasking for motion detection, image processing, and communication.
 * - Non-blocking WiFi reconnection and Telegram message handling.
 * - PIR motion detection with configurable AI classification (on/off).
 * - DHT22 sensor for temperature/humidity monitoring.
 * - High-temperature alert mode.
 * - Remote control via Telegram bot commands.
 * - Periodic logging of data to a Google Sheet via a Web App.
 * - Watchdog timer (WDT) to ensure system stability.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <DHT.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"

// --- USER CONFIGURATION ---
// Copy config.h.example to config.h and enter your credentials.
#include "config.h"
// Include your Edge Impulse model's header file here.
#include "Jdzaky-project-1_inferencing.h"

// --- Pin Definitions & Constants ---
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22
#define PIR_PIN          13
#define FLASH_LED_PIN    4
#define DHT_PIN          15
#define DHT_TYPE         DHT22
#define WDT_TIMEOUT_MS   180000 // Watchdog timeout: 180 seconds

// Google Sheets Configuration
#define GOOGLE_SHEETS_SEND_INTERVAL 120000 // Send every 2 minutes

// --- Global Variables ---
volatile bool pirEnabled = true;
volatile bool mlProcessing = false;
bool flashState = false;
bool systemReady = false;
bool alertMode = false; // High-temperature alert mode
unsigned long lastGoogleSheetsSend = 0;

// Detection stats
String lastDetectionLabel = "None";
float lastDetectionConfidence = 0.0f;
unsigned long lastDetectionTimestamp = 0;

// Classification stats
uint32_t totalMotionCount = 0;
uint32_t classificationCounts[EI_CLASSIFIER_LABEL_COUNT] = {0};

// AI on motion mode
bool mlOnMotion = true; // Default: active

// DHT sensor
DHT dht(DHT_PIN, DHT_TYPE);
float lastTemperature = 0.0f;
float lastHumidity = 0.0f;
unsigned long lastDHTSend = 0;
const unsigned long DHT_SEND_INTERVAL = 300000; // 5 minutes

// --- FreeRTOS & Communication ---
struct TelegramMessage {
    char chatId[16];
    char text[512];
    bool isPhoto;
};

typedef enum {
    TASK_PHOTO_ONLY,
    TASK_CLASSIFY,
    TASK_MOTION_ALERT
} ProcessingTask_t;

TaskHandle_t telegramTaskHandle = NULL;
TaskHandle_t motionTaskHandle = NULL;
TaskHandle_t processingTaskHandle = NULL;
QueueHandle_t xProcessingQueue;
QueueHandle_t xResponseQueue;
SemaphoreHandle_t xWiFiMutex;
SemaphoreHandle_t xBufferMutex;
SemaphoreHandle_t xTelegramMutex;
WiFiClientSecure telegramClient;

// Forward declarations
void sendTelegramResponse(String chatId, String message);
bool checkWiFiConnection();
void updateFlashLED();
String sendPhotoToTelegram();
String captureAndClassify();
void handleNewMessages(int numNewMessages, UniversalTelegramBot& bot);

// ======================================================
//               Google Sheets Logging
// ======================================================
void sendToGoogleSheets() {
    if (millis() - lastGoogleSheetsSend < GOOGLE_SHEETS_SEND_INTERVAL) {
        return;
    }

    if (!checkWiFiConnection()) {
        Serial.println("⚠️ Google Sheets: WiFi not available");
        return;
    }

    esp_task_wdt_reset();

    String detectionLabel = lastDetectionLabel;
    if (detectionLabel.length() == 0) {
        detectionLabel = "None";
    }

    String url = String(GOOGLE_SHEETS_URL) +
                 "?token=" + String(GOOGLE_SHEETS_TOKEN) +
                 "&temp=" + String(lastTemperature, 2) +
                 "&humidity=" + String(lastHumidity, 2) +
                 "&detection=" + detectionLabel +
                 "&motion=" + String(totalMotionCount);

    Serial.println("📊 Sending to Google Sheets...");
    Serial.println("URL: " + url);

    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure();

    http.begin(client, url);
    int httpCode = http.GET();

    if (httpCode == 200) {
        String response = http.getString();
        Serial.println("✅ Google Sheets: Data sent successfully");
        Serial.println("Response: " + response);
    } else {
        Serial.printf("❌ Google Sheets Error: HTTP %d\n", httpCode);
    }

    http.end();
    lastGoogleSheetsSend = millis();
    esp_task_wdt_reset();
}

void googleSheetsTask(void *pvParameters) {
    Serial.println("Google Sheets Task started.");
    while (!systemReady) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    for (;;) {
        esp_task_wdt_reset();
        sendToGoogleSheets();
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
}

// ======================================================
//              DHT Sensor & Alert Mode Task
// ======================================================
String readDHTSensor() {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
        return "❌ Failed to read from DHT22 sensor.";
    }

    lastTemperature = t;
    lastHumidity = h;

    String message = "🌡️ *Temp & Humidity*\n";
    message += "━━━━━━━━━━━━━━━\n";
    message += "🌡️ Temp: *" + String(t, 1) + "°C*\n";
    message += "💧 Humidity: *" + String(h, 1) + "%*\n";
    message += "⏰ Uptime: " + String(millis() / 1000) + "s";
    return message;
}

void dhtMonitorTask(void *pvParameters) {
    Serial.println("DHT Monitor Task started.");
    while (!systemReady) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    unsigned long lastAlertModeCheck = 0;
    const unsigned long ALERT_CHECK_INTERVAL = 10000; // Check every 10s

    for (;;) {
        esp_task_wdt_reset();

        if (millis() - lastAlertModeCheck >= ALERT_CHECK_INTERVAL) {
            float t = dht.readTemperature();
            if (!isnan(t)) {
                lastTemperature = t;
                lastHumidity = dht.readHumidity();

                if (t >= 35.0f && !alertMode) {
                    alertMode = true;
                    updateFlashLED();
                    sendTelegramResponse(String(GROUP_CHAT_ID), "⚠️ *ALERT MODE ON*\nHigh temperature detected: " + String(t, 1) + "°C");
                    Serial.println("🔥 Alert Mode ACTIVATED");
                } else if (t < 33.0f && alertMode) {
                    alertMode = false;
                    updateFlashLED();
                    sendTelegramResponse(String(GROUP_CHAT_ID), "✅ *Alert mode OFF*\nTemperature normal: " + String(t, 1) + "°C");
                    Serial.println("✅ Alert Mode DEACTIVATED");
                }
            }
            lastAlertModeCheck = millis();
        }

        if (millis() - lastDHTSend >= DHT_SEND_INTERVAL) {
            if (checkWiFiConnection()) {
                String dhtData = readDHTSensor();
                sendTelegramResponse(String(GROUP_CHAT_ID), dhtData);
                Serial.println("✅ DHT data sent to Telegram");
                lastDHTSend = millis();
            } else {
                Serial.println("⚠️ WiFi not available, skipping DHT send");
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// ======================================================
//               WiFi Connection Manager
// ======================================================
bool checkWiFiConnection() {
    if (xSemaphoreTake(xWiFiMutex, 5000 / portTICK_PERIOD_MS) != pdTRUE) {
        Serial.println("checkWiFiConnection: Failed to lock WiFi mutex (timeout).");
        return false;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected. Attempting reconnect...");
        WiFi.disconnect();
        delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        int retries = 0;
        while (WiFi.status() != WL_CONNECTED && retries < 10) {
            delay(500);
            Serial.print(".");
            retries++;
            esp_task_wdt_reset();
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi reconnected!");
            xSemaphoreGive(xWiFiMutex);
            return true;
        } else {
            Serial.println("\nFailed to reconnect WiFi");
            xSemaphoreGive(xWiFiMutex);
            return false;
        }
    }

    xSemaphoreGive(xWiFiMutex);
    return true;
}


// ======================================================
//          Non-Blocking Telegram Response Sender
// ======================================================
void sendTelegramResponse(String chatId, String message) {
    TelegramMessage response;
    memset(&response, 0, sizeof(response));
    strncpy(response.chatId, chatId.c_str(), sizeof(response.chatId) - 1);
    strncpy(response.text, message.c_str(), sizeof(response.text) - 1);
    response.isPhoto = false;

    if (xQueueSend(xResponseQueue, &response, 0) != pdPASS) {
        Serial.println("Response queue full, dropping message");
    }
}

// ======================================================
//             Image Processing & AI Task Manager
// ======================================================
String executeTaskWithRetry(ProcessingTask_t task, int maxRetries = 2) {
    String result;
    int attempt = 0;
    while (attempt < maxRetries) {
        esp_task_wdt_reset();
        switch (task) {
            case TASK_PHOTO_ONLY:
                result = sendPhotoToTelegram();
                break;
            case TASK_CLASSIFY:
                result = captureAndClassify();
                break;
            case TASK_MOTION_ALERT:
                vTaskDelay(500 / portTICK_PERIOD_MS);
                result = sendPhotoToTelegram();
                break;
        }
        if (result.indexOf("Failed") == -1 && result.indexOf("Error") == -1 && result.length() > 0) {
            return result;
        }
        attempt++;
        if (attempt < maxRetries) {
            Serial.printf("Retrying task... attempt %d/%d\n", attempt + 1, maxRetries);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }
    return result;
}

void processingManagerTask(void *pvParameters) {
    Serial.println("Processing Manager Task started.");
    ProcessingTask_t task_to_do;

    for (;;) {
        if (xQueueReceive(xProcessingQueue, &task_to_do, portMAX_DELAY) == pdPASS) {
            mlProcessing = true;
            Serial.println("Processing task received!");
            esp_task_wdt_reset();

            String result = executeTaskWithRetry(task_to_do, 2);
            String message;

            switch (task_to_do) {
                case TASK_PHOTO_ONLY:
                    message = (result == "Photo sent") ? "✅ Photo sent successfully." : "⚠️ Failed to send photo: " + result;
                    break;
                case TASK_CLASSIFY:
                    if (result.length() == 0) {
                        message = "⚠️ Classification result was empty (unexpected error).";
                    } else if (result.startsWith("Failed") || result.startsWith("Error")) {
                        message = "⚠️ " + result;
                    } else {
                        message = "✅ Classification: " + result;
                    }
                    break;
                case TASK_MOTION_ALERT:
                     message = (result == "Photo sent") ? "✅ Motion detected! Photo sent." : "⚠️ Motion detected! Failed to send photo: " + result;
                    break;
            }

            sendTelegramResponse(String(GROUP_CHAT_ID), message);
            mlProcessing = false;
            heap_caps_print_heap_info(MALLOC_CAP_8BIT);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}


// ======================================================
//                   Telegram Bot Task
// ======================================================
void telegramTask(void *pvParameters) {
    Serial.println("Telegram Task started.");
    unsigned long lastUpdate = 0;
    const unsigned long UPDATE_INTERVAL = 2000;
    WiFiClientSecure clientTCP;
    clientTCP.setInsecure();
    UniversalTelegramBot bot(BOT_TOKEN, clientTCP);

    while (!systemReady) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    bot.getUpdates(bot.last_message_received + 1); // Clear old messages

    // Send welcome message to group
    bool welcomeSent = false;
    int welcomeRetries = 0;
    while (!welcomeSent && welcomeRetries < 3) {
        if (checkWiFiConnection()) {
            String welcome = "✅ System Online.\nSend /start for a list of commands.";
            if (bot.sendMessage(String(GROUP_CHAT_ID), welcome, "")) {
                Serial.println("Welcome message sent to group");
                welcomeSent = true;
            } else {
                Serial.println("Failed to send welcome message, retrying...");
                welcomeRetries++;
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
        } else {
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            welcomeRetries++;
        }
    }

    for (;;) {
        esp_task_wdt_reset();
        TelegramMessage responseMsg;
        int processedResponses = 0;

        // Send pending responses from the queue
        while (xQueueReceive(xResponseQueue, &responseMsg, 0) == pdPASS && processedResponses < 3) {
            if (checkWiFiConnection()) {
                if (!bot.sendMessage(responseMsg.chatId, responseMsg.text, "Markdown")) {
                    Serial.printf("Failed to send response: %s\n", responseMsg.text);
                }
                processedResponses++;
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
        }

        // Check for new incoming messages
        if (millis() - lastUpdate >= UPDATE_INTERVAL && checkWiFiConnection()) {
            int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
            if (numNewMessages > 0) {
                Serial.printf("Processing %d new messages\n", numNewMessages);
                handleNewMessages(numNewMessages, bot);
            }
            lastUpdate = millis();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


// ======================================================
//              Motion Detection Task
// ======================================================
void motionTask(void *pvParameters) {
    Serial.println("Motion Detection Task started.");
    const int MOTION_THRESHOLD = 15;
    const unsigned long MOTION_COOLDOWN_MS = 30000;
    const unsigned long MIN_MOTION_DURATION_MS = 2500;
    const unsigned long POLLING_DELAY_HIGH_MS = 200;
    const unsigned long POLLING_DELAY_LOW_MS = 500;

    unsigned long lastMotionAlert = 0;
    unsigned long motionStartTime = 0;
    int consecutiveMotion = 0;
    bool motionInProgress = false;

    for (;;) {
        esp_task_wdt_reset();

        if (pirEnabled && digitalRead(PIR_PIN) == HIGH) {
            if (consecutiveMotion == 0) {
                motionStartTime = millis();
                motionInProgress = true;
                Serial.println("Motion started, validating...");
            }
            consecutiveMotion++;
            unsigned long motionDuration = millis() - motionStartTime;

            if (consecutiveMotion >= MOTION_THRESHOLD &&
                motionDuration >= MIN_MOTION_DURATION_MS &&
                !mlProcessing &&
                (millis() - lastMotionAlert > MOTION_COOLDOWN_MS)) {
                
                Serial.printf("✅ Validated Motion! (count=%d, duration=%lums)\n", consecutiveMotion, motionDuration);

                if (checkWiFiConnection()) {
                    totalMotionCount++;
                    ProcessingTask_t task = mlOnMotion ? TASK_CLASSIFY : TASK_MOTION_ALERT;
                    if (xQueueSend(xProcessingQueue, &task, 0) == pdPASS) {
                        Serial.println("Task sent to processing queue.");
                        lastMotionAlert = millis();
                    } else {
                        Serial.println("Processing queue full, motion event dropped.");
                    }
                } else {
                    Serial.println("Motion detected but WiFi unavailable.");
                }
                // Reset after a valid detection
                consecutiveMotion = 0;
                motionInProgress = false;
            }
            vTaskDelay(POLLING_DELAY_HIGH_MS / portTICK_PERIOD_MS);

        } else {
            if (motionInProgress) {
                Serial.printf("Motion ended (count=%d, incomplete)\n", consecutiveMotion);
            }
            consecutiveMotion = 0;
            motionInProgress = false;
            vTaskDelay(POLLING_DELAY_LOW_MS / portTICK_PERIOD_MS);
        }
    }
}

// ======================================================
//             System Health Monitor Task
// ======================================================
void healthMonitorTask(void *pvParameters) {
    const unsigned long LOG_INTERVAL = 300000; // 5 minutes
    unsigned long lastLog = 0;
    for (;;) {
        esp_task_wdt_reset();
        if (millis() - lastLog >= LOG_INTERVAL) {
            Serial.println("\n=== HEALTH CHECK ===");
            Serial.printf("Uptime: %lu s\n", millis() / 1000);
            Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
            Serial.printf("Free PSRAM: %u bytes\n", ESP.getFreePsram());
            Serial.printf("WiFi Status: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
            Serial.printf("Total Motions: %u\n", totalMotionCount);
            Serial.printf("ML Processing: %s\n", mlProcessing ? "Active" : "Idle");
            Serial.println("===================\n");
            lastLog = millis();
        }
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}


// ======================================================
//               Setup Functions
// ======================================================
void connectWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi connection failed! Restarting...");
        ESP.restart();
    }
}

void setupCamera() {
    Serial.println("Initializing camera...");
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
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA; // 320x240
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        ESP.restart();
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_special_effect(s, 0);
        s->set_whitebal(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
    }
    Serial.println("Camera initialized successfully");
}

void updateFlashLED() {
    // Priority: alertMode > flashState
    digitalWrite(FLASH_LED_PIN, (alertMode || flashState) ? HIGH : LOW);
}

// ======================================================
//                         SETUP
// ======================================================
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
    Serial.begin(115200);
    delay(2000);

    esp_task_wdt_deinit();
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_MS,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = true};
    esp_task_wdt_init(&wdt_config);

    pinMode(PIR_PIN, INPUT_PULLUP);
    pinMode(FLASH_LED_PIN, OUTPUT);
    dht.begin();
    Serial.println("DHT22 initialized");

    xProcessingQueue = xQueueCreate(2, sizeof(ProcessingTask_t));
    xResponseQueue = xQueueCreate(5, sizeof(TelegramMessage));
    xWiFiMutex = xSemaphoreCreateMutex();
    xBufferMutex = xSemaphoreCreateMutex();
    xTelegramMutex = xSemaphoreCreateMutex();

    if (!xProcessingQueue || !xResponseQueue || !xWiFiMutex || !xBufferMutex || !xTelegramMutex) {
        Serial.println("Failed to create RTOS resources! Restarting...");
        ESP.restart();
    }

    connectWiFi();
    setupCamera();
    systemReady = true;

    // Create tasks
    xTaskCreatePinnedToCore(telegramTask, "Telegram", 10240, NULL, 3, &telegramTaskHandle, 0);
    xTaskCreatePinnedToCore(motionTask, "Motion", 2048, NULL, 1, &motionTaskHandle, 1);
    xTaskCreatePinnedToCore(processingManagerTask, "Processing", 16384, NULL, 2, &processingTaskHandle, 1);
    xTaskCreatePinnedToCore(healthMonitorTask, "Health", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(dhtMonitorTask, "DHT", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(googleSheetsTask, "GoogleSheets", 4096, NULL, 1, NULL, 0);

    Serial.println("All tasks created successfully.");
}

// ======================================================
//             Telegram Command Handler
// ======================================================
String normalizeCommand(String command) {
    int atIndex = command.indexOf('@');
    return (atIndex != -1) ? command.substring(0, atIndex) : command;
}

void handleNewMessages(int numNewMessages, UniversalTelegramBot& bot) {
    for (int i = 0; i < numNewMessages; i++) {
        String chat_id_str = bot.messages[i].chat_id;
        String user_id = bot.messages[i].from_id;
        String text = normalizeCommand(bot.messages[i].text);
        text.trim();

        // Only process commands from the designated group chat
        if (chat_id_str != String(GROUP_CHAT_ID)) {
             Serial.printf("Ignored message from unauthorized chat_id: %s\n", chat_id_str.c_str());
             continue;
        }

        Serial.printf("Received command from user %s: '%s'\n", user_id.c_str(), text.c_str());

        if (mlProcessing && (text == "/photo" || text == "/classify")) {
            sendTelegramResponse(chat_id_str, "⚠️ System is busy. Please wait a moment...");
            continue;
        }

        if (text == "/start") {
            String response = "🤖 *ESP32-CAM AI Security Bot*\n\n"
                              "/photo - Take a photo\n"
                              "/classify - Take a photo & classify\n"
                              "/status - Show system status\n"
                              "/report - Show activity report\n"
                              "/temp - Check temperature & humidity\n"
                              "/flash - Toggle the flash LED\n"
                              "/piron - Enable motion detection\n"
                              "/piroff - Disable motion detection\n"
                              "/ml_on - Enable AI on motion\n"
                              "/ml_off - Disable AI on motion\n"
                              "/reboot - Restart the device";
            sendTelegramResponse(chat_id_str, response);
        } else if (text == "/photo") {
            sendTelegramResponse(chat_id_str, "📸 Taking photo...");
            ProcessingTask_t task = TASK_PHOTO_ONLY;
            if (xQueueSend(xProcessingQueue, &task, 0) != pdPASS) {
                sendTelegramResponse(chat_id_str, "❌ Processing queue is full. Try again.");
            }
        } else if (text == "/classify") {
            sendTelegramResponse(chat_id_str, "🔍 Starting classification...");
            ProcessingTask_t task = TASK_CLASSIFY;
            if (xQueueSend(xProcessingQueue, &task, 0) != pdPASS) {
                sendTelegramResponse(chat_id_str, "❌ Processing queue is full. Try again.");
            }
        } else if (text == "/flash") {
            flashState = !flashState;
            updateFlashLED();
            String response = flashState ? "💡 Flash ON" : "💡 Flash OFF";
            if (alertMode && !flashState) {
                response += "\n⚠️ LED stays ON due to high temp alert.";
            }
            sendTelegramResponse(chat_id_str, response);
        } else if (text == "/piron") {
            pirEnabled = true;
            sendTelegramResponse(chat_id_str, "🚨 Motion detection ENABLED");
        } else if (text == "/piroff") {
            pirEnabled = false;
            sendTelegramResponse(chat_id_str, "🚨 Motion detection DISABLED");
        } else if (text == "/ml_on") {
            mlOnMotion = true;
            sendTelegramResponse(chat_id_str, "🧠 AI classification on motion is now ACTIVE.");
        } else if (text == "/ml_off") {
            mlOnMotion = false;
            sendTelegramResponse(chat_id_str, "📸 AI classification on motion is now INACTIVE. Will only send photos.");
        } else if (text == "/temp") {
            sendTelegramResponse(chat_id_str, readDHTSensor());
        } else if (text == "/status") {
            String response = "📊 *System Status:*\n";
            response += "WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + "\n";
            response += "PIR: " + String(pirEnabled ? "Active" : "Inactive") + "\n";
            response += "Flash: " + String(flashState ? "ON" : "OFF") + "\n";
            response += "AI on Motion: " + String(mlOnMotion ? "Active" : "Inactive") + "\n";
            response += "Alert Mode: " + String(alertMode ? "ACTIVE (≥35°C)" : "Inactive") + "\n";
            response += "Free PSRAM: " + String(ESP.getFreePsram()) + " bytes\n";
            response += "Uptime: " + String(millis() / 1000) + "s\n";
            response += "🌡️ Temp: " + String(lastTemperature, 1) + "°C\n";
            response += "💧 Humidity: " + String(lastHumidity, 1) + "%\n";
            response += "🔎 Last Detection: " + lastDetectionLabel + " (" + String(lastDetectionConfidence * 100, 1) + "%)";
            sendTelegramResponse(chat_id_str, response);
        } else if (text == "/report") {
            String report = "📄 *Activity Report*\n";
            report += "- Total Motion Events: " + String(totalMotionCount) + "\n";
            report += "- Classified Objects:\n";
            bool hasClassifications = false;
            for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                if (classificationCounts[i] > 0) {
                    report += "  - " + String(ei_classifier_inferencing_categories[i]) + ": " + String(classificationCounts[i]) + "\n";
                    hasClassifications = true;
                }
            }
            if (!hasClassifications) {
                report += "  (No classifications yet)\n";
            }
            sendTelegramResponse(chat_id_str, report);
        } else if (text == "/reboot") {
            sendTelegramResponse(chat_id_str, "🔄 Rebooting device...");
            vTaskDelay(500 / portTICK_PERIOD_MS);
            ESP.restart();
        } else {
            sendTelegramResponse(chat_id_str, "❓ Unknown command. Send /start for help.");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// ======================================================
//             Image Capture & Classification
// ======================================================
String captureAndClassify() {
    esp_task_wdt_reset();
    String sendStatus = sendPhotoToTelegram();
    esp_task_wdt_reset();
    if (sendStatus != "Photo sent") {
        return sendStatus; // Return error from sending photo
    }
    
    // Now, run classification on a NEW frame to ensure it's up-to-date.
    Serial.println("🔍 Starting image classification (320x240 grayscale)...");
    
    if (xSemaphoreTake(xBufferMutex, 10000 / portTICK_PERIOD_MS) != pdTRUE) {
        return "Failed: Could not lock buffer mutex";
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        xSemaphoreGive(xBufferMutex);
        return "Failed: Could not get frame from camera";
    }

    if (fb->format != PIXFORMAT_JPEG) {
        esp_camera_fb_return(fb);
        xSemaphoreGive(xBufferMutex);
        return "Failed: Frame format is not JPEG";
    }

    // Allocate buffer for RGB conversion
    size_t rgb_size = fb->width * fb->height * 3;
    uint8_t *rgb_buffer = (uint8_t*)heap_caps_malloc(rgb_size, MALLOC_CAP_SPIRAM);
    if (!rgb_buffer) {
        esp_camera_fb_return(fb);
        xSemaphoreGive(xBufferMutex);
        return "Failed: Could not allocate RGB buffer";
    }
    
    // Convert JPEG to RGB888
    bool conv_ok = fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buffer);
    esp_camera_fb_return(fb); // Return frame buffer as soon as possible
    if (!conv_ok) {
        heap_caps_free(rgb_buffer);
        xSemaphoreGive(xBufferMutex);
        return "Failed: JPEG to RGB conversion failed";
    }

    size_t pixel_count = fb->width * fb->height;
    float *inference_buffer = (float*)heap_caps_malloc(pixel_count * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!inference_buffer) {
        heap_caps_free(rgb_buffer);
        xSemaphoreGive(xBufferMutex);
        return "Failed: Could not allocate inference buffer";
    }

    // Convert RGB to grayscale float for the model
    for (size_t i = 0; i < pixel_count; i++) {
        size_t idx = i * 3;
        float gray = (rgb_buffer[idx] * 0.299f) + (rgb_buffer[idx + 1] * 0.587f) + (rgb_buffer[idx + 2] * 0.114f);
        inference_buffer[i] = gray;
        if (i % 2000 == 0) esp_task_wdt_reset();
    }
    heap_caps_free(rgb_buffer);

    // Prepare signal for Edge Impulse
    signal_t signal;
    signal.total_length = pixel_count;
    signal.get_data = [inference_buffer, pixel_count](size_t offset, size_t length, float *out_ptr) -> int {
        if (offset + length > pixel_count) return EIDSP_OUT_OF_MEM;
        memcpy(out_ptr, inference_buffer + offset, length * sizeof(float));
        return EIDSP_OK;
    };

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
    heap_caps_free(inference_buffer);
    xSemaphoreGive(xBufferMutex);

    if (res != EI_IMPULSE_OK) {
        return "Failed: Classification error (code " + String(res) + ")";
    }

    float max_score = 0.0f;
    String detected_label = "";
    size_t detected_idx = 0;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > max_score) {
            max_score = result.classification[i].value;
            detected_label = result.classification[i].label;
            detected_idx = i;
        }
    }
    
    String output = (max_score > 0.5f)
                  ? detected_label + " (" + String(max_score * 100, 1) + "%)"
                  : "Nothing detected";

    Serial.println("✅ Classification Result: " + output);

    if (max_score > 0.5f) {
        lastDetectionLabel = detected_label;
        lastDetectionConfidence = max_score;
        lastDetectionTimestamp = millis();
        classificationCounts[detected_idx]++;
    }

    return output;
}


// ======================================================
//             Send Photo to Telegram
// ======================================================
String sendPhotoToTelegram() {
    esp_task_wdt_reset();
    if (!checkWiFiConnection()) return "Failed: No WiFi connection";
    if (ESP.getFreePsram() < 200000) return "Failed: Low PSRAM";
    
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb || fb->format != PIXFORMAT_JPEG || fb->len == 0) {
        if (fb) esp_camera_fb_return(fb);
        return "Failed: Could not capture frame";
    }

    WiFiClientSecure *client = new WiFiClientSecure;
    if (!client) {
        esp_camera_fb_return(fb);
        return "Failed: Could not allocate client";
    }
    client->setInsecure();

    int retries = 0;
    while (!client->connect("api.telegram.org", 443) && retries < 3) {
        esp_task_wdt_reset();
        delay(1000);
        retries++;
    }
    if (!client->connected()) {
        delete client;
        esp_camera_fb_return(fb);
        return "Failed: Connection to Telegram timed out";
    }

    String boundary = "----ESP32CAMBoundary";
    String body1 = "--" + boundary + "\r\n"
                   "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n" +
                   String(GROUP_CHAT_ID) + "\r\n"
                   "--" + boundary + "\r\n"
                   "Content-Disposition: form-data; name=\"photo\"; filename=\"cam.jpg\"\r\n"
                   "Content-Type: image/jpeg\r\n\r\n";
    String body2 = "\r\n--" + boundary + "--\r\n";
    
    size_t contentLength = body1.length() + fb->len + body2.length();
    String header = "POST /bot" + String(BOT_TOKEN) + "/sendPhoto HTTP/1.1\r\n"
                    "Host: api.telegram.org\r\n"
                    "Content-Type: multipart/form-data; boundary=" + boundary + "\r\n"
                    "Content-Length: " + String(contentLength) + "\r\n"
                    "Connection: close\r\n\r\n";

    client->print(header);
    client->print(body1);

    size_t sent = 0;
    size_t chunkSize = 4096;
    while (sent < fb->len) {
        size_t toSend = min(chunkSize, fb->len - sent);
        client->write(fb->buf + sent, toSend);
        sent += toSend;
        esp_task_wdt_reset();
        vTaskDelay(1);
    }
    client->print(body2);
    esp_camera_fb_return(fb);

    String response = "";
    unsigned long timeout = millis();
    while (client->connected() && millis() - timeout < 15000) {
        while (client->available()) {
            response += (char)client->read();
        }
        esp_task_wdt_reset();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    delete client;

    return (response.indexOf("\"ok\":true") != -1) ? "Photo sent" : "Failed: Invalid Telegram response";
}

// ======================================================
//                         LOOP
// ======================================================
void loop() {
    // This task is deleted in setup(); the main loop is handled by FreeRTOS tasks.
    vTaskDelete(NULL);
}
