// ESP32 Cam Motion Alert - Non-Blocking Architecture 
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
// --- KONFIGURASI PENGGUNA ---
#include "config.h"
#include "Jdzaky-project-1_inferencing.h"

// --- Definisi Pin & Konstanta ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define PIR_PIN 13
#define FLASH_LED_PIN 4
#define WDT_TIMEOUT_S 120

// --- Variabel Global ---
volatile bool pirEnabled = true;
volatile bool mlProcessing = false;
bool flashState = false;
bool systemReady = false;
// --- Statistik dan Status Deteksi ---
String lastDetectionLabel = "Tidak ada";
float lastDetectionConfidence = 0.0f;
unsigned long lastDetectionTimestamp = 0; // millis() saat deteksi

// Statistik klasifikasi
uint32_t totalMotionCount = 0;
uint32_t classificationCounts[EI_CLASSIFIER_LABEL_COUNT] = {0}; 

// Mode AI saat gerakan
bool mlOnMotion = true; // default: aktif

// --- Message Queue Structure ---
#define MAX_MSG_LENGTH 512
#define MAX_CHAT_ID_LENGTH 16
struct TelegramMessage {
  char chatId[MAX_CHAT_ID_LENGTH];
  char text[MAX_MSG_LENGTH];
  bool isPhoto;
};

// --- Task Commands ---
typedef enum {
  TASK_PHOTO_ONLY,
  TASK_CLASSIFY,
  TASK_MOTION_ALERT
} ProcessingTask_t;

// --- FreeRTOS Handles ---
TaskHandle_t telegramTaskHandle = NULL;
TaskHandle_t motionTaskHandle = NULL;
TaskHandle_t processingTaskHandle = NULL;
QueueHandle_t xProcessingQueue;
QueueHandle_t xResponseQueue;
SemaphoreHandle_t xWiFiMutex;
SemaphoreHandle_t xBufferMutex;
WiFiClientSecure telegramClient;
SemaphoreHandle_t xTelegramMutex;

String bot_token = BOT_TOKEN;
String chat_id = CHAT_ID;

// --- Fungsi Validasi Keanggotaan Grup ---
bool isGroupMember(String userId) {
  if (!checkWiFiConnection()) return false;
  if (userId == "0" || userId == "") return false;

  // Gunakan client global yang sudah ada
  if (xSemaphoreTake(xTelegramMutex, 5000 / portTICK_PERIOD_MS) != pdTRUE) {
    Serial.println("isGroupMember: Gagal ambil Telegram mutex");
    return false;
  }

  String url = "https://api.telegram.org/bot" + String(BOT_TOKEN) +
               "/getChatMember?chat_id=" + String(GROUP_CHAT_ID) +
               "&user_id=" + userId;

  HTTPClient http;
  telegramClient.setInsecure(); // pastikan di-set setiap kali
  http.begin(telegramClient, url); // <-- gunakan client yang aman

  int httpCode = http.GET();
  bool result = false;
  if (httpCode == 200) {
    String response = http.getString();
    Serial.println("✅ getChatMember response: " + response);
    if (response.indexOf("\"status\":\"left\"") == -1 &&
        response.indexOf("\"status\":\"kicked\"") == -1 &&
        response.indexOf("\"ok\":true") != -1) {
      result = true;
    }
  } else {
    String err = http.getString();
    Serial.println("❌ getChatMember HTTP Error " + String(httpCode) + ": " + err);
  }

  http.end();
  xSemaphoreGive(xTelegramMutex);
  return result;
}

// ======================================================
//          IMPROVED WIFI CONNECTION CHECK
// ======================================================
bool checkWiFiConnection() {
  if (xWiFiMutex == NULL) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected. Attempting reconnect...");
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

      int retries = 0;
      while (WiFi.status() != WL_CONNECTED && retries < 10) { // <-- KURANGI dari 20 ke 10
        delay(500);
        Serial.print(".");
        retries++;
        esp_task_wdt_reset(); // <-- TAMBAH INI
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi reconnected!");
        return true;
      } else {
        Serial.println("\nFailed to reconnect WiFi");
        return false;
      }
    }
    return true;
  }

  if (xSemaphoreTake(xWiFiMutex, 5000 / portTICK_PERIOD_MS) != pdTRUE) { 
    Serial.println("checkWiFiConnection: Gagal mengunci WiFi mutex (timeout).");
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting reconnect...");
    WiFi.disconnect();
    delay(1000);
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
//         NON-BLOCKING TELEGRAM RESPONSE SENDER
// ======================================================
void sendTelegramResponse(String chatId, String message) {
  TelegramMessage response;
  memset(&response, 0, sizeof(response));
  strncpy(response.chatId, chatId.c_str(), MAX_CHAT_ID_LENGTH - 1);
  response.chatId[MAX_CHAT_ID_LENGTH - 1] = '\0';
  strncpy(response.text, message.c_str(), MAX_MSG_LENGTH - 1);
  response.text[MAX_MSG_LENGTH - 1] = '\0';
  response.isPhoto = false;
  if (xResponseQueue == NULL) {
    Serial.println("sendTelegramResponse: xResponseQueue is NULL!");
    return;
  }
  if (xQueueSend(xResponseQueue, &response, 0) != pdPASS) {
    Serial.println("Response queue full, dropping message");
  }
}

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
    
    // Sukses jika tidak mengandung kata "Gagal" atau "Error"
    if (result.indexOf("Gagal") == -1 && result.indexOf("Error") == -1 && result.length() > 0) {
      return result;
    }
    
    attempt++;
    if (attempt < maxRetries) {
      Serial.printf("Retry attempt %d/%d\n", attempt + 1, maxRetries);
      vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait before retry
    }
  }
  
  return result; // Return last attempt result
}

// ======================================================
//         PROCESSING MANAGER - OPTIMIZED
// ======================================================
void processingManagerTask(void *pvParameters) {
  Serial.println("Processing Manager Task started.");
  ProcessingTask_t task_to_do;

  for (;;) {
    if (xProcessingQueue == NULL) {
      Serial.println("processingManagerTask: xProcessingQueue is NULL!");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    if (xQueueReceive(xProcessingQueue, &task_to_do, portMAX_DELAY) == pdPASS) {
      mlProcessing = true;
      Serial.println("Processing task received!");
      esp_task_wdt_reset();

      // Gunakan retry mechanism
      String result = executeTaskWithRetry(task_to_do, 2);
      String message;

      // Format message
      switch (task_to_do) {
        case TASK_PHOTO_ONLY:
          message = (result == "Foto terkirim") ?
                      "✅ Foto berhasil dikirim." :
                      "⚠️ Gagal mengirim foto: " + result;
          break;

        case TASK_CLASSIFY:
          if (result.length() == 0) {
            message = "⚠️ Hasil klasifikasi kosong (error tak terduga)";
          } else if (result.startsWith("Gagal") || result.startsWith("Error") || result.startsWith("Memori")) {
            message = "⚠️ " + result;
          } else {
            message = "✅ Klasifikasi: " + result;
          }
          break;

        case TASK_MOTION_ALERT:
          message = (result == "Foto terkirim") ?
                      "✅ Foto berhasil dikirim." :
                      "⚠️ Gagal mengirim foto: " + result;
          break;
      }

      if (xResponseQueue == NULL) {
        Serial.println("processingManagerTask: xResponseQueue is NULL!");
      } else {
        sendTelegramResponse(String(GROUP_CHAT_ID), message);
      }

      mlProcessing = false;
      heap_caps_print_heap_info(MALLOC_CAP_8BIT);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

// ======================================================
//         TELEGRAM TASK - FIXED HTTP CONNECTION
// ======================================================
void telegramTask(void *pvParameters) {
  Serial.println("Telegram Task started.");
  unsigned long lastUpdate = 0;
  const unsigned long UPDATE_INTERVAL = 3000;
  WiFiClientSecure clientTCP;
  clientTCP.setInsecure();
  UniversalTelegramBot bot(bot_token, clientTCP);
  bot.getUpdates(bot.last_message_received + 1);

  while (!systemReady) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // ✅ Kirim welcome hanya ke grup — hapus pesan ke chat pribadi
  bool welcomeSent = false;
  int welcomeRetries = 0;
  while (!welcomeSent && welcomeRetries < 3) {
    if (checkWiFiConnection()) {
      String welcome = "✅ Sistem Online.\nKirim /start untuk melihat daftar perintah.";
      bool groupOK = bot.sendMessage(String(GROUP_CHAT_ID), "📢 Grup 'Smart Security Monitoring' aktif!\n" + welcome, "");
      if (groupOK) {
        Serial.println("Welcome message sent successfully to group");
        welcomeSent = true;
      } else {
        Serial.println("Failed to send welcome message to group, retrying...");
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

    if (xResponseQueue == NULL) {
      Serial.println("telegramTask: xResponseQueue is NULL!");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    // Kirim respons tertunda
    while (xQueueReceive(xResponseQueue, &responseMsg, 0) == pdPASS && processedResponses < 3) {
      if (checkWiFiConnection()) {
        bool sent = bot.sendMessage(responseMsg.chatId, responseMsg.text, "");
        if (!sent) {
          Serial.print("Failed to send response: ");
          Serial.println(responseMsg.text);
        }
        processedResponses++;
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
    }

    // Ambil pesan baru
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
//         MOTION DETECTION TASK - STABILIZED
// ======================================================
void motionTask(void *pvParameters) {
  Serial.println("Motion Detection Task started.");
  
  // Konfigurasi deteksi
  const int MOTION_THRESHOLD = 6;           
  const unsigned long motionCooldown = 20000; 
  const unsigned long MIN_MOTION_DURATION = 2000; 
  const unsigned long POLLING_DELAY_HIGH = 250;  
  const unsigned long POLLING_DELAY_LOW = 250;    
  
  // State variables
  unsigned long lastMotionAlert = 0;
  unsigned long motionStartTime = 0;
  int consecutiveMotion = 0;
  bool motionInProgress = false;

  for (;;) {
    esp_task_wdt_reset();

    if (pirEnabled && digitalRead(PIR_PIN) == HIGH) {
      // Mulai hitung durasi gerakan
      if (consecutiveMotion == 0) {
        motionStartTime = millis();
        motionInProgress = true;
        Serial.println("Motion started, validating...");
      }
      
      consecutiveMotion++;
      unsigned long motionDuration = millis() - motionStartTime;

      // Validasi: threshold + durasi minimum + cooldown
      if (consecutiveMotion >= MOTION_THRESHOLD && 
          motionDuration >= MIN_MOTION_DURATION &&
          !mlProcessing &&
          (millis() - lastMotionAlert > motionCooldown)) {

        Serial.printf("✅ Validated Motion! (count=%d, duration=%lums)\n", 
                      consecutiveMotion, motionDuration);

        if (checkWiFiConnection()) {
          totalMotionCount++;
          
          if (xProcessingQueue == NULL) {
            Serial.println("motionTask: xProcessingQueue is NULL!");
            consecutiveMotion = 0;
            motionInProgress = false;
            continue;
          }

          ProcessingTask_t task = mlOnMotion ? TASK_CLASSIFY : TASK_MOTION_ALERT;
          
          if (xQueueSend(xProcessingQueue, &task, 0) == pdPASS) {
            Serial.println("Task sent to processing queue.");
            lastMotionAlert = millis();
            consecutiveMotion = 0;
            motionInProgress = false;
          } else {
            Serial.println("Queue full, task cancelled.");
          }
        } else {
          Serial.println("Motion detected but WiFi unavailable or ML busy");
          consecutiveMotion = 0;
          motionInProgress = false;
        }
      }

      vTaskDelay(POLLING_DELAY_HIGH / portTICK_PERIOD_MS);
      
    } else {
      // Reset saat tidak ada gerakan
      if (motionInProgress) {
        Serial.printf("Motion ended (count=%d, incomplete)\n", consecutiveMotion);
      }
      consecutiveMotion = 0;
      motionInProgress = false;
      vTaskDelay(POLLING_DELAY_LOW / portTICK_PERIOD_MS);
    }
  }
}

// ======================================================
//              WIFI & CAMERA SETUP FUNCTIONS
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

void healthMonitorTask(void *pvParameters) {
  const unsigned long LOG_INTERVAL = 300000;
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
    
    vTaskDelay(60000 / portTICK_PERIOD_MS); // Check every minute
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
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    ESP.restart();
  }

  // Adjust camera settings
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0-No Effect, 1-Negative, 2-Grayscale, 3-Red Tint, 4-Green Tint, 5-Blue Tint, 6-Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    s->set_framesize(s, FRAMESIZE_QQVGA); 
  }

  Serial.println("Camera initialized successfully");
}

// ======================================================
//                           SETUP
// ======================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(2000);

  // WDT lebih panjang
  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 180000, // 180 detik (naik dari 120)
    .idle_core_mask = (1 << 0) | (1 << 1),
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  // esp_task_wdt_add(NULL);

  pinMode(PIR_PIN, INPUT_PULLUP);
  pinMode(FLASH_LED_PIN, OUTPUT);

  xProcessingQueue = xQueueCreate(2, sizeof(ProcessingTask_t));
  xResponseQueue = xQueueCreate(5, sizeof(TelegramMessage));
  xWiFiMutex = xSemaphoreCreateMutex();
  xBufferMutex = xSemaphoreCreateMutex();
  xTelegramMutex = xSemaphoreCreateMutex(); 

  if (!xProcessingQueue || !xResponseQueue || !xWiFiMutex || !xBufferMutex) {
    Serial.println("Failed to create resources!");
    ESP.restart();
  }

  connectWiFi();
  setupCamera();
  systemReady = true;

  // KURANGI STACK SIZE untuk mencegah out of memory
  xTaskCreatePinnedToCore(telegramTask, "Telegram", 10240, NULL, 3, &telegramTaskHandle, 0);
  xTaskCreatePinnedToCore(motionTask, "Motion", 2048, NULL, 1, &motionTaskHandle, 1);
  xTaskCreatePinnedToCore(processingManagerTask, "Processing", 16384, NULL, 2, &processingTaskHandle, 1);

  esp_task_wdt_add(telegramTaskHandle);
  esp_task_wdt_add(motionTaskHandle);
  esp_task_wdt_add(processingTaskHandle);

  TaskHandle_t healthTaskHandle = NULL;
  xTaskCreatePinnedToCore(healthMonitorTask, "Health", 2048, NULL, 1, &healthTaskHandle, 0);
  esp_task_wdt_add(healthTaskHandle);
}

// Tambahkan fungsi helper
String normalizeCommand(String command) {
  int atIndex = command.indexOf('@');
  if (atIndex != -1) {
    return command.substring(0, atIndex);
  }
  return command;
}
// ======================================================
//              COMMAND HANDLERS
// ======================================================
void handleNewMessages(int numNewMessages, UniversalTelegramBot& bot) {
  for (int i = 0; i < numNewMessages; i++) {
    // ✅ Ambil chat_id sebagai String dulu, lalu konversi ke long
    String chat_id_str = bot.messages[i].chat_id;
    long chat_id_long = chat_id_str.toInt(); // Grup: negatif, Private: positif
    String user_id = String(bot.messages[i].from_id);

    // Abaikan pesan dari chat pribadi (hanya proses grup)
    if (chat_id_long > 0) {
      Serial.println("Ignored private message (only group allowed).");
      continue;
    }

    // Abaikan pesan sistem/bot
    if (user_id == "0" || user_id == "") {
      continue;
    }

    String text = bot.messages[i].text;
    text.trim();
    text = normalizeCommand(text); // Normalisasi: /start@bot → /start

    Serial.printf("Received command from user %s in chat %s: '%s'\n",
                  user_id.c_str(), chat_id_str.c_str(), text.c_str());


    // Cegah command saat ML sedang berjalan (kecuali non-blocking)
    if (mlProcessing && (text == "/photo" || text == "/classify")) {
      sendTelegramResponse(chat_id_str, "⚠️ Sistem sedang memproses. Tunggu sebentar...");
      continue;
    }

    if (text == "/start") {
      String response = "🤖 AEGIS ESP32-CAM Motion AI\n"
                        "/photo - Ambil foto (tanpa klasifikasi)\n"
                        "/classify - Ambil foto & klasifikasi objek\n"
                        "/flash - Nyalakan/matikan flash\n"
                        "/piron - Aktifkan deteksi gerak\n"
                        "/piroff - Nonaktifkan deteksi gerak\n"
                        "/ml_on - Aktifkan AI saat gerakan\n"
                        "/ml_off - Nonaktifkan AI (kirim foto saja)\n"
                        "/status - Tampilkan status sistem\n"
                        "/report - Tampilkan laporan aktivitas\n"
                        "/reboot - Mulai ulang perangkat";
      sendTelegramResponse(chat_id_str, response);
    }
    else if (text == "/photo") {
      sendTelegramResponse(chat_id_str, "📸 Mengambil foto...");
      if (xProcessingQueue == NULL) {
        sendTelegramResponse(chat_id_str, "❌ Queue tidak tersedia");
        continue;
      }
      ProcessingTask_t task = TASK_PHOTO_ONLY;
      if (xQueueSend(xProcessingQueue, &task, 0) != pdPASS) {
        sendTelegramResponse(chat_id_str, "❌ Queue penuh, coba lagi");
      }
    }
    else if (text == "/classify") {
      sendTelegramResponse(chat_id_str, "🔍 Memulai klasifikasi...");
      if (xProcessingQueue == NULL) {
        sendTelegramResponse(chat_id_str, "❌ Queue tidak tersedia");
        continue;
      }
      ProcessingTask_t task = TASK_CLASSIFY;
      if (xQueueSend(xProcessingQueue, &task, 0) != pdPASS) {
        sendTelegramResponse(chat_id_str, "❌ Queue penuh, coba lagi");
      }
    }
    else if (text == "/flash") {
      flashState = !flashState;
      digitalWrite(FLASH_LED_PIN, flashState);
      String response = flashState ? "💡 Flash ON" : "💡 Flash OFF";
      sendTelegramResponse(chat_id_str, response);
    }
    else if (text == "/piron") {
      pirEnabled = true;
      sendTelegramResponse(chat_id_str, "🚨 Deteksi gerak PIR DIAKTIFKAN");
    }
    else if (text == "/piroff") {
      pirEnabled = false;
      sendTelegramResponse(chat_id_str, "🚨 Deteksi gerak PIR DINONAKTIFKAN");
    }
    else if (text == "/ml_on") {
      mlOnMotion = true;
      sendTelegramResponse(chat_id_str, "🧠 AI klasifikasi AKTIF saat gerakan terdeteksi.");
    }
    else if (text == "/ml_off") {
      mlOnMotion = false;
      sendTelegramResponse(chat_id_str, "📸 AI klasifikasi NONAKTIF. Hanya kirim foto saat gerakan.");
    }
    else if (text == "/status") {
      String response = "📊 Status Sistem:\n";
      response += "WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Terhubung" : "Terputus") + "\n";
      response += "PIR: " + String(pirEnabled ? "Aktif" : "Nonaktif") + "\n";
      response += "Flash: " + String(flashState ? "ON" : "OFF") + "\n";
      response += "ML Processing: " + String(mlProcessing ? "Berjalan" : "Idle") + "\n";
      response += "AI on Motion: " + String(mlOnMotion ? "Aktif" : "Nonaktif") + "\n";
      response += "Free PSRAM: " + String(ESP.getFreePsram()) + " bytes\n";
      response += "Uptime: " + String(millis()/1000) + "s\n";
      if (lastDetectionTimestamp > 0) {
        response += "🔎 Deteksi terakhir: " + lastDetectionLabel + " (" + String(lastDetectionConfidence * 100, 1) + "%)\n";
      } else {
        response += "🔎 Deteksi terakhir: Tidak ada\n";
      }
      sendTelegramResponse(chat_id_str, response);
    }
    else if (text == "/report") {
      String report = "📄 Laporan Aktivitas\n";
      report += "- Total Gerakan Terdeteksi: " + String(totalMotionCount) + "\n";
      report += "- Objek Terklasifikasi:\n";
      bool hasClassifications = false;
      for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (classificationCounts[i] > 0) {
          report += "  - " + String(ei_classifier_inferencing_categories[i]) + ": " + String(classificationCounts[i]) + "\n";
          hasClassifications = true;
        }
        vTaskDelay(1);
      }
      if (!hasClassifications) {
        report += "  (Belum ada klasifikasi)\n";
      }
      sendTelegramResponse(chat_id_str, report);
    }
    else if (text == "/reboot") {
      sendTelegramResponse(chat_id_str, "🔄 Memulai ulang perangkat...");
      vTaskDelay(500 / portTICK_PERIOD_MS);
      ESP.restart();
    }
    else {
      Serial.printf("Unknown command: '%s'\n", text.c_str());
      sendTelegramResponse(chat_id_str, "❓ Perintah tidak dikenal. Kirim /start untuk bantuan.");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
} 

// ======================================================
//                 CAPTURE AND CLASSIFY
// ======================================================
String captureAndClassify() {
  esp_task_wdt_reset();
  String sendStatus = sendPhotoToTelegram();
  esp_task_wdt_reset();
  if (sendStatus == "Foto terkirim") {
    return classifyImage();
  }
  return sendStatus;
}

bool resize_rgb888(uint8_t* input, int in_w, int in_h, uint8_t* output, int out_w, int out_h) {
  if (!input || !output || in_w <= 0 || in_h <= 0 || out_w <= 0 || out_h <= 0) return false;

  for (int y = 0; y < out_h; y++) {
    for (int x = 0; x < out_w; x++) {
      int src_x = (x * in_w) / out_w;
      int src_y = (y * in_h) / out_h;
      int src_idx = (src_y * in_w + src_x) * 3;
      int dst_idx = (y * out_w + x) * 3;

      output[dst_idx]     = input[src_idx];     // R
      output[dst_idx + 1] = input[src_idx + 1]; // G
      output[dst_idx + 2] = input[src_idx + 2]; // B
    }
  }
  return true;
}

// ======================================================
//          CLASSIFY IMAGE - COMPLETELY FIXED
// ======================================================
String classifyImage() {
  Serial.println("🔍 classifyImage: Memulai proses klasifikasi...");
  esp_task_wdt_reset(); // <-- TAMBAH INI di awal

  if (xSemaphoreTake(xBufferMutex, 10000 / portTICK_PERIOD_MS) != pdTRUE) {
    Serial.println("❌ classifyImage: Gagal mengambil xBufferMutex (timeout)");
    return "Gagal mengunci buffer";
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ classifyImage: Gagal mendapatkan frame dari kamera");
    xSemaphoreGive(xBufferMutex);
    return "Gagal capture";
  }

  if (fb->format != PIXFORMAT_JPEG) {
    Serial.println("❌ classifyImage: Format frame bukan JPEG");
    esp_camera_fb_return(fb);
    xSemaphoreGive(xBufferMutex);
    return "Format frame tidak didukung";
  }

  int in_w = 160, in_h = 120;
  size_t rgb_full_size = in_w * in_h * 3;
  
  // Validasi memori sebelum alokasi
  if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) < rgb_full_size + 100000) {
    esp_camera_fb_return(fb);
    xSemaphoreGive(xBufferMutex);
    return "Memori PSRAM tidak cukup";
  }
  
  uint8_t* rgb_full = (uint8_t*) heap_caps_malloc(rgb_full_size, MALLOC_CAP_SPIRAM);
  if (!rgb_full) {
    esp_camera_fb_return(fb);
    xSemaphoreGive(xBufferMutex);
    return "Gagal alokasi RGB buffer";
  }

  esp_task_wdt_reset(); // <-- TAMBAH INI sebelum konversi
  bool conv_result = fmt2rgb888(fb->buf, fb->len, fb->format, rgb_full);
  esp_camera_fb_return(fb);
  
  if (!conv_result) {
    heap_caps_free(rgb_full);
    xSemaphoreGive(xBufferMutex);
    return "Gagal konversi ke RGB";
  }

  const int out_w = 96, out_h = 96;
  size_t rgb_resized_size = out_w * out_h * 3;
  uint8_t* rgb_resized = (uint8_t*) heap_caps_malloc(rgb_resized_size, MALLOC_CAP_SPIRAM);
  if (!rgb_resized) {
    heap_caps_free(rgb_full);
    xSemaphoreGive(xBufferMutex);
    return "Gagal alokasi resized buffer";
  }

  esp_task_wdt_reset(); 
  if (!resize_rgb888(rgb_full, in_w, in_h, rgb_resized, out_w, out_h)) {
    heap_caps_free(rgb_full);
    heap_caps_free(rgb_resized);
    xSemaphoreGive(xBufferMutex);
    return "Gagal resize";
  }

  heap_caps_free(rgb_full);

  size_t pixel_count = out_w * out_h;
  float* inference_buffer = (float*) heap_caps_malloc(pixel_count * sizeof(float), MALLOC_CAP_SPIRAM);
  if (!inference_buffer) {
    heap_caps_free(rgb_resized);
    xSemaphoreGive(xBufferMutex);
    return "Gagal alokasi inference buffer";
  }

  // Konversi ke grayscale dengan WDT reset
  for (size_t i = 0; i < pixel_count; i++) {
    size_t idx = i * 3;
    float gray = (rgb_resized[idx] * 0.299f) +
                 (rgb_resized[idx + 1] * 0.587f) +
                 (rgb_resized[idx + 2] * 0.114f);
    inference_buffer[i] = gray;
    if (i % 1000 == 0) esp_task_wdt_reset();
  }

  heap_caps_free(rgb_resized);

  signal_t signal;
  signal.total_length = pixel_count;
  signal.get_data = [inference_buffer, pixel_count](size_t offset, size_t length, float *out_ptr) -> int {
    if (offset + length > pixel_count) return EIDSP_OUT_OF_MEM;
    memcpy(out_ptr, inference_buffer + offset, length * sizeof(float));
    return EIDSP_OK;
  };

  ei_impulse_result_t result = { 0 };
  esp_task_wdt_reset();
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

  heap_caps_free(inference_buffer);
  xSemaphoreGive(xBufferMutex);

  if (res != EI_IMPULSE_OK) {
    Serial.printf("❌ classifyImage: run_classifier gagal (kode: %d)\n", res);
    return "Gagal klasifikasi (error internal)";
  }

  float max_score = 0.0f;
  String detected_label = "";
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > max_score) {
      max_score = result.classification[i].value;
      detected_label = String(result.classification[i].label);
    }
  }

  String output;
  if (max_score > 0.5f) {
    output = detected_label + " (" + String(max_score * 100, 1) + "%)";
  } else {
    output = "Tidak terdeteksi";
  }

  Serial.println("✅ classifyImage: Hasil = " + output);
  
  if (max_score > 0.5f) {
    lastDetectionLabel = detected_label;
    lastDetectionConfidence = max_score;
    lastDetectionTimestamp = millis();

    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      if (String(result.classification[i].label) == detected_label) {
        classificationCounts[i]++;
        break;
      }
      vTaskDelay(1);
    }
  }
  return output;
}

// ======================================================
//    sendPhotoToTelegram - OPTIMIZED & STABILIZED
// ======================================================
String sendPhotoToTelegram(String targetChatId) {
  esp_task_wdt_reset();
  if (!checkWiFiConnection()) {
    return "Gagal: Tidak ada koneksi WiFi";
  }
  if (ESP.getFreePsram() < 200000) {
    return "Memori tidak cukup";
  }
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb || fb->format != PIXFORMAT_JPEG || fb->len == 0 || fb->len > 800000) {
    if (fb) esp_camera_fb_return(fb);
    return "Gagal mengambil frame";
  }

  String url = "https://api.telegram.org/bot" + bot_token + "/sendPhoto";
  WiFiClientSecure *client = new WiFiClientSecure;
  if (!client) {
    esp_camera_fb_return(fb);
    return "Gagal alokasi client";
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
    return "Gagal koneksi ke Telegram";
  }

  String boundary = "----ESP32CAMBoundary";
  String body1 = "--" + boundary + "\r\n"
                 "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n"
                 + targetChatId + "\r\n"
                 "--" + boundary + "\r\n"
                 "Content-Disposition: form-data; name=\"photo\"; filename=\"cam.jpg\"\r\n"
                 "Content-Type: image/jpeg\r\n\r\n";

  String body2 = "\r\n--" + boundary + "--\r\n";

  size_t contentLength = body1.length() + fb->len + body2.length();
  String header = "POST /bot" + bot_token + "/sendPhoto HTTP/1.1\r\n"
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
      char c = client->read();
      response += c;
      if (response.length() % 512 == 0) {
        esp_task_wdt_reset();
      }
    }
    esp_task_wdt_reset();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  delete client;

  if (response.indexOf("\"ok\":true") != -1) {
    return "Foto terkirim";
  } else {
    Serial.println("Telegram sendPhoto response: " + response);
    return "Gagal: Respon tidak valid";
  }
}

// Overload default ke grup
String sendPhotoToTelegram() {
  return sendPhotoToTelegram(String(GROUP_CHAT_ID));
}

void loop() {
  // Semua logika dijalankan via FreeRTOS tasks
  // Fungsi ini hanya ada agar linker tidak error
    vTaskDelete(NULL); // opsional: hentikan loopTask selamanya

}
