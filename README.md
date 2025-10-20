ESP32-CAM AI Motion Security Camera with Telegram

This project transforms an ESP32-CAM into a smart security camera that detects motion, classifies objects using a custom Edge Impulse AI model, and sends real-time alerts to a Telegram group. It also logs temperature, humidity, and system status to a Google Sheet for long-term monitoring.

Features :

PIR Motion Detection: Uses a PIR sensor to wake the system and trigger an event.

AI Image Classification: Captures an image and uses a trained Edge Impulse model to classify objects (e.g., person, animal, vehicle). This is optional and can be toggled on/off.

Telegram Alerts: Sends the captured photo and AI classification results directly to a specified Telegram group.

Remote Control: Manage the camera via a simple command interface within your Telegram group.

Environmental Sensing: A DHT22 sensor logs ambient temperature and humidity.

High-Temperature Alert: Automatically enters an "Alert Mode" with the LED on if the temperature exceeds a threshold (e.g., 35°C), potentially indicating a fire risk.

Google Sheets Logging: Periodically sends all sensor data and system stats to a Google Sheet for easy tracking and analysis.

Robust \& Non-Blocking: Built on FreeRTOS with multiple tasks to handle networking, sensors, and image processing without blocking each other, ensuring stability.

Secure: Keeps all your sensitive credentials (WiFi, API tokens) in a separate config.h file, which is ignored by Git.

Hardware Required :

ESP32-CAM Module: An AI-Thinker model is recommended.

PIR Motion Sensor: HC-SR501 or similar.

DHT22/AM2302 Sensor: For temperature and humidity.

FTDI Programmer: To upload the code to the ESP32-CAM.

Jumper Wires \& Breadboard

Software \& Libraries

This project can be built using the Arduino IDE or PlatformIO.

Required Libraries:

UniversalTelegramBot: For communicating with the Telegram API.

ArduinoJson: For parsing JSON (a dependency of UniversalTelegramBot).

DHT sensor library by Adafruit.

Your Custom Edge Impulse Library: You need to train your own image classification model and export it as an Arduino library.

Setup Instructions

1\. Edge Impulse: Train Your AI Model

Go to Edge Impulse and create a free account.

Create a new project.

Collect image data for the objects you want to classify (e.g., 'person', 'background'). You can use your phone or the ESP32-CAM itself to collect data.

Create an "impulse":

Input block: Image (96x96) - Note: The code uses 320x240 for capture but the model should be trained on smaller images for efficiency.

Processing block: Image

Learning block: Transfer Learning (Images)

Train your model.

Once you are happy with the accuracy, go to the Deployment tab.

Select Arduino library and click Build.

Download the generated .zip file.

In the Arduino IDE, go to Sketch -> Include Library -> Add .ZIP Library... and select the downloaded file.

Update the #include statement in main.cpp to point to your new library's header file.

2\. Telegram: Create a Bot

Open Telegram and search for the BotFather.

Send /newbot and follow the instructions to create your bot.

BotFather will give you a Bot Token. Save it.

Create a new group in Telegram and add your bot to it.

Send a message in the group (e.g., /start).

Open your browser and go to this URL, replacing <YOUR\_BOT\_TOKEN> with your token:

https://api.telegram.org/bot<YOUR\_BOT\_TOKEN>/getUpdates

Look for the "chat" object in the JSON response. The id will be a negative number. This is your Group Chat ID. Save it.

3\. Google Sheets: Create a Data Logger

Create a new blank Google Sheet.

Go to Extensions -> Apps Script.

Delete the default content and paste the code from the google-sheets-logger.js file in this repository.

Important: Change the SECRET\_TOKEN variable in the script to a long, random, and secret password.

Click Deploy -> New deployment.

For "Select type", choose Web app.

Configure it as follows:

Description: ESP32 CAM Data Logger

Execute as: Me

Who has access: Anyone

Click Deploy. Authorize the script when prompted.

Copy the Web app URL. This is your Google Sheets URL.

4\. Configure the Project

Clone this repository.

In the project folder, copy the config.h.example file and rename the copy to config.h.

Open config.h and fill in all the required credentials you collected in the steps above:

WIFI\_SSID \& WIFI\_PASSWORD

BOT\_TOKEN

GROUP\_CHAT\_ID

GOOGLE\_SHEETS\_URL

GOOGLE\_SHEETS\_TOKEN (must match the one in your Apps Script).

5\. Upload to ESP32-CAM

Connect the FTDI programmer to your ESP32-CAM. Remember to connect GPIO0 to GND to enable flashing mode.

In the Arduino IDE, select the correct board (AI Thinker ESP32-CAM) and port.

Click Upload.

After uploading, disconnect GPIO0 from GND and press the reset button on the ESP32-CAM.

Open the Serial Monitor at 115200 baud to see the logs.

Telegram Bot Commands

You can control the device by sending these commands in your Telegram group:

/start: Shows the welcome message and lists all commands.

/photo: Takes a photo immediately and sends it to the group.

/classify: Takes a photo, runs the AI model, and sends the photo with the classification result.

/status: Provides a full report of the system's current status (WiFi, PIR, sensors, etc.).

/report: Shows a summary of motion events and classified objects since the last reboot.

/temp: Gets the current temperature and humidity reading.

/flash: Toggles the onboard flash LED.

/piron: Enables the PIR motion sensor.

/piroff: Disables the PIR motion sensor.

/ml\_on: Enables AI classification when motion is detected.

/ml\_off: Disables AI, sending only a photo when motion is detected.

/reboot: Restarts the ESP32-CAM.

License

This project is open-source. Please feel free to use, modify, and distribute it. An MIT license is recommended.



