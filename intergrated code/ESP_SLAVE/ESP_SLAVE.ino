#include <esp_now.h>
#include <WiFi.h>

typedef struct Message {
    char text[32];
} Message;

Message receivedMessage;

// ✅ Updated function signature for ESP-IDF v5.1+
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&receivedMessage, incomingData, sizeof(receivedMessage));
    Serial.print("Received: ");
    Serial.println(receivedMessage.text);
}

void setup() {
    Serial.begin(115200);
    
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed!");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);  // ✅ Now matches the correct function signature
}

void loop() {
    // Do nothing, wait for messages
}
