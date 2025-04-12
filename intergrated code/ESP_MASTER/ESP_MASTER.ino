#include <esp_now.h>
#include <WiFi.h>

// MAC Address of the Receiver ESP32
uint8_t receiverMAC[] = {0x24, 0x6F, 0x28, 0xD3, 0xB0, 0x6C};  // Change this to your receiver's MAC

typedef struct Message {
    char text[32];
} Message;

Message messageData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed!");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    strcpy(messageData.text, "Hello");
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&messageData, sizeof(messageData));

    if (result == ESP_OK) {
        Serial.println("Message sent");
    } else {
        Serial.println("Message failed to send");
    }

    delay(2000); // Send message every 2 seconds
}
