#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include "communication.h"

WiFiClient esp_client;
PubSubClient client(esp_client);

// long last_message = 0;
// char message[50];
// int value = 0;
float POS = 0.0;         // current position
float POS_DESIRED = 0.0; // desired position

void init_wifi();
void init_mqtt();
void reconnect_mqtt();
void callback_mqtt(char *topic, byte *message, unsigned int length);

void setup()
{
    Serial.begin(115200);

    init_wifi();
    init_mqtt();
}

void loop()
{
    if (!client.connected())
    {
        reconnect_mqtt();
    }
    client.loop();

    // TODO CONTROL ALGORITHM

    float velocity_out = POS;
    float omega_out = POS_DESIRED;

    char velocity_string[8];
    dtostrf(velocity_out, 1, 2, velocity_string);
    client.publish(VEL_TOPIC, velocity_string);

    char omega_string[8];
    dtostrf(omega_out, 1, 2, omega_string);
    client.publish(OMEGA_TOPIC, omega_string);
}

void init_wifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
}

void init_mqtt()
{
    client.setServer(MQTT_HOST, MQTT_PORT);
    client.setCallback(callback_mqtt);
}

void reconnect_mqtt()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect(MQTT_CLIENT_NAME))
        {
            Serial.println("connected");
            // Subscribe to topics
            client.subscribe(POS_TOPIC);
            client.subscribe(POS_DESIRED_TOPIC);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void callback_mqtt(char *topic, byte *message, unsigned int length)
{
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String message_temp;
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)message[i]);
        message_temp += (char)message[i];
    }
    if (String(topic) == POS_TOPIC)
    {
        POS = message_temp.toFloat();
    }

    if (String(topic) == POS_DESIRED_TOPIC)
    {
        POS_DESIRED = message_temp.toFloat();
    }
}
