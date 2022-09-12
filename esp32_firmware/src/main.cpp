#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include "communication.h"

WiFiClient esp_client;
PubSubClient client(esp_client);

/* Current positionn */
float POS_X = 0.0;
float POS_Y = 0.0;

/* Position desired */
float POS_DES_X = 0.0;
float POS_DES_Y = 0.0;

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

    float velocity_out = 0.0;
    float omega_out = 0.0;

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
            client.subscribe(POS_X_TOPIC);
            client.subscribe(POS_Y_TOPIC);
            client.subscribe(POS_X_DESIRED_TOPIC);
            client.subscribe(POS_Y_DESIRED_TOPIC);
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
    // Serial.print("Message arrived on topic: ");
    // Serial.print(topic);
    // Serial.print(". Message: ");
    String message_temp;
    // for (int i = 0; i < length; i++)
    // {
    //     Serial.print((char)message[i]);
    //     message_temp += (char)message[i];
    // }

    if (String(topic) == POS_X_TOPIC)
    {
        POS_X = message_temp.toFloat();
    }

    if (String(topic) == POS_Y_TOPIC)
    {
        POS_Y = message_temp.toFloat();
    }

    if (String(topic) == POS_X_DESIRED_TOPIC)
    {
        POS_DES_X = message_temp.toFloat();
    }

    if (String(topic) == POS_Y_DESIRED_TOPIC)
    {
        POS_DES_Y = message_temp.toFloat();
    }
}
