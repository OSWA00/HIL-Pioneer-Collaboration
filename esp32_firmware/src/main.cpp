#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include <math.h>
#include "communication.h"
#include "controller.h"

WiFiClient esp_client;
PubSubClient client(esp_client);

/* Current positionn */
float POS_X = 0.0;
float POS_Y = 0.0;
float THETA = 0.0;

/* Position desired */
float POS_DES_X = 0.0;
float POS_DES_Y = 0.0;

void init_wifi();
void init_mqtt();
void reconnect_mqtt();
void callback_mqtt(char *topic, byte *message, unsigned int length);
void pd_controller(float pos_des_x, float pos_des_y, float pos_X, float pos_y, float theta);

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

    pd_controller(POS_DES_X, POS_DES_Y, POS_X, POS_Y, THETA);
    delay(50);
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
            client.subscribe(THETA_TOPIC);
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
    Serial.println();

    if (String(topic) == POS_X_TOPIC)
    {
        POS_X = message_temp.toFloat();
        Serial.println(POS_X);
    }

    if (String(topic) == POS_Y_TOPIC)
    {
        POS_Y = message_temp.toFloat();
        Serial.println(POS_Y);
    }

    if (String(topic) == POS_X_DESIRED_TOPIC)
    {
        POS_DES_X = message_temp.toFloat();
        Serial.println(POS_DES_X);
    }

    if (String(topic) == POS_Y_DESIRED_TOPIC)
    {
        POS_DES_Y = message_temp.toFloat();
        Serial.println(POS_DES_Y);
    }
    if (String(topic) == THETA_TOPIC)
    {
        THETA = message_temp.toFloat();
        Serial.println(THETA);
    }
}

void pd_controller(float pos_des_x, float pos_des_y, float pos_x, float pos_y, float theta)
{

    float velocity_right = 0.0;
    float velocity_left = 0.0;

    // Calculate X and Y error x - x_d
    float x_error = pos_x - pos_des_x;
    float y_error = pos_y - pos_des_y;

    float theta_des = atan2(-y_error, -x_error);

    theta_error = theta - theta_des;

    if (theta_error > PI)
    {
        theta_error = theta_error - 2 * PI;
    }
    else if (theta_error < -PI)
    {
        theta_error = theta_error + 2 * PI;
    }

    lineal_error = sqrt(pow(x_error, 2) + pow(y_error, 2));

    // Proportional controller
    lineal_u_p = lineal_kp * lineal_error;
    theta_u_p = -theta_kp * theta_error;

    // Saturation
    if (lineal_u_p > 1.2)
    {
        lineal_u_p = 1.2;
    }

    if (theta_u_p > PI / 2)
    {
        theta_u_p = PI / 2;
    }

    if (theta_u_p < -PI / 2)
    {
        theta_u_p = -PI / 2;
    }

    // Outputs
    velocity_right = lineal_u_p + 0.5 * ROBOT_LENGTH * theta_u_p;
    velocity_left = lineal_u_p - 0.5 * ROBOT_LENGTH * theta_u_p;

    char vel_r_string[8];
    dtostrf(velocity_right, 1, 2, vel_r_string);
    client.publish(VEL_R_TOPIC, vel_r_string);

    char vel_l_string[8];
    dtostrf(velocity_left, 1, 2, vel_l_string);
    client.publish(VEL_L_TOPIC, vel_l_string);
}
