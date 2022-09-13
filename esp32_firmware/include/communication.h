// communication.h

/* WiFi network parameters */
#define SSID "equipo_1"
#define PASSWORD "password"

/* MQTT parameters */
#define MQTT_HOST "192.168.4.1"
#define MQTT_PORT 1883
#define MQTT_CLIENT_NAME "ROBOT_1"

/* Publish topics */
#define VEL_R_TOPIC "ROBOT_1/VEL_R"
#define VEL_L_TOPIC "ROBOT_1/VEL_L"

/* Subscribe topics */
#define POS_X_TOPIC "ROBOT_1/POSITION_X"
#define POS_Y_TOPIC "ROBOT_1/POSITION_Y"

#define POS_X_DESIRED_TOPIC "ROBOT_1/POSITION_DES_X"
#define POS_Y_DESIRED_TOPIC "ROBOT_1/POSITION_DES_Y"

#define THETA_TOPIC "ROBOT_1/THETA"
