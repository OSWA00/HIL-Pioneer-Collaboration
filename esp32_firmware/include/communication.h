// communication.h

/* WiFi network parameters */
#define SSID "equipo_1"
#define PASSWORD "password"

/* MQTT parameters */
#define MQTT_HOST "192.168.4.1"
#define MQTT_PORT 1883
#define MQTT_CLIENT_NAME "ROBOT_1"

/* Publish topics */
#define VEL_TOPIC "ROBOT_1/VELOCITY"
#define OMEGA_TOPIC "ROBOT_1/OMEGA"

/* Subscribe topics */
#define POS_TOPIC "ROBOT_1/POSITION"
#define POS_DESIRED_TOPIC "ROBOT_1/POSITION_DESIRED"
