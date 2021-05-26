#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include "ADC_Read.h"
#include <Kalman_Filter.h>
#include <BasicLinearAlgebra.h>
#include "MPU6050_tockn.h"
#include "HC_SR04.h"

//******************************************************************************************************************
//*********** MQTT CONFIG ******************************************************************************************
//******************************************************************************************************************
const char *mqtt_server = "ioticos.org";
const int mqtt_port = 1883;
const char *mqtt_user = "Y9DB3HCxHac6tE3";
const char *mqtt_pass = "mahtjNS6SdtSXtQ";
const char *root_topic_subscribe = "qrZ422LFAtPNzjr/input";
const char *root_topic_publish = "qrZ422LFAtPNzjr/output";

//**************************************
//*********** VAR SENSOR ***************
//**************************************
ADC pot1, pot2;
const byte ADC1_PIN = 33;
const byte ADC2_PIN = 34;
uint16_t reading1;
uint16_t reading2;

//**************************************
//*********** WIFICONFIG ***************
//**************************************
const char *ssid = "NOMBRE DE LA RED";
const char *password = "CONTRASEÑA DE LA RED";

//**************************************
//*********** GLOBALES   ***************
//**************************************
WiFiClient espClient;
PubSubClient client(espClient);
char msg[25];
long count = 0;

//************************
//** F U N C I O N E S ***
//************************
void callback(char *topic, byte *payload, unsigned int length);
void reconnect();
void setup_wifi();

//******************************************************************************************************************
//*********** END MQTT CONFIG **************************************************************************************
//******************************************************************************************************************

//******************************************************************************************************************
//*********** KALMAN ***********************************************************************************************
//******************************************************************************************************************
using namespace BLA;
Kalman kalmanazo;

int sample_period_imu = 1000; //in microseconds
int sample_period_hc = 25000; //in microseconds
bool got_hc, got_imu;

ulong start_time_imu;
ulong start_time_hc;

MPU6050 mpu6050(Wire);
float accX, accY, accZ;


//******************************************************************************************************************
//*********** HCSR04 ***********************************************************************************************
//******************************************************************************************************************

#define TRIG_PIN 2
#define ECHO_PIN 3
#define ECHO_INT 0

HC_SR04 sensor_hc(TRIG_PIN, ECHO_PIN, ECHO_INT);

void setup()
{
	Serial.begin(115200);
	setup_wifi();
	client.setServer(mqtt_server, mqtt_port);
	client.setCallback(callback);


	sensor_hc.begin();


	//******************************************************************************************************************
	//*********** FALTA EL SETUP DE KALMANAZO **************************************************************************
	//******************************************************************************************************************

	pot1.begin(ADC1_PIN, 12, 3.3F);
	pot2.begin(ADC2_PIN, 12, 3.3F);
	got_hc = false;
	got_imu = false;
	start_time_imu = micros();
	start_time_hc = micros();

	Wire.begin();
	mpu6050.begin();
	mpu6050.calcGyroOffsets(true);
}

//******************************************************************************************************************
//*********** MAIN *************************************************************************************************
//******************************************************************************************************************

void loop()
{

	if (!client.connected())
	{
		reconnect();
	}

	if (client.connected())
	{
		/*
	String str = "La cuenta es -> " + String(count);
    str.toCharArray(msg,25);
    client.publish(root_topic_publish,msg);
    count++;
	*/
	sensor_hc.start();
		if (micros() - start_time_imu > sample_period_imu)
		{
			mpu6050.update();
			accX = mpu6050.getAccX();
			accY = mpu6050.getAccY();
			accZ = mpu6050.getAccZ();
			//leer imu
			start_time_imu = micros();
			got_imu = true;
		}

		if (micros() - start_time_hc > sample_period_hc)
		{
			//leer ultrasonico
			start_time_hc = micros();
			got_hc = true;
		}

		if (got_hc && got_imu)
		{
			//perform sensor fusión
		}

		else if (got_hc)
		{
			//asignar posición mediante hc
		}

		else if (got_imu)
		{
			//asignar posición por imu
		}

		reading1 = pot1.readRaw();
		reading2 = pot2.readRaw();
		String str1 = "Voltaje del potenciometro 1 ->" + String(reading1);
		str1.toCharArray(msg, 35);
		client.publish(root_topic_publish, msg);
		//Serial.printf("%d", reading);
		String str2 = "Voltaje del potenciometro 2 ->" + String(reading2);
		str2.toCharArray(msg, 35);
		client.publish(root_topic_publish, msg);
		delay(500);
	}
	client.loop();
}

//******************************************************************************************************************
//******* END MAIN *************************************************************************************************
//******************************************************************************************************************

//*****************************
//***    CONEXION WIFI      ***
//*****************************
void setup_wifi()
{
	delay(10);
	// Nos conectamos a nuestra red Wifi
	Serial.println();
	Serial.print("Conectando a ssid: ");
	Serial.println(ssid);

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("Conectado a red WiFi!");
	Serial.println("Dirección IP: ");
	Serial.println(WiFi.localIP());
}

//*****************************
//***    CONEXION MQTT      ***
//*****************************

void reconnect()
{

	while (!client.connected())
	{
		Serial.print("Intentando conexión Mqtt...");
		// Creamos un cliente ID
		String clientId = "IOTICOS_H_W_";
		clientId += String(random(0xffff), HEX);
		// Intentamos conectar
		if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass))
		{
			Serial.println("Conectado!");
			// Nos suscribimos
			if (client.subscribe(root_topic_subscribe))
			{
				Serial.println("Suscripcion ok");
			}
			else
			{
				Serial.println("fallo Suscripciión");
			}
		}
		else
		{
			Serial.print("falló :( con error -> ");
			Serial.print(client.state());
			Serial.println(" Intentamos de nuevo en 5 segundos");
			delay(5000);
		}
	}
}

//*****************************
//***       CALLBACK        ***
//*****************************

void callback(char *topic, byte *payload, unsigned int length)
{
	String incoming = "";
	Serial.print("Mensaje recibido desde -> ");
	Serial.print(topic);
	Serial.println("");
	for (int i = 0; i < length; i++)
	{
		incoming += (char)payload[i];
	}
	incoming.trim();
	Serial.println("Mensaje -> " + incoming);
}