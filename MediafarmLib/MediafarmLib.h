// MediafarmLib.h

/* Mediafarm library 

Development: Mediaflow


*/
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#include "DHT.h" //dht library

#define DIGITAL_PIN_COUNT 4
#define DIGITAL_PIN 4
#define DIGITAL_PIN_CONTROL 4

#define ANALOG_PIN_COUNT 6
#define ANALOG_PIN_CONTROL 3


#define DHTTYPE DHT21   // DHT 21 = DHT 22  (AM2302)

//initialization(√ ±‚»≠)
#define RESET 0
#define NULL 0



// Digital Pins
#define DHTPIN1 11
#define DHTPIN2 10
#define DHTPIN3 9
#define DHTPIN4 8

// msg index
#define MSG_ADDR 0
#define MSG_MODE 1
#define REQ_TYPE 2
#define DATA_LENGTH 3

//mode definition
#define SENSOR_MODE 0
#define CONTROL_MODE 1

// rs485
#define ENABLE_PIN 2
#define SEND_DELAY 200
#define SEND_SUCCESS 3
#define SEND_FAIL 3
#define FAIL_CRC 4


// serial communication baudrate
#define BAUD_RATE1 9600
#define BAUD_RATE2 19200
#define BAUD_RATE3 57600
#define BAUD_RATE4 115200

// serial read byte
#define SERIAL_READ_BYTE 30

//sensor type
#define SENSOR_BYTE 2
#define ANALOG 0
#define DIGITAL_BUS 1
#define DIGITAL_IO 2 
#define I2C_BUS 3


//sensor byte length
#define ANALOG_BYTE 2
#define DIGITAL_BUS_BYTE 2
#define DIGITAL_IO_BYTE 1 
#define I2C_BYTE 2

//basic protocol 
#define PROTOCOL_HEADER 4
#define PROTOCOL_FIRST_DATA 4

#define PROTOCOL_RESP_SENSOR 2
#define PROTOCOL_RESP_CONTROL 3

#define PROTOCOL_ANALOG 0
#define PROTOCOL_DIGITAL_BUS 1
#define PROTOCOL_I2C 3

#define PROTOCOL_RESPONSE_ERROR 0
#define PROTOCOL_ERROR_NONRESPONSE 0
#define PROTOCOL_ERROR_PARSING 1
#define PROTOCOL_ERROR_ETC 0



//control relay
#define CONTROL_HIGH 1
#define CONTROL_LOW 0
#define READ_CONTROL_INFORMATION 0 
#define DECIDE_DIGITAL_PIN 4
#define DECIDE_ANALOG_PIN 3



//light sensor
#define LIGHT_SENSOR_DELAY1 200
#define LIGHT_SENSOR_DELAY2 150
#define LIGHT_SENSOR_READ 2

//co2
#define CO2_ANALOG_MIN 160
#define CO2_ANALOG_MAX 800
#define CO2_PPM_MIN 400
#define CO2_PPM_MAX 2000


//shift register
#define ONE_BYTE_SIZE 256
#define SHIFT8 8


//separator
#define SEPARATOR 1

//CRC
#define CRC 1
#define SEND_CRC 2


class MediafarmLib {

private:

	

public:
	// arduino setting value
	int arduino_id;
	int arduino_mode;
	static const int response_sensor = 2;


	
	/*
	static const byte RespLen = 0;
	static const byte temper[4] = {0,0,0,0};
	static const byte humi[4] = {0,0,0,0};
	static const int analog = 0;
	static const byte anal[2]={0,0};
	static const byte light_conv[2]={0,0};
	*/
	static const int LIGHT_SENSOR_ADDRESS = 0x23;//light sensor I2C address
	static const int LIGHT_SENSOR_RESOLUTION = 0x10;//light sensor I2C address

	//const int LIGHT_SENSOR_ADDRESS = 0x23;

	MediafarmLib();
	void setArduinoConfig();

	byte getCrcValue(byte checkArray[], int lastlength);

};
