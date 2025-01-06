// =================================================================================================
// eModbus: Copyright 2020 by Michael Harwerth, Bert Melis and the contributors to ModbusClient
//               MIT license - see license.md for details
// =================================================================================================
/* Example code for exchange data between DTSU666 CHINT and DTSU666H Huawei Power Meter
using eMOdbus library from https://github.com/eModbus
when you activate power meter in smartfon aplication the SUN2000 start sending request "0B 03 07 D1 00 01 D5 ED"  where
0B - Power Meter Modbus address (default 11)
03 - Function (READ_HOLD_REGISTER)
07 D1 - register address to read (dec 2001 )
00 01 - how many registers read (1 register mean 2 bytes)
D5 ED - CRC checksum

If data in register address 0x07D1 = 0x3B11  (or 0x3F80 in some old versions) the sun2000 recognize it as  PowerMeter is active
slave answer should look like this  "0B 03 3B 11 80 30 15"  in old versions "0B 03 02 3F 80 30 15" 
after this SUN200 send request  "0B 03 08 36 00 50 A7 32"
and we should send value 80  registers begnining from address 0x0836  (dec 2002)
from time to time SUN2000 send request 0B 03 08 A6 00 0A 27 24 but i do not know what exactly is stored in this 10 registers begining from 0x08A6. I asked support Huawai but they have not answered.
Information about HUAWEI registers addresses
https://forum.huawei.com/enterprise/en/how-to-connect-custom-meter-dtsu666-h-to-smartlogger1000/thread/599712-100027
Information about CHINT registers addresses
https://www.aggsoft.com/serial-data-logger/tutorials/modbus-data-logging/chint-instrument-dsu666-dtsu666.htm
I divided request to CHINT into two part because for one request the number of  registers in answer is to large
*/ 

#include "HardwareSerial.h"  // From <Arduino.h> for Serial, millis(), etc.
#include "Logging.h"
#include "ModbusClientRTU.h" // Modbus RTU client header from eModbus library
#include "ModbusServerRTU.h"
#include <WiFi.h>
#include <PubSubClient.h>

// Pin definitions and constants
#define RX1_PIN GPIO_NUM_17 // CHINT
#define TX1_PIN GPIO_NUM_4  // CHINT
#define REDE1_PIN GPIO_NUM_16 // CHINT
#define RX2_PIN GPIO_NUM_23 // Huawei
#define TX2_PIN GPIO_NUM_19 // Huawei
#define REDE2_PIN GPIO_NUM_21 // Huawei
#define HUAWEI_ID 0x0B // Address of the Huawei inverter
#define CHINT_ID 0x01 // Address of the CHINT meter
#define BAUDRATE 9600
#define HUAWEI_START_REG_1 0x0836 // Start address for Huawei data
#define HUAWEI_START_REG_2 0x08A6 // Second start address (unknown data)
#define CHINT_START_REG_1 0x2000  // Low registers
#define CHINT_START_REG_2 0x401E  // High registers
#define CHINT_REQUEST1 82 // Number of low registers
#define CHINT_REQUEST2 60 // Number of high registers
#define READ_TIMER 300 // Interval between CHINT reads (ms)
#define MQTT_TIMER 10000 // Interval for sending data to MQTT (ms)

// CHINT request mapping table
uint16_t CHINT_request_map[3][6] = {
  {0x2000, 0x201E, 0x203A, 0x401E, 0x4034, 0x4048}, 
  {15, 14, 12, 11, 10, 9}, 
  {0, 15, 29, 41, 52, 62}
};

// WiFi and MQTT configuration
const char* ssid = "wifi username";
const char* password = "wifi password";
const char* mqttServer = "mqtt server";
const int mqttPort = 1883;
const char* mqttUser = "mqtt username";
const char* mqttPassword = "mqtt password";

// Global variables
bool data_ready = false;
bool mqtt_on = true;
uint32_t mqtt_interval = millis();
int LED_BUILTIN = 2; // Built-in LED on DevKit
int errors = 1; // Error counter
int numb_chint_request = 0;
uint32_t chint_request_time = millis();
float Chint_RegData[CHINT_REQUEST1 / 2 + CHINT_REQUEST2 / 2]; // CHINT register data

int HuaweiTranslate[CHINT_REQUEST1/2+CHINT_REQUEST2/2]=
{6, 7, 8,   	// Phase A,B,C current   0 1 2
24,				// 3  ???
 3, 4, 5,		//Phase A,B,C Voltage   4 5 6
25,				//  7   ???
 0, 1, 2,		// A-B, B-C, C-A line voltage   8 9 10
34, 			// Frequency   11
 9,				// Active Power  12
10,11,12,		//Phase A,B,C Active Power   13 14 15
13,				// Reactive Power   16  
14,	15,16, 		//Reactive Power qa, qb,qc , A, B, C 17, 18 ,19,     
17,18,19,20, 	// Apparent Power total a b c  20  21  22 23
21,22,23,24,	// Power Factor pft, pfa, pfb, pfc  24, 25,26,27
36, 			// Total Active Electricity    28
37,38,39,		// Active Electricity  a b c  29 30 31 
41,				// Total Positive Active Electricity 32 
33,34,35,		// 33 34 35
46,				// Total negative Active electricity 36 
37,38,39, 		// 37 38 39  
40,				// 40  END Lower Chints registers
41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70
}; 
int Divider[CHINT_REQUEST1/2+CHINT_REQUEST2/2]=
{10, 10, 10,   		// Phase A-B,B-C,C-A Voltage  0 1 2
10, 10, 10, 		//Phase A,B,C Voltage		3 4 5
1000,1000, 1000,	// Phase A,B,C current		6 7 8
10, 				// Active Power total			9
10,10,10,			// Phase A,B,C Active Power   10,11,12
10,					// Reactive Power  total  13
10,10,10,			//Phase A,B,C Reactive Power   14 15 16
10,10,10,10,		// Apparent Power total, phase A,B,C  17, 18, 19, 20
10,10,10,10,		// Power Factor total , phase A, B, C 21,22, 23, 24
1,1,1,				// 25, 26, 27   ???
1, 					// 28  			???
1,1,1,    			// 29 30 31		???
1,					// 32			???
1,					// 33			
100,				// 34  Frequency
1,      			// 35			???
1000,				// 36  Total active electricity
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
}; 

WiFiClient espClient;
PubSubClient client(espClient);

ModbusServerRTU MBserver(Serial2, 2000, REDE2_PIN);  
ModbusClientRTU MbChint(Serial1, REDE1_PIN); 

// FC03: worker to create answer  Modbus function code 0x03 (READ_HOLD_REGISTER) for Huawei Master Client
ModbusMessage FC03(ModbusMessage request) {
	uint16_t words; 
	uint16_t address;
	float* HuReg;
	HuReg=&Chint_RegData[0];
	ModbusMessage response;  	// declare response message to be sent back to Huawei
	request.get(2, address);
	request.get(4, words);
	response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words*2));	//Set up message with serverID, FC and length of data (1 register is 2 bytes)
	
	// on request for 0x2001 register  "0B 03 07 D1 00 01 D5 ED"  we have to answer 0x3B11  (or 0x3F80 in some old versions) what means that Power Meter is active 
	if ((address == 0x07D1)&& (words==0x01))  response.add(uint16_t(0x3B11)); 
	else if ((address==HUAWEI_START_REG_1)&& (words=0x50)) {       // read data part 1 request "0B 03 08 36 00 50 A7 32"
		for (uint16_t i = 0; i < words/2; i++) {
			response.add(*(HuReg+HuaweiTranslate[i]));   // translate CHint on to Huawei addresses
		}
		for (int j = 0; j < words/2; j++)Serial.printf("  rej=%i       HUAWEIaddress=%i     wart=;%8.4f\n", j,2*j+HUAWEI_START_REG_1, *(HuReg+HuaweiTranslate[j]));
		}
	else if ((address == HUAWEI_START_REG_2)&& (words==0x0A)) {   //  read data part 2 request "0B 03 08 A6 00 0A 27 24" 
		for (uint16_t i = 0; i <  words/2; i++) {
			response.add(0x0000);  // here is possible put answer for 10 unknown registers, now answer is set to zero for safety
		}}
	else {  // No, either address or words are outside the limits. Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
	Serial.printf("ERROR bad address= %X \n", address);
	}
	return response;
}

// Data handler
// Define an onData handler function to receive the regular responses
// Arguments are received response message and the request's token
// The device has values all as IEEE754 float32 in two consecutive registers
// Read the CHint response in a loop
void handleData1(ModbusMessage response, uint32_t token) {
	uint8_t words;      			// response CHINT number of registers
	response.get(2, words);
	uint16_t offs = 3;   // First data are on pos 3, after server ID, function code and length byte
	int numb_bytes=words;
	int  Chint_RegData_ofset=0;
	switch (numb_bytes) {
	case 60:
		Chint_RegData_ofset= CHINT_request_map[2][0];
//		Serial.println("CZYTAM 15 rejestrów");
		break;
	case 56:
		Chint_RegData_ofset= CHINT_request_map[2][1];
//		Serial.println(" CZYTAM 14 rejestrów");
		break;
	case 48:
		Chint_RegData_ofset= CHINT_request_map[2][2];
//		Serial.println("   CZYTAM 12 rejestrów");
		break;
	case 44:
		Chint_RegData_ofset= CHINT_request_map[2][3];
//		Serial.println("     CZYTAM 11 rejestrów");
		break;
	case 40:
		Chint_RegData_ofset= CHINT_request_map[2][4];
//		Serial.println("      CZYTAM 10 rejestrów");
		break;
	case 36:
		Chint_RegData_ofset= CHINT_request_map[2][5];
//		Serial.println("      CZYTAM 9 rejestrów");
		break;
	default:
		Serial.printf("error read from CHINT  words= int %i  HEX %x: ",numb_bytes, words);
		words=0;
		data_ready = false;
		while(Serial.available()){
		Serial.read();
		}
		break;
	  }
//	Serial.printf(" words= int %i  HEX %x: \n",numb_bytes, words);
	for (uint8_t i = 0; i < words/4; i++) {
	offs = response.get(offs, Chint_RegData[Chint_RegData_ofset]);
//	Serial.print(Chint_RegData[Chint_RegData_ofset], HEX);
//	Serial.printf(" %i ", i);
	Chint_RegData[Chint_RegData_ofset]=Chint_RegData[Chint_RegData_ofset]/Divider[Chint_RegData_ofset];
	Chint_RegData_ofset++;
	}
	if (words>0) data_ready = true;
	chint_request_time = token;

}

// Define an onError handler function to receive error responses
// Arguments are the error code returned and a user-supplied token to identify the causing request
void handleError(Error error, uint32_t token) {
  // ModbusError wraps the error code and provides a readable error message for it
  ModbusError me(error);
  Serial.printf("\nError me number  %u  token %u \n", errors++, token);
  LOG_E("Error response: %02X - %s\n", (int)me, (const char *)me);
  digitalWrite(LED_BUILTIN, LOW);
}

void setMqttConnection(){
	uint32_t wifi_test=millis();
	WiFi.begin(ssid, password);
	while ((WiFi.status() != WL_CONNECTED) and mqtt_on) {
		if (millis() - wifi_test> 10*MQTT_TIMER){
			mqtt_on=false;
			Serial.println("Timeout, cannot connect to WiFi");
		} else { 
			delay(500);
			Serial.println("Connecting to WiFi..");
	}}
	if ((WiFi.status() == WL_CONNECTED)and mqtt_on) Serial.println("Connected to the WiFi network");
	client.setServer(mqttServer, mqttPort);
	uint32_t mqtt_test=millis();
	while (!client.connected()and mqtt_on) {
		Serial.println("Connecting to MQTT...");
		if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
			Serial.println("connected");
			} else {
				if(millis()-mqtt_test> 10*MQTT_TIMER){
					mqtt_on=false;
					Serial.println("Cannot connect to MQTT Server");
				} else {
					Serial.print("failed with state ");
					Serial.print(client.state());
					delay(2000);
	}}}
	if (client.connected()) {
		Serial.println("Connected to the MQTT server");
		digitalWrite(LED_BUILTIN, HIGH);  // blue LED on if MQTT server connection is OK
}
}
void sendMessage(String TopicName, String keyName, float keyValue) {
    String reg_data = "";
    reg_data += keyValue;
    String publishstring = "DTSU666/";
    publishstring += TopicName;
	publishstring += "/";
	publishstring += keyName;
    client.publish(publishstring.c_str(), reg_data.c_str());
}

void handleMqttPublish (){
	if (client.connected()){
		sendMessage("Voltage", "ua_V", Chint_RegData[3]);
		sendMessage("Voltage", "ub_V", Chint_RegData[4]);
		sendMessage("Voltage", "uc_V", Chint_RegData[5]);
		sendMessage("Current", "ia_A", Chint_RegData[6]);
		sendMessage("Current", "ib_A", Chint_RegData[7]);
		sendMessage("Current", "ic_A", Chint_RegData[8]);
		sendMessage("Power", "pt_kW", Chint_RegData[9]);
		sendMessage("Power", "pa_kW", Chint_RegData[10]);
		sendMessage("Power", "pb_kW", Chint_RegData[11]);
		sendMessage("Power", "pc_kW", Chint_RegData[12]);
		sendMessage("Power", "qt_kVar", Chint_RegData[13]);
		sendMessage("Power", "qa_kVar", Chint_RegData[14]);
		sendMessage("Power", "qb_kVar", Chint_RegData[15]);
		sendMessage("Power", "qc_kVar", Chint_RegData[16]);
		sendMessage("PowerFactor", "", Chint_RegData[21]);
		sendMessage("Frequency", "", Chint_RegData[34]);
		sendMessage("EnergyDemand", "", Chint_RegData[36]);
		sendMessage("Energy", "Import", Chint_RegData[41]);
		sendMessage("Energy", "Export", Chint_RegData[46]);
//		sendMessage("", "", Chint_RegData[]);
	} else{
		digitalWrite(LED_BUILTIN, LOW);	
		setMqttConnection();	
	}
}


// Setup function
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  while (!Serial) {}
  if (mqtt_on) setMqttConnection();
  
  Serial1.begin(9600, SERIAL_8N2, RX1_PIN, TX1_PIN);
  MbChint.onDataHandler(&handleData1);
  MbChint.onErrorHandler(&handleError);
  MbChint.setTimeout(2000);
  MbChint.begin(0);
  
  Serial2.begin(9600, SERIAL_8N2, RX2_PIN, TX2_PIN);
  MBserver.registerWorker(HUAWEI_ID, READ_HOLD_REGISTER, &FC03);
  MBserver.start(1);
}

// Loop function
void loop() {
  static uint32_t next_request = millis();
  if (millis() - next_request > READ_TIMER) {
    data_ready = false;
    if (numb_chint_request > 5) numb_chint_request = 0;
    Error err = MbChint.addRequest(millis(), CHINT_ID, READ_HOLD_REGISTER, 
                                   CHINT_request_map[0][numb_chint_request], 
                                   2 * CHINT_request_map[1][numb_chint_request]);
    if (err != SUCCESS) {
      ModbusError e(err);
      LOG_E("loop() Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }
    next_request = millis();
    numb_chint_request++;
  } else {
    if (data_ready) {
      digitalWrite(LED_BUILTIN, HIGH);
      data_ready = false;
      if ((millis() - mqtt_interval > MQTT_TIMER) && mqtt_on) {
        handleMqttPublish();
        mqtt_interval = millis();
      }
    }
  }
}
