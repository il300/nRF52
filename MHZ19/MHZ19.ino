/*
* https://github.com/il300/MHZ19
*/

//#define MY_DEBUG 

//#define MY_REPEATER_FEATURE

// Enable passive mode
//#define MY_PASSIVE_NODE

// Passive mode requires static node ID
#define MY_NODE_ID 121

// Enable and select radio type attached
#define MY_RADIO_NRF5_ESB

#define BAUDRATE 9600 
#define MHZ19_ERRORS 0    // Set to 0 to disable error prints

// Libraries
#include <MySensors.h>
#include "MHZ19.h"        //https://github.com/WifWaf/MH-Z19
#include <TimeLib.h>

MHZ19 sensorMHZ19;        // Constructor for library

// Feel free to change this:
#define co2MeasurementInterval 10000     // Time to wait between reads (in milliseconds).
unsigned long getDataTimer = 0;

// Mysensors settings
#define CHILD_ID_AIQ      0
#define CHILD_ID_TEMP     1        // for MySensors. Within this node each sensortype should have its own ID number.
#define CHILD_ID_CUSTOM   2

// MYSENSORS COMMUNICATION VARIABLES
//\MySensors\lib\MySensors-master\core\MyMessage.h
MyMessage Co2Msg(CHILD_ID_AIQ, V_LEVEL);
//MyMessage Co2unitMsg(CHILD_ID_AIQ, V_UNIT_PREFIX);

MyMessage tMsg(CHILD_ID_TEMP, V_TEMP);

MyMessage customMsg(CHILD_ID_CUSTOM, V_CUSTOM);
MyMessage customMsg1(CHILD_ID_CUSTOM, V_VAR1);
MyMessage customMsg2(CHILD_ID_CUSTOM, V_VAR2);

void presentation()
{
  sendSketchInfo("AIQ Sensor CO2 MH-Z19", "1.0");

  present(CHILD_ID_AIQ, S_AIR_QUALITY);
  //send(Co2unitMsg.set("ppm"));
  
  present(CHILD_ID_TEMP, S_TEMP);
  
  present(CHILD_ID_CUSTOM, S_CUSTOM);
}

// Serial
//#define PIN_SERIAL_RX       (12)
//#define PIN_SERIAL_TX       (11)

void setup() 
{
  //Serial.setPins(PIN_SERIAL_RX, PIN_SERIAL_TX);
  Serial.begin(BAUDRATE);
  sensorMHZ19.begin(Serial);
  sensorMHZ19.autoCalibration();    // Turn auto calibration ON (OFF autoCalibration(false))

  send(customMsg2.set("type command to VAR1"));
}

void loop() 
{
  if (millis() - getDataTimer >= co2MeasurementInterval)
  {
    int co2ppm = sensorMHZ19.getCO2();
    int8_t temp = sensorMHZ19.getTemperature();
        
    send(Co2Msg.set((int16_t)co2ppm)); 
    send(tMsg.set((int16_t)temp));

    getDataTimer = millis();
   }
}

void receive(const MyMessage &message) {
  if (message.type==V_VAR1) {
    String msgData = message.getString();
    
    if (msgData == "calibrate") {
      sensorMHZ19.calibrate();
      send(customMsg1.set("ok calibrate"));
    
    } else if (msgData == "recoveryReset") {
      sensorMHZ19.recoveryReset();
      send(customMsg1.set("ok recoveryReset"));

    } else if (msgData == "verify") {
      sensorMHZ19.verify();
      send(customMsg1.set((sensorMHZ19.errorCode != ERRORCODE::RESULT_OK) ? "verify Error" : "verify ok"));

    } else if (msgData == "getVersion") {
      char rVersion[4];
      sensorMHZ19.getVersion(rVersion);
      rVersion[4] = 0;
      send(customMsg1.set(rVersion));

    //} else if (msgData == "getBackgroundCO2") {
    //  int16_t iData = sensorMHZ19.getBackgroundCO2();
    //  send(customMsg1.set(iData));

    //} else if (msgData == "getTempAdjustment") {
    //  send(customMsg1.set(sensorMHZ19.getTempAdjustment()));

    } else if (msgData == "getCO2") {
      send(customMsg1.set((int16_t)sensorMHZ19.getCO2()));
      
    } else if (msgData == "getCO2Raw") {
      send(customMsg1.set((int16_t)sensorMHZ19.getCO2Raw()));

    } else if (msgData == "getTransmittance") {
      send(customMsg1.set((int16_t)sensorMHZ19.getTransmittance()));
      
    } else if (msgData == "getTemperature") {
      send(customMsg1.set((int16_t)sensorMHZ19.getTemperature()));
      
    } else if (msgData == "getRange") {
      send(customMsg1.set((int16_t)sensorMHZ19.getRange()));

    } else if (msgData == "getABC") {
      send(customMsg1.set(sensorMHZ19.getABC()));

    } else if (msgData == "getAccuracy") {
      send(customMsg1.set(sensorMHZ19.getAccuracy()));

    } else if (msgData == "getPWMStatus") {
      send(customMsg1.set(sensorMHZ19.getPWMStatus()));

    } else {
      send(customMsg1.set("unknown command"));
      //send(customMsg1.set("unknown command:" + msgData));
    }
  } else {
    send(customMsg1.set("unknown command.type"));
    //send(customMsg1.set("unknown command.type:" + String(message.type)));
  }
}

// This is called when a new time value was received
void receiveTime(unsigned long time) {
  setTime(time);
  send(customMsg1.set("receiveTime"));
  //timeReceived = true;
}
