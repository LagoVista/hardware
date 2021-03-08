#define CONFIG_SW_COEXIST_ENABLE 1

#define WIFI

#define PROD_BRD_V1

#define DEBUG_CYCLE

#include <Arduino.h>
#include <Objects.h>
#include "MessagePayload.h"

#define FIRMWARE_SKU "POOLCTLS"
#define FIRMWARE_VERSION "1.8.2"

#define POOL_SETPOINT "poolsetpoint"
#define SPA_SETPOINT "spasetpoint"
#define SAMPLE_INTERVAL "sampleinterval"

#ifdef DEBUG_CYCLE
bool debugStartup = false;
#endif

float spaSetpoint;
float poolSetpoint;
float sampleinterval = 1.0;

bool flow;
bool lowPressure;
bool highPressure;

const int TEMP1_IN_SUPPLY = 37;
const int TEMP2_IN_HEATED = 36;

const int LOW_PRESSURE = 19;
const int HIGH_PRESSURE = 22;
const int FLOW = 21;

const int FAN1 = 5;
const int FAN2 = 17;

const int COMPRESSOR1 = 18;
const int COMPRESSOR2 = 23;

bool _compressorOn = false;
bool _fanOn = false;

#define MODE_OFF "off"
#define MODE_POOL "pool"
#define MODE_SPA "spa"

String _mode;

#define BUFFER_SIZE 50
#define EXCLUDED 20

byte bufferIdx;
int bufferTemp1[BUFFER_SIZE];
int bufferTemp2[BUFFER_SIZE];
bool bufferLoaded = false;

int getAverage(int buffer[])
{
  long sum = 0;
  int i, j, n = BUFFER_SIZE;

  for (i = 0; i < n - 1; i++)
  {
    for (j = 0; j < n - i - 1; j++)
    {
      if (buffer[j] > buffer[j + 1])
      {
        int temp = buffer[j];
        buffer[j] = buffer[j + 1];
        buffer[j + 1] = temp;
      }
    }
  }

  int samplesIncluded = 0;
  for (int idx = EXCLUDED; idx < BUFFER_SIZE - EXCLUDED; ++idx)
  {
    sum += buffer[idx];
    samplesIncluded++;
    //console.print(String(buffer[idx]) + ", ");
  }

  //console.println("");

  if (samplesIncluded > 0)
  {
    return (int)(sum / samplesIncluded);
  }
  else
  {
    return samplesIncluded;
  }
}

PoolStatusMessagePayload messagePayload;

void setup_io()
{
  pinMode(TEMP1_IN_SUPPLY, INPUT);
  pinMode(TEMP2_IN_HEATED, INPUT);

  pinMode(LOW_PRESSURE, INPUT_PULLUP);
  pinMode(HIGH_PRESSURE, INPUT_PULLUP);
  pinMode(FLOW, INPUT_PULLUP);

  digitalWrite(FAN1, HIGH);
  digitalWrite(FAN2, HIGH);

  digitalWrite(COMPRESSOR1, HIGH);
  digitalWrite(COMPRESSOR2, HIGH);

  pinMode(FAN1, OUTPUT);
  pinMode(FAN2, OUTPUT);
  pinMode(COMPRESSOR1, OUTPUT);
  pinMode(COMPRESSOR2, OUTPUT);
}

void handleTopic(String cmd)
{
  if (cmd == "fan/on")
  {
    _fanOn = true;
  }
  else if (cmd == "fan/off")
  {
    _fanOn = false;
  }
  else if (cmd == "compressor/on")
  {
    _compressorOn = true;
  }
  else if (cmd == "compressor/off")
  {
    _compressorOn = false;
  }
  else if (cmd == "mode/pool")
  {
    _mode = MODE_POOL;
  }
  else if (cmd == "mode/spa")
  {
    _mode = MODE_SPA;
  }
  else if (cmd == "mode/off")
  {
    _mode = MODE_OFF;
  }
}

void registerAppSettings()
{
  state.registerFloat(POOL_SETPOINT, 90.0f);
  state.registerFloat(SPA_SETPOINT, 103.0f);
  state.registerFloat(SAMPLE_INTERVAL, 1.0);
}

void loadAppSettings()
{
  spaSetpoint = state.getFlt(SPA_SETPOINT);
  poolSetpoint = state.getFlt(POOL_SETPOINT);
  sampleinterval = state.getFlt(SAMPLE_INTERVAL);
}

void mqttCallback(String topic, byte *payload, size_t len)
{
  String topicPrefix = sysConfig.DeviceId;

  if (topicPrefix.length() + 1 > 0)
  {
    String cmd = topic.substring(topicPrefix.length() + 1);
    console.printVerbose("poolctls=receivetopic; // topic=" + cmd + ".");    
    handleTopic(cmd);
  }
}

void setup()
{
  configureI2C();
  configureConsole();

  #ifdef DEBUG_CYCLE
  pinMode(0, INPUT_PULLUP);
  #endif

  setup_io();
  initPins();

  welcome(FIRMWARE_SKU, FIRMWARE_VERSION);
  configureFileSystem();
  loadConfigurations();

  state.init(FIRMWARE_SKU, FIRMWARE_VERSION, "pcl001", 010);

  registerAppSettings();
  loadAppSettings();

  initDisplay(FIRMWARE_SKU, FIRMWARE_VERSION);

  spinWhileNotCommissioned();

  adc.setBankEnabled(1, true);
  adc.setBankEnabled(2, true);
  adc.setup(&ioConfig);

  wifiMgr.setup();
  mqttSubscribe("nuviot/dvcsrvc/" + sysConfig.DeviceId + "/#");
  mqttSubscribe(sysConfig.DeviceId + "/#");
  client.setMessageReceivedCallback(mqttCallback);
  client.WifiConnect(false);
}

void controlHeater()
{
  bool attemptStartPool = false;

  if (lowPressure &&
      highPressure &&
      flow)
  {
    digitalWrite(FAN1, _fanOn ? LOW : HIGH);
    digitalWrite(FAN2, _fanOn ? LOW : HIGH);
    digitalWrite(COMPRESSOR1, _compressorOn ? LOW : HIGH);
    digitalWrite(COMPRESSOR2, _compressorOn ? LOW : HIGH);

    messagePayload.fan = _fanOn ? "on" : "off";
    messagePayload.compressor = _compressorOn ? "on" : "off";
  }
  else
  {
    attemptStartPool = _fanOn || _compressorOn;

    digitalWrite(FAN1, HIGH);
    digitalWrite(FAN2, HIGH);
    digitalWrite(COMPRESSOR1, HIGH);
    digitalWrite(COMPRESSOR2, HIGH);

    messagePayload.fan = "off";
    messagePayload.compressor = "off";
  }

  if (attemptStartPool)
  {
    String topic = "pool/poolpumpcmd/" + sysConfig.DeviceId + "/on";
    mqttPublish(topic, "");
  }
}

void checkTemperatues()
{
  if (_mode == MODE_POOL)
  {
    if (messagePayload.poolTemperature > poolSetpoint)
    {
      if (_compressorOn)
      {
        mqttPublish("pool/heating/" + sysConfig.DeviceId + "/false");
      }

      _compressorOn = false;
      _fanOn = false;
    }
    else if (messagePayload.poolTemperature < (poolSetpoint - 1))
    {
      if (!_compressorOn)
      {
        mqttPublish("pool/heating/" + sysConfig.DeviceId + "/true");
      }

      _compressorOn = true;
      _fanOn = true;
    }
  }
  else if (_mode == MODE_SPA)
  {
    if (messagePayload.poolTemperature > spaSetpoint)
    {
      if (_compressorOn)
      {
        mqttPublish("pool/heating/" + sysConfig.DeviceId + "/false");
      }

      _compressorOn = false;
      _fanOn = false;
    }
    else if (messagePayload.poolTemperature < (spaSetpoint - 1))
    {
      if (!_compressorOn)
      {
        mqttPublish("pool/heating/" + sysConfig.DeviceId + "/true");
      }

      _compressorOn = true;
      _fanOn = true;
    }    
  }
  else
  {
    _fanOn = false;
    _compressorOn = false;
  }
}

void checkIO()
{
  bool lp = digitalRead(LOW_PRESSURE) == 1;  // ? "ok" : "warning";
  bool hp = digitalRead(HIGH_PRESSURE) == 1; // ? "ok" : "warning";
  bool flw = digitalRead(FLOW) == 0;

  messagePayload.lowPressure = lp ? "ok" : "warning";
  messagePayload.highPressure = hp ? "ok" : "warning";
  messagePayload.flow = flw ? "ok" : "warning";

  if (lowPressure != lp)
  {
    mqttPublish("pool/lowpressure/" + sysConfig.DeviceId + "/" + messagePayload.lowPressure);
    lowPressure = lp;
  }

  if (highPressure != hp)
  {
    mqttPublish("pool/highpressure/" + sysConfig.DeviceId + "/" + messagePayload.highPressure);
    highPressure = hp;
  }

  if (flow != flw)
  {
    mqttPublish("pool/flow/" + sysConfig.DeviceId + "/" + messagePayload.flow);
    flow = flw;
  }
}

void readTemperatures()
{
  uint16_t count1 = analogRead(TEMP1_IN_SUPPLY);
  uint16_t count2 = analogRead(TEMP2_IN_HEATED);

  delay(100);

  bufferTemp1[bufferIdx] = count1;
  bufferTemp2[bufferIdx] = count2;

  if (bufferIdx++ == BUFFER_SIZE)
  {
    bufferIdx = 0;
    bufferLoaded = true;
  }

  int tempOne;
  int tempTwo;

  if (bufferLoaded == true)
  {
    tempOne = getAverage(bufferTemp1);
    tempTwo = getAverage(bufferTemp2);
  }
  else
  {
    tempOne = count1;
    tempTwo = count2;
  }

  //console.println("COUNTS: " + String(count1) + " " + String(count2) + " " + String(tempOne) + " " + String(tempTwo) + " " + String(bufferIdx) + " " + String(bufferLoaded));

  messagePayload.mode = MODE_OFF;
  messagePayload.countsIn = tempTwo;
  messagePayload.countsOut = tempOne;

  messagePayload.poolTemperature = ((int)(tempTwo / (ioConfig.ADC1Scaler / 10.0f))) / 10.0f;
  messagePayload.heatedTemperature = ((int)(tempOne / (ioConfig.ADC1Scaler / 10.0f))) / 10.0f;
}

long _sendCount;
long _nextPublish;
long _lastPing;

void writeDisplay()
{
  display.clearBuffer();
  display.drawString(0, 0, "SSID:");
  display.drawString(60, 0, state.getWiFiSSID());

  display.drawString(0, 20, String(flow).c_str());
  display.drawString(20, 20, String(highPressure).c_str());
  display.drawString(40, 20, String(lowPressure).c_str());

  display.drawString(0, 30, String(messagePayload.poolTemperature).c_str());
  display.drawString(50, 30, String(messagePayload.heatedTemperature).c_str());
  display.drawString(0, 40, "Sent");
  display.drawString(35, 40, String(_sendCount).c_str());

  display.drawString(70, 40, "RSSI:");
  display.drawString(100, 40, String(wifiMgr.getRSSI()).c_str());

  display.sendBuffer();
}

void sendPayload()
{
  String topic = "pool/poolstat/" + sysConfig.DeviceId;

  messagePayload.poolSetpoint = state.getFlt(POOL_SETPOINT);
  messagePayload.spaSetpoint = state.getFlt(SPA_SETPOINT);
  messagePayload.version = FIRMWARE_VERSION;

  messagePayload.rssi = wifiMgr.getRSSI();
  mqttPublish(topic, messagePayload.getSON());
}

void loop()
{
#ifdef DEBUG_CYCLE
  if(digitalRead(0) == LOW) {
    ESP.restart();
  }
#endif

  if (!state.getIsConfigurationModeActive())
  {
    checkIO();
    readTemperatures();

    checkTemperatues();

    controlHeater();
    writeDisplay();

    if (_nextPublish < millis())
    {
      commonLoop();
      sendPayload();
      _nextPublish = millis() + (sysConfig.SendUpdateRate * 1000);
      _sendCount++;
    }
  }

  state.loop();
}