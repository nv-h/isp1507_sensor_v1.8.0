/*
  This is a library written for the SPG30
  By Ciara Jekel @ SparkFun Electronics, June 18th, 2018


  https://github.com/sparkfun/SparkFun_SGP30_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  SparkFun labored with love to create this code. Feel like supporting open
  source hardware? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14813


  CRC lookup table from Bastian Molkenthin

  Copyright (c) 2015 Bastian Molkenthin

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include <math.h>
#include <kernel.h>
#include <drivers/i2c.h>

#include "sgp30.h"

//Generates CRC8 for SGP30 from lookup table
static uint8_t sgp30_CRC8(uint16_t twoBytes);

static const uint8_t init_air_quality[2] = {0x20, 0x03};
static const uint8_t measure_air_quality[2] = {0x20, 0x08};
static const uint8_t get_baseline[2] = {0x20, 0x15};
static const uint8_t set_baseline[2] = {0x20, 0x1E};
static const uint8_t set_humidity[2] = {0x20, 0x61};
static const uint8_t measure_test[2] = {0x20, 0x32};
static const uint8_t get_feature_set_version[2] = {0x20, 0x2F};
static const uint8_t get_serial_id[2] = {0x36, 0x82};
static const uint8_t measure_raw_signals[2] = {0x20, 0x50};

static sgp30_sensor_t g_dev; // mallocできないのでここに宣言

//Start I2C communication using specified port
//Returns true if successful or false if no sensor detected
sgp30_sensor_t *sgp30_init_sensor(const struct device *i2c_dev, uint8_t addr)
{
  sgp30_sensor_t* dev = &g_dev;

  // inititalize sensor data structure
  dev->bus = i2c_dev;
  dev->addr = addr;

  SGP30ERR err = sgp30_getSerialID(dev, &dev->serialID);
  if (err != SGP30_SUCCESS)
    return NULL;
  if (dev->serialID == 0)
    return NULL;

  return dev;
}

//Initilizes sensor for air quality readings
//measureAirQuality should be called in 1 second intervals after this function
void sgp30_initAirQuality(sgp30_sensor_t *dev)
{
  i2c_write(dev->bus, init_air_quality, 2, dev->addr); //command to initialize air quality readin, dev->addrgs
}

//Measure air quality
//Call in regular intervals of 1 second to maintain synamic baseline calculations
//CO2 returned in ppm, Total Volatile Organic Compounds (TVOC) returned in ppb
//Will give fixed values of CO2=400 and TVOC=0 for first 15 seconds after init
//Returns SGP30_SUCCESS if successful or other error code if unsuccessful
bool sgp30_measureAirQuality(sgp30_sensor_t *dev)
{
  i2c_write(dev->bus, measure_air_quality, 2, dev->addr); //command to measure air quali, dev->addrty
  //Hang out while measurement is taken. datasheet says 10-12ms
  k_msleep(12);
  //Comes back in 6 bytes, CO2 data(MSB) / data(LSB) / Checksum / TVOC data(MSB) / data(LSB) / Checksum
  uint8_t rx_buf[6];
  i2c_read(dev->bus, rx_buf, 6, dev->addr);

  uint16_t _CO2 = rx_buf[0] << 8; //store MSB in CO2
  _CO2 |= rx_buf[1];              //store LSB in CO2
  uint8_t checkSum = rx_buf[2];   //verify checksum
  if (checkSum != sgp30_CRC8(_CO2))
    return false;                  //checksum failed
  uint16_t _TVOC = rx_buf[3] << 8; //store MSB in TVOC
  _TVOC |= rx_buf[4];              //store LSB in TVOC
  checkSum = rx_buf[5];            //verify checksum
  if (checkSum != sgp30_CRC8(_TVOC))
    return false;    //checksum failed
  dev->CO2 = _CO2;   //publish valid data
  dev->TVOC = _TVOC; //publish valid data
  return true;
}

//Returns the current calculated baseline from
//the sensor's dynamic baseline calculations
//Save baseline periodically to non volatile memory
//(like EEPROM) to restore after new power up or
//after soft reset using setBaseline();
//Returns SGP30_SUCCESS if successful or other error code if unsuccessful
SGP30ERR sgp30_getBaseline(sgp30_sensor_t *dev)
{
  i2c_write(dev->bus, get_baseline, 2, dev->addr);
  //Hang out while measurement is taken. datasheet says 10ms
  k_msleep(10);
  //Comes back in 6 bytes, baselineCO2 data(MSB) / data(LSB) / Checksum / baselineTVOC data(MSB) / data(LSB) / Checksum
  uint8_t rx_buf[6];
  i2c_read(dev->bus, rx_buf, 6, dev->addr);
  uint16_t _baselineCO2 = rx_buf[0] << 8; //store MSB in _baselineCO2
  _baselineCO2 |= rx_buf[1];              //store LSB in _baselineCO2
  uint8_t checkSum = rx_buf[2];           //verify checksum
  if (checkSum != sgp30_CRC8(_baselineCO2))
    return SGP30_ERR_BAD_CRC;                           //checksum failed
  uint16_t _baselineTVOC = rx_buf[3] << 8; //store MSB in _baselineTVOC
  _baselineTVOC |= rx_buf[4];              //store LSB in _baselineTVOC
  checkSum = rx_buf[5];                    //verify checksum
  if (checkSum != sgp30_CRC8(_baselineTVOC))
    return SGP30_ERR_BAD_CRC;         //checksum failed
  dev->baselineCO2 = _baselineCO2;   //publish valid data
  dev->baselineTVOC = _baselineTVOC; //publish valid data
  return SGP30_SUCCESS;
}

//Updates the baseline to a previous baseline
//Should only use with previously retrieved baselines
//to maintain accuracy
void sgp30_setBaseline(sgp30_sensor_t *dev, uint16_t baselineCO2, uint16_t baselineTVOC)
{
  uint8_t baseline_command[] = {
    set_baseline[0],
    set_baseline[1],
    baselineTVOC >> 8,   //write baseline TVOC MSB
    baselineTVOC & 0xFF, //write baseline TVOC LSB
    sgp30_CRC8(baselineTVOC), //write checksum TVOC baseline
    baselineCO2 >> 8,    //write baseline CO2 MSB
    baselineCO2 & 0xFF,  //write baseline CO2 LSB
    sgp30_CRC8(baselineCO2),  //write checksum CO2 baseline
  };

  i2c_write(dev->bus, baseline_command, sizeof(baseline_command), dev->addr);
}

//Set humidity
//humidity value is a fixed point 8.8 bit number
//Value should be absolute humidity from humidity sensor
//default value 0x0F80 = 15.5g/m^3
//minimum value 0x0001 = 1/256g/m^3
//maximum value 0xFFFF = 255+255/256 g/m^3
//sending 0x0000 resets to default and turns off humidity compensation
void sgp30_setHumidity(sgp30_sensor_t *dev, uint16_t humidity)
{
  uint8_t set_humidity_command[] = {
    set_humidity[0],
    set_humidity[1],
    humidity >> 8,   //write humidity MSB
    humidity & 0xFF, //write humidity LSB
    sgp30_CRC8(humidity), //write humidity checksum
  };

  i2c_write(dev->bus, set_humidity_command, sizeof(set_humidity_command), dev->addr);
}

//Set humidity and temperature to sensor as Compensation values
//Convert relative humidity to absolute humidity
//Temperature is needed to calc absolute humidity
void sgp30_setCompensation(sgp30_sensor_t *dev, float humidity, float temperature)
{
  //Convert relative humidity to absolute humidity
  double absHumidity = RHtoAbsolute(humidity, temperature);

  //Convert the double type humidity to a fixed point 8.8bit number
  uint16_t sensHumidity = doubleToFixedPoint(absHumidity);

  //Set the humidity compensation on the SGP30 to the measured value
  //If no humidity sensor attached, sensHumidity should be 0 and sensor will use default
  sgp30_setHumidity(dev, sensHumidity);
}

//gives feature set version number (see data sheet)
//Returns SGP30_SUCCESS if successful or other error code if unsuccessful
SGP30ERR sgp30_getFeatureSetVersion(sgp30_sensor_t *dev)
{
  i2c_write(dev->bus, get_feature_set_version, 2, dev->addr);
  //Hang out while measurement is taken. datasheet says 1-2ms
  k_msleep(2);
  //Comes back in 3 bytes, data(MSB) / data(LSB) / Checksum
  uint8_t rx_buf[3];
  i2c_read(dev->bus, rx_buf, 3, dev->addr);
  uint16_t _featureSetVersion = rx_buf[0] << 8; //store MSB in featureSetVerison
  _featureSetVersion |= rx_buf[1];              //store LSB in featureSetVersion
  uint8_t checkSum = rx_buf[2];                 //verify checksum
  if (checkSum != sgp30_CRC8(_featureSetVersion))
    return SGP30_ERR_BAD_CRC;                   //checksum failed
  dev->featureSetVersion = _featureSetVersion; //publish valid data
  return SGP30_SUCCESS;
}

//Intended for part verification and testing
//these raw signals are used as inputs to the onchip calibrations and algorithms
//Returns SUCCESS if successful or other error code if unsuccessful
SGP30ERR sgp30_measureRawSignals(sgp30_sensor_t *dev)
{
  i2c_write(dev->bus, measure_raw_signals, 2, dev->addr);
  //Hang out while measurement is taken. datasheet says 20-25ms
  k_msleep(25);
  //Comes back in 6 bytes, H2 data(MSB) / data(LSB) / Checksum / ethanol data(MSB) / data(LSB) / Checksum
  uint8_t rx_buf[6];
  i2c_read(dev->bus, rx_buf, 6, dev->addr);
  uint16_t _H2 = rx_buf[0] << 8; //store MSB in _H2
  _H2 |= rx_buf[1];              //store LSB in _H2
  uint8_t checkSum = rx_buf[2];  //verify checksum
  if (checkSum != sgp30_CRC8(_H2))
    return SGP30_ERR_BAD_CRC;                      //checksumfailed
  uint16_t _ethanol = rx_buf[3] << 8; //store MSB in ethanol
  _ethanol |= rx_buf[4];              //store LSB in ethanol
  checkSum = rx_buf[5];               //verify checksum
  if (checkSum != sgp30_CRC8(_ethanol))
    return SGP30_ERR_BAD_CRC; //checksum failed
  dev->H2 = _H2;             //publish valid data
  dev->ethanol = _ethanol;   //publish valid data
  return SGP30_SUCCESS;
}

//Soft reset - not device specific
//will reset all devices that support general call mode
void sgp30_generalCallReset(sgp30_sensor_t *dev)
{
  uint8_t general_call_reset = 0x06;
  i2c_write(dev->bus, &general_call_reset, 1, 0x00);
}

//readout of serial ID register can identify chip and verify sensor presence
//Returns SGP30_SUCCESS if successful or other error code if unsuccessful
SGP30ERR sgp30_getSerialID(sgp30_sensor_t *dev, int64_t *serialID)
{
  uint8_t rx_buf[9];
  //command to get serial ID
  i2c_write(dev->bus, get_serial_id, 2, dev->addr);

  //Hang out while measurement is taken.
  k_msleep(1);

  //Comes back in 9 bytes, H2 data(MSB) / data(LSB) / Checksum / ethanol data(MSB) / data(LSB) / Checksum
  i2c_read(dev->bus, rx_buf, 9, dev->addr);

  uint16_t _serialID1 = rx_buf[0] << 8; //store MSB to top of _serialID1
  _serialID1 |= rx_buf[1];              //store next byte in _serialID1
  uint8_t checkSum1 = rx_buf[2];        //verify checksum
  if (checkSum1 != sgp30_CRC8(_serialID1))
    return SGP30_ERR_BAD_CRC;                        //checksum failed
  uint16_t _serialID2 = rx_buf[3] << 8; //store next byte to top of _serialID2
  _serialID2 |= rx_buf[4];              //store next byte in _serialID2
  uint8_t checkSum2 = rx_buf[5];        //verify checksum
  if (checkSum2 != sgp30_CRC8(_serialID2))
    return SGP30_ERR_BAD_CRC;                        //checksum failed
  uint16_t _serialID3 = rx_buf[6] << 8; //store next byte to top of _serialID3
  _serialID3 |= rx_buf[7];              //store LSB in _serialID3
  uint8_t checkSum3 = rx_buf[8];        //verify checksum
  if (checkSum3 != sgp30_CRC8(_serialID3))
    return SGP30_ERR_BAD_CRC;                                                                            //checksum failed
  *serialID = ((uint64_t)_serialID1 << 32) + ((uint64_t)_serialID2 << 16) + ((uint64_t)_serialID3); //publish valid data
  return SGP30_SUCCESS;
}

//Sensor runs on chip self test
//Returns SGP30_SUCCESS if successful or other error code if unsuccessful
SGP30ERR sgp30_measureTest(sgp30_sensor_t *dev)
{
  i2c_write(dev->bus, measure_test, 2, dev->addr);
  //Hang out while measurement is taken. datasheet says 200-220ms
  k_msleep(220);
  //Comes back in 3 bytes, data(MSB) / data(LSB) / Checksum
  uint8_t rx_buf[3];
  i2c_read(dev->bus, rx_buf, 3, dev->addr);
  uint16_t results = rx_buf[0] << 8; //store MSB in results
  results |= rx_buf[1];              //store LSB in results
  uint8_t checkSum = rx_buf[2];      //verify checksum
  if (checkSum != sgp30_CRC8(results))
    return SGP30_ERR_BAD_CRC; //checksum failed
  if (results != 0xD400)
    return SGP30_SELF_TEST_FAIL; //self test results incorrect
  return SGP30_SUCCESS;
}

#ifndef SGP30_LOOKUP_TABLE
//Given an array and a number of bytes, this calculate CRC8 for those bytes
//CRC is only calc'd on the data portion (two bytes) of the four bytes being sent
//From: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
//Tested with: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
//x^8+x^5+x^4+1 = 0x31
uint8_t sgp30_CRC8(uint16_t data)
{
  uint8_t crc = 0xFF; //Init with 0xFF

  crc ^= (data >> 8); // XOR-in the first input byte

  for (uint8_t i = 0; i < 8; i++)
  {
    if ((crc & 0x80) != 0)
      crc = (uint8_t)((crc << 1) ^ 0x31);
    else
      crc <<= 1;
  }
  crc ^= (uint8_t)data; // XOR-in the last input byte

  for (uint8_t i = 0; i < 8; i++)
  {
    if ((crc & 0x80) != 0)
      crc = (uint8_t)((crc << 1) ^ 0x31);
    else
      crc <<= 1;
  }

  return crc; //No output reflection
}
#else
//Generates CRC8 for SGP30 from lookup table
uint8_t sgp30_CRC8(uint16_t data)
{
  uint8_t CRC = 0xFF;                          //inital value
  CRC ^= (uint8_t)(data >> 8);                 //start with MSB
  CRC = _CRC8LookupTable[CRC >> 4][CRC & 0xF]; //look up table [MSnibble][LSnibble]
  CRC ^= (uint8_t)data;                        //use LSB
  CRC = _CRC8LookupTable[CRC >> 4][CRC & 0xF]; //look up table [MSnibble][LSnibble]
  return CRC;
}
#endif

double RHtoAbsolute (float relHumidity, float tempC) {
  double eSat = 6.11 * pow(10.0, (7.5 * tempC / (237.7 + tempC)));
  double vaporPressure = (relHumidity * eSat) / 100; //millibars
  double absHumidity = 1000 * vaporPressure * 100 / ((tempC + 273) * 461.5); //Ideal gas law with unit conversions
  return absHumidity;
}

uint16_t doubleToFixedPoint( double number) {
  int power = 1 << 8;
  double number2 = number * power;
  uint16_t value = floor(number2 + 0.5);
  return value;
}
