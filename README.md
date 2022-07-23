# BLE Sensors on nRF52 device

This software mesurements temperature, humidity and pressure by i2c sensors bellow.

* nRF52 module: https://docid81hrs3j1.cloudfront.net/medialibrary/2019/06/isp_ble_DS1507.pdf
* Sensors
    + [M5Stack ENV III UNIT](https://docs.m5stack.com/en/unit/envIII)
        + [QMP6988](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/unit/enviii/QMP6988%20Datasheet.pdf)
        + [SHT30](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/unit/SHT3x_Datasheet_digital.pdf)
    + [M5Stack TVOC/eCO2 UNIT](https://docs.m5stack.com/en/unit/tvoc)
        + [SGP30](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/unit/Sensirion_Gas_Sensors_SGP30_Datasheet.pdf)
* Software environmet: nRF Connect SDK v2.0.0


## Pinout

|          | pin#  |
|----------|-------|
| LED gpio | 0.11  |
| I2C SCL  | 0.28  |
| I2C SDA  | 0.29  |


## BLE interface

* Service UUID `c7839aa8-1903-40b5-a8f0-426e09ffb390`
* Characteristic UUID `c7839aa9-1903-40b5-a8f0-426e09ffb390`

Data format is this.

```c
typedef struct {
	int battery_mV;
	float temperature;
	float humidity;
	float pressure;
} send_data_t;
```

Receive data by pc examle: [test/ble_from_pc.py](test/ble_from_pc.py)
