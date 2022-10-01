#ifndef __QMP6988_H__
#define __QMP6988_H__

#include <stdbool.h>
#include <stdint.h>

#include <drivers/i2c.h>

#define QMP6988_SLAVE_ADDRESS_L (0x70)
#define QMP6988_SLAVE_ADDRESS_H (0x56)

#define QMP6988_CHIP_ID 0x5C

#define QMP6988_CHIP_ID_REG     0xD1
#define QMP6988_RESET_REG       0xE0 /* Device reset register */
#define QMP6988_DEVICE_STAT_REG 0xF3 /* Device state register */
#define QMP6988_CTRLMEAS_REG    0xF4 /* Measurement Condition Control Register */
/* data */
#define QMP6988_PRESSURE_MSB_REG    0xF7 /* Pressure MSB Register */
#define QMP6988_TEMPERATURE_MSB_REG 0xFA /* Temperature MSB Reg */

/* compensation calculation */
#define QMP6988_CALIBRATION_DATA_START  0xA0 /* QMP6988 compensation coefficients */
#define QMP6988_CALIBRATION_DATA_LENGTH 25

/* power mode */
#define QMP6988_SLEEP_MODE  0x00
#define QMP6988_FORCED_MODE 0x01
#define QMP6988_NORMAL_MODE 0x03

#define QMP6988_CTRLMEAS_REG_MODE__POS 0
#define QMP6988_CTRLMEAS_REG_MODE__MSK 0x03
#define QMP6988_CTRLMEAS_REG_MODE__LEN 2

/* oversampling */
#define QMP6988_OVERSAMPLING_SKIPPED 0x00
#define QMP6988_OVERSAMPLING_1X      0x01
#define QMP6988_OVERSAMPLING_2X      0x02
#define QMP6988_OVERSAMPLING_4X      0x03
#define QMP6988_OVERSAMPLING_8X      0x04
#define QMP6988_OVERSAMPLING_16X     0x05
#define QMP6988_OVERSAMPLING_32X     0x06
#define QMP6988_OVERSAMPLING_64X     0x07

#define QMP6988_CTRLMEAS_REG_OSRST__POS 5
#define QMP6988_CTRLMEAS_REG_OSRST__MSK 0xE0
#define QMP6988_CTRLMEAS_REG_OSRST__LEN 3

#define QMP6988_CTRLMEAS_REG_OSRSP__POS 2
#define QMP6988_CTRLMEAS_REG_OSRSP__MSK 0x1C
#define QMP6988_CTRLMEAS_REG_OSRSP__LEN 3

/* filter */
#define QMP6988_FILTERCOEFF_OFF 0x00
#define QMP6988_FILTERCOEFF_2   0x01
#define QMP6988_FILTERCOEFF_4   0x02
#define QMP6988_FILTERCOEFF_8   0x03
#define QMP6988_FILTERCOEFF_16  0x04
#define QMP6988_FILTERCOEFF_32  0x05

#define QMP6988_CONFIG_REG             0xF1 /*IIR filter co-efficient setting Register*/
#define QMP6988_CONFIG_REG_FILTER__POS 0
#define QMP6988_CONFIG_REG_FILTER__MSK 0x07
#define QMP6988_CONFIG_REG_FILTER__LEN 3

#define SUBTRACTOR 8388608

typedef struct _qmp6988_cali_data {
    int32_t COE_a0;
    int16_t COE_a1;
    int16_t COE_a2;
    int32_t COE_b00;
    int16_t COE_bt1;
    int16_t COE_bt2;
    int16_t COE_bp1;
    int16_t COE_b11;
    int16_t COE_bp2;
    int16_t COE_b12;
    int16_t COE_b21;
    int16_t COE_bp3;
} qmp6988_cali_data_t;

typedef struct _qmp6988_fk_data {
    float a0, b00;
    float a1, a2, bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_fk_data_t;

typedef struct _qmp6988_ik_data {
    int32_t a0, b00;
    int32_t a1, a2;
    int64_t bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_ik_data_t;

typedef struct {
    const struct device *bus;
    uint16_t             addr;

    qmp6988_cali_data_t cali;
    qmp6988_fk_data_t   fk;
    qmp6988_ik_data_t   ik;

    float temperature;
    float pressure;
} qmp6988_sensor_t;

qmp6988_sensor_t *qmp6988_init_sensor(const struct device *i2c_dev, uint8_t addr);
bool              qmp6988_calcPressure(qmp6988_sensor_t *dev, float *pressure, float *temperature);
void              qmp6988_setpPowermode(qmp6988_sensor_t *dev, uint8_t mode);
void              qmp6988_setFilter(qmp6988_sensor_t *dev, uint8_t filter);
void              qmp6988_setOversamplingP(qmp6988_sensor_t *dev, uint8_t oversampling_p);
void              qmp6988_setOversamplingT(qmp6988_sensor_t *dev, uint8_t oversampling_t);

#endif
