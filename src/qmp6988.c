// https://github.com/m5stack/UNIT_ENV
//
// MIT License
//
// Copyright (c) 2021 M5Stack
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <string.h>
#include <stdlib.h>
#include <drivers/i2c.h>

#include "qmp6988.h"

static uint8_t qmp6988_deviceCheck(qmp6988_sensor_t* dev);
static bool qmp6988_softwareReset(qmp6988_sensor_t* dev);
static int qmp6988_getCalibrationData(qmp6988_sensor_t* dev);
static int16_t qmp6988_convTx02e(qmp6988_ik_data_t *ik, int32_t dt);
static int32_t qmp6988_getPressure02e(qmp6988_ik_data_t *ik, int32_t dp, int16_t tx);
static bool qmp6988_read_data(qmp6988_sensor_t* dev, uint8_t reg_addr, uint8_t *data,  uint32_t len);
static bool qmp6988_write_reg(qmp6988_sensor_t* dev, uint8_t reg_addr, uint8_t data);

static qmp6988_sensor_t g_dev; // mallocできないのでここに宣言

qmp6988_sensor_t* qmp6988_init_sensor(const struct device *i2c_dev, uint8_t addr)
{
    qmp6988_sensor_t* dev;

    // if ((dev = malloc(sizeof(qmp6988_sensor_t))) == NULL)
    //     return NULL;
    dev = &g_dev;

    // inititalize sensor data structure
    dev->bus = i2c_dev;
    dev->addr = addr;

    uint8_t ret = qmp6988_deviceCheck(dev);
    if(ret == 0) {
        // free(dev);
        return NULL;
    }

    qmp6988_softwareReset(dev);
    qmp6988_getCalibrationData(dev);
    qmp6988_setpPowermode(dev, QMP6988_NORMAL_MODE);
    qmp6988_setFilter(dev, QMP6988_FILTERCOEFF_4);
    qmp6988_setOversamplingP(dev, QMP6988_OVERSAMPLING_8X);
    qmp6988_setOversamplingT(dev, QMP6988_OVERSAMPLING_1X);

    return dev;
}

bool qmp6988_calcPressure(qmp6988_sensor_t *dev, float *pressure, float *temperature)
{
    uint8_t err = 0;
    uint32_t P_read, T_read;
    int32_t P_raw, T_raw;
    uint8_t a_data_tr[6] = {0};
    int32_t T_int, P_int;

    // press
    err = qmp6988_read_data(dev, QMP6988_PRESSURE_MSB_REG, a_data_tr, 6);
    if(!err)
    {
        return false;
    }
    P_read = (uint32_t)(
        (((uint32_t)(a_data_tr[0])) << 16) |
        (((uint16_t)(a_data_tr[1])) <<  8) | (a_data_tr[2]));
    P_raw = (int32_t)(P_read - SUBTRACTOR);

    T_read = (uint32_t)(
        (((uint32_t)(a_data_tr[3])) << 16) |
        (((uint16_t)(a_data_tr[4])) <<  8) | (a_data_tr[5]));
    T_raw = (int32_t)(T_read - SUBTRACTOR);

    T_int = qmp6988_convTx02e(&(dev->ik), T_raw);
    P_int = qmp6988_getPressure02e(&(dev->ik), P_raw, T_int);
    *temperature = (float)T_int / 256.0f;
    *pressure = (float)P_int / 1600.0f;

    return true;
}

uint8_t qmp6988_deviceCheck(qmp6988_sensor_t* dev)
{
    uint8_t ret = 0;
    uint8_t id;

    ret = qmp6988_read_data(dev, QMP6988_CHIP_ID_REG, &(id), 1);
    if (!ret) {
        return 0;
    }

    if(id != QMP6988_CHIP_ID)
    {
        return 0;
    }

    return 1;
}

bool qmp6988_softwareReset(qmp6988_sensor_t* dev)
{
    uint8_t ret;

    ret = qmp6988_write_reg(dev, QMP6988_RESET_REG, 0);
    if (!ret) {
        return false;
    }

    k_msleep(20);

    ret = qmp6988_write_reg(dev, QMP6988_RESET_REG, 0);
    if (!ret) {
        return false;
    }
}

int qmp6988_getCalibrationData(qmp6988_sensor_t* dev)
{
    uint8_t a_data_tr[QMP6988_CALIBRATION_DATA_LENGTH] = {0};

    int ret = qmp6988_read_data(dev, QMP6988_CALIBRATION_DATA_START, a_data_tr, QMP6988_CALIBRATION_DATA_LENGTH);
    if (!ret)
    {
        return 0;
    }

    dev->cali.COE_a0 = (int32_t)(((a_data_tr[18] << 12) \
                | (a_data_tr[19] << 4)                  \
                | (a_data_tr[24] & 0x0f)) << 12);
    dev->cali.COE_a0 = dev->cali.COE_a0>>12;

    dev->cali.COE_a1 = (int16_t)(((a_data_tr[20]) << 8) | a_data_tr[21]);
    dev->cali.COE_a2 = (int16_t)(((a_data_tr[22]) << 8) | a_data_tr[23]);

    dev->cali.COE_b00 = (int32_t)(((a_data_tr[0] << 12) \
                | (a_data_tr[1] << 4)                   \
                | ((a_data_tr[24] & 0xf0) >> 4)) << 12);
    dev->cali.COE_b00 = dev->cali.COE_b00>>12;

    dev->cali.COE_bt1 = (int16_t)(((a_data_tr[2]) << 8) | a_data_tr[3]);
    dev->cali.COE_bt2 = (int16_t)(((a_data_tr[4]) << 8) | a_data_tr[5]);
    dev->cali.COE_bp1 = (int16_t)(((a_data_tr[6]) << 8) | a_data_tr[7]);
    dev->cali.COE_b11 = (int16_t)(((a_data_tr[8]) << 8) | a_data_tr[9]);
    dev->cali.COE_bp2 = (int16_t)(((a_data_tr[10]) << 8) | a_data_tr[11]);
    dev->cali.COE_b12 = (int16_t)(((a_data_tr[12]) << 8) | a_data_tr[13]);
    dev->cali.COE_b21 = (int16_t)(((a_data_tr[14]) << 8) | a_data_tr[15]);
    dev->cali.COE_bp3 = (int16_t)(((a_data_tr[16]) << 8) | a_data_tr[17]);

    dev->ik.a0 = dev->cali.COE_a0; // 20Q4
    dev->ik.b00 = dev->cali.COE_b00; // 20Q4

    dev->ik.a1 = 3608L * (int32_t)dev->cali.COE_a1 - 1731677965L; // 31Q23
    dev->ik.a2 = 16889L * (int32_t) dev->cali.COE_a2 - 87619360L; // 30Q47

    dev->ik.bt1 = 2982L * (int64_t)dev->cali.COE_bt1 + 107370906L; // 28Q15
    dev->ik.bt2 = 329854L * (int64_t)dev->cali.COE_bt2 + 108083093L; // 34Q38
    dev->ik.bp1 = 19923L * (int64_t)dev->cali.COE_bp1 + 1133836764L; // 31Q20
    dev->ik.b11 = 2406L * (int64_t)dev->cali.COE_b11+ 118215883L; // 28Q34
    dev->ik.bp2 = 3079L * (int64_t)dev->cali.COE_bp2 - 181579595L; // 29Q43
    dev->ik.b12 = 6846L * (int64_t)dev->cali.COE_b12 + 85590281L; // 29Q53
    dev->ik.b21 = 13836L * (int64_t)dev->cali.COE_b21 + 79333336L; // 29Q60
    dev->ik.bp3 = 2915L * (int64_t)dev->cali.COE_bp3 + 157155561L; // 28Q65

    return 1;
}

int16_t qmp6988_convTx02e(qmp6988_ik_data_t *ik, int32_t dt)
{
  int16_t ret;
  int64_t wk1, wk2;

  // wk1: 60Q4 // bit size
  wk1 = ((int64_t)ik->a1 * (int64_t)dt); // 31Q23+24-1=54 (54Q23)
  wk2 = ((int64_t)ik->a2 * (int64_t)dt) >> 14; // 30Q47+24-1=53 (39Q33)
  wk2 = (wk2 * (int64_t)dt) >> 10; // 39Q33+24-1=62 (52Q23)
  wk2 = ((wk1 + wk2) / 32767) >> 19; // 54,52->55Q23 (20Q04)
  ret = (int16_t)((ik->a0 + wk2) >> 4); // 21Q4 -> 17Q0
  return ret;
}

int32_t qmp6988_getPressure02e(qmp6988_ik_data_t *ik, int32_t dp, int16_t tx)
{
  int32_t ret;
  int64_t wk1, wk2, wk3;

  // wk1 = 48Q16 // bit size
  wk1 = ((int64_t)ik->bt1 * (int64_t)tx); // 28Q15+16-1=43 (43Q15)
  wk2 = ((int64_t)ik->bp1 * (int64_t)dp) >> 5; // 31Q20+24-1=54 (49Q15)
  wk1 += wk2; // 43,49->50Q15
  wk2 = ((int64_t)ik->bt2 * (int64_t)tx) >> 1; // 34Q38+16-1=49 (48Q37)
  wk2 = (wk2 * (int64_t)tx) >> 8; // 48Q37+16-1=63 (55Q29)
  wk3 = wk2; // 55Q29
  wk2 = ((int64_t)ik->b11 * (int64_t)tx) >> 4; // 28Q34+16-1=43 (39Q30)
  wk2 = (wk2 * (int64_t)dp) >> 1; // 39Q30+24-1=62 (61Q29)
  wk3 += wk2; // 55,61->62Q29
  wk2 = ((int64_t)ik->bp2 * (int64_t)dp) >> 13; // 29Q43+24-1=52 (39Q30)
  wk2 = (wk2 * (int64_t)dp) >> 1; // 39Q30+24-1=62 (61Q29)
  wk3 += wk2; // 62,61->63Q29
  wk1 += wk3 >> 14; // Q29 >> 14 -> Q15
  wk2 = ((int64_t)ik->b12 * (int64_t)tx); // 29Q53+16-1=45 (45Q53)
  wk2 = (wk2 * (int64_t)tx) >> 22; // 45Q53+16-1=61 (39Q31)
  wk2 = (wk2 * (int64_t)dp) >> 1; // 39Q31+24-1=62 (61Q30)
  wk3 = wk2; // 61Q30
  wk2 = ((int64_t)ik->b21 * (int64_t)tx) >> 6; // 29Q60+16-1=45 (39Q54)
  wk2 = (wk2 * (int64_t)dp) >> 23; // 39Q54+24-1=62 (39Q31)
  wk2 = (wk2 * (int64_t)dp) >> 1; // 39Q31+24-1=62 (61Q20)
  wk3 += wk2; // 61,61->62Q30
  wk2 = ((int64_t)ik->bp3 * (int64_t)dp) >> 12; // 28Q65+24-1=51 (39Q53)
  wk2 = (wk2 * (int64_t)dp) >> 23; // 39Q53+24-1=62 (39Q30)
  wk2 = (wk2 * (int64_t)dp); // 39Q30+24-1=62 (62Q30)
  wk3 += wk2; // 62,62->63Q30
  wk1 += wk3 >> 15; // Q30 >> 15 = Q15
  wk1 /= 32767L;
  wk1 >>= 11; // Q15 >> 7 = Q4
  wk1 += ik->b00; // Q4 + 20Q4
  //wk1 >>= 4; // 28Q4 -> 24Q0
  ret = (int32_t)wk1;
  return ret;
}

void qmp6988_setpPowermode(qmp6988_sensor_t* dev, uint8_t mode)
{
    uint8_t data;

    qmp6988_read_data(dev, QMP6988_CTRLMEAS_REG, &data, 1);
    data &= 0xfc;
    data |= mode;

    qmp6988_write_reg(dev, QMP6988_CTRLMEAS_REG, data);
    k_msleep(20);
}

void qmp6988_setFilter(qmp6988_sensor_t* dev, uint8_t filter)
{
    uint8_t data;

    data = (filter&0x03);
    qmp6988_write_reg(dev, QMP6988_CONFIG_REG, data);
    k_msleep(20);
}

void qmp6988_setOversamplingP(qmp6988_sensor_t* dev, uint8_t oversampling_p)
{
  uint8_t data;

  qmp6988_read_data(dev, QMP6988_CTRLMEAS_REG, &data, 1);
  data &= 0xe3;
  data |= (oversampling_p << 2);
  qmp6988_write_reg(dev, QMP6988_CTRLMEAS_REG, data);
  k_msleep(20);
}

void qmp6988_setOversamplingT(qmp6988_sensor_t* dev, uint8_t oversampling_t)
{
  uint8_t data;

  qmp6988_read_data(dev, QMP6988_CTRLMEAS_REG, &data, 1);
  data &= 0x1f;
  data |= (oversampling_t << 5);
  qmp6988_write_reg(dev, QMP6988_CTRLMEAS_REG, data);
  k_msleep(20);
}

static bool qmp6988_read_data(qmp6988_sensor_t* dev, uint8_t reg_addr, uint8_t *data,  uint32_t len)
{
    if (!dev) return false;

    int err = i2c_write_read(dev->bus, dev->addr, &reg_addr, 1, data, len);
    if (err)
    {
        return false;
    }

    return true;
}

static bool qmp6988_write_reg(qmp6988_sensor_t* dev, uint8_t reg_addr, uint8_t data)
{
    if (!dev) return false;

    int err = i2c_reg_write_byte(dev->bus, dev->addr, reg_addr, data);
    if (err)
    {
        return false;
    }

    return true;
}
