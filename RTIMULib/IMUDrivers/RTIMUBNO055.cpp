////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//  Based on the Adafruit BNO055 driver:

/***************************************************************************
  This is a library for the BNO055 orientation sensor
  Designed specifically to work with the Adafruit BNO055 Breakout.
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products
  These sensors use I2C to communicate, 2 pins are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by KTOWN for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ***************************************************************************/

#include "RTIMUBNO055.h"
#include "RTIMUSettings.h"

#define NO_CAIL // START_CAIL or NO_CAIL

#ifdef NO_CAIL
const unsigned char offset[NUM_BNO055_OFFSET_REGISTERS]={ //5 -12 -32 -729 -89 -553 -2 -1 0 1000 796

    5,0,   0xf4,0xff,  0xe0, 0xff, 0x27,0xfd,  0xa7, 0xff, 0xd7,0xfd,  0xfe,0xff,   0xff,0xff, 0x00, 0x00, 0xe8,0x03, 0x1c,0x03};
#endif

#ifdef START_CAIL

bool RTIMUBNO055::getSensorOffsets(bno055_offsets_t &offsets_type,unsigned char status) {
    unsigned char buffer[22];
    m_slaveAddr = m_settings->m_I2CSlaveAddress;
    if (status == 0xff) {
        if (!m_settings->HALWrite(m_slaveAddr, BNO055_OPER_MODE, BNO055_OPER_MODE_CONFIG, "Failed to set BNO055 into config mode"))
            return false;
        m_settings->delayMs(50);

        if (!m_settings->HALRead(m_slaveAddr, BNO055_ACCEL_OFFSET_X_LSB_ADDR, NUM_BNO055_OFFSET_REGISTERS, buffer, "Failed to read BNO055 Calib data"))
            return false;
        
        /* Accel offset range depends on the G-range:
        +/-2g  = +/- 2000 mg
        +/-4g  = +/- 4000 mg
        +/-8g  = +/- 8000 mg
        +/-1§g = +/- 16000 mg */
        offsets_type.accel_offset_x = (int16_t)(buffer[1] << 8 | buffer[0]);
        offsets_type.accel_offset_y = (int16_t)(buffer[3] << 8 | buffer[2]);
        offsets_type.accel_offset_z = (int16_t)(buffer[5] << 8 | buffer[4]);

        /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
        offsets_type.mag_offset_x = (int16_t)(buffer[7] << 8 | buffer[6]);
        offsets_type.mag_offset_y = (int16_t)(buffer[9] << 8 | buffer[8]);
        offsets_type.mag_offset_z = (int16_t)(buffer[11] << 8 | buffer[10]);

        /* Gyro offset range depends on the DPS range:
        2000 dps = +/- 32000 LSB
        1000 dps = +/- 16000 LSB
        500 dps = +/- 8000 LSB
        250 dps = +/- 4000 LSB
        125 dps = +/- 2000 LSB
        ... where 1 DPS = 16 LSB */
        offsets_type.gyro_offset_x =(int16_t)(buffer[13] << 8 | buffer[12]);
        offsets_type.gyro_offset_y =(int16_t)(buffer[15] << 8 | buffer[14]);
        offsets_type.gyro_offset_z =(int16_t)(buffer[17] << 8 | buffer[16]);

        /* Accelerometer radius = +/- 1000 LSB */
        offsets_type.accel_radius =(int16_t)(buffer[19] << 8 | buffer[18]);

        /* Magnetometer radius = +/- 960 LSB */
        offsets_type.mag_radius =(int16_t)(buffer[21] << 8 | buffer[20]);
        
        m_settings->delayMs(50);
        if (!m_settings->HALWrite(m_slaveAddr, BNO055_OPER_MODE, BNO055_OPER_MODE_NDOF, "Failed to set BNO055 into 9-dof mode"))
            return false;
        
        m_settings->delayMs(50);

        HAL_INFO1("ACC_OFFSET_X:%d\n",offsets_type.accel_offset_x);
        HAL_INFO1("ACC_OFFSET_Y:%d\n",offsets_type.accel_offset_y);
        HAL_INFO1("ACC_OFFSET_Z:%d\n",offsets_type.accel_offset_z);
        HAL_INFO1("MAG_OFFSET_X:%d\n",offsets_type.mag_offset_x);
        HAL_INFO1("MAG_OFFSET_Y:%d\n",offsets_type.mag_offset_y);
        HAL_INFO1("MAG_OFFSET_X:%d\n",offsets_type.mag_offset_z);
        HAL_INFO1("GYR_OFFSET_X:%d\n",offsets_type.gyro_offset_x);
        HAL_INFO1("GYR_OFFSET_Y:%d\n",offsets_type.gyro_offset_y);
        HAL_INFO1("GYR_OFFSET_Z:%d\n",offsets_type.gyro_offset_z);
        HAL_INFO1("ACC_RADIUS:%d\n",offsets_type.accel_radius);
        HAL_INFO1("MAG_RADIUS:%d\n",offsets_type.mag_radius);

        uint8_t dummy;
        for(dummy = 0; dummy < NUM_BNO055_OFFSET_REGISTERS;dummy++){
            HAL_INFO1("%x\n", buffer[dummy]);
        }
        return false;
    return true;
  }
  return false;
}

#endif


RTIMUBNO055::RTIMUBNO055(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
}

RTIMUBNO055::~RTIMUBNO055()
{
}

bool RTIMUBNO055::IMUInit()
{
    unsigned char result;


    m_slaveAddr = m_settings->m_I2CSlaveAddress;
    m_lastReadTime = RTMath::currentUSecsSinceEpoch();

    // set validity flags

    m_imuData.fusionPoseValid = true;
    m_imuData.fusionQPoseValid = true;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    if (!m_settings->HALRead(m_slaveAddr, BNO055_WHO_AM_I, 1, &result, "Failed to read BNO055 id"))
        return false;

    if (result != BNO055_ID) {
        HAL_ERROR1("Incorrect IMU id %d", result);
        return false;
    }

    if (!m_settings->HALWrite(m_slaveAddr, BNO055_OPER_MODE, BNO055_OPER_MODE_CONFIG, "Failed to set BNO055 into config mode"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, BNO055_SYS_TRIGGER, 0x20, "Failed to reset BNO055"))
        return false;

    m_settings->delayMs(50);

    while (1) {
        if (!m_settings->HALRead(m_slaveAddr, BNO055_WHO_AM_I, 1, &result, ""))
            continue;
        if (result == BNO055_ID)
            break;
        m_settings->delayMs(50);
    }

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, BNO055_PWR_MODE, BNO055_PWR_MODE_NORMAL, "Failed to set BNO055 normal power mode"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, BNO055_PAGE_ID, 0, "Failed to set BNO055 page 0"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, BNO055_SYS_TRIGGER, 0x00, "Failed to start BNO055"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->HALWrite(m_slaveAddr, BNO055_UNIT_SEL, 0x87, "Failed to set BNO055 units"))
        return false;

    m_settings->delayMs(50);
    //I²C write access can be used to write a data  byte  in  one sequence

#ifdef NO_CAIL

    uint8_t dummy = 0;
    for (dummy = 0 ; dummy < NUM_BNO055_OFFSET_REGISTERS; dummy++){
        if (!m_settings->HALWrite(m_slaveAddr, BNO055_ACCEL_OFFSET_X_LSB_ADDR+dummy,offset[dummy], "Failed to set BNO055 Calibration Offset"))
            return false;
        m_settings->delayMs(50);
    }

#endif


    if (!m_settings->HALWrite(m_slaveAddr, BNO055_OPER_MODE, BNO055_OPER_MODE_NDOF, "Failed to set BNO055 into 9-dof FMC-OFF mode"))
        return false;

    m_settings->delayMs(50);

    HAL_INFO("BNO055 init complete\n");
    return true;
}

int RTIMUBNO055::IMUGetPollInterval()
{
    return (7);
}

bool RTIMUBNO055::IMURead()
{
    unsigned char buffer[24];

#ifdef START_CAIL
    unsigned char result;
    bno055_offsets_t offsets_type;
#endif

    if ((RTMath::currentUSecsSinceEpoch() - m_lastReadTime) < m_sampleInterval)
        return false;                                       // too soon

    m_lastReadTime = RTMath::currentUSecsSinceEpoch();
    if (!m_settings->HALRead(m_slaveAddr, BNO055_ACCEL_DATA, 24, buffer, "Failed to read BNO055 data"))
        return false;

    int16_t x, y, z;

    // process accel data

    x = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    y = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    z = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);

    m_imuData.accel.setX((RTFLOAT)y / 1000.0);
    m_imuData.accel.setY((RTFLOAT)x / 1000.0);
    m_imuData.accel.setZ((RTFLOAT)z / 1000.0);

    // process mag data

    x = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);
    y = (((uint16_t)buffer[9]) << 8) | ((uint16_t)buffer[8]);
    z = (((uint16_t)buffer[11]) << 8) | ((uint16_t)buffer[10]);

    m_imuData.compass.setX(-(RTFLOAT)y / 16.0);
    m_imuData.compass.setY(-(RTFLOAT)x / 16.0);
    m_imuData.compass.setZ(-(RTFLOAT)z / 16.0);

    // process gyro data

    x = (((uint16_t)buffer[13]) << 8) | ((uint16_t)buffer[12]);
    y = (((uint16_t)buffer[15]) << 8) | ((uint16_t)buffer[14]);
    z = (((uint16_t)buffer[17]) << 8) | ((uint16_t)buffer[16]);

    m_imuData.gyro.setX(-(RTFLOAT)y / 900.0);
    m_imuData.gyro.setY(-(RTFLOAT)x / 900.0);
    m_imuData.gyro.setZ(-(RTFLOAT)z / 900.0);

    // process euler angles

    x = (((uint16_t)buffer[19]) << 8) | ((uint16_t)buffer[18]);
    y = (((uint16_t)buffer[21]) << 8) | ((uint16_t)buffer[20]);
    z = (((uint16_t)buffer[23]) << 8) | ((uint16_t)buffer[22]);

    //  put in structure and do axis remap

    m_imuData.fusionPose.setX((RTFLOAT)y / 900.0);
    m_imuData.fusionPose.setY((RTFLOAT)z / 900.0);
    m_imuData.fusionPose.setZ((RTFLOAT)x / 900.0);

    m_imuData.fusionQPose.fromEuler(m_imuData.fusionPose);

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

#ifdef START_CAIL
    static bool Flag = true;
    if (m_settings->HALRead(m_slaveAddr, BNO055_CALIB_STAT, 1, &result, "BNO055 calibration status") && Flag){
        HAL_INFO("BNO055 calibration status:\t");
        switch(result & 0x03){
            case 0x01:
                HAL_INFO("MAG:1\t");
                break;
            case 0x02:
                HAL_INFO("MAG:2\t");
                break;
            case 0x03:
                HAL_INFO("MAG:3\t");
                break;
            default:
                HAL_INFO("MAG:0\t");
        }
        switch(result & 0x0C){
            case 0x04:
                HAL_INFO("ACC:1\t");
                break;
            case 0x08:
                HAL_INFO("ACC:2\t");
                break;
            case 0x0C:
                HAL_INFO("ACC:3\t");
                break;
            default:
                HAL_INFO("ACC:0\t");
        }
        switch(result & 0x30){
            case 0x10:
                HAL_INFO("GYR:1\t");
                break;
            case 0x20:
                HAL_INFO("GYR:2\t");
                break;
            case 0x30:
                HAL_INFO("GYR:3\t");
                break;
            default:
                HAL_INFO("GYR:0\t");
        }
        switch(result & 0xC0){
            case 0x40:
                HAL_INFO("SYS:1\n");
                break;
            case 0x80:
                HAL_INFO("SYS:2\n");
                break;
            case 0xC0:
                HAL_INFO("SYS:3\n");
                break;
            default:
                HAL_INFO("SYS:0\n");
        }
        
        if(getSensorOffsets(offsets_type,result))
            Flag = false;
    }

#endif
    return true;
}
