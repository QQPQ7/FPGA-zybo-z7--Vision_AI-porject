////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021 Mateusz Malinowski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include <iostream>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <cmath>
#include <algorithm>
#include "madgwick_ahrs.h"
#include "simple_ahrs.h"
#include "imu_data.h"


extern std::atomic<bool> g_imu_fault;
// I2C 클래스 (icm20948.hpp에서 가져옴)
class I2C {
public:
    I2C() : mSerial(-1), mCurrentDevice(0) {}
    virtual ~I2C() { closeSerialPort(); }
    bool openSerialPort(const char* device) {
        closeSerialPort();
        mSerial = open(device, O_RDWR);
        return mSerial >= 0;
    }
    void closeSerialPort() {
        if (mSerial >= 0) {
            close(mSerial);
            mSerial = -1;
            mCurrentDevice = 0;
        }
    }
    uint8_t readByte(const uint8_t deviceAddr, const uint8_t regAddr) const {
        uint8_t u8Ret = 0;
        if (connectToDevice(deviceAddr)) {
            if (1 == write(mSerial, &regAddr, 1)) {
                if (1 != read(mSerial, &u8Ret, 1)) {
                    syslog(LOG_ERR, "Failed to read data from device %02x, address: %02x.", deviceAddr, regAddr);
                    u8Ret = 0;
                }
            } else {
                syslog(LOG_ERR, "Failed to write request to device: %02x, address: %02x in order to obtain data.", deviceAddr, regAddr);
            }
        }
        return u8Ret;
    }
    void readNBytes(const uint8_t deviceAddr, const uint8_t regAddr, const uint8_t length, uint8_t data[]) const {
        auto t0 = std::chrono::steady_clock::now();
        if (connectToDevice(deviceAddr)) {
            if (1 == write(mSerial, &regAddr, 1)) {
                if (length != read(mSerial, data, length)) {
                    syslog(LOG_ERR, "Failed to read data from device %02x, address: %02x.", deviceAddr, regAddr);
                    memset(data, 0, length * sizeof(uint8_t));
                }
            } else {
                syslog(LOG_ERR, "Failed to write request to device: %02x, address: %02x in order to obtain data.", deviceAddr, regAddr);
            }
        }
        auto t1 = std::chrono::steady_clock::now();
        double dt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (dt_ms >500.0) {
            std::fprintf(stderr, "[IMU] readNBytes took %.3f ms\n", dt_ms);
            bool was_fault = g_imu_fault.exchange(true);
            if (!was_fault) {
                syslog(LOG_ERR,
                    "[IMU] I2C read timeout (%.3f ms) → entering IMU_FAILSAFE mode.",
                    dt_ms);
            }
        }
    }
    void writeByte(const uint8_t deviceAddr, const uint8_t regAddr, const uint8_t value) const {
        uint8_t buf[2] = {regAddr, value};
        if (connectToDevice(deviceAddr)) {
            if (2 != write(mSerial, buf, 2)) {
                syslog(LOG_ERR, "Failed to write %02x to device: %02x, address: %02x.", value, deviceAddr, regAddr);
            }
        }
    }
    void writeData(const uint8_t deviceAddr, const uint8_t length, const uint8_t data[]) const {
        if (connectToDevice(deviceAddr)) {
            if (length != write(mSerial, data, length)) {
                syslog(LOG_ERR, "Failed to write %d bytes to device: %02x, address: %02x.", length, deviceAddr, data[0]);
            }
        }
    }
private:
    bool connectToDevice(const uint8_t deviceAddr) const {
        if (deviceAddr != mCurrentDevice) {
            if (ioctl(mSerial, I2C_SLAVE, deviceAddr) >= 0) {
                mCurrentDevice = deviceAddr;
            } else {
                syslog(LOG_ERR, "Failed to connect to device: %02x.", deviceAddr);
                return false;
            }
        }
        return true;
    }
    int mSerial;
    mutable uint8_t mCurrentDevice;
};

// ICM20948 클래스 (icm20948.hpp에서 가져옴)
class ICM20948 {
public:
    enum ACC_RANGE {
        ACC_RANGE_2G = 0,
        ACC_RANGE_4G = 1,
        ACC_RANGE_8G = 2,
        ACC_RANGE_16G = 3
    };
    enum ACC_DLPF_BANDWIDTH {
        ACC_DLPF_NONE = 0b00000000,
        ACC_DLPF_BANDWIDTH_246HZ = 0b00001001,
        ACC_DLPF_BANDWIDTH_111HZ = 0b00010001,
        ACC_DLPF_BANDWIDTH_50HZ = 0b00011001,
        ACC_DLPF_BANDWIDTH_24HZ = 0b00100001,
        ACC_DLPF_BANDWIDTH_12HZ = 0b00101001,
        ACC_DLPF_BANDWIDTH_6HZ = 0b00110001,
        ACC_DLPF_BANDWIDTH_473HZ = 0b00111001
    };
    enum ACC_AVERAGING {
        ACC_AVERAGING_NONE = -1,
        ACC_AVERAGING_4X = 0,
        ACC_AVERAGING_8X = 1,
        ACC_AVERAGING_16X = 2,
        ACC_AVERAGING_32X = 3
    };
    struct AccConfig {
        bool mEnabled;
        ACC_RANGE mRange;
        ACC_DLPF_BANDWIDTH mDLPFBandwidth;
        ACC_AVERAGING mAveraging;
        uint16_t mSampleRateDivisor;
        AccConfig() : mEnabled(true), mRange(ACC_RANGE_2G), mDLPFBandwidth(ACC_DLPF_BANDWIDTH_6HZ),
                      mAveraging(ACC_AVERAGING_4X), mSampleRateDivisor(4) {}
    };
    enum GYRO_RANGE {
        GYRO_RANGE_250DPS = 0,
        GYRO_RANGE_500DPS = 1,
        GYRO_RANGE_1000DPS = 2,
        GYRO_RANGE_2000DPS = 3
    };
    enum GYRO_RANGE_DLPF_BANDWIDTH {
        GYRO_DLPF_NONE = 0b00000000,
        GYRO_DLPF_BANDWIDTH_197HZ = 0b00000001,
        GYRO_DLPF_BANDWIDTH_152HZ = 0b00001001,
        GYRO_DLPF_BANDWIDTH_120HZ = 0b00010001,
        GYRO_DLPF_BANDWIDTH_51HZ = 0b00011001,
        GYRO_DLPF_BANDWIDTH_24HZ = 0b00100001,
        GYRO_DLPF_BANDWIDTH_12HZ = 0b00101001,
        GYRO_DLPF_BANDWIDTH_6HZ = 0b00110001,
        GYRO_DLPF_BANDWIDTH_361HZ = 0b00111001
    };
    enum GYRO_AVERAGING {
        GYRO_AVERAGING_NONE = -1,
        GYRO_AVERAGING_1X = 0,
        GYRO_AVERAGING_2X = 1,
        GYRO_AVERAGING_4X = 2,
        GYRO_AVERAGING_8X = 3,
        GYRO_AVERAGING_16X = 4,
        GYRO_AVERAGING_32X = 5,
        GYRO_AVERAGING_64X = 6,
        GYRO_AVERAGING_128X = 7
    };
    struct GyroConfig {
        bool mEnabled;
        GYRO_RANGE mRange;
        GYRO_RANGE_DLPF_BANDWIDTH mDLPFBandwidth;
        GYRO_AVERAGING mAveraging;
        uint8_t mSampleRateDivisor;
        GyroConfig() : mEnabled(true), mRange(GYRO_RANGE_250DPS), mDLPFBandwidth(GYRO_DLPF_BANDWIDTH_6HZ),
                       mAveraging(GYRO_AVERAGING_4X), mSampleRateDivisor(4) {}
    };
    enum TEMP_DLPF_BANDWIDTH {
        TEMP_DLPF_NONE = 0,
        TEMP_DLPF_BANDWIDTH_218HZ = 1,
        TEMP_DLPF_BANDWIDTH_124HZ = 2,
        TEMP_DLPF_BANDWIDTH_66HZ = 3,
        TEMP_DLPF_BANDWIDTH_34HZ = 4,
        TEMP_DLPF_BANDWIDTH_17HZ = 5,
        TEMP_DLPF_BANDWIDTH_9HZ = 6
    };
    struct TempConfig {
        bool mEnabled;
        TEMP_DLPF_BANDWIDTH mDLPFBandwidth;
        TempConfig() : mEnabled(true), mDLPFBandwidth(TEMP_DLPF_BANDWIDTH_9HZ) {}
    };
    enum AHRS_ALGORITHM {
        NONE = 0,
        SIMPLE = 1,
        MADGWICK = 2
    };
    struct Config {
        char mDevice[1024];
        GyroConfig mGyro;
        AccConfig mAcc;
        TempConfig mTemp;
        bool mMagEnabled;
        AHRS_ALGORITHM mAHRS;
        float mFramerate;
        Config() : mDevice(), mGyro(), mAcc(), mTemp(), mMagEnabled(true), mAHRS(MADGWICK), mFramerate(100.0f) {}
    };

    ICM20948() : mCurrentBank(BANK_UNDEFINED), mConfig(), mGyroScale(0.0f), mAccScale(0.0f) {}
    virtual ~ICM20948() {}
    bool initialise(const Config& config = Config()) {
        uint8_t deviceID = 0;
        if (strcmp(config.mDevice, mConfig.mDevice) != 0) {
            mI2C.closeSerialPort();
        }
        mConfig = config;
        mData.mUpdatePeriod = 1.0f / mConfig.mFramerate;
        if (mI2C.openSerialPort(mConfig.mDevice)) {
            setBank(BANK_0);
            deviceID = mI2C.readByte(I2C_ADD_ICM20948, REG_ADD_WIA);
            if (REG_VAL_WIA == deviceID) {
                reset();
                setBank(BANK_3);
                mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_MST_CTRL, 0);
                configureMasterI2C();
                if (mConfig.mTemp.mEnabled) {
                    configureTemp();
                }
                if (mConfig.mGyro.mEnabled) {
                    configureGyro();
                }
                if (mConfig.mAcc.mEnabled) {
                    configureAcc();
                }
                if (mConfig.mMagEnabled) {
                    mConfig.mMagEnabled = configureMag();
                }
            }
#ifdef LOG
            else {
                printf("Connected device is not ICM-20948! ID: %d \n", deviceID);
            }
#endif
        }
#ifdef LOG
        else {
            printf("Failed to open device: %s \n", mConfig.mDevice);
        }
#endif
        return (REG_VAL_WIA == deviceID);
    }
    constexpr const Config& getConfig() const { return mConfig; }
    const IMUData& imuDataGet() {
        bool magEnabled = mConfig.mMagEnabled;
        int16_t s16Gyro[3], s16Accel[3], s16Magn[3];
        int16_t temperature;
        magEnabled &= readAllRawDAta(s16Gyro, s16Accel, s16Magn, temperature);
        mData.mGyro[0] = static_cast<float>(s16Gyro[0]) * mGyroScale * DEG_TO_RAD;
        mData.mGyro[1] = static_cast<float>(s16Gyro[1]) * mGyroScale * DEG_TO_RAD;
        mData.mGyro[2] = static_cast<float>(s16Gyro[2]) * mGyroScale * DEG_TO_RAD;
        mData.mAcc[0] = static_cast<float>(s16Accel[0]) * mAccScale;
        mData.mAcc[1] = static_cast<float>(s16Accel[1]) * mAccScale;
        mData.mAcc[2] = static_cast<float>(s16Accel[2]) * mAccScale;
        mData.mMag[0] = static_cast<float>(s16Magn[0]) * MAG_SCALE;
        mData.mMag[1] = static_cast<float>(s16Magn[1]) * MAG_SCALE;
        mData.mMag[2] = static_cast<float>(s16Magn[2]) * MAG_SCALE;
        mData.mTemp = (static_cast<float>(temperature - 21) / 333.87f) + 21.0f;
        switch (mConfig.mAHRS) {
            case NONE:
                break;
            case SIMPLE:
                SimpleAHRSupdate(mData);
                break;
            case MADGWICK:
            default:
                if (magEnabled) {
                    MadgwickAHRSupdate(mData);
                } else {
                    MadgwickAHRSupdateIMU(mData);
                }
                break;
        }
        return mData;
    }
    void calibrateGyro() const {
        int16_t i;
        int16_t s16G[3];
        int32_t s32G[3] = {0, 0, 0};
#ifdef LOG
        puts("Calibrating IMU offsets, please wait approximately 11 seconds.");
#endif
        setBank(BANK_2);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_XG_OFFS_USRH, 0);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_XG_OFFS_USRL, 0);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_YG_OFFS_USRH, 0);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_YG_OFFS_USRL, 0);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ZG_OFFS_USRH, 0);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ZG_OFFS_USRL, 0);
        for (i = 0; i < 1024; ++i) {
            readRawGyro(s16G);
            s32G[0] += static_cast<int32_t>(s16G[0]);
            s32G[1] += static_cast<int32_t>(s16G[1]);
            s32G[2] += static_cast<int32_t>(s16G[2]);
            sleepMS(10);
        }
        s16G[0] = -static_cast<int16_t>(s32G[0] >> (12 - mConfig.mGyro.mRange));
        s16G[1] = -static_cast<int16_t>(s32G[1] >> (12 - mConfig.mGyro.mRange));
        s16G[2] = -static_cast<int16_t>(s32G[2] >> (12 - mConfig.mGyro.mRange));
        setBank(BANK_2);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_XG_OFFS_USRH, static_cast<uint8_t>((s16G[0] >> 8) & 0xFF));
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_XG_OFFS_USRL, static_cast<uint8_t>(s16G[0] & 0xFF));
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_YG_OFFS_USRH, static_cast<uint8_t>((s16G[1] >> 8) & 0xFF));
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_YG_OFFS_USRL, static_cast<uint8_t>(s16G[1] & 0xFF));
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ZG_OFFS_USRH, static_cast<uint8_t>((s16G[2] >> 8) & 0xFF));
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ZG_OFFS_USRL, static_cast<uint8_t>(s16G[2] & 0xFF));
    }
private:
    enum ICM_BANK {
        BANK_0 = 0x00,
        BANK_1 = 0x10,
        BANK_2 = 0x20,
        BANK_3 = 0x30,
        BANK_UNDEFINED = 0xFF
    };
    static constexpr uint8_t I2C_ADD_ICM20948 = 0x68;
    static constexpr uint8_t I2C_ADD_ICM20948_AK09916 = 0x0C;
    static constexpr uint8_t I2C_ADD_ICM20948_AK09916_READ = 0x80;
    static constexpr uint8_t I2C_ADD_ICM20948_AK09916_WRITE = 0x00;
    static constexpr uint8_t REG_ADD_WIA = 0x00;
    static constexpr uint8_t REG_VAL_WIA = 0xEA;
    static constexpr uint8_t REG_ADD_USER_CTRL = 0x03;
    static constexpr uint8_t REG_VAL_BIT_DMP_EN = 0x80;
    static constexpr uint8_t REG_VAL_BIT_FIFO_EN = 0x40;
    static constexpr uint8_t REG_VAL_BIT_I2C_MST_EN = 0x20;
    static constexpr uint8_t REG_VAL_BIT_I2C_IF_DIS = 0x10;
    static constexpr uint8_t REG_VAL_BIT_DMP_RST = 0x08;
    static constexpr uint8_t REG_VAL_BIT_DIAMOND_DMP_RST = 0x04;
    static constexpr uint8_t REG_ADD_PWR_MIGMT_1 = 0x06;
    static constexpr uint8_t REG_VAL_ALL_RGE_RESET = 0x80;
    static constexpr uint8_t REG_VAL_RUN_MODE = 0x01;
    static constexpr uint8_t REG_ADD_LP_CONFIG = 0x05;
    static constexpr uint8_t REG_ADD_PWR_MGMT_1 = 0x06;
    static constexpr uint8_t REG_ADD_PWR_MGMT_2 = 0x07;
    static constexpr uint8_t REG_VAL_SENSORS_ON = 0x00;
    static constexpr uint8_t REG_VAL_DISABLE_GYRO = 0x07;
    static constexpr uint8_t REG_VAL_DISABLE_ACC = 0x38;
    static constexpr uint8_t REG_ADD_ACCEL_XOUT_H = 0x2D;
    static constexpr uint8_t REG_ADD_ACCEL_XOUT_L = 0x2E;
    static constexpr uint8_t REG_ADD_ACCEL_YOUT_H = 0x2F;
    static constexpr uint8_t REG_ADD_ACCEL_YOUT_L = 0x30;
    static constexpr uint8_t REG_ADD_ACCEL_ZOUT_H = 0x31;
    static constexpr uint8_t REG_ADD_ACCEL_ZOUT_L = 0x32;
    static constexpr uint8_t REG_ADD_GYRO_XOUT_H = 0x33;
    static constexpr uint8_t REG_ADD_GYRO_XOUT_L = 0x34;
    static constexpr uint8_t REG_ADD_GYRO_YOUT_H = 0x35;
    static constexpr uint8_t REG_ADD_GYRO_YOUT_L = 0x36;
    static constexpr uint8_t REG_ADD_GYRO_ZOUT_H = 0x37;
    static constexpr uint8_t REG_ADD_GYRO_ZOUT_L = 0x38;
    static constexpr uint8_t REG_ADD_TEMP_OUT_H = 0x39;
    static constexpr uint8_t REG_ADD_TEMP_OUT_L = 0x3A;
    static constexpr uint8_t REG_ADD_EXT_SENS_DATA_00 = 0x3B;
    static constexpr uint8_t REG_ADD_FIFO_EN_1 = 0x66;
    static constexpr uint8_t REG_ADD_FIFO_EN_2 = 0x67;
    static constexpr uint8_t REG_ADD_FIFO_RST = 0x68;
    static constexpr uint8_t REG_ADD_FIFO_MODE = 0x68;
    static constexpr uint8_t REG_ADD_FIFO_COUNTH = 0x70;
    static constexpr uint8_t REG_ADD_FIFO_COUNTL = 0x71;
    static constexpr uint8_t REG_ADD_FIFO_R_W = 0x72;
    static constexpr uint8_t REG_ADD_REG_BANK_SEL = 0x7F;
    static constexpr uint8_t REG_VAL_REG_BANK_0 = 0x00;
    static constexpr uint8_t REG_VAL_REG_BANK_1 = 0x10;
    static constexpr uint8_t REG_VAL_REG_BANK_2 = 0x20;
    static constexpr uint8_t REG_VAL_REG_BANK_3 = 0x30;
    static constexpr uint8_t REG_ADD_GYRO_SMPLRT_DIV = 0x00;
    static constexpr uint8_t REG_ADD_GYRO_CONFIG_1 = 0x01;
    static constexpr uint8_t REG_ADD_GYRO_CONFIG_2 = 0x02;
    static constexpr uint8_t REG_ADD_XG_OFFS_USRH = 0x03;
    static constexpr uint8_t REG_ADD_XG_OFFS_USRL = 0x04;
    static constexpr uint8_t REG_ADD_YG_OFFS_USRH = 0x05;
    static constexpr uint8_t REG_ADD_YG_OFFS_USRL = 0x06;
    static constexpr uint8_t REG_ADD_ZG_OFFS_USRH = 0x07;
    static constexpr uint8_t REG_ADD_ZG_OFFS_USRL = 0x08;
    static constexpr uint8_t REG_ADD_XA_OFFS_H = 0x14;
    static constexpr uint8_t REG_ADD_XA_OFFS_L = 0x15;
    static constexpr uint8_t REG_ADD_YA_OFFS_H = 0x17;
    static constexpr uint8_t REG_ADD_YA_OFFS_L = 0x18;
    static constexpr uint8_t REG_ADD_ZA_OFFS_H = 0x1A;
    static constexpr uint8_t REG_ADD_ZA_OFFS_L = 0x1B;
    static constexpr uint8_t REG_VAL_BIT_GYRO_DLPCFG_2 = 0x10;
    static constexpr uint8_t REG_VAL_BIT_GYRO_DLPCFG_4 = 0x20;
    static constexpr uint8_t REG_VAL_BIT_GYRO_DLPCFG_6 = 0x30;
    static constexpr uint8_t REG_VAL_BIT_GYRO_FS_250DPS = 0x00;
    static constexpr uint8_t REG_VAL_BIT_GYRO_FS_500DPS = 0x02;
    static constexpr uint8_t REG_VAL_BIT_GYRO_FS_1000DPS = 0x04;
    static constexpr uint8_t REG_VAL_BIT_GYRO_FS_2000DPS = 0x06;
    static constexpr uint8_t REG_VAL_BIT_GYRO_DLPF = 0x01;
    static constexpr uint8_t REG_ADD_ACCEL_SMPLRT_DIV_1 = 0x10;
    static constexpr uint8_t REG_ADD_ACCEL_SMPLRT_DIV_2 = 0x11;
    static constexpr uint8_t REG_ADD_ACCEL_CONFIG = 0x14;
    static constexpr uint8_t REG_ADD_ACCEL_CONFIG_2 = 0x15;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_DLPCFG_2 = 0x10;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_DLPCFG_4 = 0x20;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_DLPCFG_6 = 0x30;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_FS_2g = 0x00;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_FS_4g = 0x02;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_FS_8g = 0x04;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_FS_16g = 0x06;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_DLPF = 0x01;
    static constexpr uint8_t REG_ADD_TEMP_CONFIG = 0x53;
    static constexpr uint8_t REG_ADD_I2C_MST_CTRL = 0x01;
    static constexpr uint8_t REG_VAL_I2C_MST_CTRL_CLK_400KHZ = 0x07;
    static constexpr uint8_t REG_ADD_I2C_SLV0_ADDR = 0x03;
    static constexpr uint8_t REG_ADD_I2C_SLV0_REG = 0x04;
    static constexpr uint8_t REG_ADD_I2C_SLV0_CTRL = 0x05;
    static constexpr uint8_t REG_VAL_BIT_SLV0_EN = 0x80;
    static constexpr uint8_t REG_VAL_BIT_MASK_LEN = 0x07;
    static constexpr uint8_t REG_ADD_I2C_SLV0_DO = 0x06;
    static constexpr uint8_t REG_ADD_I2C_SLV1_ADDR = 0x07;
    static constexpr uint8_t REG_ADD_I2C_SLV1_REG = 0x08;
    static constexpr uint8_t REG_ADD_I2C_SLV1_CTRL = 0x09;
    static constexpr uint8_t REG_ADD_I2C_SLV1_DO = 0x0A;
    static constexpr uint8_t REG_ADD_MAG_WIA1 = 0x00;
    static constexpr uint8_t REG_VAL_MAG_WIA1 = 0x48;
    static constexpr uint8_t REG_ADD_MAG_WIA2 = 0x01;
    static constexpr uint8_t REG_VAL_MAG_WIA2 = 0x09;
    static constexpr uint8_t REG_ADD_MAG_ST1 = 0x10;
    static constexpr uint8_t REG_ADD_MAG_DATA = 0x11;
    static constexpr uint8_t REG_ADD_MAG_CNTL2 = 0x31;
    static constexpr uint8_t REG_VAL_MAG_MODE_PD = 0x00;
    static constexpr uint8_t REG_VAL_MAG_MODE_SM = 0x01;
    static constexpr uint8_t REG_VAL_MAG_MODE_10HZ = 0x02;
    static constexpr uint8_t REG_VAL_MAG_MODE_20HZ = 0x04;
    static constexpr uint8_t REG_VAL_MAG_MODE_50HZ = 0x05;
    static constexpr uint8_t REG_VAL_MAG_MODE_100HZ = 0x08;
    static constexpr uint8_t REG_VAL_MAG_MODE_ST = 0x10;
    static constexpr uint8_t REG_ADD_MAG_CNTL3 = 0x32;
    static constexpr uint8_t REG_VAL_MAG_RESET = 0x01;
    static constexpr short IMU_DATA_LEN = 22;
    static constexpr short GYRO_AND_ACC_DATA_LEN = 6;
    static constexpr short MAG_DATA_LEN = 8;
    static constexpr float MAG_SCALE = 0.15f;
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;

    void setBank(const ICM_BANK bank) const {
        if (bank != mCurrentBank && bank != BANK_UNDEFINED) {
            mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, bank);
            mCurrentBank = bank;
        }
    }
    void reset() const {
        uint8_t sensorsFlag = REG_VAL_SENSORS_ON;
        magI2CWrite(REG_ADD_MAG_CNTL2, 0x00);
        sleepMS(100);
        setBank(BANK_0);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_PWR_MGMT_1, REG_VAL_ALL_RGE_RESET);
        sleepMS(10);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_PWR_MGMT_1, REG_VAL_RUN_MODE | (static_cast<uint8_t>(!mConfig.mTemp.mEnabled) << 3));
        if (!mConfig.mGyro.mEnabled) {
            sensorsFlag |= REG_VAL_DISABLE_GYRO;
        }
        if (!mConfig.mAcc.mEnabled) {
            sensorsFlag |= REG_VAL_DISABLE_ACC;
        }
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_PWR_MGMT_2, sensorsFlag);
        sleepMS(10);
        magI2CWrite(REG_ADD_MAG_CNTL3, REG_VAL_MAG_RESET);
        sleepMS(100);
    }
    void configureGyro() {
        uint8_t sampleRateDivisor = mConfig.mGyro.mSampleRateDivisor;
        mGyroScale = static_cast<float>((mConfig.mGyro.mRange + 1) * 250) / 32768;
        switch (mConfig.mGyro.mAveraging) {
            case GYRO_AVERAGING_1X:
                sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor, 1);
                break;
            case GYRO_AVERAGING_2X:
                sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor, 2);
                break;
            case GYRO_AVERAGING_4X:
                sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor, 3);
                break;
            case GYRO_AVERAGING_8X:
                sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor, 5);
                break;
            case GYRO_AVERAGING_16X:
                sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor, 10);
                break;
            case GYRO_AVERAGING_32X:
                sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor, 22);
                break;
            case GYRO_AVERAGING_64X:
                sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor, 63);
                break;
            case GYRO_AVERAGING_128X:
                sampleRateDivisor = std::max<uint8_t>(mConfig.mGyro.mSampleRateDivisor, 255);
                break;
            case GYRO_AVERAGING_NONE:
                break;
            default:
#ifdef LOG
                printf("configureGyro:: this enum shouldn't be processed: %d !!! \n", static_cast<int>(mConfig.mGyro.mAveraging));
#endif
                break;
        }
        setBank(BANK_2);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_GYRO_SMPLRT_DIV, sampleRateDivisor & 0xFF);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_1, mConfig.mGyro.mDLPFBandwidth | (mConfig.mGyro.mRange << 1));
        if (mConfig.mGyro.mAveraging > GYRO_AVERAGING_NONE) {
            mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_2, mConfig.mGyro.mAveraging);
        }
    }
    void configureAcc() {
        ACC_DLPF_BANDWIDTH bandwidth = mConfig.mAcc.mDLPFBandwidth;
        uint16_t sampleRateDivisor = mConfig.mAcc.mSampleRateDivisor;
        mAccScale = powf(2.0, mConfig.mAcc.mRange + 1) / 32768;
        if (ACC_AVERAGING_NONE < mConfig.mAcc.mAveraging) {
            bandwidth = ACC_DLPF_BANDWIDTH_473HZ;
            switch (mConfig.mAcc.mAveraging) {
                case ACC_AVERAGING_4X:
                    sampleRateDivisor = std::max<uint16_t>(mConfig.mAcc.mSampleRateDivisor, 3);
                    break;
                case ACC_AVERAGING_8X:
                    sampleRateDivisor = std::max<uint16_t>(mConfig.mAcc.mSampleRateDivisor, 5);
                    break;
                case ACC_AVERAGING_16X:
                    sampleRateDivisor = std::max<uint16_t>(mConfig.mAcc.mSampleRateDivisor, 7);
                    break;
                case ACC_AVERAGING_32X:
                    sampleRateDivisor = std::max<uint16_t>(mConfig.mAcc.mSampleRateDivisor, 10);
                    break;
                case ACC_AVERAGING_NONE:
                    break;
                default:
#ifdef LOG
                    printf("configureAcc:: this enum shouldn't be processed: %d !!! \n", static_cast<int>(mConfig.mAcc.mAveraging));
#endif
                    break;
            }
        }
        setBank(BANK_2);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_1, static_cast<uint8_t>(sampleRateDivisor >> 8) & 0x0F);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_2, static_cast<uint8_t>(sampleRateDivisor) & 0xFF);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG, bandwidth | (mConfig.mAcc.mRange << 1));
        if (ACC_AVERAGING_NONE < mConfig.mAcc.mAveraging) {
            mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG_2, mConfig.mAcc.mAveraging);
        }
    }
    bool configureMag() const {
        uint8_t u8Data[MAG_DATA_LEN];
        int counter = 0;
        bool flag = checkMag();
        while (!flag && (++counter <= 10)) {
            sleepMS(100);
            flag = checkMag();
        }
        if (flag) {
#ifdef LOG
            puts("Found Magnetometer!");
#endif
            sleepMS(1000);
            magI2CWrite(REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_100HZ);
            sleepMS(10);
            magI2CRead(REG_ADD_MAG_DATA, MAG_DATA_LEN, u8Data);
        }
#ifdef LOG
        else {
            puts("Failed to detect the magnetometer!");
        }
#endif
        return flag;
    }
    void configureTemp() const {
        setBank(BANK_2);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_TEMP_CONFIG, mConfig.mTemp.mDLPFBandwidth);
    }
    void configureMasterI2C() const {
        uint8_t temp;
        setBank(BANK_0);
        temp = mI2C.readByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, temp | REG_VAL_BIT_I2C_MST_EN);
        setBank(BANK_3);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_MST_CTRL, REG_VAL_I2C_MST_CTRL_CLK_400KHZ);
        sleepMS(10);
    }
    void readRawGyro(int16_t gyro[3]) const {
        uint8_t u8Buf[GYRO_AND_ACC_DATA_LEN];
        setBank(BANK_0);
        mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_GYRO_XOUT_H, GYRO_AND_ACC_DATA_LEN, u8Buf);
        gyro[0] = static_cast<int16_t>((u8Buf[0] << 8) | u8Buf[1]);
        gyro[1] = static_cast<int16_t>((u8Buf[2] << 8) | u8Buf[3]);
        gyro[2] = static_cast<int16_t>((u8Buf[4] << 8) | u8Buf[5]);
    }
    void readRawAcc(int16_t acc[3]) const {
        uint8_t u8Buf[GYRO_AND_ACC_DATA_LEN];
        setBank(BANK_0);
        mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_ACCEL_XOUT_H, GYRO_AND_ACC_DATA_LEN, u8Buf);
        acc[0] = static_cast<int16_t>((u8Buf[0] << 8) | u8Buf[1]);
        acc[1] = static_cast<int16_t>((u8Buf[2] << 8) | u8Buf[3]);
        acc[2] = static_cast<int16_t>((u8Buf[4] << 8) | u8Buf[5]);
    }
    bool readRawMag(int16_t mag[3]) const {
        uint8_t u8Buf[MAG_DATA_LEN];
        setBank(BANK_0);
        mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00, MAG_DATA_LEN, u8Buf);
        mag[0] = static_cast<int16_t>((u8Buf[1] << 8) | u8Buf[0]);
        mag[1] = -static_cast<int16_t>((u8Buf[3] << 8) | u8Buf[2]);
        mag[2] = -static_cast<int16_t>((u8Buf[5] << 8) | u8Buf[4]);
        return (u8Buf[7] & 0x08);
    }
    int16_t readRawTemp() const {
        uint8_t u8Buf[2];
        setBank(BANK_0);
        mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_TEMP_OUT_H, 2, u8Buf);
        return (static_cast<int16_t>(u8Buf[0]) << 8) | static_cast<int16_t>(u8Buf[1]);
    }
    bool readAllRawDAta(int16_t gyro[3], int16_t acc[3], int16_t mag[3], int16_t& temp) {
        setBank(BANK_0);
        mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_ACCEL_XOUT_H, IMU_DATA_LEN, mRawDataBuf);
        acc[0] = static_cast<int16_t>((mRawDataBuf[0] << 8) | mRawDataBuf[1]);
        acc[1] = static_cast<int16_t>((mRawDataBuf[2] << 8) | mRawDataBuf[3]);
        acc[2] = static_cast<int16_t>((mRawDataBuf[4] << 8) | mRawDataBuf[5]);
        gyro[0] = static_cast<int16_t>((mRawDataBuf[6] << 8) | mRawDataBuf[7]);
        gyro[1] = static_cast<int16_t>((mRawDataBuf[8] << 8) | mRawDataBuf[9]);
        gyro[2] = static_cast<int16_t>((mRawDataBuf[10] << 8) | mRawDataBuf[11]);
        temp = static_cast<int16_t>((mRawDataBuf[12] << 8) | mRawDataBuf[13]);
        mag[0] = static_cast<int16_t>((mRawDataBuf[15] << 8) | mRawDataBuf[14]);
        mag[1] = -static_cast<int16_t>((mRawDataBuf[17] << 8) | mRawDataBuf[16]);
        mag[2] = -static_cast<int16_t>((mRawDataBuf[19] << 8) | mRawDataBuf[18]);
        return !(mRawDataBuf[21] & 0x08);
    }
    bool checkMag() const {
        uint8_t u8Ret[2];
        magI2CRead(REG_ADD_MAG_WIA1, 2, u8Ret);
#ifdef LOG
        if ((u8Ret[0] != REG_VAL_MAG_WIA1) || (u8Ret[1] != REG_VAL_MAG_WIA2)) {
            printf("Failed to obtain magnetometer address. Expected: %02x%02x, but received: %02x%02x Current bank: %d \n",
                   REG_VAL_MAG_WIA1, REG_VAL_MAG_WIA2, u8Ret[0], u8Ret[1], mCurrentBank);
        }
#endif
        return ((u8Ret[0] == REG_VAL_MAG_WIA1) && (u8Ret[1] == REG_VAL_MAG_WIA2));
    }
    void magI2CRead(const uint8_t regAddr, const uint8_t length, uint8_t* data) const {
        setBank(BANK_3);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG, regAddr);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | length);
        setBank(BANK_0);
        mI2C.readNBytes(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00, length, data);
    }
    void magI2CWrite(const uint8_t regAddr, const uint8_t value) const {
        uint8_t u8Temp;
        setBank(BANK_3);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_WRITE);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG, regAddr);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_DO, value);
        mI2C.writeByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | 1);
        sleepMS(100);
        magI2CRead(regAddr, 1, &u8Temp);
#ifdef LOG
        if (value != u8Temp) {
            printf("Failed to write %d to magnetometer address: %d. Data received: %d. Current bank: %d \n",
                   value, regAddr, u8Temp, mCurrentBank);
        }
#endif
    }
    void sleepMS(const short miliseconds) const {
        struct timespec recquired;
        struct timespec remain;
        recquired.tv_sec = 0;
        recquired.tv_nsec = static_cast<long int>(miliseconds) * 1000000;
        nanosleep(&recquired, &remain);
    }
    mutable ICM_BANK mCurrentBank;
    Config mConfig;
    float mGyroScale;
    float mAccScale;
    IMUData mData;
    uint8_t mRawDataBuf[22];
    I2C mI2C;
};

// IMUShared 클래스
class IMUShared {
public:
    void update(const IMUData& d, double ts) {
        std::lock_guard<std::mutex> lk(mtx_);
        data_ = d;
        ts_ = ts;
        has_ = true;
        ++seq_;
    }
    bool latest(IMUData& out, double& ts, uint64_t* seq = nullptr) const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!has_) return false;
        out = data_;
        ts = ts_;
        if (seq) *seq = seq_;
        return true;
    }
private:
    mutable std::mutex mtx_;
    IMUData data_{};
    double ts_{0.0};
    bool has_{false};
    uint64_t seq_{0};
};

// ... (I2C, ICM20948, IMUShared 클래스는 변경 없음)

// IMUReader 클래스
class IMUReader {
public:
    explicit IMUReader(const char* label) : label_(label), running_(true) {
        std::memset(cfg_.mDevice, 0, sizeof(cfg_.mDevice));
    }

    void setDevice(const char* devpath) {
        std::snprintf(cfg_.mDevice, sizeof(cfg_.mDevice), "%s", devpath);
    }

    ICM20948::Config& config() { return cfg_; }
    const ICM20948::Config& config() const { return cfg_; }

    bool init(bool do_calib_gyro = true) {
        if (!imu_.initialise(cfg_)) {
            std::fprintf(stderr, "[%s] 초기화 실패 (장치=%s)\n", label_, cfg_.mDevice);
            return false;
        }
        if (do_calib_gyro) imu_.calibrateGyro();
        return true;
    }

    void poll_once() {
        const IMUData& d = imu_.imuDataGet();
        shared_.update(d, now_s());
    }

    void run_loop(double hz) {
        using namespace std::chrono;
        const double Ts = (hz > 0) ? (1.0 / hz) : (1.0 / double(imu_.getConfig().mFramerate));
        while (running_.load()) {
            const auto t0 = steady_clock::now();
            poll_once();
            const auto dt = duration<double>(steady_clock::now() - t0).count();
            const double sleep_s = Ts - dt;
            // std::cerr << "sleep s: " << sleep_s << "\n";
            if (sleep_s > 0) std::this_thread::sleep_for(duration<double>(sleep_s));
        }
    }

    void stop() { running_ = false; }

    bool latest(IMUData& out, double& ts, uint64_t* seq = nullptr) const {
        return shared_.latest(out, ts, seq);
    }

    const char* label() const { return label_; }

private:
    static double now_s() {
        timespec ts{};
        clock_gettime(CLOCK_REALTIME, &ts);
        return double(ts.tv_sec) + double(ts.tv_nsec) / 1e9;
    }

    const char* label_;
    ICM20948 imu_;
    ICM20948::Config cfg_;
    IMUShared shared_;
    std::atomic<bool> running_; // 내부 실행 플래그
};

// 전역 변수와 시그널 핸들러 제거
// extern std::atomic<bool> g_running;
// inline void on_sigint(int) { g_running = false; }