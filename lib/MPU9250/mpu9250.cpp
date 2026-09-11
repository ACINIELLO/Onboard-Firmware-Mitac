
#include "mpu9250.h"

uint8_t mpuaddr = 0x68;

void MPU::init(int accelRange, int gyroRange,int sample_size,struct AStruct *accel, struct AStruct *gyro,struct AStruct *accel_bias, struct AStruct *gyro_bias)

{
    WIRE.beginTransmission(ADDR);
    WIRE.write(0x6B);
    WIRE.endTransmission(true);

    acc_setup(accelRange,sample_size,accel,accel_bias);
    gyro_setup(gyroRange,sample_size,gyro,gyro_bias);
}



void MPU::acc_setup(int range,int sample_size, struct AStruct *accel,struct AStruct *accel_bias)
{ 
    // Bias values
long ax_bias = 0, ay_bias = 0, az_bias = 0;
long ax_sum = 0, ay_sum = 0, az_sum = 0;

    // accelerometer registers setup
    WIRE.beginTransmission(ADDR);
    WIRE.write(ACC_CONFIG);

 switch (range)

    {
    case 0:
        WIRE.write(0x00);

        break;

    case 1:
        WIRE.write(0x8);

        break;

    case 2:
        WIRE.write(0x10);

        break;

    case 3:

        WIRE.write(0x18);

        break;
    }

    WIRE.endTransmission(true);
/*
    for (int i=0; i < sample_size; i++)
    {
    get_acc(range,accel);
    ax_sum += accel->XAxis;
    ay_sum += accel->YAxis;
    az_sum += accel->ZAxis;

    }
    accel_bias->XAxis = ax_sum / sample_size;
    accel_bias->YAxis = ay_sum / sample_size;
    accel_bias->ZAxis = (az_sum / sample_size) - AccelRange[range];
 */

}
void MPU::gyro_setup(int range,int sample_size, struct AStruct *gyro,struct AStruct *gyro_bias)
{ 
        // Bias values
long gx_bias = 0, gy_bias = 0, gz_bias = 0;
long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    // gyroscope registers setup
    WIRE.beginTransmission(ADDR);
    WIRE.write(GYRO_CONFIG);
    switch (range)

    {
    case 0:
        WIRE.write(0x00);

        break;

    case 1:
        WIRE.write(0x08);

        break;

    case 2:
        WIRE.write(0x10);

        break;

    case 3:

        WIRE.write(0x18);

        break;
    }

    WIRE.endTransmission(true);
/*
  for (int i=0; i < sample_size; i++)
    {
    get_gyro(range,gyro);
    gx_sum += gyro->XAxis;
    gy_sum += gyro->YAxis;
    gz_sum += gyro->ZAxis;
    }
  gyro_bias->XAxis = gx_sum / sample_size;
   gyro_bias->YAxis = gy_sum / sample_size;
  gyro_bias->ZAxis = gz_sum / sample_size;
*/
    



    
}
void MPU::get_acc(int Anum, struct AStruct *acc)
{
    WIRE.beginTransmission(ADDR);
    WIRE.write(ACCEL_XOUT_H);
    WIRE.endTransmission(false);
    WIRE.requestFrom(mpuaddr, (size_t)6, true);

    int16_t xdata = WIRE.read() << 8 | WIRE.read();
    int16_t ydata = WIRE.read() << 8 | WIRE.read();
    int16_t zdata = WIRE.read() << 8 | WIRE.read();

    acc->XAxis = (float)xdata / AccelRange[Anum];
    acc->YAxis = (float)ydata / AccelRange[Anum];
    acc->ZAxis = (float)zdata / AccelRange[Anum];

    WIRE.endTransmission(true);
}

void MPU::get_gyro(int Gnum, struct AStruct *gyro)
{
    WIRE.beginTransmission(ADDR);
    WIRE.write(GYRO_XOUT_H);
    WIRE.endTransmission(true);
    WIRE.requestFrom(mpuaddr, (size_t)6, true);

    int16_t xdata = WIRE.read() << 8 | WIRE.read();
    int16_t ydata = WIRE.read() << 8 | WIRE.read();
    int16_t zdata = WIRE.read() << 8 | WIRE.read();

    gyro->XAxis = (float)xdata / GyroRange[Gnum];
    gyro->YAxis = (float)ydata / GyroRange[Gnum];
    gyro->ZAxis = (float)zdata / GyroRange[Gnum];

    WIRE.endTransmission(true);
}

void MPU::get_magnet( struct AStruct *mag)
{
    WIRE.beginTransmission(ADDR);
    WIRE.write(MAGNET_XOUT_H);
    WIRE.endTransmission(false);
    WIRE.requestFrom(mpuaddr, (size_t)6, true);

    int16_t xdata = WIRE.read() << 8 | WIRE.read();
    int16_t ydata = WIRE.read() << 8 | WIRE.read();
    int16_t zdata = WIRE.read() << 8 | WIRE.read();

    mag->XAxis = (float)xdata * magnet_calib;
    mag->YAxis = (float)ydata * magnet_calib;
    mag->ZAxis = (float)zdata * magnet_calib;

    WIRE.endTransmission(true);
}