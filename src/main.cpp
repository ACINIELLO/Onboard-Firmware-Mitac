#include "helpers.h"
// =====================================================================================
//                                    Setup:
// =====================================================================================
void setup(void)
{



    // motor stuff:
    esc.attach(MOTOR1_PWM_PIN);
    esc2.attach(MOTOR2_PWM_PIN);
    esc3.attach(MOTOR3_PWM_PIN);
    esc4.attach(MOTOR4_PWM_PIN);

    // start serial monitor
    Serial.begin(SERIAL_MONITOR_BAUD);
    esp32_SERIAL.begin(ESP32_BAUD);

    Wire.begin();

    ina.config_setup(); // ina260.begin(); //
    setParts();

    //  mlx.pwmctrl();

}

// =====================================================================================
//                                    Main Loop:
// =====================================================================================
void loop(void)
{

    uint32_t timestamp = millis();

    radio_read_old = radio_read;
    radio_read_old2 = radio_read2;
    radio_read_old3 = radio_read3;

    setParts();

    // read MS5611
    if (partsStates.baro)
    {
        if (baro.getTempPress(&temp, &pres))
        {
            Serial.printf("baro read failed\n");
        }
    }

    // read IMU:

    mpu.get_acc(1, &imu_acc);
    mpu.get_gyro(1, &imu_gyro);
    accelx = imu_acc.XAxis; //- accel_bias.XAxis; 
    accely = imu_acc.YAxis;// - accel_bias.YAxis; 
    accelz = imu_acc.ZAxis ;//- accel_bias.ZAxis; 
    
    gyrox = imu_gyro.XAxis ;//- gyro_bias.XAxis; 
    gyroy = imu_gyro.YAxis ;//- gyro_bias.YAxis; 
    gyroz = imu_gyro.ZAxis ;//- gyro_bias.ZAxis; 


    accel_resultant = sqrt(pow(accelx, 2) + pow(accely, 2) + pow(accelz, 2));
    angleX = rad_to_deg(atan(accelz/ accelx));
    angleY = rad_to_deg(atan(accelz/ accely));
    angleZ = rad_to_deg(atan(accely/ accelz));
    angleX_diff = angleX - angleX_start;
    angleY_diff = angleY - angleY_start;
    angleZ_diff = angleZ - angleZ_start;

    gps.read_RMC(&lon, &lat);
    gps.read_GGA(&gps_quality, &gps_alt);

    // convert pres from mbar to kpa:

    pres = 0.1 * pres;
    // convert air temp from degC to kelvin:

    temp = temp + 273.15;

    alt = (log(pres / sea_press) * R_gas_const * temp) / (-gravitational_const * molecular_weight_air) + alt_dt_toronto; // https://en.wikipedia.org/wiki/Barometric_formula

    // batt_temp = mlx.get_temp();                                                                                                         // mplx.get_temp(0); // analogToTemp(pin_thermistor);

    batt_volt = ina.get_volt() / 1000.0; // ina260.readBusVoltage()/1000.0; // ;//   //


    // joystick controls:
    if (radio_SERIAL.find('X'))
    {
        radio_read = radio_SERIAL.readStringUntil('\n');
        Serial.print("joystick X: ");
        Serial.println(radio_read.c_str());

        if (radio_read != radio_read_old)
        {

            /*

         PUT YOUR flight CODE HERE
            */
        }
    }

    if (radio_SERIAL.find('Y'))
    {

        radio_read2 = radio_SERIAL.readStringUntil('\n');
        Serial.print("joystick Y:");
        Serial.println(radio_read2.c_str());
        if (radio_read2 != radio_read_old2)
        {
            /*

         PUT YOUR flight CODE HERE
            */
        }
    }
    if (radio_SERIAL.find("T"))
    {

        radio_read3 = radio_SERIAL.readStringUntil('\n');
        Serial.print("joystick T:");
        Serial.println(radio_read3.c_str());
        DUTY_CYCLE_FRACT_T = mapfloat(radio_read3.toFloat(), -1.0, 1.0, 0.0, 1.0);

        rotateMotor(1, MOTOR1_PWM_PERIOD, DUTY_CYCLE_FRACT_T);
        rotateMotor(2, MOTOR1_PWM_PERIOD, DUTY_CYCLE_FRACT_T);

        rotateMotor(3, MOTOR1_PWM_PERIOD, DUTY_CYCLE_FRACT_T);
        rotateMotor(4, MOTOR1_PWM_PERIOD, DUTY_CYCLE_FRACT_T);
        if (radio_read3.toFloat() <= radio_read_old3.toFloat() + thrust_threshold && radio_read3.toFloat() >= radio_read_old3.toFloat() - thrust_threshold)
        {

            /*

         PUT YOUR flight CODE HERE
            */
        }
    }

    // motor controls:

    // rotateMotor(MOTOR1_PWM_PERIOD, DUTY_CYCLE_FRACT_Y);

    // print stuff to serial
    sprintf(
        string, outputFormat,
        timestamp / 1000, temp, pres, alt, (alt - alt_start), lat, lon, gps_quality, gps_alt);

    Serial.printf("%s", string);
    radio_SERIAL.printf("%s", string);


    if (partsStates.sdcard)
    {
        File dataFile = SD.open(logFileName.c_str(), FILE_WRITE);

        if (dataFile)
        {
            dataFile.println(string);
            dataFile.close();
        }
        else
        {
            Serial.printf("error opening %s\n", logFileName.c_str());
            partsStates.sdcard = false;
        }
    }



}