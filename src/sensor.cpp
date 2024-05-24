#include <sensor.h>

bool sensor::imu_init(){
    bool success= true;
    if (imu_dev)
        delete imu_dev;
    imu_dev = new ICM_20948_I2C();
    imu_dev->begin(Wire,true);
    Serial2.print("Initialization of the sensor returned: ");
    Serial2.println(imu_dev->statusString());
    success &= (imu_dev->status == ICM_20948_Stat_Ok);
    if (!success)
        Serial2.println(F("Trying again..."));
    else
        Serial2.println("Device connected");
    delay(500);
    return success;
}

bool sensor::imu_dmp_init(){
    // the code is based on sparkfun example
    bool success = true;
    success &= (imu_dev->initializeDMP() == ICM_20948_Stat_Ok);


    // Enable the DMP orientation sensor
    success &= (imu_dev->enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok); // 16-bit accel
    success &= (imu_dev->enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok); // 16-bit gyro + 32-bit calibrated gyro
    success &= (imu_dev->enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok); // 32-bit 9-axis quaternion
    success &= (imu_dev->enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:

    success &= (imu_dev->setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to 225Hz
    success &= (imu_dev->setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to 225Hz
    success &= (imu_dev->setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to 225Hz
    success &= (imu_dev->setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to 225hz
    //success &= (imu_dev->setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (imu_dev->enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (imu_dev->enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (imu_dev->resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (imu_dev->resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
        Serial2.println("DMP enabled.");
    else
        Serial2.println("Enable DMP failed! \nPlease check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...");
    
    return success;
}

void sensor::print_biases(biasStore *store)
{
  Serial2.print(F("Gyro X: "));
  Serial2.print(store->biasGyroX);
  Serial2.print(F(" Gyro Y: "));
  Serial2.print(store->biasGyroY);
  Serial2.print(F(" Gyro Z: "));
  Serial2.println(store->biasGyroZ);
  Serial2.print(F("Accel X: "));
  Serial2.print(store->biasAccelX);
  Serial2.print(F(" Accel Y: "));
  Serial2.print(store->biasAccelY);
  Serial2.print(F(" Accel Z: "));
  Serial2.println(store->biasAccelZ);
  Serial2.print(F("CPass X: "));
  Serial2.print(store->biasCPassX);
  Serial2.print(F(" CPass Y: "));
  Serial2.print(store->biasCPassY);
  Serial2.print(F(" CPass Z: "));
  Serial2.println(store->biasCPassZ);

}

bool sensor::load_bias(){
    bool success = true;
    EEPROM.get(0,bias_store);

    if (is_bias_store_valid(&bias_store)){
        Serial2.println(F("Bias data in EEPROM is valid. Restoring it..."));
        success &= (imu_dev->setBiasGyroX(bias_store.biasGyroX) == ICM_20948_Stat_Ok);
        success &= (imu_dev->setBiasGyroY(bias_store.biasGyroY) == ICM_20948_Stat_Ok);
        success &= (imu_dev->setBiasGyroZ(bias_store.biasGyroZ) == ICM_20948_Stat_Ok);
        success &= (imu_dev->setBiasAccelX(bias_store.biasAccelX) == ICM_20948_Stat_Ok);
        success &= (imu_dev->setBiasAccelY(bias_store.biasAccelY) == ICM_20948_Stat_Ok);
        success &= (imu_dev->setBiasAccelZ(bias_store.biasAccelZ) == ICM_20948_Stat_Ok);
        success &= (imu_dev->setBiasCPassX(bias_store.biasCPassX) == ICM_20948_Stat_Ok);
        success &= (imu_dev->setBiasCPassY(bias_store.biasCPassY) == ICM_20948_Stat_Ok);
        success &= (imu_dev->setBiasCPassZ(bias_store.biasCPassZ) == ICM_20948_Stat_Ok);

        if (success){
            Serial2.println("Biases restored");
            print_biases(&bias_store);
        }
        else{
            Serial2.println("Bias restore failed!");
            Serial2.println("please calibrate with calibration code");

        }
    }
    return success;
}

bool sensor::is_bias_store_valid(biasStore *bias_store){
    int32_t sum = bias_store->header;

    if (sum != 0x42)
        return false;

    sum += bias_store->biasGyroX;
    sum += bias_store->biasGyroY;
    sum += bias_store->biasGyroZ;
    sum += bias_store->biasAccelX;
    sum += bias_store->biasAccelY;
    sum += bias_store->biasAccelZ;
    sum += bias_store->biasCPassX;
    sum += bias_store->biasCPassY;
    sum += bias_store->biasCPassZ;

    return (bias_store->sum == sum);
}

void sensor::imu_loop(sensor_msgs__msg__Imu *imu_msg, sensor_msgs__msg__MagneticField *mag_msg){
    imu_dev->readDMPdataFromFIFO(&imu_data);

    if ((imu_dev->status == ICM_20948_Stat_Ok) || (imu_dev->status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        uint32_t current_time_ms = millis();
        
        if ((imu_data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
        {
            
            double q1 = ((double)imu_data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)imu_data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)imu_data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            imu_msg->orientation = {
                .x = q1,
                .y = q2,
                .z = q3,
                .w = q0
            };

            imu_msg->header.stamp.sec = current_time_ms/1000;
            imu_msg->header.stamp.nanosec = (current_time_ms % 1000) * 1000000;

        }
        if ((imu_data.header & DMP_header_bitmap_Accel) > 0){

            imu_msg->angular_velocity = {
                .x = ((double)imu_data.Compass.Data.X) / 32768.0f * 2000 * M_PI / 180,
                .y = ((double)imu_data.Raw_Gyro.Data.Y) / 32768.0f * 2000 * M_PI / 180,
                .z = ((double)imu_data.Raw_Gyro.Data.Z) / 32768.0f * 2000 * M_PI / 180
            };
        }
        if ((imu_data.header & DMP_header_bitmap_Gyro) > 0){

            imu_msg->linear_acceleration = {
                .x = ((double)imu_data.Raw_Accel.Data.X) / 32768.0f * 4 * 9.80665,
                .y = ((double)imu_data.Raw_Accel.Data.Y) / 32768.0f * 4 * 9.80665,
                .z = ((double)imu_data.Raw_Accel.Data.Z) / 32768.0f * 4 * 9.80665
            };
        }
        
        if ((imu_data.header & DMP_header_bitmap_Compass) > 0){
            mag_msg->magnetic_field = {
                .x = ( (double) imu_data.Compass.Data.X ) /1000,
                .y = ((double) imu_data.Compass.Data.Y) /1000,
                .z = ((double) imu_data.Compass.Data.Z) /1000
            };
            mag_msg->header.stamp.sec = current_time_ms/1000;
            mag_msg->header.stamp.nanosec = (current_time_ms % 1000) * 1000000;
        }
    }
    if (imu_dev->status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    {
        delay(10);
    }
}

void sensor::begin(){
    while(!imu_init()){}
    while (!imu_dmp_init()){}
    load_bias();
}
