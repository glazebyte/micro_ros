#include <EEPROM.h>
#include <ICM_20948.h>
#include <Adafruit_VL53L0X.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

struct biasStore
{
  int32_t header = 0x42;
  int32_t biasGyroX = 0;
  int32_t biasGyroY = 0;
  int32_t biasGyroZ = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;
  int32_t sum = 0;
};

class sensor
{
private:
    ICM_20948_I2C *imu_dev = NULL;
    Adafruit_VL53L0X *lox_dev = NULL;
    biasStore bias_store;
    icm_20948_DMP_data_t imu_data;
    
public:
    void begin();
    // imu
    bool imu_init();
    bool imu_dmp_init();
    bool load_bias();
    bool is_bias_store_valid(biasStore *bias_store);
    void print_biases(biasStore *bias_store);
    void imu_loop(sensor_msgs__msg__Imu *imu_msg, sensor_msgs__msg__MagneticField *mag_msg);
    void get_orientation();


    // vl53l0x
    void vl53l0x_init();
    void get_distance();
};