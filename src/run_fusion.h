#include <mutex>
#include <time.h>
#include "fusion.h"
#include "Eigen/Dense"
#include "parameterReader.h"



/**
 * @brief Class with the highest level of abstraction for the GPS INS estimation system. Uses DataPoint and Fusion
 * classes to perform filtering and state estimation on the robot.
 */
class GpsIns {
public:
    GpsIns(bool verbose = false);
    ~GpsIns();
    void read_gps_data(double lat, double lon, double alt);
    void read_imu_data(double yaw_rate, double long_accel);
    void read_encoders(double vel);
    void loop();
    void set_data();
    Eigen::VectorXd get_estimated_state() const;
    DataPoint get_sensor_data();

private:
    Fusion* filter;
    DataPoint* _sensor_data;
    EKFParams params;
    clock_t _dt;
    clock_t _cur_time;
    clock_t _prev_time;

    int32_t _imucounter, _gpscounter, _enccounter;
    int32_t _prev_imu_counter, _prev_gps_counter, _prev_enc_counter;
    std::mutex m;
    DataPointType _data_type;
    Eigen::VectorXd _raw_data;
    bool verbose;
};
