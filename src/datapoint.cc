#include "datapoint.h"


using namespace std;


DataPoint::DataPoint()
{
    this->initialized = false;
}

DataPoint::DataPoint(
    const long long timestamp,
    const DataPointType& data_type,
    const VectorXd& raw)
{
    this->set(timestamp, data_type, raw);
}

void DataPoint::set(
    const long long timestamp,
    const DataPointType& data_type,
    const VectorXd& raw)
{
    this->timestamp = timestamp;
    this->data_type = data_type;
    this->raw = raw;
    this->initialized = true;
}

VectorXd DataPoint::get() const
{
    return this->raw;
}

VectorXd DataPoint::get_state() const
{
    VectorXd state(STATE_NUM);

    if (this->data_type == DataPointType::IMU) {
        double yaw_rat = this->raw(0);
        double long_acc = this->raw(1);

        state << 0.0, 0.0, 0.0, 0.0, yaw_rat, long_acc;
    } else if (this->data_type == DataPointType::GPS) {
        double px = this->raw(0);
        double py = this->raw(1);
        double heading = this->raw(2);
        double velocity = this->raw(3);

        state << px, py, heading, velocity, 0.0, 0.0;
    }

    return state;
}

VectorXd DataPoint::get_vec() const
{
    VectorXd vec(STATE_NUM);

#if 0
    if (this->data_type == DataPointType::LIDAR) {
        double px = this->raw(0);
        double py = this->raw(1);

        vec << px, py, 0.0, 0.0;
    } else if (this->data_type == DataPointType::RADAR) {
        double rho = this->raw(0);
        double phi = this->raw(1);
        double px = rho * cos(phi);
        double py = rho * sin(phi);

        vec << px, py, 0.0, 0.0;
    } else if (this->data_type == DataPointType::STATE) {
        double px = this->raw(0);
        double py = this->raw(1);
        double v = this->raw(3);
        double yaw = this->raw(4);

        double vx = v * cos(yaw);
        double vy = v * sin(yaw);

        vec << px, py, vx, vy;
    } else if (this->data_type == DataPointType::TRUTH) {
        vec = this->raw;
    }
#endif

    return vec;
}


long long DataPoint::get_timestamp() const
{
    return this->timestamp;
}

DataPointType DataPoint::get_type() const
{
    return this->data_type;
}

void DataPoint::print() const
{
#if 0
    if (this->initialized) {
        cout << "Timestamp: " << this->timestamp << endl;
        cout << "Sensor ID: " << static_cast<int>(this->data_type) << " (LIDAR = 0 | RADAR = 1 | STATE = 2) " << endl;
        cout << "Raw Data: " << endl;
        cout << this->raw << endl;
    } else {
        cout << "DataPoint is not initialized." << endl;
    }
#endif
}
