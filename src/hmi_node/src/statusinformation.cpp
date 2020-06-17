#include "statusinformation.h"

void StatusInfo::getLocation(float &x, float &y)
{
    mtx.lock();
    x = x_location_map;
    y = y_location_map;
    mtx.unlock();
}

void StatusInfo::setLocation(const double &x, const double &y) {
    mtx.lock();
    x_location_map = x;
    y_location_map = y;
    mtx.unlock();
}

void StatusInfo::getTarget(float &x, float &y)
{
    mtx.lock();
    x = x_target_map;
    y = y_target_map;
    mtx.unlock();
}

void StatusInfo::setTarget(const double &x, const double &y)
{
    mtx.lock();
    x_target_map = x;
    y_target_map = y;
    mtx.unlock();
}

void StatusInfo::getBattery_level(short &b)
{
    mtx.lock();
    b = battery_level;
    mtx.unlock();
}

void StatusInfo::setBattery_level(const short &value)
{
    mtx.lock();
    battery_level = value;
    mtx.unlock();
}

void StatusInfo::getStatus_code()
{
    //return status_code;
}

void StatusInfo::setStatus_code(short value)
{
    mtx.lock();
    status_code = value;
    mtx.unlock();
}

void StatusInfo::getSensor_feedback_code()
{
    //return sensor_feedback_code;
}

void StatusInfo::setSensor_feedback_code(bool value)
{
    mtx.lock();
    sensor_feedback_code = value;
    mtx.unlock();
}

void StatusInfo::lock_mutex() {
    mtx.lock();
}

void StatusInfo::unlock_mutex() {
    mtx.unlock();
}
