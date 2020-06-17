#ifndef STATUSINFORMATION_H
#define STATUSINFORMATION_H

#include <vector>
#include <mutex>

class StatusInfo{

public:
    void getLocation(float &x, float &y);
    void setLocation(const double &x, const double &y);

    void getTarget(float &x, float &y);
    void setTarget(const double &x, const double &y);

    void getBattery_level(short &value);
    void setBattery_level(const short &value);

    void getStatus_code();
    void setStatus_code(short value);

    void getSensor_feedback_code();
    void setSensor_feedback_code(bool value);

    void lock_mutex(void);
    void unlock_mutex(void);

private:
    /*Current position in map coordinates*/
    float x_location_map, y_location_map;
    /*Last target in map coordinates*/
    float x_target_map, y_target_map;
    short battery_level;
    short status_code;
    bool sensor_feedback_code;
    //To update/read class memebers
    std::mutex mtx;
};

#endif //STATUSINFORMATION_H
