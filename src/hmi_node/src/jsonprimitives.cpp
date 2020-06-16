#include "jsonprimitives.h"

const std::string DELIMITER = "\\#$";

void build_init_resp_message(json &j,
                             const std::string &tag,
                             const std::string &id,
                             const std::string &version) {
    j["intent_code"] = 10;
    j["tag"] = tag;
    j["data"]["fleet_map"]["id"] = id;
    j["data"]["fleet_map"]["version"] = version;
}

void build_feedback_resp_message(json &j,
                                 const short &intent_code,
                                 const std::string &tag,
                                 const bool &feedback) {
    j["intent_code"] = intent_code;
    j["tag"] = tag;
    j["data"]["feedback"] = feedback;
}

void build_status_resp_message(json &j,
                               const std::string &tag,
                               const float &x_loc, const float &y_loc,
                               const float &x_target, const float &y_target,
                               const short &status_code,
                               const short battery_level,
                               const bool &sensor_feedback_code) {
    j["intent_code"] = 300;
    j["tag"] = tag;
    j["data"]["location"] = {x_loc, y_loc};
    j["data"]["target"] = {x_target, y_target};
    j["data"]["status"] = status_code;
    j["data"]["battery"] = battery_level;
    j["data"]["sensor"] = sensor_feedback_code;
}

void append_delimiter(std::string &json_string){
    json_string.append(DELIMITER);
}
