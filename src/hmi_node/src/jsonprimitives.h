#ifndef JSONPRIMITIVES_H
#define JSONPRIMITIVES_H

#include "nlohmann/json.hpp"

using json = nlohmann::json;

void build_init_resp_message(json &j,
                             const std::string &tag,
                             const std::string &id,
                             const std::string &version);
void build_feedback_resp_message(json &j,
                                 const short &intent_code,
                                 const std::string &tag,
                                 const bool &feedback);
void build_status_resp_message(json &j,
                               const std::string &tag,
                               const float &x_loc, const float &y_loc,
                               const float &x_target, const float &y_target,
                               const short &status_code,
                               const short battery_level,
                               const bool &sensor_feedback_code);

#endif // JSONPRIMITIVES_H
