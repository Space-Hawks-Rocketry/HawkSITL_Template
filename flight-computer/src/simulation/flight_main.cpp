#include "flight_main.hpp"

ControlStep setup(json sensor_data) {
    return {.control_msg = {}, .dt = 0.01};;
}

ControlStep loop(json sensor_data) {
    return {.control_msg = {}, .dt = 0.01};
}