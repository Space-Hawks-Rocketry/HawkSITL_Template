#include "flight_main.hpp"

ControlStep setup(json external) {
    return {.control_msg = {}, .dt = 0.01};;
}

ControlStep loop(json external_data) {
    return {.control_msg = {}, .dt = 0.01};
}