#include <stdio.h>
#include "IPC_environment.hpp"
#include <format>
#include <iostream>
#include <math.h>

float F1 = 0;
float F2 = 0;

float min(float a, float b) {
    if (a > b)
        return b;
    return a;
}

float max(float a, float b) {
    if (a > b)
        return a;
    return b;
}

float clamp(float val, float a, float b) {
    return min(b, max(a, val));
}

int main() {
    IPC_Environment ipc;
    ipc.start(3563);
    
    while (true) {
        std::optional<json> msg_json_opt = ipc.recvJSON();
        if (msg_json_opt) {
            json msg_json = *msg_json_opt;
            
            float posx = msg_json["posx"].get<float>();
            float posy = msg_json["posy"].get<float>();
            float velx = msg_json["velx"].get<float>();
            float vely = msg_json["vely"].get<float>();
            float radians = msg_json["radians"].get<float>();
            float radians_rate = msg_json["radians_rate"].get<float>();

            std::cout << "\n(COMPUTER): Received sensor data -> " << std::format("px: {}, py: {}, vx: {}, vy: {}, radians: {}, radians_rate: {}", posx, posy, velx, vely, radians, radians_rate) << std::endl;
            
            float k1;
            float k2;
            
            k1 = 1;
            k2 = 1.5;
            float Fx = k1 * posx + k2 * velx;
            k1 = 1;
            k2 = 1;
            float Fy = k1 * posy + k2 * vely + 1.0*9.8;

            // Physics says --> Fx = -(F1 + F2) * sin(angle) --> angle = -arcsin(Fx / (F1 + F2))
            float target_radians;
            if ((F1 + F2) == 0) {
                target_radians = 0;
            } else {
                float drift_slope = Fx / (F1 + F2);
                if (drift_slope > 1) {
                    drift_slope = 1;
                } else if (drift_slope < -1) {
                    drift_slope = -1;
                } else {
                    target_radians = -asin(drift_slope);
                }
            }
            // float target_radians = -atan2(Fx, Fy);
            
            float max_angle = 3.14159/3;
            // (2*F1 + force_diff) * cos(radians) = Fy
            if (target_radians >= max_angle) {
                target_radians = max_angle;
            } else if (target_radians <= -max_angle) {
                target_radians = -max_angle;
            }

            k1 = 10;
            k2 = 5;
            float torque = k1 * (target_radians - radians) - k2 * radians_rate;
            float force_diff = torque / 1.0; // Difference in force between thrusters (- for T1, + for T2)
            
            
            float MAX_THRUST = 10;
            F1 = clamp(0.5 * (Fy / cos(radians) - force_diff), 0, MAX_THRUST);
            F2 = clamp(F1 + force_diff, 0, MAX_THRUST);
            

            // std::cout << F1 << std::endl;
            json response = {
                {"commands", {
                    {"F1", F1},
                    {"F2", F2}
                }}
            };

            ipc.sendJSON(response);
        }
    }

    return 0;
}