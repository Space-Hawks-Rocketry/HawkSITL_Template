#include <stdio.h>
#include "IPC_environment.hpp"
#include <format>
#include <iostream>
#include <math.h>

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

            k1 = 1.0;
            k2 = 1.0;
            float F_vert = k1 * posy + k2 * vely + 9.8 * 1.0;
            float F1_vert = F_vert / 2.0;
            if (posy < 0) {
                F1_vert = 0;
            }
            float F2_vert = F1_vert; 

            k1 = 0.1;
            k2 = 0.1;
            float target_radians = -k1 * posx - k2 * velx;
            if (target_radians > 0.523599) {
                target_radians = 0.523599;
            }
            if (target_radians < -0.523599) {
                target_radians = -0.523599;
            }

            k1 = 0.1;
            k2 = 0.3;
            float torque = k1 * (target_radians - radians) - k2 * radians_rate;

            float F1_horz = 0;
            float F2_horz = 0;
            if (torque > 0) {
                F2_horz = abs(torque) / 1.0;
            } else {
                F1_horz = abs(torque) / 1.0;
            }

            json response = {
                {"commands", {
                    {"F1", F1_horz + F1_vert},
                    {"F2", F2_horz + F2_vert}
                }}
            };

            // {"Fx", k1 * posx + k2 * velx},
            //     {"Fy", k1 * posy + k2 * vely},

            ipc.sendJSON(response);
        }
    }

    return 0;
}