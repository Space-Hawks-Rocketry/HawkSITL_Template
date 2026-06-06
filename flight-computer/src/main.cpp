#include <stdio.h>
#include "IPC_environment.hpp"
#include <format>
#include <iostream>
#include <math.h>
#include "flight_main.hpp"

int main() {
    IPC_Environment ipc;
    ipc.start(3563);
    
    // FlightMain flight_main;
    
    while (true) {
        std::optional<json> msg_json_opt = ipc.recvJSON();
        if (msg_json_opt) {
            json msg_json = *msg_json_opt;
            
            // json response = flight_main.loop(msg_json);
            json control_msg = loop(msg_json);
            json response = {
                {"control_msg", control_msg},
                {"dt", dt}
            };
            ipc.sendJSON((json)response["commands"]);
        }
    }

    return 0;
}