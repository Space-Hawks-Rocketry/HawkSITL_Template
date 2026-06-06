#include <stdio.h>
#include "IPC_environment.hpp"
#include <format>
#include <iostream>
#include <math.h>
#include "flight_main.hpp"

int main() {
    /// Initiate computer<-->environment IPC
    IPC_Environment ipc;
    ipc.start(3563);
    
    /// Execute control steps between environment simulation steps.
    while (true) {
        /// Receive data (usually sensor data) from the external environment
        std::optional<json> external_data_opt = ipc.recvJSON();
        if (!external_data_opt)
            continue;
        json external_data = *external_data_opt;
        
        /// Execute this control step
        ControlStep control_step = loop(external_data);

        /// Send the control step back to the environment 
        ipc.sendJSON({
            {"control_msg", control_step.control_msg},
            {"dt", control_step.dt}
        });
    }

    return 0;
}