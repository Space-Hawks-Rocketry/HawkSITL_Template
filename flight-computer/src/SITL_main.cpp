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

    /// Run flight setup as soon as sensor data becomes available
    while (true) {
        /// Try to receive environment data (usually sensor data)
        std::optional<json> sensor_data_opt = (ipc).recvJSON();
        if (!sensor_data_opt)
            continue;
        
        /// Execute setup and get a control step back (if provided)
        ControlStep control_step = setup(*sensor_data_opt);    
        
        /// Send the control step back to the environment 
        ipc.sendJSON({
            {"control_msg", control_step.control_msg},
            {"dt", control_step.dt}
        });
        // Beak now that setup has succesfully completed
        break;
    }

    /// Execute control loop
    while (true) {
        /// Try to receive sensor data
        std::optional<json> sensor_data_opt = (ipc).recvJSON();
        if (!sensor_data_opt)
            continue;
        
        /// Execute this control step
        ControlStep control_step = loop(*sensor_data_opt);    
        
        /// Send the control step back to the environment 
        ipc.sendJSON({
            {"control_msg", control_step.control_msg},
            {"dt", control_step.dt}
        });
    }

    return 0;
}