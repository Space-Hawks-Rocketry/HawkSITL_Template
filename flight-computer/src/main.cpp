// #include <zmq.hpp>
// #include <nlohmann/json.hpp>
// #include <iostream>
// #include <string.h>
// using json = nlohmann::json;

// int main()
// {
    // zmq::context_t context;
    // zmq::socket_t socket(context, zmq::socket_type::rep);
//     // socket.bind("inproc://test");
    
//     socket.bind("tcp://localhost:3563");
    
//     zmq::message_t request;

//     while (true) {
        
//         auto result = socket.recv(&request);

//         if (result) {
//             // Convert the raw ZMQ message into a std::string
//             std::string json_string(static_cast<char*>(request.data()), request.size());
//             std::cout << "\n(COMPUTER): Received sensor data -> " << json_string << std::endl;

//             try {
//                 // Parse the string into a JSON object
//                 json data = json::parse(json_string);
                
                // // Extract data safely using native C++ types
                // float posx = data["posx"].get<float>();
                // float posy = data["posy"].get<float>();
                // float velx = data["velx"].get<float>();
                // float vely = data["vely"].get<float>();

                // float k1 = 3.0;
                // float k2 = -3.0;

                // json response = {
                //     {"Fx", k1 * posx + k2 * velx},
                //     {"Fy", k1 * posy + k2 * vely},
                // };

//                 std::string response_str = response.dump();
//                 // // TODO: Send JSON String Back!!
//                 socket.send(zmq::buffer(response_str), zmq::send_flags::none);

//             } catch (const json::parse_error& e) {
//                 std::cerr << "JSON Parsing error: " << e.what() << std::endl;
//                 socket.send(zmq::buffer("Error: Invalid JSON"), zmq::send_flags::none);
//             }
//         }
//     }

//     // auto json_msg = json::parse(text);

//     // socket.send(zmq::str_buffer("Me!"), zmq::send_flags::dontwait);
    
//     // auto json_msg = json::parse(message.to_string());
//     // printf("%s\n", message.);

//     socket.close();
//     context.close();
// }

#include <stdio.h>
#include "IPC_environment.hpp"
#include <format>
#include <iostream>

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

            std::cout << "\n(COMPUTER): Received sensor data -> " << std::format("px: {}, py: {}, vx: {}, vy: {}", posx, posy, velx, vely) << std::endl;

            float k1 = 3.0;
            float k2 = -3.0;

            json response = {
                {"Fx", k1 * posx + k2 * velx},
                {"Fy", k1 * posy + k2 * vely},
            };

            ipc.sendJSON(response);
        }
    }

    return 0;
}