//
// Created by veon on 11/26/17.
//

#ifndef PID_POLICY_H
#define PID_POLICY_H
#include <uWS/WebSocketProtocol.h>
#include <uWS/WebSocket.h>

void sendReset(uWS::WebSocket<uWS::SERVER>& ws)
{
    std::string msg = "42[\"reset\", {}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

#endif //PID_POLICY_H
