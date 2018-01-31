#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <numeric>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int n, it, steps, p_index;
double err, best_err;
bool flag;
std::vector<double> p, dp, best_p;
static std::vector<std::string> pid_coef = {"p","i","d"};
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  p = {0.103327,0.00988239,0.736937};//0.138901,0.00631,1 //0.103327,0.00988239,0.736937
  dp = {0.03,0.001,0.5};
  n = 0;
  it = 0;
  steps = 800;
  p_index = 0;
  err = 0.0;
  best_err = std::numeric_limits<double>::max();
  best_p = {0.0, 0.0, 0.0};
  flag = true;

  pid.Init(p);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          err += pow(cte, 2);
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          n++;

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          json msgJson;

          /*if(n > steps){
            err /= steps;
            if(err < best_err){
                best_err = err;
                best_p = pid.p;
                if(it == 0){
                    flag = true;
                }
                else{
                    dp[p_index] *= 1.1;
                    p_index = (p_index + 1) % 3;
                    pid.p[p_index] += dp[p_index];
                    flag = true;
                }
            }
            else{
                if(flag){
                    pid.p[p_index] -= dp[p_index] * 2;
                    flag = false;
                }
                else{
                    pid.p[p_index] += dp[p_index];
                    dp[p_index] *= 0.9;
                    p_index = (p_index + 1) % 3;
                    pid.p[p_index] += dp[p_index];
                    flag = true;
                }
            }
            err = 0;
            n = 0;
            it++;
            std::cout << std::endl;
            std::cout << "Iteration: " << it << "\tBest error: " << best_err << "\tTuning parameter: " << pid_coef[p_index] << std::endl;
            std::cout << "tol: " << dp[0]+dp[1]+dp[2] << "\tp: " << pid.p[0] << "," << pid.p[1] << "," << pid.p[2] << "\tdp: " << dp[0] << "," << dp[1] << "," << dp[2] << std::endl;
            std::cout << "Best PID Coefficients: " << best_p[0] << "," << best_p[1] << "," << best_p[2] << std::endl;
          }*/
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
});

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
