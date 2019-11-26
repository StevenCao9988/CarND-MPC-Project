#include "json.hpp"
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
const double dt = 0 / 1000;    // 延时100ms

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
		  double delta = j[1]["steering_angle"];
		  double a = j[1]["throttle"];

          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
		  int i = 0;     // 计数器
		  double dx = 0; // 路标与汽车x轴的差值
		  double dy = 0; // 路标与汽车y轴的差值

		  // 将绝对坐标系转化为汽车的相对坐标系
		  VectorXd ptsx_car(ptsx.size());
		  VectorXd ptsy_car(ptsy.size());
		  for (i = 0; i < ptsy.size(); i++)
		  {
			  dx = ptsx[i] - px;
			  dy = ptsy[i] - py;
			  ptsx_car[i] =  dx * cos(psi) + dy * sin(psi);
			  ptsy_car[i] = -dx * sin(psi) + dy * cos(psi);
		  }

		  auto coeffs = polyfit(ptsx_car, ptsy_car, 3);   // 三阶多项式拟合

		  double dx_pred = 0 + v * dt;                    // 预测100ms后，路标与汽车x轴的差值
		  double dy_pred = 0;                             // 预测100ms后，路标与汽车y轴的差值
		  double psi_pred = 0 - v * delta * dt / Lf;      // 预测100ms后，车身偏转角
		  double v_pred = v + a * dt;                     // 预测100ms后，车辆速度

		  std::cout << "dx_pred " << dx_pred << std::endl;
		  std::cout << "dy_pred " << dy_pred << std::endl;
		  std::cout << "psi_pred " << psi_pred << std::endl;
		  std::cout << "v_pred " << v_pred << std::endl;
		  std::cout << "steer_true " << delta << std::endl;
		  std::cout << "throttle_true " << a << std::endl;

		  // 调用 MPC预测函数
		  Eigen::VectorXd state(4);
		  state << dx_pred, dy_pred, psi_pred, v_pred;
		  auto result = mpc.Solve(state, coeffs);
		  // 输出 [-1, 1]代表[-25,25]的deg2rad 弧度值,详见下文
		  double steer_value;
		  double throttle_value;
		  steer_value = result[0] / (deg2rad(25) * Lf);
		  throttle_value = result[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line  
		   * 在MPC函数中就确定好输出数据的顺序
           */
		  for (int i = 2; i < result.size(); i += 2)
		  {
			  mpc_x_vals.push_back(result[i]);
			  mpc_y_vals.push_back(result[i+1]);
		  }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line  
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */
          double poly_inc = 2.5;
          int num_points = 25;
          
          for (int i = 1; i < num_points; i++) {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }   
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(0));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}