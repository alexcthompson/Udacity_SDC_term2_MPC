#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// previous actuations and constants
double prev_delta = 0.0;
double prev_a     = 0.0;
const double Lf = 2.67;
double dt_ = 0.1;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

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
    cout << sdata << endl;
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

          // predict forward 1 step with prev_delta and prev_a
          double next_px  = px + dt_ * v * cos(psi);
          double next_py  = py + dt_ * v * sin(psi);
          double next_psi = psi - v * prev_delta * dt_ / Lf;
          double next_v   = v + prev_a * dt_;

          cout << "prev_delta    = " << prev_delta << endl;
          cout << "prev_a        = " << prev_a << endl;
          cout << "px, next_px   = " << px << ", " << next_px << endl;
          cout << "py, next_py   = " << py << ", " << next_py << endl;
          cout << "psi, next_psi = " << psi << ", " << next_psi << endl;
          cout << "v, next_v     = " << v << ", " << next_v << endl;

          // translate waypoints into vehicle space
          Eigen::VectorXd xpts_vehicle = Eigen::VectorXd(ptsx.size());
          Eigen::VectorXd ypts_vehicle = Eigen::VectorXd(ptsy.size());

          for (int i = 0; i < ptsx.size(); i++) {
            double x_translated = ptsx[i] - next_px;
            double y_translated = ptsy[i] - next_py;
            xpts_vehicle[i] = x_translated * cos(-next_psi) - y_translated * sin(-next_psi);
            ypts_vehicle[i] = x_translated * sin(-next_psi) + y_translated * cos(-next_psi);
          }

          // fit the way points
          Eigen::VectorXd coeffs = polyfit(xpts_vehicle, ypts_vehicle, 3);

          // state vector
          Eigen::VectorXd state(6);
          double x_state = 0.0;
          double y_state = 0.0;
          double psi_state = 0.0;
          double cte  = -coeffs[0];
          double epsi = -atan(coeffs[1]);

          state << x_state, y_state, psi_state, next_v, cte, epsi;
          cout << "cte = " << cte << endl;
          cout << "epsi = " << epsi << endl;

          // Solve it
          vector<double> res = mpc.Solve(state, coeffs);

          // Controls
          double steer_value = res[0];
          prev_delta = steer_value;

          double throttle_value = res[1];
          prev_a = throttle_value;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / (0.436332 * 2.67);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i =0; i < (res.size() / 2) - 1; i++) {
            mpc_x_vals.push_back(res[2 * (i + 1)]);
            mpc_y_vals.push_back(res[2 * (i + 1) + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 3.0;
          int num_points = 15;

          for (int i = 0; i < num_points; i ++) {
            double next_x = poly_inc * i;
            double next_y = polyeval(coeffs, next_x);
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
