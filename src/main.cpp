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


// Switches and constants
//==============================================================================
#define ENABLE_LATENCY_COMPENSATION (1)
#define ENABLE_DEBUG (0)
#define LF (2.67)
#define LATENCY_S (0.1)
//==============================================================================


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
    // Loop counter
    unsigned int i;

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
          double px           = j[1]["x"];
          double py           = j[1]["y"];
          double psi          = j[1]["psi"];
          double v            = j[1]["speed"];
          double steer_value  = j[1]["steering_angle"];
          // Throttle not required as v is considered constant during latency
          //double throttle_value = j[1]["throttle"];

#if ENABLE_DEBUG
          for (auto ii = ptsx.begin(); ii != ptsx.end(); ++ii)
          {
            std::cout << "ptsx " << *ii << std::endl;
            std::cout << "ptsy " << *ii << std::endl;
          }
          for (i=0; i<ptsx.size(); ++i)
          {
            std::cout << "ptsx " << i << " - " << ptsx[i] << std::endl;
            std::cout << "ptsy " << i << " - " << ptsy[i] << std::endl;
          }
          std::cout << "px, py, psi, v, steer, throttle " << px << " - " << py << " - " << psi << " - " << v << " - " << steer_value << " - " << throttle_value << std::endl;
#endif

          // Transform reported coordinated to local car coordinates
          // This will put x, y and psi to 0
          //====================================================================
          double sin_psi = sin(0-psi);
          double cos_psi = cos(0-psi);
          for (i=0; i<ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx[i] = ((shift_x * cos_psi) - (shift_y * sin_psi));
            ptsy[i] = ((shift_x * sin_psi) + (shift_y * cos_psi));

#if ENABLE_DEBUG
            std::cout << "local waypts x/y " << ptsx[i] << " - " << ptsy[i] << std::endl;
#endif
          }


          // Map transformed waypoints from vector<double> to VectorXd
          //====================================================================
          Eigen::Map<Eigen::VectorXd> ptsx_trans(&ptsx[0], 6);
          Eigen::Map<Eigen::VectorXd> ptsy_trans(&ptsy[0], 6);


          // Fit a polynomial
          //====================================================================
          auto coeffs = polyfit(ptsx_trans, ptsy_trans, 3);


          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // Calculate vehicle state
          //====================================================================
          Eigen::VectorXd state(6);

#if ENABLE_LATENCY_COMPENSATION
          // Incorporate latency by predicting vehicle state (position etc.) after latency
          double dx   = v * std::cos(steer_value) * LATENCY_S;
          double dy   = v * std::sin(steer_value) * LATENCY_S;
          double dpsi = -(v * steer_value * LATENCY_S) / LF;
          // Velocity is considered constant in latenct interval
          //double dv = v + throttle_value * LATENCY_S;

          // Calculate the cross track error
          double cte = polyeval(coeffs, dx);

          // Calculate the orientation error
          double epsi = -atan(coeffs(1) + coeffs(2) * dx + coeffs(3) * dx * dx);

          // Set state with predicted state after latency
          //====================================================================
          state << dx, dy, dpsi, v, cte, epsi;
#else
          // Calculate the cross track error - x is 0 due to transformation
          double cte = polyeval(coeffs, 0);

          // Calculate the orientation error - all x are 0 due to transformation
          double epsi = -atan(coeffs[1]);

          // Coordinated are transformed to car coordinates, therefore,
          // x, y and psi become 0
          //====================================================================
          state << 0, 0, 0, v, cte, epsi;
#endif


          // Feed state and coeffs to MPC and solve it (reduce cost)
          //====================================================================
          auto mpcResult = mpc.Solve(state, coeffs);


          // Generate data for simulator
          //====================================================================
          json msgJson;

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -mpcResult[0] / (deg2rad(25)*2.67);
          msgJson["throttle"] = mpcResult[1];


          // Display the MPC predicted trajectory (green line)
          //====================================================================
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["mpc_x"] = mpc.predicted_x;
          msgJson["mpc_y"] = mpc.predicted_y;

#if ENABLE_DEBUG
          std::cout << "st/thr " << mpcResult[0] / (deg2rad(25) * LF) << " " << mpcResult[1] << std::endl;
#endif

          // Display the waypoints/reference line (yellow line)
          //====================================================================
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          unsigned int num_points = 25;
          for (i=1; i<num_points; i++)
          {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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
