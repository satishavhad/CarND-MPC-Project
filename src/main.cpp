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

Eigen::VectorXd predicted_state(double v, double delta_acc, double a, double cte, double epsi);

// for convenience
using json = nlohmann::json;

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

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
		  //initialize change in steering and acceleration
		  double delta_acc = j[1]["steering_angle"];
		  double a = j[1]["throttle"];
		  Eigen::VectorXd vehicle_ptsx(ptsx.size());
		  Eigen::VectorXd vehicle_ptsy(ptsy.size());

		  // change the coordinate system
		  for (unsigned int i = 0; i < ptsx.size(); i++) {
			  vehicle_ptsx[i] = (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi);
			  vehicle_ptsy[i] = (ptsx[i] - px) * sin(-psi) + (ptsy[i] - py) * cos(-psi);
		  }

		  // Fit a polynomial to x and y coordinates
		  Eigen::VectorXd coeffs = polyfit(vehicle_ptsx, vehicle_ptsy, 3);

		  // Calculate the cross track error
		  double cte = polyeval(coeffs, px) - py;
		  // Calculate the orientation error
		  double epsi = psi - atan(coeffs[1]);

		  // calculate the predicted state
		  Eigen::VectorXd prdt_state = predicted_state(v, delta_acc, a, cte, epsi);

		  // Get the predicted actuations and x, y values
		  vector<double> pred_variables = mpc.Solve(prdt_state, coeffs);
		  		  
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
		  // assign the predicted steering angle and throttle values
          msgJson["steering_angle"] = pred_variables[0];
          msgJson["throttle"] = pred_variables[1];

          //Display the MPC predicted trajectory 
		  vector<double> mpc_x_vals = { prdt_state[0] };
		  vector<double> mpc_y_vals = { prdt_state[1] };

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

		  //separate the x and y values from the predicted vector
		  for (unsigned int i = 2; i < pred_variables.size(); i += 2) {
			  mpc_x_vals.push_back(pred_variables[i]);
			  mpc_y_vals.push_back(pred_variables[i + 1]);
		  }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

		  //calcualte y values based on sequntial x values for the polynomial
		  for (unsigned int i = 1; i<15; i++) {
			  next_x_vals.push_back(10*i);
			  next_y_vals.push_back(polyeval(coeffs, 10*i));
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

//calculate the predicted state
Eigen::VectorXd predicted_state(double v, double delta_acc, double a, double cte, double epsi)
{
	float Lf = 2.67;
	float dt = 0.1;

	double px_predict = 0.0 + v * dt;
	const double py_predict = 0.0;
	double psi_predict = 0.0 + v * -delta_acc / Lf * dt;
	double v_predict = v + a * dt;
	double cte_predict = cte + v * sin(epsi) * dt;
	double epsi_predict = epsi + v * -delta_acc / Lf * dt;

	Eigen::VectorXd prdt_state(6);
	prdt_state << px_predict, py_predict, psi_predict, v_predict, cte_predict, epsi_predict;
	return prdt_state;
}