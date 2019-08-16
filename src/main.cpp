#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


bool canChangeLanes(const int target_lane,
                    const double car_s,
                    const vector<vector<double>> &sensor_fusion)
{
  // Maximum s value for simulation track
  const double max_s = 6945.554;

  for (int i = 0; i < sensor_fusion.size(); ++i)
  {
    // Getting frenet coordinates of detected vehicle
    double d = sensor_fusion[i][6];
    double s = sensor_fusion[i][5];

    // If detected vehicle is in the target lane
    if (d < (2 + 4 * target_lane + 2) && d > (2 + 4 * target_lane - 2))
    {
      // If Ego car is behind detected car
      if ((car_s + 15 > max_s && s < 10) && (fmod(car_s + s, max_s) < 10))
      {
        return false;
      }
      // If Detected car is behind ego car
      else if ((s + 10 > max_s && car_s < 22 && fmod(car_s + s, max_s) < 22))
      {
        return false;
      }
      // If Ego car is in front of detetected car
      else if ((car_s > s) && (car_s - s < 22))
      {
        return false;
      }
      // If detected car is in front of ego car
      else if ((s > car_s) && (s - car_s < 10))
      {
        return false;
      }
    }
  }

  return true;
}


int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // BEGIN PROJECT CODE //
          int lane = 1;
          bool changing_lane = false;

          // Target velocity (mph)
          double target_speed = 49.0;
          int prev_size = previous_path_x.size();

          double max_s = 6945.554;

          // If we have a previous path, use the end of that path as the "car's location"
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          double closest_car = std::numeric_limits<double>::max();

          for (int i = 0; i < sensor_fusion.size(); ++i)
          {
            // Getting detected car's lane
            float d = sensor_fusion[i][6];

            // If detected car is in Ego vehicle's lane
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double det_veh_speed = sqrt(vx * vx + vy * vy);
              double det_veh_s = sensor_fusion[i][5];

              // If we are using previous points, we can project location out in time
              det_veh_s += (static_cast<double>(prev_size) * 0.02 * det_veh_speed);
              // Resetting S coordinate if we exceeded max_s
              det_veh_s = fmod(det_veh_s, max_s);

              // If the detected vehicle is less than 25m in front of us
              if ((det_veh_s > car_s && det_veh_s - car_s < 25) ||
                  ((car_s + 25 > max_s) && fmod(car_s + 25, max_s) > det_veh_s))
              {

                // If the detected vehicle is the closest vehicle in front of the Ego vehicle
                // save it's distance
                if (det_veh_s < car_s)
                {
                  if (fmod(det_veh_s + car_s, max_s) < closest_car)
                  {
                    closest_car = max_s - car_s + det_veh_s;
                  }
                  else
                  {
                    continue;
                  }
                }
                // If the detected vehicle is the closest vehicle in front of the Ego vehicle
                // save it's distance
                else
                {
                  if (det_veh_s - car_s < closest_car)
                  {
                    closest_car = det_veh_s - car_s;
                  }
                  else
                  {
                    continue;
                  }
                }

                // If the detected vehicle is travelling slower than our target velocity
                if (det_veh_speed < 49.0 / 2.24)
                {
                  // change lane left
                  if (lane - 1 >= 0 && canChangeLanes(lane - 1, car_s, sensor_fusion))
                  {
                    lane = lane - 1;
                    target_speed = 49.0;
                  }
                  // change lane right
                  else if (lane + 1 <= 2 && canChangeLanes(lane + 1, car_s, sensor_fusion))
                  {
                    lane = lane + 1;
                    target_speed = 49.0;
                  }
                  // Accelerating
                  else if (closest_car > 30)
                  {
                    target_speed = 49;
                  }
                  // Decelerating
                  else
                  {
                    target_speed = (det_veh_speed * 2.24) - 5;
                  }
                }
              }
            }
          }

          vector<double> spline_x_vals;
          vector<double> spline_y_vals;

          double spline_x_1 = car_x;
          double spline_y_1 = car_y;
          double ego_yaw = deg2rad(car_yaw);
          double current_gap;

          if (prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            spline_x_vals.push_back(prev_car_x);
            spline_x_vals.push_back(car_x);
            spline_y_vals.push_back(prev_car_y);
            spline_y_vals.push_back(car_y);
            current_gap = 0.0;
          }
          else
          {
            spline_x_1 = previous_path_x[prev_size - 1];
            spline_y_1 = previous_path_y[prev_size - 1];
            double spline_x_0 = previous_path_x[prev_size - 2];
            double spline_y_0 = previous_path_y[prev_size - 2];
            ego_yaw = atan2(spline_y_1 - spline_y_0, spline_x_1 - spline_x_0);

            spline_x_vals.push_back(spline_x_0);
            spline_x_vals.push_back(spline_x_1);
            spline_y_vals.push_back(spline_y_0);
            spline_y_vals.push_back(spline_y_1);

            current_gap = sqrt(pow(spline_x_1 - spline_x_0, 2) + pow(spline_y_1 - spline_y_0, 2));
          }

          double dist_1 = fmod(car_s + 30, max_s);
          double dist_2 = fmod(car_s + 60, max_s);
          double dist_3 = fmod(car_s + 90, max_s);

          vector<double> spline_pt_0 =
              getXY(dist_1, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> spline_pt_1 =
              getXY(dist_2, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> spline_pt_2 =
              getXY(dist_3, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          spline_x_vals.push_back(spline_pt_0[0]);
          spline_x_vals.push_back(spline_pt_1[0]);
          spline_x_vals.push_back(spline_pt_2[0]);

          spline_y_vals.push_back(spline_pt_0[1]);
          spline_y_vals.push_back(spline_pt_1[1]);
          spline_y_vals.push_back(spline_pt_2[1]);

          for (int i = 0; i < spline_x_vals.size(); ++i)
          {
            double shift_x = spline_x_vals[i] - spline_x_1;
            double shift_y = spline_y_vals[i] - spline_y_1;
            spline_x_vals[i] = (shift_x * cos(0 - ego_yaw) - shift_y * sin(0 - ego_yaw));
            spline_y_vals[i] = (shift_x * sin(0 - ego_yaw) + shift_y * cos(0 - ego_yaw));
          }

          tk::spline spline;
          spline.set_points(spline_x_vals, spline_y_vals);

          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double current_x = 0;
          double target_gap = (target_speed / 2.24) * 0.02;

          for (int i = 1; i <= 50 - previous_path_x.size(); ++i)
          {

            double delta_gap = fabs(target_gap - current_gap) / 0.447 * 0.0035;

            if (fabs(target_gap - current_gap) > 0.005)
            {
              if (target_gap > current_gap)
              {
                current_gap += delta_gap;
              }
              else if (target_gap < current_gap)
              {
                current_gap -= delta_gap;
              }
            }

            double x_point = current_x + current_gap;
            double y_point = spline(x_point);
            current_x = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ego_yaw) - y_ref * sin(ego_yaw));
            y_point = (x_ref * sin(ego_yaw) + y_ref * cos(ego_yaw));

            x_point += spline_x_1;
            y_point += spline_y_1;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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