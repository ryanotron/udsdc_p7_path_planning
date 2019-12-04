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
using std::cout;
using std::endl;

int main() {
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
    while (getline(in_map_, line)) {
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

    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
                &map_waypoints_dx,&map_waypoints_dy]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
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

                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */

//                    cout << "new cycle: " << car_s << ", " << car_d << endl;
                    double v_ref = 0.80 * 50.0 * 1609.34 / 3600;
                    double t_horz = 1.0;
                    double dt = 20.0e-3;
                    double lane = 1;

                    int N = (int) (t_horz/dt);

                    double next_d = lane*4 + 2;

                    vector<double> anchors_x;
                    vector<double> anchors_y;

                    double x_ref, y_ref, yaw_ref;

                    // prepare first two anchor points for spline
                    // if we have enough previous path, use those,
                    // otherwise, infer car pose just before current pose
                    int prevsize = previous_path_x.size();
                    if (prevsize >= 2) {
                        x_ref = previous_path_x[prevsize-1];
                        y_ref = previous_path_y[prevsize-1];

                        double x_ref_prev = previous_path_x[prevsize-2];
                        double y_ref_prev = previous_path_y[prevsize-2];

                        yaw_ref = atan2(y_ref-y_ref_prev, x_ref-x_ref_prev);

                        anchors_x.push_back(previous_path_x[0]);
                        anchors_x.push_back(x_ref);
                        anchors_y.push_back(previous_path_y[0]);
                        anchors_y.push_back(y_ref);
                    }
                    else {
                        x_ref = car_x;
                        y_ref = car_y;
                        yaw_ref = deg2rad(car_yaw);

                        double x_ref_prev = x_ref - cos(yaw_ref);
                        double y_ref_prev = y_ref - sin(yaw_ref);

                        anchors_x.push_back(x_ref_prev);
                        anchors_x.push_back(x_ref);
                        anchors_y.push_back(y_ref_prev);
                        anchors_y.push_back(y_ref);
                    }

                    cout << prevsize << ": ref pose (x, y, yaw): (" << x_ref << ", "
                         << y_ref << ", " << yaw_ref << ")" << endl;

                    // add more anchors from several meters ahead in frenet frame
                    // choose something that's guaranteed to be ahead of the last on the
                    // previous path, so that the anchors x are monotonicly increasing
//                    vector<double> sd_ref = getFrenet(x_ref, y_ref, yaw_ref, map_waypoints_x,
//                                                      map_waypoints_y);
//                    cout << "car_s, s_ref " << car_s << ", " << sd_ref[0] << endl;

//                    vector<double> anchor_xy0 = getXY(sd_ref[0]+30, next_d, map_waypoints_s,
//                                                      map_waypoints_x, map_waypoints_y);
//                    vector<double> anchor_xy1 = getXY(sd_ref[0]+60, next_d, map_waypoints_s,
//                                                      map_waypoints_x, map_waypoints_y);
//                    vector<double> anchor_xy2 = getXY(sd_ref[0]+90, next_d, map_waypoints_s,
//                                                      map_waypoints_x, map_waypoints_y);

//                    anchors_x.push_back(anchor_xy0[0]);
//                    anchors_x.push_back(anchor_xy1[0]);
//                    anchors_x.push_back(anchor_xy2[0]);

//                    anchors_y.push_back(anchor_xy0[1]);
//                    anchors_y.push_back(anchor_xy1[1]);
//                    anchors_y.push_back(anchor_xy2[1]);

                    // prepare anchors from closest map waypoints to reference
                    // starting from one behind up to fourth ahead (so, five anchors)
                    int next_wp = NextWaypoint(x_ref, y_ref, yaw_ref,
                                               map_waypoints_x, map_waypoints_y);

                    int mapsize = map_waypoints_x.size();
                    for (int i = 1; i < 4; i++) {
                        int idx = (next_wp + i + mapsize) % mapsize;
                        vector<double> xy = getXY(map_waypoints_s[idx], next_d, map_waypoints_s,
                                                  map_waypoints_x, map_waypoints_y);
                        anchors_x.push_back(xy[0]);
                        anchors_y.push_back(xy[1]);
                    }

                    // transfer anchors to car-local reference frame
                    for (int i = 0; i < anchors_x.size(); i++) {
                        double x_inter = anchors_x[i] - x_ref;
                        double y_inter = anchors_y[i] - y_ref;

                        double x_loc = cos(0-yaw_ref)*x_inter - sin(0-yaw_ref)*y_inter;
                        double y_loc = sin(0-yaw_ref)*x_inter + cos(0-yaw_ref)*y_inter;

                        anchors_x[i] = x_loc;
                        anchors_y[i] = y_loc;
                    }

                    // setup spline
                    tk::spline spl;
                    spl.set_points(anchors_x, anchors_y);

                    // setup next waypoints
                    // start with remaining previous waypoints,
                    // then add new ones from the spline
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for (int i = 0; i < prevsize; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // new waypoints from spline, each about v_ref*dt apart
                    // keep adding until we have N total waypoints
                    // remember: N was chosen so that we plan until a time horizon,
                    // assuming the car is moving at max speed
                    double xi = 0.0;
                    double yi = 0.0;
                    double psi = 0.0;
                    for (int i = 1; i <= N-prevsize; i++) {
                        double xf = xi + v_ref*dt*cos(psi);
                        double yf = spl(xf);

                        // transform to global frame
                        double xf_glo = cos(yaw_ref)*xf - sin(yaw_ref)*yf;
                        double yf_glo = sin(yaw_ref)*xf + cos(yaw_ref)*yf;

                        xf_glo += x_ref;
                        yf_glo += y_ref;

                        next_x_vals.push_back(xf_glo);
                        next_y_vals.push_back(yf_glo);

                        // setup heading for next iteration
                        psi = atan2(yf-yi, xf-xi);
                        xi = xf;
                        yi = yf;
                    }

//                    cout << endl << "global waypoints" << endl;
//                    for (int i = 0; i < next_x_vals.size(); i++) {
//                        cout << i << ", " << next_x_vals[i] << ", " << next_y_vals[i] << endl;
//                    }
//                    cout << "***" << endl << endl;

//                    double xi, yi, xf, yf;
//                    xi = 0;
//                    yi = 0;
//                    xf = 5;
//                    yf = spl(xf);

//                    double distf = distance(xi, yi, xf, yf);
//                    double div = distf/(dt*v_ref);

//                    for (int i = 1; i <= N-prevsize; i++) {
//                        xf = xi + distf/div;
//                        yf = spl(xf);

//                        xi = xf;

//                        // transfer back to global frame
//                        double x_glo = cos(yaw_ref)*xf - sin(yaw_ref)*yf + x_ref;
//                        double y_glo = sin(yaw_ref)*xf + cos(yaw_ref)*yf + y_ref;

//                        next_x_vals.push_back(x_glo);
//                        next_y_vals.push_back(y_glo);
//                    }


                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

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
