#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <iomanip>
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
using std::setprecision;
using std::setw;

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
                    vector< vector<double> > sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    // START PARAMETERS ============================================+
                    double max_s = 6945.554;
                    double v_max = 0.99 * 50.0 * 1609.34 / 3600;
                    double accel_max = 0.50*10;

                    double t_horz = 0.8;
                    double dt = 20.0e-3;
                    double dist_min = 30.0;
                    // END PARAMETERS ===============================================

                    static double v_target = v_max;
                    static int lane = 1;
                    int prev_lane = lane;

                    double v_ref;
                    int N = (int) (t_horz/dt);

                    vector<double> v_target_lanes (3, 5*v_max);

                    int prevsize = previous_path_x.size();
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for (int i = 0; i < prevsize; i++) {
                        try {
                            next_x_vals.push_back(previous_path_x[i]);
                            next_y_vals.push_back(previous_path_y[i]);
                        }
                        catch (json::type_error& e) {
                            cout << endl << endl << "NO!" << " " << e.what() << endl << endl;
                            prevsize = i;
                            break;
                        }
                    }

                    for (int i = 0; i < sensor_fusion.size(); i++) {
                        auto carspec = sensor_fusion[i];
                        double dee = carspec[6];

                        // find lane index of the car under consideration
                        // and assume it's in rightmost lane if it happens to be
                        // further right (offroading?)
                        int carlane = (int) floor(dee/4);
                        carlane = std::min(carlane, 2);

                        // Check if the car is ahead from us (in s).
                        // Then calculate its distance after time horizon,
                        // assuming we are in the same lane.
                        // If it is shorter than the `dist_min` parameter,
                        // calculate maximum speed we can have if we were behind it.
                        // Register this speed in `v_target_lanes` vector.
                        // If we can't run at max speed in this lane, check if we can
                        // run faster in one of the adjacent lanes.
                        // If yes, move there
                        double ess = carspec[5];
                        double vx = carspec[3];
                        double vy = carspec[4];
                        double vee = sqrt(vx*vx + vy*vy);

                        double ess_expect = ess + vee*t_horz;

                        // Edge case: lead car is just across the lap,
                        // and its projected pose is a small s,
                        // while we are just before the lap, big s.
                        // We want the calculation to say that the lead car
                        // is ahead of us and relevant for lane speed calculation.
                        // To detect this condition, all these must be true:
                        // - excpected lead car pose is lower than our pose in s
                        // - lead car pose + lap - our pose < our pose - lead car
                        //   (absolute distance is shorter if we pretend
                        //   the lap is longer)
                        if ((ess_expect < car_s) && ((ess_expect + max_s - car_s) < (car_s - ess_expect))) {
                            ess_expect += max_s;
                        }
                        if ((ess < car_s) && ((ess + max_s - car_s) < (car_s - ess))) {
                            ess += max_s;
                        }

                        // other car is already ahead, think about if it's safe to go behind it
                        if (ess > car_s) {
                            double v_lane = (ess_expect - dist_min - car_s)/t_horz;

                            if (v_lane < 0) {
                                if (carlane == lane) {
                                    v_lane = vee;
                                }
                                else {
                                    v_lane = 0.0;
                                }
                            }

                            if (v_lane < v_target_lanes[carlane]) {
                                v_target_lanes[carlane] = v_lane;
                            }
                        }
                        // other car is behind, but will be ahead, think about if it's safe to go in front of it
                        else if (ess_expect > car_s) {
                            double v_lane = (ess_expect + 0.50*dist_min - car_s)/t_horz;
                            if (v_lane > v_max) {
                                v_lane = 0.0;
                            }
                        }
                    }

                    cout << "lane: " << lane << ", speed: " << setw(8) << setprecision(6) << car_speed << " --- lane limits: ";
                    for (int i = 0; i < v_target_lanes.size(); i++) {
                        cout << setw(8) << setprecision(6) << v_target_lanes[i]*3600.0/1609.34 << " ";
                    }
                    cout << endl;

                    if (v_target_lanes[lane] < v_max) {
                        int neighbours[2] = {-1, 1};
                        int best_lane = lane;

                        for (int i = 0; i < 2; i++) {
                            int nix = lane + neighbours[i];
                            if (nix >= 0 && nix <= 2) {
                                if (v_target_lanes[nix] > 1.5*v_target_lanes[best_lane]) {
                                    best_lane = nix;
                                }
                            }
                        }

                        if (best_lane != lane) {
                            cout << "move to lane " << best_lane << " from " << lane << endl << endl;
                        }
                        prev_lane = lane;
                        lane = best_lane;
                    }
                    v_target = std::min(v_target_lanes[lane], v_max);

                    double next_d = lane*4 + 2;

                    vector<double> anchors_x;
                    vector<double> anchors_y;

                    double x_ref, y_ref, yaw_ref;

                    // prepare first two anchor points for spline
                    // if we have enough previous path, use those,
                    // otherwise, infer car pose just before current pose

                    if (lane != prev_lane) {
                        N = 2*N;
                    }
                    if (prevsize >= 2) {
                        x_ref = previous_path_x[prevsize-1];
                        y_ref = previous_path_y[prevsize-1];

                        double x_ref_prev = previous_path_x[prevsize-2];
                        double y_ref_prev = previous_path_y[prevsize-2];

                        yaw_ref = atan2(y_ref-y_ref_prev, x_ref-x_ref_prev);

                        v_ref = distance(x_ref, y_ref, x_ref_prev, y_ref_prev)/dt;

//                        anchors_x.push_back(previous_path_x[0]);
                        anchors_x.push_back(x_ref_prev);
                        anchors_x.push_back(x_ref);
//                        anchors_y.push_back(previous_path_y[0]);
                        anchors_y.push_back(y_ref_prev);
                        anchors_y.push_back(y_ref);
                    }
                    else {
                        x_ref = car_x;
                        y_ref = car_y;
                        yaw_ref = deg2rad(car_yaw);

                        v_ref = car_speed;

                        double x_ref_prev = x_ref - cos(yaw_ref);
                        double y_ref_prev = y_ref - sin(yaw_ref);

                        anchors_x.push_back(x_ref_prev);
                        anchors_x.push_back(x_ref);
                        anchors_y.push_back(y_ref_prev);
                        anchors_y.push_back(y_ref);
                    }

                    // add more anchors from several meters ahead in frenet frame
                    // choose something that's guaranteed to be ahead of the last on the
                    // previous path, so that the anchors x are monotonicly increasing
                    vector<double> sd_ref = getFrenet(x_ref, y_ref, yaw_ref, map_waypoints_x,
                                                      map_waypoints_y);

                    vector<double> anchor_xy0 = getXY(sd_ref[0]+30, next_d, map_waypoints_s,
                                                      map_waypoints_x, map_waypoints_y);
                    vector<double> anchor_xy1 = getXY(sd_ref[0]+60, next_d, map_waypoints_s,
                                                      map_waypoints_x, map_waypoints_y);
                    vector<double> anchor_xy2 = getXY(sd_ref[0]+90, next_d, map_waypoints_s,
                                                      map_waypoints_x, map_waypoints_y);

                    anchors_x.push_back(anchor_xy0[0]);
                    anchors_x.push_back(anchor_xy1[0]);
                    anchors_x.push_back(anchor_xy2[0]);

                    anchors_y.push_back(anchor_xy0[1]);
                    anchors_y.push_back(anchor_xy1[1]);
                    anchors_y.push_back(anchor_xy2[1]);

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


                    double x_project = 30.0;
                    double y_project = spl(x_project);
                    double dist_project = distance(0.0, 0.0, x_project, y_project);

                    double v_ave = v_ref;
                    double dv = dt*accel_max;

                    if (v_ref < v_target) {
                        v_ave += 0.5*dv;
                        v_ave = std::min(v_ave, v_max);
                    }
                    else if (v_ref > v_target) {
                        v_ave -= 0.5*dv;
                        v_ave = std::max(0.0, v_ave);
                    }

                    double division = dist_project/(v_ave*dt);
                    double dx = x_project/division;
                    double x_loc = 0.0;

                    for (int i = 1; i <= N-prevsize; i++) {
                        x_loc += dx;
                        double y_loc = spl(x_loc);

                        if (v_ave < v_target) {
                            v_ave += dv;
                            v_ave = std::min(v_ave, v_max);
                        }
                        else if (v_ave > v_target) {
                            v_ave -= dv;
                            v_ave = std::max(0.0, v_ave);
                        }

                        dist_project = distance(x_loc, y_loc, x_project, y_project);
                        division = dist_project/(v_ave*dt);
                        dx = (x_project-x_loc)/division;

                        double x_glo, y_glo;
                        x_glo = cos(yaw_ref)*x_loc - sin(yaw_ref)*y_loc;
                        y_glo = sin(yaw_ref)*x_loc + cos(yaw_ref)*y_loc;

                        x_glo += x_ref;
                        y_glo += y_ref;

                        next_x_vals.push_back(x_glo);
                        next_y_vals.push_back(y_glo);
                    }

                    double d1, d2, d3;
                    double x0, x1, x2, x3, y0, y1, y2, y3;
                    x0 = next_x_vals[0];
                    y0 = next_y_vals[0];
                    x1 = next_x_vals[1];
                    y1 = next_y_vals[1];
                    x2 = next_x_vals[2];
                    y2 = next_y_vals[2];

                    d1 = distance(x0, y0, x1, y1);
                    d2 = distance(x1, y1, x2, y2);

                    double accmax = 0.0;
                    double accmin = 0.0;

                    for (int i = 3; i < next_x_vals.size(); i++) {
                        x3 = next_x_vals[i];
                        y3 = next_y_vals[i];

                        d3 = distance(x2, y2, x3, y3);
                        double accel = (d3 - 2*d2 + d1)/(dt*dt);
                        if (accel > accmax) {
                            accmax = accel;
                        }
                        else if (accel < accmin) {
                            accmin = accel;
                        }

                        x0 = x1;
                        y0 = y1;
                        x1 = x2;
                        y1 = y2;
                        x2 = x3;
                        y2 = y3;
                        d1 = d2;
                        d2 = d3;
                    }
//                    cout << "accels: " << setw(6) << setprecision(4) << accmin << ", ";
//                    cout << setw(6) << setprecision(4) << accmax << endl;

                    // Uncomment next block to debug next waypoint generation
//                    for (int i = 0; i < next_x_vals.size(); i++) {
//                        cout << i << ": " << next_x_vals[i] << ", " << next_y_vals[i] << endl;
//                    }
//                    cout << "---" << endl;


                    // new waypoints from spline, each about v_ref*dt apart
                    // keep adding until we have N total waypoints
                    // remember: N was chosen so that we plan until a time horizon,
                    // assuming the car is moving at max speed
//                    double xi = 0.0;
//                    double yi = 0.0;
//                    double psi = 0.0;
//                    for (int i = 0; i < N-prevsize; i++) {
//                        double dee = v_ref*dt;
//                        if (dee < 1.0e-3) {
//                            dee = 0.5*accel_max*dt*dt;
//                        }
//                        double dx = dee*cos(psi);
//                        double xf = xi + dx;
//                        double yf = spl(xf);
//                        double dee_inter = distance(xi, yi, xf, yf);

//                        while (dee_inter - dee > 1.0e-3) {
//                            psi = atan2(yf-yi, dx);
//                            xf = xi + 0.9*dee_inter*cos(psi);
//                            yf = spl(xf);
//                            dee_inter = distance(xi, yi, xf, yf);
//                            cout << dee << ", " << dee_inter << endl;
//                        }

//                        if (v_ref < v_target) {
//                            v_ref += accel_max*dt;
//                            v_ref = std::min(v_max, v_ref);
//                        }
//                        else if (v_ref > v_target) {
//                            v_ref -= accel_max*dt;
//                            v_ref = std::max(0.0, v_ref);
//                        }

//                        double x_glo = cos(yaw_ref)*xf - sin(yaw_ref)*yf;
//                        double y_glo = sin(yaw_ref)*xf + cos(yaw_ref)*yf;

//                        x_glo += x_ref;
//                        y_glo += y_ref;

//                        next_x_vals.push_back(x_glo);
//                        next_y_vals.push_back(y_glo);

//                        xi = xf;
//                        yi = yf;
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
