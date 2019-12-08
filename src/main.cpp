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
using std::fixed;
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
                    double v_max = 49.0 * 1609.34 / 3600;
                    double accel_max = 0.95*10;

                    double t_horz = 0.8;
                    double dt = 20.0e-3;
                    double dist_min = 1.5*v_max*t_horz;

                    double anchor_step = 30.0;
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

//                        cout << "sense: " << carlane << ", " << ess << ", " << ess_expect << ", " << car_s << endl;

                        double v_lane;
                        if ((fabs(car_s - ess) < 0.5*dist_min) && (carlane != lane)) {
                            v_lane = 0.0;
                            v_target_lanes[carlane] = v_lane;
                        }
                        // other car is already ahead, think about if it's safe to go behind it
                        else if (ess > car_s) {
                            v_lane = (ess_expect - dist_min - car_s)/t_horz;

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
                        // Other car is behind, but may be ahead, think about if it's safe to go in front of it.
                        // If we can get a set distance ahead of its expected position within current speed, it's save
                        else {
                            v_lane = (ess_expect + 2.0*dist_min - car_s)/t_horz;
                            // if it is too close, short circuit to zero

                            if (v_lane > v_target) {
                                v_lane = 0.0;
                            }
                        }
                    }

                    cout << "ln: " << lane << ", progress: ";
                    cout << setw(6) << fixed << setprecision(3) << car_s/1000.0 << "/" << max_s/1000.0;
                    cout << " --- limits: ";
                    for (int i = 0; i < v_target_lanes.size(); i++) {
                        cout << setw(8) << fixed << setprecision(2) << v_target_lanes[i]*3600.0/1609.34 << " ";
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

                        // Change lane if there is a better one, but only if we're pretty settled in our current one
                        if (best_lane != lane) {
                            double actual_d = lane*4.0 + 2.0;
                            if (fabs(actual_d - car_d) < 0.5) {
                                cout << "move to lane " << best_lane << " from " << lane << endl << endl;
                                prev_lane = lane;
                                lane = best_lane;
                            }
                        }
                    }
                    v_target = std::min(v_target_lanes[lane], v_max);

                    double next_d = lane*4 + 2;

                    vector<double> anchors_x;
                    vector<double> anchors_y;

                    double x_ref, y_ref, yaw_ref;

                    if (lane != prev_lane) {
//                        N = 2*N;
                        anchor_step = 1.5*anchor_step;
//                        accel_max = 0.75*accel_max;
                    }

                    // prepare first two anchor points for spline
                    // if we have enough previous path, use those,
                    // otherwise, infer car pose just before current pose
                    if (prevsize >= 2) {
                        x_ref = previous_path_x[prevsize-1];
                        y_ref = previous_path_y[prevsize-1];
                        double x_ref_prev = previous_path_x[prevsize-2];
                        double y_ref_prev = previous_path_y[prevsize-2];
                        yaw_ref = atan2(y_ref-y_ref_prev, x_ref-x_ref_prev);
                        v_ref = distance(x_ref, y_ref, x_ref_prev, y_ref_prev)/dt;

                        anchors_x.push_back(x_ref_prev);
                        anchors_x.push_back(x_ref);
                        anchors_y.push_back(y_ref_prev);
                        anchors_y.push_back(y_ref);
                    }
                    else {
                        x_ref = car_x;
                        y_ref = car_y;
                        double x_ref_prev = x_ref - cos(yaw_ref);
                        double y_ref_prev = y_ref - sin(yaw_ref);
                        yaw_ref = deg2rad(car_yaw);
                        v_ref = car_speed;

                        anchors_x.push_back(x_ref_prev);
                        anchors_x.push_back(x_ref);
                        anchors_y.push_back(y_ref_prev);
                        anchors_y.push_back(y_ref);
                    }

                    // add more anchors from several meters ahead in frenet frame
                    // choose something that's guaranteed to be ahead of the last on the
                    // previous path, so that the anchors x are monotonicly increasing
                    vector<double> sd_ref = getFrenet(x_ref, y_ref, yaw_ref, map_waypoints_x, map_waypoints_y);

                    vector<double> anchor_xy0 = getXY(sd_ref[0]+anchor_step, next_d, map_waypoints_s,
                                                      map_waypoints_x, map_waypoints_y);
                    vector<double> anchor_xy1 = getXY(sd_ref[0]+2*anchor_step, next_d, map_waypoints_s,
                                                      map_waypoints_x, map_waypoints_y);
                    vector<double> anchor_xy2 = getXY(sd_ref[0]+3*anchor_step, next_d, map_waypoints_s,
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

                    // setup additional waypoints from spline
                    double x_project = 30.0;
                    double y_project = spl(x_project);
                    double dist_project = distance(0.0, 0.0, x_project, y_project);

                    double v_ave = v_ref;
                    double dv = dt*accel_max;

                    if (v_ref + dv < v_target) {
                        v_ave += 0.5*dv;
                        v_ave = std::min(v_ave, v_max);
                    }
                    else if (v_ref - dv > v_target) {
                        v_ave -= 0.5*dv;
                        v_ave = std::max(0.0, v_ave);
                    }

                    double division = dist_project/(v_ave*dt);
                    double dx = x_project/division;
                    double x_loc = 0.0;

                    for (int i = 1; i <= N-prevsize; i++) {
                        x_loc += dx;
                        double y_loc = spl(x_loc);

                        if (v_ave + dv < v_target) {
                            v_ave += dv;
                            v_ave = std::min(v_ave, v_max);
                        }
                        else if (v_ave - dv > v_target) {
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

                    // calculate instantaneous acceleration in next set of waypoints
                    // this doesn't actually work
                    double x0, x1, x2, x3, y0, y1, y2, y3;
                    x0 = next_x_vals[0];
                    y0 = next_y_vals[0];
                    x1 = next_x_vals[1];
                    y1 = next_y_vals[1];
                    x2 = next_x_vals[2];
                    y2 = next_y_vals[2];

                    double dx1, dy1, dx2, dy2, dx3, dy3;
                    double vx1, vx2, vx3, vy1, vy2, vy3;
                    double ax, ay, a1;

                    dx1 = x1 - x0;
                    dy1 = y1 - y0;
                    dx2 = x2 - x1;
                    dy2 = y2 - y1;

                    vx1 = dx1/dt;
                    vy1 = dy1/dt;

                    vx2 = dx2/dt;
                    vy2 = dy2/dt;

                    ax = (vx2 - vx1)/dt;
                    ay = (vy2 - vy1)/dt;

                    a1 = sqrt(ax*ax + ay*ay);

                    double accmax = a1;
                    double accmin = a1;

                    for (int i = 3; i < next_x_vals.size(); i++) {
                        x3 = next_x_vals[i];
                        y3 = next_y_vals[i];

                        dx3 = x3 - x2;
                        dy3 = y3 - y2;

                        vx3 = dx3/dt;
                        vy3 = dy3/dt;

                        ax = (vx3 - vx2)/dt;
                        ay = (vy3 - vy2)/dt;

                        a1 = sqrt(ax*ax + ay*ay);

                        if (a1> accmax) {
                            accmax = a1;
                        }
                        else if (a1 < accmin) {
                            accmin = a1;
                        }

                        x0 = x1;
                        y0 = y1;
                        x1 = x2;
                        y1 = y2;
                        x2 = x3;
                        y2 = y3;

                        dx2 = x2 - x1;
                        dy2 = y2 - y1;

                        vx2 = dx2/dt;
                        vy2 = dy2/dt;
                    }
//                    cout << "accels: " << setw(6) << setprecision(4) << accmin << ", ";
//                    cout << setw(6) << setprecision(4) << accmax << endl;

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
