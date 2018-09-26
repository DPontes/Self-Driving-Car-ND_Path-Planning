#include "vehicle.h"
#include <iostream>
#include <math.h>
#include "cost_function.h"

const double MS_TO_MPH = 2.23694;
const double MPH_TO_MS = 0.44704;

Vehicle::Vehicle(int lane, double target_speed) {
    ref_speed = target_speed;
    ref_lane = lane;
}

void Vehicle::Update(double ax,
                     double ay,
                     double as,
                     double ad,
                     double ayaw,
                     double aspeed,
                     int lane,
                     double target_speed,
                     double delta) {
    x = ax;
    y = ay;
    s = as;
    d = ad;
    yaw = ayaw;
    speed = aspeed;
    delta_time = delta;
    ref_speed = target_speed;
    ref_lane = lane;

    // Clean data
    _reset_data();
}

void Vehicle::_reset_data() {
    // Reset trajectory
    trajectory.lane_start = ref_lane;
    trajectory.lane_end = ref_lane;
    trajectory.target_speed = ref_speed;

    // Reset update
    update.ref_v = ref_speed;
    update.lane = ref_lane;
    update.target_v = 49.50;
    collider.collision = false;
    collider.distance = 10000;
    collider.closest_approach = 10000;
    collider.target_speed = 0;
}

void Vehicle::NextState(vector<vector<double>> sensor) {
    States current_state = state;
    vector<States> states;
    // select reachable states
    states.push_back(KL);
    if(state == PLCL) {
        states.push_back(LCL);
        states.push_back(PLCL);
    } else if(state == PLCR) {
        states.push_back(LCR);
        states.push_back(PLCR);
    } else {
        if(ref_lane != 0) {
            // check if lane change is over before LCL again
            if(d<(2+4*(ref_lane)+2) && d>(2+4*(ref_lane)-2) && speed > 20) {
                states.push_back(PLCL);
            }
        }
        if(ref_lane != 2) {
            // check if lane change is over before LCR again
            if(d<(2+4*(ref_lane)+2) && d>(2+4*(ref_lane)-2) && speed > 20) {
                states.push_back(PLCR);
            }
        }
    }

    States min_state = KL;
    double min_cost = 10000000;

    // Compute cost of all reachable states
    for( int i = 0; i < states.size(); i++) {
        States n_state = states[i];
        // Prepare state
        _reset_data();
        _realize_state(n_state, sensor);
        CostFunction cost = CostFunction(this, sensor);
        double value = cost.Compute();
        if(value < min_cost) {
            min_state = n_state;
            min_cost = value;
        }
    }

    // Update state
    state = min_state;
    _reset_data();
    _realize_state(state, sensor);

    // Update speed
    CostFunction cost = CostFunction(this, sensor);
    float v = cost.Compute();
    if(!collider.collision &&
        ref_speed < update.target_v &&
        ref_speed < 49.5) {
        update.ref_v += 0.224;
    } else if(ref_speed > update.target_v &&
              ref_speed > 0) {
        update.ref_v -= 0.224;
    }

    std::cout << "NEW STATE " << state << " with cost " << min_cost << "\n";
}

void Vehicle::_realize_state(States astate, vector<vector<double>> sensor_fusion) {
    state = astate;
    switch(state) {
        case KL: {
            // Same lane
            trajectory.lane_start = ref_lane;
            trajectory.lane_end = ref_lane;
            update.lane = ref_lane;
            break;
        }
        case PLCL: {
            // Same lane
            trajectory.lane_start = ref_lane;
            trajectory.lane_end = ref_lane - 1;
            update.lane = ref_lane;
            break;
        }
        case LCL: {
            // Same lane
            trajectory.lane_start = ref_lane;
            trajectory.lane_end = ref_lane - 1;
            update.lane = ref_lane - 1;
            break;
        }
        case PLCR: {
            // Same lane
            trajectory.lane_start = ref_lane;
            trajectory.lane_end = ref_lane + 1;
            update.lane = ref_lane;
            break;
        }
        case LCR: {
            // Same lane
            trajectory.lane_start = ref_lane;
            trajectory.lane_end = ref_lane + 1;
            update.lane = ref_lane + 1;
            break;
        }
        default:
            std::cout << "STATE ERROR\n";
    }

    // Check lane
    if(trajectory.lane_end < 0) {
        trajectory.lane_end = 0;
    } else if(trajectory.lane_end > 2) {
        trajectory.lane_end = 2;
    }

    if(trajectory.lane_start < 0) {
        trajectory.lane_start = 0;
    } else if(trajectory.lane_start > 2) {
        trajectory.lane_start = 2;
    }

    if(update.lane < 0) {
        update.lane = 0;
    } else if(update.lane > 2) {
        update.lane = 2;
    }

    double target_speed_front = 0;
    double target_distance_front = 10000;
    double target_speed_lane_front = 0;
    double target_distance_lane_front = 10000;
    double target_speed_lane_back = 0;
    double target_distance_lane_back = -10000;

    // Compute collision on start and end lane
    for (int i = 0; i < sensor_fusion.size(); i++) {
        // Car is in my lane
        float car_d = sensor_fusion[i][6];
        // Safety check for speed of car in front
        if(car_d < (2+4*(trajectory.lane_start)+2) &&
            car_d > (2+4*(trajectory.lane_start)-2)) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double) delta_time * check_speed);

            // Check s values greater than mine and s gap
            double dist_to_collision = (check_car_s - s);
            if((check_car_s >= s) && (dist_to_collision < 30)) {
                if(target_distance_front > dist_to_collision) {
                    target_speed_front = check_speed * MS_TO_MPH - 2;
                    target_distance_front = dist_to_collision;
                }
            }
        }

        // Check for car in the current lane
        if(car_d < (2+4*(trajectory.lane_end)+2) &&
            car_d > (2+4*(trajectory.lane_end)-2)) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double) delta_time * check_speed);

            // Check s values greater than mine and s gap
            double dist_to_collision = (check_car_s - s);
            if((trajectory.lane_end != trajectory.lane_start &&
                (abs(dist_to_collision) < 30)) ||
                ((check_car_s >= s) &&
                (dist_to_collision < 30))) {
                if(collider.distance > abs(dist_to_collision)) {
                    collider.distance = abs(dist_to_collision);
                    collider.collision = true;
                    collider.closest_approach = abs(dist_to_collision);
                    collider.target_speed = check_speed * MS_TO_MPH;

                    if(abs(dist_to_collision) > 30) {
                        // Change the target speed
                        if(check_car_s >= s) {
                            // Car in front
                            update.target_v = check_speed * MS_TO_MPH-2;
                            if(target_distance_lane_front > dist_to_collision){
                                target_speed_lane_front = check_speed *
                                                                    MS_TO_MPH;
                                target_distance_lane_front = dist_to_collision;
                            }
                        } else {
                            // Car in back
                            update.target_v = check_speed * MS_TO_MPH+2;
                            if(target_distance_lane_back < dist_to_collision) {
                                target_speed_lane_back = check_speed *
                                                                    MS_TO_MPH;
                                target_distance_lane_back = dist_to_collision;
                            }
                        }
                    }
                }
            } else if (!collider.collision &&
                        collider.closest_approach > dist_to_collision) {
                collider.closest_approach = dist_to_collision;
                collider.target_speed = check_speed * MS_TO_MPH;
            }
        }
    }

    // Safety Speed Check
    if(state == PLCL || state == PLCR) {
        // Safety speed adjust
        if(target_speed_lane_back != 0 
            && update.target_v < target_speed_lane_back) {
            update.target_v = target_speed_lane_back;
        }
        if(target_speed_lane_front != 0 
            && update.target_v > target_speed_lane_front) {
            update.target_v = target_speed_lane_front;
        }
    }

    if(target_speed_front != 0 && update.target_v > target_speed_front) {
        update.target_v = target_speed_front - 2;
    }
}
