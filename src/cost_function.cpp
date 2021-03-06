#include <math.h>
#include <iostream>
#include "./vehicle.h"
#include "./cost_function.h"

const double MS_TO_MPH = 2.23694;
const double MPH_TO_MS = 0.44704;

CostFunction::CostFunction(Vehicle *v, vector<vector<double>> s) {
    vehicle = v;
    sensor_fusion = s;
}

double CostFunction::Compute() {
    // compute cost
    double cost = 0;
    cost += ChangeLane();
    cost += Inefficiency();
    cost += Collision();
    cost += Buffer();
    cost += Target();

    return cost;
}

double CostFunction::Target() {
    double cost = 0;
    if (!vehicle -> collider.collision) {
        return 0;
    }

    int end_lane = vehicle -> trajectory.lane_end;
    int start_lane = vehicle -> trajectory.lane_start;
    double diff = (vehicle -> collider.target_speed - vehicle -> speed) /
                    vehicle -> collider.target_speed;
    cost = pow(diff, 2) * EFFICIENCY;
    return cost;
}

double CostFunction::ChangeLane() {
    // Compute cost to change lane, penalizes lane Away from the leftiest lane
    int end_lane = vehicle -> trajectory.lane_end;
    int start_lane = vehicle -> trajectory.lane_start;
    double cost = 0;
    if (start_lane != end_lane) {
        cost += COMFORT;
    }
    return cost;
}

double CostFunction::Inefficiency() {
    // The best efficiency is when the speed is closest to the limit
    double cost = 0;
    double diff = (49.5 - vehicle->update.target_v) / 49.5;
    cost = pow(diff, 2) * EFFICIENCY;
    return cost;
}

double CostFunction::Collision() {
    double cost = 0;
    if (vehicle->collider.collision) {
        // distance divided by the relative speed
        double time_to_collide = abs(vehicle->collider.distance) /
                                 (abs(vehicle->speed) * MPH_TO_MS);
        cost = exp(-pow(time_to_collide, 2)) * COLLISION;
        // Changing lane
        if (vehicle->trajectory.lane_end != vehicle->trajectory.lane_start) {
            if (time_to_collide > DESIRED_BUFFER) {
                // Safe to change lane
                cost /= 10;
            }
        }
    }
    return 0;
}

double CostFunction::Buffer() {
    double cost = 0;
    if (vehicle->collider.closest_approach == 10000) {
        return 0;
    }

    double time_steps = abs(vehicle->collider.closest_approach) /
                        (abs(vehicle->speed) * MPH_TO_MS);

    if (time_steps > DESIRED_BUFFER) {
        return 0;
    }

    double multiplier = 1.0 - pow((time_steps / DESIRED_BUFFER), 2);
    cost = multiplier * DANGER;
    if (vehicle->collider.closest_approach < 0) {
        cost /= 10;
    }
    return cost;
}
