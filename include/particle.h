#ifndef PARTICLE_H
#define PARTICLE_H

#include <random>
#include <vector>
#include <cmath>
#include "robot.h"

class Particle : public Robot {

private:

    double weight;

public:

    Particle(double init_x, double init_y, double init_theta, double wheel_base, double init_weight = 1.0, 
             double noise_move = 0.0, double noise_steering = 0.0, double noise_sense = 0.0);

    void move(double dt, double velocity, double steering_angle) override;

    void update(double dt, double velocity, double steering_angle, const std::vector<std::pair<double, double>>& landmarks, const Robot& real_robot);

    void sense(const std::vector<std::pair<double, double>>& landmarks) override;

    void updateWeight(const Robot& real_robot); 

    void setWeight(double value) { weight = value; }

    double getWeight() const { return weight; }
};

#endif

