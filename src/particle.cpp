#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include "robot.h"
#include "particle.h"

Particle::Particle(double init_x, double init_y, double init_theta, double wheel_base, double init_weight, 
             double noise_move, double noise_steering, double noise_sense)
        :Robot(init_x, init_y, init_theta, wheel_base), weight(init_weight) {}

void Particle::move(double dt, double velocity, double steering_angle) {

    x += velocity * cos(theta) * dt;
    y += velocity * sin(theta) * dt;
    theta += (velocity / wheel_base) * tan(steering_angle) * dt;
}

void Particle::update(double dt, double velocity, double steering_angle, const std::vector<std::pair<double, double>>& landmarks, const Robot& real_robot){

    move(dt, velocity, steering_angle);
    sense(landmarks);
}

void Particle::sense(const std::vector<std::pair<double, double>>& landmarks) { 

    z.clear();

    for (const auto& landmark : landmarks) {
        double dx = landmark.first - x; //вычисляем вектора
        double dy = landmark.second - y;

        // Вычисляем пеленг с помощью atan2
        double bearing = atan2(dy, dx) - theta;  // Пеленг относительно угла частицы
        
        while (bearing > M_PI) bearing -= 2.0 * M_PI;
        while (bearing < -M_PI) bearing += 2.0 * M_PI;

        z.push_back(bearing);
    }

}