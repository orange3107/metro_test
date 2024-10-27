#include "robot.h"
#include <iostream>
#include <cmath>
#include <random>

Robot::Robot(double init_x, double init_y, double init_theta, double wheel_base, double noise_move, double noise_steering, double noise_sense)
    : x(init_x), y(init_y), theta(init_theta), wheel_base(wheel_base), noise_move(noise_move), noise_steering(noise_steering), noise_sense(noise_sense), generator(std::random_device{}()){}

Robot::Robot(double init_x, double init_y, double init_theta, double wheel_base)
    : x(init_x), y(init_y), theta(init_theta), wheel_base(wheel_base) {}

// Обновление состояния робота
void Robot::move(double dt, double velocity, double steering_angle) {

    std::normal_distribution<double> steering_dist(steering_angle, noise_steering);
    std::normal_distribution<double> velocity_dist(velocity, noise_move);

    double noisy_steering_angle = steering_dist(generator);
    double noisy_velocity = velocity_dist(generator);

    x += noisy_velocity * cos(theta) * dt;
    y += noisy_velocity * sin(theta) * dt;
    theta += (noisy_velocity / wheel_base) * tan(noisy_steering_angle) * dt;
}

void Robot::sense(const std::vector<std::pair<double, double>>& landmarks) { 

    z.clear();

    for (const auto& landmark : landmarks) {
        double dx = landmark.first - x; //вычисляем вектора
        double dy = landmark.second - y;

        // Вычисляем пеленг с помощью atan2
        double bearing = atan2(dy, dx) - theta;  // Пеленг относительно угла робота
        
        while (bearing > M_PI) bearing -= 2.0 * M_PI;
        while (bearing < -M_PI) bearing += 2.0 * M_PI;

        std::normal_distribution<double> sense_dist(bearing, noise_sense);

        bearing = sense_dist(generator);

        z.push_back(bearing);
        
    }

}

void Robot::update(double dt, double velocity, double steering_angle, const std::vector<std::pair<double, double>>& landmarks){
    move(dt, velocity, steering_angle);
    sense(landmarks);
}

void Robot::measurement_prob(const Robot& other){

    z_prob.clear();

    double noise_sense_robot = other.noise_sense;

    for (int i = 0; i < z.size(); ++i){

        double prob = (1.0 / std::sqrt(2.0 * M_PI * noise_sense_robot)) * (std::exp(-std::pow(z[i] - other.z[i], 2) / pow(noise_sense_robot, 1)));
        z_prob.push_back(prob);
    }

}   

void Robot::printState() const {
    std::cout << "X: " << x << ", Y: " << y << ", Theta: " << theta << std::endl;
}
