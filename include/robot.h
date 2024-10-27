#ifndef ROBOT_H
#define ROBOT_H

#include <random>
#include <vector>
#include <cmath>

class Robot {
protected:
    double x, y, theta, wheel_base;
    double noise_move, noise_steering, noise_sense;
    std::default_random_engine generator;
    
public:

    std::vector<double> z_prob;
    std::vector<double> z;
    // Конструктор
    Robot(double init_x, double init_y, double init_theta, double wheel_base, double noise_move, double noise_steering, double noise_sense);

    Robot(double init_x, double init_y, double init_theta, double wheel_base);

    virtual void move(double dt, double velocity, double steering_angle);

    virtual void sense(const std::vector<std::pair<double, double>>& landmarks);

    void update(double dt, double velocity, double steering_angle, const std::vector<std::pair<double, double>>& landmarks);

    void measurement_prob(const Robot& other);

    double getX() const { return x; }
    double getY() const { return y; }
    double getTheta() const { return theta; }
    double getWheelBase() const { return wheel_base; }

    void setX(double value) { x = value; }
    void setY(double value) { y = value; }
    void setTheta(double value) { theta = value; }

    void printState() const;
};

#endif
