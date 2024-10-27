#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle.h"
#include "robot.h"
#include <vector>
#include <random>

class ParticleFilter {
private:
    std::vector<Particle> particles; // Вектор частиц (гипотез)
    int num_particles;               // Число частиц
    std::default_random_engine generator;
    double noise_part;
    std::vector<std::pair<double, double>> landmarks;
    bool first = true;
    

public:
    // Конструктор
    ParticleFilter(const Robot& real_robot_, int num_particles_, double noise_part, const std::vector<std::pair<double, double>>& landmarks_);
    void updateWeights(const Robot& real_robot);

    // Обновление фильтра частиц (перемещение, измерения, обновление весов)
    void update(double dt, double velocity, double steering_angle, const std::vector<std::pair<double, double>>& landmarks, const Robot& real_robot);

    void resample();

    void setX(double x) { x = x; }
    void setY(double y) { y = y; }
    void setTheta(double theta) { theta = theta; }

    void getEstimate(double& e_x, double& e_y, double& e_th);

};

#endif