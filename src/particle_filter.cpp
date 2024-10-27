#include "particle_filter.h"
#include "particle.h"
#include "robot.h"
#include <vector>
#include <iostream>

ParticleFilter::ParticleFilter(const Robot& real_robot, int num_particles_, double noise_part, const std::vector<std::pair<double, double>>& landmarks_)
    : num_particles(num_particles_), landmarks(landmarks_) 
{
    std::normal_distribution<double> dist_x(real_robot.getX(), noise_part);
    std::normal_distribution<double> dist_y(real_robot.getY(), noise_part);
    std::normal_distribution<double> dist_theta(real_robot.getTheta(), noise_part);

    for (int i = 0; i < num_particles; ++i) {
        particles.emplace_back(dist_x(generator), dist_y(generator), dist_theta(generator), real_robot.getWheelBase());
    }
}

void ParticleFilter::update(double dt, double velocity, double steering_angle, const std::vector<std::pair<double, double>>& landmarks, const Robot& real_robot) {
    
    if(!first){resample();}

    else{
        first = false;
    }

    for (auto& particle : particles) {
        particle.update(dt, velocity, steering_angle, landmarks, real_robot);
    }

    updateWeights(real_robot);
    
}

void ParticleFilter::updateWeights(const Robot& real_robot) {

    for(auto& part : particles){
        part.measurement_prob(real_robot);
    }

    double wt = 0;

    for(auto& part : particles){
        double weight = 1;

        for(auto& prob : part.z_prob){
           weight = weight * prob;
        }

        wt += weight;        
        part.setWeight(weight);
    }

    for(auto& part : particles){
        part.setWeight(part.getWeight()/wt);
    }

}

void ParticleFilter::resample() {

    std::vector<Particle> new_particles;
    
    // Находим максимальный вес среди всех частиц
    double max_weight = 0.0;
    for (const auto& particle : particles) {
        max_weight = std::max(max_weight, particle.getWeight());
    }

    std::uniform_real_distribution<double> dist_beta(0.0, 2.0 * max_weight);
    std::uniform_int_distribution<int> dist_index(0, particles.size() - 1);

    //колесо отбора
    int index = dist_index(generator);
    double beta = 0.0;
    for (size_t i = 0; i < particles.size(); ++i) {
        beta += dist_beta(generator);

        while (beta > particles[index].getWeight()) {
            beta -= particles[index].getWeight();
            index = (index + 1) % particles.size();
        }
        
        new_particles.push_back(particles[index]);
    }

    for (auto& particle : new_particles) {

        std::normal_distribution<double> dist_x(particle.getX(), noise_part);
        std::normal_distribution<double> dist_y(particle.getY(), noise_part);
        std::normal_distribution<double> dist_theta(particle.getTheta(), noise_part);

        particle.setX(dist_x(generator));
        particle.setY(dist_y(generator));
        particle.setTheta(dist_theta(generator));
    }

    // Обновляем частицы новыми отобранными значениями
    particles = new_particles;
}

void ParticleFilter::getEstimate(double& e_x, double& e_y, double& e_th){

    e_x = e_y = e_th = 0;

    for(auto& part : particles){
        e_x = e_x + part.getX()*part.getWeight();
        e_y = e_y + part.getY()*part.getWeight();
        e_th = e_th + part.getTheta()*part.getWeight();
    }
}
