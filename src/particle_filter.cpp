/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <cassert>

using namespace std;

ParticleFilter::ParticleFilter(uint num_particles_)
        : num_particles(num_particles_) {
    // Prepare the weights
    weights.resize(num_particles);
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // Set standard deviations for x, y, and theta
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];

    // This line creates a normal (Gaussian) distribution for x, y, theta
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    default_random_engine gen;

    for (int i = 0; i < num_particles; ++i) {
        double sample_x, sample_y, sample_theta;

        sample_x = dist_x(gen);
        sample_y = dist_y(gen);
        sample_theta = dist_theta(gen);

        particles.push_back({i, sample_x, sample_y, sample_theta, 1.0});
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // Set standard deviations for x, y, and theta
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];

    // This line creates a normal (Gaussian) distribution for x, y, theta around 0
    normal_distribution<double> dist_x(0, std_x);
    normal_distribution<double> dist_y(0, std_y);
    normal_distribution<double> dist_theta(0, std_theta);

    default_random_engine gen;

    for (Particle &particle : particles) {
        if (abs(yaw_rate) < 0.00001) {
            // Treat yaw rate as zero
            particle.x += velocity * delta_t * cos(particle.theta) + dist_x(gen);
            particle.y += velocity * delta_t * sin(particle.theta) + dist_y(gen);
            particle.theta += dist_theta(gen);
        } else {
            // Include yaw rate
            particle.x += (velocity / yaw_rate) * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta)) +
                          dist_x(gen);
            particle.y += (velocity / yaw_rate) * (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t)) +
                          dist_y(gen);
            particle.theta += delta_t * yaw_rate + dist_theta(gen);
        }
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
    // Find the predicted measurement that is closest to each observed measurement and assign the
    // observed measurement to this particular landmark.

    for (LandmarkObs &observation : observations) {

        double minDist = numeric_limits<double>::max();

        for (LandmarkObs &prediction : predicted) {
            double xDistance = observation.x - prediction.x;
            double yDistance = observation.y - prediction.y;

            double distance = xDistance * xDistance + yDistance * yDistance;

            if (distance < minDist) {
                minDist = distance;
                observation.id = prediction.id;
            }
        }
    }

}

std::vector<LandmarkObs>
ParticleFilter::findLandmarksInRange(double x, double y, double range, std::vector<Map::single_landmark_s> landmarks) {
    std::vector<LandmarkObs> result;
    for (Map::single_landmark_s landmark : landmarks) {
        double xDelta = x - landmark.x_f;
        double yDelta = y - landmark.y_f;

        if (xDelta * xDelta + yDelta * yDelta <= range * range) {
            result.push_back(LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});
        }
    }
    return result;
}

std::vector<LandmarkObs>
ParticleFilter::transform(std::vector<LandmarkObs> observations, double x, double y, double theta) {
    vector<LandmarkObs> result;
    for (LandmarkObs observation: observations) {
        double xNew = cos(theta) * observation.x - sin(theta) * observation.y + x;
        double yNew = sin(theta) * observation.x + cos(theta) * observation.y + y;
        result.push_back({observation.id, xNew, yNew});
    }
    return result;
}

double
ParticleFilter::calculateMultivariateGaussianProbability(const LandmarkObs &prediction, const LandmarkObs &observation,
                                                         double std_landmark[]) {
    // Re-calculate weight of the particle
    const double sig_x = std_landmark[0];
    const double sig_y = std_landmark[1];

    // Make sure the std dev doesn't change
    assert (sig_x == std_landmark[0]);
    assert (sig_y == std_landmark[1]);

    // calculate normalization term
    const double gauss_norm = (1 / (2 * M_PI * sig_x * sig_y));

    double exponent =
            ((observation.x - prediction.x) * (observation.x - prediction.x)) / (2 * sig_x * sig_x) +
            ((observation.y - prediction.y) * (observation.y - prediction.y)) / (2 * sig_y * sig_y);

    return gauss_norm * exp(-exponent);
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map) {
    // Clear weights
    weights.clear();

    // Go over all particles
    for (Particle &particle : particles) {

        // Find nearby landmarks
        std::vector<LandmarkObs> predictions = findLandmarksInRange(particle.x, particle.y, sensor_range,
                                                                    map.landmark_list);

        // Transpose observations into the map coordinate space
        std::vector<LandmarkObs> transposedObservations = transform(observations, particle.x, particle.y,
                                                                    particle.theta);

        // Associate landmarks and observations
        dataAssociation(predictions, transposedObservations);

        // Re-calculate weight of the particle
        particle.weight = 1;
        for (LandmarkObs &observation : transposedObservations) {

            // Find the prediction for this observation
            auto it = std::find_if(predictions.begin(), predictions.end(),
                                   [&observation](const LandmarkObs &prediction) {
                                       return prediction.id == observation.id;
                                   });

            // If prediction is found, calculate Multivariate-Gaussian Probability and multiply with previous
            if (it != predictions.end()) {
                particle.weight *= calculateMultivariateGaussianProbability(*it, observation, std_landmark);
            }
        }

        // Update weights vector for re-sampling
        weights.push_back(particle.weight);
    }

    assert(weights.size() == particles.size());
}

void ParticleFilter::resample() {
    vector<Particle> newParticles;
    newParticles.reserve(num_particles);

    default_random_engine gen;
    discrete_distribution<int> distribution(weights.begin(), weights.end());

    for (int i = 0; i < num_particles; ++i) {
        newParticles.push_back(particles[distribution(gen)]);
    }

    particles = newParticles;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
