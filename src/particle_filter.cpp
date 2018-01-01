/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Initialize random number generators
  norm_dist = normal_distribution<double>(0., 1.);
  uni_dist = uniform_real_distribution<double>(0., 1.);

  num_particles = 10;

  for(int i = 0; i < num_particles; ++i) {
    Particle p;
    p.id = i;
    p.x = norm_dist(gen) * std[0] + x;
    p.y = norm_dist(gen) * std[1] + y;
    p.theta = norm_dist(gen) * std[2] + theta;
    p.weight = 1.;
    particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  for(auto& p : particles) {
    // If yaw_rate is zero, just add some noise to x and y.
    if(fabs(yaw_rate) > 1e-6) {
      p.x += velocity / yaw_rate * (sin(p.theta + delta_t * yaw_rate) - sin(p.theta)) + norm_dist(gen) * std_pos[0];
      p.y += velocity / yaw_rate * (-cos(p.theta + delta_t * yaw_rate) + cos(p.theta)) + norm_dist(gen) * std_pos[1];
    }
    else {
      p.x += norm_dist(gen) * std_pos[0];
      p.y += norm_dist(gen) * std_pos[1];
    }

    // std_pos[2] is the GPS uncertainty of the yaw angle, too samll for adding noise to particles.
    // Therefore multiflied by 10.
    p.theta += yaw_rate * delta_t + norm_dist(gen) * std_pos[2] * 10;
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  for(auto& particle : particles) {
    double prob = 1.;
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    for(auto& obs : observations) {
      // Do not use observation longer than sensor_range.
      double len_obs = dist(0., 0., obs.x, obs.y);
      if(len_obs > sensor_range)
        continue;

      // Transform observation to map coordinate.
      LandmarkObs obs_trans = getTransformed(obs, particle);

      // Find the closest land mark.
      LandmarkObs closest;
      double min_dist = -1.;
      for(auto& lm : map_landmarks.landmark_list) {
        double lm_obs_dist = dist(lm.x_f, lm.y_f, obs_trans.x, obs_trans.y);
        if(min_dist < 0 || lm_obs_dist < min_dist) {
          closest.id = lm.id_i;
          closest.x = lm.x_f;
          closest.y = lm.y_f;
          min_dist = lm_obs_dist;
        }
      }

      // Keep record of associated landmark.
      associations.push_back(closest.id);
      sense_x.push_back(closest.x);
      sense_y.push_back(closest.y);

      // Update the total probability.
      double obs_prob = getProb(obs_trans, closest, std_landmark);
      prob *= obs_prob;
    }

    if(prob >= 0.) {
      particle.weight = prob;
      SetAssociations(particle, associations, sense_x, sense_y);
    }
    else {
      cerr << "ERROR prob: " << prob << endl;
    }
  }
}

LandmarkObs ParticleFilter::getTransformed(const LandmarkObs& obs, const Particle& p) {
  LandmarkObs obsRet;
  obsRet.x = obs.x * cos(p.theta) - obs.y * sin(p.theta) + p.x;
  obsRet.y = obs.x * sin(p.theta) + obs.y * cos(p.theta) + p.y;
  return obsRet;
}

void ParticleFilter::resample() {
  double beta = 0.;
  int index = int(num_particles * uni_dist(gen));
  weights.clear();
  vector<Particle> temp_particles;
  for(auto& p : particles)
    weights.push_back(p.weight);

  double max_weight = *max_element(weights.begin(), weights.end());
  for(int i = 0; i < num_particles; ++i) {
    beta += uni_dist(gen) * 2. * max_weight;
    while(beta > weights[index]) {
        beta -= weights[index];
        index = (index + 1) % num_particles;
    }
    temp_particles.push_back(particles[index]);
  }
  particles = temp_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
