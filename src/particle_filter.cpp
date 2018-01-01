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
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
  //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  debug = 0;
  norm_dist = normal_distribution<double>(0., 1.);
  uni_dist = uniform_real_distribution<double>(0., 1.);
  num_particles = 5;
  if(debug == 2)
    printf("init:\n");
  for(int i = 0; i < num_particles; ++i) {
    Particle p;
    p.id = i;
    p.x = norm_dist(gen) * std[0] + x;
    p.y = norm_dist(gen) * std[1] + y;
    p.theta = norm_dist(gen) * std[2] + theta;
    p.weight = 1.;
    particles.push_back(p);
    if(debug == 2)
      printf("%.4f %.4f %.4f\n", p.x, p.y, p.theta);
  }
  is_initialized = true;

  if(debug == 1)
    printf("### measurement: %.2f %.2f %.2f p0: %.2f %.2f %.2f\n", x, y, theta, particles[0].x, particles[0].y, particles[0].theta);
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  if(debug == 2)
    printf("prediction: dt %.4e v %.4e yr %.4e\n", delta_t, velocity, yaw_rate);
  for(int i = 0; i < num_particles; ++i) {
    Particle& p = particles[i];
    if(fabs(yaw_rate) > 1e-6) {
      p.x += velocity / yaw_rate * (sin(p.theta + delta_t * yaw_rate) - sin(p.theta)) + norm_dist(gen) * std_pos[0];
      p.y += velocity / yaw_rate * (-cos(p.theta + delta_t * yaw_rate) + cos(p.theta)) + norm_dist(gen) * std_pos[1];
    }
    p.theta += yaw_rate * delta_t + norm_dist(gen) * std_pos[2] * 10;
    //while(p.theta > 2 * M_PI) p.theta -= 2 * M_PI;
    //while(p.theta < 0.) p.theta += 2 * M_PI;
    if(debug == 2)
      printf("%.4f %.4f %.4f\n", p.x, p.y, p.theta);
  }

  if(debug == 1)
    printf("### p0 prediction: %.2f %.2f %.2f\n", particles[0].x, particles[0].y, particles[0].theta);
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
  for(auto& particle : particles) {
    double prob = 1.;
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    int nObs = observations.size();
    for(int iObs = 0; iObs < nObs; ++iObs) {
      auto& obs = observations[iObs];

      double len_obs = dist(0., 0., obs.x, obs.y);
      if(len_obs > sensor_range)
        continue;

      if(debug == 2)
        printf("#-- %7s: %.2f %.2f\n", "obs", obs.x, obs.y);

      LandmarkObs obs_trans = getTransformed(obs, particle);

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
      if(debug == 2) {
        printf("    %7s: %.2f %.2f\n", "trans", obs_trans.x, obs_trans.y);
        printf("    %7s: %.2f %.2f dist: %.2f\n", "closest", closest.x, closest.y, min_dist);
      }

      associations.push_back(closest.id);
      sense_x.push_back(closest.x);
      sense_y.push_back(closest.y);

      double obs_prob = getProb(obs_trans, closest, std_landmark);
      if(debug == 2)
        printf("   min_dist: %.4f obs_prob: %.8e\n", min_dist, obs_prob);
      if(false && isnan(obs_prob)) {
        cerr << "ERROR obs_prob" << '\t' << obs_prob << endl;
        printf("    %7s: %.2f %.2f\n", "trans", obs_trans.x, obs_trans.y);
        printf("    %7s: %.2f %.2f dist: %.2f\n", "closest", closest.x, closest.y, min_dist);
        printf("   min_dist: %.4f obs_prob: %.8e\n", min_dist, obs_prob);
      }
      prob *= obs_prob;
    }

    if(debug == 2)
      printf("prob %.8e\n", prob);
    if(prob >= 0.) {
      particle.weight = prob;
      SetAssociations(particle, associations, sense_x, sense_y);
    }
    else {
      cerr << "ERROR " << prob << endl;
    }
  }
  if(debug == 1) {
    int i;
    cout << "enter to continue\n";
    cin >> i;
  }
}

LandmarkObs ParticleFilter::getTransformed(const LandmarkObs& obs, const Particle& p) {
  LandmarkObs obsRet;
  double th = p.theta;
  while(th > 2 * M_PI) th -= 2 * M_PI;
  while(th < 0.) th += 2 * M_PI;
  obsRet.x = obs.x * cos(th) - obs.y * sin(th) + p.x;
  obsRet.y = obs.x * sin(th) + obs.y * cos(th) + p.y;
  if(isnan(obsRet.x) || isnan(obsRet.y))
    cerr << "ERROR nan\t" << obs.x << '\t' << obs.y << '\t' << p.x << '\t' << p.y << '\t' << th << endl;
  return obsRet;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  static int cnt = 0;
  vector<Particle> temp_particles;
  int index = int(num_particles * uni_dist(gen));
  double beta = 0.;
  weights.clear();
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
  if(debug == 2) {
    printf("resample:\n");
    for(auto& p : particles)
      printf("    %.4f %.4f %.4f\n", p.x, p.y, p.theta);
    if(++cnt > 1) {
      int i;
      cout << "enter to continue\n";
      cin >> i;
    }
  }
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
