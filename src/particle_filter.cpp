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

#define YAWDOT_EPS 0.0001
#define INIT_MIN_DISTANCE 1000

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	
	//Set number of particles
	num_particles = 100;

	//Set up generator for drawing particle initial states from normal distributionss
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	particles.resize(num_particles);
	weights.resize(num_particles);

	//Initialize all the particle positions and weights
	for(int i = 0; i<num_particles; ++i){
		Particle init_particle;
		init_particle.id = i;
		init_particle.x = dist_x(gen);
		init_particle.y = dist_y(gen);
		init_particle.theta = dist_theta(gen);
		init_particle.weight = 1.;

		particles[i] = init_particle;
		weights[i] = 1.;

	}

	//Set initialized to true
	is_initialized = true;

	cout<<"Initialized particle filter with "<<num_particles<<" particles.\n";

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	//Declare the generator to be used to draw from normal distributions
	default_random_engine gen;

	//Declare variables to hold the mean position after taking the motion model into account
	double mean_x, mean_y, mean_theta;

	//Store calculations
	double del_t_vel = delta_t * velocity;
	double del_t_yawdot = delta_t * yaw_rate;
	double vel_over_yawdot = velocity/yaw_rate;

	//Calculate mean (x,y,theta) using exact motion models
	for(int i = 0; i<num_particles; ++i){
		//Take into account yaw rates less than a threshold
		if(fabs(yaw_rate)<YAWDOT_EPS){
			mean_x = particles[i].x + del_t_vel * cos(particles[i].theta);
			mean_y = particles[i].y + del_t_vel * sin(particles[i].theta);
			mean_theta = particles[i].theta;

		}
		else{
			mean_x = particles[i].x + vel_over_yawdot * (sin(particles[i].theta + del_t_yawdot) - sin(particles[i].theta));
			mean_y = particles[i].y + vel_over_yawdot * (cos(particles[i].theta) - cos(particles[i].theta + del_t_yawdot));
			mean_theta = particles[i].theta + del_t_yawdot;
		}	

		//Setup the normal distributions using the std dev of the position (x,y,theta)
		normal_distribution<double> norm_x(mean_x, std_pos[0]);
		normal_distribution<double> norm_y(mean_y, std_pos[1]);
		normal_distribution<double> norm_theta(mean_theta, std_pos[2]);

		//Draw uncertain positions from the distributions to get a noisy state
		particles[i].x = norm_x(gen);
		particles[i].y = norm_y(gen);
		particles[i].theta = norm_theta(gen);

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs>& predicted, std::vector<LandmarkObs>& observations) {

	//Loop over all predicted measurements
	for(int p = 0; p<predicted.size(); ++p){
		//Initialize minimum distance variable to a large number
		double min_distance = INIT_MIN_DISTANCE;
		double distance = 0;
		//Loop over all landmarks for each predicted measurements to find closest landmark to each measurement
		for(int l = 0; l<observations.size(); ++l){
			//Calculate euclidean distance between predicted and observed measurement (implement nearest neighbor)
			distance = dist(predicted[p].x, predicted[p].y, observations[l].x, observations[l].y);
			if(distance < min_distance){
				//Assign the landmark element integer to the predicted measurement
				predicted[p].id = l;
				min_distance = distance;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	vector<LandmarkObs> world_obs; //vector of observations in map coordinates
	vector<LandmarkObs> world_lmarks_in_range; //vector of landmarks in range in map coordinates
	vector<double> sense_x;
	vector<double> sense_y;
	vector<int> associations;
	double distance_to_lmark;
	int n_observations = observations.size(); //number of observations for this step
	int n_lmarks = map_landmarks.landmark_list.size();

	//Initialize associations for each particle
	sense_x.resize(n_observations);
	sense_y.resize(n_observations);
	associations.resize(n_observations);

	//Initialize transformed observations
	world_obs.resize(n_observations);

	//Store calculations
	double normalizer = 2*M_PI*std_landmark[0]*std_landmark[1];
	double sigma_x_sqr_2 = 2*pow(std_landmark[0], 2);
	double sigma_y_sqr_2 = 2*pow(std_landmark[1], 2);

	//Loop over all particles
	for(int i = 0; i<num_particles; ++i){
		//Clear landmarks in range for every particle
		world_lmarks_in_range.clear();

		//Store calculations
		double cos_theta = cos(particles[i].theta);
		double sin_theta = sin(particles[i].theta);

		//Convert particle observation to real world coordinates to generate the "predicted" measurements
		for(int z = 0; z<n_observations; ++z){
			LandmarkObs current_obs;
			current_obs.id = n_lmarks; //initialize ID of all observations to non-existent landmark
			current_obs.x = particles[i].x + cos_theta*observations[z].x - sin_theta*observations[z].y;
			current_obs.y = particles[i].y + sin_theta*observations[z].x + cos_theta*observations[z].y;
			
			//Set associations
			sense_x[z] = current_obs.x;
			sense_y[z] = current_obs.y;
			//Collect transformed observations
			world_obs[z] = current_obs;
		}

		//Associate the predicted measurements to landmarks within the sensor range
		//Loop over all landmarks
		for(int l=0; l<n_lmarks; ++l){
			//Check if landmark is within sensor range
			distance_to_lmark = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[l].x_f, map_landmarks.landmark_list[l].y_f);
				if(distance_to_lmark < sensor_range){
					//Append the landmark 
					LandmarkObs lmark_in_range;
					lmark_in_range.id = map_landmarks.landmark_list[l].id_i;
					lmark_in_range.x = map_landmarks.landmark_list[l].x_f;
					lmark_in_range.y = map_landmarks.landmark_list[l].y_f;
					world_lmarks_in_range.push_back(lmark_in_range);
				} 
		}

		//Perform the data association, the IDs of world_obs now correspond to the elements of world_lmarks_in_range
		//cout << "Total measurements: "<<world_obs.size()<<" Total landmarks in range: "<<world_lmarks_in_range.size()<<"\n";
		dataAssociation(world_obs, world_lmarks_in_range);

		//Calculate the multivariate Gaussian pdf
		double new_weight = 1.;
		double exponent;

		//Loop over observations and calculate the new weights
		for(int z = 0; z<n_observations; ++z){
			int lmark_id = world_obs[z].id;

			//Set associations
			associations[z] = world_lmarks_in_range[lmark_id].id;
			
			//Debug print statements
			//cout << "Measurement " << z << " associated with landmark "<<lmark_id<<"\n";
			//cout << "Measured x, y: ("<<world_obs[z].x<<","<<world_obs[z].y<<")\n";
			//cout << "Landmark x, y: ("<<world_lmarks_in_range[lmark_id].x<<","<<world_lmarks_in_range[lmark_id].y<<")\n";
			
			exponent = -(pow((world_obs[z].x - world_lmarks_in_range[lmark_id].x),2)/sigma_x_sqr_2 
						 + pow((world_obs[z].y - world_lmarks_in_range[lmark_id].y) ,2)/sigma_y_sqr_2);
			new_weight *= (1./normalizer)*exp(exponent);

		}


		//Set the new weights
		particles[i].weight = new_weight;
		weights[i] = new_weight;

		//Set the associations
		particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y);

	}
}

void ParticleFilter::resample() {

	//Initialize a random distribution using the particle weights
	discrete_distribution<int> resample_dist(weights.begin(), weights.end());
	default_random_engine gen;

	//Construct a vector to hold resampled particles
	vector<Particle> resampled_particles;
	resampled_particles.resize(num_particles);
	for(int i = 0; i<num_particles; ++i){
		resampled_particles[i] = particles[resample_dist(gen)];
	}

	//Assign resampled particles to current particles list
	particles = resampled_particles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
