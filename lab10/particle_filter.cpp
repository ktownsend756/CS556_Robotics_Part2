#include <Pololu3piPlus32U4.h>
#include <Gaussian.h>
#include "particle_filter.h"
#include "sonar.h"
#include <math.h>  // for logf, expf, PI

using namespace Pololu3piPlus32U4;


//2 helper functions to use when you need to wrap angles and keep particles inside the map
static float wrap_pi(float a){ while(a<=-PI) a+=2*PI; while(a>PI) a-=2*PI; return a; }
static float clamp(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }



/* Initialize particle filter and particle list */
/*Initializes the particle filter with parameters like the map length, number of particles, 
and variances for translation, rotation, and measurement. 

It generates particles with random initial positions and orientations, 
setting each particle's probability uniformly.*/
ParticleFilter::ParticleFilter(const Map& map, int lenOfMap, int num_particles, float translation_variance, float rotation_variance, float measurement_variance) {
  _num_particles = num_particles;
  _translation_variance = translation_variance;
  _rotation_variance = rotation_variance;
  _measurement_variance = measurement_variance;
  _lenOfMap = lenOfMap;
  _iter = 0;

  _x_est = 0;
  _y_est = 0;
  _angle_est = 0;
  _mp = &map;


  //generate particles with random location and rotation using _lenOfMap for x, y, and angle.
  //All probabilities will be the same at the start.
  //TODO: fill in "..."
  for(uint8_t i=0;i<_num_particles; i++){
    _particle_list[i].x = random(0, _lenOfMap);
    _particle_list[i].y = random(0, _lenOfMap);
    _particle_list[i].angle = ((float) random(0,628))/100.0-PI;
    _particle_list[i].probability = 1.0/_num_particles;
  }

}

/* Propagate motion of particles */
void ParticleFilter::move_particles(float dx, float dy, float dtheta){
//Apply motion to each particle by using the Gaussian class.
//You will need to utilize _translation_variance and _rotation_variance
//TODO: Put code under here

  
//Then add the current _particle_list[i] angle value with dtheta + random rotational
//And add the current _particle_list[i] x value with dx + random translation times the appropriate sin/cos angle value
//Do the same for y
//Consider using a for loop
//Then in teleop, your “forward 20 cm” simply calls move_particles(20, 0, 0); left/right turns call move_particles(0, 0, ±PI/2).
// finally, use wrap_pi and clamp functions to wrap "angle" & keep particles "x" and"y" inside the map
//The wrapping prevents invalid grid conversions in measure().
//TODO: Put code under here
  for(int i = 0; i < _num_particles; i++){
    float noise_dx = dx + Gaussian::gaussian(0, _translation_variance);
    float noise_dy = dy + Gaussian::gaussian(0, _translation_variance);
    float noise_dtheta = dtheta + Gaussian::gaussian(0, _rotation_variance);

    _particle_list[i].angle = wrap_pi(_partice_list[i].angle + noise_dtheta);
    _particle_list[i].x = clamp(_particle_list[i].x + noise_dx * cos(_particle_list[i].angle) - noise_dy * sin(_particle_list[i].angle), 0.0, (float)_lenOfMap);
    _particle_list[i].y = clamp(_particle_list[i].y + noise_dx * sin(_particle_list[i].angle) + noise_dy * cos(_particle_list[i].angle), 0.0, (float)_lenOfMap);

  }
}



/* Calculate particle posterior probabilities*/
//Be aware, if multiply raw PDFs and normalizes, it can underflow in longer runs. 
//Thus, In measure() accumulate log-likelihood then exponentiate with max-trick before normalization.
void ParticleFilter::measure(){
  // calculates probability of robot at location given ultrasonic reading
  // compute what the distance should be, if particle position is accurate
  /*If the robot uses closest_distance to calculate posterior probabilities in a particle filter,
    the return of this function helps assess the likelihood of the particle's position given the map layout.
     
    For example, if a particle's estimated sensor reading (distance to nearest obstacle) 
    aligns with the robot's actual sensor reading, 
    this particle is likely to be close to the robot’s true position.*/
  // Grid scale: 60 cm map, 3 cells => 20 cm per grid unit
  //TODO: Put code under here
  

  // Read sonar ONCE per update (use same z for all particles)
  //TODO: Put code under here (DONE)
  float z_meas = _sonar.readDist();

  // We'll accumulate log-weights first for numerical stability
  float maxlog = -1e30f;

  //For loop
  for(int i = 0; i < _num_particles; i++){
    // Convert particle (cm) -> grid units expected by Map::closest_distance
    //TODO: Put code under here (DONE)
   float origin[2] = {
      _particle_list[i].x / 20,
      _particle_list[i].y / 20
    };

    //calculate float particledist which is particle distance using sonar data
    //TODO: Put code under here (DONE)
    float particledist = _mp->closest_distance(origin, _particle_list[i].angle); // in grid units

    //inside the same loop, per particle
    float expected_cm = particledist * (_lenOfMap/3.0f);  // predicted range in cm
    float z = z_meas;
    float s2 = _measurement_variance;        // variance (not sigma)
    float loglik = -0.5f * ( (z - expected_cm)*(z - expected_cm)/s2 + logf(2.0f * PI * s2) );

    // store log-weights (use previous weight in log-domain; guard against 0)
    _particle_list[i].probability = loglik;
    // track maximum log-weight for the max-trick
    if(loglik > maxlog){
      maxlog = loglik;
    }
    
  }
  //End of for loop


  float norm_factor = 0.0f;  // reuse your variable name as the sum of linear weights
  // Define a new loop to convert to weights safely (max-trick), then normalize
  for(int i = 0; i < _num_particles; i++){
    _particle_list[i].probability = exp(_particle_list[i].probability - maxlog);
    norm_factor += _particle_list[i].probability;
  }

  //take each probability and normalize by norm_factor (in a for loop)
  float maxprob = 0.0f;
  if (norm_factor <= 1e-12f) {
    // Degenerate case: fall back to uniform if all weights underflowed
    const float uniform = 1.0f / (float)_num_particles;
    for (uint8_t i = 0; i < _num_particles; i++) {
      _particle_list[i].probability = uniform;
    }
    //If measure() couldn’t form a meaningful posterior (sum ~ 0), it sets uniform weights and doesn’t resample—preventing collapse.
    // (Resample next cycle once we have informative measurements.)
    return;

  } else {
    for(uint8_t i = 0; i < _num_particles; i++){
      _particle_list[i].probability /= norm_factor;    // normalize probabilities
      if (maxprob < _particle_list[i].probability) {
        maxprob = _particle_list[i].probability;       // keep max normalized prob for resample()
      }
    }
  }

  // TODO Resample particles around likely hypotheses
  //Outside all for loops, call resample(...) at the end of function
  //TODO: Put code under here
  resample(maxprob);
}



/* Resample particles */
/* Resamples particles using a "roulette wheel" approach based on their probabilities, 
favoring particles with higher probabilities. This step helps refine the particle distribution 
around likely robot positions. After resampling, probabilities are normalized.
*/
void ParticleFilter::resample(float maxprob){
  float b = 0.0;
  float norm_tot=0;
  uint8_t maxind=0;
  uint8_t index = (int)random(_num_particles);

  Particle temp_particles[_num_particles];

  //roulette wheel selection of particles based on probabilities
  for(uint8_t i=0;i<_num_particles; i++){
      b += ((float)(random(100))/100)*2.0*maxprob;

     while(b>_particle_list[index].probability){
       b -= _particle_list[index].probability;
        index = (index+1)%_num_particles; 
      }
      temp_particles[i].x = _particle_list[index].x;
      temp_particles[i].y = _particle_list[index].y;
      temp_particles[i].angle = _particle_list[index].angle;
      temp_particles[i].probability = _particle_list[index].probability;
      norm_tot+= temp_particles[i].probability;
  }

   //Normalizing probabilities
   for(uint8_t i=0;i<_num_particles; i++){
      temp_particles[i].probability /= norm_tot;

  }  

  for(uint8_t i=0; i<_num_particles; i++){
    _particle_list[i] = temp_particles[i];
  }
}



/* Print particle position and probabilities to serial monitor */
void ParticleFilter::print_particles(){
  //print current iteration number, particles, and estimated location
  //Before estimating position, call the estimate_position function.
  //TODO: Put code under here

  Serial.print("Iteration: ");
  Serial.println(_iter);
  for(int i = 0; i < _num_particles; i++){
    Serial.print("Particle: ");
    Serial.print(i);
    Serial.print(", x = ");
    Serial.print(_particle_list[i].x, 2);
    Serial.print(", y = ");
    Serial.print(_particle_list[i].y, 2);
    Serial.print(", angle = ");
    Serial.print(_particle_list[i].angle, 2);
    Serial.print(", probability = ");
    Serial.println(_particle_list[i].probability, 4);
  }

  estimate_position();

  Serial.print("Estimated Position x = ");
  Serial.print(_x_est, 2);
  Serial.print(", y = ");
  Serial.print(_y_est, 2);
  Serial.print(", angle = ");
  Serial.print(_angle_est, 2);
  
}

/* Estimate the position of the robot using posterior probabilities */
void ParticleFilter::estimate_position(){
  _x_est = 0.0;
  _y_est = 0.0;
  _angle_est = 0.0;

  //Add current _x_est to particle list probability times particle list x
  //Do the same for y and angle
  //This should be in a for loop
  //TODO: Put code under here (DONE)

  for(int i = 0; i < _num_particles; i++){
    _x_est += _particle_list[i].probability * _particle_list[i].x;
    _y_est += _particle_list[i].probability * _particle_list[i].y;
    _angle_est += _particle_list[i].probability * _particle_list[i].angle;
  }
  //End of for loop
}

