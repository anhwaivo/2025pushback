#ifdef ENABLE_MCL
#include "monte.h"
#include "monte_config.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

// Forward declarations for distance sensors (defined in main.cpp)
extern pros::Distance dNorth;
extern pros::Distance dSouth;
extern pros::Distance dEast;
extern pros::Distance dWest;

namespace {
// Global vector of particles; we'll resize it during initialization.
std::vector<Particle> particles;
std::random_device rd;
std::mt19937 gen(rd()); // Mersenne Twister random number generator
std::normal_distribution<float> noise_x(0.0f, 0.1f); // Mean 0, std dev 0.1 inches
std::normal_distribution<float> noise_y(0.0f, 0.1f); // Mean 0, std dev 0.1 inches
std::normal_distribution<float> noise_theta(0.0f, 5.0f); // Std dev 5.0 degrees

// Track the last odometry pose for odometry delta calculations
// IMPORTANT: This should track the RAW odometry (before MCL correction) for motion delta calculation
lemlib::Pose lastOdomPose(0, 0, 0);
lemlib::Pose lastRawOdomPose(0, 0, 0); // Track raw odometry separately to avoid circular dependency

// Task management
pros::Task *mclTaskHandle = nullptr;
bool mclRunning = false;
lemlib::Chassis *chassisPtr = nullptr;

// Sensor usage flags (from monte_config.h)
bool useNorthSensor = USE_NORTH_SENSOR;
bool useSouthSensor = USE_SOUTH_SENSOR;
bool useEastSensor = USE_EAST_SENSOR;
bool useWestSensor = USE_WEST_SENSOR;

// Variables for update frequency control
int lastUpdateTime = 0; // Timestamp of the last MCL update
lemlib::Pose lastUpdatedPose(0, 0, 0); // Last pose at which MCL was updated

// Variables to store previous sensor readings
float prev_north_dist = -1.0f;
float prev_south_dist = -1.0f;
float prev_east_dist = -1.0f;
float prev_west_dist = -1.0f;

// Filtered pose
lemlib::Pose filteredPose(0, 0, 0);

// Resample counter
int resampleCounter = 0;
} // namespace

// Initialize particles around an initial pose estimate
void initializeParticles(const lemlib::Pose &initialPose) {
  particles.resize(PARTICLE_QUANTITY);
  lastOdomPose = initialPose;
  filteredPose = initialPose;

  std::normal_distribution<float> x_dist(initialPose.x, INIT_X_SPREAD);
  std::normal_distribution<float> y_dist(initialPose.y, INIT_Y_SPREAD);
  std::normal_distribution<float> theta_dist(initialPose.theta, INIT_THETA_SPREAD);

  for (auto &particle : particles) {
    particle = Particle(lemlib::Pose(x_dist(gen), y_dist(gen), theta_dist(gen)),
                       1.0f / PARTICLE_QUANTITY);
  }
}

// Update particles based on robot motion (prediction step)
void motionUpdate(const lemlib::Pose &localOdomDelta) {
  float motion_magnitude = std::sqrt(localOdomDelta.x * localOdomDelta.x +
                                     localOdomDelta.y * localOdomDelta.y);

  // Only add noise if motion is significant
  bool add_noise = motion_magnitude > MIN_MOTION_FOR_NOISE;

  // Noise values
  std::normal_distribution<float> motion_noise(0.0, MOTION_NOISE_STD_DEV);
  std::normal_distribution<float> rotation_noise(0.0, ROTATION_NOISE_STD_DEV);

  for (auto &particle : particles) {
    float theta_rad = particle.pose.theta * M_PI / 180.0f;
    float dx_global =
        localOdomDelta.x * cos(theta_rad) - localOdomDelta.y * sin(theta_rad);
    float dy_global =
        localOdomDelta.x * sin(theta_rad) + localOdomDelta.y * cos(theta_rad);

    // Scale noise based on motion magnitude
    float noise_scale =
        add_noise ? std::min(1.0f, motion_magnitude / 2.0f) : 0.0f;

    particle.pose.x += dx_global + noise_scale * motion_noise(gen);
    particle.pose.y += dy_global + noise_scale * motion_noise(gen);
    particle.pose.theta += localOdomDelta.theta + noise_scale * rotation_noise(gen);

    // Normalize theta to be within 0-360 degrees
    particle.pose.theta = fmod(particle.pose.theta, 360.0f);
    if (particle.pose.theta < 0) {
      particle.pose.theta += 360.0f;
    }
  }
}

// Calculate expected sensor readings for a particle
float predictSensorReading(const lemlib::Pose &particlePose,
                           const char direction) {
  float half_dimension = FIELD_DIMENSIONS / 2.0f; // 72 inches
  float theta = particlePose.theta * M_PI / 180.0f; // Robot orientation in radians
  
  // Get sensor offset in robot frame based on direction
  float offset_x_robot = 0.0f;
  float offset_y_robot = 0.0f;
  switch (direction) {
  case 'N':
    offset_x_robot = NORTH_SENSOR_X_OFFSET;
    offset_y_robot = NORTH_SENSOR_Y_OFFSET;
    break;
  case 'S':
    offset_x_robot = SOUTH_SENSOR_X_OFFSET;
    offset_y_robot = SOUTH_SENSOR_Y_OFFSET;
    break;
  case 'E':
    offset_x_robot = EAST_SENSOR_X_OFFSET;
    offset_y_robot = EAST_SENSOR_Y_OFFSET;
    break;
  case 'W':
    offset_x_robot = WEST_SENSOR_X_OFFSET;
    offset_y_robot = WEST_SENSOR_Y_OFFSET;
    break;
  default:
    return -1.0f;
  }
  
  // Transform sensor offset from robot frame to field frame
  float offset_x_field = offset_x_robot * cos(theta) - offset_y_robot * sin(theta);
  float offset_y_field = offset_x_robot * sin(theta) + offset_y_robot * cos(theta);
  
  // Calculate sensor position in field frame
  float sensor_x = particlePose.x + offset_x_field;
  float sensor_y = particlePose.y + offset_y_field;

  // Define the sensor's FOV (±12° from center)
  const float FOV_HALF_ANGLE = SENSOR_FOV_HALF_ANGLE_DEG * M_PI / 180.0f;

  // Direction-specific center angle (assuming 0° is north)
  float sensor_angle = 0.0f;
  switch (direction) {
  case 'N':
    sensor_angle = 0.0f;
    break; // Up (y = 72)
  case 'S':
    sensor_angle = M_PI;
    break; // Down (y = -72)
  case 'E':
    sensor_angle = M_PI / 2.0f;
    break; // Right (x = 72)
  case 'W':
    sensor_angle = 3.0f * M_PI / 2.0f;
    break; // Left (x = -72)
  default:
    return -1.0f;
  }

  // Adjust sensor angle based on robot orientation
  float center_angle = sensor_angle + theta;

  // Calculate distances to boundaries along the edges of the FOV cone
  float left_angle = center_angle - FOV_HALF_ANGLE;
  float right_angle = center_angle + FOV_HALF_ANGLE;

  // Parametric line equations: x = sensor_x + t * cos(angle), y = sensor_y + t * sin(angle)
  // Find intersection with boundaries (t = distance to boundary)
  float distances[4]; // Distances to north, south, east, west boundaries

  // North boundary (y = 72)
  // Avoid division by zero: check if sin(angle) is near zero
  float sin_left = sin(left_angle);
  float sin_right = sin(right_angle);
  float t_north_left = (std::abs(sin_left) > 1e-6f) ? (half_dimension - sensor_y) / sin_left : std::numeric_limits<float>::max();
  float t_north_right = (std::abs(sin_right) > 1e-6f) ? (half_dimension - sensor_y) / sin_right : std::numeric_limits<float>::max();
  distances[0] = (t_north_left > 0 && std::isfinite(t_north_left))
                     ? t_north_left
                     : std::numeric_limits<float>::max();
  distances[0] =
      std::min(distances[0], (t_north_right > 0 && std::isfinite(t_north_right))
                                 ? t_north_right
                                 : std::numeric_limits<float>::max());

  // South boundary (y = -72)
  float t_south_left = (std::abs(sin_left) > 1e-6f) ? (-half_dimension - sensor_y) / sin_left : std::numeric_limits<float>::max();
  float t_south_right = (std::abs(sin_right) > 1e-6f) ? (-half_dimension - sensor_y) / sin_right : std::numeric_limits<float>::max();
  distances[1] = (t_south_left > 0 && std::isfinite(t_south_left))
                     ? t_south_left
                     : std::numeric_limits<float>::max();
  distances[1] =
      std::min(distances[1], (t_south_right > 0 && std::isfinite(t_south_right))
                                 ? t_south_right
                                 : std::numeric_limits<float>::max());

  // East boundary (x = 72)
  // Avoid division by zero: check if cos(angle) is near zero
  float cos_left = cos(left_angle);
  float cos_right = cos(right_angle);
  float t_east_left = (std::abs(cos_left) > 1e-6f) ? (half_dimension - sensor_x) / cos_left : std::numeric_limits<float>::max();
  float t_east_right = (std::abs(cos_right) > 1e-6f) ? (half_dimension - sensor_x) / cos_right : std::numeric_limits<float>::max();
  distances[2] = (t_east_left > 0 && std::isfinite(t_east_left))
                     ? t_east_left
                     : std::numeric_limits<float>::max();
  distances[2] =
      std::min(distances[2], (t_east_right > 0 && std::isfinite(t_east_right))
                                 ? t_east_right
                                 : std::numeric_limits<float>::max());

  // West boundary (x = -72)
  float t_west_left = (std::abs(cos_left) > 1e-6f) ? (-half_dimension - sensor_x) / cos_left : std::numeric_limits<float>::max();
  float t_west_right = (std::abs(cos_right) > 1e-6f) ? (-half_dimension - sensor_x) / cos_right : std::numeric_limits<float>::max();
  distances[3] = (t_west_left > 0 && std::isfinite(t_west_left))
                     ? t_west_left
                     : std::numeric_limits<float>::max();
  distances[3] =
      std::min(distances[3], (t_west_right > 0 && std::isfinite(t_west_right))
                                 ? t_west_right
                                 : std::numeric_limits<float>::max());

  // Find the minimum positive distance within FOV
  float min_distance = std::numeric_limits<float>::max();
  for (int i = 0; i < 4; ++i) {
    if (distances[i] > 0 && distances[i] < min_distance) {
      min_distance = distances[i];
    }
  }

  // If no valid intersection, return a large value
  if (min_distance == std::numeric_limits<float>::max()) {
    return 72.0f; // Default to max field distance
  }

  return min_distance;
}

// Update particle weights based on sensor measurements
// Properly fuses all 4 distance sensors using multiplicative likelihood
void measurementUpdate(float north_dist, float south_dist, float east_dist,
                       float west_dist) {
  // Check for significant change in ANY sensor
  bool is_first_update = (prev_north_dist < 0 && prev_south_dist < 0 && 
                          prev_east_dist < 0 && prev_west_dist < 0);
  
  bool significant_change = is_first_update;
  
  if (!significant_change) {
    if (north_dist >= 0 && prev_north_dist >= 0 &&
        std::abs(north_dist - prev_north_dist) > DISTANCE_CHANGE_THRESHOLD)
      significant_change = true;
    if (south_dist >= 0 && prev_south_dist >= 0 &&
        std::abs(south_dist - prev_south_dist) > DISTANCE_CHANGE_THRESHOLD)
      significant_change = true;
    if (east_dist >= 0 && prev_east_dist >= 0 &&
        std::abs(east_dist - prev_east_dist) > DISTANCE_CHANGE_THRESHOLD)
      significant_change = true;
    if (west_dist >= 0 && prev_west_dist >= 0 &&
        std::abs(west_dist - prev_west_dist) > DISTANCE_CHANGE_THRESHOLD)
      significant_change = true;
  }
  
  // if (!significant_change)
    // return;

  // Sigma values - increased to be more forgiving and reduce over-correction
  // If sensors hit game elements instead of walls, tighter sigmas cause wrong corrections
  auto getSigma = [&](float predicted_distance) {
    // Use more forgiving sigmas to reduce over-correction from bad sensor readings
    if (predicted_distance < 5.0f) {
      return 5.0f; // More forgiving for close readings
    } else if (predicted_distance < 10.0f) {
      return 7.0f; // More forgiving for medium distances
    } else {
      return 10.0f; // More forgiving for far distances
    }
  };

  float total_weight = 0.0f;
  for (auto &particle : particles) {
    float particle_weight = 1.0f;
    int valid_readings = 0;

    // Fuse ALL 4 sensors using multiplicative likelihood
    // Each sensor contributes independently, so we multiply their likelihoods
    if (north_dist >= 0) {
      float predicted_north_dist = predictSensorReading(particle.pose, 'N');
      float north_diff = std::abs(predicted_north_dist - north_dist);
      float sigma = getSigma(predicted_north_dist);
      float north_likelihood =
          std::exp(-(north_diff * north_diff) / (2.0f * sigma * sigma));
      particle_weight *= north_likelihood;
      valid_readings++;
    }

    if (south_dist >= 0) {
      float predicted_south_dist = predictSensorReading(particle.pose, 'S');
      float south_diff = std::abs(predicted_south_dist - south_dist);
      float sigma = getSigma(predicted_south_dist);
      float south_likelihood =
          std::exp(-(south_diff * south_diff) / (2.0f * sigma * sigma));
      particle_weight *= south_likelihood;
      valid_readings++;
    }

    if (east_dist >= 0) {
      float predicted_east_dist = predictSensorReading(particle.pose, 'E');
      float east_diff = std::abs(predicted_east_dist - east_dist);
      float sigma = getSigma(predicted_east_dist);
      float east_likelihood =
          std::exp(-(east_diff * east_diff) / (2.0f * sigma * sigma));
      particle_weight *= east_likelihood;
      valid_readings++;
    }

    if (west_dist >= 0) {
      float predicted_west_dist = predictSensorReading(particle.pose, 'W');
      float west_diff = std::abs(predicted_west_dist - west_dist);
      float sigma = getSigma(predicted_west_dist);
      float west_likelihood =
          std::exp(-(west_diff * west_diff) / (2.0f * sigma * sigma));
      particle_weight *= west_likelihood;
      valid_readings++;
    }

    if (valid_readings > 0) {
      particle.weight = std::max(particle_weight, MIN_WEIGHT);
    } else {
      particle.weight = MIN_WEIGHT;
    }
    total_weight += particle.weight;
  }

  if (total_weight > 0) {
    for (auto &particle : particles) {
      particle.weight /= total_weight;
    }
  }

  // Update all previous sensor readings
  prev_north_dist = north_dist;
  prev_south_dist = south_dist;
  prev_east_dist = east_dist;
  prev_west_dist = west_dist;
}

// Perform systematic resampling based on particle weights
std::vector<Particle> weightedResample(const std::vector<Particle> &particles) {
  std::vector<Particle> new_particles(PARTICLE_QUANTITY);
  std::uniform_real_distribution<float> dist(0.0f, 1.0f);

  // Compute cumulative weights
  std::vector<float> cumulative_weights(PARTICLE_QUANTITY);
  cumulative_weights[0] = particles[0].weight;
  for (size_t i = 1; i < PARTICLE_QUANTITY; ++i) {
    cumulative_weights[i] = cumulative_weights[i - 1] + particles[i].weight;
  }

  // Systematic resampling
  float step = 1.0f / PARTICLE_QUANTITY;
  float r = dist(gen) * step; // Initial random offset
  size_t index = 0;

  for (size_t m = 0; m < PARTICLE_QUANTITY; ++m) {
    float U = r + m * step;
    while (index < PARTICLE_QUANTITY - 1 && U > cumulative_weights[index]) {
      ++index;
    }
    new_particles[m] = particles[index];
    new_particles[m].weight = 1.0f / PARTICLE_QUANTITY; // Reset weights uniformly
  }

  return new_particles;
}

// Resample particles based on their weights using systematic resampling
void resampleParticles() {
  // Perform systematic resampling
  std::vector<Particle> new_particles = weightedResample(particles);

  // Apply noise to resampled particles to prevent sample impoverishment
  std::normal_distribution<float> noise_x(0.0f, RESAMPLE_NOISE_X);
  std::normal_distribution<float> noise_y(0.0f, RESAMPLE_NOISE_Y);
  std::normal_distribution<float> noise_theta(0.0f, RESAMPLE_NOISE_THETA);

  for (auto &particle : new_particles) {
    particle.pose.x += noise_x(gen);
    particle.pose.y += noise_y(gen);
    particle.pose.theta += noise_theta(gen);

    // Normalize theta to stay within 0-360 degrees
    particle.pose.theta = fmod(particle.pose.theta, 360.0f);
    if (particle.pose.theta < 0) {
      particle.pose.theta += 360.0f;
    }
  }
  particles = new_particles;
}

// Get estimated pose with filtering and odometry blending
lemlib::Pose getEstimatedPose() {
  lemlib::Pose rawEstimated(0, 0, 0);
  float total_weight = 0.0f;

  for (const auto &particle : particles) {
    rawEstimated.x += particle.weight * particle.pose.x;
    rawEstimated.y += particle.weight * particle.pose.y;
    rawEstimated.theta += particle.weight * particle.pose.theta;
    total_weight += particle.weight;
  }

  if (total_weight > 0) {
    rawEstimated.x /= total_weight;
    rawEstimated.y /= total_weight;
    rawEstimated.theta /= total_weight;
  }

  // Apply low-pass filter
  filteredPose.x = FILTER_ALPHA * rawEstimated.x + (1 - FILTER_ALPHA) * filteredPose.x;
  filteredPose.y = FILTER_ALPHA * rawEstimated.y + (1 - FILTER_ALPHA) * filteredPose.y;

  // Filter theta with angle wrapping
  float theta_diff = rawEstimated.theta - filteredPose.theta;
  if (theta_diff > 180)
    theta_diff -= 360;
  if (theta_diff < -180)
    theta_diff += 360;
  filteredPose.theta += FILTER_ALPHA * theta_diff;

  // Normalize theta
  while (filteredPose.theta > 360)
    filteredPose.theta -= 360;
  while (filteredPose.theta < 0)
    filteredPose.theta += 360;

  // Apply odometry trust factor (blend with odometry)
  lemlib::Pose blendedPose(0, 0, 0);
  blendedPose.x = ODOMETRY_TRUST_FACTOR * lastOdomPose.x +
                  (1 - ODOMETRY_TRUST_FACTOR) * filteredPose.x;
  blendedPose.y = ODOMETRY_TRUST_FACTOR * lastOdomPose.y +
                  (1 - ODOMETRY_TRUST_FACTOR) * filteredPose.y;

  // Handle theta blending with angle wrapping
  theta_diff = filteredPose.theta - lastOdomPose.theta;
  if (theta_diff > 180)
    theta_diff -= 360;
  if (theta_diff < -180)
    theta_diff += 360;
  blendedPose.theta = lastOdomPose.theta + (1 - ODOMETRY_TRUST_FACTOR) * theta_diff;

  return blendedPose;
}

// Calculate motion delta from raw odometry (before MCL correction)
// This function uses lastRawOdomPose to avoid circular dependency
lemlib::Pose calculateMotionDelta(const lemlib::Pose &currentRawOdomPose) {
  // Calculate the change in position since the last update using RAW odometry
  lemlib::Pose delta(currentRawOdomPose.x - lastRawOdomPose.x,
                     currentRawOdomPose.y - lastRawOdomPose.y,
                     currentRawOdomPose.theta - lastRawOdomPose.theta);

  // Handle theta wrapping for delta
  if (delta.theta > 180.0f)
    delta.theta -= 360.0f;
  if (delta.theta < -180.0f)
    delta.theta += 360.0f;

  // Update the last raw odometry position for next time
  lastRawOdomPose = currentRawOdomPose;

  return delta;
}

// Background task for running Monte Carlo Localization
void mclTask(void *param) {
  if (!chassisPtr)
    return;

  // Initialize with current pose
  lemlib::Pose initialPose = chassisPtr->getPose();
  initializeParticles(initialPose);
  lastOdomPose = initialPose; // For blending in getEstimatedPose
  lastRawOdomPose = initialPose; // For motion delta calculation (raw odometry)
  lastUpdateTime = pros::millis();
  resampleCounter = 0;

  while (mclRunning) {
    // Get distance readings in mm first (check bogus values before conversion)
    float north_mm = useNorthSensor ? dNorth.get() : -1.0f;
    float south_mm = useSouthSensor ? dSouth.get() : -1.0f;
    float east_mm = useEastSensor ? dEast.get() : -1.0f;
    float west_mm = useWestSensor ? dWest.get() : -1.0f;
    
    // Convert to inches (check for bogus values: 9999mm)
    float north = (north_mm >= 0 && north_mm < 9999) ? north_mm / 25.4f : -1.0f;
    float south = (south_mm >= 0 && south_mm < 9999) ? south_mm / 25.4f : -1.0f;
    float east = (east_mm >= 0 && east_mm < 9999) ? east_mm / 25.4f : -1.0f;
    float west = (west_mm >= 0 && west_mm < 9999) ? west_mm / 25.4f : -1.0f;

    // Apply confidence and size filters
    if (useNorthSensor && north >= 0) {
      int north_conf = dNorth.get_confidence();
      int north_size = dNorth.get_object_size();
      if (north_conf < MIN_CONFIDENCE || north_size < MIN_OBJECT_SIZE ||
          north_size > MAX_OBJECT_SIZE || north_mm > MAX_DISTANCE_MM)
        north = -1.0f;
    }

    if (useSouthSensor && south >= 0) {
      int south_conf = dSouth.get_confidence();
      int south_size = dSouth.get_object_size();
      if (south_conf < MIN_CONFIDENCE || south_size < MIN_OBJECT_SIZE ||
          south_size > MAX_OBJECT_SIZE || south_mm > MAX_DISTANCE_MM)
        south = -1.0f;
    }

    if (useEastSensor && east >= 0) {
      int east_conf = dEast.get_confidence();
      int east_size = dEast.get_object_size();
      if (east_conf < MIN_CONFIDENCE || east_size < MIN_OBJECT_SIZE ||
          east_size > MAX_OBJECT_SIZE || east_mm > MAX_DISTANCE_MM)
        east = -1.0f;
    }

    if (useWestSensor && west >= 0) {
      int west_conf = dWest.get_confidence();
      int west_size = dWest.get_object_size();
      if (west_conf < MIN_CONFIDENCE || west_size < MIN_OBJECT_SIZE ||
          west_size > MAX_OBJECT_SIZE || west_mm > MAX_DISTANCE_MM)
        west = -1.0f;
    }

    int currentTime = pros::millis();

    // Only update MCL periodically or when significant motion has occurred
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
      // CRITICAL: Get raw odometry pose BEFORE any MCL corrections
      // We need to get it before setPose() to avoid circular dependency
      // The raw odometry comes from tracking wheels, not MCL-corrected pose
      lemlib::Pose currentRawOdomPose = chassisPtr->getPose();
      
      // Calculate motion delta from raw odometry (before MCL correction)
      lemlib::Pose motionDelta = calculateMotionDelta(currentRawOdomPose);

      // Motion update
      float motion_magnitude = std::sqrt(motionDelta.x * motionDelta.x +
                                         motionDelta.y * motionDelta.y);
      if (motion_magnitude > MOTION_NOISE_THRESHOLD) {
        motionUpdate(motionDelta);
      }

      // Measurement update (fuses all 4 sensors)
      measurementUpdate(north, south, east, west);

      // Only resample periodically to prevent particle depletion
      resampleCounter++;
      if (resampleCounter >= RESAMPLING_INTERVAL) {
        resampleParticles();
        resampleCounter = 0;
      }

      // Get estimated pose
      lemlib::Pose estimatedPose = getEstimatedPose();

      // Update chassis position (corrects odometry drift)
      chassisPtr->setPose(estimatedPose.x, estimatedPose.y, estimatedPose.theta);
      
      // Update lastOdomPose for blending in getEstimatedPose (use raw odometry, not corrected)
      // This ensures blending uses the actual odometry, not the MCL-corrected pose
      lastOdomPose = currentRawOdomPose;

      lastUpdateTime = currentTime;
    }

    pros::delay(MCL_DELAY);
  }
}

void startMCL(lemlib::Chassis &chassis) {
  if (mclTaskHandle != nullptr) {
    stopMCL(); // Stop existing task if running
  }

  chassisPtr = &chassis;
  mclRunning = true;
  mclTaskHandle = new pros::Task(mclTask, nullptr, "MCL Task");
}

void stopMCL() {
  if (mclTaskHandle != nullptr) {
    mclRunning = false;
    pros::delay(MCL_DELAY * 2); // Give task time to stop
    delete mclTaskHandle;
    mclTaskHandle = nullptr;
    chassisPtr = nullptr;
  }
}

#endif // ENABLE_MCL
