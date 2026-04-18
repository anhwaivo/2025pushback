#pragma once

#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include <random>
#include <vector>

// Particle structure definition
struct Particle {
  lemlib::Pose pose; // Use the lemlib::Pose for x, y, theta
  float weight;      // Weight of the particle (probability)

  // Constructor - required since lemlib::Pose has no default constructor
  Particle(const lemlib::Pose &p = lemlib::Pose(0, 0, 0), float w = 0.0f)
      : pose(p), weight(w) {}
};

// Function declarations
/**
 * Initialize particles around an initial pose estimate
 * @param initialPose The initial pose estimate from chassis.getPose()
 */
void initializeParticles(const lemlib::Pose &initialPose);

/**
 * Update particles based on robot motion (prediction step)
 * @param deltaMotion The change in pose since the last update (from tracking wheels)
 */
void motionUpdate(const lemlib::Pose &deltaMotion);

/**
 * Calculate expected sensor readings for a particle
 * @param particlePose The pose of the particle
 * @param direction Character indicating sensor direction ('N', 'S', 'E', 'W')
 * @return Expected distance reading
 */
float predictSensorReading(const lemlib::Pose &particlePose,
                           const char direction);

/**
 * Update particle weights based on sensor measurements
 * Properly fuses all 4 distance sensors using multiplicative likelihood
 * @param north_dist Distance reading from north sensor
 * @param south_dist Distance reading from south sensor
 * @param east_dist Distance reading from east sensor
 * @param west_dist Distance reading from west sensor
 */
void measurementUpdate(float north_dist, float south_dist, float east_dist,
                       float west_dist);

/**
 * Resample particles based on their weights
 */
void resampleParticles();

/**
 * Get the estimated pose from particle distribution
 * @return Estimated pose of the robot (blended with odometry)
 */
lemlib::Pose getEstimatedPose();

/**
 * Background task for running Monte Carlo Localization
 * @param param Unused parameter required by PROS task API
 */
void mclTask(void *param);

/**
 * Start the MCL background task
 * @param chassis The chassis to use for localization
 */
void startMCL(lemlib::Chassis &chassis);

/**
 * Stop the MCL background task
 */
void stopMCL();
