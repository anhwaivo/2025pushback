#pragma once

/**
 * ============================================================================
 * MONTE CARLO LOCALIZATION CONFIGURATION FILE
 * ============================================================================
 */

// ============================================================================
// CORE CONSTANTS
// ============================================================================

// Particle quantity (u-k-g repo uses 7500)
#define PARTICLE_QUANTITY 2000

// Field dimensions
#define FIELD_DIMENSIONS 144.0f  // 144 inches

// Task delay
#define MCL_DELAY 20  // Run at 50Hz

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

// Distance sensor port numbers
#define MCL_SENSOR_NORTH_PORT 7   // Port for sensor facing North (+Y, forward/trước)
#define MCL_SENSOR_SOUTH_PORT 4  // Port for sensor facing South (-Y, backward/sau đít)
#define MCL_SENSOR_EAST_PORT  5   // Port for sensor facing East (+X, right/phải)
#define MCL_SENSOR_WEST_PORT  6  // Port for sensor facing West (-X, left/trái)

// Sensor offsets relative to the tracking center (in inches)
// IMPORTANT: These offsets are measured in ROBOT FRAME (not field frame)
#define NORTH_SENSOR_X_OFFSET -7.15f  // 
#define NORTH_SENSOR_Y_OFFSET -7.15f  // 

#define SOUTH_SENSOR_X_OFFSET 6.65f  // 
#define SOUTH_SENSOR_Y_OFFSET 6.65f  // 

#define EAST_SENSOR_X_OFFSET 4.7f  // 
#define EAST_SENSOR_Y_OFFSET 4.7f  // 

#define WEST_SENSOR_X_OFFSET 4.45f  // 
#define WEST_SENSOR_Y_OFFSET 4.45f  //   

// Boolean flags to control sensor usage
#define USE_NORTH_SENSOR true
#define USE_SOUTH_SENSOR true
#define USE_EAST_SENSOR true
#define USE_WEST_SENSOR true

// ============================================================================
// MOTION UPDATE PARAMETERS
// ============================================================================

// Minimum motion required before adding noise (in inches)
#define MIN_MOTION_FOR_NOISE 0.5f

// Motion noise standard deviation (in inches)
#define MOTION_NOISE_STD_DEV 0.03f

// Rotation noise standard deviation (in degrees)
#define ROTATION_NOISE_STD_DEV 0.05f

// Motion threshold for triggering updates (in inches)
#define MOTION_NOISE_THRESHOLD 0.25f  // Threshold to consider motion as static

// ============================================================================
// MEASUREMENT UPDATE PARAMETERS
// ============================================================================

// Base measurement sigma (reduced for better drift correction)
// Note: Actual sigma is distance-dependent (2.0-4.0 inches) in measurementUpdate
#define MEASUREMENT_SIGMA_BASE 3.0f

// Distance sensor tuning constant (1.02 = +2% correction for over-estimation)
#define DISTANCE_SENSOR_CORRECTION_FACTOR 1.02f

// Minimum distance change to trigger measurement update (in inches)
#define DISTANCE_CHANGE_THRESHOLD 0.25f  // Adjust based on sensor noise

// ============================================================================
// RESAMPLING PARAMETERS
// ============================================================================

// How often to resample (every N updates)
#define RESAMPLING_INTERVAL 3

// Resampling noise to prevent particle depletion
#define RESAMPLE_NOISE_X 0.1f
#define RESAMPLE_NOISE_Y 0.1f
#define RESAMPLE_NOISE_THETA 0.05f

// ============================================================================
// POSE FILTERING PARAMETERS
// ============================================================================

// Low-pass filter alpha (0.0 to 1.0)
// Increased to 0.5 to smooth MCL estimates more and reduce sudden corrections
#define FILTER_ALPHA 0.5f

// Odometry trust factor (0.0 to 1.0)
// Lower = trust MCL more (better for correcting odometry drift)
// Higher = trust odometry more (smoother but accumulates drift)
// u-k-g uses 0.5 (50/50 blend)
// Increased to 0.7 to trust odometry more and reduce MCL over-correction
#define ODOMETRY_TRUST_FACTOR 0.7f  // 0.7 = trust odometry 70%, MCL 30%

// ============================================================================
// UPDATE FREQUENCY PARAMETERS
// ============================================================================

// Update interval in milliseconds (time-based, u-k-g uses 1000ms)
#define UPDATE_INTERVAL 200

// Distance-based update override (update if moved >2 inches)
#define MIN_UPDATE_DISTANCE 2.0f  // Override time limit if moved this distance (inches)

// ============================================================================
// WEIGHT PARAMETERS
// ============================================================================

// Minimum particle weight floor
// #define MIN_WEIGHT 0.1f  // Minimum weight floor for particles
#define MIN_WEIGHT 1e-6f

// ============================================================================
// SENSOR FILTERING PARAMETERS
// ============================================================================

// Minimum confidence value (0-63)
#define MIN_CONFIDENCE 45  // Reject readings below this confidence

// Object size range (0-400) - used to filter out game elements
#define MIN_OBJECT_SIZE 50  // Reject readings below this (too small = game element)
#define MAX_OBJECT_SIZE 401  // Reject readings above this (invalid)

// Maximum valid distance in mm
#define MAX_DISTANCE_MM 250  // ~8.3 inches

// ============================================================================
// PARTICLE INITIALIZATION PARAMETERS
// ============================================================================

// Particle initialization spread (in inches)
#define INIT_X_SPREAD 1.0f  // Spread in X direction
#define INIT_Y_SPREAD 1.0f  // Spread in Y direction
#define INIT_THETA_SPREAD 0.0f  // Spread in heading (0 = no spread, use exact heading)

// ============================================================================
// SENSOR FOV PARAMETERS
// ============================================================================

// Sensor Field of View (FOV) half-angle in degrees
#define SENSOR_FOV_HALF_ANGLE_DEG 12.0f  // 12° in radians
