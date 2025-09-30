# INS-GNSS Integration Project

This project implements an Inertial Navigation System (INS) and Global Navigation Satellite System (GNSS) integration using Unscented Kalman Filter (UKF) for advanced robot navigation. The implementation includes both feedback and feedforward architectures for state estimation and trajectory tracking.

## Project Overview

The project demonstrates the integration of inertial sensors (gyroscopes and accelerometers) with GNSS measurements to provide accurate position, velocity, and attitude estimation for robotic navigation systems. Two different approaches are implemented:

1. **Feedback Architecture** (`main_fb.py`) - 15-state UKF with bias estimation
2. **Feedforward Architecture** (`main_ff.py`) - 12-state UKF without bias estimation

## Features

- **Dual Architecture Support**: Both feedback and feedforward INS-GNSS integration
- **Unscented Kalman Filter**: Advanced nonlinear filtering for robust state estimation
- **Earth Model Integration**: WGS84 ellipsoidal Earth model with gravity calculations
- **Comprehensive State Estimation**: Position (lat/lon/alt), attitude (roll/pitch/yaw), velocity (VN/VE/VD)
- **Bias Compensation**: Accelerometer and gyroscope bias estimation (feedback only)
- **Performance Visualization**: Detailed plotting and haversine distance analysis

## File Structure

```
├── main_fb.py              # Feedback architecture implementation (15-state UKF)
├── main_ff.py              # Feedforward architecture implementation (12-state UKF)
├── process_model_fb.py     # Process model for feedback architecture
├── process_model_ff.py     # Process model for feedforward architecture
├── earth.py                # Earth geometry and gravity models (WGS84)
├── trajectory_data.csv     # Input trajectory data with sensor measurements
└── README.md              # This file
```

## State Vector Definitions

### Feedback Architecture (15 states)
- **Position**: Latitude, Longitude, Altitude (3 states)
- **Attitude**: Roll, Pitch, Yaw (3 states)
- **Velocity**: VN, VE, VD (3 states)
- **Accelerometer Bias**: X, Y, Z bias (3 states)
- **Gyroscope Bias**: X, Y, Z bias (3 states)

### Feedforward Architecture (12 states)
- **Position**: Latitude, Longitude, Altitude (3 states)
- **Attitude**: Roll, Pitch, Yaw (3 states)
- **Velocity**: VN, VE, VD (3 states)
- **Accelerometer Bias**: X, Y, Z bias (3 states)

## Data Format

The input data file (`trajectory_data.csv`) contains the following columns:
- `time`: Timestamp
- `true_lat, true_lon, true_alt`: Ground truth position
- `true_roll, true_pitch, true_heading`: Ground truth attitude
- `gyro_x, gyro_y, gyro_z`: Gyroscope measurements
- `accel_x, accel_y, accel_z`: Accelerometer measurements
- `z_lat, z_lon, z_alt`: GNSS position measurements
- `z_VN, z_VE, z_VD`: GNSS velocity measurements

## Dependencies

```python
numpy
scipy
matplotlib
haversine
pandas
```

## Usage

### Running the Feedback Architecture
```bash
python main_fb.py
```

### Running the Feedforward Architecture
```bash
python main_ff.py
```

## Key Components

### 1. UKF Implementation
- **Sigma Point Generation**: Julier sigma point method
- **Prediction Step**: State propagation using process models
- **Update Step**: Measurement fusion with GNSS data
- **Covariance Management**: Automatic covariance matrix fixing for numerical stability

### 2. Process Models
- **Attitude Update**: Rotation matrix computation with Earth rate compensation
- **Velocity Update**: Acceleration integration with gravity and Coriolis effects
- **Position Update**: Geographic coordinate propagation

### 3. Earth Model (`earth.py`)
- **WGS84 Parameters**: Semi-major axis, eccentricity, gravity constants
- **Principal Radii**: Curvature calculations for different latitudes
- **Gravity Model**: Somigliana model with vertical correction
- **Earth Rate**: Angular velocity calculations in NED frame

## Output

The implementation generates:
- **State Estimates**: Filtered position, attitude, and velocity
- **Performance Metrics**: Haversine distance between ground truth and estimates
- **Visualization Plots**: Time series plots for all state variables
- **Results Folders**: `fb_results/` and `ff_results/` containing saved plots

## Algorithm Details

### UKF Parameters
- **Alpha (α)**: 1.0 - Controls sigma point spread
- **Beta (β)**: 0.4 - Incorporates prior knowledge of distribution
- **Kappa (κ)**: 1.0 - Secondary scaling parameter
- **Time Step**: 1 second

### Noise Models
- **Process Noise**: Gaussian noise added during prediction
- **Measurement Noise**: GNSS measurement uncertainty
- **Bias Modeling**: Random walk for sensor biases (feedback only)

## Performance Analysis

The implementation includes comprehensive performance evaluation:
- **Haversine Distance**: Position accuracy in meters
- **State Comparison**: Ground truth vs. filtered estimates
- **Convergence Analysis**: Filter performance over time
- **Architecture Comparison**: Feedback vs. feedforward performance

## References

This implementation is based on principles from:
- P. D. Groves, "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd edition
- Julier, S. J., & Uhlmann, J. K. (2004). Unscented filtering and nonlinear estimation

## License

This project is licensed under the terms specified in the LICENSE file.

## Academic Context

This project was developed as part of RBE595 - Advanced Robot Navigation coursework, focusing on INS-GNSS integration techniques for autonomous vehicle navigation systems.