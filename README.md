# imu-ekf-attitude

6-DOF EKF fusing MPU6050 gyro rates with the accelerometer gravity vector for
roll/pitch. Gyro biases are tracked as states, not just subtracted once.

## State

    x = [phi, theta, bp, bq, br]^T

Prediction uses the Euler kinematics matrix; the measurement update maps
gravity into body-frame ax/ay. Covariance update is Joseph form so P stays
positive semi-definite through long accelerometer dropouts.

## Layout

    src/ekf_6dof.cpp               filter, Eigen3 only
    scripts/allan_variance_calib.py  static capture -> ARW + bias instability
    data/mpu6050_bench_raw.csv     ~30 s static bench log, 200 Hz

## Build

    make            # compiles src/ekf_6dof.cpp to build/ekf_6dof.o
    make calib      # runs the Allan variance script on the bench capture

Needs Eigen3 headers. On Debian: `apt install libeigen3-dev`.

## Notes

- Accel frames outside 0.7-1.3 g are rejected outright. Under sustained
  linear acceleration you are coasting on the gyro and the covariance grows.
- Euler parameterization is fine at moderate tilt; it will gimbal-lock near
  +/-90 deg pitch. Quaternion state is the next step if this goes on acro.


## Expanded project documentation

The `docs/` directory contains project-specific design notes, verification scenario specifications, operations guides, data contracts, configuration templates, safety notes, and project reference templates.

**Important:** added documents are planning and review aids. They do not claim that tests have been executed, that placeholder configurations are supported runtime profiles, or that this project is certified for physical/clinical/production deployment. Confirm interfaces and limits against implementation and measured evidence.
