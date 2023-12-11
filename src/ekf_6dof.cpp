/*
 * ekf_6dof.cpp - 6-DOF attitude EKF for an MPU6050.
 *
 * State: x = [phi, theta, bp, bq, br]^T
 *   phi, theta  roll/pitch [rad]
 *   bp, bq, br  gyro bias per axis [rad/s]
 *
 * Predict with gyro rates through the Euler kinematics matrix, update on the
 * accelerometer gravity projection. Covariance uses Joseph form so P stays
 * PSD through long accelerometer dropouts.
 *
 * Deps: Eigen3 only.
 */

#include <Eigen/Dense>
#include <cmath>

namespace {

constexpr int N = 5;
constexpr int M = 2;

using Vec5 = Eigen::Matrix<float, N, 1>;
using Mat5 = Eigen::Matrix<float, N, N>;
using Mat25 = Eigen::Matrix<float, M, N>;
using Vec2 = Eigen::Matrix<float, M, 1>;
using Mat2 = Eigen::Matrix<float, M, M>;

/* Tuning from the Allan-variance bench run, see scripts/allan_variance_calib.py. */
constexpr float GYRO_NOISE     = 7.6e-5f;   // rad/s/sqrt(Hz), ARW
constexpr float GYRO_BIAS_WALK = 1.3e-5f;   // rad/s/sqrt(s), ~10% of BI
constexpr float ACCEL_NOISE    = 0.025f;    // m/s^2, 1-sigma on the bench
constexpr float GRAVITY        = 9.80665f;
constexpr float MAX_DT         = 0.05f;     // clamp against loop stalls

struct Ekf6Dof {
    Vec5 x;
    Mat5 P;
    Mat5 Q;
    Mat2 R;
    bool initialized = false;
};

inline float wrap_pi(float a)
{
    constexpr float PI = 3.14159265358979f;
    while (a > PI)   a -= 2.0f * PI;
    while (a <= -PI) a += 2.0f * PI;
    return a;
}

void ekf_init(Ekf6Dof &f)
{
    f.x.setZero();
    f.P.setIdentity();
    f.P.block<2, 2>(0, 0) *= 0.5f;    // attitude, roughly level at boot
    f.P.block<3, 3>(2, 2) *= 1e-3f;   // bias, close to factory trim

    f.Q.setZero();
    f.Q.block<2, 2>(0, 0) = Eigen::Matrix2f::Identity() * (GYRO_NOISE * GYRO_NOISE);
    f.Q.block<3, 3>(2, 2) = Eigen::Matrix3f::Identity() * (GYRO_BIAS_WALK * GYRO_BIAS_WALK);

    f.R = Eigen::Matrix2f::Identity() * (ACCEL_NOISE * ACCEL_NOISE);
    f.initialized = true;
}

/* F = d(xdot)/dx, linearized about the current estimate:
 *   phidot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
 *   thetadot =     q*cos(phi)             - r*sin(phi)
 * Bias rows are identity (random walk).
 */
Mat5 build_F(const Vec5 &x, float dt)
{
    const float phi  = x(0);
    const float th   = x(1);
    const float sphi = std::sin(phi);
    const float cphi = std::cos(phi);
    const float tth  = std::tan(th);
    const float sec2 = 1.0f / (std::cos(th) * std::cos(th));

    Mat5 F = Mat5::Identity();

    F(0, 0) += dt * (x(3) * cphi * tth - x(4) * sphi * tth);
    F(0, 1) += dt * ((x(3) * sphi + x(4) * cphi) * sec2);
    F(1, 0) += dt * (-x(3) * sphi - x(4) * cphi);

    // Input gains: body rates feed the attitude rows directly.
    F(0, 2) += dt;
    F(0, 3) += dt * (sphi * tth);
    F(0, 4) += dt * (cphi * tth);
    F(1, 3) += dt * cphi;
    F(1, 4) += dt * (-sphi);

    return F;
}

void ekf_predict(Ekf6Dof &f, const Eigen::Vector3f &gyr, float dt)
{
    if (!f.initialized) ekf_init(f);
    if (dt <= 0.0f) return;
    if (dt > MAX_DT) dt = MAX_DT;

    const float phi  = f.x(0);
    const float th   = f.x(1);
    const float sphi = std::sin(phi);
    const float cphi = std::cos(phi);
    const float tth  = std::tan(th);

    const float p = gyr(0) - f.x(2);
    const float q = gyr(1) - f.x(3);
    const float r = gyr(2) - f.x(4);

    f.x(0) += dt * (p + q * sphi * tth + r * cphi * tth);
    f.x(1) += dt * (q * cphi - r * sphi);
    f.x(0) = wrap_pi(f.x(0));
    f.x(1) = wrap_pi(f.x(1));

    const Mat5 F = build_F(f.x, dt);
    f.P = F * f.P * F.transpose() + f.Q * dt;
}

/* Accelerometer update. At low dynamic acceleration the specific force is -g
 * in the body frame, giving a direct observation of roll and pitch:
 *   ax =  g * sin(theta)
 *   ay = -g * sin(phi) * cos(theta)
 * Signs assume NED-ish body axes; flip if the IMU is mounted inverted.
 */
void ekf_update_accel(Ekf6Dof &f, const Eigen::Vector3f &acc)
{
    const float norm = acc.norm();
    // Reject frames where linear acceleration dominates: |a| far from 1 g.
    if (norm < 0.7f * GRAVITY || norm > 1.3f * GRAVITY) return;

    const float phi  = f.x(0);
    const float th   = f.x(1);
    const float sphi = std::sin(phi);
    const float cphi = std::cos(phi);
    const float sth  = std::sin(th);
    const float cth  = std::cos(th);

    Vec2 z, h;
    z << acc(0), acc(1);
    h << GRAVITY * sth, -GRAVITY * sphi * cth;

    Mat25 H = Mat25::Zero();
    H(0, 1) = GRAVITY * cth;
    H(1, 0) = -GRAVITY * cphi * cth;
    H(1, 1) = GRAVITY * sphi * sth;

    const Vec2 y = z - h;
    const Mat2 S = H * f.P * H.transpose() + f.R;
    const Mat25 K = f.P * H.transpose() * S.inverse();

    f.x += K * y;
    f.x(0) = wrap_pi(f.x(0));
    f.x(1) = wrap_pi(f.x(1));

    // Joseph form: P = (I-KH) P (I-KH)^T + K R K^T
    const Mat5 IKH = Mat5::Identity() - K * H;
    f.P = IKH * f.P * IKH.transpose() + K * f.R * K.transpose();

    // Keep the bias block from wandering off during long static runs.
    for (int i = 2; i < N; i++)
        f.x(i) = std::fmax(-0.35f, std::fmin(0.35f, f.x(i)));
}

} // namespace

/* --- public API ------------------------------------------------------- */

struct AttitudeEstimator {
    Ekf6Dof ekf;
    float roll = 0.0f;
    float pitch = 0.0f;
};

void attitude_init(AttitudeEstimator &est)
{
    ekf_init(est.ekf);
}

void attitude_update(AttitudeEstimator &est, const Eigen::Vector3f &gyr_rads,
                     const Eigen::Vector3f &acc_ms2, float dt_s)
{
    ekf_predict(est.ekf, gyr_rads, dt_s);
    ekf_update_accel(est.ekf, acc_ms2);
    est.roll = est.ekf.x(0);
    est.pitch = est.ekf.x(1);
}

Eigen::Vector3f attitude_gyro_bias(const AttitudeEstimator &est)
{
    return est.ekf.x.tail<3>();
}
