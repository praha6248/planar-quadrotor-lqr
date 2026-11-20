// Unit tests for the planar quadrotor model and the discrete LQR solver.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <tuple>

#include "lqr.h"
#include "planar_quadrotor.h"

namespace {

constexpr float kMass = 0.486f;
constexpr float kInertia = 0.00383f;
constexpr float kArm = 0.25f;
constexpr float kG = 9.81f;

// State derivative z_dot = f(z, u). The model integrates with explicit Euler,
// so one step of length dt gives exactly z + dt * f(z, u).
Eigen::VectorXf derivative(const Eigen::VectorXf &z, Eigen::Vector2f u, float dt = 1e-3f) {
    PlanarQuadrotor q(z);
    Eigen::VectorXf next = q.Update(u, dt);
    return (next - z) / dt;
}

// Gain matrix with the same weights as the simulator (simulate.cpp).
Eigen::MatrixXf simulator_gain(PlanarQuadrotor &q, float dt) {
    Eigen::MatrixXf A, B;
    std::tie(A, B) = q.Linearize();
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Q.diagonal() << 10, 10, 10, 1, 10, 1.0f / M_PI;
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf Ad = Eigen::MatrixXf::Identity(6, 6) + dt * A;
    Eigen::MatrixXf Bd = dt * B;
    return LQR(Ad, Bd, Q, R);
}

}  // namespace

// ---------------------------------------------------------------- dynamics

TEST(Dynamics, HoverInputCancelsGravity) {
    PlanarQuadrotor q(Eigen::VectorXf::Zero(6));
    Eigen::Vector2f u = q.GravityCompInput();
    EXPECT_NEAR(u.sum(), kMass * kG, 1e-5);
    EXPECT_FLOAT_EQ(u[0], u[1]);
    EXPECT_LT(derivative(Eigen::VectorXf::Zero(6), u).norm(), 1e-4);
}

TEST(Dynamics, FreeFallAcceleratesDownward) {
    Eigen::VectorXf zd = derivative(Eigen::VectorXf::Zero(6), Eigen::Vector2f::Zero());
    EXPECT_NEAR(zd[4], -kG, 1e-3);  // y_ddot
    EXPECT_NEAR(zd[3], 0.0, 1e-6);  // x_ddot
    EXPECT_NEAR(zd[5], 0.0, 1e-6);  // theta_ddot
}

TEST(Dynamics, DifferentialThrustRotates) {
    Eigen::VectorXf zd = derivative(Eigen::VectorXf::Zero(6), Eigen::Vector2f(1.0f, 0.0f));
    EXPECT_NEAR(zd[5], kArm / kInertia, 1e-1);
}

TEST(Dynamics, TiltedThrustPushesSideways) {
    Eigen::VectorXf z = Eigen::VectorXf::Zero(6);
    z[2] = 0.3f;  // positive tilt
    Eigen::VectorXf zd = derivative(z, Eigen::Vector2f::Constant(kMass * kG / 2));
    EXPECT_NEAR(zd[3], -kG * std::sin(0.3f), 1e-3);
}

TEST(Dynamics, PositionIntegratesVelocity) {
    Eigen::VectorXf z = Eigen::VectorXf::Zero(6);
    z << 0, 0, 0, 1.0f, -2.0f, 0.5f;
    Eigen::VectorXf zd = derivative(z, Eigen::Vector2f::Constant(kMass * kG / 2));
    EXPECT_NEAR(zd[0], 1.0, 1e-3);
    EXPECT_NEAR(zd[1], -2.0, 1e-3);
    EXPECT_NEAR(zd[2], 0.5, 1e-3);
}

// ---------------------------------------------------------------- linearisation

TEST(Linearize, MatchesFiniteDifferenceJacobian) {
    PlanarQuadrotor q(Eigen::VectorXf::Zero(6));
    Eigen::MatrixXf A, B;
    std::tie(A, B) = q.Linearize();
    ASSERT_EQ(A.rows(), 6); ASSERT_EQ(A.cols(), 6);
    ASSERT_EQ(B.rows(), 6); ASSERT_EQ(B.cols(), 2);

    const Eigen::Vector2f u0 = q.GravityCompInput();
    const Eigen::VectorXf z0 = Eigen::VectorXf::Zero(6);
    const float h = 1e-2f;

    for (int j = 0; j < 6; j++) {
        Eigen::VectorXf dz = Eigen::VectorXf::Zero(6);
        dz[j] = h;
        Eigen::VectorXf col = (derivative(z0 + dz, u0) - derivative(z0 - dz, u0)) / (2 * h);
        for (int i = 0; i < 6; i++)
            EXPECT_NEAR(A(i, j), col[i], 2e-2) << "A(" << i << "," << j << ")";
    }
    for (int j = 0; j < 2; j++) {
        Eigen::Vector2f du = Eigen::Vector2f::Zero();
        du[j] = h;
        Eigen::VectorXf col = (derivative(z0, u0 + du) - derivative(z0, u0 - du)) / (2 * h);
        for (int i = 0; i < 6; i++)
            EXPECT_NEAR(B(i, j), col[i], 2e-1) << "B(" << i << "," << j << ")";
    }
}

// ---------------------------------------------------------------- LQR

TEST(LQR, ScalarSystemMatchesRiccatiSolution) {
    // x[k+1] = x[k] + u[k], Q = R = 1  ->  P = (1 + sqrt 5) / 2, K = P / (1 + P)
    Eigen::MatrixXf one = Eigen::MatrixXf::Ones(1, 1);
    Eigen::MatrixXf K = LQR(one, one, one, one);
    const double P = (1.0 + std::sqrt(5.0)) / 2.0;
    EXPECT_NEAR(K(0, 0), P / (1.0 + P), 1e-4);
}

TEST(LQR, ClosedLoopIsStable) {
    PlanarQuadrotor q(Eigen::VectorXf::Zero(6));
    const float dt = 1e-3f;
    Eigen::MatrixXf K = simulator_gain(q, dt);
    ASSERT_EQ(K.rows(), 2); ASSERT_EQ(K.cols(), 6);

    Eigen::MatrixXf A, B;
    std::tie(A, B) = q.Linearize();
    Eigen::MatrixXf Acl = Eigen::MatrixXf::Identity(6, 6) + dt * A - dt * B * K;
    double rho = Acl.eigenvalues().cwiseAbs().maxCoeff();
    EXPECT_LT(rho, 1.0);
}

TEST(LQR, NonlinearSimulationReachesGoal) {
    Eigen::VectorXf z0 = Eigen::VectorXf::Zero(6);
    z0 << -1.0f, 0.5f, 0.2f, 0, 0, 0;
    PlanarQuadrotor q(z0);
    Eigen::VectorXf goal = Eigen::VectorXf::Zero(6);
    goal << 1.0f, 1.5f, 0, 0, 0, 0;
    q.SetGoal(goal);

    const float dt = 1e-3f;
    Eigen::MatrixXf K = simulator_gain(q, dt);
    for (int k = 0; k < 15000; k++) {  // 15 s of flight
        Eigen::Vector2f u = q.GravityCompInput() - K * q.GetControlState();
        q.Update(u, dt);
    }
    Eigen::VectorXf err = q.GetControlState();
    EXPECT_LT(err.head(2).norm(), 0.05) << "position error";
    EXPECT_LT(std::abs(err[2]), 0.05) << "attitude error";
    EXPECT_LT(err.tail(3).norm(), 0.05) << "velocity error";
}
