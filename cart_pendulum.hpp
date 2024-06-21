
#ifndef CART_PENDULUM_HPP
#define CART_PENDULUM_HPP
#include <Eigen/Dense>


/**
 * @brief Represents a cart-pendulum system.
 *
 * This class models the dynamics of an inverted pendulum mounted on a cart.
 * The equations of motion are simulated to analyze the behavior of the system.
 */

class CartPendulum {
private:
    double M; // Mass of the cart.
    double m; // Mass of the pendulum.
    double I; // Moment of inertia of the pendulum.
    double l; // Length of the pendulum.
    double x; // Position of the cart along the x-axis.
    double z; // Vertical position of the pendulum's center of mass.
    double theta; // < Angle of the pendulum.
    double x_dot; // Velocity of the cart.
    double theta_dot; //< Angular velocity of the pendulum.

public:
    /**
     * @brief Constructor for the CartPendulum class.
     *
     * @param cartMass Mass of the cart.
     * @param pendulumMass Mass of the pendulum.
     * @param pendulumLength Length of the pendulum.
     */
    CartPendulum(const double cartMass, const double pendulumMass, const double pendulumLength);

    /**
     * @brief Update the state of the system based on the RK4 integration scheme.
     *
     * @param dt Time step.
     * @param controlForce The force applied to the cart.
     * @param k1_x_dot, k2_x_dot, k3_x_dot, k4_x_dot Intermediate values for cart velocity.
     * @param k1_theta_dot, k2_theta_dot, k3_theta_dot, k4_theta_dot Intermediate values for pendulum angular velocity.
     */
    // void RK_44update(double dt, double controlForce, double k1_x_dot, double k2_x_dot, double k3_x_dot, double k4_x_dot,
    //             double k1_theta_dot, double k2_theta_dot, double k3_theta_dot, double k4_theta_dot);

    void update(double controlForce,double dt);

    /**
     * @brief Get the position of the cart along the x-axis.
     *
     * @return The position of the cart.
     */

    
    double getCartPosition() const;

    /**
     * @brief Get the angle of the pendulum.
     *
     * @return The angle of the pendulum.
     */
    double getPendulumAngle() const;

     /* @brief Calculate acceleration of the cart.
     *
     * @param controlForce The force applied to the cart.
     * @param theta Angle of the pendulum.
     * @param theta_dot Angular velocity of the pendulum.
     * @param x Position of the cart.
     * @param x_dot Velocity of the cart.
     * @return The acceleration of the cart.
     */
    double calculateCartAcceleration(double controlForce, double theta, double theta_dot, double x, double x_dot) const; 

    /**
     * @brief Calculate acceleration of the pendulum.
     *
     * @param theta Angle of the pendulum.
     * @param theta_dot Angular velocity of the pendulum.
     * @return The acceleration of the pendulum.
     */
 /**
     * @brief Calculate the angular acceleration of the pendulum.
     *
     * @param theta The angle of the pendulum.
     * @param theta_dot The angular velocity of the pendulum.
     * @param x_double_dot The acceleration of the cart.
     * @return The angular acceleration of the pendulum.
     */
    double calculateThetaAcceleration(double controlForce, double theta, double theta_dot, double x_double_dot) const;


    /**
     * @brief Get the angular velocity of the pendulum.
     *
     * @return The angular velocity of the pendulum.
     */
    double getThetaDot() const;

    /**
     * @brief Get the velocity of the cart.
     *
     * @return The velocity of the cart.
     */
    double getCartVelocity() const;

    Eigen::VectorXd getStateObjects() const;


     // Setters for initial conditions
    void setX(double value);
    void setTheta(double value);
    void setXDot(double value);
    void setZDot(double value);
    void setThetaDot(double value);
};

#endif // CART_PENDULUM_HPP
