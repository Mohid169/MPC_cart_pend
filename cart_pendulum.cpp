#include "cart_pendulum.hpp"
#include <iostream>
#include <cmath> 

//Constants
const double g=9.81;


//Constructor for CartPendulum class
CartPendulum::CartPendulum(const double cartMass, const double pendulumMass, const double pendulumLength)
    : M(cartMass), m(pendulumMass), l(pendulumLength),
      x(0.0), z(pendulumLength), theta(M_PI/4), x_dot(0.0), theta_dot(0.0) {}

 
 void CartPendulum::update(double controlForce, double dt){
    double x_double_dot =  calculateCartAcceleration(controlForce, theta, theta_dot, x, x_dot); 
    double theta_double_dot = calculateThetaAcceleration(controlForce, theta,  theta_dot, x_double_dot);

     // Update velocities using Euler integration
    x_dot += x_double_dot * dt;
    theta_dot += theta_double_dot * dt;

    // Update positions using Euler integration
    x += x_dot * dt;
    theta += theta_dot * dt;


 };

 Eigen::VectorXd CartPendulum::getStateObjects() const{
     Eigen::VectorXd stateObjects(4);
     stateObjects << x, theta, x_dot, theta_dot; 
     return stateObjects;
 }



// Getter implementations
double CartPendulum::getThetaDot() const {
    return theta_dot;
}

double CartPendulum::getCartVelocity() const {
    return x_dot;
}


double CartPendulum::getCartPosition() const {
    return x;
}

double CartPendulum::getPendulumAngle() const{
    return theta;
}

double CartPendulum::calculateCartAcceleration(double controlForce, double theta, double theta_dot, double x, double x_dot) const {
    // Calculate cart acceleration (x_double_dot)
    double numerator = controlForce + m*g*sin(theta)*cos(theta)-m*l*sin(theta)*pow(theta_dot,2);

    double denominator = M - m* (pow(sin(theta), 2));
    std::cout<<"x_accel: ";
    std::cout<<numerator/denominator<<std::endl;
    return numerator / denominator;
}

double CartPendulum::calculateThetaAcceleration(double controlForce, double theta, double theta_dot, double x_double_dot) const {
    // Update the dynamics based on the provided equation
    double numerator = 1.0*controlForce*cos(theta) - m*l*pow(theta_dot,2 )*cos(theta)*sin(theta) + (M+m)*g*sin(theta);
    double denominator = (M - m * pow(sin(theta), 2)) * l;
    std::cout<<"theta accel";
    std::cout<<numerator/denominator<<std::endl;
    return numerator / denominator;
}



// Setter implementations
void CartPendulum::setX(double value) {
    x = value;
}


void CartPendulum::setTheta(double value) {
    theta = value;
}

void CartPendulum::setXDot(double value) {
    x_dot = value;
}


void CartPendulum::setThetaDot(double value) {
    theta_dot = value;
}



