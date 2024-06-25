#include "cart_pendulum.hpp"
#include "controller.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void drawWheels(double cartPosition) {
    // Draw wheels
    plt::plot({cartPosition - 0.1, cartPosition + 0.1}, {0, 0}, "ko");  // Wheels
}



int main() {
    // System parameters
    const double cartMass = 1.0;
    const double pendulumMass = 0.1;
    const double pendulumLength = .5;

    // Create CartPendulum instance
    CartPendulum system(cartMass, pendulumMass, pendulumLength);
    //Controller controller; 

    // Simulation parameters
    double controlForce = 0;
    const double dt = 0.01;
    const double simulationTime = 10;

    // Number of steps
    int numSteps = static_cast<int>(simulationTime / dt);

    // Set up plot
    plt::figure_size(600, 600);
    plt::axis("equal");
    const double minY = -1.5;
    const double maxY = 1.5;
    plt::ylim(minY, maxY);
    
    // Animation loop
    for (int i = 0; i < numSteps; ++i) {
        // Clear the previous plot
        plt::clf();
        //controlForce = controller.computeControlForce(system.getStateObjects());
        
        system.update(controlForce, dt);

        // Plot the cart
        double cartWidth = 0.5; // Adjust the width of the rectangle as needed
        double cartHeight = 0.1; // Adjust the height of the rectangle as needed
        double cartLeft = system.getCartPosition() - cartWidth / 2;
        double cartRight = system.getCartPosition() + cartWidth / 2;
        plt::plot({cartLeft, cartRight, cartRight, cartLeft, cartLeft},
                  {0, 0, cartHeight, cartHeight, 0}, "b-");

        // Plot the pendulum
       // Plot the pendulum
double pendulumX = system.getCartPosition();
double pendulumY = 0.0;
// Adjust the angle for plotting (assuming 0 radians corresponds to pi/2 in dynamics)
double adjustedAngle = system.getPendulumAngle() +M_PI / 2.0;
double pendulumStringX = pendulumX + pendulumLength * cos(adjustedAngle);
double pendulumStringY = pendulumY + pendulumLength * sin(adjustedAngle);
plt::plot({pendulumX, pendulumStringX}, {pendulumY, pendulumStringY}, "k-");

// Plot the pendulum mass
std::vector<double> pendulumStringXVec = {pendulumStringX};
std::vector<double> pendulumStringYVec = {pendulumStringY};
plt::plot(pendulumStringXVec, pendulumStringYVec, "ro");



        // Draw wheels
        drawWheels(system.getCartPosition());

        // Add labels and title
        plt::title("Cart-Pendulum Animation");
        plt::xlabel("Cart Position");
        plt::ylabel("Pendulum Height");

        // Set the animation speed
        plt::pause(0.01);
        std::cout<<system.getCartPosition()<<std::endl; 
        std::cout<<controlForce<<std::endl;
        std::cout<<system.getPendulumAngle()<<std::endl;
    }

    // Display the final plot
    plt::show();

    return 0;
}
