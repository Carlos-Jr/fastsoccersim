#include "robot.hpp"
#include "ball.hpp"
#include <iostream>
#include <cmath>

int main() {
    double robotDistanceToCenter = 0.1; // Distância das rodas ao centro do robô
    double timeStep = 0.1;              // Passo de tempo da simulação (segundos)

    Robot robot(robotDistanceToCenter, timeStep);
    Robot robot2(robotDistanceToCenter, timeStep); //robô parado, só pra teste de colisão
    Ball ball(1.0, 1.0, 0.05, 0.5, timeStep);

    robot.setX(0.0);
    robot.setY(0.0);
    robot.setAngle(0.0);
    robot.setV1(0.0);
    robot.setV2(0.0);

    robot2.setX(2);
    robot2.setY(2.82);
    robot2.setAngle(0.0);
    robot2.setV1(0.0);
    robot2.setV2(0.0);

    ball.setElasticity(1.0);

    double simulationTime = 10.0; // Tempo total de simulação (segundos)
    double currentTime = 0.0;
    std::cout<<"object,x,y,angle\n";
    
    while (currentTime < simulationTime) {
        double dx = ball.getX() - robot.getX();
        double dy = ball.getY() - robot.getY();
        double distanceToBall = std::sqrt(dx * dx + dy * dy);
        double angleToBall = std::atan2(dy, dx);

        double angleDifference = angleToBall - robot.getAngle();

        if (angleDifference > M_PI) angleDifference -= 2 * M_PI;
        if (angleDifference < -M_PI) angleDifference += 2 * M_PI;

        if (distanceToBall > 0.5) {
            robot.setV1(0.8);
            robot.setV2(1.0);
        } else if (std::abs(angleDifference) > 0.1) {
            robot.setV1(-1.0);
            robot.setV2(1.0);
        } else {
            robot.setV1(1.0);
            robot.setV2(1.0);
        }

        robot.step();
        ball.step();

        if (ball.checkCollision(&robot)) {
            ball.reactTo(&robot);
        }
        if (ball.checkCollision(&robot2)) {
            ball.reactTo(&robot2);
        }

        robot.printCSVstate();
        ball.printCSVstate();

        // Incrementa o tempo
        currentTime += timeStep;
    }

    return 0;
}
