#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>
#include <iostream>

class Robot {
private:
    // Propriedades do robô
    double v1; // Velocidade da roda esquerda
    double v2; // Velocidade da roda direita
    double angle; // Ângulo do robô (em radianos)
    double x; // Posição x do robô
    double y; // Posição y do robô
    double timeStep; // Tempo por passo de simulação
    double d; // Distância da roda ao centro do robô

public:
    Robot(double d, double timeStep) : v1(0), v2(0), angle(0), x(0), y(0), timeStep(timeStep), d(d) {}

    double getV1() const { return v1; }
    void setV1(double velocity) { v1 = velocity; }

    double getV2() const { return v2; }
    void setV2(double velocity) { v2 = velocity; }

    double getAngle() const { return angle; }
    void setAngle(double newAngle) { angle = newAngle; }

    double getX() const { return x; }
    void setX(double newX) { x = newX; }

    double getY() const { return y; }
    void setY(double newY) { y = newY; }

    double getTimeStep() const { return timeStep; }
    void setTimeStep(double newTimeStep) { timeStep = newTimeStep; }

    double getD() const { return d; }
    void setD(double newD) { d = newD; }

    void step() {
        double linearVelocity = (v1 + v2) / 2.0;
        double angularVelocity = (v2 - v1) / (2.0 * d);
        angle += angularVelocity * timeStep;
        angle = fmod(angle, 2 * M_PI);
        if (angle < 0) {
            angle += 2 * M_PI;
        }
        x += linearVelocity * cos(angle) * timeStep;
        y += linearVelocity * sin(angle) * timeStep;
    }
    void printCSVstate() const {
        std::cout << "robot,"<< x << "," << y << "," << angle << "\n";
    }
};

#endif