#ifndef BALL_H
#define BALL_H

#include <cmath>
#include <iostream>
#include "robot.hpp"

class Ball {
private:
    double x, y;     // Posição da bola
    double vx, vy;   // Velocidade da bola
    double radius;   // Raio da bola
    double mass;     // Massa da bola
    double elasticity; // Constante elástica da bola
    double timeStep; // Tempo por passo de simulação

public:
    Ball(double x, double y, double radius, double mass, double timeStep)
        : x(x), y(y), vx(0), vy(0), radius(radius), mass(mass), timeStep(timeStep){}

    double getX() const { return x; }
    void setX(double x) { this->x = x; }

    double getY() const { return y; }
    void setY(double y) { this->y = y; }

    double getRadius() const { return radius; }
    void setRadius(double radius) { this->radius = radius; }

    double getMass() const { return mass; }
    void setMass(double mass) { this->mass = mass; }

    double getVx() const { return vx; }
    void setVx(double vx) { this->vx = vx; }

    double getVy() const { return vy; }
    void setVy(double vy) { this->vy = vy; }

    void setV(double vx, double vy) {
        this->vx = vx;
        this->vy = vy;
    }

    double getAngle() const{
        return std::atan2(vy, vx);
    }

    double getElasticity() const { return elasticity; }
    void setElasticity(double elasticity) { this->elasticity = elasticity; }

    void step() {
        x += vx * timeStep;
        y += vy * timeStep;
    }

    bool checkCollision(const Robot* robot) {
        // Calcular os limites do robô considerando sua rotação
        double robotX = robot->getX();
        double robotY = robot->getY();
        double robotD = robot->getD();
        double robotAngle = robot->getAngle();

        double cosAngle = std::cos(robotAngle);
        double sinAngle = std::sin(robotAngle);

        // Coordenadas do centro da bola em relação ao robô rotacionado
        double relativeX = x - robotX;
        double relativeY = y - robotY;
        double rotatedX = relativeX * cosAngle + relativeY * sinAngle;
        double rotatedY = -relativeX * sinAngle + relativeY * cosAngle;

        // Verificar se a bola está dentro dos limites do robô (quadrado rotacionado)
        double robotHalfSize = robotD;
        double closestX = std::max(-robotHalfSize, std::min(rotatedX, robotHalfSize));
        double closestY = std::max(-robotHalfSize, std::min(rotatedY, robotHalfSize));

        // Transformar de volta para o espaço original
        double worldClosestX = closestX * cosAngle - closestY * sinAngle + robotX;
        double worldClosestY = closestX * sinAngle + closestY * cosAngle + robotY;

        double distance = std::sqrt((worldClosestX - x) * (worldClosestX - x) + (worldClosestY - y) * (worldClosestY - y));
        return distance <= radius;
    }

    void reactTo(const Robot* robot) {
        // Calcular os limites do robô considerando sua rotação
        double robotX = robot->getX();
        double robotY = robot->getY();
        double robotD = robot->getD();
        double robotAngle = robot->getAngle();

        double cosAngle = std::cos(robotAngle);
        double sinAngle = std::sin(robotAngle);

        // Coordenadas do centro da bola em relação ao robô rotacionado
        double relativeX = x - robotX;
        double relativeY = y - robotY;
        double rotatedX = relativeX * cosAngle + relativeY * sinAngle;
        double rotatedY = -relativeX * sinAngle + relativeY * cosAngle;

        // Verificar o ponto mais próximo do robô rotacionado
        double robotHalfSize = robotD;
        double closestX = std::max(-robotHalfSize, std::min(rotatedX, robotHalfSize));
        double closestY = std::max(-robotHalfSize, std::min(rotatedY, robotHalfSize));

        // Transformar de volta para o espaço original
        double worldClosestX = closestX * cosAngle - closestY * sinAngle + robotX;
        double worldClosestY = closestX * sinAngle + closestY * cosAngle + robotY;

        double dx = x - worldClosestX;
        double dy = y - worldClosestY;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance == 0) return;

        // Normalize the direction vector
        dx /= distance;
        dy /= distance;

        // Apply reaction velocity proportional to elasticity
        vx += elasticity * dx / mass;
        vy += elasticity * dy / mass;
    }

    void printCSVstate() const {
        std::cout << "ball," << x << "," << y << "," << getAngle() << "\n";
    }
};

#endif