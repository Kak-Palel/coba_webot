// File:          my_controller.cpp
// Date:
// Description:
// Author:        Kak-Palel
// Modifications:
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <vector>
#include <string>
#include <cmath>

#define MAX_SPEED 6.28
#define THRESHOLD 0.5
#define STEEP_THRESHOLD 0.00001
#define TIME_STEP 64

bool right = true; // follow the wall on the right side. If false, follow the wall on the left side

using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  int timeStep = (int)robot->getBasicTimeStep();

  std::vector<DistanceSensor *> ps(3);
  std::vector<std::string> psNames = {"ds_left", "ds_right", "ds_front"};
  
  for (int i = 0; i < 3; i++) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    ps[i]->enable(TIME_STEP);
  }

  Motor *leftMotor = robot->getMotor("wheel1");
  Motor *rightMotor = robot->getMotor("wheel2");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  std::vector<double> psValues(3);
  std::vector<double> newValues(3);
  psValues[0] = ps[0]->getValue();
  psValues[1] = ps[1]->getValue();
  psValues[2] = ps[2]->getValue();
  printf("Controller started\n");
  while (robot->step(timeStep) != -1) {
    newValues[0] = ps[0]->getValue();
    newValues[1] = ps[1]->getValue();
    newValues[2] = ps[2]->getValue();

    printf("[INFO][my_controller] Got distance sensor reading of (%f, %f, %f), following %d \n", newValues[0], newValues[1], newValues[2], right);

    double leftSpeed = 0;
    double rightSpeed = 0;

    if(right)
    {
      if(abs(newValues[1] - psValues[1]) <= STEEP_THRESHOLD)
      {
        leftSpeed = MAX_SPEED * 0.5;
        rightSpeed = MAX_SPEED * 0.5;
      }
      else if(newValues[1] > 0.325)
      {
        printf("[INFO][my_controller] Following right wall, newValues[1] > 2\n");
        leftSpeed = MAX_SPEED;
        rightSpeed = 0;
      }
      else if(newValues[1] > psValues[1])
      {
        leftSpeed = MAX_SPEED * (abs(newValues[1] - psValues[1]) >= 0.018 ? 1 : 25*(abs(newValues[1] - psValues[1]) - 0.002) + 0.6);
        rightSpeed = MAX_SPEED * 0.5;
      }
      else if (newValues[1] < psValues[1])
      {
        leftSpeed = MAX_SPEED * 0.5;
        rightSpeed = MAX_SPEED * (abs(newValues[1] - psValues[1]) >= 0.018 ? 1 : 25*(abs(newValues[1] - psValues[1]) - 0.002) + 0.6);
      }
    }
    else
    {
      if(abs(newValues[0] - psValues[0]) <= STEEP_THRESHOLD)
      {
        leftSpeed = MAX_SPEED * 0.5;
        rightSpeed = MAX_SPEED * 0.5;
      }
      else if(newValues[0] > 0.325)
      {
        leftSpeed = 0;
        rightSpeed = MAX_SPEED;
      }
      else if(newValues[0] > psValues[0])
      {
        leftSpeed = MAX_SPEED * 0.5;
        rightSpeed = MAX_SPEED * (abs(newValues[1] - psValues[1]) >= 0.018 ? 1 : 25*(abs(newValues[1] - psValues[1]) - 0.002) + 0.6);
      }
      else if (newValues[0] < psValues[0])
      {
        leftSpeed = MAX_SPEED * (abs(newValues[1] - psValues[1]) >= 0.018 ? 1 : 25*(abs(newValues[1] - psValues[1]) - 0.002) + 0.6);
        rightSpeed = MAX_SPEED * 0.5;
      }
    }

    if(psValues[2] < THRESHOLD)
    {
      if (psValues[0] > psValues[1])
      {
        leftSpeed = 0;
        rightSpeed = MAX_SPEED;
        right = true; // switch to left wall following
      }
      else if (psValues[0] < psValues[1])
      {
        leftSpeed = MAX_SPEED;
        rightSpeed = 0;
        right = false; // switch to right wall following
      }
    }

    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);

    psValues = newValues;
  };

  delete robot;
  return 0;
}
