
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;


// limits the min and max value of val
double limit(double min, double val, double max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

// maps val based on an f range to a t range
double map(double val, double fMin, double fMax, double tMin, double tMax) {
  return (val - fMin) * (tMax - tMin) / (fMax - fMin) + tMin;
} 

void callback(Robot* robot) {
    /*
    cout << endl;
    cout << "robot x =" << robot->pos_x << endl;
    cout << "robot y =" << robot->pos_y << endl;
    */
    double dx = goal_x - robot->pos_x;
    double dy = goal_y - robot->pos_y;
    if (abs(dx) < 0.75 && abs(dy) < 0.75) {
        cout << "we win!" << endl;
        robot->set_vel(0.0);
        robot->set_turn(0.0);
        robot->done();
        return;
    }

    
    // Count how many hits are coming from the front left/right
    bool evade = false;
    int turnLeft = 0;
    int turnRight = 0;
    for (LaserHit hit : robot->hits) {
        if (hit.range < 2 && abs(hit.angle) < 1.2 ) {  // min distance and angle
          evade = true;
            if (hit.angle <= 0.5 && hit.angle > 0) {   // on the left
              turnRight++; 
            } else if (hit.angle >= -0.5 && hit.angle < 0) { // on the right
              turnLeft++;
            }
        }
    }
    double turn = 0;
    double vel = 0;
    if (!evade) { // if no hits were detected, stir towards goal
      double angle = atan2(dy, dx) - robot->pos_t;
      turn = limit(-0.3, -angle, 0.3);

    } else {  // if there were hits, calculate how much to turn (0-0.5)

        if (turnLeft >= turnRight) {
          turnLeft = limit(0, turnLeft, 20);
          turn = map(turnLeft, 0, 20, 0, -0.5);

        } else if (turnRight > turnLeft) {
          turnRight = limit(0, turnRight, 20);
          turn = map(turnRight, 0, 20, 0, 0.5);
        }
        
      }
  
  vel = map(abs(turn), 0, 0.5, 15, 2); // calculate velocity based on turn angle 
  
  // set values
  robot->set_vel(vel);
  robot->set_turn(turn);
}

int main(int argc, char* argv[]) {
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
