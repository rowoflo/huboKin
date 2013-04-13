#include <iostream>
#include <assert.h>

#include "HuboKin.h"

using namespace std;
using namespace HK;

int main(int argc, char *argv[]) {
  HuboKin hk;
  Vector6d q0, q1, q, dq;
  Isometry3d B0, B1;
  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d> > B;
  int side = HuboKin::SIDE_LEFT;

  // Set initial joint angles
  q0 << 0, -.3, 0, 0, 0, 0;
  q1 << 0, -.3, 0, M_PI/2, 0, 0;
  
  // FK
  hk.armFK(B0, q0, side);
  hk.armFK(B1, q1, side);

  // IK
  hk.armIK(q, B0, q0, side);
  
  // Jacobian
  Matrix66d J;
  hk.armJacobian(J, q, side, B0);

  // Differential IK
  hk.armDifferentialIK(dq, B1, 1.0, q0, side);

  // Print results 
  if (side == HuboKin::SIDE_RIGHT) {
    cout << "Right Side" << endl;
  } else {
    cout << "Left Side" << endl;
  }

  cout << "Initial joint angles: " << q0.transpose() << endl;
  cout << "Joint angle after FK then IK: " << q.transpose() << endl;
  cout << "End-effector transform:\n" << B0.matrix() << endl;
  cout << "Jacobian: \n" << J.matrix() << endl;
  cout << "Change in q: \n" << dq.transpose() << endl;
  
  

  return 0;
}
