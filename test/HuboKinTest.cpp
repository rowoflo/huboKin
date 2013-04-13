#include <iostream>
#include <assert.h>

#include "HuboKin.h"

using namespace std;
using namespace HK;

int main(int argc, char *argv[]) {
  HuboKin hk;
  Vector6d q0, q;
  Isometry3d B0, B1;
  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d> > B;
  int side = HuboKin::SIDE_LEFT;

  // Set initial joint angles
  q0 << 0, -.3, 0, 0, 0, 0;
  
  // FK
  hk.armFK(B0, q0, side);

  // Isometry3d hand;
  // hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
  // hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
  // hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
  // hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
  
  
  // hk.armFK(B, q0, side, hand, -1, 6);

  // for (int i = 0; i < 8; i++) {
  //   cout << B[i].matrix() << endl;
  //  }
  
  hk.armIK(q, B0, q0, side);
  
  // Jacobian
  Matrix66d J;
  hk.armJacobian(J, q, side, B0);

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
  


  return 0;
}
