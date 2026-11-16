#include <ecn_manip/robot_turret.h>
#include <ecn_manip/trig_solvers.h>

// Model of Turret robot
// MDH table [alpha, a, theta, r], b = 0.5, d = 0.1:
//   1: [0,     0, q1, b ]
//   2: [pi/2,  0, q2, 0 ]
//   3: [-pi/2, 0, 0,  q3]
//   e: [0,     0, 0,  d ]

// Any constant transform at base or end-effector
void ecn::RobotTurret::init_wMe()
{
    const double d{0.1};
    wMe[0][0] = 1.; wMe[0][1] = 0;  wMe[0][2] = 0;  wMe[0][3] = 0;
    wMe[1][0] = 0;  wMe[1][1] = 1.; wMe[1][2] = 0;  wMe[1][3] = 0;
    wMe[2][0] = 0;  wMe[2][1] = 0;  wMe[2][2] = 1.; wMe[2][3] = d;
    wMe[3][0] = 0;  wMe[3][1] = 0;  wMe[3][2] = 0;  wMe[3][3] = 1.;
}

// Direct Geometry
vpHomogeneousMatrix ecn::RobotTurret::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;
    const double b{0.5};

    const auto c1{cos(q[0])};
    const auto s1{sin(q[0])};
    const auto c2{cos(q[1])};
    const auto s2{sin(q[1])};

    M[0][0] = c1*c2;  M[0][1] = -s1;  M[0][2] = -s2*c1;  M[0][3] = -q[2]*s2*c1;
    M[1][0] = s1*c2;  M[1][1] = c1;   M[1][2] = -s1*s2;  M[1][3] = -q[2]*s1*s2;
    M[2][0] = s2;     M[2][1] = 0;    M[2][2] = c2;      M[2][3] = b + q[2]*c2;
    M[3][0] = 0;      M[3][1] = 0;    M[3][2] = 0;       M[3][3] = 1.;

    return M;
}


// Inverse Geometry
vpColVector ecn::RobotTurret::inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{



    return bestCandidate(q0);
}

// Wrist Jacobian
vpMatrix ecn::RobotTurret::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);



    return J;
}
