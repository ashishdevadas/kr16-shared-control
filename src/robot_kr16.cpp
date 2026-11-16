#include <ecn_manip/robot_kr16.h>
#include <ecn_manip/trig_solvers.h>

// Model of Kuka KR16 robot
// MDH table [alpha, a, theta, r]:
//   1: [0,     0,     -q1,       0.675]
//   2: [-pi/2, 0.26,   q2,       0    ]
//   3: [0,     0.68,   q3+pi/2,  0    ]
//   4: [pi/2,  0.035, -q4,       0.67 ]
//   5: [-pi/2, 0,      q5,       0    ]
//   6: [pi/2,  0,     -q6,       0    ]
//   e: [0,     0,      0,        0.158]

// Any end-effector to wrist constant transform
void ecn::RobotKr16::init_wMe()
{
    wMe[0][0] = 1.;  wMe[0][1] = 0;   wMe[0][2] = 0;   wMe[0][3] = 0;
    wMe[1][0] = 0;   wMe[1][1] = 1.;  wMe[1][2] = 0;   wMe[1][3] = 0;
    wMe[2][0] = 0;   wMe[2][1] = 0;   wMe[2][2] = 1.;  wMe[2][3] = 0.158;
    wMe[3][0] = 0;   wMe[3][1] = 0;   wMe[3][2] = 0;   wMe[3][3] = 1.;
}

// Direct Geometry fixed to wrist frame
vpHomogeneousMatrix ecn::RobotKr16::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;

    const auto c1{cos(q[0])};
    const auto c2{cos(q[1])};
    const auto c4{cos(q[3])};
    const auto c5{cos(q[4])};
    const auto c6{cos(q[5])};
    const auto c23{cos(q[1]+q[2])};
    const auto s1{sin(q[0])};
    const auto s2{sin(q[1])};
    const auto s4{sin(q[3])};
    const auto s5{sin(q[4])};
    const auto s6{sin(q[5])};
    const auto s23{sin(q[1]+q[2])};

    M[0][0] = (-(s1*s4 + s23*c1*c4)*c5 - s5*c1*c23)*c6 - (s1*c4 - s4*s23*c1)*s6;
    M[0][1] = (-(s1*s4 + s23*c1*c4)*c5 - s5*c1*c23)*s6 + (s1*c4 - s4*s23*c1)*c6;
    M[0][2] = -(s1*s4 + s23*c1*c4)*s5 + c1*c5*c23;
    M[0][3] = (-0.035*s23 + 0.68*c2 + 0.67*c23 + 0.26)*c1;
    M[1][0] = ((s1*s23*c4 - s4*c1)*c5 + s1*s5*c23)*c6 - (s1*s4*s23 + c1*c4)*s6;
    M[1][1] = ((s1*s23*c4 - s4*c1)*c5 + s1*s5*c23)*s6 + (s1*s4*s23 + c1*c4)*c6;
    M[1][2] = (s1*s23*c4 - s4*c1)*s5 - s1*c5*c23;
    M[1][3] = (0.035*s23 - 0.68*c2 - 0.67*c23 - 0.26)*s1;
    M[2][0] = (s5*s23 - c4*c5*c23)*c6 + s4*s6*c23;
    M[2][1] = (s5*s23 - c4*c5*c23)*s6 - s4*c6*c23;
    M[2][2] = -s5*c4*c23 - s23*c5;
    M[2][3] = -0.68*s2 - 0.67*s23 - 0.035*c23 + 0.675;
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1.;

    return M;
}


// Inverse Geometry
// Inverse Geometry
vpColVector ecn::RobotKr16::inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{
    const auto [tx,ty,tz] = explodeTranslation(Md);
    const double L{0.68}, A{0.67}, B{0.035};

    for(const double q1: solveType2(tx, ty, 0))
    {
        const double rho{bestDivision(q1, tx, -ty)};
        const double X{rho - 0.26};
        const double Y{0.675 - tz};
        const double Z{(X*X + Y*Y - L*L - A*A - B*B) / (2*L)};

        for(const double q3: solveType2(-B, A, Z))
        {
            const auto [c3, s3] = cos_sin(q3);
            const double U{L + A*c3 - B*s3};
            const double V{A*s3 + B*c3};

            for(const double q2: solveType3(-V, U, X, U, V, Y))
            {
                const auto [c1, s1] = cos_sin(q1);
                const auto [c23, s23] = cos_sin(q2+q3);
                vpRotationMatrix R03;
                R03[0][0] = -s23*c1;  R03[0][1] = -c1*c23;  R03[0][2] = s1;
                R03[1][0] = s1*s23;   R03[1][1] = s1*c23;   R03[1][2] = c1;
                R03[2][0] = -c23;     R03[2][1] = s23;      R03[2][2] = 0;

                const auto [xx,xy,xz,yx,yy,yz,zx,zy,zz] = explodeWristMatrix(Md, R03);

                const double c5{-zy};
                const double s5abs{sqrt(std::max(0., 1 - c5*c5))};

                if(s5abs < 1e-6)
                {
                    const double q5{c5 > 0 ? 0. : M_PI};
                    const double q4{q0[3]};
                    const auto [c4, s4] = cos_sin(q4);
                    const double c6{c5*(c4*xx - s4*xz)};
                    const double s6{-(s4*xx + c4*xz)};
                    addCandidate({q1, q2, q3, q4, q5, atan2(s6, c6)}, Md);
                }
                else
                {
                    for(const double s5: {s5abs, -s5abs})
                    {
                        const double q5{atan2(s5, c5)};
                        const double q4{atan2(-zz/s5, zx/s5)};
                        const double q6{atan2(yy/s5, xy/s5)};
                        addCandidate({q1, q2, q3, q4, q5, q6}, Md);
                    }
                }
            }
        }
    }

    return bestCandidate(q0);
}


vpMatrix ecn::RobotKr16::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);
    // Generated Jacobian code
    // numerical values to initialize

    const auto c1{cos(q[0])};
    const auto c2{cos(q[1])};
    const auto c4{cos(q[3])};
    const auto c5{cos(q[4])};
    const auto c23{cos(q[1]+q[2])};
    const auto s1{sin(q[0])};
    const auto s2{sin(q[1])};
    const auto s4{sin(q[3])};
    const auto s5{sin(q[4])};
    const auto s23{sin(q[1]+q[2])};
    J[0][0] = (0.035*s23 - 0.68*c2 - 0.67*c23 - 0.26)*s1;
    J[0][1] = -(0.68*s2 + 0.67*s23 + 0.035*c23)*c1;
    J[0][2] = -(0.67*s23 + 0.035*c23)*c1;
    J[0][3] = 0;
    J[0][4] = 0;
    J[0][5] = 0;
    J[1][0] = -(-0.035*s23 + 0.68*c2 + 0.67*c23 + 0.26)*c1;
    J[1][1] = (0.68*s2 + 0.67*s23 + 0.035*c23)*s1;
    J[1][2] = (0.67*s23 + 0.035*c23)*s1;
    J[1][3] = 0;
    J[1][4] = 0;
    J[1][5] = 0;
    J[2][0] = 0;
    J[2][1] = 0.035*s23 - 0.68*c2 - 0.67*c23;
    J[2][2] = 0.035*s23 - 0.67*c23;
    J[2][3] = 0;
    J[2][4] = 0;
    J[2][5] = 0;
    J[3][0] = 0;
    J[3][1] = s1;
    J[3][2] = s1;
    J[3][3] = -c1*c23;
    J[3][4] = s1*c4 - s4*s23*c1;
    J[3][5] = (s1*s4 + s23*c1*c4)*s5 - c1*c5*c23;
    J[4][0] = 0;
    J[4][1] = c1;
    J[4][2] = c1;
    J[4][3] = s1*c23;
    J[4][4] = s1*s4*s23 + c1*c4;
    J[4][5] = -(s1*s23*c4 - s4*c1)*s5 + s1*c5*c23;
    J[5][0] = -1.;
    J[5][1] = 0;
    J[5][2] = 0;
    J[5][3] = s23;
    J[5][4] = -s4*c23;
    J[5][5] = s5*c4*c23 + s23*c5;
    // End of Jacobian code




    return J;
}
