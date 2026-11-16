// Shared-control framework for safe robotic inspection and intervention
// in CERN-like environments (KUKA KR16, ecn_manip simulator)
//
// Mode 2 (assisted teleoperation)  -> GUI "Manual Cartesian control"   (ControlMode::VELOCITY_MANUAL)
// Mode 1 (autonomous inspection)   -> GUI "Velocity point-to-point"     (ControlMode::VELOCITY_P2P)
//
// Version 0.6: + scene drawn in RViz (/cern_scene): tunnel, magnet-like equipment, KR16 on a parked
//              vehicle, keep-out planes and funnel turning red while they act, defects, camera footprint
// Version 0.5: + Option A: virtual fixtures generated automatically from the inspection report
//              (target pose, approach funnel, keep-out zone, depth stop, guidance) for Mode 2,
//              + E3 simulated operator (ASSIST=none|static|fixtures OPERATOR=sim TRIALS=10)
//              0.5.1: with OPERATOR=sim, both GUI velocity modes run the trials (no scan)
// Version 0.4: + Mode 1 autonomous inspection scan of the equipment front face
//              (raster scan, simulated camera, defect localisation, inspection report JSON)
//              0.4.1: camera tilt 30 deg (wrist margin), integral action along the raster
//              (robust to the simulator's velocity gain), tracking tolerance 20 mm
// Version 0.3: teleoperation + safety layer (keep-out planes, velocity damper,
//              anti-collision stop) + singularity-robust inverse (damped least squares)
//              + joint-velocity limit + CSV logging
// Options (environment variables):
//   ASSIST=none | static | fixtures (default)   no safety layer / keep-out planes only / planes + fixtures
//   SAFETY=0                                     same as ASSIST=none
//   DLS=0                                        plain pseudo-inverse instead of damped least squares
//   TARGET=D2                                    defect used for the fixtures (default: first defect)
//   OPERATOR=sim TRIALS=10                       E3: simulated operator instead of the GUI sliders

#include <ecn_manip/robot_init.h>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <array>
#include <vector>
#include <algorithm>
#include <random>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std;
using namespace ecn;

// ======================= Safety layer parameters =======================
// keep-out plane: the allowed side is  n . p >= d0  (p = tool position, fixed frame)
struct Plane
{
    string name;
    array<double, 3> n;   // unit normal, pointing to the allowed side
    double d0;
};

const vector<Plane> planes{
    {"floor", {0., 0., 1.}, 0.80},     // tool must stay above z = 0.80 m (top of delicate equipment)
    {"wall",  {-1., 0., 0.}, -1.50}    // tool must stay at x <= 1.50 m (front face of the delicate equipment)
};

const double D_INFLUENCE{0.10};   // [m]   damper starts below this distance
const double D_SAFE{0.02};        // [m]   allowed approach speed reaches 0 here
const double D_STOP{0.005};       // [m]   anti-collision stop
const double V_APPROACH{0.05};    // [m/s] max approach speed at the edge of the influence zone

// generic velocity-damper constraint (Faverjon & Tournassoud):  n . v >= -va (d - ds) / (di - ds)  when d < di
struct Constraint
{
    double d;               // signed distance to the boundary, > 0 on the allowed side [m]
    array<double, 3> n;     // unit normal pointing to the allowed side
    double di, ds, va;      // influence distance, zero-speed distance [m], max approach speed [m/s]
};

// modifies the linear part of fv (fixed frame) if it violates the constraint; returns true if modified
bool applyDamper(vpColVector &fv, const Constraint &c)
{
    if(c.d >= c.di)
        return false;
    const double vn{c.n[0]*fv[0] + c.n[1]*fv[1] + c.n[2]*fv[2]};
    // allowed normal speed; when already inside the forbidden side, the escape speed is limited to va
    const double vnMin{std::min(c.va, -c.va * (c.d - c.ds) / (c.di - c.ds))};
    if(vn >= vnMin)
        return false;
    for(unsigned i = 0; i < 3; ++i)
        fv[i] += (vnMin - vn) * c.n[i];
    return true;
}

struct SafetyInfo
{
    double dmin{1e9};     // smallest distance to any keep-out plane [m]
    bool active{false};   // a keep-out plane damper modified the command
    bool fixture{false};  // a virtual-fixture constraint modified the command
    bool stop{false};     // anti-collision stop triggered
};

// computes the distances and, if apply = true, modifies the linear part of fv (fixed frame)
// extra: additional constraints (virtual fixtures generated from the inspection)
SafetyInfo safetyLayer(vpColVector &fv, const vpTranslationVector &pt, const bool apply,
                       const vector<Constraint> &extra = {})
{
    SafetyInfo info;
    vector<Constraint> cs;
    for(const auto &pl: planes)
    {
        const double d{pl.n[0]*pt[0] + pl.n[1]*pt[1] + pl.n[2]*pt[2] - pl.d0};
        info.dmin = std::min(info.dmin, d);
        if(!apply)
            continue;

        // 1. anti-collision stop: too close and still approaching
        const double vn{pl.n[0]*fv[0] + pl.n[1]*fv[1] + pl.n[2]*fv[2]};
        if(d < D_STOP && vn < 0.)
        {
            info.stop = true;
            continue;
        }
        // 2. velocity damper: limit the approach speed near the plane
        cs.push_back({d, pl.n, D_INFLUENCE, D_SAFE, V_APPROACH});
    }
    if(apply)
    {
        // two passes so that constraints applied later do not undo earlier ones (corners)
        for(int pass = 0; pass < 2; ++pass)
        {
            for(const auto &c: cs)
                if(applyDamper(fv, c))
                    info.active = true;
            for(const auto &c: extra)
                if(applyDamper(fv, c))
                    info.fixture = true;
        }
    }
    if(info.stop)
        fv = 0;
    return info;
}

// applies the keep-out planes and the extra constraints to a twist, without the stop logic
// (used to check the twist actually realised by the damped least squares)
void applyAll(vpColVector &fv, const vpTranslationVector &pt, const vector<Constraint> &extra)
{
    for(int pass = 0; pass < 2; ++pass)
    {
        for(const auto &pl: planes)
            applyDamper(fv, {pl.n[0]*pt[0] + pl.n[1]*pt[1] + pl.n[2]*pt[2] - pl.d0, pl.n, D_INFLUENCE, D_SAFE, V_APPROACH});
        for(const auto &c: extra)
            applyDamper(fv, c);
    }
}

// ================= Singularity-robust inverse (v0.3) =================
const double SIGMA_DLS{0.05};     // damping starts when the smallest singular value of J is below this
const double LAMBDA_MAX{0.05};    // maximum damping factor

// smallest singular value of J (0 at a singularity)
double smallestSingularValue(const vpMatrix &J)
{
    vpMatrix U{J};
    vpColVector s;
    vpMatrix V;
    U.svd(s, V);
    return s.getMinValue();
}

// damped least squares: q_dot = J^T (J J^T + lambda^2 I)^-1 v
// lambda = 0 far from singularities (same as J^-1), grows smoothly near them
vpColVector dlsSolve(const vpMatrix &J, const vpColVector &v, const double sigmaMin, double &lambda)
{
    lambda = 0.;
    if(sigmaMin < SIGMA_DLS)
        lambda = LAMBDA_MAX * sqrt(1. - pow(sigmaMin / SIGMA_DLS, 2));
    vpMatrix I;
    I.eye(J.getRows());
    const vpMatrix A{J * J.t() + I * (lambda * lambda)};
    return J.t() * (A.pseudoInverse() * v);
}

// scale the whole joint command so that no joint exceeds its maximum velocity (direction kept)
double scaleToLimits(vpColVector &qd, const vpColVector &vMax)
{
    double k{1.};
    for(unsigned i = 0; i < qd.size(); ++i)
        k = std::max(k, std::abs(qd[i]) / vMax[i]);
    qd /= k;
    return k;
}

// ================= Mode 1: autonomous inspection (v0.4) =================
// Inspected component: the front face of the delicate equipment, plane x = X_FACE
// (the same plane as the keep-out wall). The camera optical axis is the tool z axis.
// The camera looks at the face from STANDOFF metres, tilted down by TILT, and follows a
// serpentine raster over the region [Y_MIN, Y_MAX] x [Z_MIN, Z_MAX] of the face.
const double X_FACE{1.50};                 // [m]   face plane
const double Y_MIN{-0.30}, Y_MAX{0.30};    // [m]   scanned region on the face
const double Z_MIN{1.00}, Z_MAX{1.30};
const double ROW_STEP{0.10};               // [m]   distance between raster rows
const double STANDOFF{0.25};               // [m]   camera-to-face distance along the optical axis
const double TILT{30. * M_PI / 180.};      // [rad] camera tilted down (keeps the wrist away from q5 = 0)

const double V_SCAN{0.05};                 // [m/s] scan speed along the raster
const double V_LIN_MAX{0.10};              // [m/s] linear speed limit in Mode 1
const double W_MAX{0.50};                  // [rad/s] angular speed limit in Mode 1
const double K_P{2.0};                     // [1/s] position gain
const double K_R{2.0};                     // [1/s] orientation gain
const double K_I{2.0};                     // [1/s^2] integral gain on the along-raster error
const double TRACK_TOL{0.02};              // [m]   the reference only advances if the error is below this
const double REACH_POS{0.005};             // [m]   approach finished below this position error
const double REACH_ROT{0.035};             // [rad] ... and below this orientation error (2 deg)

// simulated camera
const double PHOTO_PERIOD{0.5};            // [s]   one photo every 0.5 s during the scan
const double FOV_HALF{20. * M_PI / 180.};  // [rad] half field of view
const double MEAS_NOISE{0.003};            // [m]   localisation noise of one detection (1 sigma)

// hidden defects on the face, (y, z): only the simulated camera knows them
// (used as ground truth to evaluate the localisation error)
const vector<array<double, 2>> DEFECTS_TRUTH{{0.12, 1.17}, {-0.20, 1.04}};

enum class Phase{IDLE = 0, APPROACH = 1, SCAN = 2, DONE = 3};

// camera orientation: z (optical axis) = (cos t, 0, -sin t), x = (sin t, 0, cos t), y = z cross x = (0, -1, 0)
vpRotationMatrix scanRotation()
{
    vpRotationMatrix R;
    const double c{cos(TILT)}, s{sin(TILT)};
    R[0][0] = s;   R[0][1] = 0.;  R[0][2] = c;
    R[1][0] = 0.;  R[1][1] = -1.; R[1][2] = 0.;
    R[2][0] = c;   R[2][1] = 0.;  R[2][2] = -s;
    return R;
}

// camera positions at the ends of each raster row (serpentine)
vector<vpTranslationVector> scanWaypoints()
{
    vector<vpTranslationVector> W;
    const double c{cos(TILT)}, s{sin(TILT)};
    int row{0};
    for(double z = Z_MIN; z <= Z_MAX + 1e-6; z += ROW_STEP, ++row)
    {
        const double y0{row % 2 == 0 ? Y_MIN : Y_MAX};
        const double y1{row % 2 == 0 ? Y_MAX : Y_MIN};
        // camera = point on the face - STANDOFF * optical axis
        W.emplace_back(X_FACE - STANDOFF*c, y0, z + STANDOFF*s);
        W.emplace_back(X_FACE - STANDOFF*c, y1, z + STANDOFF*s);
    }
    return W;
}

// defect frame on the face: z axis = face normal (pointing out of the equipment, towards the robot)
vpHomogeneousMatrix defectFrame(const double y, const double z)
{
    vpHomogeneousMatrix M;
    M[0][0] = 0.; M[0][1] = 0.; M[0][2] = -1.; M[0][3] = X_FACE;
    M[1][0] = 0.; M[1][1] = 1.; M[1][2] = 0.;  M[1][3] = y;
    M[2][0] = 1.; M[2][1] = 0.; M[2][2] = 0.;  M[2][3] = z;
    return M;
}

// limit the norm of a 3-vector stored in v[i0..i0+2]
void saturate(vpColVector &v, const unsigned i0, const double vmax)
{
    const double n{sqrt(v[i0]*v[i0] + v[i0+1]*v[i0+1] + v[i0+2]*v[i0+2])};
    if(n > vmax)
        for(unsigned i = i0; i < i0+3; ++i)
            v[i] *= vmax / n;
}

double norm3(const vpTranslationVector &a)
{
    return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

struct Inspection
{
    Phase phase{Phase::IDLE};
    vector<vpTranslationVector> W{scanWaypoints()};
    vpRotationMatrix Rd{scanRotation()};
    size_t seg{0};             // current raster segment W[seg] -> W[seg+1]
    double s{0.};              // distance travelled on the current segment [m]
    vpTranslationVector ref;   // current reference position of the camera
    double tScan{0.}, tLastPhoto{-1e9}, duration{0.};
    int photos{0};
    double ePos{0.};           // position error [m]
    double iAlong{0.};         // integral of the along-raster error [m.s]
    vector<vector<array<double, 2>>> meas{vector<vector<array<double, 2>>>(DEFECTS_TRUTH.size())};
    mt19937 gen{42};           // fixed seed: reproducible runs
    normal_distribution<double> noise{0., MEAS_NOISE};
    bool reported{false};

    void reset()
    {
        *this = Inspection();
        phase = Phase::APPROACH;
        ref = W[0];
        cout << "[Mode 1] Inspection started: " << W.size()/2 << " rows, approach to first waypoint" << endl;
    }

    // computes the camera twist (fixed frame) for the current pose M
    vpColVector twist(const vpHomogeneousMatrix &M, const double t, const double dt)
    {
        vpColVector fv(6);
        const vpTranslationVector p{M.getTranslationVector()};
        const vpRotationMatrix R{M.getRotationMatrix()};

        // orientation error expressed in the fixed frame: R Rd^T = exp(theta u)
        const vpThetaUVector tu{R * Rd.t()};
        const double eRot{sqrt(tu[0]*tu[0] + tu[1]*tu[1] + tu[2]*tu[2])};

        if(phase == Phase::SCAN)
        {
            const vpTranslationVector segVec{W[seg+1] - W[seg]};
            const double L{norm3(segVec)};
            const vpTranslationVector u{segVec / L};

            // the reference moves at V_SCAN along the raster, only if the camera follows it
            const bool advance{norm3(ref - p) < TRACK_TOL};
            if(advance)
                s += V_SCAN * dt;
            if(s >= L)
            {
                s = 0.;
                if(++seg + 1 >= W.size())
                {
                    phase = Phase::DONE;
                    duration = t - tScan;
                }
            }
            if(phase == Phase::SCAN)
            {
                const vpTranslationVector u2{(W[seg+1] - W[seg]) / norm3(W[seg+1] - W[seg])};
                ref = W[seg] + u2 * s;
                // integral action along the raster: removes the steady lag/lead when the
                // realised joint velocity differs from the command (velocity gain mismatch)
                const double eAlong{(ref[0]-p[0])*u2[0] + (ref[1]-p[1])*u2[1] + (ref[2]-p[2])*u2[2]};
                const double iMax{0.5 * V_SCAN / K_I};
                iAlong = std::max(-iMax, std::min(iMax, iAlong + eAlong * dt));
                for(unsigned i = 0; i < 3; ++i)
                    fv[i] = (advance ? V_SCAN * u2[i] : 0.) + K_P * (ref[i] - p[i]) + K_I * iAlong * u2[i];
            }
        }

        if(phase == Phase::APPROACH)
        {
            ref = W[0];
            for(unsigned i = 0; i < 3; ++i)
                fv[i] = K_P * (ref[i] - p[i]);
            if(norm3(ref - p) < REACH_POS && eRot < REACH_ROT)
            {
                phase = Phase::SCAN;
                tScan = t;
                cout << "[Mode 1] First waypoint reached, scanning" << endl;
            }
        }

        if(phase == Phase::DONE)
        {
            ePos = 0.;
            return vpColVector(6);      // zero twist: hold position
        }

        for(unsigned i = 0; i < 3; ++i)
            fv[i+3] = -K_R * tu[i];
        saturate(fv, 0, V_LIN_MAX);
        saturate(fv, 3, W_MAX);
        ePos = norm3(ref - p);
        return fv;
    }

    // simulated camera: one photo every PHOTO_PERIOD during the scan
    void camera(const vpHomogeneousMatrix &M, const double t)
    {
        if(phase != Phase::SCAN || t - tLastPhoto < PHOTO_PERIOD)
            return;
        tLastPhoto = t;
        photos++;

        // intersection of the optical axis with the face
        const vpTranslationVector p{M.getTranslationVector()};
        const vpRotationMatrix R{M.getRotationMatrix()};
        const double zx{R[0][2]}, zy{R[1][2]}, zz{R[2][2]};
        if(zx < 0.1)
            return;     // not looking at the face
        const double dist{(X_FACE - p[0]) / zx};
        const double cy{p[1] + dist*zy}, cz{p[2] + dist*zz};
        const double radius{dist * tan(FOV_HALF)};

        for(size_t k = 0; k < DEFECTS_TRUTH.size(); ++k)
        {
            const double dy{DEFECTS_TRUTH[k][0] - cy}, dz{DEFECTS_TRUTH[k][1] - cz};
            if(dy*dy + dz*dz < radius*radius)
                meas[k].push_back({DEFECTS_TRUTH[k][0] + noise(gen), DEFECTS_TRUTH[k][1] + noise(gen)});
        }
    }

    // writes the inspection report (input of the virtual-fixture generator)
    template <class RobotPtr>
    void report(const string &stamp, RobotPtr &robot)
    {
        if(reported)
            return;
        reported = true;

        const string dir{string(getenv("HOME")) + "/cern_logs"};
        ostringstream js;
        js << fixed << setprecision(5);
        js << "{\n  \"version\": \"0.4\",\n  \"run\": \"" << stamp << "\",\n";
        js << "  \"component\": {\"name\": \"equipment front face\", \"plane_x\": " << X_FACE
           << ", \"normal\": [-1, 0, 0]},\n";
        js << "  \"scan\": {\"rows\": " << W.size()/2 << ", \"standoff_m\": " << STANDOFF
           << ", \"tilt_deg\": " << TILT*180./M_PI << ", \"region_y\": [" << Y_MIN << ", " << Y_MAX
           << "], \"region_z\": [" << Z_MIN << ", " << Z_MAX << "], \"duration_s\": " << duration
           << ", \"photos\": " << photos << "},\n";
        js << "  \"defects\": [";

        cout << "[Mode 1] Scan finished: " << duration << " s, " << photos << " photos" << endl;
        bool first{true};
        int id{0};
        for(size_t k = 0; k < meas.size(); ++k)
        {
            if(meas[k].empty())
                continue;
            double my{0}, mz{0};
            for(const auto &m: meas[k]) {my += m[0]; mz += m[1];}
            my /= meas[k].size();
            mz /= meas[k].size();
            double vy{0}, vz{0};
            for(const auto &m: meas[k]) {vy += pow(m[0]-my, 2); vz += pow(m[1]-mz, 2);}
            const double nm{static_cast<double>(meas[k].size())};
            // standard error of the mean position estimate
            const double sy{nm > 1 ? sqrt(vy/(nm-1)/nm) : MEAS_NOISE};
            const double sz{nm > 1 ? sqrt(vz/(nm-1)/nm) : MEAS_NOISE};
            const double err{sqrt(pow(my - DEFECTS_TRUTH[k][0], 2) + pow(mz - DEFECTS_TRUTH[k][1], 2))};

            ++id;
            js << (first ? "\n" : ",\n") << "    {\"id\": \"D" << id << "\", \"position\": [" << X_FACE << ", "
               << my << ", " << mz << "], \"normal\": [-1, 0, 0], \"std_m\": [" << sy << ", " << sz
               << "], \"detections\": " << meas[k].size() << ", \"sim_truth\": [" << X_FACE << ", "
               << DEFECTS_TRUTH[k][0] << ", " << DEFECTS_TRUTH[k][1] << "], \"sim_error_m\": " << err << "}";
            first = false;

            robot->displayFrame(defectFrame(my, mz), "defect_D" + to_string(id));
            cout << "[Mode 1] Defect D" << id << " at (" << X_FACE << ", " << my << ", " << mz
                 << "), " << meas[k].size() << " detections, error vs truth = "
                 << 1000*err << " mm" << endl;
        }
        js << "\n  ]\n}\n";

        for(const string &name: {"/inspection_" + stamp + ".json", string("/inspection_latest.json")})
        {
            ofstream f(dir + name);
            f << js.str();
        }
        cout << "[Mode 1] Report: " << dir << "/inspection_" << stamp << ".json" << endl;
    }
};

// ================= Option A: virtual fixtures generated from the inspection (v0.5) =================
// From each defect of the inspection report (position + face normal n), the generator builds:
//  - a target pose: tool point at D_WORK from the defect along n, tool z axis = -n (towards the face)
//  - an approach funnel around the axis (defect, n): radius R_TIP at the target, R_ENTRY at S_NEAR
//  - keep-out zone: the near zone (s < S_NEAR from the face) outside the funnel
//    (models the cluttered surroundings of the defect: neighbouring components, cables)
//  - a virtual depth stop at the target depth
//  - guidance in the approach region (s < S_GUIDE): lateral motion scaled by C_LAT, attraction to the
//    axis, and automatic alignment of the tool orientation with the target orientation
const double D_WORK{0.03};       // [m]   tool-to-face distance at the target
const double S_NEAR{0.15};       // [m]   near zone: the funnel is enforced closer than this to the face
const double S_GUIDE{0.25};      // [m]   guidance starts here and is fully active at S_NEAR
const double R_TIP{0.010};       // [m]   funnel radius at the target
const double R_ENTRY{0.060};     // [m]   funnel radius at the entry of the near zone
const double C_LAT{0.2};         // [-]   operator lateral velocity scaling inside the guidance region
const double K_G{1.5};           // [1/s] attraction towards the axis
const double V_G_MAX{0.05};      // [m/s] max attraction speed
const double K_RA{1.5};          // [1/s] orientation alignment gain
const double FIX_DI{0.02}, FIX_DS{0.001}, FIX_VA{0.05};      // funnel wall damper
const double DEPTH_DI{0.03}, DEPTH_DS{-0.001}, DEPTH_VA{0.05}; // depth stop damper

enum class Assist{NONE = 0, STATIC = 1, FIXTURES = 2};
const char *assistName(const Assist a)
{
    return a == Assist::NONE ? "none" : (a == Assist::STATIC ? "static" : "fixtures");
}

vpTranslationVector toT(const array<double, 3> &a)
{
    return vpTranslationVector(a[0], a[1], a[2]);
}

double dot3(const vpTranslationVector &a, const vpTranslationVector &b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

struct FixtureState
{
    double s{1e9};          // distance from the face along n [m]
    double rho{0.};         // distance to the approach axis [m]
    double w{0.};           // guidance weight (0 = free teleoperation, 1 = full guidance)
    double intrusion{0.};   // depth inside the keep-out zone (near zone outside the funnel) [m]
};

struct Fixture
{
    bool valid{false};
    string id;
    vpTranslationVector defect, n, target, entry;
    vpTranslationVector truth;      // simulated true defect position (evaluation only)
    bool hasTruth{false};
    vpRotationMatrix Rt;
    double tanA{(R_ENTRY - R_TIP) / (S_NEAR - D_WORK)};
    double cosA{1. / sqrt(1. + tanA*tanA)}, sinA{tanA / sqrt(1. + tanA*tanA)};

    // builds the fixture from a defect position and the face normal
    void build(const string &_id, const vpTranslationVector &_defect, const vpTranslationVector &_n)
    {
        id = _id;
        defect = _defect;
        n = _n / norm3(_n);
        target = defect + n * D_WORK;
        entry = defect + n * S_NEAR;
        // target orientation: z = -n, x = fixed z axis projected on the face plane, y = z cross x
        const vpTranslationVector z{-n[0], -n[1], -n[2]};
        vpTranslationVector x{0., 0., 1.};
        x = x - z * dot3(x, z);
        x = x / norm3(x);
        const vpTranslationVector y{vpTranslationVector::cross(z, x)};
        for(unsigned i = 0; i < 3; ++i)
        {
            Rt[i][0] = x[i];
            Rt[i][1] = y[i];
            Rt[i][2] = z[i];
        }
        valid = true;
    }

    double radius(const double s) const
    {
        return R_TIP + (std::max(s, D_WORK) - D_WORK) * tanA;
    }

    // geometry of the tool point p with respect to the fixture
    FixtureState state(const vpTranslationVector &p, vpTranslationVector &lat) const
    {
        FixtureState st;
        const vpTranslationVector r{p - defect};
        st.s = dot3(r, n);
        lat = r - n * st.s;
        st.rho = norm3(lat);
        st.w = std::min(1., std::max(0., (S_GUIDE - st.s) / (S_GUIDE - S_NEAR)));
        if(st.s < S_NEAR)
            st.intrusion = std::max(0., std::min(S_NEAR - st.s, (st.rho - radius(st.s)) * cosA));
        return st;
    }

    // guidance: modifies the commanded twist fv (fixed frame) inside the approach region
    FixtureState guide(vpColVector &fv, const vpHomogeneousMatrix &M) const
    {
        vpTranslationVector lat;
        const FixtureState st{state(M.getTranslationVector(), lat)};
        if(st.w <= 0.)
            return st;
        // linear: keep the axial part, scale the lateral part, attract towards the axis
        const vpTranslationVector v{fv[0], fv[1], fv[2]};
        const vpTranslationVector vAx{n * dot3(v, n)};
        const vpTranslationVector vLat{v - vAx};
        vpTranslationVector att{lat * (-K_G * st.w)};
        if(norm3(att) > V_G_MAX)
            att = att * (V_G_MAX / norm3(att));
        const vpTranslationVector vNew{vAx + vLat * (1. - st.w * (1. - C_LAT)) + att};
        // angular: blend the operator rotation with the automatic alignment to Rt
        const vpThetaUVector tu{M.getRotationMatrix() * Rt.t()};
        for(unsigned i = 0; i < 3; ++i)
        {
            fv[i] = vNew[i];
            fv[i+3] = (1. - st.w) * fv[i+3] - st.w * K_RA * tu[i];
        }
        saturate(fv, 3, W_MAX);
        return st;
    }

    // forbidden-region constraints (velocity dampers) for the tool point p
    vector<Constraint> constraints(const vpTranslationVector &p) const
    {
        vector<Constraint> cs;
        vpTranslationVector lat;
        const FixtureState st{state(p, lat)};
        const array<double, 3> na{n[0], n[1], n[2]};
        if(st.rho > R_ENTRY)
        {
            // outside the funnel entry: do not enter the near zone (and leave it along n if inside)
            cs.push_back({st.s - S_NEAR, na, FIX_DI, FIX_DS, FIX_VA});
        }
        else if(st.s < S_NEAR)
        {
            // depth stop: s >= D_WORK
            cs.push_back({st.s - D_WORK, na, DEPTH_DI, DEPTH_DS, DEPTH_VA});
            // funnel wall: rho <= radius(s), normal = sinA n - cosA e_rho
            if(st.rho > 1e-6)
            {
                const vpTranslationVector e{lat / st.rho};
                const array<double, 3> nw{sinA*n[0] - cosA*e[0], sinA*n[1] - cosA*e[1], sinA*n[2] - cosA*e[2]};
                cs.push_back({(radius(st.s) - st.rho) * cosA, nw, FIX_DI, FIX_DS, FIX_VA});
            }
        }
        return cs;
    }

    // reads the inspection report and builds the fixture for defect 'wanted' (first defect if empty)
    bool load(const string &file, const string &wanted)
    {
        valid = false;
        ifstream in(file);
        if(!in)
            return false;
        stringstream ss;
        ss << in.rdbuf();
        const string js{ss.str()};
        size_t pos{js.find("\"defects\"")};
        if(pos == string::npos)
            return false;
        const auto readVec = [&js](const size_t start, vpTranslationVector &v)
        {
            const char *c{js.c_str() + js.find('[', start) + 1};
            char *end;
            for(unsigned i = 0; i < 3; ++i)
            {
                v[i] = strtod(c, &end);
                c = end + 1;
            }
        };
        while((pos = js.find("\"id\": \"", pos)) != string::npos)
        {
            pos += 7;
            const string idRead{js.substr(pos, js.find('"', pos) - pos)};
            const size_t pp{js.find("\"position\"", pos)}, pn{js.find("\"normal\"", pos)};
            if(pp == string::npos || pn == string::npos)
                break;
            if(wanted.empty() || idRead == wanted)
            {
                vpTranslationVector d, nn;
                readVec(pp, d);
                readVec(pn, nn);
                build(idRead, d, nn);
                const size_t pt{js.find("\"sim_truth\"", pos)}, next{js.find("\"id\"", pos)};
                hasTruth = pt != string::npos && pt < next;
                if(hasTruth)
                    readVec(pt, truth);
                return true;
            }
        }
        return false;
    }

    // writes the generated fixture (documentation of the Option A output)
    void save(const string &file) const
    {
        ofstream f(file);
        f << fixed << setprecision(5);
        f << "{\n  \"version\": \"0.5\",\n  \"defect_id\": \"" << id << "\",\n";
        f << "  \"defect_position\": [" << defect[0] << ", " << defect[1] << ", " << defect[2] << "],\n";
        f << "  \"face_normal\": [" << n[0] << ", " << n[1] << ", " << n[2] << "],\n";
        f << "  \"target_position\": [" << target[0] << ", " << target[1] << ", " << target[2] << "],\n";
        f << "  \"target_tool_z\": [" << Rt[0][2] << ", " << Rt[1][2] << ", " << Rt[2][2] << "],\n";
        f << "  \"funnel\": {\"entry_position\": [" << entry[0] << ", " << entry[1] << ", " << entry[2]
          << "], \"radius_tip_m\": " << R_TIP << ", \"radius_entry_m\": " << R_ENTRY
          << ", \"near_zone_m\": " << S_NEAR << "},\n";
        f << "  \"depth_stop_m\": " << D_WORK << ",\n";
        f << "  \"guidance\": {\"start_m\": " << S_GUIDE << ", \"lateral_scaling\": " << C_LAT
          << ", \"axis_gain\": " << K_G << ", \"orientation_gain\": " << K_RA << "}\n}\n";
    }
};

// ================= E3: simulated operator (repeatable evaluation of the assistance) =================
// Model of a remote operator who sees the scene through cameras:
//  - perceived target = true target + bias (lateral SIGMA_LAT, depth SIGMA_DEPTH, orientation SIGMA_ROT)
//  - two stages: go to a pre-pose PRE in front of the perceived target, then insert along the axis
//  - proportional command + coloured noise, the operator stops when the perceived target is reached,
//    when the tool does not progress any more (blocked), or after OP_TIMEOUT
//  - with fixtures, the operator aims OVERSHOOT deeper and relies on the virtual depth stop
// The operator sees the real defect: perception and metrics use the simulated TRUE target, while the
// fixtures are built from the inspection ESTIMATE, so the E3 errors include the inspection error.
// The same bias is used for trial k in every condition (seed 1000 + k): paired comparison.
const double K_OP{1.0}, V_OP{0.06}, K_OPR{1.0}, W_OP{0.3};
const double OP_NOISE{0.008}, OP_TAU{0.3};          // [m/s], [s] coloured velocity noise
const double SIGMA_LAT{0.008}, SIGMA_DEPTH{0.015};  // [m]  perception bias
const double SIGMA_ROT{3. * M_PI / 180.};           // [rad]
const double PRE{0.20}, OVERSHOOT{0.03}, OP_TIMEOUT{45.};
const double J_GAIN{1.5}, J_VMAX{0.6};              // joint-space reset between trials

struct TrialMetrics
{
    double time{0}, ePos{0}, eLat{0}, eDepth{0}, eRot{0}, intrusion{0}, faceMin{1e9}, path{0};
    bool success{false};
};

struct SimOperator
{
    bool enabled{false};
    int nTrials{10}, trial{0};
    int stage{0};           // 0 joint-space reset to the start pose, 1 pre-approach, 2 insertion, 3 finished
    bool overshoot{false};
    vpColVector qStart;
    vpTranslationVector pHat, pCheck, pPrev;
    vpRotationMatrix RHat;
    double tOp{0}, tStage{0}, tCheck{0};
    array<double, 3> noise{0., 0., 0.};
    mt19937 gen;
    normal_distribution<double> N{0., 1.};
    TrialMetrics m;
    vector<TrialMetrics> results;
    ofstream out;
    string assist;

    // joint-space P control towards the start configuration (not part of the evaluation)
    vpColVector reset(const vpColVector &q, const double t, const Fixture &fx)
    {
        vpColVector qd{(qStart - q) * J_GAIN};
        double emax{0.};
        for(unsigned i = 0; i < qd.size(); ++i)
        {
            emax = std::max(emax, std::abs(qStart[i] - q[i]));
            qd[i] = std::max(-J_VMAX, std::min(J_VMAX, qd[i]));
        }
        if(emax < 0.003)
            startTrial(t, fx);
        return qd;
    }

    void startTrial(const double t, const Fixture &fx)
    {
        gen.seed(1000 + trial);
        // perception bias: lateral (tool x and y axes of the target frame) and depth (along n)
        vpTranslationVector ex, ey;
        for(unsigned i = 0; i < 3; ++i) {ex[i] = fx.Rt[i][0]; ey[i] = fx.Rt[i][1];}
        pHat = fx.target + ex * (SIGMA_LAT * N(gen)) + ey * (SIGMA_LAT * N(gen)) + fx.n * (SIGMA_DEPTH * N(gen));
        vpThetaUVector err;
        for(unsigned i = 0; i < 3; ++i)
            err[i] = SIGMA_ROT * N(gen);
        RHat = vpRotationMatrix(err) * fx.Rt;
        noise = {0., 0., 0.};
        m = TrialMetrics();
        stage = 1;
        tOp = tStage = t;
        cout << "[E3] trial " << trial + 1 << "/" << nTrials << " started" << endl;
    }

    // operator twist (fixed frame)
    vpColVector command(const vpHomogeneousMatrix &M, const double t, const double dt, const Fixture &fx)
    {
        vpColVector fv(6);
        if(stage != 1 && stage != 2)
            return fv;
        const vpTranslationVector p{M.getTranslationVector()};

        // metrics along the trial
        vpTranslationVector lat;
        const FixtureState st{fx.state(p, lat)};
        if(t > tOp)
            m.path += norm3(p - pPrev);
        pPrev = p;
        m.intrusion = std::max(m.intrusion, st.intrusion);
        m.faceMin = std::min(m.faceMin, st.s);

        // coloured noise
        const double a{exp(-std::max(dt, 1e-4) / OP_TAU)};
        for(auto &nz: noise)
            nz = a * nz + OP_NOISE * sqrt(1. - a*a) * N(gen);

        const vpTranslationVector goal{stage == 1 ? pHat + fx.n * PRE
                                                  : pHat - fx.n * (overshoot ? OVERSHOOT : 0.)};
        const vpTranslationVector e{goal - p};
        for(unsigned i = 0; i < 3; ++i)
            fv[i] = K_OP * e[i];
        saturate(fv, 0, V_OP);
        for(unsigned i = 0; i < 3; ++i)
            fv[i] += noise[i];
        const vpThetaUVector tu{M.getRotationMatrix() * RHat.t()};
        for(unsigned i = 0; i < 3; ++i)
            fv[i+3] = -K_OPR * tu[i];
        saturate(fv, 3, W_OP);
        const double eRot{sqrt(tu[0]*tu[0] + tu[1]*tu[1] + tu[2]*tu[2])};

        if(stage == 1 && norm3(e) < 0.01 && eRot < 0.05)
        {
            stage = 2;
            tStage = tCheck = t;
            pCheck = p;
        }
        else if(stage == 2)
        {
            bool stalled{false};
            if(t - tCheck > 1.5)
            {
                stalled = norm3(p - pCheck) < 0.001 && t - tStage > 2.;
                pCheck = p;
                tCheck = t;
            }
            if(norm3(pHat - p) < 0.002 || stalled)
                finish(M, t, fx);
        }
        if((stage == 1 || stage == 2) && t - tOp > OP_TIMEOUT)
            finish(M, t, fx);
        return stage == 3 || stage == 0 ? vpColVector(6) : fv;
    }

    void finish(const vpHomogeneousMatrix &M, const double t, const Fixture &fx)
    {
        const vpTranslationVector e{M.getTranslationVector() - fx.target};
        m.time = t - tOp;
        m.ePos = norm3(e);
        m.eDepth = dot3(e, fx.n);     // > 0: stopped short of the target, < 0: too deep
        m.eLat = norm3(e - fx.n * m.eDepth);
        const vpThetaUVector tu{M.getRotationMatrix() * fx.Rt.t()};
        m.eRot = sqrt(tu[0]*tu[0] + tu[1]*tu[1] + tu[2]*tu[2]) * 180. / M_PI;
        m.success = m.ePos < 0.005 && m.eRot < 3. && m.intrusion < 0.001 && m.faceMin > 0.;
        results.push_back(m);
        out << trial + 1 << "," << assist << "," << m.time << "," << 1000*m.ePos << "," << 1000*m.eLat << ","
            << 1000*m.eDepth << "," << m.eRot << "," << 1000*m.intrusion << "," << 1000*m.faceMin << ","
            << m.path << "," << m.success << endl;
        cout << fixed << setprecision(1) << "[E3] trial " << trial + 1 << ": time " << m.time << " s, error "
             << 1000*m.ePos << " mm (lat " << 1000*m.eLat << ", depth " << 1000*m.eDepth << "), rot "
             << m.eRot << " deg, intrusion " << 1000*m.intrusion << " mm, closest to face "
             << 1000*m.faceMin << " mm, " << (m.success ? "SUCCESS" : "fail") << endl;
        cout.unsetf(ios::floatfield);
        trial++;
        stage = trial < nTrials ? 0 : 3;
        if(stage == 3)
            summary();
    }

    void summary() const
    {
        const auto stats = [this](auto get, double &mean, double &sd)
        {
            mean = 0.; sd = 0.;
            for(const auto &r: results) mean += get(r);
            mean /= results.size();
            for(const auto &r: results) sd += pow(get(r) - mean, 2);
            sd = results.size() > 1 ? sqrt(sd / (results.size() - 1)) : 0.;
        };
        double mt, st, me, se, mr, sr, mi, si;
        stats([](const TrialMetrics &r){return r.time;}, mt, st);
        stats([](const TrialMetrics &r){return 1000*r.ePos;}, me, se);
        stats([](const TrialMetrics &r){return r.eRot;}, mr, sr);
        stats([](const TrialMetrics &r){return 1000*r.intrusion;}, mi, si);
        int ok{0}, intr{0};
        for(const auto &r: results) {ok += r.success; intr += r.intrusion > 0.001;}
        cout << fixed << setprecision(2) << "[E3] ===== " << assist << ": " << results.size() << " trials =====\n"
             << "[E3] time " << mt << " +- " << st << " s | position error " << me << " +- " << se
             << " mm | orientation error " << mr << " +- " << sr << " deg\n"
             << "[E3] intrusion " << mi << " +- " << si << " mm (" << intr << " trials) | success "
             << ok << "/" << results.size() << endl;
        cout.unsetf(ios::floatfield);
    }
};

// ================= Scene drawn in RViz (v0.6) =================
// Display only: the control above does not use these shapes. Frame: base_link (RViz fixed frame).
// In RViz: Add -> By topic -> /cern_scene -> MarkerArray
// Scenario: the KR16 is mounted on a vehicle parked in a tunnel, in front of a magnet-like piece of
// equipment whose flat end face is the inspected face (plane x = X_FACE).
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

const double EQ_RADIUS{0.5}, EQ_LENGTH{2.5}, EQ_Z{1.15};   // equipment cylinder, axis along x, end face at X_FACE
const double FLOOR_Z{-0.60};                                // tunnel floor (robot base on the vehicle at z = 0)
const double RED_HOLD{0.4};                                 // [s] a constraint stays red this long after acting

struct DefectMarker
{
    string id;
    vpTranslationVector est, truth;
    bool hasTruth{false};
};

// all defects of the inspection report (estimate and, in simulation, true position)
vector<DefectMarker> loadDefects(const string &file)
{
    vector<DefectMarker> out;
    ifstream in(file);
    if(!in)
        return out;
    stringstream ss;
    ss << in.rdbuf();
    const string js{ss.str()};
    const auto readVec = [&js](const size_t start, vpTranslationVector &v)
    {
        const char *c{js.c_str() + js.find('[', start) + 1};
        char *end;
        for(unsigned i = 0; i < 3; ++i)
        {
            v[i] = strtod(c, &end);
            c = end + 1;
        }
    };
    size_t pos{js.find("\"defects\"")};
    if(pos == string::npos)
        return out;
    while((pos = js.find("\"id\": \"", pos)) != string::npos)
    {
        pos += 7;
        DefectMarker d;
        d.id = js.substr(pos, js.find('"', pos) - pos);
        const size_t next{js.find("\"id\"", pos)};
        const size_t pp{js.find("\"position\"", pos)}, pt{js.find("\"sim_truth\"", pos)};
        if(pp == string::npos || pp > next)
            break;
        readVec(pp, d.est);
        d.hasTruth = pt != string::npos && pt < next;
        if(d.hasTruth)
            readVec(pt, d.truth);
        out.push_back(d);
    }
    return out;
}

struct Scene
{
    rclcpp::Node::SharedPtr node{std::make_shared<rclcpp::Node>("cern_scene")};
    rclcpp::Publisher<MarkerArray>::SharedPtr pub{node->create_publisher<MarkerArray>("cern_scene", 10)};
    vector<DefectMarker> defects;
    double tWallRed{-1e9}, tFloorRed{-1e9}, tFunnelRed{-1e9};

    static Marker make(const int id, const string &ns, const int type)
    {
        Marker m;
        m.header.frame_id = "base_link";
        m.ns = ns;
        m.id = id;
        m.type = type;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.;
        m.scale.x = m.scale.y = m.scale.z = 1.;
        m.color.a = 1.;
        return m;
    }
    static void place(Marker &m, const double x, const double y, const double z)
    {
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
    }
    static void size(Marker &m, const double x, const double y, const double z)
    {
        m.scale.x = x;
        m.scale.y = y;
        m.scale.z = z;
    }
    static void paint(Marker &m, const double r, const double g, const double b, const double a)
    {
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = a;
    }
    static geometry_msgs::msg::Point point(const double x, const double y, const double z)
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    // remembers which constraints are currently slowing the robot (drawn in red)
    void flag(const double t, const SafetyInfo &info, const vpTranslationVector &p)
    {
        if(info.active || info.stop)
        {
            if(p[2] - planes[0].d0 < D_INFLUENCE)
                tFloorRed = t;
            if(X_FACE - p[0] < D_INFLUENCE)
                tWallRed = t;
        }
        if(info.fixture)
            tFunnelRed = t;
    }

    void publish(const double t, const vpHomogeneousMatrix &M, const bool teleop, const bool inspect,
                 const Assist assist, const Fixture &fixture, const Inspection &insp, const SimOperator &op)
    {
        MarkerArray arr;
        const double s45{sqrt(0.5)};

        // ---- static environment: tunnel, equipment, vehicle ----
        Marker floor{make(0, "tunnel", Marker::CUBE)};
        place(floor, 1.5, 0., FLOOR_Z - 0.02);
        size(floor, 8., 5., 0.04);
        paint(floor, 0.55, 0.55, 0.55, 1.);
        arr.markers.push_back(floor);
        for(int k = 0; k < 2; ++k)
        {
            Marker wall{make(1 + k, "tunnel", Marker::CUBE)};
            place(wall, 1.5, k == 0 ? -2.5 : 2.5, FLOOR_Z + 1.5);
            size(wall, 8., 0.05, 3.);
            paint(wall, 0.85, 0.82, 0.72, 0.25);
            arr.markers.push_back(wall);
        }

        Marker eq{make(3, "equipment", Marker::CYLINDER)};      // cylinder axis z rotated onto x
        place(eq, X_FACE + EQ_LENGTH / 2., 0., EQ_Z);
        eq.pose.orientation.y = s45;
        eq.pose.orientation.w = s45;
        size(eq, 2. * EQ_RADIUS, 2. * EQ_RADIUS, EQ_LENGTH);
        paint(eq, 0.17, 0.42, 0.69, 1.);
        arr.markers.push_back(eq);
        const double hSup{EQ_Z - EQ_RADIUS - FLOOR_Z};
        for(int k = 0; k < 2; ++k)
        {
            Marker sup{make(4 + k, "equipment", Marker::CUBE)};
            place(sup, X_FACE + (k == 0 ? 0.5 : EQ_LENGTH - 0.4), 0., FLOOR_Z + hSup / 2.);
            size(sup, 0.3, 0.6, hSup);
            paint(sup, 0.35, 0.35, 0.35, 1.);
            arr.markers.push_back(sup);
        }

        Marker body{make(6, "vehicle", Marker::CUBE)};
        place(body, -0.15, 0., -0.25);
        size(body, 1.1, 0.8, 0.5);
        paint(body, 0.95, 0.75, 0.10, 1.);
        arr.markers.push_back(body);
        int id{7};
        for(const double wx: {-0.50, 0.20})
            for(const double wy: {-0.44, 0.44})
            {
                Marker wheel{make(id++, "vehicle", Marker::CYLINDER)};   // axis along y
                place(wheel, wx, wy, FLOOR_Z + 0.15);
                wheel.pose.orientation.x = s45;
                wheel.pose.orientation.w = s45;
                size(wheel, 0.30, 0.30, 0.08);
                paint(wheel, 0.08, 0.08, 0.08, 1.);
                arr.markers.push_back(wheel);
            }

        // ---- keep-out planes (red while the damper slows the robot) ----
        const bool wallRed{t - tWallRed < RED_HOLD}, floorRed{t - tFloorRed < RED_HOLD};
        Marker wallZone{make(11, "keep-out", Marker::CUBE)};
        place(wallZone, X_FACE - 0.005, 0., EQ_Z + 0.1);
        size(wallZone, 0.01, 2.0, 1.3);
        paint(wallZone, wallRed ? 1. : 1., wallRed ? 0.1 : 0.85, wallRed ? 0.1 : 0.2,
              assist == Assist::NONE ? 0. : (wallRed ? 0.45 : 0.10));
        arr.markers.push_back(wallZone);
        Marker floorZone{make(12, "keep-out", Marker::CUBE)};
        place(floorZone, 1.05, 0., planes[0].d0);
        size(floorZone, 0.9, 2.0, 0.01);
        paint(floorZone, 1., floorRed ? 0.1 : 0.85, floorRed ? 0.1 : 0.2,
              assist == Assist::NONE ? 0. : (floorRed ? 0.45 : 0.10));
        arr.markers.push_back(floorZone);

        // ---- inspection: planned raster on the face, live camera footprint ----
        Marker raster{make(13, "inspection", Marker::LINE_STRIP)};
        raster.scale.x = 0.004;
        paint(raster, 1., 1., 1., 0.7);
        int row{0};
        for(double z = Z_MIN; z <= Z_MAX + 1e-6; z += ROW_STEP, ++row)
        {
            raster.points.push_back(point(X_FACE - 0.003, row % 2 == 0 ? Y_MIN : Y_MAX, z));
            raster.points.push_back(point(X_FACE - 0.003, row % 2 == 0 ? Y_MAX : Y_MIN, z));
        }
        arr.markers.push_back(raster);

        Marker foot{make(14, "inspection", Marker::LINE_STRIP)};
        foot.scale.x = 0.005;
        const vpTranslationVector p{M.getTranslationVector()};
        const double zx{M[0][2]};
        const bool showFoot{inspect && insp.phase == Phase::SCAN && zx > 0.1};
        const double dist{showFoot ? (X_FACE - p[0]) / zx : 0.1};
        const double cy{p[1] + dist * M[1][2]}, cz{p[2] + dist * M[2][2]}, rad{dist * tan(FOV_HALF)};
        for(int k = 0; k <= 36; ++k)
            foot.points.push_back(point(X_FACE - 0.005, cy + rad * cos(k * M_PI / 18.), cz + rad * sin(k * M_PI / 18.)));
        paint(foot, 0.1, 0.9, 1., showFoot ? 0.9 : 0.);
        arr.markers.push_back(foot);

        // ---- defects from the inspection report: true (red) and estimated (orange) ----
        for(size_t k = 0; k < defects.size(); ++k)
        {
            if(defects[k].hasTruth)
            {
                Marker tr{make(100 + static_cast<int>(k), "defects", Marker::SPHERE)};
                place(tr, X_FACE - 0.004, defects[k].truth[1], defects[k].truth[2]);
                size(tr, 0.02, 0.02, 0.02);
                paint(tr, 0.9, 0.05, 0.05, 1.);
                arr.markers.push_back(tr);
            }
            Marker es{make(200 + static_cast<int>(k), "defects", Marker::SPHERE)};
            place(es, X_FACE - 0.006, defects[k].est[1], defects[k].est[2]);
            size(es, 0.035, 0.035, 0.035);
            paint(es, 1., 0.55, 0., 0.45);
            arr.markers.push_back(es);
        }

        // ---- generated fixture: funnel (green, red while it constrains the motion) and target ----
        if(fixture.valid)
        {
            const bool funnelRed{t - tFunnelRed < RED_HOLD};
            Marker funnel{make(15, "fixture", Marker::TRIANGLE_LIST)};
            vpTranslationVector ex, ey;
            for(unsigned i = 0; i < 3; ++i) {ex[i] = fixture.Rt[i][0]; ey[i] = fixture.Rt[i][1];}
            const auto ring = [&](const double s, const double a)
            {
                const vpTranslationVector q{fixture.defect + fixture.n * s
                                            + (ex * cos(a) + ey * sin(a)) * fixture.radius(s)};
                return point(q[0], q[1], q[2]);
            };
            const int nSeg{32};
            for(int k = 0; k < nSeg; ++k)
            {
                const double a0{2. * M_PI * k / nSeg}, a1{2. * M_PI * (k + 1) / nSeg};
                const auto p00{ring(D_WORK, a0)}, p01{ring(D_WORK, a1)}, p10{ring(S_NEAR, a0)}, p11{ring(S_NEAR, a1)};
                funnel.points.insert(funnel.points.end(), {p00, p10, p11, p00, p11, p01});
            }
            paint(funnel, funnelRed ? 1. : 0.18, funnelRed ? 0.15 : 0.80, funnelRed ? 0.15 : 0.44,
                  assist == Assist::FIXTURES ? (funnelRed ? 0.55 : 0.35) : 0.12);
            arr.markers.push_back(funnel);

            Marker tgt{make(16, "fixture", Marker::SPHERE)};
            place(tgt, fixture.target[0], fixture.target[1], fixture.target[2]);
            size(tgt, 0.015, 0.015, 0.015);
            paint(tgt, 0.1, 0.9, 0.3, 1.);
            arr.markers.push_back(tgt);
        }

        // ---- status text ----
        Marker txt{make(17, "status", Marker::TEXT_VIEW_FACING)};
        place(txt, 0.6, 0., 2.1);
        txt.scale.z = 0.09;
        paint(txt, 1., 1., 1., 1.);
        ostringstream os;
        if(inspect)
            os << "Mode 1 - autonomous inspection ("
               << (insp.phase == Phase::APPROACH ? "approach" : insp.phase == Phase::SCAN ? "scanning" : "done")
               << ", " << insp.photos << " photos)";
        else if(teleop)
        {
            os << "Mode 2 - teleoperation, assistance: " << assistName(assist);
            if(op.enabled)
                os << " (simulated operator, trial " << std::min(op.trial + 1, op.nTrials) << "/" << op.nTrials << ")";
        }
        else
            os << "Idle - select a velocity mode in the GUI";
        txt.text = os.str();
        arr.markers.push_back(txt);

        pub->publish(arr);
    }
};

// ============================ Logging ============================
// time stamp used in the log names
string timeStamp()
{
    char stamp[32];
    const time_t now{time(nullptr)};
    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", localtime(&now));
    return stamp;
}

string envString(const char *name, const string &def = "")
{
    const char *e{getenv(name)};
    return e ? string(e) : def;
}

int main(int argc, char ** argv)
{
    const auto robot{initRobot(argc, argv, 100)};
    const unsigned n{robot->getDofs()};

    // assistance level: ASSIST=none | static | fixtures (default)   (SAFETY=0 is the same as ASSIST=none)
    Assist assist{Assist::FIXTURES};
    const string assistEnv{envString("ASSIST", "fixtures")};
    if(assistEnv == "none" || envString("SAFETY") == "0")
        assist = Assist::NONE;
    else if(assistEnv == "static")
        assist = Assist::STATIC;
    cout << "Assistance: " << assistName(assist)
         << (assist == Assist::NONE ? " (no safety layer)" :
             assist == Assist::STATIC ? " (static keep-out planes)" :
                                        " (keep-out planes + virtual fixtures from the inspection)") << endl;

    // damped least squares + joint-velocity limit ON by default, OFF with DLS=0
    const bool dlsOn{envString("DLS") != "0"};
    cout << "Singularity-robust inverse (DLS + velocity limit): " << (dlsOn ? "ON" : "OFF") << endl;
    const vpColVector vMax{robot->vMax()};

    vpColVector q(n);           // joint position
    vpColVector vCommand(n);    // joint velocity command
    vpColVector v(6);           // operator twist from the GUI (end-effector frame)
    vpColVector fvOp(6);        // commanded twist in the fixed frame (operator or autonomous, before assistance)
    vpColVector fv(6);          // twist sent to the robot (after assistance)
    vpHomogeneousMatrix M;      // current end-effector pose
    vpPoseVector p;             // current pose as (t, theta u)
    Inspection insp;            // Mode 1 state

    const string dir{string(getenv("HOME")) + "/cern_logs"};
    filesystem::create_directories(dir);
    const string stamp{timeStamp()};

    // ---- Option A: virtual fixtures from the latest inspection report ----
    const string target{envString("TARGET")};     // defect id, e.g. TARGET=D2 (default: first defect)
    Scene scene;        // v0.6: scene drawn in RViz
    Fixture fixture;    // built from the inspection estimate: used for the assistance
    Fixture evalFix;    // built from the simulated true defect: used by the simulated operator and the E3 metrics
    const auto loadFixture = [&]()
    {
        scene.defects = loadDefects(dir + "/inspection_latest.json");
        if(fixture.load(dir + "/inspection_latest.json", target))
        {
            evalFix = fixture;
            if(fixture.hasTruth)
                evalFix.build(fixture.id, fixture.truth, fixture.n);
            fixture.save(dir + "/fixtures_latest.json");
            cout << "[Fixtures] generated for defect " << fixture.id << ": target (" << fixture.target[0] << ", "
                 << fixture.target[1] << ", " << fixture.target[2] << "), funnel " << 1000*R_ENTRY << " -> "
                 << 1000*R_TIP << " mm, depth stop " << 1000*D_WORK << " mm from the face" << endl;
        }
        else
            cout << "[Fixtures] no inspection report (run Mode 1 first): fixtures disabled" << endl;
    };
    loadFixture();

    // ---- E3: simulated operator (OPERATOR=sim TRIALS=10) ----
    SimOperator op;
    op.enabled = envString("OPERATOR") == "sim";
    if(op.enabled)
    {
        if(!fixture.valid)
        {
            cout << "[E3] the simulated operator needs a target: run Mode 1 first" << endl;
            return 1;
        }
        op.nTrials = std::max(1, atoi(envString("TRIALS", "10").c_str()));
        op.overshoot = assist == Assist::FIXTURES;
        op.assist = assistName(assist);
        // start pose: 0.35 m in front of the target, offset sideways, camera orientation of the scan;
        // start configuration from the inverse geometry (elbow-down branch, same for every trial)
        const vpTranslationVector pStart{fixture.target + fixture.n * 0.35 + vpTranslationVector(0., 0.10, 0.06)};
        vpColVector qHint(n);
        qHint[1] = -0.2; qHint[2] = -1.2; qHint[4] = 1.6; qHint[5] = 3.1;
        op.qStart = robot->inverseGeometry(vpHomogeneousMatrix(pStart, scanRotation()), qHint);
        op.out.open(dir + "/e3_" + op.assist + "_" + stamp + ".csv");
        op.out << "trial,assist,time_s,e_pos_mm,e_lat_mm,e_depth_mm,e_rot_deg,intrusion_mm,face_min_mm,path_m,success"
               << endl;
        op.out << fixed << setprecision(4);
        cout << "[E3] simulated operator, " << op.nTrials << " trials, select 'Manual Cartesian control' in the GUI"
             << endl;
    }

    // ---- CSV log ----
    const string path{dir + "/run_" + stamp + ".csv"};
    ofstream log(path);
    log << "t,mode";
    for(unsigned i = 0; i < n; ++i) log << ",q" << i+1;
    log << ",x,y,z,tux,tuy,tuz";
    for(unsigned i = 0; i < 6; ++i) log << ",v" << i+1;
    log << ",vop_x,vop_y,vop_z,vsafe_x,vsafe_y,vsafe_z";
    for(unsigned i = 0; i < n; ++i) log << ",qd" << i+1;
    log << ",dmin,damper,stop,sigma_min,lambda,vel_scale";
    log << ",phase,photos,ref_x,ref_y,ref_z,e_pos";
    log << ",assist,trial,op_stage,s_axis,rho,intrusion,w_guide,fixture_active" << endl;
    log << fixed << setprecision(5);
    cout << "Logging to " << path << endl;

    double tStart{-1.}, tPrev{0.};
    bool prevStop{false};
    int cycle{0};

    while(robot->ok())
    {
        const auto t{robot->time()};
        if(tStart < 0) tStart = tPrev = t;
        const double dt{t - tPrev};
        tPrev = t;
        cycle++;

        q = robot->jointPosition();
        M = robot->fMe(q);
        p.buildFrom(M);

        v = 0;
        fvOp = 0;
        fv = 0;
        vCommand = 0;
        // with the simulated operator, both velocity modes of the GUI run the E3 trials (no scan)
        const bool teleop{robot->mode() == ControlMode::VELOCITY_MANUAL ||
                          (op.enabled && robot->mode() == ControlMode::VELOCITY_P2P)};
        const bool inspect{robot->mode() == ControlMode::VELOCITY_P2P && !op.enabled};
        if(op.enabled && !teleop && cycle % 300 == 1)
            cout << "[E3] waiting: select 'Manual Cartesian control' in the GUI" << endl;
        bool jointReset{false};
        FixtureState fst;

        if(teleop)
        {
            // ---- Mode 2: teleoperation (GUI operator or simulated operator) ----
            if(op.enabled)
            {
                if(op.stage == 0)
                {
                    vCommand = op.reset(q, t, evalFix);
                    jointReset = true;
                }
                else
                    fvOp = op.command(M, t, dt, evalFix);
            }
            else
            {
                // operator twist from the GUI sliders, in the end-effector frame -> fixed frame
                v = robot->guiVelocityScrew();
                vpMatrix R(6, 6);
                putAt(R, M.getRotationMatrix(), 0, 0);
                putAt(R, M.getRotationMatrix(), 3, 3);
                fvOp = R * v;
            }
        }

        if(inspect)
        {
            // ---- Mode 1: autonomous inspection ----
            if(insp.phase == Phase::IDLE)
                insp.reset();
            fvOp = insp.twist(M, t, dt);
            insp.camera(M, t);
            if(insp.phase == Phase::DONE && !insp.reported)
            {
                insp.report(stamp, robot);
                loadFixture();      // Option A: the new inspection immediately generates the fixtures
            }
            robot->displayFrame(vpHomogeneousMatrix(insp.ref, insp.Rd), "scan_ref");
        }
        else if(insp.phase != Phase::IDLE)
        {
            // leaving Mode 1: the next time Mode 1 is selected, a new scan starts
            if(insp.phase != Phase::DONE)
                cout << "[Mode 1] Inspection aborted" << endl;
            insp.phase = Phase::IDLE;
        }
        fv = fvOp;

        // ---- assistance: guidance (fixtures), then keep-out planes + fixture constraints ----
        const bool useFixture{teleop && !jointReset && assist == Assist::FIXTURES && fixture.valid};
        if(useFixture)
            fst = fixture.guide(fv, M);
        else if(fixture.valid)
        {
            vpTranslationVector lat;
            fst = fixture.state(M.getTranslationVector(), lat);
        }
        const vector<Constraint> extra{useFixture ? fixture.constraints(M.getTranslationVector())
                                                  : vector<Constraint>{}};
        const SafetyInfo info{safetyLayer(fv, M.getTranslationVector(),
                                          (teleop || inspect) && !jointReset && assist != Assist::NONE, extra)};
        if(info.stop && !prevStop)
            cout << "ANTI-COLLISION STOP (d = " << info.dmin << " m)" << endl;
        prevStop = info.stop;
        scene.flag(t, info, M.getTranslationVector());

        double sigmaMin{0.}, lambda{0.}, velScale{1.};
        if((teleop || inspect) && !jointReset)
        {
            const vpMatrix J{robot->fJe(q)};
            sigmaMin = smallestSingularValue(J);
            if(dlsOn)
            {
                // singularity-robust: damped least squares, then joint-velocity limit
                vCommand = dlsSolve(J, fv, sigmaMin, lambda);
                // near a singularity the realised twist J q_dot differs from fv: if it violates a
                // constraint, correct the command (a few iterations)
                if(lambda > 0. && assist != Assist::NONE && !info.stop)
                {
                    for(int it = 0; it < 3; ++it)
                    {
                        const vpColVector vr{J * vCommand};
                        vpColVector vc{vr};
                        applyAll(vc, M.getTranslationVector(), extra);
                        const double corr{sqrt(pow(vc[0]-vr[0], 2) + pow(vc[1]-vr[1], 2) + pow(vc[2]-vr[2], 2))};
                        if(corr < 1e-5)
                            break;
                        for(unsigned i = 0; i < 3; ++i)
                            fv[i] += vc[i] - vr[i];
                        vCommand = dlsSolve(J, fv, sigmaMin, lambda);
                    }
                }
                velScale = scaleToLimits(vCommand, vMax);
            }
            else
            {
                // plain pseudo-inverse (as in MANIP Exercise 2): q_dot = J+ fv
                vCommand = J.pseudoInverse() * fv;
            }
        }
        if(teleop || inspect)
            robot->setJointVelocity(vCommand);

        if(fixture.valid && cycle % 20 == 0)
        {
            robot->displayFrame(vpHomogeneousMatrix(fixture.target, fixture.Rt), "fixture_target");
            robot->displayFrame(vpHomogeneousMatrix(fixture.entry, fixture.Rt), "fixture_entry");
        }

        if(cycle % 8 == 0)
            scene.publish(t, M, teleop, inspect, assist, fixture, insp, op);

        // ---- log one line per cycle ----
        log << t - tStart << "," << static_cast<int>(robot->mode());
        for(unsigned i = 0; i < n; ++i) log << "," << q[i];
        for(unsigned i = 0; i < 6; ++i) log << "," << p[i];
        for(unsigned i = 0; i < 6; ++i) log << "," << v[i];
        for(unsigned i = 0; i < 3; ++i) log << "," << fvOp[i];
        for(unsigned i = 0; i < 3; ++i) log << "," << fv[i];
        for(unsigned i = 0; i < n; ++i) log << "," << vCommand[i];
        log << "," << info.dmin << "," << info.active << "," << info.stop;
        log << "," << sigmaMin << "," << lambda << "," << velScale;
        log << "," << static_cast<int>(insp.phase) << "," << insp.photos;
        log << "," << insp.ref[0] << "," << insp.ref[1] << "," << insp.ref[2] << "," << insp.ePos;
        log << "," << static_cast<int>(assist) << "," << (op.enabled ? op.trial + 1 : 0) << ","
            << (op.enabled ? op.stage : -1) << "," << std::min(fst.s, 9.) << "," << fst.rho << ","
            << fst.intrusion << "," << fst.w << "," << info.fixture << endl;
    }
}
