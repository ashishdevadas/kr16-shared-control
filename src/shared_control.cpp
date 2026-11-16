// Shared-control framework for safe robotic inspection and intervention
// in CERN-like environments (KUKA KR16, ecn_manip simulator)
//
// Mode 2 (assisted teleoperation)  -> GUI "Manual Cartesian control"   (ControlMode::VELOCITY_MANUAL)
// Mode 1 (autonomous inspection)   -> GUI "Velocity point-to-point"     (ControlMode::VELOCITY_P2P)
//
// Version 0.4: + Mode 1 autonomous inspection scan of the equipment front face
//              (raster scan, simulated camera, defect localisation, inspection report JSON)
//              0.4.1: camera tilt 30 deg (wrist margin), integral action along the raster
//              (robust to the simulator's velocity gain), tracking tolerance 20 mm
// Version 0.3: teleoperation + safety layer (keep-out planes, velocity damper,
//              anti-collision stop) + singularity-robust inverse (damped least squares)
//              + joint-velocity limit + CSV logging
// For comparisons:
//   safety layer OFF:          SAFETY=0 ros2 run cern_shared_control shared_control
//   plain pseudo-inverse:      DLS=0    ros2 run cern_shared_control shared_control

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

struct SafetyInfo
{
    double dmin{1e9};     // smallest distance to any plane [m]
    bool active{false};   // the damper modified the command
    bool stop{false};     // anti-collision stop triggered
};

// computes the distances and, if apply = true, modifies the linear part of fv (fixed frame)
SafetyInfo safetyLayer(vpColVector &fv, const vpTranslationVector &pt, const bool apply)
{
    SafetyInfo info;
    for(const auto &pl: planes)
    {
        const double d{pl.n[0]*pt[0] + pl.n[1]*pt[1] + pl.n[2]*pt[2] - pl.d0};
        info.dmin = std::min(info.dmin, d);
        if(!apply)
            continue;

        // normal component of the commanded linear velocity (positive = moving away)
        const double vn{pl.n[0]*fv[0] + pl.n[1]*fv[1] + pl.n[2]*fv[2]};

        // 1. anti-collision stop: too close and still approaching
        if(d < D_STOP && vn < 0.)
        {
            info.stop = true;
            continue;
        }

        // 2. velocity damper: limit the approach speed near the plane
        if(d < D_INFLUENCE)
        {
            const double vnMin{-V_APPROACH * (d - D_SAFE) / (D_INFLUENCE - D_SAFE)};
            if(vn < vnMin)
            {
                for(unsigned i = 0; i < 3; ++i)
                    fv[i] += (vnMin - vn) * pl.n[i];
                info.active = true;
            }
        }
    }
    if(info.stop)
        fv = 0;
    return info;
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

// ============================ Logging ============================
// time stamp used in the log names
string timeStamp()
{
    char stamp[32];
    const time_t now{time(nullptr)};
    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", localtime(&now));
    return stamp;
}

int main(int argc, char ** argv)
{
    const auto robot{initRobot(argc, argv, 100)};
    const unsigned n{robot->getDofs()};

    // safety layer ON by default, OFF with SAFETY=0
    const char *env{getenv("SAFETY")};
    const bool safetyOn{!(env && string(env) == "0")};
    cout << "Safety layer: " << (safetyOn ? "ON" : "OFF") << endl;

    // damped least squares + joint-velocity limit ON by default, OFF with DLS=0
    const char *envDls{getenv("DLS")};
    const bool dlsOn{!(envDls && string(envDls) == "0")};
    cout << "Singularity-robust inverse (DLS + velocity limit): " << (dlsOn ? "ON" : "OFF") << endl;
    const vpColVector vMax{robot->vMax()};

    vpColVector q(n);           // joint position
    vpColVector vCommand(n);    // joint velocity command
    vpColVector v(6);           // operator twist (end-effector frame)
    vpColVector fvOp(6);        // commanded twist in the fixed frame (operator or autonomous, before safety)
    vpColVector fv(6);          // twist sent to the robot (after safety)
    vpHomogeneousMatrix M;      // current end-effector pose
    vpPoseVector p;             // current pose as (t, theta u)
    Inspection insp;            // Mode 1 state

    // ---- CSV log ----
    const string dir{string(getenv("HOME")) + "/cern_logs"};
    filesystem::create_directories(dir);
    const string stamp{timeStamp()};
    const string path{dir + "/run_" + stamp + ".csv"};
    ofstream log(path);
    log << "t,mode";
    for(unsigned i = 0; i < n; ++i) log << ",q" << i+1;
    log << ",x,y,z,tux,tuy,tuz";
    for(unsigned i = 0; i < 6; ++i) log << ",v" << i+1;
    log << ",vop_x,vop_y,vop_z,vsafe_x,vsafe_y,vsafe_z";
    for(unsigned i = 0; i < n; ++i) log << ",qd" << i+1;
    log << ",dmin,damper,stop,sigma_min,lambda,vel_scale";
    log << ",phase,photos,ref_x,ref_y,ref_z,e_pos" << endl;
    log << fixed << setprecision(5);
    cout << "Logging to " << path << endl;

    double tStart{-1.}, tPrev{0.};
    bool prevStop{false};

    while(robot->ok())
    {
        const auto t{robot->time()};
        if(tStart < 0) tStart = tPrev = t;
        const double dt{t - tPrev};
        tPrev = t;

        q = robot->jointPosition();
        M = robot->fMe(q);
        p.buildFrom(M);

        v = 0;
        fvOp = 0;
        fv = 0;
        vCommand = 0;
        const bool teleop{robot->mode() == ControlMode::VELOCITY_MANUAL};
        const bool inspect{robot->mode() == ControlMode::VELOCITY_P2P};

        if(teleop)
        {
            // ---- Mode 2: teleoperation ----
            // operator twist from the GUI sliders, in the end-effector frame
            v = robot->guiVelocityScrew();

            // express it in the fixed frame: fv = [fRe 0; 0 fRe] ev
            vpMatrix R(6, 6);
            putAt(R, M.getRotationMatrix(), 0, 0);
            putAt(R, M.getRotationMatrix(), 3, 3);
            fvOp = R * v;
        }

        if(inspect)
        {
            // ---- Mode 1: autonomous inspection ----
            if(insp.phase == Phase::IDLE)
                insp.reset();
            fvOp = insp.twist(M, t, dt);
            insp.camera(M, t);
            if(insp.phase == Phase::DONE)
                insp.report(stamp, robot);
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

        // ---- safety layer (distances always computed, command modified in Mode 1 and Mode 2) ----
        const SafetyInfo info{safetyLayer(fv, M.getTranslationVector(), (teleop || inspect) && safetyOn)};
        if(info.stop && !prevStop)
            cout << "ANTI-COLLISION STOP (d = " << info.dmin << " m)" << endl;
        prevStop = info.stop;

        double sigmaMin{0.}, lambda{0.}, velScale{1.};
        if(teleop || inspect)
        {
            const vpMatrix J{robot->fJe(q)};
            sigmaMin = smallestSingularValue(J);
            if(dlsOn)
            {
                // singularity-robust: damped least squares, then joint-velocity limit
                vCommand = dlsSolve(J, fv, sigmaMin, lambda);
                velScale = scaleToLimits(vCommand, vMax);
            }
            else
            {
                // plain pseudo-inverse (as in MANIP Exercise 2): q_dot = J+ fv
                vCommand = J.pseudoInverse() * fv;
            }
            robot->setJointVelocity(vCommand);
        }

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
        log << "," << insp.ref[0] << "," << insp.ref[1] << "," << insp.ref[2] << "," << insp.ePos << endl;
    }
}
