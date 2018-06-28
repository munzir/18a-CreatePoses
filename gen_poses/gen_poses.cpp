// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To generate a set of poses (balanced/collision free) in which the xCOM is within a certain
// tolerance level

// Input and Output file format (DART Style)
// axis-angle1, axis-angle2, axis-angle3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6
// Use alphabet scheme to identify them as variables in order

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <nlopt.hpp>
#include <iostream>
#include <fstream>

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// Defines
#define MAXBUFSIZE ((int) 1e6)

// Global Variables
double pi = M_PI;
double bendLimit = 2.0944;

// // Limits of each joint to prevent collision with the previous joint
// // Below values obtained from hardware init script on Krang: https://github.gatech.edu/WholeBodyControlAttempt1/Krang-Software/blob/master/project/krang/util/scripts/initd/krang
// Format for below vectors
// qWaist, qTorso, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
// negative value (-) is clockwise from parent axis

double lowerLimit[] = {0, -1.57, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi};
double upperLimit[] = {2.88, 1.57, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi};


// Structs and functions for balancing Krang Model (Full)
struct comOptParams {
    SkeletonPtr robot;
    Eigen::Matrix<double, 25, 1> qInit;
};

double comOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
    comOptParams* optParams = reinterpret_cast<comOptParams *>(my_func_data);
    Eigen::Matrix<double, 25, 1> q(x.data());

    if (!grad.empty()) {
        Eigen::Matrix<double, 25, 1> mGrad = q-optParams->qInit;
        Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
    }
    return (0.5*pow((q-optParams->qInit).norm(), 2));
}

double comConstraint(const std::vector<double> &x, std::vector<double> &grad, void *com_const_data) {
    comOptParams* optParams = reinterpret_cast<comOptParams *>(com_const_data);
    Eigen::Matrix<double, 25, 1> q(x.data());
    optParams->robot->setPositions(q);
    return (pow(optParams->robot->getCOM()(0)-optParams->robot->getPosition(3), 2) \
        + pow(optParams->robot->getCOM()(1)-optParams->robot->getPosition(4), 2));
}

double wheelAxisConstraint(const std::vector<double> &x, std::vector<double> &grad, void *wheelAxis_const_data) {
    comOptParams* optParams = reinterpret_cast<comOptParams *>(wheelAxis_const_data);
    Eigen::Matrix<double, 25, 1> q(x.data());
    optParams->robot->setPositions(q);
    return optParams->robot->getBodyNode(0)->getTransform().matrix()(2,0);
}

double headingConstraint(const std::vector<double> &x, std::vector<double> &grad, void *heading_const_data) {
    comOptParams* optParams = reinterpret_cast<comOptParams *>(heading_const_data);
    Eigen::Matrix<double, 25, 1> q(x.data());
    optParams->robot->setPositions(q);
    Eigen::Matrix<double, 4, 4> Tf = optParams->robot->getBodyNode(0)->getTransform().matrix();
    double heading = atan2(Tf(0,0), -Tf(1,0));
    optParams->robot->setPositions(optParams->qInit);
    Tf = optParams->robot->getBodyNode(0)->getTransform().matrix();
    double headingInit = atan2(Tf(0,0), -Tf(1,0));
    return heading-headingInit;
}

// Structs and functions for balancing Fixed Wheel Krang Model (Simple)
struct inequalityOptParams {
    Eigen::MatrixXd P;
    Eigen::VectorXd b;
};

double comSimpleOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
    comOptParams* optParams = reinterpret_cast<comOptParams *>(my_func_data);
    Eigen::Matrix<double, 1, 1> q1(x.data());
    Eigen::Matrix<double, 18, 1> q;
    q << q1, optParams->qInit.tail(17);

    optParams->robot->setPositions(q);

    // ATTN: getCOM()(2) instead of (1) because urdf is aligned on z-axis
    return optParams->robot->getCOM()(2);
}

void qBaseConstraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data) {
    inequalityOptParams* constParams = reinterpret_cast<inequalityOptParams *>(f_data);

    if (grad != NULL) {
        for(int i=0; i<m; i++) {
            for(int j=0; j<n; j++){
                grad[i*n+j] = constParams->P(i, j);
            }
        }
    }

    Eigen::Matrix<double, 1, 1> X;
    for(size_t i=0; i<n; i++) X(i) = x[i];

    Eigen::VectorXd mResult;
    mResult = constParams->P*X - constParams->b;
    for(size_t i=0; i<m; i++) {
        result[i] = mResult(i);
    }
}

// Function Prototypes
// // Generation Methods
Eigen::MatrixXd genCustom2ComPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath);
Eigen::MatrixXd genStepPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, string inputVectorsFilename);
Eigen::MatrixXd genFilterPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, string inputPosesFilename, int stopCount, int lineToSkip);
Eigen::MatrixXd genRandomPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, int numPoses);

// // Balance and Collision Method
Eigen::MatrixXd balanceAndCollision(Eigen::MatrixXd inputPose, Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck);

// // Balancing Methods
Eigen::MatrixXd fullBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose);
Eigen::MatrixXd fixedWheelBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose);
Eigen::MatrixXd doNotBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose);

// // Collision Check Method
bool isColliding(SkeletonPtr robot, Eigen::MatrixXd pose);

// // Read file for step/limit vectors
Eigen::MatrixXd readInputFileAsVectors(string inputVectorsFilename);

// // Read file as matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename, int stopCount, int lineToSkip);

// // Extract filename
string extractFilename(string filename);

// // Random Value
double fRand(double min, double max);

// Main Function
// TODO: commandline arguments and default values right now inputs are hard
// coded
int main() {
    // TODO: Command line flags and arguments
    // TODO: Default values
    double defTolerance = 99;
    int defStopCount = INT_MAX;
    // Default balMethod = full
    // Default genMethod = random

    //INPUT on below line (tolerance level (in meters) for xCOM value from opt code)
    //double tolerance = defTolerance;
    double tolerance = 0.001;

    //INPUT on below line (number of poses to generate)
    int numPoses = 500;

    //INPUT on below line (check for collision)
    bool collisionCheck = false;

    //INPUT on below line (input file for step/limit vectors)
    string inputVectorsFilename = "../randomPoses5000.txt";

    //INPUT on below line (input pose file)
    string inputPosesFilename = "../randomPoses5000.txt";

    //INPUT on below line (stop count)
    int stopCount = 10;

    //INPUT on below line (lineToSkip)
    int lineToSkip = 1;

    //INPUT on below lilne (balancing method)
    string balMethod = "full";
    // Options: none, full, fixedwheel

    //INPUT on below line (generation method)
    string genMethod = "random";
    // Options: custom2com, step, filter, random

    //INPUT on below line (full robot path)
    string fullRobotPath = "/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf";

    //INPUT on below line (fixed wheel robot path)
    string fixedWheelRobotPath = "/home/apatel435/Desktop/09-URDF/KrangFixedWheels/krang_fixed_wheel.urdf";

    // Name output file
    string outfilename;

    if (balMethod != "none" && balMethod != "fixedwheel") {
        balMethod = "full";
    }
    balMethod = balMethod + "balance";

    string strTolerance = to_string(tolerance) + "tol";
    if (tolerance == defTolerance) {
       strTolerance = "notol";
    }

    string collision = "unsafe";
    if (collisionCheck) {
        collision = "safe";
    }

    // Extract actual filename of inputs
    string inputVectorsName = extractFilename(inputVectorsFilename);
    string inputPosesName = extractFilename(inputPosesFilename);

    string ext = ".txt";

    // Generate output
    Eigen::MatrixXd outputPoses;

    cout << "Generating Poses ...\n";

    if (genMethod == "custom2com") {

        outputPoses = genCustom2ComPoses(doNotBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath);
        outfilename = genMethod + balMethod + strTolerance + collision + ext;

    } else if (genMethod == "step") {

        if (balMethod == "none") {
            outputPoses = genStepPoses(doNotBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputVectorsFilename);
        } else if (balMethod == "fixedwheel") {
            outputPoses = genStepPoses(fixedWheelBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputVectorsFilename);
        } else {
            // Default to Full Balance
            outputPoses = genStepPoses(fullBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputVectorsFilename);
        }
        outfilename = genMethod + inputVectorsName + balMethod + strTolerance + collision + ext;

    } else if (genMethod == "filter") {

        if (balMethod == "none") {
            outputPoses = genFilterPoses(doNotBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputPosesFilename, stopCount, lineToSkip);
        } else if (balMethod == "fixedwheel") {
            outputPoses = genFilterPoses(fixedWheelBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputPosesFilename, stopCount, lineToSkip);
        } else {
            // Default to Full Balance
            outputPoses = genFilterPoses(fullBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputPosesFilename, stopCount, lineToSkip);
        }
        outfilename = genMethod + inputPosesName + to_string(outputPoses.rows()) + to_string(lineToSkip) + "skip" + balMethod + strTolerance + collision + ext;

    } else {
        // Default to Random
        genMethod = "random";
        if (balMethod == "none") {
            outputPoses = genRandomPoses(doNotBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, numPoses);
        } else if (balMethod == "fixedwheel") {
            outputPoses = genRandomPoses(fixedWheelBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, numPoses);
        } else {
            // Default to Full Balance
            // Efficiency rate seems to be 75:1 // 5000:66
            // 10000 Poses took 5 hrs and 5 mins for 0.001 tolerance level
            outputPoses = genRandomPoses(fullBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, numPoses);
        }
        outfilename = genMethod + to_string(numPoses) + balMethod + strTolerance + collision + ext;
   }

    cout << "|-> Done\n";

    // Write outputPoses to file
    cout << "Writing Poses to " << outfilename << " ...\n";

    ofstream outfile;
    outfile.open(outfilename);
    outfile << outputPoses;
    outfile.close();

    cout << "|-> Done\n";
}

// Functions
// // Generation Methods
Eigen::MatrixXd genCustom2ComPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath) {
    Eigen::MatrixXd outputPoses;

    // step size of the joint positions
    // Waist - 3
    // Torso - 4
    // Left Shoulder - 4
    // Right Shoulder - 4
    int step[] = {3, 4, 4, 4};

    //Number of arm poses for each step of waist, torso, & left/right shoulders
    int armPoses = 3;
    int numArmValues = 6;

    // Poses for Rest of the Arms obtained from 2_com pdf file located on Krang
    // Google Drive

    double jointConfig[] = {2.5,0,0,1.57,-0.785,0,-0.785,0,-0.785,0,-1.57,0.785,0,0.785,0,0.785,0,
                          2.5,0,0,1.57,-1.57,0,0,0,0,0,0,0,0,0,0,0,0,
                          2.5,0,0,0,0,0,1.57,0,1.57,0,-1.57,0,0,-1.57,0,-1.57,0,
                          2,0,0,1.57,-0.785,0,-0.785,0,-0.785,0,-1.57,0.785,0,0.785,0,0.785,0,
                          2,0,0,1.57,-1.57,0,0,0,0,0,0,0,0,0,0,0,0,
                          2,0,0,0,0,0,1.57,0,1.57,0,-1.57,0,0,-1.57,0,-1.57,0,
                          1.5,0,0,1.57,-0.785,0,-0.785,0,-0.785,0,-1.57,0.785,0,0.785,0,0.785,0,
                          1,0,0,1.57,-1.57,0,0,0,0,0,0,0,0,0,0,0,0,
                          1.5,0,0,0,0,0,1.57,0,1.57,0,-1.57,0,0,-1.57,0,-1.57,0,
                          1.5,0,0,0,-0.785,0,-0.785,0,-0.785,0,0,0.785,0,0.785,0,0.785,0};

    double armPoseLArr[] = {-0.785, 0, -0.785, 0, -0.785, 0,
                             -1.57, 0,      0, 0,      0, 0,
                              1.57, 0,   1.57, 0,  -1.57, 0};
    double armPoseRArr[] = { 0.785, 0,  0.785, 0,  0.785, 0,
                                 0, 0,      0, 0,      0, 0,
                                 0, 0,  -1.57, 0,  -1.57, 0};

    double a = 0; //axis angle 1
    double b = 0; //axis angle 2
    double c = 0; //axis angle 3
    double d = 0; //x
    double e = 0; //y
    double f = 0; //z
    double g = 0; //qLWheel
    double h = 0; //qRWheel
    double k = 0; //qKinect

    // Instantiate full robot
    DartLoader loader;
    SkeletonPtr fullRobot = loader.parseSkeleton(fullRobotPath);

    SkeletonPtr fixedWheelRobot;
    if (balance == fixedWheelBalancePose) {
        SkeletonPtr fixedWheelRobot = loader.parseSkeleton(fixedWheelRobotPath);
    }

    // Check xCOM value
    double xCOM;

    Eigen::MatrixXd customPoseParams(25, 1);

    int poseCounter = 0;
    cout << "Poses: " << poseCounter;

    for (double i = lowerLimit[0] ; i < upperLimit[0];  i += (upperLimit[0]  - lowerLimit[0] ) / step[0] ) {
    for (double j = lowerLimit[1] ; j < upperLimit[1];  j += (upperLimit[1]  - lowerLimit[1] ) / step[1] ) {
    for (double l = lowerLimit[2] ; l < upperLimit[2];  l += (upperLimit[2]  - lowerLimit[2] ) / step[2] ) {
    for (double s = lowerLimit[9] ; s < upperLimit[9];  s += (upperLimit[9]  - lowerLimit[9] ) / step[3] ) {
    for (int armPoseL = 0; armPoseL < armPoses; armPoseL++) {
    for (int armPoseR = 0; armPoseR < armPoses; armPoseR++) {

        // Write the default values first, qWaist, qTorso, qKinect
        customPoseParams(0, 0) = a;
        customPoseParams(1, 0) = b;
        customPoseParams(2, 0) = c;
        customPoseParams(3, 0) = d;
        customPoseParams(4, 0) = e;
        customPoseParams(5, 0) = f;
        customPoseParams(6, 0) = g;
        customPoseParams(7, 0) = h;
        customPoseParams(8, 0) = i;
        customPoseParams(9, 0) = j;
        customPoseParams(10, 0) = k;

        // Write left arm values (shoulder first)
        customPoseParams(11, 0) = l;
        int index = 12;
        for (int ii = 0; ii < 6; ii++) {
            customPoseParams(index, 0) = armPoseLArr[armPoseL*numArmValues+ii];
            index++;
        }

        // Write right arm values (shoulder first)
        customPoseParams(index, 0) = s;
        index++;
        for (int ii = 0; ii < 6; ii++) {
            customPoseParams(index, 0) = armPoseRArr[armPoseR*numArmValues+ii];
            index++;
        }

        // Run it through balancing
        try {
            Eigen::MatrixXd balPoseParams;

            if (balance == fixedWheelBalancePose) {
                balPoseParams = balance(fixedWheelRobot, customPoseParams);
            } else {
                balPoseParams = balance(fullRobot, customPoseParams);
            }

            // Run it through collision check
            bool isCollision = false;
            if (collisionCheck == true) {
                isCollision = isColliding(fullRobot, balPoseParams);
            }

            if (!isCollision) {

                // Set position of full robot to the pose
                fullRobot->setPositions(balPoseParams);

                // Get x center of mass
                xCOM = fullRobot->getCOM()(0);

                // Check for tolerance; if it passes increment pose counter
                if (balance == doNotBalancePose || abs(xCOM) < tolerance) {

                    balPoseParams.transposeInPlace();

                    Eigen::MatrixXd tmp(outputPoses.rows()+balPoseParams.rows(), balPoseParams.cols());

                    if (outputPoses.rows() == 0) {
                        tmp << balPoseParams;
                    } else {
                        tmp << outputPoses,
                               balPoseParams;
                    }
                    outputPoses = tmp;
                    cout << "\rPoses: " << ++poseCounter;
                }
            }
        } catch(std::exception &e) {
            //Do nothing
        }

    }}}}}}

    cout << endl;

    return outputPoses;
}
// TODO: Add in the input file with step/limit vectors
Eigen::MatrixXd genStepPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, string inputVectorsFilename) {
    Eigen::MatrixXd outputPoses;

    // What we change: qWaist, qTorso, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
    // i, j, l, m, n, o, p, q, r, s, t, u, v, w, x, y
    // What we keep same
    double a = 0; //axis-angle1
    double b = 0; //axis-angle2
    double c = 0; //axis-angle3
    double d = 0; //x
    double e = 0; //y
    double f = 0; //z
    double g = 0; //qLWheel
    double h = 0; //qRWheel
    double k = 0; //qKinect

    // TODO: get from another method
    //double lowerLimit[] = {0, -1.57, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi};
    //double upperLimit[] = {2.88, 1.57, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi};
    int stepAll = 1;
    int step[] = {stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll};

    Eigen::MatrixXd stepPoseParams(25, 1);

    // Instantiate full robot
    DartLoader loader;
    SkeletonPtr fullRobot = loader.parseSkeleton(fullRobotPath);

    SkeletonPtr fixedWheelRobot;
    if (balance == fixedWheelBalancePose) {
        SkeletonPtr fixedWheelRobot = loader.parseSkeleton(fixedWheelRobotPath);
    }

    // Check xCOM value
    double xCOM;

    int poseCounter = 0;
    cout << "Poses: " << poseCounter;

    for (double i = lowerLimit[0] ; i < upperLimit[0];  i += (upperLimit[0]  - lowerLimit[0] ) / step[0] ) {
    for (double j = lowerLimit[1] ; j < upperLimit[1];  j += (upperLimit[1]  - lowerLimit[1] ) / step[1] ) {
    for (double l = lowerLimit[2] ; l < upperLimit[2];  l += (upperLimit[2]  - lowerLimit[2] ) / step[2] ) {
    for (double m = lowerLimit[3] ; m < upperLimit[3];  m += (upperLimit[3]  - lowerLimit[3] ) / step[3] ) {
    for (double n = lowerLimit[4] ; n < upperLimit[4];  n += (upperLimit[4]  - lowerLimit[4] ) / step[4] ) {
    for (double o = lowerLimit[5] ; o < upperLimit[5];  o += (upperLimit[5]  - lowerLimit[5] ) / step[5] ) {
    for (double p = lowerLimit[6] ; p < upperLimit[6];  p += (upperLimit[6]  - lowerLimit[6] ) / step[6] ) {
    for (double q = lowerLimit[7] ; q < upperLimit[7];  q += (upperLimit[7]  - lowerLimit[7] ) / step[7] ) {
    for (double r = lowerLimit[8] ; r < upperLimit[8];  r += (upperLimit[8]  - lowerLimit[8] ) / step[8] ) {
    for (double s = lowerLimit[9] ; s < upperLimit[9];  s += (upperLimit[9]  - lowerLimit[9] ) / step[9] ) {
    for (double t = lowerLimit[10]; t < upperLimit[10]; t += (upperLimit[10] - lowerLimit[10]) / step[10]) {
    for (double u = lowerLimit[11]; u < upperLimit[11]; u += (upperLimit[11] - lowerLimit[11]) / step[11]) {
    for (double v = lowerLimit[12]; v < upperLimit[12]; v += (upperLimit[12] - lowerLimit[12]) / step[12]) {
    for (double w = lowerLimit[13]; w < upperLimit[13]; w += (upperLimit[13] - lowerLimit[13]) / step[13]) {
    for (double x = lowerLimit[14]; x < upperLimit[14]; x += (upperLimit[14] - lowerLimit[14]) / step[14]) {
    for (double y = lowerLimit[15]; y < upperLimit[15]; y += (upperLimit[15] - lowerLimit[15]) / step[15]) {

        stepPoseParams << a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x, y;

        // Run it through balancing
        try {
            Eigen::MatrixXd balPoseParams;

            if (balance == fixedWheelBalancePose) {
                balPoseParams = balance(fixedWheelRobot, stepPoseParams);
            } else {
                balPoseParams = balance(fullRobot, stepPoseParams);
            }

            // Run it through collision check
            bool isCollision = false;
            if (collisionCheck == true) {
                isCollision = isColliding(fullRobot, balPoseParams);
            }

            if (!isCollision) {

                // Set position of full robot to the pose
                fullRobot->setPositions(balPoseParams);

                // Get x center of mass
                xCOM = fullRobot->getCOM()(0);

                // Check for tolerance; if it passes increment pose counter
                if (balance == doNotBalancePose || abs(xCOM) < tolerance) {

                    balPoseParams.transposeInPlace();

                    Eigen::MatrixXd tmp(outputPoses.rows()+balPoseParams.rows(), balPoseParams.cols());

                    if (outputPoses.rows() == 0) {
                        tmp << balPoseParams;
                    } else {
                        tmp << outputPoses,
                               balPoseParams;
                    }
                    outputPoses = tmp;
                    cout << "\rPoses: " << ++poseCounter;
                }
            }
        } catch(std::exception &e) {
            //Do nothing
        }

    }}}}}}}}}}}}}}}}

    cout << endl;

    return outputPoses;
}

Eigen::MatrixXd genFilterPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, string inputPosesFilename, int stopCount, int lineToSkip) {
    Eigen::MatrixXd outputPoses;

    // Read input file into Eigen::MatrixXd
    Eigen::MatrixXd unfilteredPoses = readInputFileAsMatrix(inputPosesFilename, stopCount, lineToSkip);

    // Each row is a pose, each colomn is a param
    int numInputPoses = unfilteredPoses.rows();

    // Instantiate full robot
    DartLoader loader;
    SkeletonPtr fullRobot = loader.parseSkeleton(fullRobotPath);

    SkeletonPtr fixedWheelRobot;
    if (balance == fixedWheelBalancePose) {
        SkeletonPtr fixedWheelRobot = loader.parseSkeleton(fixedWheelRobotPath);
    }

    // Check xCOM value
    double xCOM;

    int poseCounter = 0;
    cout << "Poses: " << poseCounter;

    // Run it through balancing
    while (poseCounter < numInputPoses) {
        Eigen::MatrixXd unfilteredPoseParams = unfilteredPoses.row(poseCounter);

        try {
            Eigen::MatrixXd balPoseParams;

            if (balance == fixedWheelBalancePose) {
                balPoseParams = balance(fixedWheelRobot, unfilteredPoseParams);
            } else {
                balPoseParams = balance(fullRobot, unfilteredPoseParams);
            }

            // Run it through collision check
            bool isCollision = false;
            if (collisionCheck == true) {
                isCollision = isColliding(fullRobot, balPoseParams);
            }

            if (!isCollision) {

                // Set position of full robot to the pose
                fullRobot->setPositions(balPoseParams);

                // Get x center of mass
                xCOM = fullRobot->getCOM()(0);

                // Check for tolerance; if it passes increment pose counter
                if (balance == doNotBalancePose || abs(xCOM) < tolerance) {

                    balPoseParams.transposeInPlace();

                    Eigen::MatrixXd tmp(outputPoses.rows()+balPoseParams.rows(), balPoseParams.cols());

                    if (outputPoses.rows() == 0) {
                        tmp << balPoseParams;
                    } else {
                        tmp << outputPoses,
                               balPoseParams;
                    }
                    outputPoses = tmp;
                    cout << "\rPose: " << ++poseCounter;
                }
            }
        } catch(std::exception &e) {
            //Do nothing
        }
    }

    cout << endl;

    return outputPoses;
}

Eigen::MatrixXd genRandomPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, int numPoses) {
    Eigen::MatrixXd outputPoses;

    double a = 0; //axis-angle1
    double b = 0; //axis-angle2
    double c = 0; //axis-angle3
    double d = 0; //x
    double e = 0; //y
    double f = 0; //z
    double g = 0; //qLWheel
    double h = 0; //qRWheel
    double k = 0; //qKinect

    // Instantiate full robot
    DartLoader loader;
    SkeletonPtr fullRobot = loader.parseSkeleton(fullRobotPath);

    SkeletonPtr fixedWheelRobot;
    if (balance == fixedWheelBalancePose) {
        SkeletonPtr fixedWheelRobot = loader.parseSkeleton(fixedWheelRobotPath);
    }

    // Check xCOM value
    double xCOM;

    int poseCounter = 0;
    cout << "Pose: " << poseCounter;
    while (poseCounter < numPoses) {

        // Create random pose vector
        Eigen::MatrixXd randomPoseParams(25, 1);

        // Add the default values first
        randomPoseParams(0, 0) = a;
        randomPoseParams(1, 0) = b;
        randomPoseParams(2, 0) = c;
        randomPoseParams(3, 0) = d;
        randomPoseParams(4, 0) = e;
        randomPoseParams(5, 0) = f;
        randomPoseParams(6, 0) = g;
        randomPoseParams(7, 0) = h;

        int index = 8;

        // Loop through adding the rest of the values (qWaist, qTorso)
        int ii = 0;
        for (;ii < 2; ii++) {
            double ard = fRand(lowerLimit[ii], upperLimit[ii]);
            randomPoseParams(index, 0) = ard;
            index++;
        }

        // Add qKinect
        randomPoseParams(index, 0) = k;
        index++;

        // Add the rest of the values (qArms)
        for (;ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
            double ard = fRand(lowerLimit[ii], upperLimit[ii]);
            randomPoseParams(index, 0) = ard;
            index++;
        }

        // Run it through balancing
        try {
            Eigen::MatrixXd balPoseParams;

            if (balance == fixedWheelBalancePose) {
                balPoseParams = balance(fixedWheelRobot, randomPoseParams);
            } else {
                balPoseParams = balance(fullRobot, randomPoseParams);
            }

            // Run it through collision check
            bool isCollision = false;
            if (collisionCheck == true) {
                isCollision = isColliding(fullRobot, balPoseParams);
            }

            if (!isCollision) {

                // Set position of full robot to the pose
                fullRobot->setPositions(balPoseParams);

                // Get x center of mass
                xCOM = fullRobot->getCOM()(0);

                // Check for tolerance; if it passes increment pose counter
                if (balance == doNotBalancePose || abs(xCOM) < tolerance) {

                    balPoseParams.transposeInPlace();

                    Eigen::MatrixXd tmp(outputPoses.rows()+balPoseParams.rows(), balPoseParams.cols());

                    if (outputPoses.rows() == 0) {
                        tmp << balPoseParams;
                    } else {
                        tmp << outputPoses,
                               balPoseParams;
                    }
                    outputPoses = tmp;

                    cout << "\rPose: " << ++poseCounter;
                }
            }
        } catch(std::exception &e) {
            //Do nothing
        }
    }

    cout << endl;

    return outputPoses;
}

// // Balance and Collision
// TODO: Write this and figure out how to throw an exception?
Eigen::MatrixXd balanceAndCollision(Eigen::MatrixXd inputPose, Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck) {
    return inputPose;
}

// // Balancing Methods
Eigen::MatrixXd fullBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose) {
    const int dof = (const int) robot->getNumDofs();
    comOptParams optParams;
    optParams.robot = robot;
    optParams.qInit << unBalPose;
    nlopt::opt opt(nlopt::LN_COBYLA, dof);
    std::vector<double> unbalPoseParams(dof);
    double minf;
    opt.set_min_objective(comOptFunc, &optParams);
    opt.add_equality_constraint(comConstraint, &optParams, 1e-8);
    opt.add_equality_constraint(wheelAxisConstraint, &optParams, 1e-8);
    opt.add_equality_constraint(headingConstraint, &optParams, 1e-8);
    opt.set_xtol_rel(1e-4);
    opt.set_maxtime(10);
    opt.optimize(unbalPoseParams, minf);
    Eigen::Matrix<double, 25, 1> balPoseParams(unbalPoseParams.data());

    return balPoseParams;
}

// TODO Need to fix this one (packing/dimension issues)
Eigen::MatrixXd fixedWheelBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose) {
    comOptParams optParams;
    optParams.robot = robot;

    // TODO: need to unpackage unBalPose as well for 18 dof instead of input 25
    // dof
    optParams.qInit << unBalPose;

    // The specific algorithm to use (COBYLA: Constrained Optimization BY
    // Linear Approximations
    nlopt::opt opt(nlopt::LN_COBYLA, 1);
    vector<double> unbalPoseParams(1);
    double minf;

    // Minimize the objective function
    opt.set_min_objective(comSimpleOptFunc, &optParams);

    // Add the inequality constraint on qBase
    const vector<double> inequalityconstraintTol(2, 1e-3);
    inequalityOptParams inequalityconstraintParams;

    // Set P and b vectors
    // A way to represent the inequality constraint where qBase should be
    // between -pi/2 and pi/2 so that our COM is above the wheel not below
    // it
    Eigen::MatrixXd setP(2,1);
    Eigen::MatrixXd setb(2,1);

    setP(0,0) = 1;
    setP(1,0) = -1;
    setb(0,0) = M_PI/2;
    setb(1,0) = M_PI/2;

    inequalityconstraintParams.P = setP;
    inequalityconstraintParams.b = setb;

    opt.add_inequality_mconstraint(qBaseConstraint, &inequalityconstraintParams, inequalityconstraintTol);

    //Set relative tolerance on opt params
    opt.set_xtol_rel(1e-4);

    //Set max time allocation for optimizing
    opt.set_maxtime(10);

    opt.optimize(unbalPoseParams, minf);

    Eigen::Matrix<double, 18, 1> balPoseParams(unbalPoseParams.data());

    // TODO: Pack result in 25 dof from 18 dof for full robot
    double z = 0; //axis-angle1
    double a = 0; //axis-angle2
    double b = 0; //axis-angle3
    double c = 0; //x
    double d = 0; //y
    double e = 0; //z
    double f = 0; //qLWheel
    double g = 0; //qRWheel
    double j = 0; //qKinect

    return balPoseParams;
}

Eigen::MatrixXd doNotBalancePose(SkeletonPtr robot, Eigen::MatrixXd unbalPose) {
    return unbalPose;
}

// // Collision Check
// TODO: Need to figure out how to use collision method for skeletonPtr in DART
bool isColliding(SkeletonPtr robot, Eigen::MatrixXd pose) {
    bool isColliding = false;

    if (false) {
        isColliding = true;
    }

    return isColliding;
}

// // Random Value
double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

// // Read file for step/limit vectors
// TODO: (determine if vectors are feasible if not then exit program)
Eigen::MatrixXd readInputFileAsVectors(string inputVectorsFilename) {
    return readInputFileAsMatrix(inputVectorsFilename, INT_MAX, 1);
}

// // Read file as Matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename, int stopCount, int lineToSkip) {
    // Read numbers (the pose params)
    ifstream infile;
    infile.open(inputPosesFilename);

    cout << "Reading input poses ...\n";

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    int lineNumber = 1;
    while(! infile.eof() && rows <= stopCount) {
        if (lineNumber == lineToSkip) {
            string line;
            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while(! stream.eof())
                stream >> buff[cols*rows+temp_cols++];
            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
            lineNumber = 0;
        }
        lineNumber++;
    }

    infile.close();
    cout << "|-> Done\n";
    rows--;

    // Populate matrix with numbers.
    Eigen::MatrixXd outputMatrix(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            outputMatrix(i,j) = buff[cols*i+j];

    return outputMatrix;
}

// // Extract Filename
string extractFilename(string filename) {
    // Remove directory if present.
    // Do this before extension removal incase directory has a period character.
    const size_t last_slash_idx = filename.find_last_of("\\/");
    if (std::string::npos != last_slash_idx) {
        filename.erase(0, last_slash_idx + 1);
    }
    // Remove extension if present.
    const size_t period_idx = filename.rfind('.');
    if (std::string::npos != period_idx) {
        filename.erase(period_idx);
    }

    return filename;
}
