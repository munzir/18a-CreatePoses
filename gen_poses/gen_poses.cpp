// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To generate a set of poses (balanced/collision free) in which the xCOM is within a certain
// tolerance level

// Input and Output file format (Munzir Style)
// heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6
// Use alphabet scheme to identify them as variables in order

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "../../18h-Util/balance.hpp"
#include "../../18h-Util/collision.hpp"
#include "../../18h-Util/convert_pose_formats.hpp"
#include "../../18h-Util/file_ops.hpp"
#include "../../18h-Util/random.hpp"

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

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

// Function Prototypes
// // Generation Methods
Eigen::MatrixXd genCustom2ComPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath);

Eigen::MatrixXd genStepPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, string inputVectorsFilename);

Eigen::MatrixXd genFilterPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, string inputPosesFilename, int stopCount, int lineToSkip);

Eigen::MatrixXd genRandomPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, int numPoses);

// // Read file for step/limit vectors
Eigen::MatrixXd readInputFileAsVectors(string inputVectorsFilename);

// // Read file as matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename, int stopCount, int lineToSkip);

// Main Function
// TODO: commandline arguments and default values right now inputs are hard
// coded
// TODO: Need to convert from munzir to dart format before passing it through
// dart methods
int main() {
    // TODO: Command line flags and arguments
    // TODO: Default values
    double defTolerance = 99;
    int defStopCount = INT_MAX;
    // Default balMethod = full
    // Default genMethod = random

    // Set random seed
    //srand(time(0));
    srand(0);

    //INPUT on below line (full robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/KrangCollision.urdf";

    //INPUT on below line (fixed wheel robot path)
    string fixedWheelRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/KrangFixedWheels/krang_fixed_wheel.urdf";

    //INPUT on below line (generation method)
    string genMethod = "random";
    // Options: custom2com, step, filter, random

    //INPUT on below lilne (balancing method)
    string balMethod = "angle";
    // Options: none, full, fixedwheel, angle

    //INPUT on below line (tolerance level (in meters) for xCOM value from opt code)
    double tolerance = defTolerance;
    tolerance = 0.001;

    //INPUT on below line (check for collision)
    bool collisionCheck = true;
    //bool collisionCheck = false;

    //Step Only
    //INPUT on below line (input file for step/limit vectors)
    string inputVectorsFilename = "../randomPoses5000.txt";

    //Filter Only
    //INPUT on below line (input pose file)
    string inputPosesFilename = "../randomPoses5000.txt1";

    //INPUT on below line (stop count)
    int stopCount = 10;

    //INPUT on below line (lineToSkip)
    int lineToSkip = 1;

    //Random Only
    //INPUT on below line (number of poses to generate)
    int numPoses = 20000;
    //int numPoses = 10;

    // Name output file
    string outfilename;

    if (balMethod != "none" && balMethod != "fixedwheel" && balMethod != "angle") {
        balMethod = "full";
    }
    if (balMethod == "none") {
        tolerance = defTolerance;
    }
    string balMethodName = balMethod + "balance";

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

        if (balMethod == "none") {
            outputPoses = genCustom2ComPoses(doNotBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath);
        } else if (balMethod == "fixedwheel") {
            outputPoses = genCustom2ComPoses(fixedWheelBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath);
        } else if (balMethod == "angle") {
            outputPoses = genCustom2ComPoses(angleBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath);
        } else {
            // Default to full balance
            outputPoses = genCustom2ComPoses(fullBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath);
        }
        outfilename = genMethod + balMethodName + strTolerance + collision + ext;

    } else if (genMethod == "step") {

        if (balMethod == "none") {
            outputPoses = genStepPoses(doNotBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputVectorsFilename);
        } else if (balMethod == "fixedwheel") {
            outputPoses = genStepPoses(fixedWheelBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputVectorsFilename);
        } else if (balMethod == "angle") {
            outputPoses = genStepPoses(angleBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputVectorsFilename);
        } else {
            // Default to Full Balance
            outputPoses = genStepPoses(fullBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputVectorsFilename);
        }
        outfilename = genMethod + inputVectorsName + balMethodName + strTolerance + collision + ext;

    } else if (genMethod == "filter") {

        if (balMethod == "none") {
            outputPoses = genFilterPoses(doNotBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputPosesFilename, stopCount, lineToSkip);
        } else if (balMethod == "fixedwheel") {
            outputPoses = genFilterPoses(fixedWheelBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputPosesFilename, stopCount, lineToSkip);
        } else if (balMethod == "angle") {
            outputPoses = genFilterPoses(angleBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputPosesFilename, stopCount, lineToSkip);
        } else {
            // Default to Full Balance
            outputPoses = genFilterPoses(fullBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, inputPosesFilename, stopCount, lineToSkip);
        }
        outfilename = genMethod + inputPosesName + to_string(outputPoses.rows()) + to_string(lineToSkip) + "skip" + balMethodName + strTolerance + collision + ext;

    } else {
        // Default to Random
        genMethod = "random";
        if (balMethod == "none") {
            outputPoses = genRandomPoses(doNotBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, numPoses);
        } else if (balMethod == "fixedwheel") {
            outputPoses = genRandomPoses(fixedWheelBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, numPoses);
        } else if (balMethod == "angle") {
            // Best balancing method
            // Efficiency rate seems to be 45348:20000 // 2.3:1
            // Took 2 mins
            outputPoses = genRandomPoses(angleBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, numPoses);
        } else {
            // Default to Full Balance
            // Efficiency rate seems to be 75:1 // 5000:66
            // 10000 Poses took 5 hrs and 5 mins for 0.001 tolerance level
            outputPoses = genRandomPoses(fullBalancePose, tolerance, collisionCheck, fullRobotPath, fixedWheelRobotPath, numPoses);
        }
        outfilename = genMethod + to_string(numPoses) + balMethodName + strTolerance + collision + ext;
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
    // step size of the joint positions
    // Waist - 3
    // Torso - 4
    // Left Shoulder - 4
    // Right Shoulder - 4
    int step[] = {2, 3, 3, 3};

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

    int totalPoses = 3 * 4 * 4 * 4 * 3 * 3;
    Eigen::MatrixXd outputPoses(totalPoses, 23);

    double a = 0; //heading
    double b = 0; //qBase
    double c = 0; //x
    double d = 0; //y
    double e = 0.264; //z
    double f = 0; //qLWheel
    double g = 0; //qRWheel

    // Instantiate full robot
    DartLoader loader;
    SkeletonPtr fullRobot = loader.parseSkeleton(fullRobotPath);

    SkeletonPtr fixedWheelRobot;
    if (balance == fixedWheelBalancePose) {
        SkeletonPtr fixedWheelRobot = loader.parseSkeleton(fixedWheelRobotPath);
    }

    Eigen::MatrixXd customPoseParams(23, 1);

    int poseCounter = 0;
    int totalPoseCounter = 0;
    cout << "Written Pose: " << poseCounter << " Filtered Pose: " << totalPoseCounter << "/" << totalPoses;

    for (double h = lowerLimit[0] ; h <= upperLimit[0];  h += (upperLimit[0]  - lowerLimit[0] ) / step[0] ) {
    for (double i = lowerLimit[1] ; i <= upperLimit[1];  i += (upperLimit[1]  - lowerLimit[1] ) / step[1] ) {
    for (double j = lowerLimit[2] ; j <= upperLimit[2];  j += (upperLimit[2]  - lowerLimit[2] ) / step[2] ) {
    for (double q = lowerLimit[9] ; q <= upperLimit[9];  q += (upperLimit[9]  - lowerLimit[9] ) / step[3] ) {
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

        // Write left arm values (shoulder first)
        customPoseParams(11, 0) = j;
        int index = 9;
        for (int ii = 0; ii < 6; ii++) {
            customPoseParams(index, 0) = armPoseLArr[armPoseL*numArmValues+ii];
            index++;
        }

        // Write right arm values (shoulder first)
        customPoseParams(index, 0) = q;
        index++;
        for (int ii = 0; ii < 6; ii++) {
            customPoseParams(index, 0) = armPoseRArr[armPoseR*numArmValues+ii];
            index++;
        }

        // Run it through balancing and collision check, if it passes then add
        // it to final output
        try {
            Eigen::MatrixXd balPoseParams = balanceAndCollision(customPoseParams, fullRobot, fixedWheelRobot, balance, tolerance, collisionCheck);
            outputPoses.row(poseCounter) = balPoseParams;
            poseCounter++;
        } catch (exception& e) {
            // Continue without adding that pose
        }

        totalPoseCounter++;
        cout << "\rWritten Pose: " << poseCounter << " Filtered Pose: " << totalPoseCounter << "/" << totalPoses;

    }}}}}}

    cout << endl;

    Eigen::MatrixXd finalOutputPoses = outputPoses.topRows(poseCounter);
    return finalOutputPoses;
}

// TODO: Add in the input file with step/limit vectors
// TODO: (determine if step/limit vectors are feasible if not then exit program)
Eigen::MatrixXd genStepPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, string inputVectorsFilename) {
    // What we change: qWaist, qTorso, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
    // h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w
    // What we keep same
    double a = 0; //heading
    double b = 0; //qBase
    double c = 0; //x
    double d = 0; //y
    double e = 0.264; //z
    double f = 0; //qLWheel
    double g = 0; //qRWheel

    // TODO: get from another method
    //double lowerLimit[] = {0, -1.57, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi};
    //double upperLimit[] = {2.88, 1.57, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi};
    int stepAll = 1;
    int step[] = {stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll};

    int totalPoses = 1;
    int stepArrSize = sizeof(step)/sizeof(step[0]);
    for (int index = 0; index < stepArrSize; index++) {
        totalPoses = totalPoses * (step[index] + 1);
    }
    Eigen::MatrixXd outputPoses(totalPoses, 23);

    Eigen::MatrixXd stepPoseParams(23, 1);

    // Instantiate full robot
    DartLoader loader;
    SkeletonPtr fullRobot = loader.parseSkeleton(fullRobotPath);

    SkeletonPtr fixedWheelRobot;
    if (balance == fixedWheelBalancePose) {
        SkeletonPtr fixedWheelRobot = loader.parseSkeleton(fixedWheelRobotPath);
    }

    int poseCounter = 0;
    int totalPoseCounter = 0;
    cout << "Written Pose: " << poseCounter << " Filtered Pose: " << totalPoseCounter << "/" << totalPoses;

    for (double h = lowerLimit[0] ; h <= upperLimit[0];  h += (upperLimit[0]  - lowerLimit[0] ) / step[0] ) {
    for (double i = lowerLimit[1] ; i <= upperLimit[1];  i += (upperLimit[1]  - lowerLimit[1] ) / step[1] ) {
    for (double j = lowerLimit[2] ; j <= upperLimit[2];  j += (upperLimit[2]  - lowerLimit[2] ) / step[2] ) {
    for (double k = lowerLimit[3] ; k <= upperLimit[3];  k += (upperLimit[3]  - lowerLimit[3] ) / step[3] ) {
    for (double l = lowerLimit[4] ; l <= upperLimit[4];  l += (upperLimit[4]  - lowerLimit[4] ) / step[4] ) {
    for (double m = lowerLimit[5] ; m <= upperLimit[5];  m += (upperLimit[5]  - lowerLimit[5] ) / step[5] ) {
    for (double n = lowerLimit[6] ; n <= upperLimit[6];  n += (upperLimit[6]  - lowerLimit[6] ) / step[6] ) {
    for (double o = lowerLimit[7] ; o <= upperLimit[7];  o += (upperLimit[7]  - lowerLimit[7] ) / step[7] ) {
    for (double p = lowerLimit[8] ; p <= upperLimit[8];  p += (upperLimit[8]  - lowerLimit[8] ) / step[8] ) {
    for (double q = lowerLimit[9] ; q <= upperLimit[9];  q += (upperLimit[9]  - lowerLimit[9] ) / step[9] ) {
    for (double r = lowerLimit[10]; r <= upperLimit[10]; r += (upperLimit[10] - lowerLimit[10]) / step[10]) {
    for (double s = lowerLimit[11]; s <= upperLimit[11]; s += (upperLimit[11] - lowerLimit[11]) / step[11]) {
    for (double t = lowerLimit[12]; t <= upperLimit[12]; t += (upperLimit[12] - lowerLimit[12]) / step[12]) {
    for (double u = lowerLimit[13]; u <= upperLimit[13]; u += (upperLimit[13] - lowerLimit[13]) / step[13]) {
    for (double v = lowerLimit[14]; u <= upperLimit[14]; u += (upperLimit[14] - lowerLimit[14]) / step[14]) {
    for (double w = lowerLimit[15]; u <= upperLimit[15]; u += (upperLimit[15] - lowerLimit[15]) / step[15]) {

        stepPoseParams << a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w;

        // Run it through balancing and collision check, if it passes then add
        // it to final output
        try {
            Eigen::MatrixXd balPoseParams = balanceAndCollision(stepPoseParams, fullRobot, fixedWheelRobot, balance, tolerance, collisionCheck);
            outputPoses.row(poseCounter) = balPoseParams;
            ++poseCounter;
        } catch (exception& e) {
            // Continue without adding that pose
        }

        totalPoseCounter++;
        cout << "\rWritten Pose: " << poseCounter << " Filtered Pose: " << totalPoseCounter << "/" << totalPoses;

    }}}}}}}}}}}}}}}}

    cout << endl;

    Eigen::MatrixXd finalOutputPoses = outputPoses.topRows(poseCounter);
    return finalOutputPoses;
}

Eigen::MatrixXd genFilterPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, string inputPosesFilename, int stopCount, int lineToSkip) {
    // Read input file into Eigen::MatrixXd
    Eigen::MatrixXd unfilteredPoses;
    try {
        cout << "Reading input poses ...\n";
        unfilteredPoses = readInputFileAsMatrix(inputPosesFilename, stopCount, lineToSkip);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        exit(EXIT_FAILURE);
    }

    // Each row is a pose, each colomn is a param
    int numInputPoses = unfilteredPoses.rows();

    // Output matrix
    Eigen::MatrixXd outputPoses(numInputPoses, 23);

    // Instantiate full robot
    DartLoader loader;
    SkeletonPtr fullRobot = loader.parseSkeleton(fullRobotPath);

    SkeletonPtr fixedWheelRobot;
    if (balance == fixedWheelBalancePose) {
        SkeletonPtr fixedWheelRobot = loader.parseSkeleton(fixedWheelRobotPath);
    }

    int poseCounter = 0;
    int totalPoseCounter = 0;
    cout << "Written Pose: " << poseCounter << " Filtered Pose: " << totalPoseCounter << "/" << numInputPoses;

    // Run it through balancing
    while (poseCounter < numInputPoses) {
        Eigen::MatrixXd unfilteredPoseParams = unfilteredPoses.row(poseCounter);

        // Run it through balancing and collision check, if it passes then add
        // it to final output
        try {
            Eigen::MatrixXd balPoseParams = balanceAndCollision(unfilteredPoseParams, fullRobot, fixedWheelRobot, balance, tolerance, collisionCheck);
            outputPoses.row(poseCounter) = balPoseParams;
            ++poseCounter;
        } catch (exception& e) {
            // Continue without adding that pose
        }

        totalPoseCounter++;
        cout << "\rWritten Pose: " << poseCounter << " Filtered Pose: " << totalPoseCounter << "/" << numInputPoses;
    }

    cout << endl;
    Eigen::MatrixXd finalOutputPoses = outputPoses.topRows(poseCounter);
    return finalOutputPoses;
}

Eigen::MatrixXd genRandomPoses(Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck, string fullRobotPath, string fixedWheelRobotPath, int numPoses) {
    Eigen::MatrixXd outputPoses(numPoses, 23);

    double a = 0; //heading
    double b = 0; //qBase
    double c = 0; //x
    double d = 0; //y
    double e = 0.264; //z
    double f = 0; //qLWheel
    double g = 0; //qRWheel

    // Instantiate full robot
    DartLoader loader;
    SkeletonPtr fullRobot = loader.parseSkeleton(fullRobotPath);

    SkeletonPtr fixedWheelRobot;
    if (balance == fixedWheelBalancePose) {
        SkeletonPtr fixedWheelRobot = loader.parseSkeleton(fixedWheelRobotPath);
    }

    int poseCounter = 0;
    int totalPoseCounter;
    cout << "Written Pose: " << poseCounter << "/" << numPoses << " Filtered Pose: " << totalPoseCounter;
    while (poseCounter < numPoses) {

        // Create random pose vector
        Eigen::MatrixXd randomPoseParams(23, 1);

        // Add the default values first
        randomPoseParams(0, 0) = a;
        randomPoseParams(1, 0) = b;
        randomPoseParams(2, 0) = c;
        randomPoseParams(3, 0) = d;
        randomPoseParams(4, 0) = e;
        randomPoseParams(5, 0) = f;
        randomPoseParams(6, 0) = g;

        int index = 7;

        // Loop through adding the rest of the values (qWaist, qTorso)
        int ii = 0;
        for (int ii = 0; ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
            double ard = fRand(lowerLimit[ii], upperLimit[ii]);
            randomPoseParams(index, 0) = ard;
            index++;
        }

        // Run it through balancing and collision check, if it passes then add
        // it to final output
        try {
            Eigen::MatrixXd balPoseParams = balanceAndCollision(randomPoseParams, fullRobot, fixedWheelRobot, balance, tolerance, collisionCheck);
            outputPoses.row(poseCounter) = balPoseParams;
            poseCounter++;
        } catch (exception& e) {
            // Continue without adding that pose
            //cout << endl << e.what() << endl;
        }

        totalPoseCounter++;
        cout << "\rWritten Pose: " << poseCounter << "/" << numPoses << " Filtered Pose: " << totalPoseCounter;
    }

    cout << endl;

    return outputPoses;
}
