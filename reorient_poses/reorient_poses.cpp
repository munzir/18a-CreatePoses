// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To reorient poses since pose generation has resulted in qBase angle
// offset by 180 degrees and wheels below z(blue)-plane by 0.264

// Munizr Coordinate Format
// heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// DART Coordinate format
// axis-angle1, axis-angle2, axis-angle3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "../../18h-Util/convert_pose_formats.hpp"
#include "../../18h-Util/file_ops.hpp"

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// Function Prototypes
Eigen::MatrixXd reorientAllPoses(Eigen::MatrixXd inputPoses, string fullRobotPath, string reorientOption);

// // Reorient a single pose
Eigen::MatrixXd reorientSinglePose(Eigen::RowVectorXd inputPose, SkeletonPtr robot);

// // Reorient a single pose
Eigen::MatrixXd reorientHeadingSinglePose(Eigen::RowVectorXd inputPose, SkeletonPtr robot);

// Main Function
int main() {
    // TODO: Command line flags and arguments

    //INPUT on below line (input file to convert)
    //string inputPosesFilename = "../randomPoses5000.txt";
    string inputPosesFilename = "../finalSet.txt";

    //INPUT on below line (full robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/KrangNoKinect.urdf";

    //INPUT on below line (reorientation option)
    string reorientOption = "heading";
    // options: all, heading

    // Read input file
    Eigen::MatrixXd inputPoses;
    try {
        cout << "Reading input poses ...\n";
        inputPoses = readInputFileAsMatrix(inputPosesFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }

    // Generate output
    cout << "Reorienting Poses ...\n";
    Eigen::MatrixXd outputPoses = reorientAllPoses(inputPoses, fullRobotPath, reorientOption);
    cout << "|-> Done\n";

    // Name output file
    string outfilename;
    string inputName = extractFilename(inputPosesFilename);
    string outputFormat;

    string ext = ".txt";
    outfilename = "reoriented" + inputName + outputFormat + ext;

    // Writing to file
    cout << "Writing Poses to " << outfilename << " ...\n";
    ofstream outfile;
    outfile.open(outfilename);
    outfile << outputPoses;
    outfile.close();
    cout << "|-> Done\n";

    return 0;
}

// Functions
Eigen::MatrixXd reorientAllPoses(Eigen::MatrixXd inputPoses, string fullRobotPath, string reorientOption) {
    int numInputPoses = inputPoses.rows();
    int outputCols = inputPoses.cols();

    // Robot used to find transform matrix
    DartLoader loader;
    SkeletonPtr robot = loader.parseSkeleton(fullRobotPath);

    Eigen::MatrixXd outputPoses(numInputPoses, outputCols);

    // Tranpose big matrix first
    // Cols are now poses, and rows are now params
    inputPoses.transposeInPlace();

    Eigen::MatrixXd reorientedPose;

    int poseCounter = 0;
    cout << "Pose: " << poseCounter;

    while (poseCounter < numInputPoses) {

        if (reorientOption == "heading") {
            reorientedPose = reorientHeadingSinglePose(inputPoses.col(poseCounter), robot);
        } else {
            reorientedPose = reorientSinglePose(inputPoses.col(poseCounter), robot);
        }

        reorientedPose.transposeInPlace();
        outputPoses.row(poseCounter) = reorientedPose;

        cout << "\rPose: " << ++poseCounter;

    }

    cout << endl;
    return outputPoses;
}

Eigen::MatrixXd reorientSinglePose(Eigen::RowVectorXd inputPose, SkeletonPtr robot) {
    // Find the pose in munzir format
    Eigen::Matrix<double, 22, 1> unchangedValues;
    unchangedValues << inputPose.segment(3,22).transpose();

    // Calculating the headingInit and qBase Init from the axis angle representation of orientation:
    robot->setPositions(inputPose);
    Eigen::MatrixXd baseTf = robot->getBodyNode(0)->getTransform().matrix();
    double headingInit = atan2(baseTf(0, 0), -baseTf(1, 0));
    double qBaseInit = atan2(baseTf(0,1)*cos(headingInit) + baseTf(1,1)*sin(headingInit), baseTf(2,1));
    // I thought the below would work but its not qbase is off by 1.57 rads (90
    // degs) the above expression for qBaseInit fixes that
    //double qBaseInit = atan2(baseTf(2,1), baseTf(2,2));

    // Need to make sure heading is 0
    headingInit = 0;
    // Change qBase angle by -pi
    qBaseInit -= M_PI;
    // Shift z-coordinate by +0.264 to account the distance from center of wheel
    // to the bottom wheel
    unchangedValues(2) = 0.264;

    // Now compile this data back into dart format
    Eigen::Matrix<double, 24, 1> munzirPose;
    munzirPose << headingInit, qBaseInit, unchangedValues;

    Eigen::MatrixXd reorientedPose = munzirToDart(munzirPose);
    return reorientedPose;
}

Eigen::MatrixXd reorientHeadingSinglePose(Eigen::RowVectorXd inputPose, SkeletonPtr robot) {
    // Find the pose in munzir format
    Eigen::Matrix<double, 22, 1> unchangedValues;
    unchangedValues << inputPose.segment(3,22).transpose();

    // Calculating the headingInit and qBase Init from the axis angle representation of orientation:
    robot->setPositions(inputPose);
    Eigen::MatrixXd baseTf = robot->getBodyNode(0)->getTransform().matrix();
    double headingInit = atan2(baseTf(0, 0), -baseTf(1, 0));
    double qBaseInit = atan2(baseTf(0,1)*cos(headingInit) + baseTf(1,1)*sin(headingInit), baseTf(2,1));
    // I thought the below would work but its not qbase is off by 1.57 rads (90
    // degs) the above expression for qBaseInit fixes that
    //double qBaseInit = atan2(baseTf(2,1), baseTf(2,2));

    // Need to make sure heading is 0
    headingInit = 0;

    // Now compile this data back into dart format
    Eigen::Matrix<double, 24, 1> munzirPose;
    munzirPose << headingInit, qBaseInit, unchangedValues;

    Eigen::MatrixXd reorientedPose = munzirToDart(munzirPose);
    return reorientedPose;
}
