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

// Function Prototypes
Eigen::MatrixXd reorientAllPoses(Eigen::MatrixXd inputPoses, string fullRobotPath, string reorientOption);

// // Reorient a single pose
Eigen::MatrixXd reorientSinglePose(Eigen::RowVectorXd inputPose, SkeletonPtr robot);

// // Reorient a single pose
Eigen::MatrixXd reorientHeadingSinglePose(Eigen::RowVectorXd inputPose, SkeletonPtr robot);

// // Convert from munzir format back to dart format
Eigen::MatrixXd munzirToDart(Eigen::RowVectorXd munzirPose);

// // Read file as matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename);

// // Extract filename
string extractFilename(string filename);

// Main Function
int main() {
    // TODO: Command line flags and arguments

    //INPUT on below line (input file to convert)
    //string inputPosesFilename = "../randomPoses5000.txt";
    string inputPosesFilename = "../finalSet.txt";

    //INPUT on below line (full robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/Krang.urdf";

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

Eigen::MatrixXd munzirToDart(Eigen::RowVectorXd munzirPose) {
    // Convert input

    // Find the pose in DART formats
    double headingInit = munzirPose(0);
    double qBaseInit = munzirPose(1);
    Eigen::Matrix<double, 22, 1> unchangedValues;
    unchangedValues << munzirPose.segment(2,22).transpose();

    // Calculating the axis angle representation of orientation from headingInit and qBaseInit:
    // RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
    Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));

    // Now compile this data into dartPoseParams
    Eigen::Matrix<double, 25, 1> dartPose;
    dartPose << aa.angle()*aa.axis(), unchangedValues;

    return dartPose;
}

// // Read file as Matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename) {
    // Read numbers (the pose params)
    ifstream infile;
    infile.open(inputPosesFilename);

    if (!infile.is_open()) {
        throw runtime_error(inputPosesFilename + " can not be read, potentially does not exit!");
    }

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    while(! infile.eof()) {
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
    }

    infile.close();
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
