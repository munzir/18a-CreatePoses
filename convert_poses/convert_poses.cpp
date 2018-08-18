// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To convert between Munzir's set of coordinates and DART coordinates

// Munizr Coordinate Format
// heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// DART Coordinate format
// axis-angle1, axis-angle2, axis-angle3, x, y, z, qLWheel, qRWheel, qWaist, qTorso,
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
Eigen::MatrixXd convertPoses(string convert, Eigen::MatrixXd inputPosesFilename, string fullRobotPath);

// Main Function
int main() {
    // TODO: Command line flags and arguments

    //INPUT on below line (output format)
    //string convert = "dart2munzir";
    string convert = "munzir2dart";
    // Options: dart2munzir, munzir2dart

    //INPUT on below line (input file to convert)
    //string inputPosesFilename = "../randomPoses5000.txt";
    //string inputPosesFilename = "../finalSetDart.txt";
    //string inputPosesFilename = "../hardware-balanced-posesmunzirdart.txt";
    string inputPosesFilename = "./hardware-balanced-posesmunzirdartmunzir.txt";

    //INPUT on below line (full robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/KrangNoKinect.urdf";

    // Read input file
    Eigen::MatrixXd inputPoses;
    try {
        cout << "Reading input poses ...\n";
        inputPoses = readInputFileAsMatrix(inputPosesFilename);
        //inputPoses = load_file<Eigen::MatrixXd>(inputPosesFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }

    // Generate output
    cout << "Converting Poses ...\n";
    Eigen::MatrixXd outputPoses = convertPoses(convert, inputPoses, fullRobotPath);
    cout << "|-> Done\n";

    // Name output file
    string outfilename;
    string inputName = extractFilename(inputPosesFilename);
    string outputFormat;
    if (convert == "dart2munzir") {
        outputFormat = "munzir";
    } else {
        outputFormat = "dart";
    }
    string ext = ".txt";
    outfilename = inputName + outputFormat + ext;

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
Eigen::MatrixXd convertPoses(string convert, Eigen::MatrixXd inputPoses, string fullRobotPath) {
    int numInputPoses = inputPoses.rows();
    int outputCols;

    // Robot used to find transform matrix
    DartLoader loader;
    SkeletonPtr robot;

    if (convert == "dart2munzir") {
        robot = loader.parseSkeleton(fullRobotPath);
        outputCols = inputPoses.cols() - 1;
    } else {
        outputCols = inputPoses.cols() + 1;
    }

    Eigen::MatrixXd outputPoses(numInputPoses, outputCols);

    // Tranpose big matrix first
    // Cols are now poses, and rows are now params
    inputPoses.transposeInPlace();

    Eigen::MatrixXd convertedPose;

    int poseCounter = 0;
    cout << "Pose: " << poseCounter;

    while (poseCounter < numInputPoses) {

        if (convert == "dart2munzir") {
            convertedPose = dartToMunzir(inputPoses.col(poseCounter), robot);
        } else {
            convertedPose = munzirToDart(inputPoses.col(poseCounter));
        }


        convertedPose.transposeInPlace();
        outputPoses.row(poseCounter) = convertedPose;

        cout << "\rPose: " << ++poseCounter;

    }

    cout << endl;
    return outputPoses;
}
