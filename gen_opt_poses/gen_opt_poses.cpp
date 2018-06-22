// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To generate a set of poses in which the xCOM is within a certain
// tolerance level

// Input file format
// heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// Output file format (DART Style)
// axis-angle1, axis-angle2, axis-angle3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <nlopt.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

#define MAXBUFSIZE ((int) 1e6)

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

int filterOptPosesToFile(double tolerance);

int writeRandomOptPosesToFile(int numPoses, double tolerance);
double fRand(double min, double max);

int main() {

    //INPUT on below line (tolerance level (in meters) for xCOM value from opt code)
    double tolerance = 0.001;

    //INPUT on below line (number of poses to generate)
    int numPoses = 10000;

    //filterOptPosesToFile(tolerance);

    //Efficiency rate seems to be 75:1
    // 5000:66
    writeRandomOptPosesToFile(numPoses, tolerance);
}

int filterOptPosesToFile(double tolerance) {

    // Put a hard stop on reading poses just in case
    // INPUT on below line (Hard stop to number of pose readings)
    int controlPoseNums = 1000;
    // INPUT on below line (lines to skip so an even distribution of samples can
    // be taken) Dependent on file lines
    //int linesToSkip = 1000/controlPoseNums;

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers (the pose params)
    ifstream infile;
    // INPUT on below line (input pose file)
    //infile.open("../2_comPoses3ArmPoses.txt");
    infile.open("../randomPoses5000.txt");
    cout << "Reading input poses ...\n";
    //int lineNumber = 0;
    while(! infile.eof() && rows <= controlPoseNums) {
        //if (lineNumber == linesToSkip) {
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
        //lineNumber = 0;
        //}
        //lineNumber++;
    }

    infile.close();
    cout << "|-> Done\n";
    rows--;

    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    int numInputPoses = rows;
    int numParams = cols;

    Eigen::MatrixXd allInitPoseParamsFromFile(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            allInitPoseParamsFromFile(i,j) = buff[cols*i+j];

    Eigen::MatrixXd allInitPoseParams(cols, rows);
    allInitPoseParams = allInitPoseParamsFromFile.transpose();

    // Instantiate "ideal" robot and n other robots
    dart::utils::DartLoader loader;
    // INPUT on below line (absolute path of the Krang URDF file)
    dart::dynamics::SkeletonPtr idealRobot = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

    // Open output file to write balanced poses which are within specified
    // tolerance levels
    ofstream toleranceBalancedPosesFile;
    toleranceBalancedPosesFile.open("toleranceBalancedPoses.txt");

    //Create file to print the actual xCOM of the poses for the ideal robot
    ofstream realxCOMFile;
    realxCOMFile.open("realxCOM.txt");
    double xCOMIdealRobot;

    // Calculate the balanced poses in DART format
    cout << "Calculating Balanced Poses ...\n";

    // Find the pose in DART formats
    for (int pose = 0; pose < numInputPoses; pose++) {
        Eigen::Matrix<double, 24, 1> initPoseParams;
        for (int j = 0; j < numParams; j++) {
            initPoseParams(j) = allInitPoseParams(j, pose);
        }

        double headingInit = initPoseParams(0);
        double qBaseInit = initPoseParams(1);
        Eigen::Matrix<double, 22, 1> unchangedValues;
        unchangedValues << initPoseParams.segment(2,22);

        // Calculating the axis angle representation of orientation from headingInit and qBaseInit:
        // RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
        Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));

        // Now compile this data into dartPoseParams
        Eigen::Matrix<double, 25, 1> dartPoseParams;
        dartPoseParams << aa.angle()*aa.axis(), unchangedValues;

        // Change Pose such that CoM is right on top of wheel axis
        // Im not sure on how this works or the accuracy of it
        // What I am sure is that it changes the input pose to something
        // different than specified
        const int dof = (const int) idealRobot->getNumDofs();
        comOptParams optParams;
        optParams.robot = idealRobot;
        optParams.qInit << dartPoseParams;
        nlopt::opt opt(nlopt::LN_COBYLA, dof);
        std::vector<double> unoptDartPoseParams(dof);
        double minf;
        opt.set_min_objective(comOptFunc, &optParams);
        opt.add_equality_constraint(comConstraint, &optParams, 1e-8);
        opt.add_equality_constraint(wheelAxisConstraint, &optParams, 1e-8);
        opt.add_equality_constraint(headingConstraint, &optParams, 1e-8);
        opt.set_xtol_rel(1e-4);
        opt.set_maxtime(10);
        opt.optimize(unoptDartPoseParams, minf);
        Eigen::Matrix<double, 25, 1> optDartPoseParams(unoptDartPoseParams.data());

        // Set position of ideal robot to the pose in DART format
        idealRobot->setPositions(optDartPoseParams);

        // Print the optDartPoseParams that obey the tolerance level specified

        // Remember this is in Dart's pose format which is
        // axis-angle1, axis-angle2, axix-angle3, x, y, z, qLWheel, qRWheel,
        // qWaist, qTorso, qKinect,
        // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6
        // The difference in input pose and dart pose format is the conversion
        // from heading and base to axis-angle representation
        // all other values are the same

        // Get x center of mass
        double xCOMIdealRobot = idealRobot->getCOM()(0);

        if (abs(xCOMIdealRobot) < tolerance) {
            realxCOMFile << xCOMIdealRobot << "\n";
            toleranceBalancedPosesFile << optDartPoseParams.transpose() << "\n";
        }

    }

    // Close the tolerance balanced poses file
    toleranceBalancedPosesFile.close();

    // Close the real xCOM file
    realxCOMFile.close();

    return 0;
}

int writeRandomOptPosesToFile (int numPoses, double tolerance) {

    ofstream toleranceBalancedPosesFile;
    std::string name = "randomOptPoses";
    std::string ext = ".txt";
    toleranceBalancedPosesFile.open(name + std::to_string(numPoses) + ext);

    double z = 0; //axis-angle1
    double a = 0; //axis-angle2
    double b = 0; //axis-angle3
    double c = 0; //x
    double d = 0; //y
    double e = 0; //z
    double f = 0; //qLWheel
    double g = 0; //qRWheel
    double j = 0; //qKinect

    //exact value of pi or is 3.14 fine? eh prob good enough

    double pi = 3.14;
    double bendLimit = 2.0944;

    double lowerLimit[] = {0, -1.57, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi};
    double upperLimit[] = {2.88, 1.57, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi};

    // Instantiate "ideal" robot
    dart::utils::DartLoader loader;
    // INPUT on below line (absolute path of the Krang URDF file)
    dart::dynamics::SkeletonPtr idealRobot = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

    cout << "Creating and writing balanced poses ...\n";

    int pose = 0;
    while (pose < numPoses) {

        // Create random pose vector
        //
        Eigen::MatrixXd randomPoseParams(25, 1);

        // Write the default values first
        randomPoseParams(0, 0) = z;
        randomPoseParams(1, 0) = a;
        randomPoseParams(2, 0) = b;
        randomPoseParams(3, 0) = c;
        randomPoseParams(4, 0) = d;
        randomPoseParams(5, 0) = e;
        randomPoseParams(6, 0) = f;
        randomPoseParams(7, 0) = g;

        int index = 8;

        // Loop through writing the rest of the values
        int ii = 0;
        for (;ii < 2; ii++) {
            double ard = fRand(lowerLimit[ii], upperLimit[ii]);
            randomPoseParams(index, 0) = ard;
            index++;

        }
        // Write qKinect
        randomPoseParams(index, 0) = j;
        index++;

        //Write the rest of the values
        for (;ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
            double ard = fRand(lowerLimit[ii], upperLimit[ii]);
            randomPoseParams(index, 0) = ard;
            index++;

        }

        // Run it through opt
        //
        const int dof = (const int) idealRobot->getNumDofs();
        comOptParams optParams;
        optParams.robot = idealRobot;
        optParams.qInit << randomPoseParams;
        nlopt::opt opt(nlopt::LN_COBYLA, dof);
        std::vector<double> unoptDartPoseParams(dof);
        double minf;
        opt.set_min_objective(comOptFunc, &optParams);
        opt.add_equality_constraint(comConstraint, &optParams, 1e-8);
        opt.add_equality_constraint(wheelAxisConstraint, &optParams, 1e-8);
        opt.add_equality_constraint(headingConstraint, &optParams, 1e-8);
        opt.set_xtol_rel(1e-4);
        opt.set_maxtime(10);

        try {
            opt.optimize(unoptDartPoseParams, minf);
            Eigen::Matrix<double, 25, 1> optDartPoseParams(unoptDartPoseParams.data());
            // Set position of ideal robot to the pose in DART format
            idealRobot->setPositions(optDartPoseParams);

            // Print the optDartPoseParams that obey the tolerance level specified

            // Get x center of mass
            double xCOMIdealRobot = idealRobot->getCOM()(0);

            // If it passes the tolerance check then print it to file and increment pose
            // counter
            if (abs(xCOMIdealRobot) < tolerance) {
                toleranceBalancedPosesFile << optDartPoseParams.transpose() << "\n";
                pose++;
            }
        } catch(std::exception &e) {
        }

    }

    // Close the tolerance balanced poses file
    toleranceBalancedPosesFile.close();

    cout << "|-> Done\n";

    return 0;
}

double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
