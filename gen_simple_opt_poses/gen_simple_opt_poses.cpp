// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To generate a set of poses in which the xCOM is within a certain
// tolerance level

// Output file format for fixed wheel (DART Format)
// TODO
// axis-angle1, axis-angle2, axis-angle3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6
// Assuming:
// qBase, qWaist, qTorso, qKinect,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// TODO
// The real xCOM for fixed wheel krang might not be getLocalCOM(0) but might be on another axis maybe
// z (2)

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

// Functions for optimizing Full Krang Model
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

// Functions for optimizing Fixed Wheel Krang Model
// TODO: Need to change
double comSimpleOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(my_func_data);
  Eigen::Matrix<double, 25, 1> q(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, 25, 1> mGrad = q-optParams->qInit;
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  return (0.5*pow((q-optParams->qInit).norm(), 2));
}

// TODO: Need to change
double comSimpleConstraint(const std::vector<double> &x, std::vector<double> &grad, void *com_const_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(com_const_data);
  Eigen::Matrix<double, 25, 1> q(x.data());
  optParams->robot->setPositions(q);
  return (pow(optParams->robot->getCOM()(0)-optParams->robot->getPosition(3), 2) \
    + pow(optParams->robot->getCOM()(1)-optParams->robot->getPosition(4), 2));
}

// TODO: Need to add range bounds on qBase


int randomSimpleOptPosesToFile(int numPoses);
double fRand(double min, double max);

int main() {

    //INPUT on below line (tolerance level (in meters) for xCOM value from opt code)
    double tolerance = 0.001;

    //INPUT on below line (number of poses to generate)
    int numPoses = 50;

    randomSimpleOptPosesToFile(numPoses);

}

int randomSimpleOptPosesToFile(int numPoses) {

    ofstream toleranceSimpleBalancedPosesFile;
    std::string nameP = "simpleRandomOptPoses";
    std::string ext = ".txt";
    toleranceSimpleBalancedPosesFile.open(nameP + std::to_string(numPoses) + ext);

    ofstream xCOMValuesFile;
    std::string nameX = "simpleOptXCOMValues";
    xCOMValuesFile.open(nameX + std::to_string(numPoses) + ext);

    double g = 0; //qBase
    double j = 0; //qKinect

    //exact value of pi or is 3.14 fine? eh prob good enough

    double pi = 3.14;
    double bendLimit = 2.0944;

    double lowerLimit[] = {0, -1.57, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi};
    double upperLimit[] = {2.88, 1.57, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi};

    // Instantiate "fixed wheel krang" robot
    dart::utils::DartLoader loader;
    // INPUT on below line (absolute path of the Krang URDF file)
    dart::dynamics::SkeletonPtr fixedWheelKrang = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/KrangFixedWheels/krang_fixed_wheel.urdf");

    //TODO: Need to figure out the format of the URDF file so that I can create
    //an appropriate vector (Assuming format as shown in the beginning of the
    //file)

    cout << "Creating and writing balanced poses ...\n";

    int pose = 0;
    while (pose < numPoses) {

        // Create random pose vector
        //
        Eigen::MatrixXd randomPoseParams(18, 1);

        // Write the default values first
        randomPoseParams(0, 0) = g;

        int index = 1;

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
        const int dof = (const int) fixedWheelKrang->getNumDofs();
        comOptParams optParams;
        optParams.robot = fixedWheelKrang;

        // Getting Eigen::Matrix dimensionality error here
        optParams.qInit << randomPoseParams;
        // The specific algorithm to use (COBYLA: Constrained Optimization BY
        // Linear Approximations
        nlopt::opt opt(nlopt::LN_COBYLA, dof);
        std::vector<double> unoptDartPoseParams(dof);
        double minf;

        //TODO: Need to edit the comSimpleOptFunc
        opt.set_min_objective(comSimpleOptFunc, &optParams);

        //TODO: Need to edit the xCOM constraint equal to zero
        opt.add_equality_constraint(comSimpleConstraint, &optParams, 1e-8);

        //TODO: Need to add the range bounds on qBase

        //Set relative tolerance on opt params
        opt.set_xtol_rel(1e-4);
        //Set max time allocation for optimizing
        opt.set_maxtime(10);

        try {
            opt.optimize(unoptDartPoseParams, minf);
            Eigen::Matrix<double, 18, 1> optDartPoseParams(unoptDartPoseParams.data());
            // Set position of ideal robot to the pose in DART format
            fixedWheelKrang->setPositions(optDartPoseParams);

            // Print the optDartPoseParams that obey the tolerance level specified

            // Get x center of mass
            double xCOMFixedWheelKrang = fixedWheelKrang->getCOM()(0);

            // If it passes the tolerance check then print it to file and increment pose
            // counter
            // Don't include tolerance check right now (make sure the simple_opt
            // works)
            //if (abs(xCOMIdealRobot) < tolerance) {
            toleranceSimpleBalancedPosesFile << optDartPoseParams.transpose() << "\n";

            //Print the xCOM to check the actual values
            xCOMValuesFile << xCOMFixedWheelKrang << "\n";
            pose++;
            //}
        } catch(std::exception &e) {
        }

    }

    // Close the tolerance balanced poses file
    toleranceSimpleBalancedPosesFile.close();

    // Close the xCOM values file
    xCOMValuesFile.close();

    cout << "|-> Done\n";

    return 0;
}

double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
