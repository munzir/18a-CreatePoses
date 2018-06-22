// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To generate a set of poses in which the xCOM is within a certain
// tolerance level

// Output file format for fixed wheel (DART Format)
// qBase, qWaist, qTorso, qKinect,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// The real xCOM for fixed wheel krang is getLocalCOM(2) z (2)

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

// Structs and functions for optimizing Fixed Wheel Krang Model
struct comOptParams {
  SkeletonPtr robot;
  Eigen::Matrix<double, 18, 1> qInit;
};

struct inequalityOptParams {
    Eigen::MatrixXd P;
    Eigen::VectorXd b;
};

//Add inequality constraints on qBase
void constraintFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data) {

    inequalityOptParams* constParams = reinterpret_cast<inequalityOptParams *>(f_data);
    //cout << "done reading inequalityOptParams\n";

    if (grad != NULL) {
        for(int i=0; i<m; i++) {
            for(int j=0; j<n; j++){
                grad[i*n+j] = constParams->P(i, j);
            }
        }
    }
    //cout << "done with gradient\n";


    Eigen::Matrix<double, 1, 1> X;
    for(size_t i=0; i<n; i++) X(i) = x[i];
    //cout << "done reading x\n";

    Eigen::VectorXd mResult;
    mResult = constParams->P*X - constParams->b;
    for(size_t i=0; i<m; i++) {
        result[i] = mResult(i);
    }
    //cout << "done calculating the result\n";
}

// Objective Function
double comSimpleOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(my_func_data);
  Eigen::Matrix<double, 1, 1> q1(x.data());
  Eigen::Matrix<double, 18, 1> q;
  q << q1, optParams->qInit.tail(17);

  optParams->robot->setPositions(q);

  return optParams->robot->getCOM()(2);

}

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
    string nameP = "simpleRandomOptPoses";
    string ext = ".txt";
    toleranceSimpleBalancedPosesFile.open(nameP + to_string(numPoses) + ext);

    ofstream xCOMValuesFile;
    string nameX = "simpleOptXCOMValues";
    xCOMValuesFile.open(nameX + to_string(numPoses) + ext);

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

        // Run random pose through opt
        //
        const int dof = (const int) fixedWheelKrang->getNumDofs();
        comOptParams optParams;
        optParams.robot = fixedWheelKrang;

        optParams.qInit << randomPoseParams;
        // The specific algorithm to use (COBYLA: Constrained Optimization BY
        // Linear Approximations
        nlopt::opt opt(nlopt::LN_COBYLA, 1);
        vector<double> unoptDartPoseParams(1);
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

        opt.add_inequality_mconstraint(constraintFunc, &inequalityconstraintParams, inequalityconstraintTol);

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
            double xCOMFixedWheelKrang = fixedWheelKrang->getCOM()(2);

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
        } catch(exception &e) {
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
