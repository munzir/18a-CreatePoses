// Author Akash Patel (apatel435@gatech.edu)
// Purpose: Generate ALL (feasible and infeasible) angular poses for Krang
// Eventually generate base set of feasible poses

// Written file format
// heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6
// Use alphabet scheme for ease of variable use
// What we change: qWaist, qTorso, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
// h, i, k, l, m, n, o, p, q, r, s, t, u, v, w, x

// Limits of single joints conflicting with parent joint found via inspection in
// DART
// TODO: These limits can be mathematically found knowning the width of the link
// IDEA: For finding limits of 3 links the upper limit would be the requirements
// of a triangle, for 4 a quadilateral, 5 a pentagon etc.
// negative value (-) is clockwise from parent axis
// Basically need to determine the base set of poses that are also safe
// (non-colliding)

// Below values obtained from hardware init script on Krang: https://github.gatech.edu/WholeBodyControlAttempt1/Krang-Software/blob/master/project/krang/util/scripts/initd/krang

// TORSO & Waist
// qWaist = [0, 2.88]
// qTorso = [-1.57, 1.57]

// From visual inspection of Krang URDF sim in DART bending is [-2.2, 2,2]

// LEFT ARM
// qLArm6 = [-pi, pi] (rotational)
// qLArm5 = [-2.0944, 2.0944] (bending)
// qLArm4 = [-pi, pi] (rotational)
// qLArm3 = [-2.0944, 2.0944] (bending)
// qLArm2 = [-pi, pi] (rotational)
// qLArm1 = [-2.0944, 2.0944] (bending)
// qLArm0 = [-pi, pi] (rotational)

// RIGHT ARM
// qRArm6 = [-pi, pi] (rotational)
// qRArm5 = [-2.0944, 2.0944] (bending)
// qRArm4 = [-pi, pi] (rotational)
// qRArm3 = [-2.0944, 2.0944] (bending)
// qRArm2 = [-pi, pi] (rotational)
// qRArm1 = [-2.0944, 2.0944] (bending)
// qRArm0 = [-pi, pi] (rotational)

// What values to keep constant and at what values
// heading = a = ?
// qBase = b = ?
// x = c = ?
// y = d = ?
// z = e = ?
// qLWheel = f = ?
// qRWheel = g = ?
// qKinect = j = ?

#include <iostream>
#include <fstream>

using namespace std;

int writePosesToFile();
int krangHardwarePoses();
int writeRandomPosesToFile(int numPoses);
double fRand(double min, double max);

int main() {
    //writeAllPosesToFile();
    srand(0);
    krangHardwarePoses();
    //writeRandomPosesToFile(5000);
}

int krangHardwarePoses() {
    // Step sizes
    // Waist - 3
    // Torso - 4
    // Left Shoulder - 4
    // Right Shoulder - 4
    //
    // Poses for Rest of the Arms
    // Random pose for each one

    double pi = 3.14;
    double bendLimit = 2.0944;

    double lowerLimit[] = {0, -1.57, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi};
    double upperLimit[] = {2.88, 1.57, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi};

    // step size of the joint positions
    int step[] = {3, 4, 4, 4};

    //Number of arm poses for each step of waist, torso, & left/right shoulders
    int armPoses = 6;

    double a = 0; //heading
    double b = 0; //qBase
    double c = 0; //x
    double d = 0; //y
    double e = 0; //z
    double f = 0; //qLWheel
    double g = 0; //qRWheel
    double j = 0; //qKinect

    ofstream myfile;
    myfile.open("krangHardwarePoses.txt");

    for (double h = lowerLimit[0] ; h < upperLimit[0];  h += (upperLimit[0]  - lowerLimit[0] ) / step[0] ) {
    for (double i = lowerLimit[1] ; i < upperLimit[1];  i += (upperLimit[1]  - lowerLimit[1] ) / step[1] ) {
    for (double k = lowerLimit[2] ; k < upperLimit[2];  k += (upperLimit[2]  - lowerLimit[2] ) / step[2] ) {
    for (double r = lowerLimit[2] ; r < upperLimit[2];  r += (upperLimit[2]  - lowerLimit[2] ) / step[2] ) {
    for (int jj = 0; jj < armPoses; jj++) {
        // Write the default values first
        myfile << a << " " << b << " " << c << " " << d << " " << e << " " << f << " " << g << " ";

        // Write waist and torso
        myfile << h << " " << i << " ";

        // Write qKinect
        myfile << j;

        // Write left arm values (shoulder first)
        myfile << " " << k;

        int ii = 3;
        //Write the rest of the values
        for (;ii < 6+3; ii++) {
            double ard = fRand(lowerLimit[ii], upperLimit[ii]);
            myfile << " " << ard;
        }

        // Write right arm values (shoulder first)
        ii = ii + 1;
        myfile << " " << r;

        //Write the rest of the values
        for (;ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
            double ard = fRand(lowerLimit[ii], upperLimit[ii]);
            myfile << " " << ard;
        }

        myfile << "\n";

    }}}}}

    myfile.close();
    return 0;
}

int writeAllPosesToFile () {

    ofstream myfile;
    myfile.open("allPosesStep2.txt");

    double a = 0; //heading
    double b = 0; //qBase
    double c = 0; //x
    double d = 0; //y
    double e = 0; //z
    double f = 0; //qLWheel
    double g = 0; //qRWheel
    double j = 0; //qKinect

    //make step and limit arrays so that each joint can be varied independently
    //exact value of pi or is 3.14 fine? eh prob good enough

    double pi = 3.14;
    double bendLimit = 2.0944;

    double lowerLimit[] = {0, -1.57, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi};
    double upperLimit[] = {2.88, 1.57, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi};
    // step size of the joint positions
    int stepAll = 2;
    int step[] = {stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll, stepAll};

    for (double h = lowerLimit[0] ; h < upperLimit[0];  h += (upperLimit[0]  - lowerLimit[0] ) / step[0] ) {
    for (double i = lowerLimit[1] ; i < upperLimit[1];  i += (upperLimit[1]  - lowerLimit[1] ) / step[1] ) {
    for (double k = lowerLimit[2] ; k < upperLimit[2];  k += (upperLimit[2]  - lowerLimit[2] ) / step[2] ) {
    for (double l = lowerLimit[3] ; l < upperLimit[3];  l += (upperLimit[3]  - lowerLimit[3] ) / step[3] ) {
    for (double m = lowerLimit[4] ; m < upperLimit[4];  m += (upperLimit[4]  - lowerLimit[4] ) / step[4] ) {
    for (double n = lowerLimit[5] ; n < upperLimit[5];  n += (upperLimit[5]  - lowerLimit[5] ) / step[5] ) {
    for (double o = lowerLimit[6] ; o < upperLimit[6];  o += (upperLimit[6]  - lowerLimit[6] ) / step[6] ) {
    for (double p = lowerLimit[7] ; p < upperLimit[7];  p += (upperLimit[7]  - lowerLimit[7] ) / step[7] ) {
    for (double q = lowerLimit[8] ; q < upperLimit[8];  q += (upperLimit[8]  - lowerLimit[8] ) / step[8] ) {
    for (double r = lowerLimit[9] ; r < upperLimit[9];  r += (upperLimit[9]  - lowerLimit[9] ) / step[9] ) {
    for (double s = lowerLimit[10]; s < upperLimit[10]; s += (upperLimit[10] - lowerLimit[10]) / step[10]) {
    for (double t = lowerLimit[11]; t < upperLimit[11]; t += (upperLimit[11] - lowerLimit[11]) / step[11]) {
    for (double u = lowerLimit[12]; u < upperLimit[12]; u += (upperLimit[12] - lowerLimit[12]) / step[12]) {
    for (double v = lowerLimit[13]; v < upperLimit[13]; v += (upperLimit[13] - lowerLimit[13]) / step[13]) {
    for (double w = lowerLimit[14]; w < upperLimit[14]; w += (upperLimit[14] - lowerLimit[14]) / step[14]) {
    for (double x = lowerLimit[15]; x < upperLimit[15]; x += (upperLimit[15] - lowerLimit[15]) / step[15]) {

        myfile << a << " " << b << " " << c << " " << d << " " << e << " " << f << " " << g << " " << h << " " << i << " " << j << " " << k << " " << l << " " << m << " " << n << " " << o << " " << p << " " << q << " " << r << " " << s << " " << t << " " << u << " " << v << " " << w << " " << x << "\n";
    }}}}}}}}}}}}}}}}

    myfile.close();
    return 0;
}

int writeRandomPosesToFile (int numPoses) {

    ofstream myfile;
    std::string name = "randomPoses";
    std::string ext = ".txt";
    myfile.open(name + std::to_string(numPoses) + ext);

    double a = 0; //heading
    double b = 0; //qBase
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


    // Write numPoses random poses
    for (int pose = 0; pose < numPoses; pose++) {
        // Write the default values first
        myfile << a << " " << b << " " << c << " " << d << " " << e << " " << f << " " << g << " ";

        // Loop through writing the rest of the values
        int ii = 0;
        for (;ii < 2; ii++) {
            double ard = fRand(lowerLimit[ii], upperLimit[ii]);
            myfile << ard << " ";

        }
        // Write qKinect
        myfile << j;

        //Write the rest of the values
        for (;ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
            double ard = fRand(lowerLimit[ii], upperLimit[ii]);
            myfile << " " << ard;
        }

        myfile << "\n";

    }

    myfile.close();
    return 0;
}

double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
