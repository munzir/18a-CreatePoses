# 18a: Create Poses
This project contains multiple scripts to generate different sets of poses to be used for data analysis.

## gen\_poses
A script that creates custom, stepped, and random pose sets. Can balance and
check for collisions. Can also filter existing pose sets.

### Dependencies
- DART (at least version 6) [Dart Homepage](https://dartsim.github.io)

### Build and Run
1: Enter the gen\_poses directory

2: Build the project

    mkdir build
    cd build
    cmake ..
    make

3: Run the project

    ./gen_poses

## convert\_pose\_formats
A script that converts between Munzir's and DART's coordinates.

### Dependencies
Same as gen\_poses

### Build and Run
Same as gen\_poses
