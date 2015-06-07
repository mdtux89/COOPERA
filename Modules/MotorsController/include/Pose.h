/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef POSE_H
#define POSE_H

#include <shark/Data/Csv.h>
#include <shark/Algorithms/Trainers/LDA.h>
#include <string>
#include <vector>
#include <SharedInfo.h>
#define MINACTION 10
using namespace std;

/**
 * Utilities used to handle the robot postures.
 * The poses considered are: facedow, faceup, sideright and sideleft.
 */
class Pose {
private:
    shark::LinearClassifier<> lda;
    SharedInfo shared;
    void copy(ifstream *ifile);

public:
    string pose;
    void test();
    Pose(string trainsetFile, SharedInfo shared);
    void classify(vector<int> v);
    string getPose();
};

#endif
