/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#ifndef POSE_H
#define POSE_H

#include <shark/Data/Csv.h>
#include <shark/Algorithms/Trainers/LDA.h>
#include <string>
#include <vector>
#include "Hierarchy.h"
#include <boost/thread.hpp>

#define MOTIONRANGE 10
#define TORQUELIMIT 400
#define TORQUELIMITTEST 256
#define TIMEOUT_SHORT 60
#define TIMEOUT_LONG 300
#define N 4

using namespace std;

/**
 * Utilities used to handle the robot postures.
 * The poses considered are: facedow, faceup, sideright and sideleft.
 */
class Pose{
private:
    shark::LinearClassifier<> lda;

public:
    int nPoses;
    string pose;
    Pose(string trainsetFile);
    void classify();
    void test();

    /**
     * Utility to map the index pose with the string description of the pose
     * @param index the index pose
     * @return the string description
     */
    static string indexToPose(int index) {
        string pose;
        switch(index){
            case 0:
                pose = "facedown";
                break;
            case 1:
                pose = "faceup";
                break;
            case 2:
                pose = "sideleft";
                break;
            case 3:
                pose = "sideright";
                break;
            default:
                pose = "UNKNOWN";
                break;
        }
        return pose;
    }
};

#endif
