/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#include "SarsaLearner.h"

using namespace yarp::os;
using namespace yarp::dev;

int main()
{

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return -1;
    }

    SarsaLearner mod;
    ResourceFinder rf;
    return mod.runModule(rf);
}



