/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef SARSAPOLICY_H
#define SARSAPOLICY_H

#include <Configuration.h>
#include <StatesActions.h>
#include <Sarsa.h>

/**
 * Yarp module used to simulate a Sarsa exploiting experiment.
 * It generates a policy for the robot to follow.
 * The RL task is defined through a Json configuration file and the initial state is
 * passed in the class constructor. The actual Sarsa experiment is done in
 * MotorsController module.
 */
class SarsaPolicy : public yarp::os::Thread
{
protected:
    yarp::sig::Vector resetState, targetState;
    string qTableName;
    SarsaPolicyAlgo *learner;
    vector<vector<int> > plan;
    int m_testEpisodes;
    int m_testRuns;


public:

    SarsaPolicy();

    ~SarsaPolicy();

    virtual void run();

    /**
     * Get the learner thread
     * @return the learner thread
     */
    SarsaPolicyAlgo* getLearner()
    {
        return (learner);
    }

    /**
     * Get the plan generated by the experiment
     * @return the plan
     */
    vector<vector<int> > getPlan(){
        return plan;
    }

};

#endif // POLICYEXPERIMENT_H