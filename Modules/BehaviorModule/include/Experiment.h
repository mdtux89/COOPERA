/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef LEARNEREXPERIMENT_H
#define LEARNEREXPERIMENT_H

#include <Configuration.h>
#include <Sarsa.h>

/**
 * It prepares and launches the Sarsa RL learner experiment
 */
class LearnerExperiment : public yarp::os::Thread
{
protected:
    int m_trainEpisodes;
    int m_trainRuns;
    int m_testEpisodes;
    int m_testRuns;

    LearnerThread *learner;
    Evolution *evolution;
    Pose* pose;

public:

    LearnerExperiment(Evolution* evolution, Pose* pose, bool learning = true);

    ~LearnerExperiment();

    virtual bool threadInit();

    virtual void threadRelease();

    virtual void run();

    void OneRun(int episodes, bool learn);

    void RunAll();

    /**
     * Get the learner thread
     * @return the learner thread
     */
    LearnerThread* getLearner()
    {
        return (learner);
    }
};

#endif // LEARNEREXPERIMENT_H
