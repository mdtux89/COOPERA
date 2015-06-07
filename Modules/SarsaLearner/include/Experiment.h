/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef LEARNEREXPERIMENT_H
#define LEARNEREXPERIMENT_H

#include <Configuration.h>
#include <StatesActions.h>
#include <Sarsa.h>

/**
 * Sarsa RL learner experiment: it learns the Q-values that SarsaPolicy (or MotorsController) will use
 * to generate a policy.
 * The RL task is defined through a Json configuration file.
 */
class LearnerExperiment : public yarp::os::Thread
{
protected:
    int m_trainEpisodes;
    int m_trainRuns;
    int m_testEpisodes;
    int m_testRuns;
    int m_averagedOver;

    LearnerThread *learner;

public:

    LearnerExperiment();

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
