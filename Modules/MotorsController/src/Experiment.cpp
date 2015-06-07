/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#include <Experiment.h>

Experiment::Experiment(vector<int> initial)
{
    //read paramters
    qTableName = Config::instance()->root["Learn"]["AfterLearn"]["QFile"].asString();
    Json::Value cnfg = Config::instance()->root["Experiment"];
    m_testEpisodes = cnfg["TestEpisodes"].asInt();
    m_testRuns = cnfg["TestRuns"].asInt();
    learner = new SarsaAlgo(initial);
    learner->threadInit();
}

Experiment::~Experiment()
{
    delete learner;
}

/**
 * Run the experiment and put in vector the policy generated for
 * the robot to follow
 */
void Experiment::run()
{
    for(int i=0; i < m_testRuns; ++i)
    {
        if(!learner->InitializeLearner(qTableName.c_str()))
            break;

        int m =0;
        do
        {
            learner->run();
            plan.push_back(learner->getNext());
            m = learner->GetEpisodeCount();

        }while(m < m_testEpisodes);
    }

    // Finally stop learner thread
    learner->threadRelease();
}

