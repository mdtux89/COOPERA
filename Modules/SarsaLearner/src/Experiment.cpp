/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#include <Experiment.h>

LearnerExperiment::LearnerExperiment()
{
    Json::Value cnfg = Config::instance()->root["Experiment"];
    m_trainEpisodes = cnfg["TrainEpisodes"].asInt();
    m_testEpisodes = cnfg["TestEpisodes"].asInt();
    m_trainRuns = cnfg["TrainRuns"].asInt();
    m_testRuns = cnfg["TestRuns"].asInt();
    m_averagedOver = cnfg["AveragedOver"].asInt();

}

LearnerExperiment::~LearnerExperiment()
{
    std::cout<<"Done with destruction of experiment.\n";
}

/**
 * Initialize the learner thread
 * @return true after the thread is properly initialized
 */
bool LearnerExperiment::threadInit()
{

    learner = new LearnerThread();
    learner->threadInit();

    return true;
}

/**
 * Run a number of episodes
 * @param episodes number of episodes
 * @param learn true if training, false if testing
 */
void LearnerExperiment::OneRun(int episodes, bool learn)
{
    if(learn)
        learner->InitializeLearner();

    int m =0;
    do
    {
        learner->run(learn);
        m = learner->GetEpisodeCount();        

    }while(m < episodes);

}

/**
 * Call several times runOne() for training first, then for testing
 */
void LearnerExperiment::RunAll()
{
    for(int i=0; i < m_trainRuns; ++i)
        OneRun(m_trainEpisodes,true);

    for(int i=0; i < m_testRuns; ++i)
        OneRun(m_testEpisodes,false);

    learner->SendQtable();

}

/**
 * Run the learner experiment
 */
void LearnerExperiment::run()
{
    for(int trials=0; trials<Config::instance()->root["Experiment"]["Trials"].asInt();++trials)
    {
        RunAll();
        // copy the data
        std::stringstream copyCmd;
        copyCmd << "cp ../resources/data/qT.dat ../resources/data/qT_" << trials
                << ".dat\n";
        system(copyCmd.str().c_str());
    }

}

/**
 * Release the learner thread
 */
void LearnerExperiment::threadRelease()
{
    std::cout << "Releasing experiment thread...\n";
    // Finally stop learner  // Will leak memory if interupted!
    learner->threadRelease();

    // Free allocations
    delete learner;
}
