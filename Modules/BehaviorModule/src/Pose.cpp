/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */
#include <Pose.h>
int est;

Pose::Pose(string trainsetFile){
    this->pose = "UNKNOWN";
    this->nPoses = N;
    shark::ClassificationDataset data;
    try {
        shark::import_csv(data, trainsetFile, shark::LAST_COLUMN, ' ');
    }
    catch (...) {
        cerr << "unable to read data from file: " <<  trainsetFile << endl;
    }

    // define learning algorithm
    shark::LDA ldaTrainer;

    // train model
    ldaTrainer.train(lda, data);
}

/**
 * Read the torques used to perform pose estimation
 * @return detected torques
 */
vector<int> getData(){

   int waitTime = Info::motionWait/2;

   vector<int> v;
   for(int i=0;i<Info::nParts;i++){ //for each robot parts

       //create the IPositionControl and ITorqueControl interfaces
       IPositionControl *position;
       ITorqueControl *torque;
       if (!Info::drivers[i].view(position) || !Info::drivers[i].view(torque)) {
         break;
       }

       // two movements will be performed while getting torques
       double t1,t2;
       for(int j=0;j<(*Info::nJoints)[i];j++){
           //first movement
           position->setRefSpeed(j,Info::velocity);
           position->relativeMove(j,MOTIONRANGE);
           for(int k=0; k<waitTime*20;k++){
               torque->getTorque(j,&t1);
               Time::delay(0.05);
               if(abs(t1)>TORQUELIMIT){
                   position->stop(j);
                   if(t1 > 0) t1 = TORQUELIMIT;
                   else t1 = -TORQUELIMIT;
                   break;
               }
           }

           //back to initial position
           position->relativeMove(j,-MOTIONRANGE);
           Time::delay(0.1);

           //second movement
           position->relativeMove(j,-MOTIONRANGE);
           for(int k=0; k<waitTime*20;k++){
               torque->getTorque(j,&t2);
               Time::delay(0.05);
               if(abs(t2)>TORQUELIMIT){
                   position->stop(j);
                   if(t2 > 0) t2 = TORQUELIMIT;
                   else t2 = -TORQUELIMIT;
                   break;
               }
           }

           //back to initial position
           position->relativeMove(j,MOTIONRANGE);
           Time::delay(0.1);

           //save values
           v.push_back(t1);
           v.push_back(t2);
       }
  }
  return v;
}

void testBody(string pose){

    string read;
    ifstream *inFile = new ifstream;
    int waitTime = Info::motionWait/2;

    inFile->open("../resources/data/check/"+pose+".txt");
    while(getline(*inFile,read)){ //until file ends
        if(read == "--"){ // end of action
            continue;
        }
        stringstream ss(read);

        //split the line in part of the robot, joint id and polarity of the action
        string part,s_id,s_pol;
        getline(ss,part,',');
        getline(ss,s_id,',');
        getline(ss,s_pol,',');
        int id = atoi(s_id.c_str());
        int pol = atoi(s_pol.c_str());

        //create the IPositionControl interface
        ITorqueControl *torque;
        IPositionControl *position;
        if (!Info::drivers[(*Info::parts)[part]].view(position) ||
                !Info::drivers[(*Info::parts)[part]].view(torque)) {
          return;
        }

        position->setRefSpeed(id,Info::velocity);
        position->relativeMove(id,pol);
        for(int k=0; k<waitTime*10;k++){
            double t;
            torque->getTorque(id,&t);
            Time::delay(0.1);
            if(abs(t)>TORQUELIMITTEST){
                position->stop(id);
                inFile->close();
                pose = "UNKNOWN";
                return;
            }
        }
    }

    inFile->close();
    return;
}

/**
 * Test whether the believed pose is the actual one
 */
void Pose::test(){
    if (pose != "UNKNOWN") {
        boost::thread api_caller(::testBody, pose);
        if (!api_caller.timed_join(boost::posix_time::seconds(TIMEOUT_SHORT))) {
            pose = "UNKNOWN";
        }
    }
}

void classifierBody(shark::LinearClassifier<> lda){
    vector<int> v = getData();
    ofstream o("../resources/data/predict.csv");
    for(int i=0;i<v.size();i++){
        o << v[i] << " ";
        if(i==(v.size()-1)){
            o << 0;
        }
    }
    o << endl;
    o.close();
    shark::ClassificationDataset test;
    shark::import_csv(test, "../resources/data/predict.csv", shark::LAST_COLUMN, ' ');

    // predict pose
    shark::Data<unsigned int> prediction;
    prediction = lda(test.inputs());
    est = prediction.elements().front();
}

/**
 * Perform the pose estimation by means of a linear classifier
 */
void Pose::classify(){
    boost::thread api_caller(::classifierBody, lda);
    if (!api_caller.timed_join(boost::posix_time::seconds(TIMEOUT_LONG))) {
        pose = "UNKNOWN";
    }
    else {
        switch(est){
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
    }
}
