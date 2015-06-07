/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */
#include <Pose.h>

Pose::Pose(string trainsetFile, SharedInfo shared){
    this->pose = "unknown";
    this->shared = shared;
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
 * For each line of the input file, read the encoders
 * and start the threads passing the encoders read. Wait all the threads to finish before
 * continuing on the next input line
 * @param ifile the file used to read the encoders
 */
void Pose::copy(ifstream *ifile){

   while(true){ //until file ends

       string read;
       string tok;
       if(!getline(*ifile,read)){ //read one line
           return;
       }
       stringstream ss(read);
       vector<double> encoders;

       for(int i=0;i<shared.nParts;i++){ //for each robot parts
           encoders.clear();
           //split the line and push the values in a vector
           for(int j=0; j<shared.nJoints[i]; j++){
               getline(ss,tok,',');
               encoders.push_back(atof(tok.c_str()));
           }

           //pass everything to the threads
           shared.threads[i].set(&shared.drivers[i],encoders,shared.velocity);
           shared.threads[i].start();

      }

      //wait threads
      for(int i=0;i<shared.nParts;i++){
          shared.threads[i].join();
      }

      // wait for each motion to terminate
      Time::delay(shared.joinTime);
   }

}

/**
 * Test whether the believed pose is the actual one
 */
void Pose::test(){

    string read;
    double limit = 256;
    ifstream *inFile = new ifstream;

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
        if (!shared.drivers[shared.parts[part]].view(position) || !shared.drivers[shared.parts[part]].view(torque)) {
          return;
        }

        position->setRefSpeed(id,shared.velocity);
        position->relativeMove(id,pol);
        for(int k=0; k<shared.waitTime*10;k++){
            double t;
            torque->getTorque(id,&t);
            Time::delay(0.1);
            if(abs(t)>limit){
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
 * Perform the pose estimation by means of a linear classifier
 * @param v the pose to classify
 */
void Pose::classify(vector<int> v){
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

    switch(prediction.elements().front()){
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
            break;
    }
}
