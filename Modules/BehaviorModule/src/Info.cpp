/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#include "Evolution.h"
#include "Controller.h"

int Info::nParts = 0;
vector<int>* Info::nJoints = new vector<int>;
int Info::nTotalJoints = 0;
PolyDriver* Info::drivers = new PolyDriver[nParts];
map<string,int>* Info::starts = new map<string, int>;
map<string,int>* Info::parts = new map<string, int>;
yarp::os::BufferedPort<Bottle>* Info::sensorPort = new yarp::os::BufferedPort<Bottle>;
int Info::velocity = 0;
int Info::tiltIndex = 0;
int Info::motionWait = 0;
int Info::macroWait = 0;
string Info::pose = "";
bool Info::poseEstimation = false;

vector<int> Guardian::lastState;
yarp::os::Semaphore Guardian::lock;
bool Guardian::overload = false;
