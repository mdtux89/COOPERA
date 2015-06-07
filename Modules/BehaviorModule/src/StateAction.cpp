/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 */

#include "StateAction.h"

map<int, vector<int>> State::id2values;
map<vector<int>, int> State::values2id;
int State::counter = 6;
