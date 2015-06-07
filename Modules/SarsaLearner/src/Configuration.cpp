/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#include <Configuration.h>

bool Config::instanceFlag = false;
bool Config::isParsed = false;
Config* Config::single = NULL;

/**
 * Parse the Json configuration file
 * @return pointer to the configuration instance
 */
Config* Config::instance()
{
    if(! instanceFlag)
    {
        single = new Config();
        instanceFlag = true;
    }
    if(!isParsed)
    {
        std::cout<<"Parsing failed!\n";
    }
    return single;
}

/**
 * Check the file
 * @param pFile configuration file
 * @return whether the file is empty or not
 */
bool file_is_empty(std::ifstream& pFile)
{
    return pFile.peek() == std::ifstream::traits_type::eof();
}
