/*
 * Copyright (C) 2013 Marco Damonte
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * Author: Marco Damonte - mdtux89@gmail.com (University of Genoa, Italy), 2013
 * Based on work of Shashank Pathak for Reinforcement Learning algorithm
 */

#ifndef SAMPLER_H
#define SAMPLER_H
#define DEFAULT_GAUSS_SEED 13

#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/normal_distribution.hpp>

#include <algorithm>
#include <iostream>
using namespace std;

/**
 * Utilities for random numbers generation (gaussian distribution).
 */
class GaussianSampler
{
protected:
    // Gaussian
    typedef boost::minstd_rand base_generator_type;
    base_generator_type genGaussian;
    boost::random::normal_distribution<> randGauss;
    boost::variate_generator<base_generator_type&, boost::random::normal_distribution<> >* getGaussian;

    double m_mean;
    double m_sigma;

public:
    GaussianSampler(int seed = DEFAULT_GAUSS_SEED) : genGaussian(seed),randGauss(0.0,1.0)
    {
        getGaussian = new boost::variate_generator<base_generator_type&, boost::random::normal_distribution<> >(genGaussian,randGauss);
        m_mean = 0;
        m_sigma = 1.0;
    }

    GaussianSampler(double sigma, double mean = 0, int seed = DEFAULT_GAUSS_SEED) : genGaussian(seed),randGauss(0.0,1.0)
    {
        getGaussian = new boost::variate_generator<base_generator_type&, boost::random::normal_distribution<> >(genGaussian,randGauss);
        m_mean = mean;
        m_sigma = sigma;
    }

    ~GaussianSampler()
    {
        delete getGaussian;
    }

    double sample()
    {
        return (m_mean + m_sigma * ((*getGaussian)()));
    }

};

#define DEFAULT_UNIFORM_SEED 13

/**
 * Utilities for random numbers generation (uniform distribution).
 */
class UniformSampler
{
public:
    int n_samples;
protected:
    typedef boost::minstd_rand base_generator_type;
    // Uniform
    base_generator_type genUniRand01;
    boost::uniform_real<> randReal01;
    boost::variate_generator<base_generator_type&, boost::uniform_real<> >* getRand01;

    double m_min;
    double m_range;

public:
    UniformSampler(int seed = DEFAULT_UNIFORM_SEED) : genUniRand01(seed),randReal01(0,1)
    {
        getRand01 = new boost::variate_generator<base_generator_type&, boost::uniform_real<> >(genUniRand01,randReal01);
        m_min = 0;
        m_range = 1.0;
        n_samples = 0;
    }

    UniformSampler(double range, double minimum = 0, int seed = DEFAULT_UNIFORM_SEED) : genUniRand01(seed),randReal01(0,1)
    {
        getRand01 = new boost::variate_generator<base_generator_type&, boost::uniform_real<> >(genUniRand01,randReal01);
        m_min = minimum;
        m_range = range;
        n_samples = 0;
    }

    ~UniformSampler()
    {
        delete getRand01;
    }

    double sample(bool incr = true)
    {
        if(incr) n_samples++;
        return m_min + m_range *(*getRand01)();
    }

};


#endif // SAMPLER_H
