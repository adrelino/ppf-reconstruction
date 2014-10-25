//
//  RandomN.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 20.10.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

//Random normally/gaussian distributed Eigen Matrices

#ifndef RANDOMN_H
#define RANDOMN_H

#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>
#include <eigen3/Eigen/Dense>


//RandomN::hist();
//vector<double> vals;
//for(int i=0; i<10000; i++) vals.push_back(RandomN::generateGaussianNoise(1));
//RandomN::meanAndVar(vals);

class RandomN{
public:
static void hist()
{
    std::random_device rd;
    std::mt19937 gen(rd());

    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<> d(5,2);

    std::map<int, int> hist;
    for(int n=0; n<10000; ++n) {
        ++hist[std::round(d(gen))];
    }
    for(auto p : hist) {
        std::cout << std::fixed << std::setprecision(1) << std::setw(2)
                  << p.first << ' ' << std::string(p.second/200, '*') << '\n';
    }
}

static VectorXd RandomNGet(int n){
    VectorXd vec(n);
    for (int i = 0; i < n; ++i) {
        vec[i]=RandomN::generateGaussianNoise(1);
    }
    return vec;
}

//http://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
#define TWO_PI 6.2831853071795864769252866
static double generateGaussianNoise(const double &variance)
{
    static bool haveSpare = false;
    static double rand1, rand2;

    if(haveSpare)
    {
        haveSpare = false;
        return sqrt(variance * rand1) * sin(rand2);
    }

    haveSpare = true;

    rand1 = rand() / ((double) RAND_MAX);
    if(rand1 < 1e-100) rand1 = 1e-100;
    rand1 = -2 * log(rand1);
    rand2 = (rand() / ((double) RAND_MAX)) * TWO_PI;

    return sqrt(variance * rand1) * cos(rand2);
}

static void meanAndVar(vector<double> vals){
    int n=vals.size();

    double mean=0;
    for(int i=0; i<n;i++){
        mean += vals[i];
    }
    mean /=n;

    double variance=0;
    for(int i=0; i<n;i++){
        double diff = vals[i] - mean;
        variance +=diff*diff;
    }
    variance /=n;

    cout<<"mean= "<<mean<<" var: "<<variance<<endl;
}
};

#endif // RANDOMN_H
