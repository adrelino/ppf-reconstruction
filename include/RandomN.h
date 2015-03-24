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

namespace RandomN{

void hist(){
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

//http://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
#define TWO_PI 6.2831853071795864769252866
double generateGaussianNoise(const double &variance)
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

void meanAndVar(vector<double> vals){
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

VectorXf RandomNGet(int n){
    VectorXf vec(n);
    for (int i = 0; i < n; ++i) {
        vec[i]=generateGaussianNoise(1);
    }
    return vec;
}

/////Represents the exception for taking the median of an empty list
//class median_of_empty_list_exception:public std::exception{
//  virtual const char* what() const throw() {
//    return "Attempt to take the median of an empty list of numbers.  "
//      "The median of an empty list is undefined.";
//  }
//};

/////Return the median of a sequence of numbers defined by the random
/////access iterators begin and end.  The sequence must not be empty
/////(median is undefined for an empty set).
/////
/////The numbers must be convertible to double.
//template<class RandAccessIter>
//double median(RandAccessIter begin, RandAccessIter end)
//  throw(median_of_empty_list_exception){
//  if(begin == end){ throw median_of_empty_list_exception(); }
//  std::size_t size = end - begin;
//  std::size_t middleIdx = size/2;
//  RandAccessIter target = begin + middleIdx;
//  std::nth_element(begin, target, end);

//  if(size % 2 != 0){ //Odd number of elements
//    return *target;
//  }else{            //Even number of elements
//    double a = *target;
//    RandAccessIter targetNeighbor= target-1;
//    std::nth_element(begin, targetNeighbor, end);
//    return (a+*targetNeighbor)/2.0;
//  }
//}

//http://stackoverflow.com/questions/7616511/calculate-mean-and-standard-deviation-from-a-vector-of-samples-in-c-using-boos

void summary(std::vector<double> v){  //Like R's summary
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double std = std::sqrt(sq_sum / v.size() - mean * mean);

    std::sort(v.begin(),v.end());
    double min=v[0];
    double firstQuantile=v[v.size()*0.25];
    double median=v[v.size()*0.5];
    double thirdQuantile=v[v.size()*0.75];
    double max=v[v.size()-1];

    cout<<"Summary of "<<v.size()<<" bucket sizes:"<<endl;
    cout<<"Min\t.25\tMed\tMean\t.75\tMax \tStd"<<endl;
    cout<<min<<" \t"<<firstQuantile<<" \t"<<median<<" \t"<<round(mean*100)*0.01<<" \t"<<thirdQuantile<<" \t"<<max<<" \t"<<round(std*100)*0.01<<endl;





    /*std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
                   std::bind2nd(std::minus<double>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size());


    double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
    double m =  sum / v.size();

    double accum = 0.0;
    std::for_each (std::begin(v), std::end(v), [&](const double d) {
        accum += (d - m) * (d - m);
    });

    double stdev = sqrt(accum / (v.size()-1));*/

}

} // end namespace

#endif // RANDOMN_H
