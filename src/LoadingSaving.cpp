//
//  LoadingSaving.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 27.06.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "LoadingSaving.h"
#include <fstream>
#include <vector>
#include <numeric>

using namespace std;

Matrix4d LoadingSaving::loadProjectionMatrix(std::string filename){

    std::ifstream file(filename,std::ifstream::in);

    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
        return Matrix4d::Zero();

    }

    RowVectorXd p(4);
    Matrix4d mat;

    int i=0;
    while(file >> p(0) >> p(1) >> p(2) >> p(3))
    {
        cout<<p<<endl;
        mat.row(i++)=p;

    }

    cout<<"Loaded "<<i<<" rows from "<<filename<<endl;

    return mat;

}


MatrixXd LoadingSaving::loadXYZ(std::string filename){
    std::ifstream file(filename,std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
        return MatrixXd::Zero(1, 6);

    }
    
    vector<RowVectorXd> vec;
    
    RowVectorXd p(6);
    
    int i=0;
    while(file >> p(0) >> p(1) >> p(2) >> p(3) >> p(4) >> p(5))
    {
        i++;
        //cout<<p<<endl;
        vec.push_back(p);
    }
    
    
    MatrixXd mat(i,6);
    int j=0;
    for (auto it : vec) {
        //cout<<it<<endl;
        mat.row(j++)=it;
    }
    
    cout<<"Loaded "<<i<<" pts from "<<filename<<endl;

    return mat;
    
}

void LoadingSaving::saveXYZ(std::string filename, MatrixXd mat){
    
        std::ofstream outputFile(filename, std::ofstream::out) ;
    
        long rows=mat.rows();
        long cols=mat.cols();
        //cout<<"saving "<<rows<<"x"<<cols<<" matrix"<<endl;
    
        int i=0;
        for(; i<rows; i++)
        {
            for(int j=0; j<cols; j++)
            {
                outputFile << mat(i,j) <<" ";
            }
            outputFile<< std::endl;
        }
    
        outputFile.close( );
    
    cout<<"Wrote "<<i<<" pts to "<<filename<<endl;
}

void LoadingSaving::saveVector(std::string filename, vector<double> vec){
    
    std::ofstream outputFile(filename, std::ofstream::out) ;
    
    for (auto it : vec){
        outputFile << it <<endl;
    }
    
    outputFile.close( );
    
    cout<<"Wrote "<<vec.size()<<" numbers to "<<filename<<endl;
}

//http://stackoverflow.com/questions/7616511/calculate-mean-and-standard-deviation-from-a-vector-of-samples-in-c-using-boos

void LoadingSaving::summary(std::vector<double> v){
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
