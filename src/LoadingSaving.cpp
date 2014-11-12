//
//  LoadingSaving.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 27.06.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "LoadingSaving.h"
#include <fstream>
#include <numeric>
#include <iostream>

namespace LoadingSaving{

template<typename Number>
Matrix<Number,Dynamic,Dynamic> loadMatrix(std::string filename, std::string type){
    std::ifstream file(filename,std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
        return Matrix<Number,Dynamic,Dynamic>::Zero(1, 1);

    }


    std::string line;
    std::getline(file,line);
    std::stringstream lineStream(line);

    int cols=0;
    Number n;
    while(lineStream >> n)
    {
        cols++;
    }
    if(cols==0){
        return Matrix<Number,Dynamic,Dynamic>::Zero(1, 1);
    }

    vector< Matrix<Number,1,Dynamic> > vec;


    int rows=0;

    do{
        rows++;
        std::stringstream lineStream2(line);
        Matrix<Number,1,Dynamic> row(cols);
        int col = 0;
        while(lineStream2 >> n)
        {
            row(col)=n;
            col++;
        }

        vec.push_back(row);
    }while(std::getline(file,line));


    Matrix<Number,Dynamic,Dynamic> mat(rows,cols);
    int j=0;
    for (auto it : vec) {
        //cout<<it<<endl;
        mat.row(j++)=it;
    }

    //cout<<"Loaded MatrixX"<<type<<"("<<rows<<","<<cols<<") from "<<filename<<endl;

    return mat;
}

Matrix4f loadMatrix4f(std::string filename){
    std::ifstream file(filename,std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
        return Matrix4f::Zero();

    }
    float array[16];
    int i=0;
    while(file >> array[i++]){}

    //cout<<"Loaded Matrix4f from "<<filename<<endl;
    return Map< Matrix<float,4,4,RowMajor> > (array);
}

MatrixXd loadMatrixXd(std::string filename){
    return loadMatrix<double>(filename,"d");
}

MatrixXf loadMatrixXf(std::string filename){
    return loadMatrix<float>(filename,"f");
}

MatrixXi loadMatrixXi(std::string filename){
    return loadMatrix<int>(filename,"i");
}


template<typename Number>
void saveMatrix(std::string filename, Matrix<Number,Dynamic,Dynamic> mat, string type){
    std::ofstream outputFile(filename, std::ofstream::out) ;
    outputFile << mat.format(FullPrecision);
    outputFile.close();

    //cout<<"Saved MatrixX"<<type<<"("<<mat.rows()<<","<<mat.cols()<<") to "<<filename<<endl;

}

void saveMatrix4f(std::string filename, Matrix4f mat){
    std::ofstream outputFile(filename, std::ofstream::out) ;
    outputFile << mat.format(FullPrecision);
    outputFile.close();

    //cout<<"Saved Matrix4f to "<<filename<<endl;
}

void saveMatrixXd(std::string filename, MatrixXd mat){return saveMatrix<double>(filename,mat,"d");}
void saveMatrixXf(std::string filename, MatrixXf mat){return saveMatrix<float>(filename,mat,"f");}
void saveMatrixXi(std::string filename, MatrixXi mat){return saveMatrix<int>(filename,mat,"i");}

void saveVector(std::string filename, vector<double> vec){
    
    std::ofstream outputFile(filename, std::ofstream::out) ;
    
    for (auto it : vec){
        outputFile << it <<endl;
    }
    
    outputFile.close( );
    
    cout<<"Wrote "<<vec.size()<<" numbers to "<<filename<<endl;
}

} // end ns
