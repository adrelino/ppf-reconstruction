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
#include <iomanip>

namespace LoadingSaving{

//pair<vector<Vector3f>,vector<Vector3f>> loadMatrixPointsWithNormals(std::string filename){
//    std::ifstream f(filename,std::ifstream::in);
//    if( f.fail() == true )
//    {
//        cerr << filename << " could not be opened" << endl;
//    }


//    vector<Vector3f> pts;
//    vector<Vector3f> nor;

//    Vector3f pt;
//    Vector3f no;

//    int n=0;
//    while( f >> pt(0) && f >> pt(1) && f >> pt(2) && f >> no(0) && f >> no(1) && f >> no(2)){
//        pts.push_back(pt);
//        nor.push_back(no);
//        n++;
//    }

//    cout<<"Loaded "<<n<< " pts + nor from "<<filename<<endl;

//    return make_pair(pts,nor);
//}

PointCloud loadPLY(const std::string filename, bool withNormals)
{
  PointCloud cloud;
  int numVertices=0;

  std::ifstream ifs(filename.c_str());

  if (!ifs.is_open())
  {
    printf("Cannot open file...\n");
    return cloud;
  }

  std::string str;
  while (str.substr(0, 10) !="end_header")
  {
    std::string entry = str.substr(0, 14);
    if (entry == "element vertex")
    {
      numVertices = atoi(str.substr(15, str.size()-15).c_str());
    }
    std::getline(ifs, str);
  }

  cloud.pts = vector<Vector3f>(numVertices);
  if (withNormals) cloud.nor=vector<Vector3f>(numVertices);


  for (int i = 0; i < numVertices; i++)
  {
    float* data1 = (float*)(&cloud.pts[i]);
    ifs >> data1[0] >> data1[1] >> data1[2];

    if (withNormals)
    {
      float* data = (float*)(&cloud.nor[i]);
      ifs >> data[0] >> data[1] >> data[2];

      // normalize to unit norm
      double norm = sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]);
      if (norm>0.00001)
      {
        data[0]/=(float)norm;
        data[1]/=(float)norm;
        data[2]/=(float)norm;
      }
    }
    else
    {
    }
  }

  //cloud *= 5.0f;
  return cloud;
}

void savePLY(const std::string filename, PointCloud PC)
{
  std::ofstream outFile( filename.c_str() );

  if ( !outFile )
  {
    //cerr << "Error opening output file: " << FileName << "!" << endl;
    printf("Error opening output file: %s!\n", filename.c_str());
    exit( 1 );
  }

  ////
  // Header
  ////

  const int pointNum = ( int ) PC.pts.size();
  //const int vertNum  = ( int ) PC.cols;

  outFile << "ply" << std::endl;
  outFile << "format ascii 1.0" << std::endl;
  outFile << "element vertex " << pointNum << std::endl;
  outFile << "property float x" << std::endl;
  outFile << "property float y" << std::endl;
  outFile << "property float z" << std::endl;
  if (PC.nor.size()>0)
  {
    outFile << "property float nx" << std::endl;
    outFile << "property float ny" << std::endl;
    outFile << "property float nz" << std::endl;
  }
  outFile << "end_header" << std::endl;

  ////
  // Points
  ////

  for ( int i = 0; i < pointNum; ++i )
  {
    float* point = (float*)(&PC.pts[i]);

    outFile << point[0] << " "<<point[1]<<" "<<point[2];

    if (PC.nor.size()>0)
    {
      outFile<<" " << PC.nor[i].x() << " "<<PC.nor[i].y()<<" "<<PC.nor[i].z();
    }

    outFile << std::endl;
  }

  return;
}

PointCloud loadPointCloud(std::string filename, int ptLimit){
    std::ifstream file(filename.c_str(),std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
    }

    vector<Vector3f> pts,nor;

    while(file && (ptLimit==-1 || (ptLimit--) > 0)){
        Vector3f pt,no;
        file >> pt.x() >> pt.y() >> pt.z() >> no.x() >> no.y() >> no.z();
        pts.push_back(pt);
        nor.push_back(no);
    }

    PointCloud C;
    C.pts=pts;
    C.nor=nor;

    return C;
}

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
    if(type=="f"){
        //std::cout.precision(8);
        outputFile << mat.format(8);//StreamPrecision);  #TODO: somehow 0.10793996f cant be saved correctly otherwise
    }else{
        outputFile << mat.format(FullPrecision);
    }
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
