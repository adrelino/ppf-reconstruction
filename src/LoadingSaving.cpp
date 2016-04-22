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

#include <opencv2/opencv.hpp>

#include "OpenCVHelpers.h"

#include "PointCloudManipulation.h"


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

void loadPointCloudFromDepthMap(const std::string& filename, const Matrix3f& K, vector<Vector3f>& pts, string maskname, bool show){
    std::ifstream ifs(filename.c_str());
    if (!ifs.is_open()) cout<<"Cannot open file:"<<filename<<endl;

    cout<<"LoadingSaving::loadPointCloudFromDepthMap before cv::imread "<<filename<<endl;
    cv::Mat depth = cv::imread(filename,cv::IMREAD_UNCHANGED);
    int type=depth.type();
    int nc=depth.channels();
    cout<<"LoadingSaving::loadPointCloudFromDepthMap after cv::imread "<<filename<<" \t type: "<<OpenCVHelpers::getImageType(type)<<" channels:"<<nc<<endl;

    cv::Mat mask; //is 1 at the object, 0 outside

    bool maskGiven = false;

    if(maskname!=""){
        mask = cv::imread(maskname,cv::IMREAD_UNCHANGED);
        //cout<<"LoadingSaving read mask: "<<maskname<<" "<< mask.rows <<","<<mask.cols<<endl;
        maskGiven = true;
    }

    float fx = K(0,0);
    float fy = K(1,1);
    float cx = K(0,2);
    float cy = K(1,2);

    //todo add possibility to add segmentation mask
    if(type==CV_16U && nc==1){ //depth_0.png from kinect cam, 0 means no measure, depth in mm (470 means 0.47m);
        if(!maskGiven) mask = (depth > 0.001f);
        depth.convertTo( depth, CV_32FC1, 0.001); // convert to meters
    }else if(type==CV_32FC3 && nc==3){ //depth_000000.exr made from blender, inf means no measure, depth in mm
        cv::cvtColor(depth,depth,cv::COLOR_BGR2GRAY);
        if(!maskGiven) mask = (depth != std::numeric_limits<float>::infinity());
        depth.convertTo( depth, CV_32FC1, 0.001); // convert to meters
    }else if(type==CV_8UC3 && nc==3){ //Siemens steamturbine
        cout<<"ATTENTTION DOWNSAMPLIG INPUT IMAGE"<<endl;
        std::vector<cv::Mat> depth_channels;
        cv::split( depth, depth_channels );
        cv::Mat r_channel, g_channel, b_channel;
        depth_channels[2].convertTo( r_channel, CV_32SC1 ); depth_channels[1].convertTo( g_channel, CV_32SC1 ); depth_channels[0].convertTo( b_channel, CV_32SC1 );
        cv::Mat depthMap;
        depthMap.create( depth.rows, depth.cols, CV_32FC1 );
        //mask.create( depth.rows, depth.cols, CV_8UC1 );
        for (int r = 0; r < depthMap.rows; ++r)
            for (int c = 0; c < depthMap.cols; ++c) {
                float val = 1e-6f * ( ( r_channel.at<int>(r,c) << 16 ) | ( g_channel.at<int>(r,c) << 8 ) | b_channel.at<int>(r,c) );
                depthMap.at<float>(r,c) = val;
                //mask.at<uchar>(r,c) = ( val > 0.f );
            }
        if(!maskGiven) mask = (depthMap > 0.f);

        depth=depthMap;

        //cout<<"mask type: "<<OpenCVHelpers::getImageType(mask.type())<<" channels:"<<mask.channels()<<endl;


        float factor=0.2f;

        resize(depth, depth, cv::Size(), factor, factor);
        resize(mask, mask, cv::Size(), factor, factor);
        //cout<<"mask type after resize: "<<OpenCVHelpers::getImageType(mask.type())<<" channels:"<<mask.channels()<<endl;

        cx *= factor;
        cy *= factor;
        fx *= factor;
        fy *= factor;
    }else{
        cout<<"unsupported depth image type"<<endl;
    }




    if(show){
//        if(depth.rows>500){
//            cv::Mat depthSmall, maskSmall;
//            resize(depth, depthSmall, cv::Size(), 0.25, 0.25);
//            resize(mask, maskSmall, cv::Size(), 0.25, 0.25);
//            OpenCVHelpers::showDepthImage("showDepthImage",depthSmall,maskSmall,true);
//       }else{
            OpenCVHelpers::showDepthImage("depth",depth);
            OpenCVHelpers::showDepthImage("depthWithMask",depth,mask);
            //OpenCVHelpers::showImage("mask",mask);

 //      }
        cv::waitKey(2);
    }

    //PointCloudManipulation::fromDepthImage(depth,mask,pts,K);
    //void PointCloudManipulation::fromDepthImage(const cv::Mat& depth, const cv::Mat& mask, vector<Vector3f>& pts, const Matrix3f& K){
        int m=depth.rows; //Y
        int n=depth.cols; //X

        //int x=n,y=m,xM=0,yM=0;

        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < n; ++j) {
                if(mask.at<unsigned char>(i,j)){ //values lower than 255 were achieved by non-border preserving smoothing before subsampling
                    float Z = depth.at<float>(i,j);
                    if(Z<=0.0001 || Z > 999999) continue;
                    float X = (j-cx)*Z*(1/fx);
                    float Y = (i-cy)*Z*(1/fy);
                    pts.push_back(Vector3f(X,Y,Z));
                    //if(j<x) x=j;if(j>xM) xM=j;if(i<y) y=i;if(i>yM) yM=i;
                }
            }
        }
    //}
}

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

void writePointCloud(std::string filename, PointCloud C){
    Matrix3Xf A = C.ptsMat();
    MatrixXf D;

    if(C.nor.size()>0){
        Matrix3Xf B = C.norMat();

        D = MatrixXf(A.rows()+B.rows(), A.cols());
        D << A,
             B;
    }else{
        D = A;
    }


    MatrixXf DD = D.transpose();

    saveMatrixXf(filename,DD);
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

vector<Isometry3f> loadPosesFromFile(std::string filename){
    std::ifstream file(filename,std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
    }

    vector<Isometry3f> vec;

    std::string line;

    int j=0;

    int row=0;

    Matrix4f pose;

    while(std::getline(file,line)){
        std::stringstream lineStream(line);
        //cout<<"line: "<<line<<endl;
        if(line.length()<6){
            lineStream >> j;
            //cout<<"pose "<<j<<":"<<endl;
        }else{
            int col=0;
            float elem;
            while(lineStream >> elem){
                pose(row,col)=elem;
                //cout<<pose(row,col)<<endl;
                col++;
            }
            row++;
            if(row==4){
                row=0;
                //cout<<pose<<endl;
                vec.push_back(Isometry3f(pose));
            }
        }
    }

    return vec;

}

vector<Isometry3f> loadPosesFromDir(std::string dir, std::string prefix){
    vector<string> poses = getAllTextFilesFromFolder(dir,prefix);
    vector<Isometry3f> vec;

    for(auto it : poses){
        vec.push_back(Isometry3f(LoadingSaving::loadMatrix4f(it)));
    }

//    cout<<"posesfromdir"<<endl;
//    for(auto it : vec){
//        cout<<endl<<it.matrix()<<endl;
//    }

    cout<<"loaded "<<vec.size()<<" poses from file "<<poses[0]<<endl;

    return vec;
}

vector<Isometry3f> loadPoses(string dir, string prefix){
    vector<string> poses = getAllTextFilesFromFolder(dir,prefix);
    vector<Isometry3f> vec;
    if(poses.size() == 1){
        return loadPosesFromFile(poses[0]);
    }else if(poses.size()>1){
        for(auto it : poses){
            vec.push_back(Isometry3f(LoadingSaving::loadMatrix4f(it)));
        }
        cout<<"loaded "<<vec.size()<<" poses from dir "<<dir<<" with prefix "<<prefix<<endl;
    }else{
        cerr<<"no poses provided"<<endl;
    }
    return vec;
}

Matrix4f loadMatrix4f(std::string filename){
    std::ifstream file(filename,std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
        return Matrix4f::Zero();

    }
    float array[16]={0};
    array[15]=1;
    int i=0;
    while(file >> array[i++]){}

    //cout<<"Loaded Matrix4f from "<<filename<<endl;
    return Map< Matrix<float,4,4,RowMajor> > (array);
}

Matrix3f loadMatrix3f(std::string filename){
    std::ifstream file(filename,std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
        return Matrix3f::Zero();

    }
    float array[9];
    int i=0;
    while(file >> array[i++]){}

    //cout<<"Loaded Matrix4f from "<<filename<<endl;
    return Map< Matrix<float,3,3,RowMajor> > (array);
}

Vector3f loadVector3f(std::string filename){
    std::ifstream file(filename,std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
        return Vector3f::Zero();

    }
    float array[3];
    int i=0;
    while(file >> array[i++]){}

    //cout<<"Loaded Matrix4f from "<<filename<<endl;
    return Map< Vector3f > (array);
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
}

void saveMatrix3f(std::string filename, Matrix3f mat){
    std::ofstream outputFile(filename, std::ofstream::out) ;
    outputFile << mat.format(FullPrecision);
    outputFile.close();
}

void saveMatrixXd(std::string filename, MatrixXd mat){return saveMatrix<double>(filename,mat,"d");}
void saveMatrixXf(std::string filename, MatrixXf mat){return saveMatrix<float>(filename,mat,"f");}
void saveMatrixXi(std::string filename, MatrixXi mat){return saveMatrix<int>(filename,mat,"i");}


template<typename Number>
void saveVector(std::string filename, vector<Number> vec){

    std::ofstream outputFile(filename, std::ofstream::out) ;

    for (auto it : vec){
        outputFile << it <<endl;
    }

    outputFile.close( );

    cout<<"Wrote "<<vec.size()<<" numbers to "<<filename<<endl;
}
void saveVector(std::string filename, vector<bool> vec){saveVector<bool>(filename,vec);}
void saveVectorf(std::string filename, vector<float> vec){saveVector<float>(filename,vec);}

void savePosesEvalutationGroundTruth(vector<Isometry3f>& poses){
    std::stringstream ss;
    ss<<Params::getInstance()->dir<<"/"<<"evaluationQ.txt";
    //<<setfill('0')<< setw(6)<<cloud.imgSequenceIdx<<Params::getInstance()->est_poses_suffix;
    string filename = ss.str();
    std::ofstream outputFile(filename, std::ofstream::out) ;

    //std::ostream& o = std::cout;

    for (int i = 0; i< poses.size(); i++){
        std::stringstream timestampss;
        timestampss<<setfill('0')<< setw(6)<<i; //cloud.imgSequenceIdx<<Params::getInstance()->est_poses_suffix;

        outputFile<<timestampss.str()<<" ";

        outputFile<<std::setprecision(8);

        Isometry3f pose = poses[i];
        outputFile<<pose.translation().transpose()<<" ";
        Quaternionf q(pose.linear());
        outputFile<<q.coeffs().transpose()<<endl;
        //o<<q.x()<< " "<<q.y()<<" "<<q.z() << " "<<q.w();

        //outputFile << timestampss.str() << " " <<   endl;
    }

   outputFile.close( );

    cout<<"Wrote "<<poses.size()<<" poses to "<<filename<<endl;
}

void savePosesEvalutation(const vector<std::shared_ptr<PointCloud> >& frames){
    std::stringstream ss,ss2;
    string suffix="";
    ss<<Params::getInstance()->dir<<"/"<<"P"<<suffix<<".txt";
    ss2<<Params::getInstance()->dir<<"/"<<"Q"<<suffix<<".txt";

    string filename = ss.str();
    std::ofstream outputFile(filename, std::ofstream::out) ;
    string filename2 = ss2.str();
    std::ofstream outputFile2(filename2, std::ofstream::out) ;

    //std::ostream& o = std::cout;

    for (int i = 0; i< frames.size(); i++){
        std::stringstream timestampss;
        timestampss<<setfill('0')<< setw(6)<<frames[i]->imgSequenceIdx;//<<Params::getInstance()->est_poses_suffix;

        //P
        {
            outputFile<<timestampss.str()<<" ";

            outputFile<<std::setprecision(12);

            Isometry3d pose = frames[i]->pose.cast<double>();
            outputFile<<pose.translation().transpose()<<" ";
            Quaterniond q(pose.linear());
            outputFile<<q.coeffs().transpose()<<endl;
        }

        //Q
        {
            outputFile2<<timestampss.str()<<" ";

            outputFile2<<std::setprecision(12);

            Isometry3d pose2 = frames[i]->poseGroundTruth.cast<double>();
            outputFile2<<pose2.translation().transpose()<<" ";
            Quaterniond q2(pose2.linear());
            outputFile2<<q2.coeffs().transpose()<<endl;
        }
    }

     outputFile.close( );
     outputFile2.close();

    cout<<"Wrote "<<frames.size()<<" poses to "<<filename<< " and "<<filename2<<endl;
}

void savePosesEvalutationEstimates(const vector<std::shared_ptr<PointCloud> >& frames,string suffix){
    std::stringstream ss;
    ss<<Params::getInstance()->dir<<"/"<<"evaluationP_"<<suffix<<".txt";
    //<<setfill('0')<< setw(6)<<cloud.imgSequenceIdx<<Params::getInstance()->est_poses_suffix;
    string filename = ss.str();
    std::ofstream outputFile(filename, std::ofstream::out) ;

    //std::ostream& o = std::cout;

    for (int i = 0; i< frames.size(); i++){
        std::stringstream timestampss;
        timestampss<<setfill('0')<< setw(6)<<frames[i]->imgSequenceIdx;//<<Params::getInstance()->est_poses_suffix;

        outputFile<<timestampss.str()<<" ";

        outputFile<<std::setprecision(12);

        Isometry3d pose = frames[i]->pose.cast<double>();
        outputFile<<pose.translation().transpose()<<" ";
        Quaterniond q(pose.linear());
        outputFile<<q.coeffs().transpose()<<endl;
        //o<<q.x()<< " "<<q.y()<<" "<<q.z() << " "<<q.w();

        //outputFile << timestampss.str() << " " <<   endl;
    }

   outputFile.close( );

    cout<<"Wrote "<<frames.size()<<" poses to "<<filename<<endl;
}


template<typename Number>
vector<Number> loadVector(std::string filename){

    vector<Number> vec;

    std::ifstream file(filename,std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
        return vec;
    }

    Number elem;
    while(file >> elem){
        vec.push_back(elem);
    }

    return vec;
}

vector<bool> loadVector(std::string filename){return loadVector<bool>(filename);}
vector<float> loadVectorf(std::string filename){return loadVector<float>(filename);}


string getOSSeparator() {
#ifdef _WIN32
  return "\\";
#else
  return "/";
#endif
}

#include <dirent.h>
#include <vector>

bool isPrefixAndSuffix(const char* file, uint16_t filename_length, string prefix, string suffix){


   const char* startSuffix=strstr(file,suffix.c_str());
   bool isSuffix = (startSuffix-file) == filename_length - suffix.length();
   if(!isSuffix) return false;


   if(prefix[0]=='*'){
        string contains = prefix.substr (1);
        const char* startContains=strstr(file,contains.c_str());
        bool isContains = (startContains-file >= 0); //contains
        return isContains;
   }

    const char* startPrefix=strstr(file,prefix.c_str());
    bool isPrefix = (startPrefix-file == 0);

    //cout<<"start: "<<isPrefix<<" end:"<<isSuffix<<endl;

    return isPrefix;
}
#include <algorithm>    // std::any_of
//#include <array>        // std::array

const vector<string> SUFFIX_IMAGE = {".png", ".jpg",".tif",".exr",".bmp"};
const vector<string> SUFFIX_TEXT  = {".txt",".xyz"};
//const std::array<string,1> SUFFIX_PLY   = {".ply"};

bool hasPrefixAndSuffixes(const char* file, uint16_t filename_length, string prefix,std::vector<string> suffixes){
  return std::any_of(suffixes.begin(), suffixes.end(),
         [&](string suffix){return isPrefixAndSuffix(file,filename_length,prefix,suffix);
  });
}

//http://stackoverflow.com/questions/9277906/stdvector-to-string-with-custom-delimiter
string join(vector<string>& v, const string& delim) {
    ostringstream s;
    for (const auto& i : v) {
        if (&i != &v[0]) {
            s << delim;
        }
        s << i;
    }
    return s.str();
}

vector<string> getAllFilesFromFolder(string dirStr, string prefix, vector<string> suffixes) {
  DIR *dir = NULL;
  struct dirent *entry;
  vector<string> allImages;

  dir = opendir(dirStr.c_str());

  if (!dir) {
    cerr << "Could not open directory " << dirStr <<endl;//<< ". Exiting..." << endl;
    return allImages;
    //exit(1);
  }

  const string sep = getOSSeparator();

  while((entry = (readdir(dir)))) {
      string fileName(entry->d_name);
    if (hasPrefixAndSuffixes(entry->d_name,/*entry->d_namlen*/fileName.size(),prefix,suffixes)){
      string fileName(entry->d_name);
      string fullPath = dirStr + sep + fileName;
      allImages.push_back(fullPath);
    }
  }
  closedir(dir);

  std::sort(allImages.begin(), allImages.end(), [](const std::string &left, const std::string &right) {
      int lengthdiff=left.size()-right.size();
      if(lengthdiff==0){
          return left < right;
      }
      return lengthdiff<0;
  });

  string joined = join(suffixes,"|");

  cout<<"Found "<<allImages.size()<<" files in "<<dirStr << " that match the pattern: "<<prefix<<"*"<<joined<<" .";
  if(allImages.size()>0){
      cout<<" first is: "<<allImages[0];
  }
  cout<<endl;
  return allImages;
}

vector<string> getAllImagesFromFolder(string dirStr, string prefix){
    return getAllFilesFromFolder(dirStr,prefix,SUFFIX_IMAGE);
}

vector<string> getAllTextFilesFromFolder(string dirStr, string prefix){
    return getAllFilesFromFolder(dirStr,prefix,SUFFIX_TEXT);
}

} // end ns
