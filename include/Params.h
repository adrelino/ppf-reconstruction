#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>
#include <memory>


// parameter processing
template<typename T> bool getParam(std::string param, T &var, int argc, char **argv)
{
    const char *c_param = param.c_str();
    for(int i=argc-1; i>=1; i--)
    {
        if (argv[i][0]!='-') continue;
        if (strcmp(argv[i]+1, c_param)==0)
        {
            if (!(i+1<argc)) continue;
            std::stringstream ss;
            ss << argv[i+1];
            ss >> var;
            std::cout<<"PARAM[SET]: "<<param<<" : "<<var<<std::endl;
            return (bool)ss;
        }
    }
    std::cout<<"PARAM[DEF]: "<<param<<" : "<<var<<std::endl;
    return false;
}

// parameter processing: template specialization for T=bool
//template<> inline bool getParam<bool>(std::string param, bool &var, int argc, char **argv)
//{
//    const char *c_param = param.c_str();
//    for(int i=argc-1; i>=1; i--)
//    {
//        if (argv[i][0]!='-') continue;
//        if (strcmp(argv[i]+1, c_param)==0)
//        {
//            if (!(i+1<argc) || argv[i+1][0]=='-') {
//                var = true;
//                std::cout<<"PARAM[SET]: "<<param<<" : "<<var<<std::endl;
//                return true;
//            }
//            std::stringstream ss;
//            ss << argv[i+1];
//            ss >> var;
//            std::cout<<"PARAM[SET]: "<<param<<" : "<<var<<std::endl;
//            return (bool)ss;
//        }
//    }
//    std::cout<<"PARAM[DEF]: "<<param<<" : "<<var<<std::endl;
//    return false;
//}

using namespace std;

#include <math.h>
#include <opencv2/opencv.hpp>
#include <map>

class Params
{
private:

    static Params* instance;

    //std::map<string,string> nonDefaultParams;

    Params(){
        //model diameter, downsampling
        diamM=0.15f; //model diameter //stanford bunny // max dist is diagonal
        tau_d=0.1f; //sampling rate, set like in paper. ddist=tau_d*diam(M);
        ddist = tau_d*diamM;//0.01 paper: 0.05*diam(M)=o.o5*0.15=0.0075

        //ppf's
        ndist=1/tau_d; //number of distance buckets
        nangle=30;     //number of angle buckets
        dangle = 2*M_PI/nangle; //normal's derivation of up to 12 degree like in paper (360/30=12)

        minPtsPerVoxel = 20; //outlier rejection in downsampling, makes normals more stable

        //for pose cluster averaging
        thresh_tra = 0.05f * diamM; //float thresh_tra=0.02; //2cm
        thresh_rot = 15; //180 max

        dir = "../samples/Bunny_RealData";
        est_poses_prefix = "estimates_";
        est_poses_suffix = ".txt";

        stopFrames=true;
    }




public:
    cv::Mat grayMap,colorMap;


    static string getDir(){
        return getInstance()->dir;
    }

    static Params* getInstance();

//namespace Params{

    //model diameter, downsampling
     float diamM;
     float tau_d;
     float ddist;

    //ppf's
     int ndist;
     int nangle;
     float dangle;

     int minPtsPerVoxel;

    //for pose cluster averaging
     float thresh_tra;
     float thresh_rot;

     std::string est_poses_prefix;
     std::string est_poses_suffix;

     std::string dir;

     bool useFlann = true;
     bool pointToPlane = true; //otherwise pointToPoint

     bool stopFrames=false;


     void getParams(int argc, char **argv){

         getParam("dir", dir, argc, argv);

         getParam("diamM", diamM, argc, argv);
         getParam("tau_d", tau_d, argc, argv);
         ddist = tau_d*diamM;//0.01 paper: 0.05*diam(M)=o.o5*0.15=0.0075
         ndist=1/tau_d;
         cout<<"--> ddist=tau_d*diamM:"<<ddist<<" ndist:"<<ndist<<endl;

         thresh_tra = 0.05f * diamM;
         cout<<"--> thresh_tra=0.05*diamM:"<<thresh_tra<<endl;


         getParam("nangle", nangle, argc, argv);
         getParam("thresh_tra",thresh_tra, argc, argv);
         getParam("thresh_rot",thresh_rot,argc,argv);

         getParam("minPtsPerVoxel",minPtsPerVoxel,argc,argv);

         getParam("pointToPlane", pointToPlane,argc,argv);
         getParam("useFlann", useFlann,argc,argv);

         getParam("stopFrames",stopFrames,argc,argv);


    }

    Eigen::Vector3f colorJet(float val, float min, float max);


};

#endif // PARAMS_H
