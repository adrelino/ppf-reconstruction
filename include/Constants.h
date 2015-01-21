//
//  Params.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef PointPairFeatures_Params_h
#define PointPairFeatures_Params_h

#include <eigen3/Eigen/Dense>
#include <vector>
#include <unordered_map>
#include "pointcloud.h"

using namespace std;

//Typedefs
//
typedef vector<PPF> Bucket;
//key type, value, hasher
typedef unordered_map<int,Bucket> GlobalModelDescription;
struct Match {PPF scenePPF; Bucket modelPPFs;};
typedef std::pair<int, Bucket> KeyBucketPair;
typedef vector<KeyBucketPair> KeyBucketPairList;
typedef vector<Match> Matches;
typedef pair<Isometry3f,int> Pose;
typedef vector<Pose> Poses;
typedef std::pair< Matches,vector<int> > MatchesWithSceneRefIdx;



struct TrainedModel{GlobalModelDescription modelDescr;
                    PointCloud mSmall;
                    //Translation3f centroid;
                   };

//Params
//

//model diameter, downsampling
static float diamM=0.15f; //model diameter //stanford bunny // max dist is diagonal
static float tau_d=0.1f; //sampling rate, set like in paper. ddist=tau_d*diam(M);
static float ddist = tau_d*diamM;//0.01 paper: 0.05*diam(M)=o.o5*0.15=0.0075

// reestimate normals after downsampling
static float neighbourBallSize=ddist*2.5f; //neighbors in 2*ddist ball around p1 (e.g. 2cm)

//ppf's
static int ndist=1/tau_d; //number of distance buckets
static int nangle=30;     //number of angle buckets
static float dangle = 2*M_PI/nangle; //normal's derivation of up to 12 degree like in paper (360/30=12)
static float sceneRefPtsFraction = 0.8f; //20percent of pts in scene picked (at random so far) as reference points to compute ppfs to all other model points

//for pose cluster averaging
//static float thresh_tra = 0.05f * diamM; //float thresh_tra=0.02; //2cm
//static float thresh_rot_degrees = 20; //180 max

//for syntetic bunny
static float thresh_tra = 0.01f * diamM; //float thresh_tra=0.02; //2cm
static float thresh_rot_degrees = 30; //180 max

//Macros
//
#define rad2deg(r) (180*(r)/M_PI)
#define deg2rad(d) (M_PI*(d)/180)

namespace Colormap{


//http://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
static float interpolate( float val, float y0, float x0, float y1, float x1 ) {
    return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

static float base( float val ) {
    if ( val <= -0.75 ) return 0;
    else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
    else if ( val <= 0.25 ) return 1.0;
    else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
    else return 0.0;
}

static float red( float gray ) {
    return base( gray - 0.5 );
}
static float green( float gray ) {
    return base( gray );
}
static float blue( float gray ) {
    return base( gray + 0.5 );
}

static RowVector3f Jet(float gray){
    return RowVector3f(red(gray),green(gray),blue(gray));
}

static Vector4f RED1=Vector4f(0.8,0,0.4,1);
static Vector4f RED2=Vector4f(0.4,0.0,0.1,0.5);
static Vector4f GREEN1=Vector4f(0.0,1.0,0.5,1);
static Vector4f GREEN2=Vector4f(0.0,.4,0.1,0.5);

}

//namespace PosePrint{

static void printPose(Isometry3f P,string title=""){
    //cout<<P.matrix()<<endl;
    Vector3f tra(P.translation());
    Quaternionf q(P.linear());
    Vector4f rot(q.x(),q.y(),q.z(),q.w());
    //cout<<setprecision(3);
    if(title!="") cout<<title<<endl;
    cout<<"tra: "<<tra.transpose()<<endl;
    cout<<"rot: "<<rot.transpose()<<endl;
}

static void printPose(Pose pose,string title=""){
    if(title!="") title+=" ";
    cout<< title <<"score : "<<pose.second<<endl;
    printPose(pose.first);
}


static void printPoses(Poses vec){
    for(auto pose : vec){
        printPose(pose);
    }
}

static float err(Isometry3f P, Isometry3f Pest, bool fullOutput=false){
    Vector3f tra(P.translation());
    Quaternionf q(P.linear());
    Vector4f rot(q.x(),q.y(),q.z(),q.w());

    Vector3f tra_est(Pest.translation());
    Quaternionf qest(Pest.linear());
    Vector4f rot_est(qest.x(),qest.y(),qest.z(),qest.w());

    Vector3f tra_diff = (tra - tra_est);
    float tra_error=tra_diff.array().cwiseAbs().sum();

    Vector4f rot_diff = (rot - rot_est);
    float rot_error=rot_diff.array().cwiseAbs().sum();

//    if(rot_error>1.5){
//        cout<<"flipping rot"<<endl;
//        rot_est*=-1;
//        rot_diff = (rot - rot_est);
//        rot_error=rot_diff.array().cwiseAbs().sum();
//    }

    float meanError=(tra_error + rot_error)*0.5f;
    cout<<"pose meanError:"<<meanError<<endl;

    if(fullOutput){
        //cout<<setprecision(3);
        cout<<"tra_ori: "<<tra.transpose()<<endl;
        cout<<"tra_est: "<<tra_est.transpose()<<endl;
        //cout<<"tra_dif: "<<tra_diff.transpose()<<"\n --->tra_error: "<<tra_error<<endl;
        cout<<"tra_error:----------------------------------------------> "<<tra_error<<endl;

        cout<<"rot_ori: "<<rot.transpose()<<endl;
        cout<<"rot_est: "<<rot_est.transpose()<<endl;
        //cout<<"rot_dif: "<<rot_diff.transpose()<<endl;
        cout<<"rot_error:----------------------------------------------> "<<rot_error<<endl;
    }

    return meanError;
}

static float err(Isometry3f P, Pose PoseEst){
    cout<<"//------- Error between P_gold and P_est with "<<PoseEst.second<<" votes --------\\"<<endl;
    Isometry3f Pest=PoseEst.first;
    return err(P,Pest);
}

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
template<> inline bool getParam<bool>(std::string param, bool &var, int argc, char **argv)
{
    const char *c_param = param.c_str();
    for(int i=argc-1; i>=1; i--)
    {
        if (argv[i][0]!='-') continue;
        if (strcmp(argv[i]+1, c_param)==0)
        {
            if (!(i+1<argc) || argv[i+1][0]=='-') { var = true; return true; }
            std::stringstream ss;
            ss << argv[i+1];
            ss >> var;
            return (bool)ss;
        }
    }
    return false;
}

//}

#endif
