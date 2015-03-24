//
//  Params.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef PointPairFeatures_Params_h
#define PointPairFeatures_Params_h

#include "Params.h"

#include <eigen3/Eigen/Dense>
#include <vector>
#include <unordered_map>
#include "PointCloud.h"

using namespace std;

//Typedefs
//
typedef vector<PPF> Bucket;
//key type, value, hasher
//typedef unordered_map<int,Bucket> GlobalModelDescription;
//struct Match {PPF scenePPF; Bucket modelPPFs;};
typedef std::pair<int, Bucket> KeyBucketPair;
typedef vector<KeyBucketPair> KeyBucketPairList;
//typedef vector<Match> Matches;
typedef pair<Isometry3f,int> Pose;
typedef vector<Pose> Poses;


//Macros
//
#define rad2degM(r) (180*(r)/M_PI)
#define deg2radM(d) (M_PI*(d)/180)

#define mod(a,b) ( (a + b) % b ) //works only if |a|<b

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

static Vector3f Jet(float gray){
    return Vector3f(red(gray),green(gray),blue(gray));
}

static Vector4f MAG1=Vector4f(0.3,0,0.8,1);
static Vector4f MAG2=Vector4f(0.1,0.0,0.5,0.5);
static Vector4f RED1=Vector4f(0.8,0,0.4,1);
static Vector4f RED2=Vector4f(0.4,0.0,0.1,0.5);
static Vector4f GREEN1=Vector4f(0.0,1.0,0.5,1);
static Vector4f GREEN2=Vector4f(0.0,.4,0.1,0.5);
static Vector3f BLUE1=Vector3f(0.0,0,0.5);
static Vector3f BLUE2=Vector3f(0.0,0,0.8);
static Vector3f ORANGE1=Vector3f(1,0.6,0);
static Vector3f ORANGE2=Vector3f(1,0.6,0);

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

static void printPose(Isometry3d P,string title=""){
    //cout<<P.matrix()<<endl;
    Vector3d tra(P.translation());
    Quaterniond q(P.linear());
    Vector4d rot(q.x(),q.y(),q.z(),q.w());
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

static float err(Isometry3f P, Isometry3f Pest, bool fullOutput=false, bool withRot=false){
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

    float meanError=(tra_error);// + rot_error)*0.5f;
    if(withRot){
        meanError+=rot_error;
        meanError/=2;
    }

    if(fullOutput){

    cout<<"pose meanError:"<<meanError<<endl;

        //cout<<setprecision(3);
//        cout<<"tra_ori: "<<tra.transpose()<<endl;
//        cout<<"tra_est: "<<tra_est.transpose()<<endl;
//        //cout<<"tra_dif: "<<tra_diff.transpose()<<"\n --->tra_error: "<<tra_error<<endl;
//        cout<<"tra_error:----------------------------------------------> "<<tra_error<<endl;

//        cout<<"rot_ori: "<<rot.transpose()<<endl;
//        cout<<"rot_est: "<<rot_est.transpose()<<endl;
//        //cout<<"rot_dif: "<<rot_diff.transpose()<<endl;
//        cout<<"rot_error:----------------------------------------------> "<<rot_error<<endl;
    }

    return meanError;
}

static float err(Isometry3f P, Pose PoseEst){
    cout<<"//------- Error between P_gold and P_est with "<<PoseEst.second<<" votes --------\\"<<endl;
    Isometry3f Pest=PoseEst.first;
    return err(P,Pest);
}

//https://forum.kde.org/viewtopic.php?f=74&t=94839
static Matrix3Xf vec2mat(vector<Vector3f>& vec){
    Map<Matrix<float,3,Dynamic,ColMajor> > mat(&vec[0].x(), 3, vec.size());
    return mat;
}

static vector<Vector3f> mat2vec(const Matrix3Xf& mat){
    vector<Vector3f> vec(mat.cols());
    for(int i=0; i<mat.cols(); i++){
        vec[i]=mat.col(i);
    }

    //http://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
    //vector<Vector3f> vec(mat.data(),mat.data() + mat.rows() * mat.cols());
    return vec;
}


//float diff_rot= 1 - d*d; //http://www.ogre3d.org/forums/viewtopic.php?f=10&t=79923

//float diff_rot2 = rot1.angularDistance(rot2);

//http://math.stackexchange.com/questions/90081/quaternion-distance
//float thresh_rot=0.25; //0 same, 1 180deg ////M_PI/10.0; //180/15 = 12
//float diff_rot_bertram = acos((rot1.inverse() * rot2).norm()); //bertram
//cout<<"diff_rot_0to1nor\t="<<diff_rot<<endl;
//cout<<"diff_rot_bertram\t="<<diff_rot_bertram<<endl;

//cout<<std::fixed<<std::setprecision(3);

//cout<<"rot="<<diff_rot_degrees<<"<="<<thresh_rot_degrees<<" && tra="<<diff_tra<<"<= "<<thresh_tra<<" ?: ";

static
bool isPoseSimilar(Isometry3f P_i, Isometry3f P_j, float t_rot, float t_tra){
    //Translation
    float d_tra=(P_i.translation()-P_j.translation()).norm();
    //Rotation
    float d = Quaternionf(P_i.linear()).dot(Quaternionf(P_j.linear()));
    float d_rot = rad2degM(acos(2*d*d - 1));
    return d_rot <= t_rot && d_tra <= t_tra;
}

static bool isPoseSimilar(Isometry3f P1, Isometry3f P2){
    return isPoseSimilar(P1,P2,Params::getInstance()->thresh_rot,Params::getInstance()->thresh_tra);
}

#include <string>
#include <sstream>

static std::string poseDiff(Isometry3f P1, Isometry3f P2){
    //cout<<"isPoseSimilar tra: "<<thresh_tra_l<<" rot: "<<thresh_rot_l<<endl;
    Vector3f    tra1 = P1.translation();
    Quaternionf rot1(P1.linear());

    Vector3f    tra2 = P2.translation();
    Quaternionf rot2(P2.linear());


    //Translation
    float diff_tra=(tra1-tra2).norm();
    //Rotation
    float d = rot1.dot(rot2);
    float diff_rot_degrees = rad2degM(acos(2*d*d - 1));


    stringstream ss;
    ss<<"diff_tra:"<<diff_tra<<" diff_rot_degrees:"<<diff_rot_degrees<<endl;

    return ss.str();

}

static std::string poseDiff(Isometry3d P1, Isometry3d P2){
    //cout<<"isPoseSimilar tra: "<<thresh_tra_l<<" rot: "<<thresh_rot_l<<endl;
    Vector3d    tra1 = P1.translation();
    Quaterniond rot1(P1.linear());

    Vector3d    tra2 = P2.translation();
    Quaterniond rot2(P2.linear());


    //Translation
    double diff_tra=(tra1-tra2).norm();
    //Rotation
    double d = rot1.dot(rot2);
    double val = 2*d*d - 1;
    if(val< -1) val = -1;
    if(val> 1) val = 1;
    double diff_rot_degrees = rad2degM(acos(val));


    stringstream ss;
    ss<<"diff_tra:"<<diff_tra<<" diff_rot_degrees:"<<diff_rot_degrees<<endl;

    return ss.str();

}

static bool isPoseCloseToIdentity(Isometry3f P1, float eps){
    Vector3f    tra1 = P1.translation();
    Quaternionf rot1(P1.linear());

    //Translation
    float diff_tra=tra1.norm();
    //Rotation
    Vector3f rot1V(rot1.x(),rot1.y(),rot1.z());  //w should be close to 1
    float diff_rot=rot1V.norm();

    return diff_tra < eps && diff_rot < eps;

}


/*
 * cv::Point3f colorMapped(int j, int max, int colorMap){
    cv::Mat depthMap(1,max,CV_8UC1);
    for (int i = 0; i <= max; ++i) {
        depthMap.at<uint8_t>(0,i)=i;
    }
    cv::Mat heatMap;
    cv::applyColorMap(depthMap, heatMap,colorMap);
    return heatMap.at<cv::Point3f>(0,j);

}
*/

static Eigen::Matrix3f hat(Vector3f x){
    Matrix3f mat;
    mat<< 0,    -x(3), x(2),
          x(3),  0 ,   -x(1),
          -x(2), x(1), 0;
    return mat;
}




//}

#endif
