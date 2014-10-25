#include "LoadingSaving.h"
#include "PointPairFeatures.h"
#include "PointCloudManipulation.h"
#include "PPF.h"
#include <iostream>

using namespace std;

int main(int argc, char * argv[])
{
    MatrixXd m=PointCloudManipulation::downSample(LoadingSaving::loadXYZ("bunny/scene.xyz"),false);
    Quaterniond q=Quaterniond::Identity();
    q = q * AngleAxisd(radians(180), Vector3d::UnitX());
    q = q * AngleAxisd(radians(-180),Vector3d::UnitY());
    q = q * AngleAxisd(radians(360*3),Vector3d::UnitZ());


    Vector4d rot(q.x(),q.y(),q.z(),q.w());
    Vector3d tra(.1,300,-4);

    Projective3d P = Translation3d(tra)*Quaterniond(rot);
    PointPairFeatures::printPose(P,"P_original:");
    MatrixXd s=PointCloudManipulation::projectPointsAndNormals(P, m);

    //now we simulate that there is a ppf correspondence between first row of sSmall and sSmallProjected

    //PPF:planarRotationAngle
    RowVectorXd p1(6),p2(6);
    p1=m.row(0);
    p2=m.row(1); //model
    PPF ppfModel = PPF::makePPF(p1,p2,0,1);  //also gets alpha-> planar rot angle to local coords

    RowVectorXd q1(6),q2(6);
    q1=s.row(0);
    q2=s.row(1); //scene
    PPF ppfScene = PPF::makePPF(q1,q2,0,1);  //also gets alpha-> planar rot angle to local coords


    //PointPairFeatures::computePose

    double alpha=-ppfModel.alpha+ppfScene.alpha; //TODO: works, but why so many abs??

    cout<<"ppfModel.alpha: "<<degrees(ppfModel.alpha)<<endl;

    cout<<"ppfScene.alpha: "<<degrees(ppfScene.alpha)<<endl;

    cout<<"alpha: "<<degrees(alpha)<<endl;

    //if(alpha<0) alpha+=M_PI*2;
    //cout<<"alpha positive: "<<alpha<<endl;


    //TODO wrong: this is just the translation, not rotation
    Projective3d Tgs(ppfScene.T.inverse());//==ppfScene.tra.inverse() * ppfScene.rot.inverse());

    AngleAxisd Rx(alpha, Vector3d::UnitX()); //TODO: in the end, we can only detect rotations around x axis... why???

    Projective3d Tmg(ppfModel.T); //==(ppfModel.rot * ppfModel.tra);

    Projective3d Pest(Tgs*Rx*Tmg);

    PointPairFeatures::printPose(Pest,"P_est:");

    PointPairFeatures::err(P,Pest);




}

