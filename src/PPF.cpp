//
//  PPF.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 02.08.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#include "PPF.h"
#include "Constants.h"
#include <eigen3/Eigen/Geometry>
#include <math.h>

//PPF2 PPF::makePPF2(const vector<Vector3f> &pts, const vector<Vector3f> &nor, int i, int j){
//    PPF2 ppf;
//    ppf.i=i;
//    //ppf.j=j;

//    Vector3f m1=pts[i];
//    Vector3f n1=nor[i];
//    Vector3f m2=pts[j];
//    Vector3f n2=nor[j];

//    Vector3f dist = m1-m2;
//    Vector3f dn=dist.normalized();

//    //discretise
//    uint8_t d,n1d,n2d,n1n2;  //all of these are <255
//    d = dist.norm()/Params::ddist; //Euclidean distance
//    n1d = acos(n1.dot(dn))/Params::dangle;
//    n2d = acos(n2.dot(dn))/Params::dangle;
//    n1n2 = acos(n1.dot(n2))/Params::dangle;

//    //hashkey
//    //return p(0)*ndist*ndist*ndist + p(1)*nangle*nangle + p(2)*nangle + p(3);  //TODO does it still work if nangle != ndist?
//    //return k.d + k.n1d*nangle + k.n2d*nangle*nangle + k.n1n2*nangle*nangle*nangle;  //ndist=20 must be smaller than nangle=30
//    ppf.k = (d | (n1d<<8) | (n2d<<16) | (n1n2<<24));

//    //calculate angle
//    ppf.alpha = planarRotAngle(m1,n1,m2);

//    return ppf;

//}

PPF::PPF(){
    //cout<<"default const"<<endl;
}

//Constructor
PPF::PPF(const vector<Vector3f> &pts, const vector<Vector3f> &nor, int i, int j){
    this->i=i;
    //this->j=j;

    //TODO
    //Vector<uint8_t,4> fd;

    Vector3f m1=pts[i];
    Vector3f n1=nor[i];
    Vector3f m2=pts[j];
    Vector3f n2=nor[j];

    Vector4f f = computePPF(m1,n1,m2,n2);

    //discretise
    uint8_t d,n1d,n2d,n1n2;  //all of these are <255
    d=f.x()/Params::getInstance()->ddist;
    n1d=f.y()/Params::getInstance()->dangle;
    n2d=f.z()/Params::getInstance()->dangle;
    n1n2=f.w()/Params::getInstance()->dangle;

    //hashkey
    //return p(0)*ndist*ndist*ndist + p(1)*nangle*nangle + p(2)*nangle + p(3);  //TODO does it still work if nangle != ndist?
    //return k.d + k.n1d*nangle + k.n2d*nangle*nangle + k.n1n2*nangle*nangle*nangle;  //ndist=20 must be smaller than nangle=30
    k = (d | (n1d<<8) | (n2d<<16) | (n1n2<<24));

    //calculate angle
    alpha = planarRotAngle(m1,n1,m2);
}

//F(m1, m2) = (∥d∥2, ∠(n1, d), ∠(n2, d), ∠(n1, n2)),
Vector4f PPF::computePPF(Vector3f m1, Vector3f n1, Vector3f m2, Vector3f n2){
    Vector4f f;
    Vector3f dist = m1-m2;
    Vector3f dn=dist.normalized();

    f.x() = dist.norm(); //Euclidean distance
    f.y() = acos(n1.dot(dn));
    f.z() = acos(n2.dot(dn));
    f.w() = acos(n1.dot(n2));

    return f;
}

void PPF::print(){
    //cout <<"{"<< i <<","<< j<<"} ";
    cout<<k<<endl;
    //cout<<d<<" "<<(180*n1d/M_PI)<<" "<<(180*n2d/M_PI)<<" "<<(180*n1n2/M_PI)<<endl;
}


bool PPF::operator==(const PPF &o) const{
    return k==o.k;//d==o.d && n1d==o.n1d && n2d==o.n2d && n1n2 == o.n1n2;
}

bool PPF::operator<(const PPF &o) const{
    return k < o.k;
}

//hashCode so we can insert this class in unordered_map as key directly
//int PPF::operator()(const PPF& k) const
unsigned int PPF::hashKey()
{
    return k;
}

/*

          Eigen::Vector3f model_point_transformed = transform_mg * model_point;
          float angle = atan2f ( -model_point_transformed(2), model_point_transformed(1));
          if (sin (angle) * model_point_transformed(2) < 0.0f)
            angle *= (-1);
          p.alpha_m = -angle;

          */
//cout<<"m1="<<m1<<" n1="<<n1<<endl;
//TODO: think about reuse
//Matrix4f Pm=T.matrix();

//std::cout<<Pm<<endl;

//std::cout<<Pm.size()<<endl;
//std::cout<<m1.homogeneous().size()<<endl;

//    Vector4f mm=m.homogeneous();
//    Vector4f nn=n.homogeneous();
//    nn(3)=0;


//Vector3f m22=m2.transpose(); //m2 only needed to get alpha
//cout<<"mm="<<(Pm*mm).transpose()<<endl;
//cout<<"nn="<<(Pm*nn).transpose()<<endl;

    //cout<<"mm2="<<m22P.transpose()<<endl;

//    if (sin (angle) * model_point_transformed(2) < 0.0f)
//      angle *= (-1);
//    p.alpha_m = -angle;


//cout<<"transformed"<<endl;
//cout<<result<<endl;
//cout<<"m1T="<<t*m<<" n1T="<<t*n<<endl;

//planartotangle

/*
 *  * #include <pcl/features/ppf.h>

 *           // Calculate alpha_m angle
          Eigen::Vector3f model_reference_point = input_->points[i].getVector3fMap (),
                          model_reference_normal = normals_->points[i].getNormalVector3fMap (),
                          model_point = input_->points[j].getVector3fMap ();
          float rotation_angle = acosf (model_reference_normal.dot (Eigen::Vector3f::UnitX ()));
          bool parallel_to_x = (model_reference_normal.y() == 0.0f && model_reference_normal.z() == 0.0f);
          Eigen::Vector3f rotation_axis = (parallel_to_x)?(Eigen::Vector3f::UnitY ()):(model_reference_normal.cross (Eigen::Vector3f::UnitX ()). normalized());
          Eigen::AngleAxisf rotation_mg (rotation_angle, rotation_axis);
          Eigen::Affine3f transform_mg (Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg);
*/

//cout<<"m="<<endl<<m.transpose()<<endl;
//cout<<"n="<<endl<<n.transpose()<<endl;

//Vector3f axis = (0.5*(n+xAxis)).transpose(); //Vector3f(1,0,0)
//cout<<"axis="<<endl<<axis.transpose()<<endl;
//AngleAxisf rot(M_PI, axis);
// Eigen::AngleAxisf rotation_mg (rotation_angle, rotation_axis);
// Eigen::Isometry3f transform_mg (Eigen::Translation3f ( rotation_mg * ((-1) * m)) * rotation_mg);
 //return transform_mg;

//TODO: same as half of both normals as axis, rotate for 180 degrees


