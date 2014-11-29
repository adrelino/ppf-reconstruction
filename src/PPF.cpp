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

//Constructor
PPF::PPF(PointCloud C, int i, int j){
    this->i=i;
    this->j=j;

    //TODO
    //Vector<uint8_t,4> fd;

    Vector3f m1=C.pts[i];
    Vector3f n1=C.nor[i];
    Vector3f m2=C.pts[j];
    Vector3f n2=C.nor[j];

    Vector4f f = computePPF(m1,n1,m2,n2);

    //discretise
    d=f.x()/ddist;
    n1d=f.y()/dangle;
    n2d=f.z()/dangle;
    n1n2=f.w()/dangle;

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
    cout<<d<<" "<<(180*n1d/M_PI)<<" "<<(180*n2d/M_PI)<<" "<<(180*n1n2/M_PI)<<endl;
}


bool PPF::operator==(const PPF &o) const{
    return d==o.d && n1d==o.n1d && n2d==o.n2d && n1n2 == o.n1n2;
}

//hashCode so we can insert this class in unordered_map as key directly
//int PPF::operator()(const PPF& k) const
int PPF::hashKey()
{
    //return p(0)*ndist*ndist*ndist + p(1)*nangle*nangle + p(2)*nangle + p(3);  //TODO does it still work if nangle != ndist?
    
    //return k.d + k.n1d*nangle + k.n2d*nangle*nangle + k.n1n2*nangle*nangle*nangle;  //ndist=20 must be smaller than nangle=30

    int hashKey = (d | (n1d<<8) | (n2d<<16) | (n1n2<<24));

    return hashKey;
}

double PPF::planarRotAngle(Vector3f m, Vector3f n, Vector3f m2){
    //cout<<"m1="<<m1<<" n1="<<n1<<endl;

    Isometry3f T=twistToLocalCoords(m,n); //TODO: think about reuse
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
    Vector3f m22P=T*m2;
    //cout<<"mm2="<<m22P.transpose()<<endl;
    
    double alpha=atan2(m22P(2), m22P(1)); //can ignore x coordinate, since we rotate around x axis, x coord has to be same for model and matched scene point m2 and s2
    return alpha;
    

    //cout<<"transformed"<<endl;
    //cout<<result<<endl;
    //cout<<"m1T="<<t*m<<" n1T="<<t*n<<endl;

}

Isometry3f PPF::twistToLocalCoords(Vector3f m, Vector3f n){
    //cout<<"m="<<endl<<m.transpose()<<endl;
    //cout<<"n="<<endl<<n.transpose()<<endl;

    Vector3f xAxis(1,0,0);

    //Vector3f axis = (0.5*(n+xAxis)).transpose(); //Vector3f(1,0,0)
    //cout<<"axis="<<endl<<axis.transpose()<<endl;
    //AngleAxisf rot(M_PI, axis);

    double angle = acos(xAxis.dot(n));
    Vector3f orthogonalAxis = (n.cross(xAxis).normalized());  //TODO: same as half of both normals as axis, rotate for 180 degrees


    AngleAxisf rot(angle, orthogonalAxis);
    Translation3f tra(-m);


    return Isometry3f(rot*tra);
}
