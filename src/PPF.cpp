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

PPF PPF::makePPF(RowVectorXf p1,RowVectorXf p2, int i, int j){
    PPF p;
    p.i=i;
    p.j=j;
    p.pointPairFeature(p1.head(3), p2.head(3), p1.tail(3), p2.tail(3));
    return p;
}

//F(m1, m2) = (∥d∥2, ∠(n1, d), ∠(n2, d), ∠(n1, n2)),
void PPF::pointPairFeature(RowVector3f m1,RowVector3f m2,RowVector3f n1,RowVector3f n2){
    this->m1=m1;this->m2=m2;this->n1=n1;this->n2=n2;
    
    RowVector3f dist = m2-m1;
    RowVector3f dn=dist.normalized(), n1n=n1.normalized(), n2n=n2.normalized();
    
    _d= dist.norm(); //Euclidean distance
    _n1d=acos(n1n.dot(dn));
    _n2d=acos(n2n.dot(dn));
    _n1n2=acos(n1n.dot(n2n));
    
    //discretise
    d=_d/ddist;
    n1d=_n1d/dangle;
    n2d=_n2d/dangle;
    n1n2=_n1n2/dangle;
    
    planarRotAngle();
}

void PPF::print(){
    cout <<"{"<< i <<","<< j<<"} ";
    cout<<_d<<" "<<(180*_n1d/M_PI)<<" "<<(180*_n2d/M_PI)<<" "<<(180*_n1n2/M_PI)<<endl;
}


bool PPF::operator==(const PPF &o) const{
    return d==o.d && n1d==o.n1d && n2d==o.n2d && n1n2 == o.n1n2;
}

//hashCode so we can insert this class in unordered_map as key directly
std::size_t PPF::operator()(const PPF& k) const
{
    //return p(0)*ndist*ndist*ndist + p(1)*nangle*nangle + p(2)*nangle + p(3);  //TODO does it still work if nangle != ndist?
    
    return k.d + k.n1d*nangle + k.n2d*nangle*nangle + k.n1n2*nangle*nangle*nangle;  //ndist=20 must be smaller than nangle=30
}

void PPF::planarRotAngle(){
    //cout<<"m1="<<m1<<" n1="<<n1<<endl;
    Vector3f m=m1.transpose();
    Vector3f n=n1.transpose().normalized();

    Isometry3f T=twistToLocalCoords(m,n); //TODO: think about reuse
    Matrix4f Pm=T.matrix();
    
    //std::cout<<Pm<<endl;
    
    //std::cout<<Pm.size()<<endl;
    //std::cout<<m1.homogeneous().size()<<endl;
    
    Vector4f mm=m.homogeneous();
    Vector4f nn=n.homogeneous();
    nn(3)=0;
    

    Vector3f m22=m2.transpose(); //m2 only needed to get alpha
    //cout<<"mm="<<(Pm*mm).transpose()<<endl;
    //cout<<"nn="<<(Pm*nn).transpose()<<endl;
    Vector4f m22P=Pm*m22.homogeneous();
    //cout<<"mm2="<<m22P.transpose()<<endl;
    
    alpha=atan2(m22P(2), m22P(1)); //can ignore x coordinate, since we rotate around x axis, x coord has to be same for model and matched scene point m2 and s2

    

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
