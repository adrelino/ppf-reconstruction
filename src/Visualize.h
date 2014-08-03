//
//  Visualize.h
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 29.06.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//

#ifndef __PointPairFeatures__Visualize__
#define __PointPairFeatures__Visualize__

#include <iostream>
#include <math.h>
#include <vector>

//WINDOWS
//#include <Windows.h>
//#define M_PI 3.141592654
//#include "GL\glut.h"

//MAC
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>

//LINUX
//#include "GL/freeglut.h"
//#include "GL/gl.h"

#include <eigen3/Eigen/Dense>
#include "Constants.h"

#define glRED glColor3f(0.0,1,0.2);


using namespace Eigen;

using namespace std;


#endif /* defined(__PointPairFeatures__Visualize__) */


class Visualize {
    // global variable
    // pi value can be accessed using the built-in M_PI
    bool m_Smooth = false;
    bool m_Highlight = false;
    bool m_ColorMaterial = true;
    bool m_Emission = false;
    bool m_Ambient = true; //use white as ambient color, not the one set by colorMaterial
    bool m_Origin = false;
    bool m_Normals = false;
    bool m_Buckets = false;
    bool m_Voxels = false;

    int modifier = -1;
    
    GLfloat angle = 0;   /* in degrees */
    GLfloat angle2 = 0;   /* in degrees */
    GLfloat angle3 = 0;   /* in degrees */
    GLfloat zoom = 15.0;
    int mouseButton = 0;
    int moving, startx, starty;
    
    GLfloat offsetY = -0.075f, offsetX = 0;
    
    const static int WINDOW_SIZE = 1024;
    int current_object = 0;
    
private:
    Visualize(){};
    
    
    
    void bucketInfo();
    static Visualize *instance;
    
    MatrixXd m;
    
    
public:
    
    vector< pair<MatrixXd, RowVector3f> > ms;

    KeyBucketPairList b;
    int bucketIndex=0;
    
    MatrixXd model,scene,modelT;
    Matches matches;
    int matchIndex=0;



    
    static void visualize();
    static Visualize* getInstance();
    
    
    
    void glColorHex(int rgbHex);
    
    
    void drawCylinderAdvanced(double r, double l, bool coverback, bool coverfront, bool normalInwards);
    void drawCylinder(double r, double l);
    void drawOrigin();
    
    void drawNormals(MatrixXd m,RowVector3f color);
    void drawPointCloud(MatrixXd m, RowVector3f color);
    void drawPPF(int i, int j, MatrixXd m);
    void drawPPfs(Bucket, MatrixXd m);
    void drawCubes(MatrixXd C, double size);
    void drawMatches(Matches matches);
    void printMatches(Matches matches);

    
    void drawAll(MatrixXd m, RowVector3f color, RowVector3f colorNormals);


    
    void drawEllipticParaboloid(double max, double a, double b, double c, bool backSideNormal);
    void drawHyperbolicParaboloid(double maxx, double maxy, double a, double b, double c, bool backSideNormal);
    
    double getRotationAngleApprox(double xdiff, double ydiff, double x, double y);

    
    void display(void);
    void keyboard (unsigned char key, int x, int y);
    void mouse(int button, int state, int x, int y);
    void motion(int x, int y);
    
    static void displayW(void);
    static void keyboardW (unsigned char key, int x, int y);
    static void mouseW(int button, int state, int x, int y);
    static void motionW(int x, int y);
    
    
    int mainVisualize(int argc, char **argv);
  
};