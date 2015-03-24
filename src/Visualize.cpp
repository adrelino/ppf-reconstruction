//
//  Visualize.cpp
//  PointPairFeatures
//
//  Created by Adrian Haarbach on 29.06.14.
//  Copyright (c) 2014 Adrian Haarbach. All rights reserved.
//
#include "Visualize.h"
#include "viz/cameraSimple.h"
#include "viz/frustum.h"
#include <iostream>

#include "LoadingSaving.h"

#include <iomanip>

using namespace std;

GLubyte colorEdge[4]={254,122,0,14};
GLubyte colorEdgeSel[4]={122,254,0,15};

void drawText(string s, Vector3f posMiddle){
    const unsigned char *text = (const unsigned char*) s.c_str();
    glRasterPos3fv(posMiddle.data());
    glutBitmapString(GLUT_BITMAP_HELVETICA_10,text);
}


Visualize* Visualize::instance = 0;

bool Visualize::isInitalized = false;

string filename = "keyToggle_";
string filename2 = "pose_";

void Visualize::readPose(char key){
    cout<<"Read pose "<<key<<endl;
    vector<float> pose = LoadingSaving::loadVectorf(filename2+key);
    if(pose.size()==7){
        angle=pose[0]; angle2=pose[1]; angle3=pose[2];
        offsetX=pose[3]; offsetY = pose[4]; offsetZ = pose[5];
        zoom = pose[6];
    }
}

void Visualize::readKeyToggle(char key){
    cout<<"Read key toggle "<<key<<endl;
    vector<bool> savedEntries = LoadingSaving::loadVector(filename+key);
    if(savedEntries.size()==256){
        std::copy(savedEntries.begin(),savedEntries.end(),std::begin(keyToggle));
    }
}

void Visualize::writePose(char key){
    cout<<"Save pose "<<key<<endl;

    vector<float> pose{angle,angle2,angle3,offsetX,offsetY,offsetZ,zoom};
    LoadingSaving::saveVectorf(filename2+key,pose);
}

void Visualize::writeKeyToggle(char key){
    cout<<"Save key toggle "<<key<<endl;

    vector<bool> savedEntries(keyToggle,keyToggle+256);
    LoadingSaving::saveVector(filename+key,savedEntries);
}

Visualize* Visualize::getInstance(){
    if(instance == 0){
        instance = new Visualize();
    }
    return instance;
}

Visualize::Visualize()
    : modifier(-1)
    , angle(0.0)
    , angle2(0.0)
    , angle3(0.0)
    , mouseButton(0)
    , zoom(15)
    , offsetY(-0.075f)
    , offsetX(0)
    , offsetZ()
    //, current_object(0)
{
    std::fill(std::begin(keyToggle), std::end(keyToggle), false);
    std::cout<<"Visualize construct"<<std::endl;
    keyToggle['s']=true;
    keyToggle['h']=true;
    keyToggle['c']=true;
    keyToggle['e']=true;
    keyToggle['a']=true;
    keyToggle['m']=true;
    keyToggle['l']=true; //lines for icp
    keyToggle['t']=true; //trajectory
    keyToggle['p']=true; //poses
    keyToggle['d']=true; //downsampled cloud
    keyToggle['r']=true; //keyToggle
    keyToggle['g']=true;
    S = Isometry3f::Identity();
    readKeyToggle('1');
}

//draws cylinder towards z direction
void Visualize::drawCylinderAdvanced(double r, double l, bool coverback, bool coverfront, bool normalInwards){
    int i;
	int n = 20;
    
	int z=normalInwards ? -1 : 1;
    
    for(i=0;i<2*n;i++){
        glBegin(GL_POLYGON);
        // Explanation: the normal of the whole polygon is the coordinate of the center of the polygon for a sphere
        glNormal3d(sin((i+0.5)*M_PI/n)*z,cos((i+0.5)*M_PI/n)*z,0); //middle
        glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),0);
        glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),0);
        glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),l);
        glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),l);
        glEnd();
        //bottom
        if(coverback){
            glBegin(GL_POLYGON);
            glNormal3d(0,0,-1*z); //middle
            glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),0);
            glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),0);
            glVertex3d(0,0,0);
            glEnd();
        }
        //top
        if(coverfront){
            glBegin(GL_POLYGON);
            glNormal3d(0,0,1*z); //middle
            glVertex3d(r*sin(i*M_PI/n),r*cos(i*M_PI/n),l);
            glVertex3d(r*sin((i+1)*M_PI/n),r*cos((i+1)*M_PI/n),l);
            glVertex3d(0,0,l);
            glEnd();
        }
    }
    
}

void Visualize::drawCylinder(double r, double l)
{
	drawCylinderAdvanced(r,l,true,true,false);
}

// for debugging, press o to display

void Visualize::drawOrigin(){
	glPushMatrix();
	glColor3f(0,0,1);
	drawCylinder(0.02,1); //z blue
	glRotatef(90,0,1,0);
	glColor3f(1,0,0);
	drawCylinder(0.02,1); //x  red
	glRotatef(-90,1,0,0);
	glColor3f(0,1,0);
	drawCylinder(0.02,1); //y  green
	glPopMatrix();
}

void Visualize::drawNormals(const PointCloud* m, Vector3f& color){
    if(m->nor.size()==0) return;
    glLineWidth(1);

    glBegin(GL_LINES);
    glColor3fv(color.data());
    for (int i=0; i<m->pts.size(); i++) {
        const Vector3f& p=m->pts[i];
        const Vector3f& n=m->nor[i];

        Vector3f q=p+n/100;
        
        glVertex3fv(p.data());
        glVertex3fv(q.data());
    }
	glEnd();
    
}

void Visualize::drawPointCloud(PointCloud* m, int i){

    if(keyToggle['g']) drawCameraPose(m->poseGroundTruth,i,Colormap::GREEN1,Colormap::GREEN2);  //those are fixed

    glPushMatrix();
        glMultMatrixf(S.matrix().data());

        if(m->fixed){
            drawCameraPose(m->pose,i,Colormap::MAG1,Colormap::MAG2);
        }else{
            drawCameraPose(m->pose,i,Colormap::RED1,Colormap::RED2);
        }

        Vector3f color(0.6,0.1,0);
        Vector3f colorNormals(0.8,0.2,0);

        if(selectedFrame==i){
            color = Colormap::BLUE1;
            colorNormals = (Colormap::BLUE2);
    //        if(keyToggle['l']){
    //            drawLines(m->src,m->dst);
    //        }
        }

        if(ingoingEdgeFrame==i){
            color = Colormap::ORANGE1;
            colorNormals = (Colormap::ORANGE2);
        }


        if(keyToggle['c'] || selectedFrame==i || ingoingEdgeFrame==i){
            glPushMatrix();
                glMultMatrixf(m->pose.matrix().data());
                if(keyToggle['d']){ //downsampled
                    if(keyToggle['C'] && m->cur.size() == m->pts.size()){
                        drawPoints(m->pts,m->getCurvColors(),6);
                    }else{
                        drawPoints(m->pts,color);
                    }
                    if(keyToggle['n']) drawNormals(m,colorNormals);
                }
                if(keyToggle['v']) drawCubes(m->pts,Params::getInstance()->ddist);
                if(keyToggle['O']){ //original point cloud
                    drawPoints(m->ptsOrig,color,1);
                }
                if(keyToggle['V']) drawCubes(m->ptsOrig,Params::getInstance()->ddist);

                if(keyToggle['b']){
                    int N=m->pts.size();
                    int j=refPtIdx;
                    if(j<0) j=0;
                    if(j>=N) j=N-1;
                    for(int i=0; i<N; i++){
                        drawPPF(j,i,m);
                    }
                }

            glPopMatrix();
        }

    glPopMatrix();

    //correct
    //drawPoints(m->getPtsInGlobalFrame(),Vector3f(0,1,0),pointSize+1);
}

void Visualize::drawPoints(const vector<Vector3f>& pts, const Vector3f& color, float pointSize){
    glPointSize(pointSize);
    glColor3fv(color.data());
    glBegin(GL_POINTS);
    for (int i=0; i<pts.size(); i++) {
        glVertex3fv(pts[i].data());
    }
	glEnd();
}

void Visualize::drawPoints(const vector<Vector3f>& pts, const vector<Vector3f>& colors, float pointSize){
    glPointSize(pointSize);
    glBegin(GL_POINTS);
    for (int i=0; i<pts.size(); i++) {
        glColor3fv(colors[i].data());
        glVertex3fv(pts[i].data());
    }
    glEnd();
}

void Visualize::setOffset(Vector3f offset){
    offsetX=offset(0);
    offsetY=offset(1);
    offsetZ=offset(2);
    cout<<"setOffset: "<<offset.transpose()<<endl;
}


void Visualize::drawPPF(int i, int j, const PointCloud* m)
{
    glLineWidth(0.05f);
    glBegin(GL_LINE_STRIP);
        const Vector3f& p=m->pts[i];
        const Vector3f& n=m->nor[i];
        const Vector3f& p2=m->pts[j];
        const Vector3f& n2=m->nor[j];
        Vector3f q=p+n/100;
        Vector3f q2=p2+n2/100;
    
        glColor3f(0.0,0.2,1);
        glVertex3f(q(0),q(1),q(2)); //first normal
        glVertex3f(p(0),p(1),p(2));
    
        glColor3f(0.8,0,0.8);
        glVertex3f(p2(0),p2(1),p2(2));  //distance line

        glColor3f(0.0,0.2,1);
        glVertex3f(q2(0),q2(1),q2(2));  //second normal
    
    glEnd();
}

//void Visualize::drawLines(const vector<int>& vertices)
//{
//    glLineWidth(0.05f);

//    glBegin(GL_LINES);
//    glColor3f(0.8,0,0.8);

//    int n = vertices.size();

//    for (int i = 0; i < n; ++i) {
//        const Vector3f& p1=scene->pts[i];
//        const Vector3f& p2=modelT->pts[vertices[i]];

//        glVertex3f(p1(0),p1(1),p1(2));  //distance line
//        glVertex3f(p2(0),p2(1),p2(2));  //distance line
//    }

//    glEnd();
//}
//#include <iomanip>
void Visualize::drawLines(const vector<Vector3f>& v1, const vector<Vector3f>& v2)
{
    glLineWidth(0.05f);

    int n = v1.size();

    for (int i = 0; i < n; ++i) {
        const Vector3f& p1=v1[i];
        const Vector3f& p2=v2[i];

        glBegin(GL_LINES);
        glColor3f(0.8,0,0.8);
        glVertex3fv(p1.data());  //distance line
        glVertex3fv(p2.data());  //distance line
        glEnd();
//        stringstream ss;
//        ss<<setprecision(2)<<(p1-p2).norm()*1000<<"mm";
//        drawText(ss.str(),0.5f*(p1+p2));

    }

}

void Visualize::drawCubes(const vector<Vector3f>& pts, double size){
    glColor3f(0.5,0.5,0.5);
    glLineWidth(0.8f);
    float ddist = Params::getInstance()->ddist;


    for (int i=0; i<pts.size(); i++) {
        glPushMatrix();
        float x=floor(pts[i].x()/ddist)*ddist+ddist/2.0;
        float y=floor(pts[i].y()/ddist)*ddist+ddist/2.0;
        float z=floor(pts[i].z()/ddist)*ddist+ddist/2.0;

        glTranslatef(x,y,z);
        glutWireCube(size);
        glPopMatrix();
    }
}

void Visualize::drawSpheres(const vector<Vector3f>& pts, double radius){
    glColor3f(0.9,0.9,0.9);
    for (int i=0; i<pts.size(); i++) {
        glPushMatrix();
        glTranslatef(pts[i].x(),pts[i].y(),pts[i].z());
        glutWireSphere(radius,5,5);
        glPopMatrix();
    }
}

void Visualize::drawFrustumIntrinsics(Vector4f colorLine, Vector4f colorPlane){
//    cv::Matx33d K = cv::Matx33d::zeros();
//    K(0,0) = 542;
//    K(1,1) = 540;
//    K(0,2) = 320;
//    K(1,2) = 240;
    double f_x =524;// K(0,0),
    double f_y = 540; //K(1,1),
    double c_x = 320; //K(1,2);
    double c_y = 240; //K(1,2);

    // Assuming that this is an ideal camera (c_y and c_x are at the center of the image)
    double fovy = 2.0 * atan2(c_y, f_y) * 180 / M_PI;
    double aspect_ratio = c_x / c_y;

    drawFrustum(fovy,aspect_ratio,-0.02f,colorLine,colorPlane);
}

double Visualize::getRotationAngleApprox(double xdiff, double ydiff, double x, double y){
	int xs=x>0 ? 1 : -1;
	int ys=y>0 ? 1 : -1;
	std::cout<<x<<"+"<<xdiff<<" "<<y<<"+"<<ydiff<<std::endl;
	return ydiff*xs+xdiff*ys;
}

void Visualize::drawCameraPose(Isometry3f& P,int i,Vector4f& colorLine,Vector4f& colorPlane){
    if(keyToggle['p']){
        glEnable(GL_STENCIL_TEST);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

        //Vector3f t = -P.inverse().translation();
        glPushMatrix();

        //glMultMatrixf(Isometry3f(P.linear()).matrix().data());
        //glTranslatef(t(0),t(1),t(2));

        glMultMatrixf(P.matrix().data());
        glStencilFunc(GL_ALWAYS, i, GL_ALL_ATTRIB_BITS);

        if(selectedFrame==i){
            drawFrustumIntrinsics(colorLine,Vector4f(0,0,0.5,0.5));
        }else{
            drawFrustumIntrinsics(colorLine,colorPlane);

        }
        glPopMatrix();
    }

    if(keyToggle['T']){
        std::stringstream ss;
        ss<<i;
        const unsigned char *text = (const unsigned char*) ss.str().c_str();
        Vector3f origin=Vector3f::Zero();
        Vector3f pos = P*origin;

        glColor4fv(colorLine.data());
        if(selectedFrame==i){ //dont know why we need -1 here, but it works
            //cout<<"sel: "<<selectedFrame<<" i"<<i<<" s1="<<cameraPoses.size()<<" s2="<<cameraPosesGroundTruth.size()<<endl;
            glColor3f(0,0,1);
        }
        glRasterPos3fv(pos.data());
        glStencilFunc(GL_ALWAYS, i, GL_ALL_ATTRIB_BITS);
        glutBitmapString(GLUT_BITMAP_HELVETICA_18,text);


//        glPushMatrix();
//            glMultMatrixf(P.matrix().data());
//            glRasterPos3f(0,0,0);
//            glColor3f(0,1,0);
//            if(selectedFrame-1==i){ //dont know why we need -1 here, but it works
//                //cout<<"sel: "<<selectedFrame<<" i"<<i<<" s1="<<cameraPoses.size()<<" s2="<<cameraPosesGroundTruth.size()<<endl;
//                glColor3f(1,0,0);
//            }
//            glStencilFunc(GL_ALWAYS, i, GL_ALL_ATTRIB_BITS);
//            glutBitmapString(GLUT_BITMAP_HELVETICA_18,text);
//        glPopMatrix();
    }
}

void Visualize::drawCameraPoses(vector<Isometry3f>& cameraPoses, Vector4f& colorLine, Vector4f& colorPlane){

    for(int i =0; i<cameraPoses.size(); i++){
        drawCameraPose(cameraPoses[i],i,colorLine,colorPlane);
    }


    if(keyToggle['t']){
        glBegin(GL_LINE_STRIP);
        glLineWidth(8);
        glColor4fv(colorLine.data());
        for(Isometry3f P : cameraPoses){
            Vector3f origin=Vector3f::Zero();
            Vector3f pos = P*origin;
            glVertex3fv(pos.data());
        }
        glEnd();
    }
}

void::Visualize::drawEdges(int i){
    shared_ptr<PointCloud>& v1=(*ms)[i];

    Vector3f origin=Vector3f::Zero();
    Vector3f pos1 = S*v1->pose*origin;

    for(int k=0; k<v1->neighbours.size(); k++){
        OutgoingEdge pair = v1->neighbours[k];
        int j=pair.neighbourIdx;

        shared_ptr<PointCloud>& v2=(*ms)[j];
        Vector3f pos2 = S*v2->pose*origin;



        Vector3f posMiddle(pos1*0.25f+pos2*0.75f);
        if(keyToggle['W']) posMiddle.y() += i>j ? 0.02f : -0.02f;

        if(keyToggle['w']){//weights
            stringstream ss;
            ss<</*setprecision(3)<<*/pair.weight;
            //ss<<pair.src.size();

            const unsigned char *text = (const unsigned char*) ss.str().c_str();

            if(i==selectedFrame && k==selectedOutgoingEdgeIdx){
                GLubyte color[4]={colorEdgeSel[0],colorEdgeSel[1],colorEdgeSel[2],(GLubyte) k};
                glColor4ubv(color);
            }else{
                GLubyte color[4]={colorEdge[0],colorEdge[1],colorEdge[2],(GLubyte) k};
                glColor4ubv(color);
            }
            glRasterPos3fv(posMiddle.data());
    //        if(selectedFrame-1==i){ //dont know why we need -1 here, but it works
    //            //cout<<"sel: "<<selectedFrame<<" i"<<i<<" s1="<<cameraPoses.size()<<" s2="<<cameraPosesGroundTruth.size()<<endl;
    //            glColor3f(0,0,1);
    //        }

            glStencilFunc(GL_ALWAYS, i, GL_ALL_ATTRIB_BITS);
            glutBitmapString(GLUT_BITMAP_HELVETICA_18,text);
        }



        glLineWidth(i==selectedFrame && k==selectedOutgoingEdgeIdx ? 3 : 1);

        glBegin(GL_LINE_STRIP);
        if(i==selectedFrame && k==selectedOutgoingEdgeIdx){
            glColor4fv(Colormap::MAG2.data());
        }else{
            glColor4fv(Colormap::MAG1.data());
        }
        glVertex3fv(pos1.data());
        glVertex3fv(posMiddle.data());
        glVertex3fv(pos2.data());
        glEnd();


        if(i==selectedFrame && k==selectedOutgoingEdgeIdx){
            vector<Vector3f> src,dst;
            for(auto corr : pair.correspondances){
                src.push_back(S*v1->pose*v1->pts[corr.first]);
                dst.push_back(S*v2->pose*v2->pts[corr.second]);
            }
            //drawPoints(pair.src,Colormap::BLUE1,6.0f);
            //drawPoints(pair.dst,Colormap::ORANGE1,6.0f);
            drawLines(src,dst);
        }
    }
}


void Visualize::display(void)
{
    glClearStencil(255); //is background for mouse clicks
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);




	glPushMatrix();
    glTranslatef(0, 0, -15);

    glRotatef(angle3, 0.0, 0.0, 1.0);
    glRotatef(angle2, 1.0, 0.0, 0.0);
    glRotatef(angle, 0.0, 1.0, 0.0);


    
    float z = zoom;
    glScalef(z,z,z);
    if(keyToggle['R']) drawOrigin(); //rotation center


    glTranslatef(offsetX, offsetY, offsetZ);

        
    if(keyToggle['o']) drawOrigin();
    
    if(ms){
        for(int i=0; i<ms->size(); i++){
            shared_ptr<PointCloud>& it=(*ms)[i];
            if(it){
                drawPointCloud(it.get(),i);
            }
        }
    }

    if(keyToggle['e'] && ms){ //edges in pose graph
        for(int i=0; i<ms->size(); i++){
            drawEdges(i);
        }
    }


	glPopMatrix();
    glutSwapBuffers();
}

void Visualize::displayW(void){
    instance->display();
}

void Visualize::bucketInfo(){
    //printMatches(matches);

    //Match match=matches[bucketIndex];
    //cout<<"bucketIndex= "<<bucketIndex<<"i="<<match.scenePPF.i<<"j="<<match.scenePPF.j<<"modelSize"<<match.modelPPFs.size()<<endl;
    
    
    
    //" key="<<b[bucketIndex].first<<" size="<<b[bucketIndex].second.size()<<endl;
}

void exitFun(){
    glutLeaveMainLoop();
    exit(0);
}

#include <iomanip>

void Visualize::keyboard (unsigned char key, int x, int y)
{
    int modifiers = glutGetModifiers();

    //modifiers & GLUT_ACTIVE_SHIFT
    bool ctrl = modifiers & GLUT_ACTIVE_CTRL;
    bool alt = modifiers & GLUT_ACTIVE_ALT;
    std::cout<<"keyboard: "<<key<< " at " << x <<"," << y;
    if(ctrl) cout<<" ctrl";
    if(alt) cout<<" alt ";
    cout<<endl;
    lastKey=key;

    keyToggle[key] = !keyToggle[key];

    if(functions[key]) functions[key]();

	switch (key) {
        case ' ':
            {   selectedFrame=255;
                selectedOutgoingEdgeIdx=255;
                ingoingEdgeFrame=255;
            }break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '0':
            if(alt){
                writeKeyToggle(key);
                writePose(key);
            }else{
                readKeyToggle(key);
                readPose(key);
            }break;
        case 'Q':
            exitFun();
            break;
        case 'K':
            S = Isometry3f::Identity();
            break;
        case '+':
            refPtIdx++;
            break;
        case '-':
            refPtIdx--;
        default:
            break;
	}
    
	glutPostRedisplay();
}

void Visualize::keyboardW (unsigned char key, int x, int y){
    instance->keyboard(key, x, y);
}

void Visualize::setS(Isometry3f S){
    instance->S=S;
}

void Visualize::mouse(int button, int state, int x, int y)
{
    //cout<<"bla"<<endl;
    //printf("button=%d %s At %d %d\n", button, (state == GLUT_DOWN) ? "Down" : "Up", x, y);
    
    modifier=glutGetModifiers();
    
    if (state == GLUT_DOWN) {
        int window_height = glutGet(GLUT_WINDOW_HEIGHT);

        GLubyte color[4];
        GLfloat depth;
        GLuint index;

        glReadPixels(x, window_height - y - 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, color);
        glReadPixels(x, window_height - y - 1, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
        glReadPixels(x, window_height - y - 1, 1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_INT, &index);


        if(depth<1){
            printf("Clicked on pixel %d, %d, color %02hhx%02hhx%02hhx%02hhx, depth %f, stencil index %u\n",
               x, y, color[0], color[1], color[2], color[3], depth, index);

            selectedFrame = index;

            if(ms && selectedFrame<ms->size()){
                cout<<"Frame: "<<selectedFrame<<" "<<(*(*ms)[index])<<endl;

                if(color[0]==colorEdge[0] && color[1]==colorEdge[1] && color[2]==colorEdge[2]){ //clicked on edge
                    selectedOutgoingEdgeIdx = color[3];
                    cout<<"neighbourVectorIdx: "<<selectedOutgoingEdgeIdx<<endl;
                    OutgoingEdge* selEdge=&(*(*ms)[selectedFrame]).neighbours[selectedOutgoingEdgeIdx];
                    cout<<"Edge: "<<selectedFrame<<"->"<<selEdge->neighbourIdx<<" with weight: "<<selEdge->weight<<endl;
                    ingoingEdgeFrame=selEdge->neighbourIdx;
                }else{
                    selectedOutgoingEdgeIdx=254;
                    ingoingEdgeFrame=254;
                }
            }



        }


        mouseButton = button;
        moving = 1;
        startx = x;
        starty = y;
    }
    if (state == GLUT_UP) {
        mouseButton = button;
        moving = 0;
    }
}

void Visualize::mouseW(int button, int state, int x, int y){
    instance->mouse(button, state, x, y);
}

void Visualize::motion(int x, int y)
{
    if (moving) {
        if (modifier==GLUT_ACTIVE_CTRL) { //Panning
            if(mouseButton==GLUT_LEFT_BUTTON){
                offsetY+=(y-starty)*0.001;
                offsetX+=(x-startx)*0.001;
            }else if(mouseButton==GLUT_MIDDLE_BUTTON || mouseButton==GLUT_RIGHT_BUTTON){
                offsetZ+=((x-startx)+(y-starty))*0.001;
            }
        }else{
            if(mouseButton==GLUT_LEFT_BUTTON){
//                if (modifier==GLUT_ACTIVE_CTRL) {
//                    offsetY+=(y-starty)*0.001;
//                    offsetX+=(x-startx)*0.001;
//                }else if (modifier==GLUT_ACTIVE_ALT){
//                    offsetZ+=(x-startx)*0.001;
//                }else{
                    angle = angle + (x - startx);
                    angle2 = angle2 + (y - starty);
//                }
            }else if(mouseButton==GLUT_MIDDLE_BUTTON || (mouseButton==GLUT_RIGHT_BUTTON && modifier==GLUT_ACTIVE_ALT)){ //if we dont have mousewheel
                int xCenter=x-(WINDOW_WIDTH/2);
                int yCenter=y-(WINDOW_HEIGHT/2);
                int startxCenter=startx-(WINDOW_WIDTH/2);
                int startyCenter=starty-(WINDOW_HEIGHT/2);
                angle3 -=(atan2(yCenter,xCenter)-atan2(startyCenter,startxCenter))*(180/M_PI); //rotate object around z axis with the angle corresponding to polar coordinates of mouse displacement
                //angle3 -= getRotationAngleApprox(xCenter-startxCenter,startyCenter-yCenter,xCenter,yCenter);
            }else if(mouseButton==GLUT_RIGHT_BUTTON){
                zoom += (((y-starty)+(x-startx))*0.1); //allows to mirror the object if zoom <0
            }
        }
        startx = x;
        starty = y;
        glutPostRedisplay();
    }
    
}

void Visualize::motionW(int x, int y){
    instance->motion(x, y);
}


int Visualize::mainVisualize(int argc, char **argv)
{
    
    //instance = this;
    
//	cout<<"Adrian's Point Cloud Visualizer"<< endl<< endl;
    
//	cout << "1-9: Draw different objects"<<endl;
//	cout << "Q: Quit" <<endl;
    
//	cout << "Left mouse click and drag: rotate the object"<<endl;
//	cout << "Right mouse click and drag: zooming"<<endl;
//    cout << "Control + Left mouse click and drag: zooming"<<endl;
        
    
	glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
    glutInitWindowSize (WINDOW_WIDTH, WINDOW_HEIGHT);
	glutInitWindowPosition (50, 50);
	glutCreateWindow ("Adrian's Point Cloud Visualizer");
    //glClearColor (.5,119/256.0,244/256.0, 0.5);
    glClearColor(1,1,1,1);

    
    
    //auto ddisp = std::mem_fn(&Visualize::display);
	glutDisplayFunc(displayW);
	glutMouseFunc(mouseW);
	glutMotionFunc(motionW);
    glutKeyboardFunc(keyboardW);

    glutCloseFunc(exitFun);

    //glutTimerFunc();
   // glutIdleFunc(idleW);
	//setupLighting();
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);


    
    glMatrixMode(GL_PROJECTION);
//       glLoadIdentity();
//       double w = glutGet( GLUT_WINDOW_WIDTH );
//       double h = glutGet( GLUT_WINDOW_HEIGHT );
//       double ar = w / h;
//       glOrtho( -2 * ar, 2 * ar, -2, 2, -1, 1);

       gluPerspective( /* field of view in degree */ 40.0,
                   /* aspect ratio */ 1.0,
                   /* Z near */ 1.0, /* Z far */ 80.0);

	glMatrixMode(GL_MODELVIEW);


    
    
    
    //glutMainLoop();

    isInitalized=true;
    
	return 0;
}


void Visualize::visualize(){
    getInstance()->mainVisualize(0,0);
}

void Visualize::start(){
    cout<<"start Visualize thread "<<endl;
    getInstance()->visThread = new std::thread(visualize);
}

void Visualize::waitKeyQuit(){
    getInstance()->visThread->join();
}

void Visualize::spin(){
    if(!isInitalized) visualize();
    while(Visualize::waitKey('q')){
        glutPostRedisplay();
        glutMainLoopEvent();
    }
}

void Visualize::spinLast(){
    if(!isInitalized) visualize();
    while(Visualize::waitKey('Q')){
        glutPostRedisplay();
        glutMainLoopEvent();
    }
}

void Visualize::spin(int i){
    if(!isInitalized) visualize();
    while(i-- > 0){
        std::chrono::milliseconds dura( 5 );
        std::this_thread::sleep_for( dura );
        glutPostRedisplay();
        glutMainLoopEvent();
    }
}

void Visualize::spinToggle(int i){
    if(!isInitalized) visualize();
    if(getInstance()->keyToggle['r']){
        spin(i);
    }else{
        spin();
    }
}

bool Visualize::waitKeyInst(unsigned char key){
    //cout<<"waiting for "<<key<<endl;
    std::chrono::milliseconds dura( 10 );
    //while(true){
        std::this_thread::sleep_for( dura );
        //sleep(1);
        if(lastKey==key){
            lastKey=-1;
            //cout<<"ok, exit waiting = "<<endl;
            return false;
        }else{
            return true;
        }
    //}
}

bool Visualize::waitKey(unsigned char key){
    return getInstance()->waitKeyInst(key);
}

void Visualize::setClouds(vector< shared_ptr<PointCloud> >* mypair){
    getInstance()->ms=mypair;
}

void Visualize::setSelectedIndex(int i){
    getInstance()->selectedFrame=i;
    if(i<getInstance()->ms->size()){
        PointCloud& cloud = *(getInstance()->ms->at(i));
        if(cloud.neighbours.size()>0){
            getInstance()->ingoingEdgeFrame=cloud.neighbours[0].neighbourIdx;
        }
    }
}

void Visualize::setCallbackForKey(char key, std::function<void ()> f){
    getInstance()->functions[key]=f;
}

void Visualize::simulateKeypress(char key){
    if(getInstance()->functions[key]){
        getInstance()->functions[key]();
    }else{
        cout<<"Visualize: no func on key: "<<key<<endl;
    }
}
