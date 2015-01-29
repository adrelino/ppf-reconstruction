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

using namespace std;


Visualize* Visualize::instance = 0;

bool Visualize::isInitalized = false;

string filename = "keyToggle";

void Visualize::readPose(){
    cout<<"Read current viewing pose"<<endl;
    Vector3f angles= LoadingSaving::loadVector3f("viz-angles.txt");
    angle=angles.x(); angle2=angles.y(); angle3=angles.z();
    Vector3f zoomAndTransl = LoadingSaving::loadVector3f("viz-zoomAndTransl.txt");
    zoom = zoomAndTransl.x(); offsetX=zoomAndTransl.y(); offsetY = zoomAndTransl.z();

    vector<bool> savedEntries = LoadingSaving::loadVector(filename);
    cout<<"keyToggleReadSize: "<<savedEntries.size()<<endl;
    std::copy(savedEntries.begin(),savedEntries.end(),std::begin(keyToggle));
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
    //, current_object(0)
{
    std::fill(std::begin(keyToggle), std::end(keyToggle), false);
    std::cout<<"Visualize construct"<<std::endl;
    keyToggle['s']=true;
    keyToggle['h']=true;
    keyToggle['c']=true;
    keyToggle['e']=true;
    keyToggle['a']=true;
    keyToggle['o']=true;
    keyToggle['m']=true;
    keyToggle['l']=true; //lines for icp
    keyToggle['t']=true; //trajectory
    keyToggle['p']=true; //poses
    readPose();
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
    glLineWidth(0.02f);
    bool colorPerVertex=false;

    int nCol=m->nor_color.size();
    if(nCol>1 && nCol==m->pts.size()){
       colorPerVertex=true;
    }else if(nCol==1){
       color = m->nor_color[0];
    }

    glBegin(GL_LINES);
    glColor3fv(color.data());
    for (int i=0; i<m->pts.size(); i++) {
        const Vector3f& p=m->pts[i];
        const Vector3f& n=m->nor[i];

        Vector3f q=p+n.normalized()/100;
        
        if(colorPerVertex) glColor3fv(m->nor_color[i].data());

        glVertex3fv(p.data());
        glVertex3fv(q.data());
    }
	glEnd();
    
}

void Visualize::drawPointCloud(PointCloud* m, int i){

    drawCameraPose(m->pose,i,Colormap::RED1,Colormap::RED2);
    if(keyToggle['g']) drawCameraPose(m->poseGroundTruth,i,Colormap::GREEN1,Colormap::GREEN2);

    Vector3f color(0.6,0.1,0);
    Vector3f colorNormals(0.8,0.2,0);

    if(selectedFrame==i){
        color = Colormap::BLUE1;
        colorNormals = (Colormap::BLUE2);
    }

    glPushMatrix();
        glMultMatrixf(m->pose.matrix().data());
        drawPoints(m->pts,color);
        if(keyToggle['n']) drawNormals(m,colorNormals);
        if(keyToggle['v']) drawCubes(m->pts,ddist);

    glPopMatrix();



    //correct
    //drawPoints(m->getPtsInGlobalFrame(),Vector3f(0,1,0),pointSize+1);
}

//draws a square r/2 in front of x y plane with normal facing towards viewer;
void Visualize::drawPoints(const vector<Vector3f>& pts, const Vector3f& color, float pointSize){
    glPointSize(pointSize);
//    bool colorPerVertex=true;
//    if(color.size()==1){
//        colorPerVertex=false;
        glColor3fv(color.data());
//    }
    glBegin(GL_POINTS);
    for (int i=0; i<pts.size(); i++) {
        //if(colorPerVertex) glColor3fv(color[i].data());
        glVertex3fv(pts[i].data());
    }
	glEnd();
}


//void Visualize::drawPPF(int i, int j, const PointCloud& m)
//{
//    glLineWidth(0.05f);
//    glBegin(GL_LINE_STRIP);
//        Vector3f& p=m.pts[i];
//        Vector3f& n=m.nor[i];
//        Vector3f& p2=m.pts[j];
//        Vector3f& n2=m.nor[j];
//        n.normalize();
//        n/=100;
//        n2.normalize();
//        n2/=100;
//        Vector3f q=p+n;
//        Vector3f q2=p2+n2;
    
//        glColor3f(0.0,0.2,1);
//        glVertex3f(q(0),q(1),q(2)); //first normal
//        glVertex3f(p(0),p(1),p(2));
    
//        glColor3f(0.8,0,0.8);
//        glVertex3f(p2(0),p2(1),p2(2));  //distance line

//        glColor3f(0.0,0.2,1);
//        glVertex3f(q2(0),q2(1),q2(2));  //second normal
    
//	glEnd();
//}

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

void Visualize::drawLines(const vector<Vector3f>& v1, const vector<Vector3f>& v2)
{
    glLineWidth(0.05f);

    glBegin(GL_LINES);
    glColor3f(0.8,0,0.8);

    int n = v1.size();

    for (int i = 0; i < n; ++i) {
        const Vector3f& p1=v1[i];
        const Vector3f& p2=v2[i];

        glVertex3fv(p1.data());  //distance line
        glVertex3fv(p2.data());  //distance line
    }

    glEnd();
}

//void Visualize::drawPPfs(Bucket& b,PointCloud& m){
////    for (auto it : b){
////        drawPPF(it.i, it.j,m);
////    }
//}

void Visualize::drawCubes(const vector<Vector3f>& pts, double size){
    glColor3f(0.9,0.9,0.9);
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

//void Visualize::drawMatches(Matches& matches){
//    Match match=matches[bucketIndex];
//    //drawPPfs(match.modelPPFs, model);
//    //drawPPF(match.scenePPF.i, match.scenePPF.j, scene);
//}

//void Visualize::printMatches(Matches matches){
//    Match match=matches[bucketIndex];
    
//    cout<<"scene:";
//    match.scenePPF.print();
//    cout<<"model:";
//    match.modelPPFs[0].print();
//    /*for (auto it : match.modelPPFs) {
//        it.ppf.print();
//    }*/
//    cout<<endl;
//}

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

        glRasterPos3fv(pos.data());
        glColor3f(1,1,1);
        if(selectedFrame-1==i){ //dont know why we need -1 here, but it works
            //cout<<"sel: "<<selectedFrame<<" i"<<i<<" s1="<<cameraPoses.size()<<" s2="<<cameraPosesGroundTruth.size()<<endl;
            glColor3f(0,0,1);
        }
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
    Vector3f pos = v1->pose*origin;
    auto mat = pos.data();

    glBegin(GL_LINES);

    if(selectedFrame==i){
        glLineWidth(10);
        glColor3fv(Colormap::BLUE1.data());
    }else{
        glLineWidth(5);
        glColor4fv(Colormap::RED1.data());
    }

    for(int j : v1->neighbours){
        if(j==selectedFrame) continue; //should be blue, not red, only draw once
        glVertex3fv(mat);
        shared_ptr<PointCloud>& v2=(*ms)[j];
        Vector3f pos2 = v2->pose*origin;
        glVertex3fv(pos2.data());
    }
    glEnd();

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
    
    glScalef(zoom,zoom,zoom);
    
    glTranslatef(offsetX, offsetY, 0);
        
    if(keyToggle['o']) drawOrigin();
    
    //glRED;
    //drawPointCloud(ms.at(current_object).first,ms.at(current_object).second);
    

            //if(cameraPoses) drawCameraPoses(*cameraPoses,Colormap::RED1,Colormap::RED2);
            //if(keyToggle['g'] && cameraPosesGroundTruth) drawCameraPoses(*cameraPosesGroundTruth,Colormap::GREEN1,Colormap::GREEN2);

            //if(keyToggle['m'] && model && model->pts.size()>0) drawAll(*model,Map<Vector3f>(Colormap::GREEN2.data()),Map<Vector3f>(Colormap::GREEN1.data())); //green
            //if(keyToggle['m'] && model.pts.size()>0) drawAll(model,Vector3f(1,0,0),Vector3f(0,1,0.2)); //red green
            //if(keyToggle['s'] && scene && scene->pts.size()>0) drawAll(*scene,Vector3f(1,0.5,0),Vector3f(0,1,1)); //orange blue
            //if(keyToggle['e'] && modelT && modelT->pts.size()>0) drawAll(*modelT,Vector3f(1,0,1),Vector3f(1,1,1));//magenta

            //if(matches.size()>0) drawMatches(matches);
            //if(keyToggle['l'] && closestPtsSceneToModel.size()>0) drawLines(closestPtsSceneToModel);
            if(keyToggle['l'] && src && dst && src->size()>0) drawLines( (*src) ,(*dst) );

            if(keyToggle['c'] && ms){
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



	glutSwapBuffers ();
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


void Visualize::keyboard (unsigned char key, int x, int y)
{
    //std::cout<<"keyboard: "<<key<< " at " << x << y <<std::endl;
    lastKey=key;

    keyToggle[key] = !keyToggle[key];

	switch (key) {
        case 'p':
        case 'P':
            //printMatches(matches);
            break;
        case 'w':
        case 'W':
            glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
            break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
            //current_object = key - '1';
            //m=ms.at(current_object).first;
            break;
            
        case 'Q':
            exitFun();
            break;
        case 'q':
            break;
        case '+':
            bucketIndex++;
            bucketInfo();
            break;
        case '-':
            if(bucketIndex>=1) bucketIndex--;
            bucketInfo();
            break;
        case 'S': {
            cout<<"Save current viewing pose and keyToggleState"<<endl;
            Vector3f angles(angle,angle2,angle3);
            Vector3f zoomAndTransl(zoom,offsetX,offsetY);
            LoadingSaving::saveMatrixXf("viz-angles.txt",angles);
            LoadingSaving::saveMatrixXf("viz-zoomAndTransl.txt",zoomAndTransl);



            vector<bool> savedEntries(keyToggle,keyToggle+256);
            LoadingSaving::saveVector(filename,savedEntries);
            cout<<"keyToggleSaveSize: "<<savedEntries.size()<<endl;
            }break;
        case 'R': {
            readPose();
            }break;
        default:
            break;
	}
    
	glutPostRedisplay();
}

void Visualize::keyboardW (unsigned char key, int x, int y){
    instance->keyboard(key, x, y);
}

void Visualize::mouse(int button, int state, int x, int y)
{
    //cout<<"bla"<<endl;
    //printf("button=%d %s At %d %d\n", button, (state == GLUT_DOWN) ? "Down" : "Up", x, y);
    
    modifier=glutGetModifiers();
    
    if (state == GLUT_DOWN) {
        int window_height = glutGet(GLUT_WINDOW_HEIGHT);

        GLbyte color[4];
        GLfloat depth;
        GLuint index;

        glReadPixels(x, window_height - y - 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, color);
        glReadPixels(x, window_height - y - 1, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
        glReadPixels(x, window_height - y - 1, 1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_INT, &index);

        if(depth<1){
            if(selectedFrame!=255){
               poseDiff((*(*ms)[selectedFrame]).pose,(*(*ms)[index]).pose);
            }

            selectedFrame = index;
            if(ms){
                cout<<"Frame "<<selectedFrame<<" "<<(*(*ms)[selectedFrame])<<endl;
            }

            //printf("Clicked on pixel %d, %d, color %02hhx%02hhx%02hhx%02hhx, depth %f, stencil index %u\n",
              // x, y, color[0], color[1], color[2], color[3], depth, index);
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
        if(mouseButton==GLUT_LEFT_BUTTON)
        {
            if (modifier==GLUT_ACTIVE_CTRL) {
                offsetY-=(y-starty)*0.001;
                offsetX+=(x-startx)*0.001;
            }else{
                angle = angle + (x - startx);
                angle2 = angle2 + (y - starty);
            }
        }else if(mouseButton==GLUT_MIDDLE_BUTTON || (mouseButton==GLUT_RIGHT_BUTTON && modifier==GLUT_ACTIVE_ALT)){ //if we dont have mousewheel
        	int xCenter=x-(WINDOW_SIZE/2);
        	int yCenter=y-(WINDOW_SIZE/2);
        	int startxCenter=startx-(WINDOW_SIZE/2);
        	int startyCenter=starty-(WINDOW_SIZE/2);
        	angle3 -=(atan2(yCenter,xCenter)-atan2(startyCenter,startxCenter))*(180/M_PI); //rotate object around z axis with the angle corresponding to polar coordinates of mouse displacement
            //angle3 -= getRotationAngleApprox(xCenter-startxCenter,startyCenter-yCenter,xCenter,yCenter);
        }else if(mouseButton==GLUT_RIGHT_BUTTON){
        	zoom += (((y-starty)+(x-startx))*0.1); //allows to mirror the object if zoom <0
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
	glutInitWindowSize (WINDOW_SIZE, WINDOW_SIZE);
	glutInitWindowPosition (50, 50);
	glutCreateWindow ("Adrian's Point Cloud Visualizer");
    glClearColor (.5,119/256.0,244/256.0, 0.5);


    
    
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


//void Visualize::setModel(PointCloud& m){
//    getInstance()->model=&m;
//}

//void Visualize::setScene(PointCloud& s){
//    getInstance()->scene=&s;
//}

//void Visualize::setModelTransformed(PointCloud& mT){
//    getInstance()->modelT=&mT;
//}

void Visualize::setLines(shared_ptr< vector<Vector3f> > src, shared_ptr< vector<Vector3f> > dst){
    getInstance()->src=src;
    getInstance()->dst=dst;
}

//void Visualize::addCloud(PointCloud& &mypair){
//    getInstance()->ms.push_back(mypair);
//}

//void Visualize::setLastCloud(PointCloud& &mypair){
//    getInstance()->ms.back()=mypair;
//}

void Visualize::setClouds(vector< shared_ptr<PointCloud> >* mypair){
    getInstance()->ms=mypair;
}

//void Visualize::addCameraPose(Isometry3f pose){
//    getInstance()->cameraPoses.push_back(pose);
//}

//void Visualize::setLastCameraPose(Isometry3f pose){
//    getInstance()->cameraPoses.back()=pose;
//}

//void Visualize::setCameraPoses(vector<Isometry3f>& poses){
//    getInstance()->cameraPoses=&poses;
//}

//void Visualize::addCameraPoseGroundTruth(Isometry3f pose){
//    getInstance()->cameraPosesGroundTruth.push_back(pose);
//}

//void Visualize::setLastCameraPoseGroundTruth(Isometry3f pose){
//    getInstance()->cameraPosesGroundTruth.back()=pose;
//}

//void Visualize::setCameraPosesGroundTruth(vector<Isometry3f>& poses){
//    getInstance()->cameraPosesGroundTruth=&poses;
//}

void Visualize::setSelectedIndex(int i){
    getInstance()->selectedFrame=i;
}
