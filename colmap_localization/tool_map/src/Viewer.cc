#include "Viewer.h"

namespace Ulocal{

void ViewerLY::View()
{
    testpangolin();
}

void ViewerLY::testpangolin()
{
    pangolin::CreateWindowAndBind("Colmap Ulocal: Map Viewer",1224,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(200));
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowFused("menu.Show Fused map",false,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<float> menu_sizepoint("menu. Point Size",2.0,1.0,50);
    pangolin::Var<float> menu_camerasize("menu. Camera Size",0.05,0.05,0.20);
    pangolin::Var<float> menu_cameralinewidth("menu. Camera Line Width",1.0,1.0,5.0);
    pangolin::Var<bool> menuShowPlane("menu.Show Grid Plane",true,true);
    pangolin::Var<int> menu_gridnumber("menu. Grid Element Number",25.0,3.0,40.0);
    pangolin::Var<float> menu_gridelementsize("menu. Grid Element Size",0.2,0.1,2.0);
    pangolin::Var<float> menu_gridheight("menu. Grid height",1.38,-5.0,5.0);
/*
    pangolin::Var<bool> menuShowObject("menu.Show Object",false,true);
    pangolin::Var<float> menu_objectsize("menu. Object size",5.0,0.1,10);
*/

/*
    //pangolin::CreatePanel("menul").SetBounds(0.0,0.5,0.0,pangolin::Attach::Pix(200));
    pangolin::Var<bool> menu_buttonXplus("menul.X plus",false,false);
    pangolin::Var<bool> menu_buttonXminus("menur.X minus",false,false);

    pangolin::Var<bool> menu_buttonZplus("menul.Z plus",false,false);
    pangolin::Var<bool> menu_buttonZminus("menur.Z minus",false,false);

    pangolin::Var<bool> menu_buttonYplus("menul.rotate Y plus",false,false);
    pangolin::Var<bool> menu_buttonYminus("menur.rotate Y minus",false,false);
*/

    //pangolin::CreatePanel("menur").SetBounds(0.0,
    //                       0.5,pangolin::Attach::Pix(100),pangolin::Attach::Pix(200));

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
 

    const GLfloat light_ambient[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
    const GLfloat light_diffuse[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat light_position[] = { 2.0f, -5.0f, 15.0f, 0.0f };

    const GLfloat mat_ambient[]    = { 0.0f, 0.0f, 0.0f, 1.0f };
    const GLfloat mat_diffuse[]    = { 0.8f, 0.8f, 0.8f, 1.0f };
    const GLfloat mat_specular[]   = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat high_shininess[] = { 1.0f };

    glCullFace(GL_BACK);
    glDepthFunc(GL_LESS);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glMaterialfv(GL_FRONT, GL_AMBIENT,   mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,   mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);  

    while(!pangolin::ShouldQuit())
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        d_cam.Activate(s_cam);

        mKeyFrameSize = menu_camerasize;
        mCameraLineWidth = menu_cameralinewidth;
        ndivs = menu_gridnumber;
        ndivsize = menu_gridelementsize;
        height = menu_gridheight;
        mPointSize = menu_sizepoint;

/*
        if( pangolin::Pushed(menu_buttonXplus) )
             objectX += 0.1;

        if( pangolin::Pushed(menu_buttonXminus) )
             objectX -= 0.1;

        if( pangolin::Pushed(menu_buttonZplus) )
             objectZ += 0.1;

        if( pangolin::Pushed(menu_buttonZminus) )
             objectZ -= 0.1;

        if( pangolin::Pushed(menu_buttonYplus) )
             rotateY += 0.1;

        if( pangolin::Pushed(menu_buttonYminus) )
             rotateY -= 0.1;
*/
        //std::cout << objectX << " " << objectZ << std::endl;

        if(menuShowPoints)
        {
            // draw map points
            glPointSize(mPointSize);
            glBegin(GL_POINTS);

            for(size_t i=0, iend=mapPoints.size(); i<iend;i++)
            {
            
                float xpt = mapPoints[i].XYZ()(0);
                float ypt = mapPoints[i].XYZ()(1);
                float zpt = mapPoints[i].XYZ()(2);
                float rcolor = mapPoints[i].Color(0) / 255.0F;
                float gcolor = mapPoints[i].Color(1) / 255.0F;
                float bcolor = mapPoints[i].Color(2) / 255.0F;
                glColor3f(rcolor,gcolor,bcolor);
                glVertex3f(xpt,ypt,zpt);
            }
            glEnd();
        }

        if(menuShowFused){
            DrawFused();
        }

        if(menuShowKeyFrames)
        {
            DrawKeyframes();
        }

        if(menuShowPlane)
        {
            DrawPlane();
        }
/*
        if(menuShowObject)
        {
            glEnable(GL_CULL_FACE);
            glEnable(GL_NORMALIZE);
            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);
            glEnable(GL_COLOR_MATERIAL);

            DrawTable(menu_objectsize);

            glDisable(GL_CULL_FACE);
            glDisable(GL_NORMALIZE);  
            glDisable(GL_LIGHTING);
            glDisable(GL_LIGHT0);
            glDisable(GL_COLOR_MATERIAL);
        }
*/
        //std::cout << handler.Selected_P_w().transpose() << std::endl;

        DrawCurrentframe();

        DrawCoodinateSystem();
 
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
}

void ViewerLY::DrawCurrentframe()
{
    const float w = mKeyFrameSize*1.5;
    const float h = w*0.75;
    const float z = w*0.6;

    for(size_t i = 0; i < currentTwcs.size(); i++){
   
    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(currentTwcs[i].m);
#else
        glMultMatrixd(currentTwcs[i].m);
#endif

    glLineWidth(mCameraLineWidth+1);
    glColor3f(1.0f,0.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);
    
    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();

    }
}


void ViewerLY::DrawCoodinateSystem()
{
    pangolin::OpenGlMatrix tempTwc;
    tempTwc.SetIdentity();

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(tempTwc.m);
#else
        glMultMatrixd(tempTwc.m);
#endif

    glLineWidth(mCameraLineWidth+2);
    glBegin(GL_LINES);
    glColor3f(1.0f,0.0f,0.0f);
    glVertex3f(0,0,0);
    glVertex3f(coordinateLength,0,0);

    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(0,0,0);
    glVertex3f(0,coordinateLength,0);

    glColor3f(0.0f,0.0f,1.0f);
    glVertex3f(0,0,0);
    glVertex3f(0,0,coordinateLength);

    glEnd();

    glPopMatrix();

}


void ViewerLY::DrawKeyframes()
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;
    
    for(size_t i = 0; i < Twcs.size(); i++)
    {
        glPushMatrix();

#ifdef HAVE_GLES
            glMultMatrixf(Twcs[i].m);
#else
            glMultMatrixd(Twcs[i].m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
    
        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }

}

void ViewerLY::DrawPlane()
{
    pangolin::OpenGlMatrix tempTwc;
    tempTwc.SetIdentity();

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(tempTwc.m);
#else
        glMultMatrixd(tempTwc.m);
#endif

    // Plane parallel to x-z at origin with normal -y
    const float minx = -ndivs*ndivsize;
    const float minz = -ndivs*ndivsize;
    const float maxx = ndivs*ndivsize;
    const float maxz = ndivs*ndivsize;


    glLineWidth(2);
    glColor3f(0.7f,0.7f,1.0f);
    glBegin(GL_LINES);

    for(int n = 0; n<=2*ndivs; n++)
    {
        //draw a line in x direction
        glVertex3f(minx+ndivsize*n,height,minz);
        glVertex3f(minx+ndivsize*n,height,maxz);
        //draw a line in z direction
        glVertex3f(minx,height,minz+ndivsize*n);
        glVertex3f(maxx,height,minz+ndivsize*n);
    }

    glEnd();

    glPopMatrix();

}


void ViewerLY::DrawTable(float ratio){

    pangolin::OpenGlMatrix tempTwc;
    tempTwc.SetIdentity();
    tempTwc.m[12] = objectX;
    tempTwc.m[13] = height + objectBottom*ratio;
    tempTwc.m[14] = objectZ;
    tempTwc.m[0] = std::cos(rotateY);
    tempTwc.m[10] = - tempTwc.m[0];
    tempTwc.m[5] = - 1;
    tempTwc.m[8] = std::sin(rotateY);
    tempTwc.m[2] = tempTwc.m[8];

    glPushMatrix();
#ifdef HAVE_GLES
        glMultMatrixf(tempTwc.m);
#else
        glMultMatrixd(tempTwc.m);
#endif
    glColor3f(0.5, 0.25, 0.0);

    bool add_normal = true;

    for(int i = 0 ; i < numFace ; i ++){
        if(add_normal){
            glNormal3f(normals[i][0],normals[i][1],normals[i][2]); 
        }

        glBegin(GL_TRIANGLES);
        for(int j = 1; j <= indexs[i][0]; j ++){
            glVertex3f(vertex_arrayX[indexs[i][j]]*ratio, vertex_arrayY[indexs[i][j]]*ratio,
                         vertex_arrayZ[indexs[i][j]]*ratio);
        }
        glEnd();
    }
    

    bool drawLines = false;
    if(drawLines){
        glLineWidth(1);
        glBegin(GL_LINES);
        glColor3f(0.0, 0.0, 0.0);

        for(int i = 0 ; i < numFace ; i ++){
            for(int j = 1; j <= indexs[i][0]; j ++){
                glVertex3f(vertex_arrayX[indexs[i][j]]*ratio, vertex_arrayY[indexs[i][j]]*ratio,
                        vertex_arrayZ[indexs[i][j]]*ratio);
                int k = j + 1;
                if(k > 3){
                    k = 1;
                }
                glVertex3f(vertex_arrayX[indexs[i][k]]*ratio, vertex_arrayY[indexs[i][k]]*ratio,
                          vertex_arrayZ[indexs[i][k]]*ratio);
            }
        }
        glEnd();
    }

    glPopMatrix();
}

void ViewerLY::DrawFused()
{
    glPointSize(mPointSize);
    glBegin(GL_POINTS);

    for(size_t i=0, iend=plypoints.size(); i<iend;i++)
    {
            
        float xpt = plypoints[i].x;
        float ypt = plypoints[i].y;
        float zpt = plypoints[i].z;
        float rcolor = plypoints[i].r / 255.0F;
        float gcolor = plypoints[i].g / 255.0F;
        float bcolor = plypoints[i].b / 255.0F;
        glColor3f(rcolor,gcolor,bcolor);
        glVertex3f(xpt,ypt,zpt);
    }
    glEnd();
}

void ViewerLY::SetCurrentPose(Eigen::Vector4d qvec, Eigen::Vector3d tvec)
{
    Eigen::Quaterniond quaternion(qvec(0),qvec(1),qvec(2),qvec(3));

    // with inverse
    Eigen::Matrix3x4d currentMatrix;
    Eigen::MatrixXd rotationMat = quaternion.matrix().transpose();
    tvec = - rotationMat * tvec;

    currentMatrix.block(0,0,3,3) = rotationMat;
    currentMatrix(0,3) = tvec(0);
    currentMatrix(1,3) = tvec(1);
    currentMatrix(2,3) = tvec(2);

    pangolin::OpenGlMatrix TwcNew;
    GetOpenGLCameraMatrix(currentMatrix, TwcNew);
    currentTwcs.push_back(TwcNew);
}


void ViewerLY::GetOpenGLCameraMatrix(Eigen::Matrix3x4d matrix, pangolin::OpenGlMatrix &M)
{
    M.m[0] = matrix(0,0);
    M.m[1] = matrix(1,0);
    M.m[2] = matrix(2,0);
    M.m[3]  = 0.0;

    M.m[4] = matrix(0,1);
    M.m[5] = matrix(1,1);
    M.m[6] = matrix(2,1);
    M.m[7]  = 0.0;

    M.m[8] = matrix(0,2);
    M.m[9] = matrix(1,2);
    M.m[10] = matrix(2,2);
    M.m[11]  = 0.0;

    M.m[12] = matrix(0,3);
    M.m[13] = matrix(1,3);
    M.m[14] = matrix(2,3);
    M.m[15]  = 1.0;
}

} // namespace



