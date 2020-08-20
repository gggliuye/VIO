#ifndef VIEWER_UTOPA_HPP_
#define VIEWER_UTOPA_HPP_

#include "OrbMapping.h"

#include <pangolin/pangolin.h>

void DrawCamera(float mCameraSize)
{
    //float mCameraSize = 0.6;
    const float w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

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

}

pangolin::OpenGlMatrix TransformToOpenGL(Eigen::Matrix4d Twc)
{
    pangolin::OpenGlMatrix currentT;
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            currentT.m[4*j + i] = Twc(i,j);
        }
    }

    return currentT;
}

void GLidarViewer(std::shared_ptr<BASTIAN::OrbMapping> pOrbMapping)
{
    // create pangolin window and plot the map
    pangolin::CreateWindowAndBind("Orb Map Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 0.7, -1.8, 0, 0, 0, 0.0, 1.0, 0.0)
    );

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(200));
    pangolin::Var<int> menu_sizepoint("menu. Point Size",2,1,10);
    pangolin::Var<bool> menu_showcamera("menu. Show Camera",true,true);
    pangolin::Var<float> menu_sizecamera("menu. Camera Size",0.15,0.05,0.5);

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);        

        // draw poses
/*
        std::vector<BASTIAN::ColMapResult> vColMapResults = pOrbMapping->GetColmapResults();
        int num_frames = vColMapResults.size();
        for(int i = 0; i < num_frames; i++){
            pangolin::OpenGlMatrix currentT = TransformToOpenGL(vColMapResults[i].pose);
            glPushMatrix();
            glMultMatrixd(currentT.m);

            if(menu_showcamera){
                glColor3f(0.0f,0.0f,1.0f); glLineWidth(2); 
                DrawCamera(menu_sizecamera);
            }
            //glEnd();
            glPopMatrix();
        }
*/

        // draw map points
        std::vector<BASTIAN::OrbFrame*> vpOrbFrames = pOrbMapping->GetOrbFrames();
        for(int i = 0; i < vpOrbFrames.size() ; i++){
            BASTIAN::OrbFrame* pOrbFrame = vpOrbFrames[i];
            std::vector<BASTIAN::OrbPoint*> vpOrbPoints = pOrbFrame->vpOrbPoints;

            pangolin::OpenGlMatrix currentT = TransformToOpenGL(pOrbFrame->mPose);
            glPushMatrix();
            glMultMatrixd(currentT.m);

            if(menu_showcamera){
                glColor3f(0.0f,0.0f,1.0f); glLineWidth(2); 
                DrawCamera(menu_sizecamera);
            }
            glPopMatrix();

            glPointSize(menu_sizepoint);
            glBegin(GL_POINTS);
            for (std::size_t k = 0; k < pOrbFrame->N; k++){
                if(vpOrbPoints[k]){
                    glColor3f(1.0, 0, 0);
                    glVertex3d(vpOrbPoints[k]->mPose(0), vpOrbPoints[k]->mPose(1), vpOrbPoints[k]->mPose(2));
                }
            }
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(10);
    }

}



#endif
