#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>
#include "LoopClosure/KeyFrameMapPoint.h"

void DrawCamera()
{
    float mCameraSize = 0.2;
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

void DrawKeyframe(cv::Mat Twc)
{
    pangolin::OpenGlMatrix currentT;
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            currentT.m[4*j + i] = Twc.at<float>(i,j);
        }
    }

    glPushMatrix();
    glMultMatrixd(currentT.m);

    DrawCamera();

    glPopMatrix();
}


void DrawTest(std::vector<GKeyFrame*> &vKeyFrames, std::vector<GMapPoint*> &vMapPoints, std::vector<cv::Mat> &successedImages) 
{   
    // create pangolin window and plot the map
    pangolin::CreateWindowAndBind("Map Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

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
        glColor3f(0.0f,1.0f,0.0f);
        glLineWidth(2);
        int nPath_size = vKeyFrames.size();
        for(int i = 0; i < nPath_size; ++i)
        {        
            DrawKeyframe(vKeyFrames[i]->Twc);
        }
      
        glColor3f(0.0f,0.0f,1.0f);
        glLineWidth(5);
        for(size_t i = 0 ; i < successedImages.size() ; i ++)
            DrawKeyframe(successedImages[i]);
        

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        int pointsize = vMapPoints.size();
        for(int i = 0; i < pointsize;++i)
        {
            glColor3f(1, 0, 0);
            glVertex3d(vMapPoints[i]->worldPose.at<float>(0),
                       vMapPoints[i]->worldPose.at<float>(1),
                       vMapPoints[i]->worldPose.at<float>(2));
        }
        glEnd();


        pangolin::FinishFrame();
        usleep(10);
    }
}
