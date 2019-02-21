#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ygz/Viewer.h"
#include "ygz/Settings.h"
#include "ygz/Frame.h"
#include "ygz/MapPoint.h"
#include "ygz/Feature.h"
#include "ygz/BackendInterface.h"

#define UI_WIDTH 240

namespace ygz {

    Viewer::Viewer(bool startViewer) {

        if (startViewer) {
            mbRunning = true;
            mViewerThread = thread(&Viewer::Run, this);
        }

        mBiasLogLabels.clear();
        mBiasLogLabels.push_back("bg_x");
        mBiasLogLabels.push_back("bg_y");
        mBiasLogLabels.push_back("bg_z");
        mBiasLogLabels.push_back("ba_x");
        mBiasLogLabels.push_back("ba_y");
        mBiasLogLabels.push_back("ba_z");
        mBiasLog.SetLabels(mBiasLogLabels);
    }

    Viewer::~Viewer() {
    }

    void Viewer::RunAndSpin() {

        if (mbRunning == false) {
            mbRunning = true;
            Run();
        }
    }

    void Viewer::WaitToFinish() {
        if (mbRunning == true) {
            mViewerThread.join();
        }
    }

    void Viewer::SetTrackStatus(int sta, int trackInliers) {
        unique_lock<mutex> lk(mMutexTrackerStatus);
        mTrackState = sta;
        mTrackInliers = trackInliers;
    }

    void Viewer::Run() {

        int w = setting::imageWidth;
        int h = setting::imageHeight;
        pangolin::CreateWindowAndBind("YGZ-Viewer", 1280, 720);

        // Vector3d up(0, 0, 1);
        Vector3d up(0, -1, 0);
        up = setting::TBC * up;

        // 3D viewer
        pangolin::OpenGlRenderState Visualization3D_camera(
                pangolin::ProjectionMatrix(1024, 768, 500, 500, w / 2, h / 2, 0.1, 1000),
                pangolin::ModelViewLookAt(0, 0, -0.5, 0, 0, 0, up[0], up[1], up[2])
        );

        pangolin::View &Visualization3D_display = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -w / (float) h)
                .SetHandler(new pangolin::Handler3D(Visualization3D_camera));

        // Show the image and feature points
        /*
        pangolin::View &d_video = pangolin::Display("Gray image")
                .SetAspect(w / (float) h);

        pangolin::CreateDisplay()
                .SetBounds(0.0, 0.3, 0.0, pangolin::Attach::Pix(UI_WIDTH))
                .AddDisplay(d_video);

        pangolin::GlTexture texVideo(w, h, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
         */

        // Show the curve of IMU bias
        pangolin::Plotter plotter_biascurve(&mBiasLog, 0.0f, 40.0f * 20, -0.4f, 0.4f, 1.0f / 20, 0.1f);
        plotter_biascurve.Track("$i");
        plotter_biascurve.SetAspect(w / (float) h);

        pangolin::CreateDisplay()
                .SetBounds(0.3, 0.6, 0.0, pangolin::Attach::Pix(UI_WIDTH))
                .AddDisplay(plotter_biascurve);

        pangolin::CreatePanel("ui")
                .SetBounds(0.6, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

        pangolin::Var<int> nTrackFeats("ui.TrackInliers", 0);
        pangolin::Var<int> nTrackState("ui.TrackState", 0);

        while (!pangolin::ShouldQuit() && mbRunning) {

            if (mbNewFrameUpdated) {

                {
                    unique_lock<mutex> lock(mMutexNewFrame);
                    mCurrentFrame = mpNewCurrent;
                    mKeyFrames.push_back(mpNewFrame);
                    mbNewFrameUpdated = false;
                }

                for (shared_ptr<Feature> feat: mCurrentFrame->mFeaturesLeft) {
                    if (feat == nullptr)
                        continue;
                    if (feat->mpPoint && feat->mpPoint->isBad() == false)
                        mPoints.insert(feat->mpPoint);
                }

                if (mbRecordTrajectory) {
                    mTrajectory.push_back(mCurrentFrame->Ow());
                }

            }

            // Clear entire screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // show frames
            // Activate efficiently by object
            Visualization3D_display.Activate(Visualization3D_camera);

            DrawOrigin();

            if (mCurrentFrame) {

                // make the camera follow current frame
                auto Twc = GetCurrentGLPose();
                Visualization3D_camera.Follow(Twc);
                Visualization3D_display.Activate(Visualization3D_camera);

                // draw current frame
                // 红的是Current
                DrawFrame(mCurrentFrame, Vector3d(1, 0, 0));
            }

            // 否则，显示自己记录的内容
            // other frames
            if (!mKeyFrames.empty()) {
                for (weak_ptr<Frame> frame : mKeyFrames) {
                    if (frame.expired())
                        continue;
                    shared_ptr<Frame> f = frame.lock();
                    if (f == mCurrentFrame)
                        continue;
                    // 蓝的是其他Frame
                    DrawFrame(f, Vector3d(0, 0, 1));
                }
            }
            // all points
            DrawPoints(Vector3d(0, 1, 0));

            if (mbRecordTrajectory) {
                DrawTrajectory();
            }

            if (mbShowTrajGT)
                DrawTrajectoryGT();

            // show image
            if (mCurrentFrame && mbShowCurrentImg) {
                cv::Mat im = DrawImage();
                imshow("image",im);
                cv::waitKey(1);
                /*
                texVideo.Upload(im.data, GL_BGR, GL_UNSIGNED_BYTE);
                d_video.Activate();
                glColor3f(1.0f, 1.0f, 1.0f);
                texVideo.RenderToViewportFlipY();
                Visualization3D_display.Activate();
                 */
            }

            // draw bias curve
            if (mCurrentFrame) {
                static double preFrameTime = -1;
                if (mCurrentFrame->mTimeStamp != preFrameTime) {
                    preFrameTime = mCurrentFrame->mTimeStamp;
                    plotter_biascurve.Activate();
                    Vector6d bias;
                    bias.head<3>() = mCurrentFrame->BiasG();
                    bias.tail<3>() = mCurrentFrame->BiasA();
                    mBiasLog.Log(bias);
                    Visualization3D_display.Activate();
                }
            }


            // show status and track points
            {
                unique_lock<mutex> lk(mMutexTrackerStatus);
                nTrackFeats = mTrackInliers;
                nTrackState = mTrackState;
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();

            usleep(30000);
        }

        mbRunning = false;
    }

    void Viewer::AddFrame(shared_ptr<Frame> frame, bool setToCurrent) {

        unique_lock<mutex> lock(mMutexNewFrame);
        mpNewFrame = frame;
        if (setToCurrent)
            mpNewCurrent = frame;
        mbNewFrameUpdated = true;

    }

    void Viewer::SetCurrentFrame(shared_ptr<Frame> frame) {
        unique_lock<mutex> lock(mMutexNewFrame);
        mpNewCurrent = frame;
        mbNewFrameUpdated = true;
    }

    void Viewer::DrawOrigin() {
        // x axis
        glColor3d(1, 0, 0);     // red is x
        glLineWidth(4);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
        glEnd();

        // y axis
        glColor3d(0, 1, 0);     // green is y
        glLineWidth(4);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);
        glEnd();

        // z axis
        glColor3d(0, 0, 1);     // blue is z
        glLineWidth(4);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
        glEnd();
    }

    void Viewer::DrawFrame(shared_ptr<Frame> frame, const Vector3d &color) {

        Sophus::SE3d Twc(frame->mRwc, frame->mOw);
        Matrix4d m = Twc.matrix();
        float sz = setting::cameraSize;

        const float w = 1 * sz;
        const float h = w * .75;
        const float z = w;

        glPushMatrix();
        glMultMatrixd((GLdouble *) m.data());

        glColor3d(color[0], color[1], color[2]);

        // The camera shape
        glLineWidth(2);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();
        glPopMatrix();

        if (frame->mpReferenceKF.expired() == false) {
            // 画出此Frame到它参考帧的连线
            Vector3d Ow = frame->mOw;
            Vector3d OwRef = frame->mpReferenceKF.lock()->mOw;
            glBegin(GL_LINES);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(OwRef[0], OwRef[1], OwRef[2]);
            glEnd();
        }

        if (mbShowKFGT) {
            glPushMatrix();
            m = frame->GetPoseGT().matrix();
            m = m * setting::TBC.matrix();
            m = m.transpose();
            glMultMatrixd((GLdouble *) m.data());
            glColor3d(1 - color[0], 1 - color[1], 1 - color[2]);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);
            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);
            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);
            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();
            glPopMatrix();
        }
    }

    void Viewer::DrawPoints(const Vector3d &color) {

        glPointSize(4);
        glColor3d(color[0], color[1], color[2]);

        for (auto iter = mPoints.begin(); iter != mPoints.end();) {

            if (iter->expired()) {
                iter = mPoints.erase(iter);
                continue;
            }

            shared_ptr<MapPoint> mp = iter->lock();
            if (mp == nullptr)
                continue;

            glBegin(GL_POINTS);
            Vector3d pos = mp->GetWorldPos();

            if (mp->Status() == MapPoint::IMMATURE) {
                glColor3d(1, 0, 0);     // immature 的画红色
            } else {
                glColor3b(mp->mGray, mp->mGray, mp->mGray);
                // glColor3d(0, 1, 0);
            }

            glVertex3d(pos[0], pos[1], pos[2]);
            glEnd();

            if (mbShowConnection) {
                if (mp->mpRefKF.expired())
                    continue;
                shared_ptr<Frame> f = mp->mpRefKF.lock();
                Vector3d ow = f->Ow();
                glBegin(GL_LINES);
                glLineWidth(1);
                glVertex3d(ow[0], ow[1], ow[2]);
                glVertex3d(pos[0], pos[1], pos[2]);
                glEnd();
            }

            iter++;
        }
    }

    void Viewer::DrawTrajectory() {

        if (mTrajectory.size() <= 1)
            return;

        glLineWidth(2);
        glColor3d(33.0 / 255.0, 131.0 / 255.0, 203.0 / 255.0);

        glBegin(GL_LINES);
        for (size_t i = 0; i < mTrajectory.size() - 1; i++) {
            glVertex3d(mTrajectory[i][0], mTrajectory[i][1], mTrajectory[i][2]);
            glVertex3d(mTrajectory[i + 1][0], mTrajectory[i + 1][1], mTrajectory[i + 1][2]);
        }
        glEnd();
    }

    cv::Mat Viewer::DrawImage() {
        if (mCurrentFrame == nullptr)
            return Mat();

        shared_ptr<Frame> f = mCurrentFrame;

        cv::Mat im;
        cv::cvtColor(f->mImLeft, im, CV_GRAY2BGR);

        for (shared_ptr<Feature> feat: f->mFeaturesLeft) {
            if (feat == nullptr)
                continue;
            if (feat->mpPoint == nullptr)
                continue;
            if (feat->mpPoint->Status() == MapPoint::IMMATURE) {
                cv::Point2f pt(feat->mPixel[0], feat->mPixel[1]);
                cv::Point2f pt1, pt2;
                pt1.x = pt.x - 2;
                pt1.y = pt.y - 2;
                pt2.x = pt.x + 2;
                pt2.y = pt.y + 2;
                cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 255), -1);
            } else {
                // 有深度，用彩色表示
                Vector3f color = MakeRedGreen3B(feat->mfInvDepth);
                cv::Point2f pt(feat->mPixel[0], feat->mPixel[1]);
                cv::Point2f pt1, pt2;
                pt1.x = pt.x - 2;
                pt1.y = pt.y - 2;
                pt2.x = pt.x + 2;
                pt2.y = pt.y + 2;
                cv::rectangle(im, pt1, pt2, cv::Scalar(255 * color[0], 255 * color[1], 255 * color[2]), -1);
            }
        }

        return im;
    }

    pangolin::OpenGlMatrix Viewer::GetCurrentGLPose() {
        if (mCurrentFrame == nullptr) {
            pangolin::OpenGlMatrix m;
            m.SetIdentity();
            return m;
        }

        shared_ptr<Frame> f = mCurrentFrame;
        pangolin::OpenGlMatrix M;
        Matrix4d Twc = SE3d(f->mRwc, f->mOw).matrix();

        M.m[0] = Twc(0, 0);
        M.m[1] = Twc(1, 0);
        M.m[2] = Twc(2, 0);
        M.m[3] = 0.0;

        M.m[4] = Twc(0, 1);
        M.m[5] = Twc(1, 1);
        M.m[6] = Twc(2, 1);
        M.m[7] = 0.0;

        M.m[8] = Twc(0, 2);
        M.m[9] = Twc(1, 2);
        M.m[10] = Twc(2, 2);
        M.m[11] = 0.0;

        M.m[12] = Twc(0, 3);
        M.m[13] = Twc(1, 3);
        M.m[14] = Twc(2, 3);
        M.m[15] = 1.0;
        return M;
    }

    void Viewer::DrawTrajectoryGT() {

        if (mGTTraj.size() < 1)
            return;
        if (mCurrentFrame == nullptr)
            return;

        glLineWidth(2);
        glColor3d(230.0 / 255.0, 230.0 / 255.0, 0.0 / 255.0);

        glBegin(GL_LINES);
        auto iter = mGTTraj.begin(), iterNext = mGTTraj.begin();
        iterNext++;
        for (; iterNext != mGTTraj.end(); iter++, iterNext++) {
            if (iter->first < mCurrentFrame->mTimeStamp) {
                Vector3d t1 = iter->second;
                Vector3d t2 = iterNext->second;
                // glVertex3d(-t1[1], -t1[2], t1[0]);
                // glVertex3d(-t2[1], -t2[2], t2[0]);
                glVertex3d(t1[0], t1[1], t1[2]);
                glVertex3d(t2[0], t2[1], t2[2]);
            } else
                break;
        }
        glEnd();
    }

}
