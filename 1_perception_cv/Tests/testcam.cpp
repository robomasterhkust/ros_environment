//g++ -std=c++11 `pkg-config --cflags opencv`   testcam.cpp `pkg-config --libs opencv`     -I./mvux_camera   -lMVSDK

#include "Camera.hpp"
#include "../Settings/Settings.hpp"
#include "../Camera/Camera.hpp"
#include "mvCamera.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

Settings settings("testcamSetting.xml");

int main()
{
    settings.load();
    vector<Camera *> cams;
    loadCams(settings, cams, 1);
    cout << cams.size();
    if (!cams[0]->initialize())
    {
        cout << "cams[0]->initialize(); ERROR!\n";
    }

    cams[0]->startStream();
    static FrameInfo *tempOut;
    while (1)
    {
        cams[0]->tryWork();
        if (cams[0]->outQ[0].dequeue(tempOut))
        {
            imshow("frame", tempOut->img);
            delete tempOut;
        }
        cv::waitKey(10);
    }
}