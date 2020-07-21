#include <iostream>
#include <random>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include "LoopClosure/ReadOfflineMap.h"

int main()
{
    std::cout << "Loop closure test." << std::endl;

    std::string datafile = "/home/viki/UTOPA/test.dat";

    std::string orbvocfile = "/home/viki/UTOPA/ZED/orb_mur.fbow";
    std::string orbdbowfile = "/home/viki/Documents/ORBvoc.bin";

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    OfflineMap offlineMap;
    offlineMap.ReadMapFromDataFile(datafile);

    offlineMap.LoadVocORB(orbdbowfile);

//    offlineMap.LoadFBOWvocabulary(orbvocfile);
//    offlineMap.AddImagesToVoc();

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    std::cout << " [OFFLINE MAP] done, using time : " << ttrack << " seconds. " << std::endl << std::endl;

    std::cout << " [TEST] test match new image." << std::endl;

    int succ = 0;
    offlineMap.InitCameraParameters(493,493,320,240);
    for(int i = 893; i < 1500; i ++){
        std::cout << i << " ";
        cv::Mat image = cv::imread("/home/viki/Documents/VioDeepBlue/datasets/images/"+std::to_string(i)+".jpg",
                                  cv::IMREAD_GRAYSCALE);
        if(offlineMap.LoopDetector(image)){
            succ++;
            
            std::cout << " [LOOP DETECT] SUCCESS! \n ";
        }
    }
    std::cout << " " << succ << " frames successfully localized. \n";

    offlineMap.Draw();

    return 0;
}


