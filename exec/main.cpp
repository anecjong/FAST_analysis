#include "fast_classes/featurecnm.hpp"


int main(int argv, char** argc){
    std::string str1, str2;
    double resize_factor = 0.7;
    if (argv == 2)
    {
        switch(atoi(argc[1])){
            case 0:
                str1 = "../resources/kitti_far_01.png";
                str2 = "../resources/kitti_far_02.png";
                break;
            case 1:
                str1 = "../resources/kitti_close_01.png";
                str2 = "../resources/kitti_close_02.png";
                break;
            default:
                str1 = "../resources/kitti_far_01.png";
                str2 = "../resources/kitti_far_02.png";
                break;
        }  
    }
    else{
            str1 = "../resources/kitti_far_01.png";
            str2 = "../resources/kitti_far_02.png";
    }

    std::cout << "image loading: " << std::endl << str1 << std::endl << str2 << std::endl;

    cv::Mat img1 = cv::imread(str1, cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread(str2, cv::IMREAD_COLOR);
    cv::resize(img1, img1, cv::Size(), resize_factor, resize_factor, cv::INTER_AREA);
    cv::resize(img2, img2, cv::Size(), resize_factor, resize_factor, cv::INTER_AREA);
    std::cout << "Image cols and rows" << std::endl << img1.cols << ", " << img1.rows << std::endl;
    std::cout << std::endl;

    for (int i=0; i<8; ++i){
        int thresh = 10 + 5 * i;
        show_results(thresh, img1, img2);
    }

    response_matching_quality(10, img1, img2);
    return 0;
}
