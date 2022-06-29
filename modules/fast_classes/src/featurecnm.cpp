#include "fast_classes/featurecnm.hpp"

FEATURE_compare::FEATURE_compare(cv::Mat src, cv::Mat dst, int thresh):
src(src), dst(dst), thresh(thresh)
{
    get_feature_info();
}

void FEATURE_compare::get_feature_info(){
    std::vector<cv::KeyPoint> temp_keypoint;
    // cv::Mat temp_desc;

    std::chrono::steady_clock::time_point start_t = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; ++i){
        cv::FAST(src, temp_keypoint, thresh);
    }
    std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
    time1 = std::chrono::duration_cast<std::chrono::microseconds>(end_t-start_t).count()/10.0;
    kp1 = temp_keypoint;

    start_t = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; ++i){
        cv::FAST(dst, temp_keypoint, thresh);
    }
    end_t = std::chrono::steady_clock::now();
    time2 = std::chrono::duration_cast<std::chrono::microseconds>(end_t-start_t).count()/10.0;
    kp2 = temp_keypoint;

    // desc1 = temp_desc;
    NoF1 = kp1.size();
    NoF2 = kp2.size();

    response1 = 0;
    response2 = 0;
    for (auto kp : kp1){
        response1 += kp.response;
    }
    response1/=NoF1;

    for (auto kp : kp2){
        response2 += kp.response;
    }
    response2/=NoF2;
}

std::vector<cv::KeyPoint> FEATURE_compare::get_kp1(){
    return kp1;
}
std::vector<cv::KeyPoint> FEATURE_compare::get_kp2(){
    return kp2;
}
int FEATURE_compare::get_NoF1(){
    return NoF1;
}
int FEATURE_compare::get_NoF2(){
    return NoF2;
}
double FEATURE_compare::get_response1(){
    return response1;
}
double FEATURE_compare::get_response2(){
    return response2;
}

double FEATURE_compare::get_time1(){
    return time1;
}
double FEATURE_compare::get_time2(){
    return time2;
}
void FEATURE_compare::change_thresh(int thresh_)
{
    thresh = thresh_;
}


FEATURE_match::FEATURE_match(cv::Mat src, cv::Mat dst,
    std::vector<cv::KeyPoint> kp1, std::vector<cv::KeyPoint> kp2):
    src(src), dst(dst), kp1(kp1), kp2(kp2)
{
    features = cv::ORB::create();
    matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    inlier_thresh = 0.05;
    dist_thresh = std::min(src.cols, src.rows) * inlier_thresh;
}

void FEATURE_match::get_match_info(){
    desc_times = time_compute();
    matching_time = 0;

    std::chrono::steady_clock::time_point start_t, end_t;
    start_t = std::chrono::steady_clock::now();
    for(int i=0; i<10; ++i){
        matcher->match(desc1, desc2, matches);
    }
    end_t = std::chrono::steady_clock::now();
    matching_time = (std::chrono::duration_cast<std::chrono::microseconds>(end_t-start_t).count()/10.0);

    // picking good match case, 10%
    float percentage = 0.10f;
    std::sort(matches.begin(), matches.end());

    int temp_good_match = 0;
    for (size_t i = 0; i < matches.size(); ++i){
        // std::cout << matches[i].distance << " ";
        if (matches[i].distance < 64) temp_good_match++;
    }
    NofGM_over_NofAM = std::round((double)temp_good_match/matches.size() * 100);

    std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + (int)(percentage * matches.size()));

    std::vector<cv::Point2f> pt1, pt2;
    for (size_t i = 0; i < good_matches.size(); ++i){
        pt1.push_back(kp1[good_matches[i].queryIdx].pt);
        pt2.push_back(kp2[good_matches[i].queryIdx].pt);
    }

    homography_time = 0;
    accuracy = 0;
    matching_cnt = matches.size();
    std::vector<cv::Point2f> pt2_est;
    std::vector<cv::Point2f> pt1_all;
    pt1_all.reserve(matches.size());

    for (size_t i = 0; i < (int)(matches.size()); ++i){
        pt1_all.push_back(kp1[matches[i].queryIdx].pt);
    }

    for(int i=0; i<10; ++i){
        pt2_est.clear();
        start_t = std::chrono::steady_clock::now();
        H = cv::findHomography(pt1, pt2, cv::RANSAC);
        end_t = std::chrono::steady_clock::now();
        homography_time += (std::chrono::duration_cast<std::chrono::microseconds>(end_t-start_t).count()/10.0);

        cv::perspectiveTransform(pt1_all, pt2_est, H);
        double dist = 0;
        int inlier_match = 0;

        for (size_t i = 0; i<(int)(matches.size()); ++i){
            // dist = pow(pt2[matches[i].queryIdx].x - pt2_est[i].x, 2)
            //         + pow(pt2[matches[i].queryIdx].y - pt2_est[i].y, 2);
            // dist = pow(dist, 0.5);
            dist = abs(pt2[matches[i].queryIdx].y - pt2_est[i].y);
            if (dist < dist_thresh) inlier_match++;
        }
        
        accuracy += (double)inlier_match/matches.size();
    }

    homography_time /= 10;
    accuracy /= 10;
}

std::vector<double> FEATURE_match::time_compute(){
    std::vector<double> times;

    std::chrono::steady_clock::time_point start_t = std::chrono::steady_clock::now();
    for(int i=0; i<10; ++i){
        features->compute(src, kp1, desc1);
    }
    std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
    times.push_back(std::chrono::duration_cast<std::chrono::microseconds>(end_t-start_t).count()/10.0);

    start_t = std::chrono::steady_clock::now();
    for(int i=0; i<10; ++i){
        features->compute(src, kp2, desc2);
    }
    end_t = std::chrono::steady_clock::now();
    times.push_back(std::chrono::duration_cast<std::chrono::microseconds>(end_t-start_t).count()/10.0);

    return times;
}

int FEATURE_match::get_matching_cnt(){
    return matching_cnt;
}

double FEATURE_match::get_accuracy(){
    return accuracy;
}

double FEATURE_match::get_matching_time(){
    return matching_time;
}

double FEATURE_match::get_homography_time(){
    return homography_time;
}

double FEATURE_match::get_NofGM_over_NofAM(){
    return NofGM_over_NofAM;
}

std::vector<double> FEATURE_match::get_desc_time(){
    return desc_times;
}
std::vector<cv::DMatch> FEATURE_match::get_matches(){
    return matches;
}

void show_results(int thresh, cv::Mat img1, cv::Mat img2){

    cv::Mat img1_gray, img2_gray;
    cv::Mat img1_kp_drawn, img2_kp_drawn;
    cv::Mat img_match_drawn_in_64, img_match_drawn_out_64;

    cv::cvtColor(img1, img1_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2, img2_gray, cv::COLOR_BGR2GRAY);

    FEATURE_compare img_fast(img1_gray, img2_gray, thresh);

    std::vector<cv::KeyPoint> kp1, kp2;
    kp1 = img_fast.get_kp1();
    kp2 = img_fast.get_kp2();
    int NoF1 = img_fast.get_NoF1();
    int NoF2 = img_fast.get_NoF2();
    double response1 = img_fast.get_response1();
    double response2 = img_fast.get_response2();
    double time1 = img_fast.get_time1();
    double time2 = img_fast.get_time2();

    cv::drawKeypoints(img1, kp1, img1_kp_drawn);
    cv::drawKeypoints(img2, kp2, img2_kp_drawn);

    FEATURE_match matcher(img1_gray, img2_gray, kp1, kp2);
    matcher.get_match_info();
    double accuracy = matcher.get_accuracy();
    int matching_cnt = matcher.get_matching_cnt();
    double homography_time = matcher.get_homography_time();
    double matching_time = matcher.get_matching_time();
    double NofGM_over_NofAM = matcher.get_NofGM_over_NofAM();
    std::vector<double> desc_time = matcher.get_desc_time();
    std::vector<cv::DMatch> matches = matcher.get_matches();
    std::vector<cv::DMatch> matches_in_64, matches_out_64;

    for (auto& match: matches){
        if (match.distance < 64){
            matches_in_64.push_back(match);
        }
        else{
            matches_out_64.push_back(match);
        }
    }
    std::vector<cv::DMatch> matches_in_64_20(matches_in_64.begin(), matches_in_64.begin()+20);
    std::vector<cv::DMatch> matches_out_64_20(matches_out_64.end()-21, matches_out_64.end());

    cv::drawMatches(img1, kp1, img2, kp2, matches_in_64_20, img_match_drawn_in_64, cv::Scalar_<double>::all(-1),cv::Scalar_<double>::all(-1), std::vector< char >(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    cv::drawMatches(img1, kp1, img2, kp2, matches_out_64_20, img_match_drawn_out_64,cv::Scalar_<double>::all(-1),cv::Scalar_<double>::all(-1), std::vector< char >(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    std::string name1 = std::string("../match_imgs_threshold/") += std::to_string(thresh) += std::string("_in_64.png");
    std::string name2 = std::string("../match_imgs_threshold/") += std::to_string(thresh) += std::string("_out_64.png");
    cv::imwrite(name1, img_match_drawn_in_64);
    cv::imwrite(name2, img_match_drawn_out_64);

    std::cout << "====thresh " << thresh << " matching====" << std::endl;
    std::cout << "1st NoF:           " << NoF1 << std::endl;
    std::cout << "1st response:      " << response1 << std::endl;
    std::cout << "1st time:          " << time1 << std::endl;
    std::cout << "2nd NoF:           " << NoF2 << std::endl;
    std::cout << "2nd response:      " << response2 << std::endl;
    std::cout << "2nd time:          " << time2 << std::endl;

    std::cout << "Accuracy:          " << accuracy << std::endl;
    std::cout << "matching accuracy: " << NofGM_over_NofAM  << " %" << std::endl;
    std::cout << "Matching cnt:      " << matching_cnt << std::endl;
    std::cout << "Desc time:         " << desc_time[0] + desc_time[1] << std::endl;
    std::cout << "Matching time:     " << matching_time << std::endl;
    std::cout << "Homography time:   " << homography_time << std::endl;
    std::cout << std::endl;

}

void response_matching_quality(int thresh, cv::Mat img1, cv::Mat img2){

    cv::Mat img1_gray, img2_gray;

    cv::cvtColor(img1, img1_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2, img2_gray, cv::COLOR_BGR2GRAY);

    FEATURE_compare img_fast(img1_gray, img2_gray, thresh);

    std::vector<cv::KeyPoint> kp1, kp2;
    kp1 = img_fast.get_kp1();
    kp2 = img_fast.get_kp2();

    std::vector<std::vector<cv::KeyPoint>> kp1_r {std::vector<cv::KeyPoint>{},std::vector<cv::KeyPoint>{},std::vector<cv::KeyPoint>{},std::vector<cv::KeyPoint>{}};
    std::vector<std::vector<cv::KeyPoint>> kp2_r {std::vector<cv::KeyPoint>{},std::vector<cv::KeyPoint>{},std::vector<cv::KeyPoint>{},std::vector<cv::KeyPoint>{}};
    for (auto& kp : kp1){
        if (kp.response < 10) kp1_r[0].push_back(kp);
        else if (kp.response < 20) kp1_r[1].push_back(kp);
        else if (kp.response < 30) kp1_r[2].push_back(kp);
        else if (kp.response < 40) kp1_r[3].push_back(kp);
    }
    for (auto& kp : kp2){
        if (kp.response <10) kp2_r[0].push_back(kp);
        else if (kp.response < 20) kp2_r[1].push_back(kp);
        else if (kp.response < 30) kp2_r[2].push_back(kp);
        else if (kp.response < 40) kp2_r[3].push_back(kp);
    }

    cv::Ptr<cv::Feature2D> feature = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);

    for (int i = 0; i < kp1_r.size();++i){
        cv::Mat desc1, desc2;
        cv::Mat img_drawn1, img_drawn2;
        std::vector<cv::DMatch> matches, matches_20, matches_10p;
        feature->compute(img1_gray, kp1_r[i], desc1);
        feature->compute(img2_gray, kp2_r[i], desc2);
        matcher->match(desc1, desc2, matches);

        matches_10p = std::vector<cv::DMatch>(matches.begin(), matches.begin() + (int)(0.1 * matches.size()));

        if (matches.size() > 20){
            matches_20 = std::vector<cv::DMatch>(matches.begin(), matches.begin() + 20);
        }
        else{
            matches_20 = std::vector<cv::DMatch>(matches.begin(), matches.end());
        } 

        int temp_good_match = 0;
        for (size_t i = 0; i < matches.size(); ++i){
            if (matches[i].distance < 64) temp_good_match++;
        }

        int NofGM_over_NofAM = std::round((double)temp_good_match/matches.size() * 100);
        if (temp_good_match == 0) NofGM_over_NofAM = 0;

        std::cout << "response < " << std::to_string(i*10+10) << " matching accuracy: " << NofGM_over_NofAM  << " %" << std::endl;
        cv::drawMatches(img1, kp1_r[i], img2, kp2_r[i], matches_20, img_drawn1, cv::Scalar_<double>::all(-1),cv::Scalar_<double>::all(-1), std::vector< char >(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        cv::drawMatches(img1, kp1_r[i], img2, kp2_r[i], matches_10p, img_drawn2, cv::Scalar_<double>::all(-1),cv::Scalar_<double>::all(-1), std::vector< char >(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        std::string name1 = std::string("../match_imgs_response/response_under_") += std::to_string(i*10+10) += std::string("_20pts.png");
        std::string name2 = std::string("../match_imgs_response/response_under_") += std::to_string(i*10+10) += std::string("_10percent.png");
        cv::imwrite(name1, img_drawn1);
        cv::imwrite(name2, img_drawn2);
    }
}