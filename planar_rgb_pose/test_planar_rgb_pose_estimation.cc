#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>

#include "stats.h" // Stats structure definition
#include "utils.h" // Drawing and printing functions

using namespace std;
using namespace cv;

const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
// Minimal number of inliers to generate pose output.
const int bb_min_inliers = 25; 
const int stats_update_period = 10; // On-screen statistics are updated every 10 frames

namespace example {
class Tracker
{
public:
    Tracker(Ptr<cuda::ORB> _detector, Ptr<cuda::DescriptorMatcher> _matcher) :
        detector(_detector),
        matcher(_matcher)
    {}

    void setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats);
    Mat process(const Mat frame, Stats& stats);
    Ptr<Feature2D> getDetector() {
        return detector;
    }
protected:
    //Ptr<Feature2D> detector;
    Ptr<cuda::ORB> detector;
    Ptr<cuda::DescriptorMatcher> matcher;
    Mat first_frame, first_desc;
    cuda::GpuMat first_frame_gpu, first_desc_gpu;
    vector<KeyPoint> first_kp;
    vector<Point2f> object_bb;
};

void Tracker::setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats)
{
    cv::Point *ptMask = new cv::Point[bb.size()];
    const Point* ptContain = { &ptMask[0] };
    int iSize = static_cast<int>(bb.size());
    for (size_t i=0; i<bb.size(); i++) {
        ptMask[i].x = static_cast<int>(bb[i].x);
        ptMask[i].y = static_cast<int>(bb[i].y);
    }
    first_frame = frame.clone();
    cv::Mat matMask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::fillPoly(matMask, &ptContain, &iSize, 1, cv::Scalar::all(255));
    
    Mat frame_cp;
    cv::cvtColor(first_frame, frame_cp, CV_BGR2GRAY);
    //frame.convertTo(frame_cp, CV_8UC1);
    first_frame_gpu = cuda::GpuMat(frame_cp);
    //first_frame_gpu = cuda::GpuMat(frame_cp.size(), CV_8UC1);
    //first_frame_gpu.upload(frame_cp);
    cuda::GpuMat mask_gpu(matMask);
    //mask_gpu.upload(matMask);
    std::cout << "Before detector" << std::endl;
    //detector->detectAndCompute(first_frame, matMask, first_kp, first_desc);
    std::cout << first_frame_gpu.type() << "," << CV_8UC1 << std::endl;
    detector->detectAndCompute(first_frame_gpu, mask_gpu, first_kp, first_desc_gpu);
    std::cout << "First frame detect" << std::endl;

    stats.keypoints = (int)first_kp.size();
    drawBoundingBox(first_frame, bb);
    putText(first_frame, title, Point(0, 60), FONT_HERSHEY_PLAIN, 5, Scalar::all(0), 4);
    object_bb = bb;
    delete[] ptMask;
}

Mat Tracker::process(const Mat frame, Stats& stats)
{
    clock_t begin_time, end_time;
    vector<KeyPoint> kp;
    //Mat desc;
    //detector->detectAndCompute(frame, noArray(), kp, desc);

    // GPU match.
    begin_time = clock();
    Mat frame_cp;
    cv::cvtColor(frame, frame_cp, CV_BGR2GRAY);
    cuda::GpuMat frame_gpu(frame_cp);
    //frame_gpu.upload(frame_cp);
    end_time = clock();
    std::cout << "Elapsed time upload gpu frame: " << double(end_time - begin_time) / CLOCKS_PER_SEC << std::endl; 

    cuda::GpuMat desc;
    std::cout << "Convert to GPU frame" << std::endl;
    begin_time = clock();
    detector->detectAndCompute(frame_gpu, noArray(), kp, desc);
    end_time = clock();
    std::cout << "Elapsed time feature extractor orb: " << double(end_time - begin_time) / CLOCKS_PER_SEC << std::endl; 

    stats.keypoints = (int)kp.size();

    vector< vector<DMatch> > matches;
    vector<KeyPoint> matched1, matched2;
    begin_time = clock();
    matcher->knnMatch(first_desc_gpu, desc, matches, 2);
    end_time = clock();
    std::cout << "Elapsed time knn: " << double(end_time - begin_time) / CLOCKS_PER_SEC << std::endl;  
       
    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
            matched1.push_back(first_kp[matches[i][0].queryIdx]);
            matched2.push_back(      kp[matches[i][0].trainIdx]);
        }
    }
    // vector<DMatch> matches;
    // vector<KeyPoint> matched1, matched2;
    // matcher->match(first_desc_gpu, desc, matches);
    // double average_dist = 0;
    // for (unsigned i = 0; i < matches.size(); ++i) {
    //     average_dist += matches[i].distance;
    // }
    // average_dist = average_dist / matches.size();

    // for (unsigned i = 0; i < matches.size(); ++i) {
    //     if (matches[i].distance < average_dist) {
    //     matched1.push_back(first_kp[matches[i].queryIdx]);
    //     matched2.push_back(      kp[matches[i].trainIdx]);
    // }
    // }

    stats.matches = (int)matched1.size();

    Mat inlier_mask, homography;
    vector<KeyPoint> inliers1, inliers2;
    vector<DMatch> inlier_matches;
    if(matched1.size() >= 4) {
       begin_time = clock();
       // homography = findHomography(Points(matched1), Points(matched2),
       //                             RANSAC, ransac_thresh, inlier_mask);
      // std::cout << homography << std::endl;
       Mat affine_t = estimateRigidTransform(Points(matched1), Points(matched2), false);
       std::cout << affine_t << std::endl;
       homography = Mat::eye(3, 3, CV_64F);
       for (int i = 0; i < 2; ++i) 
        for (int j = 0; j < 3; ++j) {
            homography.at<double>(i,j) = affine_t.at<double>(i,j);
       }
       //std::cout << homography << std::endl;
       end_time = clock();
       std::cout << "Elapsed time estimate transform: " << double(end_time - begin_time) / CLOCKS_PER_SEC << std::endl;  
    }

    if(matched1.size() < 4 || homography.empty()) {
        Mat res;
        hconcat(first_frame, frame, res);
        stats.inliers = 0;
        stats.ratio = 0;
        return res;
    }
    for(unsigned i = 0; i < matched1.size(); i++) {
        //if(inlier_mask.at<uchar>(i)) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            inlier_matches.push_back(DMatch(new_i, new_i, 0));
        //}
    }
    stats.inliers = (int)inliers1.size();
    stats.ratio = stats.inliers * 1.0 / stats.matches;

    vector<Point2f> new_bb;
    perspectiveTransform(object_bb, new_bb, homography);
    //transform(object_bb, new_bb, homography);
    Mat frame_with_bb = frame.clone();
    if(stats.inliers >= bb_min_inliers) {
        drawBoundingBox(frame_with_bb, new_bb);
    }
    Mat res;
    drawMatches(first_frame, inliers1, frame_with_bb, inliers2,
                inlier_matches, res,
                Scalar(255, 0, 0), Scalar(255, 0, 0));
    return res;
}
}

int main(int argc, char **argv)
{
    if(argc < 2) {
        cerr << "Usage: " << endl
             << "akaze_track input_path" << endl
             << "  (input_path can be a camera id, like 0,1,2 or a video filename)" << endl;
        return 1;
    }

    std::string video_name = argv[1];
    std::stringstream ssFormat;
    ssFormat << atoi(argv[1]);

    VideoCapture video_in;
    if (video_name.compare(ssFormat.str())==0) {    //test str==str(num)
        video_in.open(atoi(argv[1]));
    }
    else {
        video_in.open(video_name);
    }

    if(!video_in.isOpened()) {
        cerr << "Couldn't open " << argv[1] << endl;
        return 1;
    }

    Stats stats, akaze_stats, orb_stats;
    // Ptr<AKAZE> akaze = AKAZE::create();
    // akaze->setThreshold(akaze_thresh);
    // Ptr<ORB> orb = ORB::create();
    Ptr<cuda::ORB> akaze = cuda::ORB::create(500);
    Ptr<cuda::ORB> orb = cuda::ORB::create(500);


    //Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    
    //cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1); // instantiate LSH index parameters
    // cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);       // instantiate flann search parameters
    // cv::DescriptorMatcher * matcher = new cv::FlannBasedMatcher(indexParams, searchParams);         // instantiate FlannBased matcher
    
    Ptr< cuda::DescriptorMatcher > matcher = cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
    
    example::Tracker akaze_tracker(akaze, matcher);
    example::Tracker orb_tracker(orb, matcher);

    Mat frame;
    video_in >> frame;
    namedWindow(video_name, WINDOW_NORMAL);
    cv::resizeWindow(video_name, frame.cols, frame.rows);

    cout << "Please select a bounding box, and press any key to continue." << endl;
    vector<Point2f> bb;
    cv::Rect uBox = cv::selectROI(video_name, frame);
    bb.push_back(cv::Point2f(static_cast<float>(uBox.x), static_cast<float>(uBox.y)));
    bb.push_back(cv::Point2f(static_cast<float>(uBox.x+uBox.width), static_cast<float>(uBox.y)));
    bb.push_back(cv::Point2f(static_cast<float>(uBox.x+uBox.width), static_cast<float>(uBox.y+uBox.height)));
    bb.push_back(cv::Point2f(static_cast<float>(uBox.x), static_cast<float>(uBox.y+uBox.height)));

    akaze_tracker.setFirstFrame(frame, bb, "AKAZE", stats);
    orb_tracker.setFirstFrame(frame, bb, "ORB", stats);

    Stats akaze_draw_stats, orb_draw_stats;
    Mat akaze_res, orb_res, res_frame;
    int i = 0;
    clock_t begin_time, end_time;
    for(;;) {
        i++;
        bool update_stats = (i % stats_update_period == 0);
        video_in >> frame;
        // stop the program if no more images
        if(frame.empty()) break;

        begin_time = clock();
        akaze_res = akaze_tracker.process(frame, stats);
        end_time = clock();
        std::cout << "Elapsed time akaze : " << double(end_time - begin_time) / CLOCKS_PER_SEC << std::endl; 
        akaze_stats += stats;
        if(update_stats) {
            akaze_draw_stats = stats;
        }
        begin_time = clock();
        //orb->setMaxFeatures(stats.keypoints);
        orb->setMaxFeatures(1000);
        orb_res = orb_tracker.process(frame, stats);
        end_time = clock();
        std::cout << "Elapsed time orb: " << double(end_time - begin_time) / CLOCKS_PER_SEC << std::endl; 

        orb_stats += stats;
        if(update_stats) {
            orb_draw_stats = stats;
        }

        drawStatistics(akaze_res, akaze_draw_stats);
        drawStatistics(orb_res, orb_draw_stats);
        vconcat(akaze_res, orb_res, res_frame);
        cv::imshow(video_name, res_frame);
        if(waitKey(1)==27) break; //quit on ESC button
    }
    akaze_stats /= i - 1;
    orb_stats /= i - 1;
    printStatistics("AKAZE", akaze_stats);
    printStatistics("ORB", orb_stats);
    return 0;
}
