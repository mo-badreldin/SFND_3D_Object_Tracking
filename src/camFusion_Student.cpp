
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
     int count_array[prevFrame.boundingBoxes.size()][currFrame.boundingBoxes.size()] = {};

     cout << "Bounding Boxes Size current: " << currFrame.boundingBoxes.size() << " Bounding Boxes Size Prev: " << prevFrame.boundingBoxes.size() << endl;

     for(auto keyPt_match : matches)
     {
         cv::KeyPoint currKeyPt = currFrame.keypoints[keyPt_match.trainIdx];
         cv::KeyPoint prevKeyPt = prevFrame.keypoints[keyPt_match.queryIdx];

         vector<int> currBoundingBoxIDs;
         for(auto currImgBoundingBox : currFrame.boundingBoxes)
         {
             if(currImgBoundingBox.roi.contains(currKeyPt.pt))
             {
                 currBoundingBoxIDs.push_back(currImgBoundingBox.boxID);
             }
         }

         vector<int> prevBoundingBoxIDs;
         for(auto prevImgBoundingBox : prevFrame.boundingBoxes)
         {
             if(prevImgBoundingBox.roi.contains(prevKeyPt.pt))
             {
                 prevBoundingBoxIDs.push_back(prevImgBoundingBox.boxID);
             }
         }

         if(currBoundingBoxIDs.size() == 1 && prevBoundingBoxIDs.size() == 1)
         {
             count_array[prevBoundingBoxIDs[0]][currBoundingBoxIDs[0]] ++;
         }
     }


     for(int prev_idx = 0; prev_idx<prevFrame.boundingBoxes.size();prev_idx++)
     {
         auto array_itr = count_array[prev_idx];
         bbBestMatches[prev_idx] =  distance(array_itr,
                                             max_element(array_itr,array_itr + currFrame.boundingBoxes.size()));
         //Printing code for debugging
//         cout << "Prev BB Idx: " << prev_idx << endl;
//         for(int cur_idx = 0; cur_idx<currFrame.boundingBoxes.size();cur_idx++)
//         {
//             cout << "Current BB Idx: " << cur_idx << "Current BB Value: " << count_array[prev_idx][cur_idx];
//             cout << endl;
//         }
//         cout << endl;
     }

     std::cout << "Before FIlter: \n" ;
     for(auto itr = bbBestMatches.begin(); itr!=bbBestMatches.end();itr++)
     {
         std::cout << (*itr).first <<"\t" << (*itr).second << "\n";
     }

     auto primary_itr = bbBestMatches.begin();
     while(primary_itr != bbBestMatches.end())
     {
         auto secondary_itr = next(primary_itr) ;
         while (secondary_itr != bbBestMatches.end())
         {
             if(primary_itr->second == secondary_itr->second)
             {
                 if(count_array[primary_itr->first][primary_itr->second] >=
                 count_array[secondary_itr->first][secondary_itr->second])
                 {
                     secondary_itr = bbBestMatches.erase(secondary_itr);
                     continue;
                 }
                 else
                 {
                     primary_itr = bbBestMatches.erase(primary_itr);
                     secondary_itr = next(primary_itr) ;
                     continue;
                 }
             }
             else
             {
                 secondary_itr++;
             }
         }

         primary_itr++;
     }

     std::cout << "After FIlter: \n" ;
     for(auto newitr = bbBestMatches.begin(); newitr!=bbBestMatches.end();newitr++)
     {
         std::cout << (*newitr).first <<"\t" << (*newitr).second << "\n";
     }
}


#if 0
//just for testing and comparing
void matchBoundingBoxes2(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // use the matched keypoints to identify the matching between bounding boxes-
    // first, associate the keypoints to detected bounding boxes and then check which bounding boxes are relevant.

    // loop through all keypoint matches and assign them the bounding boxes they belong to.

    cv::Mat bbMatchTable = cv::Mat::zeros(prevFrame.boundingBoxes.size(), currFrame.boundingBoxes.size(), CV_32S);

    for (const auto &match : matches)
    {
        const cv::KeyPoint &currKpt = currFrame.keypoints[ match.trainIdx ];
        const cv::KeyPoint &prevKpt = prevFrame.keypoints[ match.queryIdx ];

        for (const BoundingBox& prevBoundingBox : prevFrame.boundingBoxes)
        {
            for (const BoundingBox& currBoundingBox : currFrame.boundingBoxes)
            {
                if (prevBoundingBox.roi.contains(prevKpt.pt) && currBoundingBox.roi.contains(currKpt.pt))
                {
                    bbMatchTable.at<int>(prevBoundingBox.boxID, currBoundingBox.boxID)++;
                }
            }
        }
    }

    // loop through the prev frame counts
    for (int i = 0; i < bbMatchTable.rows; i++)
    {
        int bestMatchCounts = 0;
        int bestMatchIndex = -1;

        // loop through the curr frame counts
        for (int j = 0; j < bbMatchTable.cols; j++)
        {
            if (bbMatchTable.at<int>(i, j) > 0 && bbMatchTable.at<int>(i, j) > bestMatchCounts)
            {
                bestMatchCounts = bbMatchTable.at<int>(i, j);
                bestMatchIndex = j;
            }
        }

        if (bestMatchIndex != -1)
        {
            bbBestMatches.emplace(i, bestMatchIndex);
        }
    }

    std::cout << "MEthod 2: \n" ;
    for(auto newitr = bbBestMatches.begin(); newitr!=bbBestMatches.end();newitr++)
    {
        std::cout << (*newitr).first <<"\t" << (*newitr).second << "\n";
    }
}
#endif

