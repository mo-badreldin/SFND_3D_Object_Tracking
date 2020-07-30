
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
void clusterKptMatchesWithROI(BoundingBox &prevboundingBox, BoundingBox &currboundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
    double away_from_mean_perc = 50.0;
    double mean = 0;
    double num_matches = 0;
    std::vector<cv::DMatch> tempkptMatches;
    for(auto keyPt_match : kptMatches)
    {
        cv::KeyPoint currKeyPt = kptsCurr[keyPt_match.trainIdx];
        cv::KeyPoint prevKeyPt = kptsPrev[keyPt_match.queryIdx];


        if(currboundingBox.roi.contains(currKeyPt.pt)
                && prevboundingBox.roi.contains(prevKeyPt.pt))
        {
            // match provide two key points (current and previous keypoint) which both lie
            // in the two previously matched Boudning boxes of current and previous frame
            tempkptMatches.push_back(keyPt_match);

            mean+=keyPt_match.distance;
            num_matches++;
        }
    }

    mean =  mean / num_matches;
    double max_distance_threshold = (away_from_mean_perc/100) * mean;
	
    auto itr = tempkptMatches.begin();
    while(itr != tempkptMatches.end())
    {
        if(abs((*itr).distance - mean) > max_distance_threshold)
        {
            //the difference bet distance and mean is greater than 50 percent of mean value
            // considered very far and can be removed
            // removes 25% around 50 keypts
            // removes 50% around 30 keypts
            itr = tempkptMatches.erase(itr);
        }
        else
        {
            itr++;
        }
    }

    currboundingBox.kptMatches = tempkptMatches;
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
    double minXPrev = 1e9, minXCurr = 1e9;
    sort(lidarPointsPrev.begin(),lidarPointsPrev.end(),[](const LidarPoint &first, const LidarPoint &second)
         {

        return first.x < second.x;
         });

    sort(lidarPointsCurr.begin(),lidarPointsCurr.end(),[](const LidarPoint &first, const LidarPoint &second)
             {

            return first.x < second.x;
             });

    double max_diff_threshold = 1.0;
    int min_num_consecutive_pts = 50;
    int counter = 0;
    double actual_prevmin = lidarPointsPrev.begin()->x;
    for(auto itr = lidarPointsPrev.begin(); itr!=lidarPointsPrev.end();itr++)
    {
        double diff = ((*next(itr)).x -(*itr).x) * 100;
        if(diff <= max_diff_threshold)
        {
            counter++;
        }
        else
        {
            counter = 0;
            actual_prevmin = (*next(itr)).x;
        }

        if(counter > min_num_consecutive_pts)
        {
            // found 50 consecutve points
            minXPrev =  actual_prevmin;
            break;
        }
    }

    counter = 0;
    double actual_currmin = lidarPointsCurr.begin()->x;
    for(auto itr = lidarPointsCurr.begin(); itr!=lidarPointsCurr.end();itr++)
    {
        double diff = ((*next(itr)).x -(*itr).x) * 100;
        if(diff <= max_diff_threshold)
        {
            counter++;
        }
        else
        {
            counter = 0;
            actual_currmin = (*next(itr)).x;
        }

        if(counter > min_num_consecutive_pts)
        {
            // found 50 consecutve points
            minXCurr =  actual_currmin;
            break;
        }
    }

    TTC = minXCurr * (1/frameRate) / (minXPrev - minXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
     int count_array[prevFrame.boundingBoxes.size()][currFrame.boundingBoxes.size()] = {};


     auto itr = matches.begin();
     while(itr != matches.end())
     {
         cv::KeyPoint currKeyPt = currFrame.keypoints[(*itr).trainIdx];
         cv::KeyPoint prevKeyPt = prevFrame.keypoints[(*itr).queryIdx];

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
             itr++;
         }
         else
         {
             //Either of the two keypoints was not located  in a Boundingbox (size ==0)
             //Either of the two keypoints was located in more than one Boundingbox size > 1
             //Remove this keypoint match for more robust detection
             itr = matches.erase(itr);
         }
     }


     for(int prev_idx = 0; prev_idx<prevFrame.boundingBoxes.size();prev_idx++)
     {
         auto array_itr = count_array[prev_idx];
         bbBestMatches[prev_idx] =  distance(array_itr,
                                             max_element(array_itr,array_itr + currFrame.boundingBoxes.size()));
     }

     // Filter to remove non-unique matches. After this loop each previous BB match to only one BB or none
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

}
