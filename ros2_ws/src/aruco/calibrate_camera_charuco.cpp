// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html
#include <memory>
#include <iostream>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include "aruco_samples_utility.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
using std::placeholders::_1;


using namespace std;
using namespace cv;

class ImageConverter : public rclcpp::Node
{
  public:
    ImageConverter()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/cam0/image_raw", 10, std::bind(&ImageConverter::topic_callback, this, _1));
    }
  public:
    // Collect data from each frame
    vector<Mat> allCharucoCorners;
    vector<Mat> allCharucoIds;

    vector<vector<Point2f>> allImagePoints;
    vector<vector<Point3f>> allObjectPoints;

    vector<Mat> allImages;
    Size imageSize;
                int i =0;

  private:
    int topic_callback(const sensor_msgs::msg::Image & msg)
    {
        try
            {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                // Now you have the image in cv::Mat format
                cv::Mat image = cv_ptr->image;


                int squaresX = 7;
                int squaresY = 5;
                float squareLength = 0.035;
                float markerLength = 0.025;
                string outputFile = "calibration.yaml";

                bool showChessboardCorners = false;

                int calibrationFlags = 0;
                float aspectRatio = 1;

                aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
                aruco::Dictionary dictionary = aruco::getPredefinedDictionary(16);
                // Create charuco board object
                aruco::CharucoBoard board(Size(squaresX, squaresY), squareLength, markerLength, dictionary);
                aruco::CharucoParameters charucoParams;

                aruco::CharucoDetector detector(board, charucoParams, detectorParams);


                
                Mat imageCopy;

                vector<int> markerIds;
                vector<vector<Point2f>> markerCorners, rejectedMarkers;
                Mat currentCharucoCorners;
                Mat currentCharucoIds;
                vector<Point3f> currentObjectPoints;
                vector<Point2f> currentImagePoints;

                // Detect ChArUco board
                detector.detectBoard(image, currentCharucoCorners, currentCharucoIds);

                // Draw results
		
                image.copyTo(imageCopy);
                if(!markerIds.empty()) {
                    aruco::drawDetectedMarkers(imageCopy, markerCorners);
                }

                if(currentCharucoCorners.total() > 3) {
                    aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
                }

                putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                        Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

                imshow("out", imageCopy);
                waitKey(1);
		

                if(currentCharucoCorners.total() > 3) {
                    // Match image points
                    board.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);

                    if(currentImagePoints.empty() || currentObjectPoints.empty()) {
                        cout << "Point matching failed, try again." << endl;
                        return 0;
                    }

                    cout << "Frame captured" << endl;

                    allCharucoCorners.push_back(currentCharucoCorners);
                    allCharucoIds.push_back(currentCharucoIds);
                    allImagePoints.push_back(currentImagePoints);
                    allObjectPoints.push_back(currentObjectPoints);
                    allImages.push_back(image);
                        i = i + 1;
			cout << i<<endl;

                    imageSize = image.size();
                    if (i>40){
			    cout << "a" << endl;
                        if(allCharucoCorners.size() < 4) {
                            cerr << "Not enough corners for calibration" << endl;
                            return 0;
                        }

                        Mat cameraMatrix, distCoeffs;
			    cout << "b" << endl;

                        if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
                            cameraMatrix = Mat::eye(3, 3, CV_64F);
                            cameraMatrix.at<double>(0, 0) = aspectRatio;
                        }
			    cout << "c" << endl;

                        // Calibrate camera using ChArUco
                        double repError = calibrateCamera(
                            allObjectPoints, allImagePoints, imageSize,
                            cameraMatrix, distCoeffs, noArray(), noArray(), noArray(),
                            noArray(), noArray(), calibrationFlags
                        );
			    cout << "d" << endl;

                        bool saveOk =  saveCameraParams(
                            outputFile, imageSize, aspectRatio, calibrationFlags,
                            cameraMatrix, distCoeffs, repError
                        );
			    cout << "e" << endl;

                        if(!saveOk) {
                            cerr << "Cannot save output file" << endl;
                            return 0;
                        }

                        cout << "Rep Error: " << repError << endl;
                        cout << "Calibration saved to " << outputFile << endl;

                        // Show interpolated charuco corners for debugging

                        return 0;
                    }
                }
            }
        catch (cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return 0;
            }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageConverter>());
  rclcpp::shutdown();
  return 0;
}

