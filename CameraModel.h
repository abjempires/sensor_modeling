#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

#include <vector>
#include <opencv2/opencv.hpp>

// Intrinsics for the camera
struct CameraIntrinsics {
    double fx;  // Focal length in pixels (x-axis)
    double fy;  // Focal length in pixels (y-axis)
    double cx;  // Principal point x-coordinate
    double cy;  // Principal point y-coordinate
    double k1, k2, p1, p2;  // Distortion coefficients
};

// Extrinsics for the camera (relative to the vehicle)
struct CameraExtrinsics {
    double roll;  // Rotation about x-axis
    double pitch; // Rotation about y-axis
    double yaw;   // Rotation about z-axis
    double tx, ty, tz;  // Translation
};

class CameraSensor {
public:
    CameraSensor(const CameraIntrinsics& intrinsics, const CameraExtrinsics& extrinsics, int width, int height);

    cv::Mat captureImage(const cv::Mat& scene);
    cv::Mat detectLanes(const cv::Mat& image);
    std::vector<cv::Rect> detectObjects(const cv::Mat& image);

private:
    CameraIntrinsics intrinsics;
    CameraExtrinsics extrinsics;
    int imageWidth;
    int imageHeight;

    cv::Mat undistortImage(const cv::Mat& image);
};

#endif
