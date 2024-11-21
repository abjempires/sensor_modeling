#include "CameraModel.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

CameraSensor::CameraSensor(const CameraIntrinsics& intr, const CameraExtrinsics& extr, int width, int height)
    : intrinsics(intr), extrinsics(extr), imageWidth(width), imageHeight(height) {}

cv::Mat CameraSensor::undistortImage(const cv::Mat& image) {
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        intrinsics.fx, 0, intrinsics.cx, 
        0, intrinsics.fy, intrinsics.cy, 
        0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << intrinsics.k1, intrinsics.k2, intrinsics.p1, intrinsics.p2, 0);

    cv::Mat undistorted;
    cv::undistort(image, undistorted, cameraMatrix, distCoeffs);
    return undistorted;
}

cv::Mat CameraSensor::captureImage(const cv::Mat& scene) {
    // Simulate an image from the scene (apply perspective transform, noise, etc.)
    cv::Mat output;
    cv::resize(scene, output, cv::Size(imageWidth, imageHeight));
    return output;
}

cv::Mat CameraSensor::detectLanes(const cv::Mat& image) {
    cv::Mat gray, edges;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, edges, 50, 150);

    // Hough transform to detect lane lines
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

    cv::Mat lanes = image.clone();
    for (const auto& line : lines) {
        cv::line(lanes, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 2);
    }
    return lanes;
}

std::vector<cv::Rect> CameraSensor::detectObjects(const cv::Mat& image) {
    cv::CascadeClassifier classifier("haarcascade_car.xml");
    std::vector<cv::Rect> objects;
    classifier.detectMultiScale(image, objects, 1.1, 3, 0, cv::Size(30, 30));
    return objects;
}
