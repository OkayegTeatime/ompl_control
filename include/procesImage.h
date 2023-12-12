#include <opencv2/opencv.hpp>

void applyMask(cv::Mat& image) {
    cv::Scalar lowerColor = cv::Scalar(154, 154, 154);  // Lower bound for (155, 155, 155)
    cv::Scalar upperColor = cv::Scalar(156, 156, 156); 
    cv::inRange(image, lowerColor, upperColor, image);
    cv::bitwise_not(mask, mask);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // Kernel size = 3x3
    cv::dilate(mask, mask, element);
}