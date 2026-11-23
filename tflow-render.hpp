#pragma once

#include "tflow-build-cfg.hpp"

#if _WIN32
#include <WinSock2.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <opencv2/opencv.hpp>

extern const cv::Scalar blue;
extern const cv::Scalar cyan;
extern const cv::Scalar violet;
extern const cv::Scalar violet_l;
extern const cv::Scalar violet_ll;
extern const cv::Scalar red;
extern const cv::Scalar yellow;
extern const cv::Scalar orange;
extern const cv::Scalar green;
extern const cv::Scalar coral;
extern const cv::Scalar white;
extern const cv::Scalar gray;

extern cv::Rect2f rect2f_offset(const cv::Rect2f& off, const cv::Rect2f& rect_in);
extern cv::Point2f point2f_offset(const cv::Rect2f& off, const cv::Point2f& point_in);
