#include "tflow-render.hpp"

const cv::Scalar blue      = cv::Scalar{ 255, 0,     0 }; // B/G/R
const cv::Scalar cyan      = cv::Scalar{ 255, 255, 128 }; // B/G/R
const cv::Scalar violet    = cv::Scalar{ 255,   0, 153 }; // B/G/R
const cv::Scalar violet_l  = cv::Scalar{ 255, 139, 191 }; // B/G/R
const cv::Scalar violet_ll = cv::Scalar{ 255, 163, 204 }; // B/G/R
const cv::Scalar red       = cv::Scalar{ 0,     0, 255 }; // B/G/R
const cv::Scalar yellow    = cv::Scalar{ 0,   255, 255 }; // B/G/R
const cv::Scalar orange    = cv::Scalar{ 0,   128, 255 }; // B/G/R
const cv::Scalar green     = cv::Scalar{ 0,   255,   0 };
const cv::Scalar coral     = cv::Scalar{ 129, 129, 241 };
const cv::Scalar white     = cv::Scalar{ 255, 255, 255 };
const cv::Scalar gray      = cv::Scalar{  92,  92,  92 };
const cv::Scalar black     = cv::Scalar{   0,   0,   0 };

cv::Rect2f rect2f_offset(const cv::Rect2f& off, const cv::Rect2f& rect_in) { 
    cv::Rect2f rect_out(rect_in);
    rect_out.x += off.x;
    rect_out.y += off.y;
    return rect_out;
};

cv::Point2f point2f_offset(const cv::Rect2f& off, const cv::Point2f& point_in) { 
    return cv::Point2f(point_in.x + off.x, point_in.y + off.y);
};
