
#pragma once
#ifndef CAPTURE_STITCHING_MOBILE_HPP
#define CAPTURE_STITCHING_MOBILE_HPP

#include "opencv2/opencv_modules.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/matchers.hpp"


/**
 * stitching module initialization. if initialization succeeds, return true; if not return false.
 * @return true(initialization succeed)
 *         false(initialization fail)
 */
bool IdtStitchInit(const std::string & strLogPath);

///**
// * set template pic, save features
// * @param src template image
// * @param src_features template img feature
// * @return 1 success
// *         0 fail
// */
//bool IdtSetTemplate(cv::Mat &currentFrame);

/**
 * calculate the projected points
 * @param dst target image
 * @param src_features template img feature
 * @param src_pnts target image projected points in template image coordinate
 * @param dst_pnts template image projected points in target image coordinate
 * @return 1 success
 *         0 fail
 */
bool IdtCalOverLap(cv::Mat &currentFrame, std::vector<cv::Point> &dst_pnts, float &Score);

/**
 * capture a new image. if image stitching succeeds, return true; if not return false.
 * @param src   input image
 * @param dst   output panorama
 * @param mode   0: stitch
 *               1: delete last pic
 * @param strLogPath   output algorithm log file
 * @return true(stitching succeed)
 *         false(stitching fail)
 */
bool IdtStitch(const cv::Mat &src, const cv::Point3f angle, cv::Mat &dst,  int & bestPOV, double Homo[9]);

void IdtRetry(cv::Mat & pano);

/**
 * before exiting the program, please run the function below to release resources
 */
void IdtStitchClean();

#endif // CAPTURE_STITCHING_MOBILE_HPP
