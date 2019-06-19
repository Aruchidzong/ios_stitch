#pragma once
#ifndef CAPTURE_STITCHING_MOBILE_HPP
#define CAPTURE_STITCHING_MOBILE_HPP
#include <opencv2/core/utility.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "opencv2/stitching/detail/camera.hpp"


using namespace std;
using namespace cv;
using namespace cv::detail;

/**
 * capture a new image. if image stitching succeeds, return true; if not return false.
 * @param src   input image
 * @param dst   output panorama
 * @param mode   0: stitch
 *               1: delete last pic
 * @return true(stitching succeed)
 *         false(stitching fail)
 */
bool capture_pic(cv::Mat &src, cv::Mat &dst, int mode);

/**
 * stitching module initialization. if initialization succeeds, return true; if not return false.
 * @param img_pixel  0 default image size
 *                    1 360p
 *                    2 540p
 *                    3 720p
 * @param warptype   0 plane
 *                   1 cylindrical
 * @return true(initialization succeed)
 *         false(initialization fail)
 */
//bool capture_stitching_init(int img_pixel=0, int warptype=0, int maxedge=2000);
bool capture_stitching_init(int img_pixel = 0, int warptype = 0,  int MaxMatchNums = 5, int isWarpInit = 0 , double composeScale = 0.05, int MaxPix = 1e6,  int maxedge = 2000);
/**
 * before exiting the program, please run the function below to release resources
 */
void capture_stitching_release();

/**
 * set template pic, save features
 * @param src template image
 * @return 1 success
 *         0 fail
 */
bool set_src_feature(cv::Mat &src);
/**
 * calculate the projected points
 * @param dst target image
 * @param src_pnts target image projected points in template image coordinate
 * @param dst_pnts template image projected points in target image coordinate
 * @return 1 success
 *         0 fail
 */
bool overlap_point(cv::Mat &dst, std::vector<cv::Point> &src_pnts, std::vector<cv::Point> &dst_pnts);

/**
 * stitching N images
 * @param imglist image list
 * @param dst panorama
 * @return 1 success
 *         0 fail
 */
//bool stitching(std::vector<cv::Mat> &imglist, cv::Mat &dst);
bool stitching(std::vector<cv::Mat> &imglist, cv::Mat &dst, std::string & strLogPath);
bool stitching_ori(std::vector<cv::Mat> &imglist, cv::Mat &dst, std::string & strLogPath);
//目前用
//mode 0 正常拍摄，1 重拍
//imglist 传object为1 或 空的数据
bool stitching(std::vector <cv::Mat> &imglist, Mat &dst, int mode, std::string & strLogPath);

bool stitching(std::vector<cv::Mat> &imglist, cv::Mat &dst, std::string & strLogPath, int &findtime, int &matchtime, int &adjusttime, int &composetime);
/**
 * get stitching parameters
 * @param    sum_corner_points_    corner points of each images in the final stitched image
 * @param    result_size_        stitched image size
 * @param    camera_params_        cv::detail::CameraParams of each images
 * @param    input_scale_        input image scale
 * @param    image_map_index_    map mode index of each images
 */
void stitching_params(std::vector<std::vector<cv::Point> > &sum_corner_points_, cv::Size &result_size_, std::vector<cv::detail::CameraParams> &camera_params_, double &input_scale_, vector<cv::Point> &image_map_index_);

void overlap_test(cv::Mat &src, std::string &timestring);

#endif // CAPTURE_STITCHING_MOBILE_HPP
