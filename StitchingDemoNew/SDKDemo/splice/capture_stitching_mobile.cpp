#include "capture_stitching_mobile.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "opencv2/opencv_modules.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include <numeric>
#include <time.h> //zt
#include <sys/time.h> //zt

using namespace std;
using namespace cv;
using namespace cv::detail;

#define ENABLE_LOG 0
#define LOG(msg) std::cout << msg
#define LOGLN(msg) std::cout << msg << std::endl

typedef std::set<std::pair<int,int> > MatchesSet;//zt

static bool try_cuda = false;
static double work_megapix = 0.2;  //特征计算图像尺寸
static double seam_megapix = 0.1;  //接缝计算图像尺寸
static double compose_megapix = 0.05;  //合成图像尺寸  //0.05
static bool try_compose = true;  //是否合成图片
static float conf_thresh = 0.9;//0.9;  //匹配置信度
//static int match_num = 30;
static int inliner_num = 16;  //最小匹配内点数
static double confidence_thresh = 1.0;
static string features_type = "gftt-sift";//"orb"; ;  //拼接使用特征
static string matcher_type = "homography";  //匹配矩阵求取方法
static string estimator_type = "homography";  //相机估计方法
static int ba_iter_num = 10;  //bundleadjustment 迭代次数
static string ba_cost_func = "ray";//"ray",原始是ray;  //bundleadjustment迭代方法  "reproj" "ray" "affine" "no"
static string ba_refine_mask = "xxxxx";  //BA mask，只对reproject有效
static bool do_wave_correct = false;
static WaveCorrectKind wave_correct = detail::WAVE_CORRECT_VERT;
static bool save_graph = false;
static string warp_type = "cylindrical";  //warping 类型
static int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
static float match_conf = 0.3f;  //匹配最低置信度
static string seam_find_type = "voronoi";  //接缝线求取方法
static int blend_type = Blender::MULTI_BAND;  //图片合成方法
static int timelapse_type = Timelapser::AS_IS;
static float blend_strength = 5;  //合成强度
//static string result_name = "result.jpg";
static bool timelapse = false;
static int range_width = 2;  //相邻range_width大小的图片都要进行匹配

static double work_scale = 1;  //特征计算scale
static double seam_scale = 1;  //接缝计算scale
static double compose_scale = 1;  //合成scale
static bool is_work_scale_set = false; //是否设定了特征计算scale
static bool is_seam_scale_set = false; //是否设定了接缝计算scale
static bool is_compose_scale_set = false; //是否设定了合成scale
static double seam_work_aspect = 1; //接缝计算scale与特征计算scale比例
static double compose_seam_aspect = 1; //合成scale与接缝计算scale比例
static double compose_work_aspect = 1; //合成scale与特征计算scale比例

static cv::Mat matchmask;
static cv::Mat overlapmask;
//
static Ptr<FeaturesFinder> finder; //特征计算指针
static Ptr<FeaturesMatcher> matcher; //特征匹配指针
static Ptr<Estimator> estimator; //相机估计指针
static Ptr<detail::BundleAdjusterBase> adjuster; //相机BA指针
static Ptr<SeamFinder> seam_finder; //接缝计算指针
static Ptr<WarperCreator> warper_creator; //warping计算指针
static Ptr<Blender> blender; //合成计算指针
static Ptr<FeaturesMatcher> affine_matcher;
static Ptr<Estimator> affine_estimator;
static Ptr<WarperCreator> affine_warper_creator;

//两两拼接维护list
static vector<Mat> capture_input_images;  //输入图片list
static vector<Mat> capture_images;  //输入特征计算图片list
static vector<ImageFeatures> capture_features;  //特征list
static vector<Size> capture_img_sizes; //特征计算size list
static vector<MatchesInfo> capture_pairwise_matches; //特征匹配 list1
static vector<MatchesInfo> capture_pairwise_matches2; //特征匹配 list2
static vector<Mat> pano_vec; //全景图list
static Mat pano_bak; //全景图备份

//compose list
static vector<Point> sum_corners;  //合成图片左上角坐标 list
static vector<UMat> sum_masks_warped;  //合成图片warping mask list
static vector<UMat> sum_images_warped;  //合成图片warping list
static vector<Size> sum_sizes;  //合成图片size list
static vector<UMat> sum_masks;  //合成图片mask list
static vector<CameraParams> cameras;  //相机 list

//save parameters
static vector<vector<Point>> sum_corner_points;  //合成图片4个点坐标 list
static Size result_size; //合成图片size
static vector<CameraParams> camera_params;  //相机参数 list
static vector<CameraParams> global_camera_params;  //相机参数 list //zt
static vector<Point> image_map_index; //每张图像的地图模式坐标

//重拍备份
static Mat capture_input_images_bak; //输入图片备份
static Mat capture_images_bak; //特征计算图片备份
static ImageFeatures capture_features_bak; //特征备份
static Size capture_img_sizes_bak; //图片尺寸备份
static MatchesInfo capture_pairwise_matches_bak; //特征匹配备份
static MatchesInfo capture_pairwise_matches2_bak; //特征匹配2备份

static bool is_corners_set = false;
static std::vector<Point2f> obj_corners(4);
static std::vector<Point2f> scene_corners(4);
static bool get_pairwise_bak = false;

//输入图片尺寸
static Mat input;  //输入图片变量
static double input_megapix = 0.2;  //输入图片默认尺寸
static double input_scale;  //输入图片scale
static bool is_input_scale_set = false;  //是否设定了输入图片scale
static int work_pixel = 0;  //输入图片尺寸选择
static vector<int> work_pixel_vec;  //输入图片尺寸list

//
static float warped_image_scale = 0;  //warping scale
static vector<float> warped_scales;  //warping scale list
static vector<UMat> sum_seam_masks;  //接缝计算mask
static vector<Mat> blend_imgs;  //合成图片 list
static vector<Mat> blend_masks;  //合成图片mask list
vector <Point> blend_corners;  //合成图片左上角 list
vector <Size> blend_sizes;  //合成图片size list

//重复区域
Ptr<FeatureDetector> detector;  //重复区域特征检测指针
Ptr<DescriptorExtractor> extractor;  //重复区域特征描述指针
Ptr<FastFeatureDetector> fastdetector;  //重复区域fast特征计算指针
std::vector<Point2f> obj_pt, scene_pt;  //重复区域匹配点list
//std::vector<DMatch> tmpmatches, goodmatches;
Ptr<cv::flann::Index> tree;  //重复区域flann索引指针
Ptr<cv::flann::SearchParams> flann_search;  //重复区域flann搜索指针
ImageFeatures src_features, dst_features;  //重复区域模板图特征，目标图特征
int maxkeypoint = 5000;  //重复区域最大匹配点数
//vector<Point> dst_pnts(4);
//Mat input_src;

//全景图最大边长
int pano_max_edge = 2000;

//zt
//////////////////
int MaxPix_ = 0;
int InputImgNums = 0;
int MaxImgNums_ = 0; //大于MaxImgNums_张就更新最前面的一张, 实际最多匹配MaxImgNums_张图片
static vector <Mat> global_images;
static vector <Mat> global_input_images;
static vector <ImageFeatures> global_img_feature(0); //大小会随着特征增加而改变
static vector <MatchesInfo> global_pairwise_matches; // 特征匹配结果

static vector <Mat> all_images;
static vector <Mat> all_input_images;
static vector <ImageFeatures> all_img_feature(0); //
static vector <MatchesInfo> all_pairwise_matches; //


vector <Point> global_corners;
Point corner_tmp; //保存当前corner最后一个元素
////////////////

//zt adjust

Mat cam_params_;
int num_images_;
const ImageFeatures *features_;
const MatchesInfo *pairwise_matches_;
std::vector<std::pair<int,int> > edges_;
int total_num_matches_;

int num_params_per_cam_ = 4; //射线发散
int num_errs_per_measurement_ = 3;//射线发散
Mat refinement_mask_;

//Levenberg-Marquardt algorithm termination criteria
TermCriteria term_criteria_;
Mat err1_, err2_;

struct IncDistance
{
    IncDistance(std::vector<int> &vdists) : dists(&vdists[0]) {}
    void operator ()(const GraphEdge &edge) { dists[edge.to] = dists[edge.from] + 1; }
    int* dists;
};


#ifndef ADJUST
//////////////////////////////////////////
//BundleAdjusterRay::
void setUpInitialCameraParams(const std::vector<CameraParams> &cameras)
{
    
    cam_params_.create(num_images_ * 4, 1, CV_64F);
    SVD svd;
    for (int i = 0; i < num_images_; ++i)
    {
        cam_params_.at<double>(i * 4, 0) = cameras[i].focal;
        
        //for (int i = 0; i < cameras.rows; ++i)
        {
            std::cout << "cam_params_ " << cam_params_.at<double>(i*4,0) << std::endl;
            
        }
        
        svd(cameras[i].R, SVD::FULL_UV);
        Mat R = svd.u * svd.vt;
        if (determinant(R) < 0)
            R *= -1;
        
        Mat rvec;
        Rodrigues(R, rvec);
        CV_Assert(rvec.type() == CV_32F);
        cam_params_.at<double>(i * 4 + 1, 0) = rvec.at<float>(0, 0);
        cam_params_.at<double>(i * 4 + 2, 0) = rvec.at<float>(1, 0);
        cam_params_.at<double>(i * 4 + 3, 0) = rvec.at<float>(2, 0);
        
        std::cout << "cam_params_ " << cam_params_.at<double>(i*4 + 1,0) << std::endl;
        std::cout << "cam_params_ " << cam_params_.at<double>(i*4 + 2,0) << std::endl;
        std::cout << "cam_params_ " << cam_params_.at<double>(i*4 + 3,0) << std::endl;
    }
}

//BundleAdjusterRay::
void obtainRefinedCameraParams(std::vector<CameraParams> &cameras)
{
    for (int i = 0; i < num_images_; ++i)
    {
        cameras[i].focal = cam_params_.at<double>(i * 4, 0);
        
        Mat rvec(3, 1, CV_64F);
        rvec.at<double>(0, 0) = cam_params_.at<double>(i * 4 + 1, 0);
        rvec.at<double>(1, 0) = cam_params_.at<double>(i * 4 + 2, 0);
        rvec.at<double>(2, 0) = cam_params_.at<double>(i * 4 + 3, 0);
        Rodrigues(rvec, cameras[i].R);
        
        Mat tmp;
        cameras[i].R.convertTo(tmp, CV_32F);
        cameras[i].R = tmp;
    }
}

//BundleAdjusterRay::
void calcError(Mat &err)
{
    err.create(total_num_matches_ * 3, 1, CV_64F);    //定义误差矩阵
    
    int match_idx = 0;    //表示重映射误差的索引
    //遍历最大生成树的边
    for (size_t edge_idx = 0; edge_idx < edges_.size(); ++edge_idx)
    {
        int i = edges_[edge_idx].first;    //表示边的始端图像
        int j = edges_[edge_idx].second;    //表示边的终端图像
        double f1 = cam_params_.at<double>(i * 4, 0);    //始端图像的fu
        double f2 = cam_params_.at<double>(j * 4, 0);    //终端图像的fu
        
        double R1[9];
        Mat R1_(3, 3, CV_64F, R1);
        Mat rvec(3, 1, CV_64F);
        rvec.at<double>(0, 0) = cam_params_.at<double>(i * 4 + 1, 0);    //始端图像的rx
        rvec.at<double>(1, 0) = cam_params_.at<double>(i * 4 + 2, 0);    //始端图像的ry
        rvec.at<double>(2, 0) = cam_params_.at<double>(i * 4 + 3, 0);    //始端图像的rz
        //旋转向量rvec由Rodrigues算法得到始端图像的旋转矩阵R
        Rodrigues(rvec, R1_);
        
        double R2[9];
        Mat R2_(3, 3, CV_64F, R2);
        rvec.at<double>(0, 0) = cam_params_.at<double>(j * 4 + 1, 0);    //终端图像的rx
        rvec.at<double>(1, 0) = cam_params_.at<double>(j * 4 + 2, 0);    //终端图像的ry
        rvec.at<double>(2, 0) = cam_params_.at<double>(j * 4 + 3, 0);    //终端图像的rz
        //旋转向量rvec由Rodrigues算法得到终端图像的旋转矩阵R
        Rodrigues(rvec, R2_);
        
        const ImageFeatures& features1 = features_[i];    //得到始端图像的特征
        const ImageFeatures& features2 = features_[j];    //得到终端图像的特征
        //得到两者的图像匹配信息
        const MatchesInfo& matches_info = pairwise_matches_[i * num_images_ + j];
        //为始端图像的相机内参数矩阵K赋值
        Mat_<double> K1 = Mat::eye(3, 3, CV_64F);
        K1(0,0) = f1; K1(0,2) = features1.img_size.width * 0.5;
        K1(1,1) = f1; K1(1,2) = features1.img_size.height * 0.5;
        //为终端图像的相机内参数矩阵K赋值
        Mat_<double> K2 = Mat::eye(3, 3, CV_64F);
        K2(0,0) = f2; K2(0,2) = features2.img_size.width * 0.5;
        K2(1,1) = f2; K2(1,2) = features2.img_size.height * 0.5;
        //计算两个单应矩阵
        Mat_<double> H1 = R1_ * K1.inv();
        Mat_<double> H2 = R2_ * K2.inv();
        
        for (size_t k = 0; k < matches_info.matches.size(); ++k)    //遍历匹配点对
        {
            if (!matches_info.inliers_mask[k])    //表示不是内点
                continue;
            
            const DMatch& m = matches_info.matches[k];    //表示内点信息
            
            Point2f p1 = features1.keypoints[m.queryIdx].pt;    //表示内点的一个点
            //由单应矩阵得到p1点的三维空间坐标
            double x1 = H1(0,0)*p1.x + H1(0,1)*p1.y + H1(0,2);
            double y1 = H1(1,0)*p1.x + H1(1,1)*p1.y + H1(1,2);
            double z1 = H1(2,0)*p1.x + H1(2,1)*p1.y + H1(2,2);
            double len = sqrt(x1*x1 + y1*y1 + z1*z1);    //式62
            x1 /= len; y1 /= len; z1 /= len;    //式63
            
            Point2f p2 = features2.keypoints[m.trainIdx].pt;    //表示内点的另一个点
            //由单应矩阵得到p2点的三维空间坐标
            double x2 = H2(0,0)*p2.x + H2(0,1)*p2.y + H2(0,2);
            double y2 = H2(1,0)*p2.x + H2(1,1)*p2.y + H2(1,2);
            double z2 = H2(2,0)*p2.x + H2(2,1)*p2.y + H2(2,2);
            len = sqrt(x2*x2 + y2*y2 + z2*z2);    //式62
            x2 /= len; y2 /= len; z2 /= len;    //式63
            
            double mult = sqrt(f1 * f2);    //式64的根号内的部分
            //式64
            err.at<double>(3 * match_idx, 0) = mult * (x1 - x2);
            err.at<double>(3 * match_idx + 1, 0) = mult * (y1 - y2);
            err.at<double>(3 * match_idx + 2, 0) = mult * (z1 - z2);
            
            match_idx++;    //累计
        }
    }
}

void calcDeriv(const Mat &err1, const Mat &err2, double h, Mat res)
{
    //res.create(err1.rows, err1.cols,err1.type());
    for (int i = 0; i < err1.rows; ++i)
    {
        // std::cout << "err2data0 = " << err2.at<double>(i, 0) << ",  err1data0 = "<< err1.at<double>(i, 0) << std::endl;
        //     double err2data = err2.at<double>(i, 0);
        //      double err1data = err1.at<double>(i, 0);
        //        if(err1data != err2data)
        //        {
        //            std::cout << "i = " << i << ",   err2data = " << err2data << ",  err1data = "<< err1data<<std::endl;
        //        }
        res.at<double>(i, 0) = (err2.at<double>(i, 0) - err1.at<double>(i, 0)) / h;
    }
    // printf("222222\n"); //zt
    
}

//BundleAdjusterRay::
void calcJacobian(Mat &jac)
{
    jac.create(total_num_matches_ * 3, num_images_ * 4, CV_64F);
    
    double val;
    const double step = 1e-3;
    
    for (int i = 0; i < num_images_; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            
            
            val = cam_params_.at<double>(i * 4 + j, 0);
            cam_params_.at<double>(i * 4 + j, 0) = val - step; //0us
            
            
            struct timeval eriv_start, eriv_end;
            gettimeofday( &eriv_start, NULL );
            calcError(err1_);
            
            gettimeofday( &eriv_end, NULL );
            int erivtimeuse = 1000000 * ( eriv_end.tv_sec - eriv_start.tv_sec ) + eriv_end.tv_usec - eriv_start.tv_usec;
            printf("eriv time: %d us\n", erivtimeuse);
            cam_params_.at<double>(i * 4 + j, 0) = val + step;
            calcError(err2_);
            
            
            calcDeriv(err1_, err2_, 2 * step, jac.col(i * 4 + j));  //500us /per
            
            cam_params_.at<double>(i * 4 + j, 0) = val;
        }
    }
}

void findMaxSpanningTree_idt(int num_images, const std::vector<MatchesInfo> &pairwise_matches,
                             Graph &span_tree, std::vector<int> &centers)
{
    Graph graph(num_images);
    std::vector<GraphEdge> edges;
    
    // Construct images graph and remember its edges
    for (int i = 0; i < num_images; ++i)
    {
        for (int j = 0; j < num_images; ++j)
        {
            if (pairwise_matches[i * num_images + j].H.empty())
                continue;
            float conf = static_cast<float>(pairwise_matches[i * num_images + j].num_inliers);
            graph.addEdge(i, j, conf);
            edges.push_back(GraphEdge(i, j, conf));
        }
    }
    
    DisjointSets comps(num_images);
    span_tree.create(num_images);
    std::vector<int> span_tree_powers(num_images, 0);
    
    // Find maximum spanning tree
    sort(edges.begin(), edges.end(), std::greater<GraphEdge>());
    for (size_t i = 0; i < edges.size(); ++i)
    {
        int comp1 = comps.findSetByElem(edges[i].from);
        int comp2 = comps.findSetByElem(edges[i].to);
        if (comp1 != comp2)
        {
            comps.mergeSets(comp1, comp2);
            span_tree.addEdge(edges[i].from, edges[i].to, edges[i].weight);
            span_tree.addEdge(edges[i].to, edges[i].from, edges[i].weight);
            span_tree_powers[edges[i].from]++;
            span_tree_powers[edges[i].to]++;
        }
    }
    
    // Find spanning tree leafs
    std::vector<int> span_tree_leafs;
    for (int i = 0; i < num_images; ++i)
        if (span_tree_powers[i] == 1)
            span_tree_leafs.push_back(i);
    
    // Find maximum distance from each spanning tree vertex
    std::vector<int> max_dists(num_images, 0);
    std::vector<int> cur_dists;
    for (size_t i = 0; i < span_tree_leafs.size(); ++i)
    {
        cur_dists.assign(num_images, 0);
        span_tree.walkBreadthFirst(span_tree_leafs[i], IncDistance(cur_dists));
        for (int j = 0; j < num_images; ++j)
            max_dists[j] = std::max(max_dists[j], cur_dists[j]);
    }
    
    // Find min-max distance
    int min_max_dist = max_dists[0];
    for (int i = 1; i < num_images; ++i)
        if (min_max_dist > max_dists[i])
            min_max_dist = max_dists[i];
    
    // Find spanning tree centers
    centers.clear();
    for (int i = 0; i < num_images; ++i)
        if (max_dists[i] == min_max_dist)
            centers.push_back(i);
    CV_Assert(centers.size() > 0 && centers.size() <= 2);
}
Mat refinementMask() { return refinement_mask_.clone(); }
void setRefinementMask(const Mat &mask)
{
    CV_Assert(mask.type() == CV_8U && mask.size() == Size(3, 3));
    refinement_mask_ = mask.clone();
}

TermCriteria termCriteria() { return term_criteria_; }

void setTermCriteria(const TermCriteria& term_criteria) { term_criteria_ = term_criteria; }

//BundleAdjusterBase::
bool bundleadjust(const std::vector<ImageFeatures> &features,
                  const std::vector<MatchesInfo> &pairwise_matches,
                  std::vector<CameraParams> &cameras)
{
    struct timeval start, end;
    gettimeofday( &start, NULL );
    setTermCriteria(TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, ba_iter_num, DBL_EPSILON));
    
    //    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    //    if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
    //    if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
    //    if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
    //    if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
    //    if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
    //    setRefinementMask(refine_mask);
    
    num_images_ = static_cast<int>(features.size());
    features_ = &features[0];
    pairwise_matches_ = &pairwise_matches[0];
    
    struct timeval init_start, init_end;
    gettimeofday( &init_start, NULL );
    
    setUpInitialCameraParams(cameras);
    
    for (int i = 0; i < cam_params_.rows; ++i)
    {
        std::cout << "cam_params 2222_ " << cam_params_.at<double>(i,0) << std::endl;
        
    }
    
    gettimeofday( &init_end, NULL );
    int inittimeuse = 1000000 * ( init_end.tv_sec - init_start.tv_sec ) + init_end.tv_usec - init_start.tv_usec;
    printf("init time: %d us\n", inittimeuse);  //93us
    // Leave only consistent image pairs
    edges_.clear();
    
    //    for (int i = 0; i < num_images_ - 1; ++i)
    //    {
    //        for (int j = i + 1; j < num_images_; ++j)
    //        {
    //            const MatchesInfo& matches_info = pairwise_matches_[i * num_images_ + j];
    //            //if (matches_info.confidence > conf_thresh)
    //            if (matches_info.confidence > conf_thresh && matches_info.num_inliers >= 32) //zt
    //                edges_.push_back(std::make_pair(i, j));
    //        }
    //    }
    for (int i = 0; i < num_images_ - 1; ++i)
    {
        for (int j = i + 1; j < num_images_; ++j)
        {
            const MatchesInfo& matches_info = pairwise_matches_[i * num_images_ + j];
            if (matches_info.confidence > conf_thresh && matches_info.num_inliers >= 6 /*&& ((j - 1) <= i)*/)
                //if (matches_info.confidence > conf_thresh && matches_info.num_inliers >= 32 && (j - 1) <= i) //zt,一般来说相邻的两张图才满足
                edges_.push_back(std::make_pair(i, j));
        }
    }
    
    // Compute number of correspondences
    total_num_matches_ = 0;
    //    for (size_t i = 0; i < edges_.size(); ++i)
    //        total_num_matches_ += static_cast<int>(pairwise_matches[edges_[i].first * num_images_ +
    //                                                                edges_[i].second].num_inliers);
    for (size_t i = 0; i < edges_.size(); ++i)
    {
        int pair_match_inlier = pairwise_matches[edges_[i].first * num_images_ + edges_[i].second].num_inliers;
        
        total_num_matches_ += pair_match_inlier;
    }
    std::cout << "total num matches is " << total_num_matches_ << std::endl;
    CvLevMarq solver(num_images_ * num_params_per_cam_,
                     total_num_matches_ * num_errs_per_measurement_,
                     term_criteria_);
    
    Mat err, jac;
    CvMat matParams = cam_params_;
    //std::cout << "param width = " << solver.param->width << ", param height = " << solver.param->height << std::endl;
    cvCopy(&matParams, solver.param);
    
    
    struct timeval for_start, for_end;
    gettimeofday( &for_start, NULL );
    
    int iter = 0;
    int temp = 0;
    for(;;)
    {
        const CvMat* _param = 0;
        CvMat* _jac = 0;
        CvMat* _err = 0;
        
        bool proceed = solver.update(_param, _jac, _err);
        
        cvCopy(_param, &matParams);
        /////
        for (int i = 0; i < _param->rows; i++){
            for (int j = 0; j < _param->cols; j++){
                printf("%.2f  ", CV_MAT_ELEM(*_param, float, i, j) );
            }
            printf("\n");
        }
        for (int i = 0; i < cam_params_.rows; ++i)
        {
            std::cout << "cam_params 44444 " << cam_params_.at<double>(i,0) << std::endl;
            
        }
        /////
        
        if (!proceed || !_err)
            break;
        
        temp++;
        struct timeval jac_start, jac_end;
        gettimeofday( &jac_start, NULL );
        if (_jac)
        {
            calcJacobian(jac);
            CvMat tmp = jac;
            cvCopy(&tmp, _jac);
        }
        gettimeofday( &jac_end, NULL );
        int jactimeuse = 1000000 * ( jac_end.tv_sec - jac_start.tv_sec ) + jac_end.tv_usec - jac_start.tv_usec;
        printf("jac time: %d us\n", jactimeuse);
        
        struct timeval err_start, err_end;
        gettimeofday( &err_start, NULL );
        if (_err)
        {
            calcError(err);
            iter++;
            CvMat tmp = err;
            cvCopy(&tmp, _err);
        }
        
        gettimeofday( &err_end, NULL );
        int errtimeuse = 1000000 * (err_end.tv_sec - err_start.tv_sec ) + err_end.tv_usec - err_start.tv_usec;
        //printf("err time: %d us\n", errtimeuse);
    }
    for (int i = 0; i < cam_params_.rows; ++i)
    {
        std::cout << "cam_params 44444 " << cam_params_.at<double>(i,0) << std::endl;
        
    }
    std::cout << "temp = " << temp << std::endl;
    gettimeofday( &for_end, NULL );
    int fortimeuse = 1000000 * ( for_end.tv_sec - for_start.tv_sec ) + for_end.tv_usec - for_start.tv_usec;
    printf("for time: %d us\n", fortimeuse); //3125472us
    
    //    LOGLN_CHAT("");
    //    LOGLN_CHAT("Bundle adjustment, final RMS error: " << std::sqrt(err.dot(err) / total_num_matches_));
    //    LOGLN_CHAT("Bundle adjustment, iterations done: " << iter);
    struct timeval nan_start, nan_end;
    gettimeofday( &nan_start, NULL );
    // Check if all camera parameters are valid
    bool ok = true;
    for (int i = 0; i < cam_params_.rows; ++i)
    {
        std::cout << "cam_params_ " << cam_params_.at<double>(i,0) << std::endl;
        
    }
    for (int i = 0; i < cam_params_.rows; ++i)
    {
        if (cvIsNaN(cam_params_.at<double>(i,0)))
        {
            ok = false;
            break;
        }
    }
    if (!ok)
        return false;
    
    gettimeofday( &nan_end, NULL );
    int nantimeuse = 1000000 * ( nan_end.tv_sec - nan_start.tv_sec ) + nan_end.tv_usec - nan_start.tv_usec;
    printf("nan time: %d us\n", nantimeuse);  //1us
    
    struct timeval refine_start, refine_end;
    gettimeofday( &refine_start, NULL );
    
    obtainRefinedCameraParams(cameras);
    
    gettimeofday( &refine_end, NULL );
    int refinetimeuse = 1000000 * ( refine_end.tv_sec - refine_start.tv_sec ) + refine_end.tv_usec - refine_start.tv_usec;
    printf("refine time: %d us\n", inittimeuse);  //93us
    
    
    // Normalize motion to center image
    Graph span_tree;
    std::vector<int> span_tree_centers;
    
    struct timeval fmst_start, fmst_end;
    gettimeofday( &fmst_start, NULL );
    
    findMaxSpanningTree_idt(num_images_, pairwise_matches, span_tree, span_tree_centers);
    
    gettimeofday( &fmst_end, NULL );
    int fmsttimeuse = 1000000 * (fmst_end.tv_sec - fmst_start.tv_sec ) + fmst_end.tv_usec - fmst_start.tv_usec;
    //printf("fmst time: %d us\n", fmsttimeuse);  //102us
    
    Mat R_inv = cameras[span_tree_centers[0]].R.inv();
    // Mat R_inv = cameras[0].R.inv();
    for (int i = 0; i < num_images_; ++i)
        cameras[i].R = R_inv * cameras[i].R;
    
    gettimeofday( &end, NULL );
    int timeuse = 1000000 * (end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec;
    //printf("total22222 time: %d us\n", timeuse);
    // LOGLN_CHAT("Bundle adjustment, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");
    return true;
}

#endif
//////////////////////////////////////////////////////////////////////////////
//zt adjust

void backup_info();
void recover_info();
void delete_pic(int num);
void compute_mask_c1(Mat &mask, std::vector<Point2f> &corners);
bool overlap_mask(cv::Mat &src, cv::Mat &dst);

template<class T>
void toString(string &s, const T &t)
{
    stringstream ss;
    ss << t;
    s = ss.str();
    //或ss >> s;
}

bool capture_pic(cv::Mat &src, cv::Mat &dst, int mode)
{
    //删除最后一张图片返回倒数第二次的拼接图
    if (mode == 1 ){
        if (capture_images.size() > 0){
            delete_pic(capture_images.size());
            if (pano_vec.size() >0){
                pano_vec.back().copyTo(dst);
            }
            return true;
        }
        else
            return false;
    }
    
    if (src.empty())
        return false;
    
    //resize input size
    if (!is_input_scale_set) {
        if (work_pixel == 0){
            input_scale = min(1.0, sqrt(input_megapix * 1e6 / src.size().area()));
            is_input_scale_set = true;
        }
        else{
            if (min(src.rows, src.cols)<= work_pixel){
                input_scale = 1;
                is_input_scale_set = true;
            }
            else{
                input_scale = work_pixel*1./min(src.rows, src.cols);
                is_input_scale_set = true;
            }
        }
    }
    resize(src, input, Size(), input_scale, input_scale, INTER_NEAREST);
    
    int num_images = capture_images.size();
    
    Mat img;
    ImageFeatures img_features;
    
    //resize 特征计算尺寸以及设定接缝计算尺寸
    if (work_megapix < 0)
    {
        img = input.clone();
        work_scale = 1;
        is_work_scale_set = true;
    }
    else {
        if (!is_work_scale_set) {
            if (work_pixel == 0){
                work_scale = min(1.0, sqrt(work_megapix * 1e6 / input.size().area()));
                is_work_scale_set = true;
            }
            else{
                if (min(src.rows, src.cols)<= work_pixel){
                    work_scale = 1;
                    is_work_scale_set = true;
                }
                else{
                    work_scale = work_pixel*1./min(src.rows, src.cols);
                    is_work_scale_set = true;
                }
            }
        }
        resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
    }
    if (!is_seam_scale_set)
    {
        seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / input.size().area()));
        seam_work_aspect = seam_scale / work_scale;
        is_seam_scale_set = true;
    }
    
    //特征计算
    (*finder)(img, img_features);
    
    img_features.img_idx = num_images;
    
    //resize 接缝计算尺寸
    if (try_compose == true)
        resize(input, img, Size(), seam_scale, seam_scale, INTER_NEAREST);
    else{
        if (!is_compose_scale_set) {
            if (work_pixel == 0) {
                compose_scale = min(1.0, sqrt(compose_scale * 1e6 / input.size().area()));
                is_compose_scale_set = true;
            } else {
                if (min(src.rows, src.cols) <= work_pixel) {
                    compose_scale = 1;
                    is_compose_scale_set = true;
                } else {
                    compose_scale = work_pixel * 1. / min(src.rows, src.cols);
                    is_compose_scale_set = true;
                }
            }
        }
        resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
    }
    
    //check keypoints size
    if (img_features.keypoints.size()< 100){
        return false;
    }//backup
    else if (capture_images.size() == 0){
        capture_input_images.push_back(input.clone());
        capture_images.push_back(img.clone());
        capture_features.push_back(img_features);
        capture_img_sizes.push_back(input.size());
        
        resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
        pano_vec.push_back(img.clone());
        input.copyTo(dst);
        return true;
    }
    
    finder->collectGarbage();
    
    //构造两两匹配变量
    vector<MatchesInfo> two_matches;
    vector<ImageFeatures> two_features;
    num_images = capture_images.size();
    two_features.push_back(capture_features[num_images-1]);
    two_features.push_back(img_features);
    //特征匹配
    (*matcher)(two_features, two_matches, matchmask.getUMat(ACCESS_READ));
    matcher->collectGarbage();
    //check feature matching
    if (two_matches[1].num_inliers < inliner_num || two_matches[1].confidence < 0.9){
        return false;
    }
    
    
    //temp push feature
    capture_features.push_back(img_features);
    
    vector<CameraParams> two_cameras;
    if (!(*estimator)(two_features, two_matches, two_cameras))
    {
        capture_features.pop_back();
        return false;
    }
    
    for (size_t i = 0; i < two_cameras.size(); ++i) {
        Mat R;
        two_cameras[i].R.convertTo(R, CV_32F);
        two_cameras[i].R = R;
    }
    
    //BundleAdjustment
    if (!(*adjuster)(two_features, two_matches, two_cameras)) {
        capture_features.pop_back();
        return false;
    }
    
    //check pano size
    warped_image_scale = (two_cameras[0].focal+two_cameras[1].focal)/2;
    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
    Mat_<float> K;
    two_cameras[0].K().convertTo(K, CV_32F);
    float cwa = (float) compose_work_aspect;
    K(0, 0) *= cwa;
    K(0, 2) *= cwa;
    K(1, 1) *= cwa;
    K(1, 2) *= cwa;
    Rect roi = warper->warpRoi(capture_images[0].size(), K, two_cameras[1].R);
    //cv::Point corner1 = warper->warpRoi(capture_images[0].size(), K, two_cameras[1].R).tl();
    //int x, y;
    //x = roi.width - abs(corner1.x);
    //y = roi.height - abs(corner1.y);
    //if (x/capture_images[0].cols > 4 || y/capture_images[0].rows > 4){
    //    return false;
    //}
    if (roi.width > pano_max_edge || roi.height > pano_max_edge){
        capture_features.pop_back();
        return false;
    }
    
    //push img, feature, matchinfo
    capture_input_images.push_back(input.clone());
    capture_images.push_back(img.clone());
    //    capture_features.push_back(img_features);
    capture_img_sizes.push_back(input.size());
    capture_pairwise_matches.push_back(two_matches[1]);
    capture_pairwise_matches2.push_back(two_matches[2]);
    
    //push camera
    if (capture_features.size() == 2) {
        cameras.push_back(two_cameras[0]);
        cameras.push_back(two_cameras[1]);
    } else {
        two_cameras[1].R = cameras.back().R * two_cameras[1].R;
        two_cameras[0].R = cameras.back().R * two_cameras[0].R;
        cameras.push_back(two_cameras[1]);
    }
    
    //    warped_image_scale = (two_cameras[0].focal+two_cameras[1].focal)/2;
    if (warped_scales.size() < 2) {
        warped_scales.push_back(warped_image_scale);
        warped_scales.push_back(warped_image_scale);
    } else
        warped_scales.push_back(warped_image_scale);
    
    num_images = capture_images.size();
    
    //Directly Warp and Blend images
    if (try_compose == false){
        
        int id_offset = sum_corners.size();
        cv::UMat imgmask;
        // Preapre images masks
        for (int i = id_offset; i < num_images; ++i) {
            imgmask.create(capture_images[i].size(), CV_8U);
            imgmask.setTo(Scalar::all(255));
            sum_masks.push_back(imgmask);
        }
        
        // Compute relative scales
        double compose_work_aspect = compose_scale / work_scale;
        Ptr<RotationWarper> warper = warper_creator->create(
                                                            static_cast<float>(warped_image_scale * compose_work_aspect));
        
        //
        for (int i = id_offset; i < num_images; ++i) {
            // Update intrinsics
            Mat_<float> K;
            cameras[i].K().convertTo(K, CV_32F);
            float cwa = (float) compose_work_aspect;
            K(0, 0) *= cwa;
            K(0, 2) *= cwa;
            K(1, 1) *= cwa;
            K(1, 2) *= cwa;
            
            cv::UMat warped;
            cv::Point corner_pt;
            corner_pt = warper->warp(capture_images[i], K, cameras[i].R, INTER_LINEAR, BORDER_CONSTANT, warped);
            
            if (id_offset >= 2) {
                cv::Point corner0;
                Rect roi = warper->warpRoi(capture_images[i - 1].size(), K, two_cameras[0].R);
                corner0 = roi.tl();
                corner_pt.x = corner_pt.x + (sum_corners[i - 1].x - corner0.x);
                corner_pt.y = corner_pt.y + (sum_corners[i - 1].y - corner0.y);
            }
            
            sum_corners.push_back(corner_pt);
            sum_images_warped.push_back(warped);
            sum_sizes.push_back(warped.size());
            
            warper->warp(sum_masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, warped);
            sum_masks_warped.push_back(warped);
            
        }
        
        //Blend images
        blender->prepare(sum_corners, sum_sizes);
        cv::Mat img_warped_s;
        for (int img_idx = 0; img_idx < num_images; ++img_idx){
            // Blend the current image
            sum_images_warped[img_idx].convertTo(img_warped_s, CV_16S);
            blender->feed(img_warped_s, sum_masks_warped[img_idx], sum_corners[img_idx]);
        }
        
        Mat result, result_mask;
        blender->blend(result, result_mask);
        
        result.copyTo(dst);
        pano_vec.push_back(result);
#if ENABLE_LOG
        //        LOGLN("Compositing, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");
        //        LOGLN("Finished, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec");
#endif
        // Release unused memory
        img.release();
        
        return true;
    }
    
#if ENABLE_LOG
    t = getTickCount();
#endif
    
    int id_offset = sum_corners.size();
    
    // Preapre images masks
    cv::UMat imgmask;
    for (int i = id_offset; i < num_images; ++i) {
        imgmask.create(capture_images[i].size(), CV_8U);
        imgmask.setTo(Scalar::all(255));
        sum_masks.push_back(imgmask);
    }
    
    //    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_scales[id_offset] * seam_work_aspect));
    //warping 并保存mask，corner，size
    for (int i = id_offset; i < num_images; ++i) {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float) seam_work_aspect;
        K(0, 0) *= swa;
        K(0, 2) *= swa;
        K(1, 1) *= swa;
        K(1, 2) *= swa;
        
        cv::UMat warped;
        cv::Point corner_pt;
        
        corner_pt = warper->warp(capture_images[i], K, cameras[i].R, INTER_LINEAR, BORDER_CONSTANT, warped);
        
        if (id_offset >= 2) {
            cv::Point corner0 = warper->warpRoi(capture_images[i - 1].size(), K, two_cameras[0].R).tl();
            corner_pt.x = corner_pt.x + (sum_corners[i - 1].x - corner0.x);
            corner_pt.y = corner_pt.y + (sum_corners[i - 1].y - corner0.y);
        }
        
        sum_corners.push_back(corner_pt);
        
        sum_images_warped.push_back(warped);
        sum_sizes.push_back(warped.size());
        warper->warp(sum_masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, warped);
        sum_masks_warped.push_back(warped);
    }
    
    //    //compute all seams
    vector<UMat> images_warped_f(num_images);
    for (int i = 0; i < num_images; ++i)
        sum_images_warped[i].convertTo(images_warped_f[i], CV_32F);
    
    vector<UMat> seam_masks_warped;
    vector<Point> seam_corners;
    for (int i = 0; i < sum_masks_warped.size(); i++)
        seam_masks_warped.push_back(sum_masks_warped[i].clone());
    seam_finder->find(images_warped_f, sum_corners, seam_masks_warped);
    
    sum_seam_masks.clear();
    for(int i=0; i<sum_masks_warped.size(); ++i )
        sum_seam_masks.push_back(seam_masks_warped[i]);
    
    //compute two seams
    //    vector<UMat> images_warped_f;
    //    cv::UMat img_f;
    //    if (id_offset<2){
    //        for (int i = 0; i < 2; ++i){
    //            sum_images_warped[i].convertTo(img_f, CV_32F);
    //            images_warped_f.push_back(img_f);
    //        }
    //    }
    //    else{
    //        for (int i = id_offset-1; i < num_images; ++i){
    //            sum_images_warped[i].convertTo(img_f, CV_32F);
    //            images_warped_f.push_back(img_f);
    //        }
    //    }
    //    img_f.release();
    //
    //    vector<UMat> seam_masks_warped;
    //    vector<Point> seam_corners;
    //    if (id_offset<2){
    //        for(int i=0; i<num_images;i++){
    //            seam_corners.push_back(sum_corners[i]);
    //            seam_masks_warped.push_back(sum_masks_warped[i]);
    //        }
    //        seam_finder->find(images_warped_f, seam_corners, seam_masks_warped);
    //        sum_seam_masks.push_back(seam_masks_warped[0]);
    //        sum_seam_masks.push_back(seam_masks_warped[1]);
    //    }
    //    else{
    //        for(int i=id_offset-1; i<num_images;i++){
    //            seam_corners.push_back(sum_corners[i]);
    //            seam_masks_warped.push_back(sum_masks_warped[i]);
    //        }
    //        seam_finder->find(images_warped_f, seam_corners, seam_masks_warped);
    //        sum_seam_masks.push_back(seam_masks_warped[1]);
    //    }
    
    //    for (int i = 0; i < sum_seam_masks.size(); i++){
    //        cv::imshow("mask", sum_seam_masks[i]);
    //        cv::waitKey(0);
    //    }
    
    // Release unused memory
    //    global_images.clear();
    //    images_warped.clear();
    images_warped_f.clear();
    //    masks.clear();
    
    //合成变量
    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    //设定合成scale
    if (!is_compose_scale_set && compose_megapix > 0) {
        if (work_pixel == 0){
            compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / input.size().area()));
            is_compose_scale_set = true;
        }
        else{
            if (min(src.rows, src.cols)<= work_pixel){
                compose_scale = 1;
                is_compose_scale_set = true;
            }
            else{
                compose_scale = work_pixel*1./min(src.rows, src.cols);
                is_compose_scale_set = true;
            }
        }
    }
    
    // Compute relative scales
    //            compose_seam_aspect = compose_scale / seam_scale;
    compose_work_aspect = compose_scale / work_scale;
    
    // Update corners and sizes
    for (int i = id_offset; i < num_images; ++i) {
        // Update intrinsics
        warper->setScale(warped_scales[i] * static_cast<float>(compose_work_aspect));
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float cwa = (float) compose_work_aspect;
        K(0, 0) *= cwa;
        K(0, 2) *= cwa;
        K(1, 1) *= cwa;
        K(1, 2) *= cwa;
        
        // Update corner and size
        Size sz = capture_img_sizes[i];
        if (std::abs(compose_scale - 1) > 1e-1) {
            sz.width = cvRound(capture_img_sizes[i].width * compose_scale);
            sz.height = cvRound(capture_img_sizes[i].height * compose_scale);
        }
        
        Rect roi = warper->warpRoi(sz, K, cameras[i].R);
        if (id_offset >= 2) {
            cv::Point corner0 = warper->warpRoi(sz, K, two_cameras[0].R).tl();
            cv::Point corner_pt = roi.tl();
            //            corner_pt.x = corner_pt.x + (sum_corners[i - 1].x - corner0.x);
            //            corner_pt.y = corner_pt.y + (sum_corners[i - 1].y - corner0.y);
            //            sum_corners[i] = corner_pt;
            //            sum_sizes[i] = roi.size();
            corner_pt.x = corner_pt.x + (blend_corners[i - 1].x - corner0.x);
            corner_pt.y = corner_pt.y + (blend_corners[i - 1].y - corner0.y);
            blend_corners.push_back(corner_pt);
            blend_sizes.push_back(roi.size());
            
        } else {
            //            sum_corners[i] = roi.tl();
            //            sum_sizes[i] = roi.size();
            blend_corners.push_back(roi.tl());
            blend_sizes.push_back(roi.size());
        }
    }
    
    //warp blend images
    for (int img_idx = id_offset; img_idx < num_images; ++img_idx) {
        warper->setScale(warped_scales[img_idx] * static_cast<float>(compose_work_aspect));
        Mat_<float> K;
        cameras[img_idx].K().convertTo(K, CV_32F);
        float cwa = (float) compose_work_aspect;
        K(0, 0) *= cwa;
        K(0, 2) *= cwa;
        K(1, 1) *= cwa;
        K(1, 2) *= cwa;
        
        if (abs(compose_scale - 1) > 1e-1)
            resize(capture_input_images[img_idx], img, Size(), compose_scale, compose_scale, INTER_NEAREST);
        else
            img = capture_input_images[img_idx];
        
        Size img_size = img.size();
        
        //        Mat K;
        //        cameras[img_idx].K().convertTo(K, CV_32F);
        // Warp the current image
        warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);
        
        // Warp the current image mask
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);
        
        blend_imgs.push_back(img_warped);
        blend_masks.push_back(mask_warped);
        
        img_warped.release();
        img.release();
        mask.release();
    }
    
    //    vector<Point> blend_corners;
    //    vector<Size> blend_sizes;
    //    for(int img_idx = max(0, id_offset-1); img_idx < num_images; ++img_idx){
    //        blend_corners.push_back(sum_corners[img_idx]);
    //        blend_sizes.push_back(sum_sizes[img_idx]);
    //    }
    //set and blend images
    for (int img_idx = 0; img_idx < num_images; ++img_idx) {
        
        blend_imgs[img_idx].convertTo(img_warped_s, CV_16S);
        
        dilate(sum_seam_masks[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, blend_masks[img_idx].size(), 0, 0, INTER_NEAREST);
        mask_warped = seam_mask & blend_masks[img_idx];
        
        if (!blender && !timelapse) {
            blender = Blender::createDefault(blend_type, try_cuda);
            Size dst_sz = resultRoi(blend_corners, blend_sizes).size();
            //            Size dst_sz = resultRoi(sum_corners, sum_sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, try_cuda);
            else if (blend_type == Blender::MULTI_BAND) {
                MultiBandBlender *mb = dynamic_cast<MultiBandBlender *>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width) / log(2.)) - 1.));
            }
            blender->prepare(blend_corners, blend_sizes);
            //            blender->prepare(sum_corners, sum_sizes);
        }
        // Blend the current image
        //        blender->feed(img_warped_s, mask_warped, sum_corners[img_idx]);
        blender->feed(img_warped_s, mask_warped, blend_corners[img_idx]);
    }
    
    //    Mat result, result_mask;
    //    Mat blend_result, blend_result_mask;
    //    blender->blend(blend_result, blend_result_mask);
    //    blend_result.convertTo(blend_result, CV_8UC3);
    //    cv::imshow("blend_result", blend_result);
    //    cv::waitKey(0);
    //
    //    cv::Rect blend_rect = resultRoi(blend_corners, blend_sizes);
    //    //update wpared images
    //    int dx, dy;
    //    if (id_offset==0){
    //        dx = blend_corners[0].x - blend_rect.x;
    //        dy = blend_corners[0].y - blend_rect.y;
    //        cv::Rect newrect(abs(dx), abs(dy), blend_sizes[0].width, blend_sizes[0].height);
    //        cv::Mat tmp_img = blend_result(newrect);
    ////        blend_imgs.push_back(tmp_img);
    //        blend_imgs[0] = tmp_img;
    //    }
    //    else{
    //        dx = blend_corners[0].x - blend_rect.x;
    //        dy = blend_corners[0].y - blend_rect.y;
    //        cv::Rect newrect(abs(dx), abs(dy), blend_sizes[0].width, blend_sizes[0].height);
    //        cv::Mat tmp_img = blend_result(newrect);
    ////        blend_imgs.back() = tmp_img;
    //        blend_imgs[blend_imgs.size()-2] = tmp_img;
    //    }
    //
    //    dx = blend_corners[1].x - blend_rect.x;
    //    dy = blend_corners[1].y - blend_rect.y;
    //    cv::Rect newrect(abs(dx), abs(dy), blend_sizes[1].width, blend_sizes[1].height);
    //    cv::Mat tmp_img = blend_result(newrect);
    //    blend_imgs.back() = tmp_img;
    ////    blend_imgs.push_back(tmp_img);
    //
    //    blender = Blender::createDefault(Blender::NO, try_cuda);
    //    blender->prepare(sum_corners, sum_sizes);
    //    for (int img_idx = 0; img_idx < num_images; ++img_idx) {
    //        // Blend the current image
    //        blend_imgs[img_idx].convertTo(img_warped_s, CV_16S);
    //        blender->feed(img_warped_s, blend_masks[img_idx], sum_corners[img_idx]);
    //    }
    //    blender->blend(result, result_mask);
    //copy and save result
    Mat result, result_mask;
    blender->blend(result, result_mask);
    
    //    result.convertTo(result, CV_8UC3);
    result.copyTo(dst);
    pano_vec.push_back(result);
    //    cv::imshow("result", result);
    //    cv::waitKey(0);
    
    return true;
}

int wp_vec[] = {0,360, 540, 720};

#ifdef INITORIGIN
bool capture_stitching_init(int img_pixel, int warptype, int maxedge)
{
    //warp type
    warp_type = "plane";
    if(warptype==1)
        warp_type = "cylindrical";
    
    if (work_pixel_vec.empty())
        for (int i=0; i<4;i++)
            work_pixel_vec.push_back(wp_vec[i]);
    
    for(int i=1; i<work_pixel_vec.size(); ++i){
        if (img_pixel == i){
            work_pixel = work_pixel_vec[i];
            break;
        }
    }
    pano_max_edge = maxedge;
    
    //finder
    if (features_type == "surf")
    {
        finder = makePtr<SurfFeaturesFinder>(200);
    }
    else if (features_type == "sift")
    {
        finder = makePtr<SiftFeaturesFinder>(0, 3, 0.01, 50, 1.6);
    }
    //    else if (features_type == "orb-sift")
    //    {
    //        finder = makePtr<ORBSiftFeaturesFinder>(cv::Size(3,1), 1000, 1.3f, 1, 15, 0, 2, ORB::FAST_SCORE, 31, 10);
    //    }
    else if (features_type == "gftt-sift")
    {
        finder = makePtr<GFTTSiftFeaturesFinder>(0, 10e-4, 10e-3, 3, true, 0.04, 1, 10000, 0);
    }
    //    else if (features_type == "gftt-freak")
    //    {
    //        finder = makePtr<GFTTFREAKFeaturesFinder>(0, 10e-3, 10e-3, 3, true, 0.06, 10000, false, false, 22., 1 );
    //    }
    //    else if (features_type == "harrislaplace-sift")
    //    {
    //        finder = makePtr<HarrisLaplaceSiftFeaturesFinder>(1, 0.001, 0.001, 1000, 2, 2000, 0);
    //    }
    //    else if (features_type == "gftt-orb")
    //    {
    //        finder = makePtr<GFTTORBFeaturesFinder>(0, 10e-3, 10e-3, 3, true, 0.04, 10000,
    //                                                Size(1,1), 10000, 1.1f, 5, 10);
    ////                                                31, 0, 2, ORB::HARRIS_SCORE,
    ////                                                31, 20);
    //    }
    else if (features_type == "akaze")
    {
        finder = makePtr<AKAZEFeaturesFinder>();
    }
    else if (features_type == "orb")
    {
        finder = makePtr<OrbFeaturesFinder>(cv::Size(1,1), 10000, 1.3f, 1);
    }
    else
    {
        cout << "Unknown 2D features type: '" << features_type << "'.\n";
        return false;
    }
    
    //matcher
    if (matcher_type == "affine")
        matcher = makePtr<AffineBestOf2NearestMatcher>(true, try_cuda, match_conf);
    else if (range_width == -1)
        matcher = makePtr<BestOf2NearestMatcher>(try_cuda, match_conf);
    else
        matcher = makePtr<BestOf2NearestRangeMatcher>(range_width, try_cuda, match_conf);
    
    //estimator
    if (estimator_type == "affine")
        estimator = makePtr<AffineBasedEstimator>();
    else
        estimator = makePtr<HomographyBasedEstimator>();
    
    //affine matcher
    affine_matcher = makePtr<AffineBestOf2NearestMatcher>(true, try_cuda, match_conf);
    //affine estimator
    affine_estimator = makePtr<AffineBasedEstimator>();
    //affine_warper
    affine_warper_creator = makePtr<cv::AffineWarper>();
    
    //BA
    if (ba_cost_func == "reproj") adjuster = makePtr<detail::BundleAdjusterReproj>();
    else if (ba_cost_func == "ray") adjuster = makePtr<detail::BundleAdjusterRay>();
    else if (ba_cost_func == "affine") adjuster = makePtr<detail::BundleAdjusterAffinePartial>();
    else if (ba_cost_func == "no") adjuster = makePtr<NoBundleAdjuster>();
    else
    {
        cout << "Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n";
        return false;
    }
    adjuster->setTermCriteria(TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, ba_iter_num, DBL_EPSILON));
    adjuster->setConfThresh(conf_thresh);
    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
    if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
    if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
    if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
    if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
    adjuster->setRefinementMask(refine_mask);
    
    //Seam
    if (seam_find_type == "no")
        seam_finder = makePtr<detail::NoSeamFinder>();
    else if (seam_find_type == "voronoi")
        seam_finder = makePtr<detail::VoronoiSeamFinder>();
    else if (seam_find_type == "gc_color")
    {
#ifdef HAVE_OPENCV_CUDALEGACY
        if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
            seam_finder = makePtr<detail::GraphCutSeamFinderGpu>(GraphCutSeamFinderBase::COST_COLOR);
        else
#endif
            seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
    }
    else if (seam_find_type == "gc_colorgrad")
    {
#ifdef HAVE_OPENCV_CUDALEGACY
        if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
            seam_finder = makePtr<detail::GraphCutSeamFinderGpu>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
        else
#endif
            seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
    }
    else if (seam_find_type == "dp_color")
        seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR);
    else if (seam_find_type == "dp_colorgrad")
        seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
    if (!seam_finder)
    {
        cout << "Can't create the following seam finder '" << seam_find_type << "'\n";
        return false;
    }
    
    //warper
#ifdef HAVE_OPENCV_CUDAWARPING
    if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
    {
        if (warp_type == "plane")
            warper_creator = makePtr<cv::PlaneWarperGpu>();
        else if (warp_type == "cylindrical")
            warper_creator = makePtr<cv::CylindricalWarperGpu>();
        else if (warp_type == "spherical")
            warper_creator = makePtr<cv::SphericalWarperGpu>();
    }
    else
#endif
    {
        if (warp_type == "plane")
            warper_creator = makePtr<cv::PlaneWarper>();
        else if (warp_type == "affine")
            warper_creator = makePtr<cv::AffineWarper>();
        else if (warp_type == "cylindrical")
            warper_creator = makePtr<cv::CylindricalWarper>();
    }
    
    if (!warper_creator)
    {
        cout << "Can't create the following warper '" << warp_type << "'\n";
        return false;
    }
    
    //blender
    //    if (try_compose == true){
    //        blend_type = Blender::MULTI_BAND;
    //    }
    blender = Blender::createDefault(blend_type, try_cuda);
    
    //warm up
    cv::Mat zeromap = cv::Mat::zeros(100, 100, CV_8UC3);
    ImageFeatures feature;
    (*finder)(zeromap, feature);
    finder->collectGarbage();
    
    //init vector
    capture_input_images.clear();
    capture_images.clear();
    capture_features.clear();
    capture_img_sizes.clear();
    capture_pairwise_matches.clear();
    capture_pairwise_matches2.clear();
    pano_vec.clear();
    
    //scale
    work_scale = 1;
    seam_scale = 1;
    compose_scale = 1;
    is_work_scale_set = false;
    is_seam_scale_set = false;
    is_compose_scale_set = false;
    seam_work_aspect = 1;
    compose_seam_aspect = 1;
    compose_work_aspect = 1;
    
    input_scale = 1;
    is_input_scale_set = false;
    
    //compose
    sum_corners.clear();
    sum_masks_warped.clear();
    sum_images_warped.clear();
    sum_sizes.clear();
    sum_masks.clear();
    
    cameras.clear();
    
    warped_scales.clear();
    sum_seam_masks.clear();
    blend_imgs.clear();
    blend_masks.clear();
    
    //matchmask
    matchmask = Mat::zeros(2, 2, CV_8UC1);
    matchmask.at<uchar>(0, 1) = 1;
    
    //overlap
    is_corners_set = false;
    Ptr<ORB> orb_ = ORB::create(10000, 1.5f, 1, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
    Ptr< FastFeatureDetector> fast_ = FastFeatureDetector::create (20, true, FastFeatureDetector::TYPE_9_16);
    detector = fast_;
    extractor = orb_;
    fastdetector = fast_;
    flann_search = makePtr<flann::SearchParams>(32,0, false);
    
    return true;
}
#endif

bool capture_stitching_init(int img_pixel, int warptype, int MaxMatchNums, int isWarpInit , double composeScale, int MaxPix, int maxedge)
{
    //warp type
    warp_type = "plane";
    if(warptype==1)
        warp_type = "cylindrical";
    
    if (work_pixel_vec.empty())
        for (int i=0; i<4;i++)
            work_pixel_vec.push_back(wp_vec[i]);
    
    for(int i=1; i<work_pixel_vec.size(); ++i){
        if (img_pixel == i){
            work_pixel = work_pixel_vec[i];
            break;
        }
    }
    pano_max_edge = maxedge;
    MaxPix_ = MaxPix;
    compose_megapix = composeScale;
    MaxImgNums_ = MaxMatchNums;
    
    //finder
    if (features_type == "surf")
    {
        finder = makePtr<SurfFeaturesFinder>(200);
    }
    else if (features_type == "sift")
    {
        finder = makePtr<SiftFeaturesFinder>(0, 3, 0.01, 50, 1.6);
    }
    //    else if (features_type == "orb-sift")
    //    {
    //        finder = makePtr<ORBSiftFeaturesFinder>(cv::Size(3,1), 1000, 1.3f, 1, 15, 0, 2, ORB::FAST_SCORE, 31, 10);
    //    }
    else if (features_type == "gftt-sift")
    {
        finder = makePtr<GFTTSiftFeaturesFinder>(0, 10e-4, 10e-3, 3, true, 0.04, 1, 10000, 0);
    }
    //    else if (features_type == "gftt-freak")
    //    {
    //        finder = makePtr<GFTTFREAKFeaturesFinder>(0, 10e-3, 10e-3, 3, true, 0.06, 10000, false, false, 22., 1 );
    //    }
    //    else if (features_type == "harrislaplace-sift")
    //    {
    //        finder = makePtr<HarrisLaplaceSiftFeaturesFinder>(1, 0.001, 0.001, 1000, 2, 2000, 0);
    //    }
    //    else if (features_type == "gftt-orb")
    //    {
    //        finder = makePtr<GFTTORBFeaturesFinder>(0, 10e-3, 10e-3, 3, true, 0.04, 10000,
    //                                                Size(1,1), 10000, 1.1f, 5, 10);
    ////                                                31, 0, 2, ORB::HARRIS_SCORE,
    ////                                                31, 20);
    //    }
    else if (features_type == "akaze")
    {
        finder = makePtr<AKAZEFeaturesFinder>();
    }
    else if (features_type == "orb")
    {
        finder = makePtr<OrbFeaturesFinder>(cv::Size(1,1), 10000, 1.3f, 1);
    }
    else
    {
        cout << "Unknown 2D features type: '" << features_type << "'.\n";
        return false;
    }
    
    //matcher
    if (matcher_type == "affine")
        matcher = makePtr<AffineBestOf2NearestMatcher>(true, try_cuda, match_conf);
    else if (range_width == -1)
        matcher = makePtr<BestOf2NearestMatcher>(try_cuda, match_conf);
    else
        matcher = makePtr<BestOf2NearestRangeMatcher>(range_width, try_cuda, match_conf);
    
    //estimator
    if (estimator_type == "affine")
        estimator = makePtr<AffineBasedEstimator>();
    else
        estimator = makePtr<HomographyBasedEstimator>();
    
    //affine matcher
    affine_matcher = makePtr<AffineBestOf2NearestMatcher>(true, try_cuda, match_conf);
    //affine estimator
    affine_estimator = makePtr<AffineBasedEstimator>();
    //affine_warper
    affine_warper_creator = makePtr<cv::AffineWarper>();
    
    //BA
    if (ba_cost_func == "reproj") adjuster = makePtr<detail::BundleAdjusterReproj>();
    else if (ba_cost_func == "ray") adjuster = makePtr<detail::BundleAdjusterRay>();
    else if (ba_cost_func == "affine") adjuster = makePtr<detail::BundleAdjusterAffinePartial>();
    else if (ba_cost_func == "no") adjuster = makePtr<NoBundleAdjuster>();
    else
    {
        cout << "Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n";
        return false;
    }
    adjuster->setTermCriteria(TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, ba_iter_num, DBL_EPSILON));
    adjuster->setConfThresh(conf_thresh);
    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
    if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
    if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
    if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
    if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
    adjuster->setRefinementMask(refine_mask);
    
    //Seam
    if (seam_find_type == "no")
        seam_finder = makePtr<detail::NoSeamFinder>();
    else if (seam_find_type == "voronoi")
        seam_finder = makePtr<detail::VoronoiSeamFinder>();
    else if (seam_find_type == "gc_color")
    {
#ifdef HAVE_OPENCV_CUDALEGACY
        if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
            seam_finder = makePtr<detail::GraphCutSeamFinderGpu>(GraphCutSeamFinderBase::COST_COLOR);
        else
#endif
            seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
    }
    else if (seam_find_type == "gc_colorgrad")
    {
#ifdef HAVE_OPENCV_CUDALEGACY
        if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
            seam_finder = makePtr<detail::GraphCutSeamFinderGpu>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
        else
#endif
            seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
    }
    else if (seam_find_type == "dp_color")
        seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR);
    else if (seam_find_type == "dp_colorgrad")
        seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
    if (!seam_finder)
    {
        cout << "Can't create the following seam finder '" << seam_find_type << "'\n";
        return false;
    }
    
    //warper
#ifdef HAVE_OPENCV_CUDAWARPING
    if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
    {
        if (warp_type == "plane")
            warper_creator = makePtr<cv::PlaneWarperGpu>();
        else if (warp_type == "cylindrical")
            warper_creator = makePtr<cv::CylindricalWarperGpu>();
        else if (warp_type == "spherical")
            warper_creator = makePtr<cv::SphericalWarperGpu>();
    }
    else
#endif
    {
        if (warp_type == "plane")
            warper_creator = makePtr<cv::PlaneWarper>();
        else if (warp_type == "affine")
            warper_creator = makePtr<cv::AffineWarper>();
        else if (warp_type == "cylindrical")
            warper_creator = makePtr<cv::CylindricalWarper>();
    }
    
    if (!warper_creator)
    {
        cout << "Can't create the following warper '" << warp_type << "'\n";
        return false;
    }
    
    //blender
    //    if (try_compose == true){
    //        blend_type = Blender::MULTI_BAND;
    //    }
    blender = Blender::createDefault(blend_type, try_cuda);
    
    if(isWarpInit)
    {
        //warm up
        cv::Mat zeromap = cv::Mat::zeros(100, 100, CV_8UC3);
        ImageFeatures feature;
        (*finder)(zeromap, feature);
        finder->collectGarbage();
    }
    //init vector
    capture_input_images.clear();
    capture_images.clear();
    capture_features.clear();
    capture_img_sizes.clear();
    capture_pairwise_matches.clear();
    capture_pairwise_matches2.clear();
    pano_vec.clear();
    
    //scale
    work_scale = 1;
    seam_scale = 1;
    compose_scale = 1;
    is_work_scale_set = false;
    is_seam_scale_set = false;
    is_compose_scale_set = false;
    seam_work_aspect = 1;
    compose_seam_aspect = 1;
    compose_work_aspect = 1;
    
    input_scale = 1;
    is_input_scale_set = false;
    
    //compose
    sum_corners.clear();
    sum_masks_warped.clear();
    sum_images_warped.clear();
    sum_sizes.clear();
    sum_masks.clear();
    
    cameras.clear();
    
    warped_scales.clear();
    sum_seam_masks.clear();
    blend_imgs.clear();
    blend_masks.clear();
    
    //matchmask
    matchmask = Mat::zeros(2, 2, CV_8UC1);
    matchmask.at<uchar>(0, 1) = 1;
    
    //overlap
    is_corners_set = false;
    Ptr<ORB> orb_ = ORB::create(10000, 1.5f, 1, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
    Ptr< FastFeatureDetector> fast_ = FastFeatureDetector::create (20, true, FastFeatureDetector::TYPE_9_16);
    detector = fast_;
    extractor = orb_;
    fastdetector = fast_;
    flann_search = makePtr<flann::SearchParams>(32,0, false);
    
    return true;
}

void capture_stitching_release()
{
    //zt
    global_images.clear();
    global_input_images.clear();
    global_img_feature.clear();
    for(int i = 0; i < global_pairwise_matches.size(); ++i)
    {
        global_pairwise_matches[i].matches.clear();
        global_pairwise_matches[i].H.release();
        global_pairwise_matches[i].inliers_mask.clear();
    }
    global_pairwise_matches.clear(); // 特征匹配结果
    global_camera_params.clear();
    global_corners.clear();
    InputImgNums = 0;
    
    all_images.clear();
    all_input_images.clear();
    all_img_feature.clear();
    all_pairwise_matches.clear();
    
    finder.release();
    matcher.release();
    estimator.release();
    adjuster.release();
    seam_finder.release();
    warper_creator.release();
    blender.release();
    
    affine_matcher.release();
    affine_estimator.release();
    affine_warper_creator.release();
    
    capture_input_images.clear();
    capture_images.clear();
    capture_features.clear();
    capture_img_sizes.clear();
    capture_pairwise_matches.clear();
    capture_pairwise_matches2.clear();
    pano_vec.clear();
    
    //compose
    sum_corners.clear();
    sum_masks_warped.clear();
    sum_images_warped.clear();
    sum_sizes.clear();
    sum_masks.clear();
    
    cameras.clear();
    
    warped_scales.clear();
    sum_seam_masks.clear();
    blend_imgs.clear();
    blend_masks.clear();
    blend_corners.clear();
    blend_sizes.clear();
    
    //recover
    work_pixel = 0;
    
    //overlap
    detector.release();
    extractor.release();
    fastdetector.release();
    
    //最大边长
    pano_max_edge = 2000;
}

//void backup_info() {
//
//    capture_input_images_bak = capture_input_images.back();
//    capture_input_images.pop_back();
//
//    capture_images_bak = capture_images.back();
//    capture_images.pop_back();
//
//    capture_img_sizes_bak = capture_img_sizes.back();
//    capture_img_sizes.pop_back();
//
//    capture_features_bak = capture_features.back();
//    capture_features.pop_back();
//
//    pano_bak = pano_vec.back();
//    pano_vec.pop_back();
//
//    if (capture_pairwise_matches.size() > 0) {
//        capture_pairwise_matches_bak = capture_pairwise_matches.back();
//        capture_pairwise_matches.pop_back();
//
//        capture_pairwise_matches2_bak = capture_pairwise_matches2.back();
//        capture_pairwise_matches2.pop_back();
//
//        get_pairwise_bak = true;
//    } else
//        get_pairwise_bak = false;
//
//}
//
//void recover_info() {
//    capture_input_images.push_back(capture_input_images_bak);
//    capture_images.push_back(capture_images_bak);
//    capture_features.push_back(capture_features_bak);
//    capture_img_sizes.push_back(capture_img_sizes_bak);
//    pano_vec.push_back(pano_bak);
//    if (get_pairwise_bak) {
//        capture_pairwise_matches.push_back(capture_pairwise_matches_bak);
//        capture_pairwise_matches2.push_back(capture_pairwise_matches2_bak);
//    }
//
//}

bool overlap_mask(cv::Mat &src, cv::Mat &mask) {
    
    if (src.empty())
        return false;
    
#if ENABLE_LOG
    int64 t = getTickCount();
#endif
    
    if (!is_input_scale_set) {
        input_scale = min(1.0, sqrt(input_megapix * 1e6 / src.size().area()));
        is_input_scale_set = true;
    }
    resize(src, input, Size(), input_scale, input_scale, INTER_NEAREST);
    
    int num_images = capture_images.size();
    
    Mat img;
    ImageFeatures img_features;
    
    if (work_megapix < 0) {
        img = input.clone();
        work_scale = 1;
        is_work_scale_set = true;
    } else {
        if (!is_work_scale_set) {
            work_scale = min(1.0, sqrt(work_megapix * 1e6 / input.size().area()));
            is_work_scale_set = true;
        }
        resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
    }
    
    (*finder)(img, img_features);
    
    //    timestring += "Finding features, time: ";
    //    toString(logstring, ((getTickCount() - t) / getTickFrequency()));
    //    timestring += logstring;
    //    timestring += " sec";
    //    timestring += "\n";
    
    //check keypoints size
    if (img_features.keypoints.size() < 100) {
        return false;
    }
    
    finder->collectGarbage();
    
    vector<MatchesInfo> two_matches;
    vector<ImageFeatures> two_features;
    two_features.push_back(capture_features[num_images - 1]);
    two_features.push_back(img_features);
    
    //    t = getTickCount();
    (*affine_matcher)(two_features, two_matches, matchmask.getUMat(ACCESS_READ));
    affine_matcher->collectGarbage();
    //    timestring += "Matching, time: ";
    //    toString(logstring, ((getTickCount() - t) / getTickFrequency()));
    //    timestring += logstring;
    //    timestring += " sec";
    //    timestring += "\n";
    
    if (two_matches[1].num_inliers < inliner_num /*|| two_matches[1].confidence < 0.9*/) {
#if ENABLE_LOG
        LOGLN("inliner_num or confidence not enough");
        LOGLN("num_inliers: " << two_matches[1].num_inliers);
#endif
        return false;
    }
    
    //    vector<CameraParams> cameras;
    //    if (!(*affine_estimator)(two_features, two_matches, cameras)) {
    //#if ENABLE_LOG
    //        cout << "affine estimation failed.\n";
    //#endif
    //        return false;
    //    }
    //
    //    for (size_t i = 0; i < cameras.size(); ++i) {
    //        Mat R;
    //        cameras[i].R.convertTo(R, CV_32F);
    //        cameras[i].R = R;
    //    }
    //
    //    //get overlap_mask
    //    if (!is_corners_set) {
    //        overlapmask.create(img.size(), CV_8UC1);
    //        overlapmask.setTo(Scalar::all(255));
    //    }
    //    float warped_image_scale = cameras[0].focal;
    //    compose_work_aspect = compose_scale / work_scale;
    //    Ptr<RotationWarper> warper = affine_warper_creator->create(static_cast<float>(warped_image_scale * compose_work_aspect));
    //    // Update intrinsics
    //    Mat_<float> K;
    //    cameras[1].K().convertTo(K, CV_32F);
    //    float cwa = (float) compose_work_aspect;
    //    K(0, 0) *= cwa;
    //    K(0, 2) *= cwa;
    //    K(1, 1) *= cwa;
    //    K(1, 2) *= cwa;
    //
    //    Point tl = warper->warp(overlapmask, K, cameras[1].R, INTER_NEAREST, BORDER_CONSTANT, mask);
    //    cv::imshow("s1", mask);
    //    cv::waitKey(0);
    //    cv::resize(mask , mask, src.size(), 1, 1, INTER_NEAREST);
    
    //    t = getTickCount();
    if (!is_corners_set) {
        int m_width = int(input.cols * work_scale);
        int m_height = int(input.rows * work_scale);
        overlapmask = cv::Mat::zeros(m_height, m_width, CV_8UC1);
        obj_corners[0] = Point2f(0, 0);
        obj_corners[1] = Point2f(m_width, 0);
        obj_corners[2] = Point2f(m_width, m_height);
        obj_corners[3] = Point2f(0, m_height);
    }
    
    perspectiveTransform(obj_corners, scene_corners, two_matches[1].H);
    compute_mask_c1(overlapmask, scene_corners);
    cv::resize(overlapmask, mask, src.size(), 1, 1, INTER_NEAREST);
    
    //    timestring += "Drawing, time: ";
    //    toString(logstring, ((getTickCount() - t) / getTickFrequency()));
    //    timestring += logstring;
    //    timestring += " sec";
    //    timestring += "\n";
    
    return true;
}

bool pInQuadrangle(Point2f &mLTPoint, Point2f &mRTPoint, Point2f &mRBPoint, Point2f &mLBPoint, Point &p) {
    
    int a = (mLTPoint.x - mLBPoint.x) * (p.y - mLBPoint.y) - (mLTPoint.y - mLBPoint.y) * (p.x - mLBPoint.x);
    int b = (mRTPoint.x - mLTPoint.x) * (p.y - mLTPoint.y) - (mRTPoint.y - mLTPoint.y) * (p.x - mLTPoint.x);
    int c = (mRBPoint.x - mRTPoint.x) * (p.y - mRTPoint.y) - (mRBPoint.y - mRTPoint.y) * (p.x - mRTPoint.x);
    int d = (mLBPoint.x - mRBPoint.x) * (p.y - mRBPoint.y) - (mLBPoint.y - mRBPoint.y) * (p.x - mRBPoint.x);
    
    if ((a > 0 && b > 0 && c > 0 && d > 0) || (a < 0 && b < 0 && c < 0 && d < 0)) {
        return true;
    }
    
    return false;
}

void compute_mask_c1(Mat &mask, std::vector<Point2f> &corners) {
    cv::Point temp_pt;
    int nr = mask.rows; // 将3通道转换为1通道
    int nl = mask.cols;
    uchar *inData;
    for (int k = 0; k < nr; k++) { // 每一行图像的指针
        inData = mask.ptr<uchar>(k);
        for (int g = 0; g < nl; g++) {
            temp_pt.x = g;
            temp_pt.y = k;
            if (pInQuadrangle(corners[0], corners[1], corners[2], corners[3], temp_pt)) {
                *inData = 255;
            }
            inData += 1;
        }
    }
    
}

#ifdef ORI
void delete_pic(int num){
    capture_input_images.pop_back();
    capture_images.pop_back();
    capture_img_sizes.pop_back();
    capture_features.pop_back();
    pano_bak = pano_vec.back();
    pano_vec.pop_back();
    
    if (capture_pairwise_matches.size() > 0) {
        capture_pairwise_matches.pop_back();
        capture_pairwise_matches2.pop_back();
        
        get_pairwise_bak = true;
    } else
        get_pairwise_bak = false;
    
    if (num > 2) {
        sum_corners.pop_back();
        sum_masks_warped.pop_back();
        sum_images_warped.pop_back();
        sum_sizes.pop_back();
        sum_masks.pop_back();
        
        cameras.pop_back();
        
        if (try_compose == true){
            warped_scales.pop_back();
            sum_seam_masks.pop_back();
            blend_imgs.pop_back();
            blend_masks.pop_back();
            blend_corners.pop_back();
            blend_sizes.pop_back();
        }
        
    } else {
        sum_corners.clear();
        sum_masks_warped.clear();
        sum_images_warped.clear();
        sum_sizes.clear();
        sum_masks.clear();
        
        cameras.clear();
        
        if (try_compose == true){
            warped_scales.clear();
            sum_seam_masks.clear();
            blend_imgs.clear();
            blend_masks.clear();
            blend_corners.clear();
            blend_sizes.clear();
        }
    }
    
}
#endif

void delete_pic(int num){
    
    all_images.pop_back();
    all_input_images.pop_back();
    all_img_feature.pop_back();
    pano_vec.pop_back();
    InputImgNums --;
    //////20190416 add
    global_input_images.clear();
    global_images.clear();
    global_img_feature.clear();
    global_pairwise_matches.clear();
    
    if(num < 2)
    {
        all_pairwise_matches.clear();
    }
    
    if(all_images.size() > MaxImgNums_)
    {
        int tmp = all_images.size() - MaxImgNums_ + 1;
        for(int i = 0; i < MaxImgNums_ - 1; ++i)
        {
            global_input_images.push_back(all_input_images[tmp + i]);
            global_images.push_back(all_images[tmp + i]);
            global_img_feature.push_back(all_img_feature[tmp + i]);
        }
    }
    else
    {
        global_input_images = all_input_images;
        global_images = all_images;
        global_img_feature = all_img_feature;
    }
    
    struct timeval match_start, match_end;
    gettimeofday( &match_start, NULL );
    if(global_img_feature.size() > 1)
    {
        (*matcher)(global_img_feature, global_pairwise_matches/*, cv::UMat()*/);
        // gettimeofday( &match_end, NULL );
        matcher->collectGarbage();
    }
}

void HarrisResponses(const Mat& img, std::vector<KeyPoint>& pts, int blockSize, float harris_k);

void overlap_test(cv::Mat &src, std::string &timestring){
    if (src.empty())
        return;
    
    is_input_scale_set = false;
    
    //parameter
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> extractor;
    Ptr<cv::DescriptorMatcher> f_matcher, b_matcher;
    std::vector< std::vector<DMatch> > pair_matches;
    std::vector<DMatch> tmpmatches, goodmatches;
    std::vector<Point2f> obj_pt, scene_pt;
    std::vector<uchar> inliers_mask;
    int maxkeypoint = 5000;
    
    Ptr<ORB> orbextractor;
    Ptr<FastFeatureDetector> fastdetector;
    Ptr<xfeatures2d::BriefDescriptorExtractor> briefextractor;
    Ptr<cv::flann::SearchParams> flann_search;
    
    if (!is_input_scale_set) {
        if (work_pixel == 0){
            input_scale = min(1.0, sqrt(input_megapix * 1e6 / src.size().area()));
            is_input_scale_set = true;
        }
        else{
            if (min(src.rows, src.cols)<= work_pixel){
                input_scale = 1;
                is_input_scale_set = true;
            }
            else{
                input_scale = work_pixel*1./min(src.rows, src.cols);
                is_input_scale_set = true;
            }
        }
    }
    resize(src, input, Size(), input_scale, input_scale, INTER_NEAREST);
    
    if (features_type == "gftt-sift"){
        Ptr<GFTTDetector> gftt_ = GFTTDetector::create(0, 10e-4, 10e-3, 3, true, 0.04);
        Ptr<cv::xfeatures2d::SIFT> sift_ = cv::xfeatures2d::SIFT::create(0, 1, 0.04, 10, 1.6);
        detector = gftt_;
        extractor = sift_;
    }
    else if (features_type == "orb"){
        Ptr<ORB> orb_ = ORB::create(10000, 1.5f, 1, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
        detector = orb_;
        extractor = orb_;
        Ptr< FastFeatureDetector> fast_ = FastFeatureDetector::create (20, true);
        Ptr<cv::xfeatures2d::BriefDescriptorExtractor> brief_ =
        cv::xfeatures2d::BriefDescriptorExtractor::create( 32, false );
        //        detector = fast_;
        //        extractor = brief_;
        fastdetector = fast_;
        briefextractor = brief_;
        
        //        orbextractor = makePtr<ORBextractor>(10000, 1.5f, 1, 20, 20);
        orbextractor = orb_;
    }
    
    b_matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
    f_matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    flann_search = makePtr<flann::SearchParams>(32,0, false);
    
    Ptr<cv::flann::Index> tree;
    
    for(;maxkeypoint<=5000;){
        for(int kk=0; kk<1; kk++){
            double t = cv::getTickCount();
            
            pair_matches.clear();
            tmpmatches.clear();
            goodmatches.clear();
            obj_pt.clear();
            scene_pt.clear();
            inliers_mask.clear();
            
#if ENABLE_LOG
            timestring += "input size: ";
            toString(logstring, (input.size()));
            timestring += logstring;
            timestring += "\n";
#endif
            
            ImageFeatures features;
            cv::Mat gray;
            if(input.channels() == 3)
                cv::cvtColor(input, gray, CV_BGR2GRAY);
            else
                input.copyTo(gray);
            //        cv::GaussianBlur(gray, gray, cv::Size(3,3), 0.5, 0.5);
            //        cv::medianBlur(gray, gray, 3);
            fastdetector->detect(gray, features.keypoints);
            
#if ENABLE_LOG
            timestring += "detect time: ";
            toString(logstring, ((getTickCount() - t) / getTickFrequency()));
            timestring += logstring;
            timestring += " sec";
            timestring += "\n";
            
            timestring += "keypoints.size(): ";
            toString(logstring, ((int)features.keypoints.size()));
            timestring += logstring;
            timestring += "\n";
#endif
            
            //            if (features_type != "orb")
            //                HarrisResponses(gray, features.keypoints, 3, 0.04);
            KeyPointsFilter::runByImageBorder(features.keypoints, gray.size(), 31);
            KeyPointsFilter::retainBest(features.keypoints, maxkeypoint);
            
            t = cv::getTickCount();
            //            extractor->detectAndCompute(gray, cv::Mat(), features.keypoints, features.descriptors, true);
            //            orbextractor->compute(gray, features.keypoints, features.descriptors);
            //            cv::Mat descriptors;
            //            orbextractor->computeORBDescriptors(gray, features.keypoints, descriptors);
            //            features.descriptors = descriptors.getUMat(ACCESS_RW);
            extractor->compute(gray, features.keypoints, features.descriptors);
            //            briefextractor->compute(gray, features.keypoints, features.descriptors);
            features.descriptors = features.descriptors.reshape(1, (int)features.keypoints.size());
            
            //        detector->detectAndCompute(gray, cv::Mat(), features.keypoints, features.descriptors, false);
            //        features.descriptors = features.descriptors.reshape(1, (int)features.keypoints.size());
#if ENABLE_LOG
            timestring += "descriptor time: ";
            toString(logstring, ((getTickCount() - t) / getTickFrequency()));
            timestring += logstring;
            timestring += " sec";
            timestring += "\n";
            
            timestring += "descriptors.size(): ";
            toString(logstring, (features.descriptors.rows));
            timestring += logstring;
            timestring += "\n";
#endif
            
            
            
            t = cv::getTickCount();
            //        cv::flann::Index tree(features.descriptors, cv::flann::LshIndexParams(16, 16, 0), cvflann::FLANN_DIST_HAMMING);
            tree = makePtr<cv::flann::Index>(features.descriptors, cv::flann::LshIndexParams(8, 18, 0), cvflann::FLANN_DIST_HAMMING);
#if ENABLE_LOG
            timestring += "Index_time, time: ";
            toString(logstring, ((getTickCount() - t) / getTickFrequency()));
            timestring += logstring;
            timestring += " sec";
            timestring += "\n";
#endif
            //        t = getTickCount();
            
            //        if (features.descriptors.depth() == CV_8U)
            //            b_matcher->knnMatch(features.descriptors, features.descriptors, pair_matches, 2);
            //        else
            //            f_matcher->knnMatch(features.descriptors, features.descriptors, pair_matches, 2);
            //
            //        for (int i=0; i<pair_matches.size(); ++i){
            //            if (pair_matches[i].size() < 2)
            //                continue;
            //            if (pair_matches[i][0].distance < (1.f - match_conf) * pair_matches[i][1].distance)
            //            {
            //                tmpmatches.push_back(pair_matches[i][0]);
            //                obj_pt.push_back( features.keypoints[ pair_matches[i][0].queryIdx ].pt );
            //                scene_pt.push_back( features.keypoints[ pair_matches[i][0].trainIdx ].pt );
            //            }
            //        }
            
            t = getTickCount();
            cv::Mat indices, dists;
            tree->knnSearch(features.descriptors, indices, dists, 2, *flann_search);
            
            float *dists_ptr;
            int *indeces_ptr;
            //        float matchthresh = 1.f - match_conf;
            for (int i = 0; i < dists.rows; i++) {
                dists_ptr = dists.ptr<float>(i);
                indeces_ptr = indices.ptr<int>(i);
                if (dists_ptr[0] < (1.f - match_conf) * dists_ptr[1]) {
                    DMatch dmatches(indeces_ptr[0], i, dists_ptr[0]);
                    tmpmatches.push_back(dmatches);
                    obj_pt.push_back(features.keypoints[indeces_ptr[0]].pt);
                    scene_pt.push_back(features.keypoints[i].pt);
                }
                //            dists_ptr+=2;
                //            indeces_ptr+=2;
            }
            
            //            match_conf = 0.5;
            //            float* dists_ptr;
            //            int* indeces_ptr;
            //            float m=0;
            //            dists_ptr=dists.ptr<float>(0);
            //            for(int i=0;i<dists.rows;i++,dists_ptr++){
            //                if(dists_ptr[0]>0){
            //                    m += dists_ptr[0];
            //                }
            //            }
            //            m /= dists.rows;
            //
            //            float matchthresh = match_conf*m;
            //            for(int i=0;i<dists.rows;i++)
            //            {
            //                dists_ptr=dists.ptr<float>(i);
            //                indeces_ptr = indices.ptr<int>(i);
            //                if (dists_ptr[0]<=matchthresh)
            //                {
            //                    DMatch dmatches(indeces_ptr[0], i, dists_ptr[0]);
            //                    tmpmatches.push_back(dmatches);
            //                    obj_pt.push_back( features.keypoints[indeces_ptr[0]].pt );
            //                    scene_pt.push_back( features.keypoints[i].pt );
            //                }
            //            }
            
            cv::Mat H = estimateAffine2D(obj_pt, scene_pt, inliers_mask);
            H.push_back(Mat::zeros(1, 3, CV_64F));
            H.at<double>(2, 2) = 1;
            
            for (int i=0; i<inliers_mask.size();++i){
                if (inliers_mask[i] != '\0')
                    goodmatches.push_back(tmpmatches[i]);
            }
            
#if ENABLE_LOG
            timestring += "match_time, time: ";
            toString(logstring, ((getTickCount() - t) / getTickFrequency()));
            timestring += logstring;
            timestring += " sec";
            timestring += "\n";
#endif
        }
        
        maxkeypoint +=1000;
        
        //        pair_matches.clear();
        //        tmpmatches.clear();
        //        goodmatches.clear();
        //        obj_pt.clear();
        //        scene_pt.clear();
        //        inliers_mask.clear();
        //
        //        gray.release();
        //        indices.release();
        //        dists.release();
        //        H.release();
    }
    //    input.release();
    //    tree.release();
}

void HarrisResponses(const Mat& img, std::vector<KeyPoint>& pts, int blockSize, float harris_k)
{
    CV_Assert( img.type() == CV_8UC1 && blockSize*blockSize <= 2048 );
    
    size_t ptidx, ptsize = pts.size();
    
    const uchar* ptr00 = img.ptr<uchar>();
    int step = (int)(img.step/img.elemSize1());
    int r = blockSize/2;
    
    float scale = (1 << 2) * blockSize * 255.0f;
    scale = 1.0f / scale;
    float scale_sq_sq = scale * scale * scale * scale;
    
    AutoBuffer<int> ofsbuf(blockSize*blockSize);
    int* ofs = ofsbuf;
    for( int i = 0; i < blockSize; i++ )
        for( int j = 0; j < blockSize; j++ )
            ofs[i*blockSize + j] = (int)(i*step + j);
    
    for( ptidx = 0; ptidx < ptsize; ptidx++ )
    {
        int x0 = cvRound(pts[ptidx].pt.x - r);
        int y0 = cvRound(pts[ptidx].pt.y - r);
        
        const uchar* ptr0 = ptr00 + y0*step + x0;
        int a = 0, b = 0, c = 0;
        
        for( int k = 0; k < blockSize*blockSize; k++ )
        {
            const uchar* ptr = ptr0 + ofs[k];
            int Ix = (ptr[1] - ptr[-1])*2 + (ptr[-step+1] - ptr[-step-1]) + (ptr[step+1] - ptr[step-1]);
            int Iy = (ptr[step] - ptr[-step])*2 + (ptr[step-1] - ptr[-step-1]) + (ptr[step+1] - ptr[-step+1]);
            a += Ix*Ix;
            b += Iy*Iy;
            c += Ix*Iy;
        }
        pts[ptidx].response = ((float)a * b - (float)c * c -
                               harris_k * ((float)a + b) * ((float)a + b))*scale_sq_sq;
    }
}

bool set_src_feature(cv::Mat &src){
    
    if (src.empty())
        return false;
    
    //    input_src = src.clone();
    //resize input image
    if (!is_input_scale_set) {
        if (work_pixel == 0){
            input_scale = min(1.0, sqrt(input_megapix * 1e6 / src.size().area()));
            is_input_scale_set = true;
        }
        else{
            if (min(src.rows, src.cols)<= work_pixel){
                input_scale = 1;
                is_input_scale_set = true;
            }
            else{
                input_scale = work_pixel*1./min(src.rows, src.cols);
                is_input_scale_set = true;
            }
        }
    }
    resize(src, input, Size(), input_scale, input_scale, INTER_NEAREST);
    
    cv::Mat gray;
    if(input.channels() == 3)
        cv::cvtColor(input, gray, CV_BGR2GRAY);
    else
        input.copyTo(gray);
    //    cv::GaussianBlur(gray, gray, cv::Size(3,3), 0.5, 0.5);
    //    cv::medianBlur(gray, gray, 3);
    //feature detect
    fastdetector->detect(gray, src_features.keypoints);
    
    if (src_features.keypoints.size() < 200)
        return false;
    
    KeyPointsFilter::runByImageBorder(src_features.keypoints, gray.size(), 31);
    KeyPointsFilter::retainBest(src_features.keypoints, maxkeypoint);
    
    //feature description
    extractor->compute(gray, src_features.keypoints, src_features.descriptors);
    src_features.descriptors = src_features.descriptors.reshape(1, (int)src_features.keypoints.size());
    
    //build flann index
    tree.release();
    tree = makePtr<cv::flann::Index>(src_features.descriptors, cv::flann::LshIndexParams(5, 15, 0), cvflann::FLANN_DIST_HAMMING);
    
    //set initial points
    obj_corners[0] = Point2f(0, 0);
    obj_corners[1] = Point2f( (float)input.cols, 0 );
    obj_corners[2] = Point2f( (float)input.cols, (float)input.rows );
    obj_corners[3] = Point2f( 0, (float)input.rows );
    
    return true;
}

bool overlap_point(cv::Mat &dst, vector<Point> &src_pnts, vector<Point> &dst_pnts){
    if (dst.empty())
        return false;
    //resize input image
    if (!is_input_scale_set) {
        if (work_pixel == 0){
            input_scale = min(1.0, sqrt(input_megapix * 1e6 / dst.size().area()));
            is_input_scale_set = true;
        }
        else{
            if (min(dst.rows, dst.cols)<= work_pixel){
                input_scale = 1;
                is_input_scale_set = true;
            }
            else{
                input_scale = work_pixel*1./min(dst.rows, dst.cols);
                is_input_scale_set = true;
            }
        }
    }
    if(dst.channels() == 3)
        cv::cvtColor(dst, input, CV_BGR2GRAY);
    else
        dst.copyTo(input);
    resize(input, input, Size(), input_scale, input_scale, INTER_NEAREST);
    
    //    cv::Mat gray;
    //    cv::cvtColor(input, gray, CV_BGR2GRAY);
    
    //    cv::medianBlur(input, input, 3);
    //    cv::GaussianBlur(input, input, cv::Size(3,3), 0.5, 0.5);
    //feature detect
    fastdetector->detect(input, dst_features.keypoints);
    
    if (dst_features.keypoints.size() < 200)
        return false;
    
    KeyPointsFilter::runByImageBorder(dst_features.keypoints, input.size(), 31);
    KeyPointsFilter::retainBest(dst_features.keypoints, maxkeypoint);
    
    //feature description
    extractor->compute(input, dst_features.keypoints, dst_features.descriptors);
    dst_features.descriptors = dst_features.descriptors.reshape(1, (int)dst_features.keypoints.size());
    
    //flann knn search
    cv::Mat indices, dists;
    tree->knnSearch(dst_features.descriptors, indices, dists, 2, *flann_search);
    
    //get match points
    obj_pt.clear();
    scene_pt.clear();
    float* dists_ptr;
    int* indeces_ptr;
    for(int i=0;i<dists.rows;i++)
    {
        dists_ptr=dists.ptr<float>(i);
        indeces_ptr = indices.ptr<int>(i);
        if (dists_ptr[0]<(1.f - match_conf)*dists_ptr[1])
        {
            //            DMatch dmatches(indeces_ptr[0], i, dists_ptr[0]);
            //            tmpmatches.push_back(dmatches);
            obj_pt.push_back( src_features.keypoints[indeces_ptr[0]].pt );
            scene_pt.push_back( dst_features.keypoints[i].pt );
        }
    }
    
    if (obj_pt.size() < 50)
        return false;
    //compute warping matrix
    std::vector<uchar> inliers_mask;
    cv::Mat H = estimateAffine2D(obj_pt, scene_pt, inliers_mask);
    H.push_back(Mat::zeros(1, 3, CV_64F));
    H.at<double>(2, 2) = 1;
    
    int good_num = 0;
    for (int i=0; i<inliers_mask.size();++i){
        if (inliers_mask[i] != '\0')
            good_num++;
    }
    
    float conf = good_num /(8 + 0.3 * (obj_pt.size()));
    if (good_num < 10 || conf < 0.5)
        return false;
    
    //dst points transformation
    perspectiveTransform( obj_corners, scene_corners, H);
    //
    //    if (abs(scene_corners[0].x-scene_corners[1].x) > 4*abs(obj_corners[0].x-obj_corners[1].x) ||
    //        abs(scene_corners[2].x-scene_corners[3].x) > 4*abs(obj_corners[2].x-obj_corners[3].x) ||
    //        abs(scene_corners[1].y-scene_corners[2].y) > 4*abs(obj_corners[1].y-obj_corners[2].y) ||
    //        abs(scene_corners[3].y-scene_corners[0].y) > 4*abs(obj_corners[3].y-obj_corners[0].y) )
    //
    //        return false;
    
    //upsample
    float scale = 1. / input_scale;
    for(int i=0; i<4; i++){
        dst_pnts[i].x = scene_corners[i].x*scale;
        dst_pnts[i].y = scene_corners[i].y*scale;
    }
    //src points transformation
    perspectiveTransform( obj_corners, scene_corners, H.inv());
    //upsample
    for(int i=0; i<4; i++){
        src_pnts[i].x = scene_corners[i].x*scale;
        src_pnts[i].y = scene_corners[i].y*scale;
    }
    
    return true;
}

template <typename T>
vector<long unsigned int> sort_indexes_e(vector<T> &v)
{
    vector<long unsigned int> idx(v.size());
    iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(), [&v](long unsigned int i1, long unsigned int i2) {return v[i1] < v[i2]; });
    return idx;
}

//最原始的匹配逻辑，一次多张,
#ifdef ORIGIN
bool stitching_ori(vector<Mat> &imglist, Mat &dst, std::string &strLogPath){
    
    struct timeval stitch_start, stitch_end;
    gettimeofday( &stitch_start, NULL );
    
    ofstream output_file;
    if(!strLogPath.empty())
    {
        output_file.open(strLogPath, ios::out); //zt
        if (imglist.size() == 0) {
            output_file << "LOGERROR: imglist size equal 0, please input more images. " << endl;  //zt
            output_file.close();
            return false;
        }
        if (imglist.size()==1) {
            dst = imglist[0];
            output_file << "LOGERROR: imglist size equal 1, please input more images. " << endl;  //zt
            output_file.close();
            return true;
        }
    }
    else
    {
        if (imglist.size() == 0) {
            std::cout << "LOGERROR: imglist size equal 0, please input more images. " << endl;  //zt
            return false;
        }
        if (imglist.size()==1) {
            dst = imglist[0];
            std::cout << "LOGERROR: imglist size equal 1, please input more images. " << endl;  //zt
            return true;
        }
    }
    
    if (!is_input_scale_set) {
        if (work_pixel == 0){
            input_scale = min(1.0, sqrt(input_megapix * 1e6 / imglist[0].size().area()));
            is_input_scale_set = true;
        }
        else{
            if (min(imglist[0].rows, imglist[0].cols) <= work_pixel){
                input_scale = 1;
                is_input_scale_set = true;
            }
            else{
                input_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                is_input_scale_set = true;
            }
        }
    }
    
    int num_images = imglist.size();
    vector<ImageFeatures> features(num_images);
    vector<Mat> images(num_images);
    vector<Mat> input_images(num_images);
    Mat img;
    for(int i=0; i< imglist.size(); i++){
        
        resize(imglist[i], input, Size(), input_scale, input_scale, INTER_NEAREST);
        
        if (work_megapix < 0)
        {
            img = input.clone();
            work_scale = 1;
            is_work_scale_set = true;
        }
        else {
            if (!is_work_scale_set) {
                if (work_pixel == 0){
                    work_scale = min(1.0, sqrt(work_megapix * 1e6 / input.size().area()));
                    is_work_scale_set = true;
                }
                else{
                    if (min(imglist[i].rows, imglist[i].cols)<= work_pixel){
                        work_scale = 1;
                        is_work_scale_set = true;
                    }
                    else{
                        work_scale = work_pixel*1./min(imglist[i].rows, imglist[i].cols);
                        is_work_scale_set = true;
                    }
                }
            }
            resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
        }
        if (!is_seam_scale_set)
        {
            seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / input.size().area()));
            seam_work_aspect = seam_scale / work_scale;
            is_seam_scale_set = true;
        }
        
        struct timeval find_start, find_end;
        gettimeofday( &find_start, NULL );
        (*finder)(img, features[i]);
        gettimeofday( &find_end, NULL );
        //求出两次时间的差值，单位为us
        int findtimeuse = 1000000 * ( find_end.tv_sec - find_start.tv_sec ) + find_end.tv_usec - find_start.tv_usec;
        if(!strLogPath.empty())
            output_file << "find time is " << findtimeuse << "  us."<< std::endl;
        else
            printf("find time: %d us\n", findtimeuse);
        
        int FeatureSize = features.size();
        // zt log features
        if(!strLogPath.empty())
        {
            output_file << "LOGINFO: img idx is " << i << ", img width is " << img.cols << ", img height is " << img.rows << std::endl;
            //zt
            //            output_file << "LOGINFO: feature size is " << FeatureSize  << " , 第 " <<  i << " 张图 keypoints size is " << features[i].keypoints.size()<< std::endl;
            //            output_file << "LOGINFO: feature point: " << std::endl;
            //            for (int t = 0; t < features[i].keypoints.size(); ++t)
            //            {
            //                output_file << "    x = " << features[i].keypoints[t].pt.x << " , y = "  << features[i].keypoints[t].pt.y << endl;  //zt
            //            }
        }
        features[i].img_idx = i;
        
        if (try_compose == true)
            resize(input, img, Size(), seam_scale, seam_scale, INTER_NEAREST);
        else {
            if (!is_compose_scale_set) {
                if (work_pixel == 0) {
                    compose_scale = min(1.0, sqrt(compose_scale * 1e6 / input.size().area()));
                    is_compose_scale_set = true;
                } else {
                    if (min(imglist[i].rows, imglist[i].cols) <= work_pixel) {
                        compose_scale = 1;
                        is_compose_scale_set = true;
                    } else {
                        compose_scale = work_pixel * 1. / min(imglist[i].rows, imglist[i].cols);
                        is_compose_scale_set = true;
                    }
                }
            }
            resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
        }
        
        images[i] = img.clone();
        input_images[i] = input.clone();
        
    }
    
    finder->collectGarbage();
    img.release();
    
    vector< MatchesInfo > pairwise_matches;
    //clock_t match_start = clock();
    struct timeval match_start, match_end;
    gettimeofday( &match_start, NULL );
    (*matcher)(features, pairwise_matches/*, cv::UMat()*/);
    gettimeofday( &match_end, NULL );
    //求出两次时间的差值，单位为us
    int matchtimeuse = 1000000 * ( match_end.tv_sec - match_start.tv_sec ) + match_end.tv_usec - match_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "match time is " << matchtimeuse << "  us."<< std::endl;
    else
        printf("match time: %d us\n", matchtimeuse);
    
    // std::cout << "match time is " << clock() - match_start << std::endl;
    matcher->collectGarbage();
    //检查是否所有的都符合拼接
    
    vector<int> indices;
    indices.clear();
    indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);
    if(indices.size() < num_images)
    {
        std::cout << "indices size < num images" << std::endl;
        return false;
    }
    
    // check pairwise matches
    vector<int> pair_idx;
    vector<double> pair_confidence;
    vector<int> pair_points;
    for (int i = 0; i < num_images; ++i) {
        pair_idx.push_back(-1);
        pair_confidence.push_back(0.0);
        pair_points.push_back(0);
    }
    int src_img_idx;
    int dst_img_idx;
    double confidence;
    int num_inliers;
    for (int i = 0; i < pairwise_matches.size(); ++i) {
        src_img_idx = pairwise_matches[i].src_img_idx;
        dst_img_idx = pairwise_matches[i].dst_img_idx;
        confidence = pairwise_matches[i].confidence;
        num_inliers = pairwise_matches[i].num_inliers;
        if (pair_idx[src_img_idx] == -1) {
            pair_idx[src_img_idx] = dst_img_idx;
            pair_confidence[src_img_idx] = confidence;
            pair_points[src_img_idx] = num_inliers;
        } else if (confidence > pair_confidence[src_img_idx]) {
            pair_idx[src_img_idx] = dst_img_idx;
            pair_confidence[src_img_idx] = confidence;
            pair_points[src_img_idx] = num_inliers;
        }
    }
    
    bool bflag = false;//zt
    if(!strLogPath.empty())
    {
        output_file << "LOGINFO: MatchePoint Info:  " << std::endl;
        
        for (int i = 0; i < num_images; ++i) {
            // output_file << "    src img is " << i << ", largest pair idx is " << pair_idx[i] << ", max confidence is " << pair_confidence[i] << ", num inliers is "<< pair_points[i]  << std::endl;
            
            if (pair_points[i] <= inliner_num) {
                bflag = true;
            }
            //        if (pair_confidence[i] < confidence_thresh) {
            //           // return false;
            //            flag = true;
            //        }
        }
        
        if(bflag)
        {
            output_file << "LOGINFO: inliner_num < 32" << std::endl;
            output_file.close(); //zt
            return false; //zt
        }
    }
    else
    {
        for (int i = 0; i < num_images; ++i) {
            //std::cout << "src img is " << i << ", largest pair idx is " << pair_idx[i] << ", max confidence is " << pair_confidence[i] << ", num inliers is "<< pair_points[i]  << std::endl;
            //std::cout << "    src img is " << i << ", largest pair idx is " << pair_idx[i] << ", max confidence is " << pair_confidence[i] << ", num inliers is "<< pair_points[i]  << std::endl;
            if (pair_points[i] <= inliner_num) {
                bflag = true;
            }
            //        if (pair_confidence[i] < confidence_thresh) {
            //           // return false;
            //            flag = true;
            //        }
        }
        
        if(bflag)
        {
            std::cout << "LOGINFO: inliner_num < 32" << std::endl;
            return false; //zt
        }
    }
    
    vector<CameraParams> cameras;
    //estimator
    struct timeval estimator_start, estimator_end;
    gettimeofday( &estimator_start, NULL );
    
    if(!strLogPath.empty())
    {
        if (!(*estimator)(features, pairwise_matches, cameras)){
            output_file << "LOGINFO: camera estimat failed." << std::endl; //zt
            output_file.close(); //zt
            return false;
        }
    }
    else
    {
        if (!(*estimator)(features, pairwise_matches, cameras)){
            std::cout << "LOGINFO: camera estimat failed." << std::endl; //zt
            return false;
        }
    }
    gettimeofday( &estimator_end, NULL );
    int estimatortimeuse = 1000000 * ( estimator_end.tv_sec - estimator_start.tv_sec ) + estimator_end.tv_usec - estimator_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "estimator time is " << estimatortimeuse << "  us."<< std::endl;
    else
        printf("estimator time: %d us\n", estimatortimeuse);
    
    
    for (size_t i = 0; i < cameras.size(); ++i){
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
    }
#ifndef ADJUST
    //  adjust
    struct timeval adjust_start, adjust_end;
    gettimeofday( &adjust_start, NULL);
    if(!strLogPath.empty())
    {
        if (!(*adjuster)(features, pairwise_matches, cameras)){
            output_file << "LOGERROR: camera adjuster failed." << std::endl; //zt
            output_file.close(); //zt
            return false;
        }
        // check camera parameter R
        CameraParams camera;
        for (int i = 0; i < cameras.size(); ++i) {
            camera = cameras[i];
            for (int row = 0; row < camera.R.rows; row++) {
                for (int col = 0; col < camera.R.cols; col++) {
                    if (isnan(camera.R.at<int>(row,col))) {
                        output_file << "LOGINFO: camera check failed." << std::endl; //zt
                        output_file.close(); //zt
                        return false;
                    }
                }
            }
        }
        
    }
    else
    {
        if (!(*adjuster)(features, pairwise_matches, cameras)) {
            std::cout << "LOGERROR: camera adjuster failed." << std::endl; //zt
            return false;
        }
        // check camera parameter R
        CameraParams camera;
        for (int i = 0; i < cameras.size(); ++i) {
            camera = cameras[i];
            for (int row = 0; row < camera.R.rows; row++) {
                for (int col = 0; col < camera.R.cols; col++) {
                    if (isnan(camera.R.at<int>(row,col))) {
                        std::cout << "LOGINFO: camera check failed." << std::endl; //zt
                        return false;
                    }
                }
            }
        }
    }
    
    gettimeofday( &adjust_end, NULL );
    int adjusttimeuse = 1000000 * ( adjust_end.tv_sec - adjust_start.tv_sec ) + adjust_end.tv_usec - adjust_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "adjust time is " << adjusttimeuse << "  us."<< std::endl;
    else
        printf("adjust time: %d us\n", adjusttimeuse);
#endif
    // std::cout << "adjust time is " << clock() - adjust_start << std::endl;
    //zt save camera params
    camera_params = cameras;
    
    vector<double> focals;
    focals.clear(); //zt
    for (size_t i = 0; i < cameras.size(); ++i){
        focals.push_back(cameras[i].focal);
    }
    
    sort(focals.begin(), focals.end());
    float warped_image_scale;
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;
    
    vector<Point> corners(num_images);
    vector<UMat> masks_warped(num_images);
    vector<UMat> images_warped(num_images);
    vector<Size> sizes(num_images);
    vector<UMat> masks(num_images);
    
    //warp
    struct timeval warp_start, warp_end;
    gettimeofday( &warp_start, NULL );
#ifndef WARP
    // Preapre images masks
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }
    
    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
    
    for (int i = 0; i < num_images; ++i)
    {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;
        
        //check
        Rect roi = warper->warpRoi(images[i].size(), K, cameras[i].R);
        if (roi.width > pano_max_edge || roi.height > pano_max_edge){
            if(!strLogPath.empty())
            {   output_file << "LOGERROR： roi.width or roi.height > pano_max_edge. "<< std::endl;
                output_file.close();
                return false;
            }
            else
            {
                std::cout << "LOGERROR： roi.width or roi.height > pano_max_edge. "<< std::endl;
                return false;
            }
        }
        
        corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();
        
        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
        
    }
    gettimeofday( &warp_end, NULL );
    int warptimeuse = 1000000 * ( warp_end.tv_sec - warp_start.tv_sec ) + warp_end.tv_usec - warp_start.tv_usec;
    printf("warp time: %d us\n", warptimeuse);
    
    
    vector<UMat> images_warped_f(num_images);
    for (int i = 0; i < num_images; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);
    //seam
    struct timeval seam_start, seam_end;
    gettimeofday( &seam_start, NULL );
    
    seam_finder->find(images_warped_f, corners, masks_warped);
    
    gettimeofday( &seam_end, NULL );
    int seamtimeuse = 1000000 * ( seam_end.tv_sec - seam_start.tv_sec ) + seam_end.tv_usec - seam_start.tv_usec;
    printf("seam time: %d us\n", seamtimeuse);
    
    //    for(int i=0; i< masks_warped.size(); ++i){
    //        imshow("mask", masks_warped[i]);
    //        imshow("image", images_warped[i]);
    //        waitKey(0);
    //    }
    
    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    Mat input_img;
    double compose_seam_aspect = 1;
    double compose_work_aspect = 1;
    
    //compose
    struct timeval compose_start, compose_end;
    gettimeofday( &compose_start, NULL );
    
    if (!is_compose_scale_set ) {
        compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / input.size().area()));
        is_compose_scale_set = true;
    }
    
    // Compute relative scales
    compose_seam_aspect = compose_scale / seam_scale;
    compose_work_aspect = compose_scale / work_scale;
    
    // Update warped image scale
    warped_image_scale *= static_cast<float>(compose_work_aspect);
    warper = warper_creator->create(warped_image_scale);
    
    // Update corners and sizes
    for (int i = 0; i < num_images; ++i)
    {
        // Update intrinsics
        cameras[i].focal *= compose_work_aspect;
        cameras[i].ppx *= compose_work_aspect;
        cameras[i].ppy *= compose_work_aspect;
        
        // Update corner and size
        Size sz = input.size();
        if (std::abs(compose_scale - 1) > 1e-1)
        {
            sz.width = cvRound(input.size().width * compose_scale);
            sz.height = cvRound(input.size().height * compose_scale);
        }
        
        Mat K;
        cameras[i].K().convertTo(K, CV_32F);
        Rect roi = warper->warpRoi(sz, K, cameras[i].R);
        corners[i] = roi.tl();
        sizes[i] = roi.size();
    }
    
    for (int img_idx = 0; img_idx < num_images; ++img_idx)
    {
        // Read image and resize it if necessary
        input_img = input_images[img_idx].clone();
        
        if (abs(compose_scale - 1) > 1e-1)
            resize(input_img, img, Size(), compose_scale, compose_scale, INTER_LINEAR_EXACT);
        else
            img = input_img.clone();
        
        input_img.release();
        Size img_size = img.size();
        
        Mat K;
        cameras[img_idx].K().convertTo(K, CV_32F);
        
        // Warp the current image
        warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);
        
        // Warp the current image mask
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);
        
        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();
        
        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
        mask_warped = seam_mask & mask_warped;
        
        if (!blender && !timelapse)
        {
            blender = Blender::createDefault(blend_type, try_cuda);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, try_cuda);
            else if (blend_type == Blender::MULTI_BAND)
            {
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
            }
            blender->prepare(corners, sizes);
        }
        
        // Blend the current image
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }
    
    Mat result, result_mask;
    blender->blend(result, result_mask);
    
    gettimeofday( &compose_end, NULL );
    int composetimeuse = 1000000 * ( compose_end.tv_sec - compose_start.tv_sec ) + compose_end.tv_usec - compose_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "compose time is " << composetimeuse << "  us."<< std::endl;
    else
        printf("compose time: %d us\n", composetimeuse);
    
    //other
    struct timeval other_start, other_end;
    gettimeofday( &other_start, NULL );
    
    result.copyTo(dst);
    result_size = result.size();
    //zt log camera parameters
    if(!strLogPath.empty())
    {
        for (size_t tt = 0; tt < cameras.size(); ++tt)
        {
            //        for (int k = 0; k < cameras[tt].R.cols; ++k) { // 每一行图像的指针
            //
            //            for (int g = 0; g < cameras[tt].R.rows; g++) {
            //                // std::cout << cameras[tt].R.at<float>(k,g);
            //
            //            }
            //        }
            //        std::cout << "LOGINFO111: camera information: " << std::endl;  //zt
            //        std::cout << "    aspect is " << cameras[tt].aspect << endl;
            //        std::cout << "    focal is " << cameras[tt].focal << endl;
            //        std::cout << "    Principal point X is " << cameras[tt].ppx  << ", Principal point Y is " << cameras[tt].ppy << std::endl;
            //        std::cout << "    R matrix is " << cameras[tt].R << std::endl;
            //        std::cout << "    t matrix is " << cameras[tt].t << std::endl;
            
            output_file << "LOGINFO: camera information: " << std::endl;  //zt
            output_file << "    aspect is " << cameras[tt].aspect << endl;
            output_file << "    focal is " << cameras[tt].focal << endl;
            output_file << "    Principal point X is " << cameras[tt].ppx  << ", Principal point Y is " << cameras[tt].ppy << std::endl;
            output_file << "    R matrix is " << cameras[tt].R << std::endl;
            output_file << "    t matrix is " << cameras[tt].t << std::endl;
        }
        // zt log pano size
        output_file << "LOGINFO: pano size is " << result_size << std::endl; //zt
    }
    Point left_top;
    Point right_bottom;
    int stitch_x = 0;
    int stitch_y = 0;
    Size blend_image_size;
    vector<Point> corner_points;
    corner_points.clear();
    Point map_idx;
    vector<int> left_top_x;
    left_top_x.clear();
    vector<int> left_top_y;
    left_top_y.clear();
    //convert left_top points and size into corner points
    for (int img_idx = 0; img_idx < num_images; ++img_idx) {
        // calculate blend images 4 corner points
        blend_image_size = sizes[img_idx];
        left_top = corners[img_idx];
        if (left_top.x < stitch_x) {
            stitch_x = left_top.x;
        }
        if (left_top.y < stitch_y) {
            stitch_y = left_top.y;
        }
        right_bottom.x = left_top.x + blend_image_size.width;
        right_bottom.y = left_top.y + blend_image_size.height;
        //std::cout << "left_top: " << left_top << std::endl;
        //std::cout << "right_bottom: " << right_bottom << std::endl;
        corner_points.push_back(left_top);
        corner_points.push_back(right_bottom);
        sum_corner_points.push_back(corner_points);
        corner_points.clear();
    }
    if(!strLogPath.empty())
        output_file << " LOGINFO: stitch_left_topx is " << stitch_x << " ,stitch_left_topy is " << stitch_y << std::endl;
    for (int i = 0; i < num_images; ++i) {
        sum_corner_points[i][0].x = sum_corner_points[i][0].x - stitch_x;
        sum_corner_points[i][0].y = sum_corner_points[i][0].y - stitch_y;
        sum_corner_points[i][1].x = sum_corner_points[i][1].x - stitch_x;
        sum_corner_points[i][1].y = sum_corner_points[i][1].y - stitch_y;
        Point tmp_lt_pnt = sum_corner_points[i][0];
        Point tmp_rb_pnt = sum_corner_points[i][1];
        if(!strLogPath.empty())
            output_file << " LOGINFO: cornetPoints Info " << i << " image corner points in pano: " << tmp_lt_pnt << ", " << tmp_rb_pnt << std::endl;
    }
    //image map index
    for (int img_idx = 0; img_idx < num_images; ++img_idx) {
        left_top_x.push_back(corners[img_idx].x);
        left_top_y.push_back(corners[img_idx].y);
    }
    vector<long unsigned int> sorted_x_idx;
    sorted_x_idx.clear();
    vector<long unsigned int> sorted_y_idx;
    sorted_y_idx.clear();
    sorted_x_idx = sort_indexes_e(left_top_x);
    sorted_y_idx = sort_indexes_e(left_top_y);
    int x;
    int y;
    int w_diff;
    int h_diff;
    vector<int> vec_w_diff;
    vector<int> vec_h_diff;
    vector<int> corner_dist;
    vector<long unsigned int> sorted_dist_idx;
    vec_w_diff.clear();
    vec_h_diff.clear();
    corner_dist.clear();
    sorted_dist_idx.clear();
    int cloest_idx;
    Point cloest_point;
    int step;
    int step_direction; // 0: x; 1: y
    for (int img_idx = 0; img_idx < num_images; ++img_idx) {
        if (img_idx == 0) {
            map_idx.x = 0;
            map_idx.y = 0;
        } else {
            x = left_top_x[img_idx];
            y = left_top_y[img_idx];
            for (int i = 0; i < image_map_index.size(); ++i) {
                w_diff = x - left_top_x[i];
                h_diff = y - left_top_y[i];
                vec_w_diff.push_back(w_diff);
                vec_h_diff.push_back(h_diff);
                corner_dist.push_back(abs(w_diff) + abs(h_diff));
            }
            sorted_dist_idx = sort_indexes_e(corner_dist);
            cloest_idx = sorted_dist_idx[0];
            cloest_point = image_map_index[cloest_idx];
            
            w_diff = vec_w_diff[cloest_idx];
            h_diff = vec_h_diff[cloest_idx];
            if(!strLogPath.empty())
            {
                output_file << " LOGINFO: cloest_idx: " << cloest_idx << std::endl;
                output_file << " LOGINFO: cloest_point: " << cloest_point << std::endl;
                
                output_file << " LOGINFO: w_diff: " << w_diff << std::endl;
                output_file << " LOGINFO: h_diff: " << h_diff << std::endl;
            }
            //            std::cout << " LOGINFO: cloest_idx: " << cloest_idx << std::endl;
            //            std::cout << " LOGINFO: cloest_point: " << cloest_point << std::endl;
            //
            //            std::cout << " LOGINFO: w_diff: " << w_diff << std::endl;
            //            std::cout << " LOGINFO: h_diff: " << h_diff << std::endl;
            if (abs(w_diff) >= abs(h_diff)) {
                step_direction = 0;
                if (w_diff >= 0) {
                    step = 1;
                } else {
                    step = -1;
                }
                map_idx.x = cloest_point.x + step;
                map_idx.y = cloest_point.y;
            } else {
                step_direction = 1;
                if (h_diff >= 0) {
                    step = 1;
                } else {
                    step = -1;
                }
                map_idx.y = cloest_point.y + step;
                map_idx.x = cloest_point.x;
            }
            for (int i = 0; i < image_map_index.size(); ++i) {
                Point tmp_point = image_map_index[i];
                if (step_direction == 0 && tmp_point.y == map_idx.y && tmp_point.x >= map_idx.x) {
                    ++image_map_index[i].x;
                } else if (step_direction == 1 && tmp_point.x == map_idx.x && tmp_point.y >= map_idx.y) {
                    ++image_map_index[i].y;
                }
            }
        }
        image_map_index.push_back(map_idx);
        vec_w_diff.clear();
        vec_h_diff.clear();
        corner_dist.clear();
        sorted_dist_idx.clear();
    }
    //image map index reset
    vector<int> map_idx_x;
    map_idx_x.clear();
    vector<int> map_idx_y;
    map_idx_y.clear();
    vector<int> map_idx_first_row;
    map_idx_first_row.clear();
    for (int i = 0; i < num_images; ++i) {
        map_idx_x.push_back(image_map_index[i].x);
        map_idx_y.push_back(image_map_index[i].y);
        if (image_map_index[i].y == 0) {
            map_idx_first_row.push_back(image_map_index[i].x);
        }
    }
    //convert negative index to postive
    int min_x = *min_element(map_idx_x.begin(), map_idx_x.end());
    int min_y = *min_element(map_idx_y.begin(), map_idx_y.end());
    if (min_x < 0) {
        for (int i = 0; i < map_idx_x.size(); ++i) {
            map_idx_x[i] = map_idx_x[i] + abs(min_x);
        }
        for (int i = 0; i < map_idx_first_row.size(); ++i) {
            map_idx_first_row[i] = map_idx_first_row[i] + abs(min_x);
        }
    }
    if (min_y < 0) {
        for (int i = 0; i < num_images; ++i) {
            map_idx_y[i] = map_idx_y[i] + abs(min_y);
        }
    }
    //find the miss first row col after positive convert
    map_idx_first_row.clear();
    for (int i = 0; i < num_images; ++i) {
        if (map_idx_y[i] == 0) {
            map_idx_first_row.push_back(map_idx_x[i]);
            // if(!strLogPath.empty())
            //output_file<< " LOGINFO: first x index: " << map_idx_first_row[i] << std::endl;
        }
    }
    //find the col index of missing point in first row
    vector<int>::iterator ret;
    vector<int> miss_x;
    miss_x.clear();
    for (int i = 0; i < num_images; ++i) {
        if (map_idx_y[i] > 0) {
            ret = find(map_idx_first_row.begin(), map_idx_first_row.end(), map_idx_x[i]);
            if (ret == map_idx_first_row.end()) {
                miss_x.push_back(map_idx_x[i]);
            }
        }
    }
    //pop up the missing column until touch the first row
    vector<vector<int>> miss_columns;
    miss_columns.clear();
    vector<vector<int>> tmp_idxes;
    tmp_idxes.clear();
    vector<int> col;
    col.clear();
    for (int i = 0; i < miss_x.size(); ++i) {
        miss_columns.push_back(col);
        tmp_idxes.push_back(col);
    }
    int col_idx;
    for (int i = 0; i < num_images; ++i) {
        if (map_idx_y[i] > 0) {
            ret = find(miss_x.begin(), miss_x.end(), map_idx_x[i]);
            if (ret != miss_x.end()) {
                col_idx = distance(miss_x.begin(), ret);
                miss_columns[col_idx].push_back(map_idx_y[i]);
                tmp_idxes[col_idx].push_back(i);
            }
        }
    }
    for (int i = 0; i < miss_columns.size(); ++i) {
        col = miss_columns[i];
        int min_y_in_col = *min_element(col.begin(), col.end());
        for (int j = 0; j < col.size(); ++j) {
            map_idx_y[tmp_idxes[i][j]] = map_idx_y[tmp_idxes[i][j]] - min_y_in_col;
        }
    }
    //connect the break point in col
    col.clear();
    int n_col = *max_element(map_idx_x.begin(), map_idx_x.end()) + 1;
    vector<int> idx_in_col;
    idx_in_col.clear();
    int src_idx;
    int prev_idx;
    for (int i = 0; i <= n_col; ++i) {
        for (int j = 0; j < map_idx_y.size(); ++j) {
            if (map_idx_x[j] == i) {
                col.push_back(map_idx_y[j]);
                idx_in_col.push_back(j);
            }
        }
        vector<long unsigned int> sorted_col_idx = sort_indexes_e(col);
        for (int j = 1; j < sorted_col_idx.size(); ++j) {
            src_idx = idx_in_col[sorted_col_idx[j]];
            prev_idx = idx_in_col[sorted_col_idx[j-1]];
            if (map_idx_y[src_idx] != map_idx_y[prev_idx] + 1) {
                map_idx_y[src_idx] = map_idx_y[prev_idx] + 1;
            }
        }
        col.clear();
        idx_in_col.clear();
        sorted_col_idx.clear();
    }
    //connect the break point in row
    vector<int> row;
    row.clear();
    int n_row = *max_element(map_idx_y.begin(), map_idx_y.end()) + 1;
    vector<int> idx_in_row;
    idx_in_row.clear();
    src_idx = 0;
    prev_idx = 0;
    for (int i = 0; i <= n_row; ++i) {
        for (int j = 0; j < map_idx_x.size(); ++j) {
            if (map_idx_y[j] == i) {
                row.push_back(map_idx_x[j]);
                idx_in_row.push_back(j);
            }
        }
        vector<long unsigned int> sorted_row_idx = sort_indexes_e(row);
        for (int j = 1; j < sorted_row_idx.size(); ++j) {
            src_idx = idx_in_row[sorted_row_idx[j]];
            prev_idx = idx_in_row[sorted_row_idx[j-1]];
            if (map_idx_x[src_idx] != map_idx_x[prev_idx] + 1) {
                map_idx_x[src_idx] = map_idx_x[prev_idx] + 1;
            }
        }
        row.clear();
        idx_in_row.clear();
        sorted_row_idx.clear();
    }
    //restore map index, start from (1, 1)
    
    for (int i = 0; i < num_images; ++i) {
        image_map_index[i].x = map_idx_x[i] + 1;
        image_map_index[i].y = map_idx_y[i] + 1;
        std::cout << " map_idx_x is "<< map_idx_x[i]<<  ",map_idx_y is " << map_idx_y[i] << std::endl;
    }
    
    // zt log image_map_index
    //TODO
    for (int i = 0; i < num_images; ++i) {
        std::cout << " image_map_index  x is "<< image_map_index[i].x <<  ", image_map_index y is " << image_map_index[i].y << std::endl;
    }
    //        image_map_index.clear();
    //        image_map_index[0].x = map_idx_x[num_images - 1] + 1;
    //        image_map_index[0].y = map_idx_y[num_images - 1] + 1;
    // zt log image_map_index
    
    //std::cout << "    x is " << image_map_index[0].x << ", y is " << image_map_index[0].y << std::endl;
    // Release unused memory
    images.clear();
    input_images.clear();
    images_warped.clear();
    images_warped_f.clear();
    masks.clear();
    cameras.clear();
    corners.clear();
    sizes.clear();
    
    img_warped_s.release();
    dilated_mask.release();
    seam_mask.release();
    mask_warped.release();
    result.release();
    result_mask.release();
    
    corner_points.clear();
    left_top_x.clear();
    left_top_y.clear();
    sorted_x_idx.clear();
    sorted_y_idx.clear();
    map_idx_x.clear();
    map_idx_y.clear();
    map_idx_first_row.clear();
    miss_x.clear();
    miss_columns.clear();
    tmp_idxes.clear();
    col.clear();
#endif
    gettimeofday( &other_end, NULL );
    int othertimeuse = 1000000 * ( other_end.tv_sec - other_start.tv_sec ) + other_end.tv_usec - other_start.tv_usec;
    printf("other time: %d us\n", othertimeuse);
    
    
    gettimeofday( &stitch_end, NULL );
    //求出两次时间的差值，单位为us
    int stitchtimeuse = 1000000 * ( stitch_end.tv_sec - stitch_start.tv_sec ) + stitch_end.tv_usec - stitch_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "total time is " << stitchtimeuse << "  us."<< std::endl;
    else
        printf("total time: %d us\n", stitchtimeuse);
    
    if(!strLogPath.empty())
        output_file.close(); //zt
    
    return true;
}
#endif
//新的拼接逻辑，一次一张 , 已经按照 NEWMETHOD_MAP 去除滑窗, //一次一张, 已验证,没问题
#ifdef NEWMETHOD
bool stitching(std::vector <cv::Mat> &imglist, Mat &dst, std::string & strLogPath){
    
    struct timeval stitch_start, stitch_end;
    gettimeofday( &stitch_start, NULL );
    
    ofstream output_file;
    if(!strLogPath.empty())
    {
        output_file.open(strLogPath, ios::out); //zt
        if(imglist.size() == 0)
        {
            output_file << "LOGERROR: input images = 0." << std::endl;
            output_file.close();
            return false;
        }
        if(imglist.size() > 1)
        {
            output_file << "LOGERROR: input images > 1, please input just one img." << std::endl;
            output_file.close();
            return false;
        }
        
        if(imglist[0].empty())
        {
            output_file << "LOGERROR: input img is empty." << std::endl;
            output_file.close();
            return false;
        }
    }
    else
    {
        if(imglist.size() == 0)
        {
            printf("input images = 0.");
            return false;
        }
        if(imglist.size() > 1)
        {
            printf("input images > 1, please input just one img");
            return false;
        }
        
        if(imglist[0].empty())
        {
            printf("input img is empty.");
            return false;
        }
    }
    
    if (!is_input_scale_set) {
        if (work_pixel == 0){
            input_scale = min(1.0, sqrt(input_megapix * MaxPix_ / imglist[0].size().area()));
            is_input_scale_set = true;
        }
        else{
            if (min(imglist[0].rows, imglist[0].cols)<= work_pixel){
                input_scale = 1;
                is_input_scale_set = true;
            }
            else{
                input_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                is_input_scale_set = true;
            }
        }
    }
    
    //每次只能送一张进行stitching
    if(InputImgNums == 0)
        InputImgNums = imglist.size(); // 当前图片就1张
    
    if(InputImgNums == 1)  //first image , return current img as pino.jpg
    {
        dst = imglist[0];
        InputImgNums++;
        ImageFeatures featureTemp;
        Mat img;
        resize(imglist[0], input, Size(), input_scale, input_scale, INTER_NEAREST);
        
        if (work_megapix < 0)
        {
            img = input.clone();
            work_scale = 1;
            is_work_scale_set = true;
        }
        else {
            if (!is_work_scale_set) {
                if (work_pixel == 0){
                    work_scale = min(1.0, sqrt(work_megapix * MaxPix_ / input.size().area()));
                    is_work_scale_set = true;
                }
                else{
                    if (min(imglist[0].rows, imglist[0].cols)<= work_pixel){
                        work_scale = 1;
                        is_work_scale_set = true;
                    }
                    else{
                        work_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                        is_work_scale_set = true;
                    }
                }
            }
            resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
        }
        if (!is_seam_scale_set)
        {
            seam_scale = min(1.0, sqrt(seam_megapix * MaxPix_ / input.size().area()));
            seam_work_aspect = seam_scale / work_scale;
            is_seam_scale_set = true;
        }
        //find feature
        struct timeval first_find_start, first_find_end;
        gettimeofday( &first_find_start, NULL );
        (*finder)(img, featureTemp);
        gettimeofday( &first_find_end, NULL );
        //求出两次时间的差值，单位为us
        int firstfindtimeuse = 1000000 * ( first_find_end.tv_sec - first_find_start.tv_sec ) + first_find_end.tv_usec - first_find_start.tv_usec;
        if(!strLogPath.empty())
            output_file << "first img find time is " << firstfindtimeuse << "  us."<< std::endl;
        else
            printf("first img find time: %d us\n", firstfindtimeuse);
        
        featureTemp.img_idx = 0;
        global_img_feature.push_back(featureTemp); //debug
        
        finder->collectGarbage();
        
        if (try_compose == true)
            resize(input, img, Size(), seam_scale, seam_scale, INTER_NEAREST);
        else {
            if (!is_compose_scale_set) {
                if (work_pixel == 0) {
                    compose_scale = min(1.0, sqrt(compose_scale * MaxPix_ / input.size().area()));
                    is_compose_scale_set = true;
                } else {
                    if (min(imglist[0].rows, imglist[0].cols) <= work_pixel) {
                        compose_scale = 1;
                        is_compose_scale_set = true;
                    } else {
                        compose_scale = work_pixel * 1. / min(imglist[0].rows, imglist[0].cols);
                        is_compose_scale_set = true;
                    }
                }
            }
            resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
        }
        global_input_images.push_back(input.clone());//debug
        global_images.push_back(img.clone());  //debug
        if(!strLogPath.empty())
            output_file.close();
        return true;
    }
    
    std::cout << "global_input_images size is " << global_input_images.size() << std::endl;
    std::cout << "global_images size is " << global_images.size() << std::endl;
    std::cout << "global_img_feature size is " << global_img_feature.size() << std::endl;
    std::cout << "InputImgNums is " << InputImgNums << std::endl;
    struct timeval st_start, st_end;
    gettimeofday( &st_start, NULL );
    if(!stitchflag)  //前面拼接失败,需要删除前面最近一次的匹配信息
    {
        global_pairwise_matches.pop_back();
        
        if(InputImgNums == 2)
            global_pairwise_matches.clear();
        else  //大于最大匹配数
        {
            for(int i = 0; i < global_pairwise_matches.size(); ++i)
            {
                if(global_pairwise_matches[i].src_img_idx == InputImgNums - 1 || global_pairwise_matches[i].dst_img_idx == InputImgNums - 1)
                {
                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                    i = -1;
                }
            }
        }
    }
    gettimeofday( &st_end, NULL );
    int st_timeuse = 1000000 * ( st_end.tv_sec - st_start.tv_sec ) + st_end.tv_usec - st_start.tv_usec;
    printf("stichflag time: %d us\n", st_timeuse);
    
    Mat img;
    resize(imglist[0], input, Size(), input_scale, input_scale, INTER_NEAREST);
    
    if (work_megapix < 0)
    {
        img = input.clone();
        work_scale = 1;
        is_work_scale_set = true;
    }
    else {
        if (!is_work_scale_set) {
            if (work_pixel == 0){
                work_scale = min(1.0, sqrt(work_megapix * MaxPix_ / input.size().area()));
                is_work_scale_set = true;
            }
            else{
                if (min(imglist[0].rows, imglist[0].cols) <= work_pixel){
                    work_scale = 1;
                    is_work_scale_set = true;
                }
                else{
                    work_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                    is_work_scale_set = true;
                }
            }
        }
        resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
    }
    if (!is_seam_scale_set)
    {
        seam_scale = min(1.0, sqrt(seam_megapix * MaxPix_ / input.size().area()));
        seam_work_aspect = seam_scale / work_scale;
        is_seam_scale_set = true;
    }
    
    ImageFeatures feature;
    //find feature
    struct timeval find_start, find_end;
    gettimeofday( &find_start, NULL );
    (*finder)(img, feature);
    
    gettimeofday( &find_end, NULL );
    //求出两次时间的差值，单位为us
    int find_timeuse = 1000000 * ( find_end.tv_sec - find_start.tv_sec ) + find_end.tv_usec - find_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "LOGINFO: Finder time is " << find_timeuse << " us."<< std::endl;
    else
        printf("Finder time: %d us\n", find_timeuse);
    
    feature.img_idx = InputImgNums - 1;
    
    if (try_compose == true)
        resize(input, img, Size(), seam_scale, seam_scale, INTER_NEAREST);
    else {
        if (!is_compose_scale_set) {
            if (work_pixel == 0) {
                compose_scale = min(1.0, sqrt(compose_scale * MaxPix_ / input.size().area()));
                is_compose_scale_set = true;
            } else {
                if (min(imglist[0].rows, imglist[0].cols) <= work_pixel) {
                    compose_scale = 1;
                    is_compose_scale_set = true;
                } else {
                    compose_scale = work_pixel * 1. / min(imglist[0].rows, imglist[0].cols);
                    is_compose_scale_set = true;
                }
            }
        }
        resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
    }
    
    global_input_images.push_back(input.clone());
    global_images.push_back(img.clone());
    global_img_feature.push_back(feature);
    finder->collectGarbage();
    
    // 将matcher 分拆 ///////////////////////////////////////
    vector<MatchesInfo> pairwise_matches;
    pairwise_matches.clear();
    
    struct timeval match_start, match_end;
    gettimeofday( &match_start, NULL );
    
    int num_images = global_img_feature.size();
    
    // 创建匹配 pair, 每次新进来的要clear
    std::vector<std::pair<int,int> > near_pairs;
    near_pairs.clear();
    
    for(int idx = 0; idx < global_img_feature.size() - 1; ++idx)
        if (global_img_feature[global_img_feature.size() - 1].keypoints.size() > 0 && global_img_feature[0].keypoints.size() > 0)
            near_pairs.push_back(std::make_pair(idx,global_img_feature.size() - 1));
    
    pairwise_matches.resize(num_images * num_images);
    
    cv::RNG rng = cv::theRNG(); // save entry rng state
    Range r = Range(0, static_cast<int>(near_pairs.size()));
    
    vector <MatchesInfo> tmp_pairwise_matches;
    tmp_pairwise_matches.clear();// zt
    int MatchError = 0; //定义新进来的图片与之前图片匹配的错误数量，根据此来作为后续判断当前图片是否可以与之前图片拼接
    for (int i = r.start; i < r.end; ++i) {
        struct timeval submatch_start, submatch_end;
        gettimeofday( &submatch_start, NULL );
        // std::cout << "r size is " << r.size() << ",   rng is " << rng.state + i << std::endl;
        //cv::theRNG() = cv::RNG(rng.state + i); // force "stable" RNG seed for each processed pair
        //std::cout << "MatchPairsBody -> rng state = " << rng.state << std::endl;
        cv::theRNG() = cv::RNG(4294967295); //4294967295
        
        int from = near_pairs[i].first;
        int to = near_pairs[i].second;
        int pair_idx = from * num_images + to;
        //        std::cout << "MatchPairsBody -> from = " << from << std::endl;
        //        std::cout << "MatchPairsBody -> to = " << to << std::endl;
        //        std::cout << "MatchPairsBody -> pair_idx " << pair_idx << std::endl;
        
        MatchesInfo matches_info;
        matches_info.matches.clear();//zt
        //matcher(features[from], features[to], pairwise_matches[pair_idx]); // 先进BestOf2NearestMatcher::match ，再调用CpuMatcher match
        CV_Assert(global_img_feature[from].descriptors.type() == global_img_feature[to].descriptors.type());
        CV_Assert(global_img_feature[to].descriptors.depth() == CV_8U || global_img_feature[to].descriptors.depth() == CV_32F);
        Ptr<cv::DescriptorMatcher> Dmatcher;
        {
            Ptr<flann::IndexParams> indexParams = makePtr<flann::KDTreeIndexParams>();
            Ptr<flann::SearchParams> searchParams = makePtr<flann::SearchParams>();
            
            if (global_img_feature[to].descriptors.depth() == CV_8U) {
                indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
                searchParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
            }
            
            Dmatcher = makePtr<FlannBasedMatcher>(indexParams, searchParams);
        }
        std::vector<std::vector<DMatch> > pair_matches;
        pair_matches.clear();
        MatchesSet matches;
        
        // Find 1->2 matches
        Dmatcher->knnMatch(global_img_feature[from].descriptors, global_img_feature[to].descriptors, pair_matches, 2);
        //
        //        for(int j = 0; j < pair_matches.size(); ++j)
        //        {
        //            for(int k = 0; k < pair_matches[j].size(); ++k) {
        //                std::cout << "pair_matches trainTdx = " << pair_matches[j][k].trainIdx << std::endl;
        //                std::cout << "pair_matches queryIdx = " << pair_matches[j][k].queryIdx << std::endl;
        //                std::cout << "pair_matches distance = " << pair_matches[j][k].distance << std::endl;
        //                std::cout << "pair_matches imgIdx = " << pair_matches[j][k].imgIdx << std::endl;
        //            }
        //        }
        
        for (size_t i = 0; i < pair_matches.size(); ++i) {
            if (pair_matches[i].size() < 2)
                continue;
            const DMatch &m0 = pair_matches[i][0];
            const DMatch &m1 = pair_matches[i][1];
            if (m0.distance < (1.f - match_conf) * m1.distance) {
                matches_info.matches.push_back(m0);
                matches.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
            }
        }
        // LOG("\n1->2 matches: " << matches_info.matches.size() << endl);
        // Find 2->1 matches
        pair_matches.clear();
        Dmatcher->knnMatch(global_img_feature[to].descriptors, global_img_feature[from].descriptors, pair_matches, 2);
        for (size_t i = 0; i < pair_matches.size(); ++i) {
            if (pair_matches[i].size() < 2)
                continue;
            const DMatch &m0 = pair_matches[i][0];
            const DMatch &m1 = pair_matches[i][1];
            if (m0.distance < (1.f - match_conf) * m1.distance)
                if (matches.find(std::make_pair(m0.trainIdx, m0.queryIdx)) == matches.end())
                    matches_info.matches.push_back(DMatch(m0.trainIdx, m0.queryIdx, m0.distance));
        }
        // LOG("1->2 & 2->1 matches: " << matches_info.matches.size() << endl);
        
        
        // Check if it makes sense to find homography
        if (matches_info.matches.size() >= static_cast<size_t>(6))
        {
            ////
            // Construct point-point correspondences for homography estimation
            Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
            Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
            // std::cout << "BestOf2NearestMatcher::match-> matches_info.matches.size is " << matches_info.matches.size() << std::endl;
            //TODO 取前五百对最小距离的
            for (size_t i = 0; i < matches_info.matches.size(); ++i)
            {
                const DMatch& m = matches_info.matches[i];
                
                Point2f p = global_img_feature[from].keypoints[m.queryIdx].pt;
                p.x -= global_img_feature[from].img_size.width * 0.5f;
                p.y -= global_img_feature[from].img_size.height * 0.5f;
                src_points.at<Point2f>(0, static_cast<int>(i)) = p;
                
                p = global_img_feature[to].keypoints[m.trainIdx].pt;
                p.x -= global_img_feature[to].img_size.width * 0.5f;
                p.y -= global_img_feature[to].img_size.height * 0.5f;
                dst_points.at<Point2f>(0, static_cast<int>(i)) = p;
            }
            
            // Find pair-wise motion
            matches_info.H = findHomography(src_points, dst_points, matches_info.inliers_mask, RANSAC);
            //std::cout << "matches_info pair-wise motion H is " << matches_info.H << std::endl;
            //下面的判断语句是原始源码的,在这需要改成下面的判断并加上符合修改后的判断里面的执行语句,否则在H为空的时候直接用原始的会报错
            //        if (matches_info.H.empty() || std::abs(determinant(matches_info.H)) < std::numeric_limits<double>::epsilon())
            //        {
            //            std::cout << "matches_info.H is empty." << std::endl;
            //            continue;
            //        }
            if (!matches_info.H.empty() && std::abs(determinant(matches_info.H)) >= std::numeric_limits<double>::epsilon())
                // Find number of inliers
            {
                
                matches_info.num_inliers = 0;
                for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
                    if (matches_info.inliers_mask[i])
                        matches_info.num_inliers++;
                //std::cout << "BestOf2NearestMatcher::match-> matches_info.num_inliers is " << matches_info.num_inliers<< std::endl;
                // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic Image Stitching
                // using Invariant Features"
                matches_info.confidence = matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());
                //  std::cout << "BestOf2NearestMatcher::match-> matches_info.confidence is " << matches_info.confidence  << std::endl;
                if(matches_info.confidence > 3) //zt
                {
                    if(!strLogPath.empty())
                    {
                        output_file << "LOGINFO: two pictures matches is too close." << std::endl;
                    }
                    else
                        std::cout << "LOGINFO: two pictures matches is too close." << std::endl;
                }
                // Set zero confidence to remove matches between too close global_images, as they don't provide
                // additional information anyway. The threshold was set experimentally.
                matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;
                
                ///////zt
                // Check if we should try to refine motion
                if (matches_info.num_inliers >= 6)
                {
                    //zt add 新加判断匹配是否符合拼接条件
                    //如果当前图片匹配失败, 需要删掉当前图片以及当前图片的特征和匹配的信息,并返回
                    if(matches_info.num_inliers < inliner_num || matches_info.confidence < conf_thresh)
                    {
                        MatchError++; //当前图片与之前每一次的匹配是否满足匹配条件，
                    }
                    
                    // Construct point-point correspondences for inliers only
                    src_points.create(1, matches_info.num_inliers, CV_32FC2);
                    dst_points.create(1, matches_info.num_inliers, CV_32FC2);
                    int inlier_idx = 0;
                    for (size_t i = 0; i < matches_info.matches.size(); ++i)
                    {
                        if (!matches_info.inliers_mask[i])
                            continue;
                        
                        const DMatch& m = matches_info.matches[i];
                        
                        Point2f p = global_img_feature[from].keypoints[m.queryIdx].pt;
                        
                        p.x -= global_img_feature[from].img_size.width * 0.5f;
                        p.y -= global_img_feature[from].img_size.height * 0.5f;
                        src_points.at<Point2f>(0, inlier_idx) = p;
                        
                        p = global_img_feature[to].keypoints[m.trainIdx].pt;
                        p.x -= global_img_feature[to].img_size.width * 0.5f;
                        p.y -= global_img_feature[to].img_size.height * 0.5f;
                        dst_points.at<Point2f>(0, inlier_idx) = p;
                        
                        inlier_idx++;
                    }
                    
                    // Rerun motion estimation on inliers only
                    matches_info.H = findHomography(src_points, dst_points, RANSAC);
                    pairwise_matches[pair_idx] = matches_info; //zt
                }
                else
                {
                    MatchError++;
                }
            }
            else
            {
                MatchError++;
            }
        } else
            MatchError++;
        
        pairwise_matches[pair_idx].src_img_idx = from;
        pairwise_matches[pair_idx].dst_img_idx = to;
        
        size_t dual_pair_idx = to * num_images + from;
        // std::cout << "MatchPairsBody -> dual_pair_idx is " << dual_pair_idx << std::endl;
        pairwise_matches[dual_pair_idx] = pairwise_matches[pair_idx];
        pairwise_matches[dual_pair_idx].src_img_idx = to;
        pairwise_matches[dual_pair_idx].dst_img_idx = from;
        
        if (!pairwise_matches[pair_idx].H.empty())
            pairwise_matches[dual_pair_idx].H = pairwise_matches[pair_idx].H.inv();
        
        for (size_t j = 0; j < pairwise_matches[dual_pair_idx].matches.size(); ++j)
            std::swap(pairwise_matches[dual_pair_idx].matches[j].queryIdx,
                      pairwise_matches[dual_pair_idx].matches[j].trainIdx);
        
        Dmatcher.release(); //zt
        //zt
        if(pairwise_matches[pair_idx].src_img_idx != -1)
        {
            tmp_pairwise_matches.push_back(pairwise_matches[pair_idx]);
            tmp_pairwise_matches.push_back(pairwise_matches[dual_pair_idx]);
        }
        
        gettimeofday( &submatch_end, NULL );
        //求出两次时间的差值，单位为us
        int submatch_timeuse = 1000000 * ( submatch_end.tv_sec - submatch_start.tv_sec ) + submatch_end.tv_usec - submatch_start.tv_usec;
        //printf("submatch time: %d us\n", submatch_timeuse);
    }
    
    if(num_images == 2)  // 直接返回结果
    {
        global_pairwise_matches.push_back(pairwise_matches[0]);
        for(int i = 0; i < num_images; ++i)
            global_pairwise_matches.push_back(tmp_pairwise_matches[i]);
        global_pairwise_matches.push_back(pairwise_matches[0]);
    }
    // 因为 temp 里面的都是成对出现, 如0-1 和 1-0, 0-2 和 2-0, 1-2 和 2-1, 0-3 和 3-0 , 所以下面的直接取除temp元素,不需要再取排序
    // 下面每次insert 完可以将temp里面的erase 掉, 这里我没做这个
    else if(num_images == 3)  // 先将0-2 插进去, 再将1-2 插到end(), 再 2-0 和 2-1
    {
        global_pairwise_matches.insert(global_pairwise_matches.begin() + num_images - 1,tmp_pairwise_matches[0]);// 将0-2 插进去
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            //先将 1-2 插进去,
            if(tmp_pairwise_matches[i].src_img_idx != 0 && tmp_pairwise_matches[i].dst_img_idx == num_images - 1)
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
        }
        //再插 2-0 和 2-1
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            if (tmp_pairwise_matches[i].src_img_idx == num_images - 1)
                global_pairwise_matches.insert(global_pairwise_matches.end(), tmp_pairwise_matches[i]);
        }
        global_pairwise_matches.push_back(global_pairwise_matches[0]);
    }
    else  //num_images > 3
    {
        // global_pairwise_matches.insert(global_pairwise_matches.begin() + num_images - 1,tmp_pairwise_matches[0]);
        //查找每次新的 (0-(num_images -1)) ,(0-(num_image - 2)),(1-(num_images - 1)),...,插入到对应位置
        for (int i = 0; i < tmp_pairwise_matches.size(); i++ ){
            for(int j = 0; j < global_pairwise_matches.size(); ++j)
            {
                if(global_pairwise_matches[j].src_img_idx == tmp_pairwise_matches[i].src_img_idx && global_pairwise_matches[j].dst_img_idx == num_images - 2)
                {
                    global_pairwise_matches.insert(global_pairwise_matches.begin() + j + 1,tmp_pairwise_matches[i]);
                    tmp_pairwise_matches.erase(tmp_pairwise_matches.begin() + i);
                }
            }
        }
        //最后几个上面没有对应的位置,直接插到末尾
        for(int i = 0; i < tmp_pairwise_matches.size(); ++i)
        {
            if(tmp_pairwise_matches[i].src_img_idx != num_images - 1)
            {
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
                tmp_pairwise_matches.erase(tmp_pairwise_matches.begin() + i);
            }
        }
        //从 (num_imagse -1 )-0 , (num_imagse -1 )-1, (num_imagse -1 )-2, ... 插入末尾
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            
            if(tmp_pairwise_matches[i].src_img_idx == num_images - 1 )
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
        }
        //最后一个跟自己匹配的结果(num_images - 1)-(num_images - 1), 内容和第一个一样,直接插入
        global_pairwise_matches.insert(global_pairwise_matches.end(),pairwise_matches[0]);
    }
    gettimeofday( &match_end, NULL );
    //求出两次时间的差值，单位为us
    int match_timeuse = 1000000 * ( match_end.tv_sec - match_start.tv_sec ) + match_end.tv_usec - match_start.tv_usec;
    
    if(!strLogPath.empty())
        output_file << "LOGINFO: Matcher Feature time is " << match_timeuse << "  us."<< std::endl;
    else
    {
        printf("match time: %d us\n", match_timeuse);
    }
    
    //当前图片与之前匹配不满足条件的个数如果等于所有匹配次数，则此次匹配失败，删除保存的当前图片和特征, 返回拼接失败的结果
    //    if(MatchError == num_images - 1)
    //    {
    //        global_images.pop_back();
    //        global_input_images.pop_back();
    //        global_img_feature.pop_back();
    //        stitchflag = false;
    //
    //        if(!strLogPath.empty())
    //            output_file << "Match error = num_images - 1 ." << std::endl;
    //        else
    //            std::cout << "Match error == num_images - 1 ." << std::endl;
    //        return false;
    //    }
    
    //判断哪些是不能拼接的, 在matcher里面我已经通过设置匹配置信度和内点数量进行了判断
    vector<int> indices;
    indices.clear();
    vector <ImageFeatures> img_feature_tmp;
    img_feature_tmp.clear();
    img_feature_tmp = global_img_feature;
    vector <MatchesInfo> pairwise_matches_tmp;
    pairwise_matches_tmp.clear();
    pairwise_matches_tmp = global_pairwise_matches;
    indices = leaveBiggestComponent(img_feature_tmp, pairwise_matches_tmp, conf_thresh);
    if(indices.size() < num_images)
    {
        if(!strLogPath.empty())
        {
            output_file << "indice size < num_images." << std::endl;
            output_file.close();
        }
        else
            std::cout << "indice size < num_images." << std::endl;
        global_images.pop_back();
        global_input_images.pop_back();
        global_img_feature.pop_back();
        
        stitchflag = false;
        
        return false;
    }
    
    //寻找最大匹配
    vector<int> pair_idx;
    vector<double> pair_confidence;
    vector<int> pair_points;
    for (int i = 0; i < global_img_feature.size(); ++i) {
        pair_idx.push_back(-1);
        pair_confidence.push_back(0.0);
        pair_points.push_back(0);
    }
    int src_img_idx;
    int dst_img_idx;
    double confidence;
    int num_inliers;
    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
        src_img_idx = global_pairwise_matches[i].src_img_idx;
        dst_img_idx = global_pairwise_matches[i].dst_img_idx;
        confidence = global_pairwise_matches[i].confidence;
        num_inliers = global_pairwise_matches[i].num_inliers;
        if (pair_idx[src_img_idx] == -1) {
            pair_idx[src_img_idx] = dst_img_idx;
            pair_confidence[src_img_idx] = confidence;
            pair_points[src_img_idx] = num_inliers;
        } else if (confidence > pair_confidence[src_img_idx]) {
            pair_idx[src_img_idx] = dst_img_idx;
            pair_confidence[src_img_idx] = confidence;
            pair_points[src_img_idx] = num_inliers;
        }
    }
    
    bool bflag = false;//zt
    int ImgNum = global_img_feature.size(); //这里的ImgNum包括当前传进来的图片
    for (int i = 0; i < ImgNum; ++i) {
        if (pair_points[i] <= inliner_num) {
            bflag = true;
        }
    }
    
    if(!strLogPath.empty())
    {
        if(bflag)
        {
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            stitchflag = false;
            output_file << "LOGINFO: inliner_num < 16" << std::endl;
            output_file.close(); //zt
            return false; //zt
        }
        
    } else
    {
        if(bflag)
        {
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            std::cout << "LOGINFO: inliner_num < 16" << std::endl;
            stitchflag = false;
            return false; //zt
        }
    }
    
    
    vector<CameraParams> cameras;
    struct timeval eatimator_start, estimator_end;
    gettimeofday( &eatimator_start, NULL );
    
    if(!strLogPath.empty())
    {
        if (!(*estimator)(global_img_feature, global_pairwise_matches, cameras)){
            output_file << "LOGINFO: camera estimat failed." << std::endl; //zt
            output_file.close(); //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            stitchflag = false;
            return false;
        }
    }
    else
    {
        if (!(*estimator)(global_img_feature, global_pairwise_matches, cameras)){
            std::cout << "LOGINFO: camera estimat failed." << std::endl; //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            stitchflag = false;
            return false;
        }
    }
    gettimeofday( &estimator_end, NULL );
    //求出两次时间的差值，单位为us
    int estimator_timeuse = 1000000 * ( estimator_end.tv_sec - eatimator_start.tv_sec ) + estimator_end.tv_usec - eatimator_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "estimator time is " << estimator_timeuse  << "  us."<< std::endl;
    else
        printf("estimator_ time: %d us\n", estimator_timeuse);
    
    for (size_t i = 0; i < cameras.size(); ++i){
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
    }
    
    struct timeval adjust_start, adjust_end;
    gettimeofday( &adjust_start, NULL);
    
    if(!strLogPath.empty())
    {
        if (!(*adjuster)(global_img_feature, global_pairwise_matches, cameras)){
            output_file << "LOGERROR: camera adjuster failed." << std::endl; //zt
            output_file.close(); //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            stitchflag = false;
            return false;
        }
        // check camera parameter R
        CameraParams camera;
        for (int i = 0; i < cameras.size(); ++i) {
            camera = cameras[i];
            for (int row = 0; row < camera.R.rows; row++) {
                for (int col = 0; col < camera.R.cols; col++) {
                    if (isnan(camera.R.at<int>(row,col))) {
                        output_file << "LOGINFO: camera check failed." << std::endl; //zt
                        output_file.close(); //zt
                        global_images.pop_back();
                        global_input_images.pop_back();
                        global_img_feature.pop_back();
                        
                        stitchflag = false;
                        return false;
                    }
                }
            }
        }
    }
    else
    {
        if (!(*adjuster)(global_img_feature, global_pairwise_matches, cameras)){
            std::cout << "LOGERROR: camera adjuster failed." << std::endl; //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            stitchflag = false;
            return false;
        }
        // check camera parameter R
        CameraParams camera;
        for (int i = 0; i < cameras.size(); ++i) {
            camera = cameras[i];
            for (int row = 0; row < camera.R.rows; row++) {
                for (int col = 0; col < camera.R.cols; col++) {
                    if (isnan(camera.R.at<int>(row,col))) {
                        std::cout << "LOGINFO: camera check failed." << std::endl; //zt
                        global_images.pop_back();
                        global_input_images.pop_back();
                        global_img_feature.pop_back();
                        
                        stitchflag = false;
                        return false;
                    }
                }
            }
        }
    }
    
    gettimeofday( &adjust_end, NULL );
    //求出两次时间的差值，单位为us
    int adjust_timeuse = 1000000 * ( adjust_end.tv_sec - adjust_start.tv_sec ) + adjust_end.tv_usec - adjust_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "adjust time is " << adjust_timeuse << "  us." << std::endl;
    else
        printf("adjust time: %d us\n", adjust_timeuse);
    
    camera_params = cameras;
    global_camera_params = cameras;  //此处为了与滑窗拼接方式统一, 所以最后传 global_camera_params 给 stitch_param 函数
    
    vector<double> focals;
    focals.clear(); //zt
    for (size_t i = 0; i < cameras.size(); ++i){
        focals.push_back(cameras[i].focal);
    }
    
    sort(focals.begin(), focals.end());
    float warped_image_scale;
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;
    
    vector<Point> corners(global_input_images.size());
    vector<UMat> masks_warped(global_input_images.size());
    vector<UMat> images_warped(global_input_images.size());
    vector<Size> sizes(global_input_images.size());
    vector<UMat> masks(global_input_images.size());
    
    
    struct timeval warp_start, warp_end;
    gettimeofday( &warp_start, NULL );
    
#ifndef WARP
    // Preapre images masks
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(global_images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }
    
    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
    
    for (int i = 0; i < num_images; ++i)
    {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;
        
        //check
        Rect roi = warper->warpRoi(global_images[i].size(), K, cameras[i].R);
        
        if (roi.width > pano_max_edge || roi.height > pano_max_edge){
            if(!strLogPath.empty())
            {   output_file << "LOGERROR： roi.width or roi.height > pano_max_edge. "<< std::endl;
                output_file.close();
                global_images.pop_back();
                global_input_images.pop_back();
                global_img_feature.pop_back();
                
                stitchflag = false;
                return false;
            }
            else
            {
                std::cout << "LOGERROR： roi.width or roi.height > pano_max_edge. "<< std::endl;
                global_images.pop_back();
                global_input_images.pop_back();
                global_img_feature.pop_back();
                
                stitchflag = false;
                return false;
            }
        }
        
        corners[i] = warper->warp(global_images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();
        
        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
    }
    
    vector<UMat> images_warped_f(num_images);
    for (int i = 0; i < num_images; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);
    
    gettimeofday( &warp_end, NULL );
    //求出两次时间的差值，单位为us
    int warp_timeuse = 1000000 * ( warp_end.tv_sec - warp_start.tv_sec ) + warp_end.tv_usec - warp_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "warp time is " << warp_timeuse << "  us." << std::endl;
    else
        printf("warp time: %d us\n", warp_timeuse);
    //seam time
    struct timeval seam_start, seam_end;
    gettimeofday( &seam_start, NULL );
    
    seam_finder->find(images_warped_f, corners, masks_warped);
    gettimeofday( &seam_end, NULL );
    //求出两次时间的差值，单位为us
    int seam_timeuse = 1000000 * ( seam_end.tv_sec - seam_start.tv_sec ) + seam_end.tv_usec - seam_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "seam time is " << seam_timeuse << "  us." << std::endl;
    else
        printf("seam time: %d us\n", seam_timeuse);
    
    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    Mat input_img;
    double compose_seam_aspect = 1;
    double compose_work_aspect = 1;
    
    struct timeval compose_start, compose_end;
    gettimeofday( &compose_start, NULL );
    
    if (!is_compose_scale_set ) {
        compose_scale = min(1.0, sqrt(compose_megapix * MaxPix_ / input.size().area()));
        is_compose_scale_set = true;
    }
    
    // Compute relative scales
    compose_seam_aspect = compose_scale / seam_scale;
    compose_work_aspect = compose_scale / work_scale;
    
    // Update warped image scale
    warped_image_scale *= static_cast<float>(compose_work_aspect);
    warper = warper_creator->create(warped_image_scale);
    
    // Update corners and sizes
    for (int i = 0; i < num_images; ++i)
    {
        // Update intrinsics
        cameras[i].focal *= compose_work_aspect;
        cameras[i].ppx *= compose_work_aspect;
        cameras[i].ppy *= compose_work_aspect;
        
        // Update corner and size
        Size sz = input.size();
        if (std::abs(compose_scale - 1) > 1e-1)
        {
            sz.width = cvRound(input.size().width * compose_scale);
            sz.height = cvRound(input.size().height * compose_scale);
        }
        
        Mat K;
        cameras[i].K().convertTo(K, CV_32F);
        Rect roi = warper->warpRoi(sz, K, cameras[i].R);
        corners[i] = roi.tl();
        sizes[i] = roi.size();
    }
    
    for (int img_idx = 0; img_idx < num_images; ++img_idx)
    {
        // Read image and resize it if necessary
        input_img = global_input_images[img_idx].clone();
        
        if (abs(compose_scale - 1) > 1e-1)
            cv::resize(input_img, img, Size(), compose_scale, compose_scale, INTER_LINEAR_EXACT);
        else
            img = input_img.clone();
        
        input_img.release();
        Size img_size = img.size();
        
        Mat K;
        cameras[img_idx].K().convertTo(K, CV_32F);
        
        // Warp the current image
        warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);
        
        // Warp the current image mask
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);
        
        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();
        
        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
        mask_warped = seam_mask & mask_warped;
        
        if (!blender && !timelapse)
        {
            blender = Blender::createDefault(blend_type, try_cuda);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, try_cuda);
            else if (blend_type == Blender::MULTI_BAND)
            {
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
            }
            blender->prepare(corners, sizes);
        }
        
        // Blend the current image
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }
    
    Mat result, result_mask;
    blender->blend(result, result_mask);
    
    gettimeofday( &compose_end, NULL );
    //求出两次时间的差值，单位为us
    int compose_timeuse = 1000000 * ( compose_end.tv_sec - compose_start.tv_sec ) + compose_end.tv_usec - compose_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "compose time is " << compose_timeuse << "  us." << std::endl;
    else
        printf("compose time: %d us\n", compose_timeuse);
    
    result.copyTo(dst);
    result_size = result.size();
    //zt log camera parameters
    if(!strLogPath.empty())
    {
        for (size_t tt = 0; tt < cameras.size(); ++tt)
        {
            output_file << " LOGINFO: camera information: " << std::endl;  //zt
            output_file << "    aspect is " << cameras[tt].aspect << endl;
            output_file << "    focal is " << cameras[tt].focal << endl;
            output_file << "    Principal point X is " << cameras[tt].ppx  << ", Principal point Y is " << cameras[tt].ppy << std::endl;
            output_file << "    R matrix is " << cameras[tt].R << std::endl;
            output_file << "    t matrix is " << cameras[tt].t << std::endl;
        }
        // zt log pano size
        output_file << " LOGINFO: pano size is " << result_size << std::endl; //zt
    }
    
    ///MAP compute
    //zt 保存全局 corners
    global_corners = corners;
    
    Point left_top;
    Point right_bottom;
    int stitch_x = 0;
    int stitch_y = 0;
    Size blend_image_size;
    vector<Point> corner_points;
    corner_points.clear();
    Point map_idx;
    vector<int> left_top_x;
    left_top_x.clear();
    vector<int> left_top_y;
    left_top_y.clear();
    //sum_corner_points.clear(); //zt 20190404 如果没有调用save_param 需clear
    //convert left_top points and size into corner points
    for (int img_idx = 0; img_idx < InputImgNums; ++img_idx) {
        // calculate blend global_images 4 corner points
        blend_image_size = sizes[img_idx];
        left_top = global_corners[img_idx];
        if (left_top.x < stitch_x) {
            stitch_x = left_top.x;
        }
        if (left_top.y < stitch_y) {
            stitch_y = left_top.y;
        }
        right_bottom.x = left_top.x + blend_image_size.width;
        right_bottom.y = left_top.y + blend_image_size.height;
        //std::cout << "left_top: " << left_top << std::endl;
        //std::cout << "right_bottom: " << right_bottom << std::endl;
        corner_points.push_back(left_top);
        corner_points.push_back(right_bottom);
        sum_corner_points.push_back(corner_points);
        corner_points.clear();
    }
    if (!strLogPath.empty())
        output_file << " LOGINFO: stitch_left_topx is " << stitch_x << " ,stitch_left_topy is " << stitch_y
        << std::endl;
    for (int i = 0; i < InputImgNums; ++i) {
        sum_corner_points[i][0].x = sum_corner_points[i][0].x - stitch_x;
        sum_corner_points[i][0].y = sum_corner_points[i][0].y - stitch_y;
        sum_corner_points[i][1].x = sum_corner_points[i][1].x - stitch_x;
        sum_corner_points[i][1].y = sum_corner_points[i][1].y - stitch_y;
        Point tmp_lt_pnt = sum_corner_points[i][0];
        Point tmp_rb_pnt = sum_corner_points[i][1];
        if (!strLogPath.empty())
            output_file << " LOGINFO: cornetPoints Info " << i << " image corner points in pano: " << tmp_lt_pnt << ", "
            << tmp_rb_pnt << std::endl;
    }
    //image map index
    for (int img_idx = 0; img_idx < InputImgNums; ++img_idx) {
        left_top_x.push_back(global_corners[img_idx].x);
        left_top_y.push_back(global_corners[img_idx].y);
    }
    vector<long unsigned int> sorted_x_idx;
    sorted_x_idx.clear();
    vector<long unsigned int> sorted_y_idx;
    sorted_y_idx.clear();
    sorted_x_idx = sort_indexes_e(left_top_x);
    sorted_y_idx = sort_indexes_e(left_top_y);
    int x;
    int y;
    int w_diff;
    int h_diff;
    vector<int> vec_w_diff;
    vector<int> vec_h_diff;
    vector<int> corner_dist;
    vector<long unsigned int> sorted_dist_idx;
    vec_w_diff.clear();
    vec_h_diff.clear();
    corner_dist.clear();
    sorted_dist_idx.clear();
    int cloest_idx;
    Point cloest_point;
    int step;
    int step_direction; // 0: x; 1: y
    for (int img_idx = 0; img_idx < InputImgNums; ++img_idx) {
        if (img_idx == 0) {
            map_idx.x = 0;
            map_idx.y = 0;
        } else {
            x = left_top_x[img_idx];
            y = left_top_y[img_idx];
            // std::cout << "image_map_index " << image_map_index.size() << std::endl;
            for (int i = 0; i < image_map_index.size(); ++i) {
                w_diff = x - left_top_x[i];
                h_diff = y - left_top_y[i];
                vec_w_diff.push_back(w_diff);
                vec_h_diff.push_back(h_diff);
                corner_dist.push_back(abs(w_diff) + abs(h_diff));
            }
            sorted_dist_idx = sort_indexes_e(corner_dist);
            cloest_idx = sorted_dist_idx[0];
            cloest_point = image_map_index[cloest_idx];
            
            w_diff = vec_w_diff[cloest_idx];
            h_diff = vec_h_diff[cloest_idx];
            //            if(!strLogPath.empty())
            //            {
            //                output_file << " LOGINFO: cloest_idx: " << cloest_idx << std::endl;
            //                output_file << " LOGINFO: cloest_point: " << cloest_point << std::endl;
            //
            //                output_file << " LOGINFO: w_diff: " << w_diff << std::endl;
            //                output_file << " LOGINFO: h_diff: " << h_diff << std::endl;
            //            }
            
            //            std::cout << " LOGINFO: cloest_idx: " << cloest_idx << std::endl;
            //            std::cout << " LOGINFO: cloest_point: " << cloest_point << std::endl;
            //
            //            std::cout << " LOGINFO: w_diff: " << w_diff << std::endl;
            //            std::cout << " LOGINFO: h_diff: " << h_diff << std::endl;
            if (abs(w_diff) >= abs(h_diff)) {
                step_direction = 0;
                if (w_diff >= 0) {
                    step = 1;
                } else {
                    step = -1;
                }
                map_idx.x = cloest_point.x + step;
                map_idx.y = cloest_point.y;
            } else {
                step_direction = 1;
                if (h_diff >= 0) {
                    step = 1;
                } else {
                    step = -1;
                }
                map_idx.y = cloest_point.y + step;
                map_idx.x = cloest_point.x;
            }
            for (int i = 0; i < image_map_index.size(); ++i) {
                Point tmp_point = image_map_index[i];
                if (step_direction == 0 && tmp_point.y == map_idx.y && tmp_point.x >= map_idx.x) {
                    ++image_map_index[i].x;
                } else if (step_direction == 1 && tmp_point.x == map_idx.x && tmp_point.y >= map_idx.y) {
                    ++image_map_index[i].y;
                }
            }
        }
        image_map_index.push_back(map_idx);
        vec_w_diff.clear();
        vec_h_diff.clear();
        corner_dist.clear();
        sorted_dist_idx.clear();
    }
    //image map index reset
    vector<int> map_idx_x;
    map_idx_x.clear();
    vector<int> map_idx_y;
    map_idx_y.clear();
    vector<int> map_idx_first_row;
    map_idx_first_row.clear();
    for (int i = 0; i < InputImgNums; ++i) {
        map_idx_x.push_back(image_map_index[i].x);
        map_idx_y.push_back(image_map_index[i].y);
        if (image_map_index[i].y == 0) {
            map_idx_first_row.push_back(image_map_index[i].x);
        }
    }
    //convert negative index to postive
    int min_x = *min_element(map_idx_x.begin(), map_idx_x.end());
    int min_y = *min_element(map_idx_y.begin(), map_idx_y.end());
    if (min_x < 0) {
        for (int i = 0; i < map_idx_x.size(); ++i) {
            map_idx_x[i] = map_idx_x[i] + abs(min_x);
        }
        for (int i = 0; i < map_idx_first_row.size(); ++i) {
            map_idx_first_row[i] = map_idx_first_row[i] + abs(min_x);
        }
    }
    if (min_y < 0) {
        for (int i = 0; i < InputImgNums; ++i) {
            map_idx_y[i] = map_idx_y[i] + abs(min_y);
        }
    }
    //find the miss first row col after positive convert
    map_idx_first_row.clear();
    for (int i = 0; i < InputImgNums; ++i) {
        if (map_idx_y[i] == 0) {
            map_idx_first_row.push_back(map_idx_x[i]);
        }
    }
    //find the col index of missing point in first row
    vector<int>::iterator ret;
    vector<int> miss_x;
    miss_x.clear();
    for (int i = 0; i < InputImgNums; ++i) {
        if (map_idx_y[i] > 0) {
            ret = find(map_idx_first_row.begin(), map_idx_first_row.end(), map_idx_x[i]);
            if (ret == map_idx_first_row.end()) {
                miss_x.push_back(map_idx_x[i]);
            }
        }
    }
    //pop up the missing column until touch the first row
    vector<vector<int>> miss_columns;
    miss_columns.clear();
    vector<vector<int>> tmp_idxes;
    tmp_idxes.clear();
    vector<int> col;
    col.clear();
    for (int i = 0; i < miss_x.size(); ++i) {
        miss_columns.push_back(col);
        tmp_idxes.push_back(col);
    }
    int col_idx;
    for (int i = 0; i < InputImgNums; ++i) {
        if (map_idx_y[i] > 0) {
            ret = find(miss_x.begin(), miss_x.end(), map_idx_x[i]);
            if (ret != miss_x.end()) {
                col_idx = distance(miss_x.begin(), ret);
                miss_columns[col_idx].push_back(map_idx_y[i]);
                tmp_idxes[col_idx].push_back(i);
            }
        }
    }
    for (int i = 0; i < miss_columns.size(); ++i) {
        col = miss_columns[i];
        int min_y_in_col = *min_element(col.begin(), col.end());
        for (int j = 0; j < col.size(); ++j) {
            map_idx_y[tmp_idxes[i][j]] = map_idx_y[tmp_idxes[i][j]] - min_y_in_col;
        }
    }
    //connect the break point in col
    col.clear();
    int n_col = *max_element(map_idx_x.begin(), map_idx_x.end()) + 1;
    vector<int> idx_in_col;
    idx_in_col.clear();
    int src_idx;
    int prev_idx;
    for (int i = 0; i <= n_col; ++i) {
        for (int j = 0; j < map_idx_y.size(); ++j) {
            if (map_idx_x[j] == i) {
                col.push_back(map_idx_y[j]);
                idx_in_col.push_back(j);
            }
        }
        vector<long unsigned int> sorted_col_idx = sort_indexes_e(col);
        for (int j = 1; j < sorted_col_idx.size(); ++j) {
            src_idx = idx_in_col[sorted_col_idx[j]];
            prev_idx = idx_in_col[sorted_col_idx[j - 1]];
            if (map_idx_y[src_idx] != map_idx_y[prev_idx] + 1) {
                map_idx_y[src_idx] = map_idx_y[prev_idx] + 1;
            }
        }
        col.clear();
        idx_in_col.clear();
        sorted_col_idx.clear();
    }
    //connect the break point in row
    vector<int> row;
    row.clear();
    int n_row = *max_element(map_idx_y.begin(), map_idx_y.end()) + 1;
    vector<int> idx_in_row;
    idx_in_row.clear();
    src_idx = 0;
    prev_idx = 0;
    for (int i = 0; i <= n_row; ++i) {
        for (int j = 0; j < map_idx_x.size(); ++j) {
            if (map_idx_y[j] == i) {
                row.push_back(map_idx_x[j]);
                idx_in_row.push_back(j);
            }
        }
        vector<long unsigned int> sorted_row_idx = sort_indexes_e(row);
        for (int j = 1; j < sorted_row_idx.size(); ++j) {
            src_idx = idx_in_row[sorted_row_idx[j]];
            prev_idx = idx_in_row[sorted_row_idx[j - 1]];
            if (map_idx_x[src_idx] != map_idx_x[prev_idx] + 1) {
                map_idx_x[src_idx] = map_idx_x[prev_idx] + 1;
            }
        }
        row.clear();
        idx_in_row.clear();
        sorted_row_idx.clear();
    }
    //find the col index of missing point in first row
    miss_x.clear();
    for (int i = 0; i < InputImgNums; ++i) {
        if (map_idx_y[i] > 0) {
            ret = find(map_idx_first_row.begin(), map_idx_first_row.end(), map_idx_x[i]);
            if (ret == map_idx_first_row.end()) {
                miss_x.push_back(map_idx_x[i]);
            }
        }
    }
    //pop up the missing column until touch the first row
    miss_columns.clear();
    tmp_idxes.clear();
    col.clear();
    for (int i = 0; i < miss_x.size(); ++i) {
        miss_columns.push_back(col);
        tmp_idxes.push_back(col);
    }
    col_idx = 0;
    for (int i = 0; i < InputImgNums; ++i) {
        if (map_idx_y[i] > 0) {
            ret = find(miss_x.begin(), miss_x.end(), map_idx_x[i]);
            if (ret != miss_x.end()) {
                col_idx = distance(miss_x.begin(), ret);
                miss_columns[col_idx].push_back(map_idx_y[i]);
                tmp_idxes[col_idx].push_back(i);
            }
        }
    }
    for (int i = 0; i < miss_columns.size(); ++i) {
        col = miss_columns[i];
        int min_y_in_col = *min_element(col.begin(), col.end());
        for (int j = 0; j < col.size(); ++j) {
            map_idx_y[tmp_idxes[i][j]] = map_idx_y[tmp_idxes[i][j]] - min_y_in_col;
        }
    }
    
    //restore map index, start from (1, 1)
    for (int i = 0; i < InputImgNums; ++i) {
        image_map_index[i].x = map_idx_x[i] + 1;
        image_map_index[i].y = map_idx_y[i] + 1;
        // std::cout << "map_idx_x = " << map_idx_x[i] << ",  map_idx_y = " << map_idx_y[i] << std::endl;
    }
    
    // Release unused memory
    //global_images.clear();
    //global_input_images.clear();
    images_warped.clear();
    images_warped_f.clear();
    masks.clear();
    cameras.clear();
    corners.clear();
    sizes.clear();
    
    img_warped_s.release();
    dilated_mask.release();
    seam_mask.release();
    mask_warped.release();
    result.release();
    result_mask.release();
    
    corner_points.clear();
    left_top_x.clear();
    left_top_y.clear();
    sorted_x_idx.clear();
    sorted_y_idx.clear();
    map_idx_x.clear();
    map_idx_y.clear();
    map_idx_first_row.clear();
    miss_x.clear();
    miss_columns.clear();
    tmp_idxes.clear();
    col.clear();
#endif
    
    gettimeofday( &stitch_end, NULL );
    //求出两次时间的差值，单位为us
    int stitch_timeuse = 1000000 * ( stitch_end.tv_sec - stitch_start.tv_sec ) + stitch_end.tv_usec - stitch_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "total time is " << stitch_timeuse << "  us." << std::endl;
    else
        printf("total time: %d us\n", stitch_timeuse);
    
    if(!strLogPath.empty())
    {
        output_file.close(); //zt
    }
    InputImgNums ++;
    stitchflag = true;
    return true;
}
#endif

//新的拼接逻辑，一次一张, 超过5张就替换最前面的那张, 还原MAP
#ifdef NEWMETHOD_MAP
bool stitching(std::vector <cv::Mat> &imglist, Mat &dst, std::string & strLogPath){
    
    struct timeval stitch_start, stitch_end;
    gettimeofday( &stitch_start, NULL );
    
    ofstream output_file;
    if(!strLogPath.empty())
    {
        output_file.open(strLogPath, ios::out); //zt
        if(imglist.size() == 0)
        {
            output_file << "LOGERROR: input images = 0." << std::endl;
            output_file.close();
            return false;
        }
        if(imglist.size() > 1)
        {
            output_file << "LOGERROR: input images > 1, please input just one img." << std::endl;
            output_file.close();
            return false;
        }
        
        if(imglist[0].empty())
        {
            output_file << "LOGERROR: input img is empty." << std::endl;
            output_file.close();
            return false;
        }
    }
    else
    {
        if(imglist.size() == 0)
        {
            printf("input images = 0.");
            return false;
        }
        if(imglist.size() > 1)
        {
            printf("input images > 1, please input just one img");
            return false;
        }
        
        if(imglist[0].empty())
        {
            printf("input img is empty.");
            return false;
        }
    }
    
    if (!is_input_scale_set) {
        if (work_pixel == 0){
            input_scale = min(1.0, sqrt(input_megapix * MaxPix_ / imglist[0].size().area()));
            is_input_scale_set = true;
        }
        else{
            if (min(imglist[0].rows, imglist[0].cols)<= work_pixel){
                input_scale = 1;
                is_input_scale_set = true;
            }
            else{
                input_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                is_input_scale_set = true;
            }
        }
    }
    
    //每次只能送一张进行stitching
    if(InputImgNums == 0)
        InputImgNums = imglist.size(); // 当前图片就1张
    
    if(InputImgNums == 1)  //first image , return current img as pino.jpg
    {
        dst = imglist[0];
        InputImgNums++;
        ImageFeatures featureTemp;
        Mat img;
        resize(imglist[0], input, Size(), input_scale, input_scale, INTER_NEAREST);
        
        if (work_megapix < 0)
        {
            img = input.clone();
            work_scale = 1;
            is_work_scale_set = true;
        }
        else {
            if (!is_work_scale_set) {
                if (work_pixel == 0){
                    work_scale = min(1.0, sqrt(work_megapix * MaxPix_ / input.size().area()));
                    is_work_scale_set = true;
                }
                else{
                    if (min(imglist[0].rows, imglist[0].cols)<= work_pixel){
                        work_scale = 1;
                        is_work_scale_set = true;
                    }
                    else{
                        work_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                        is_work_scale_set = true;
                    }
                }
            }
            resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
        }
        if (!is_seam_scale_set)
        {
            seam_scale = min(1.0, sqrt(seam_megapix * MaxPix_ / input.size().area()));
            seam_work_aspect = seam_scale / work_scale;
            is_seam_scale_set = true;
        }
        //find feature
        struct timeval first_find_start, first_find_end;
        gettimeofday( &first_find_start, NULL );
        (*finder)(img, featureTemp);
        gettimeofday( &first_find_end, NULL );
        //求出两次时间的差值，单位为us
        int firstfindtimeuse = 1000000 * ( first_find_end.tv_sec - first_find_start.tv_sec ) + first_find_end.tv_usec - first_find_start.tv_usec;
        if(!strLogPath.empty())
            output_file << "first img find time is " << firstfindtimeuse << "  us."<< std::endl;
        else
            printf("first img find time: %d us\n", firstfindtimeuse);
        
        featureTemp.img_idx = 0;
        global_img_feature.push_back(featureTemp); //debug
        
        finder->collectGarbage();
        
        if (try_compose == true)
            resize(input, img, Size(), seam_scale, seam_scale, INTER_NEAREST);
        else {
            if (!is_compose_scale_set) {
                if (work_pixel == 0) {
                    compose_scale = min(1.0, sqrt(compose_scale * MaxPix_ / input.size().area()));
                    is_compose_scale_set = true;
                } else {
                    if (min(imglist[0].rows, imglist[0].cols) <= work_pixel) {
                        compose_scale = 1;
                        is_compose_scale_set = true;
                    } else {
                        compose_scale = work_pixel * 1. / min(imglist[0].rows, imglist[0].cols);
                        is_compose_scale_set = true;
                    }
                }
            }
            resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
        }
        global_input_images.push_back(input.clone());//debug
        global_images.push_back(img.clone());  //debug
        if(!strLogPath.empty())
            output_file.close();
        return true;
    }
    
    std::cout << "global_input_images size is " << global_input_images.size() << std::endl;
    std::cout << "global_images size is " << global_images.size() << std::endl;
    std::cout << "global_img_feature size is " << global_img_feature.size() << std::endl;
    
    if(!stitchflag)  //前面拼接失败,需要删除前面最近一次的匹配信息
    {
        global_pairwise_matches.pop_back();
        
        if(InputImgNums == 2)
            global_pairwise_matches.clear();
        else if(InputImgNums <= MaxImgNums_ && InputImgNums > 2)
        {
            int srcMaxIdx = 0, dstMaxIdx = 0;
            global_pairwise_matches.pop_back();
            for(int i= 0; i< global_pairwise_matches.size(); ++i)
            {
                if(global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                    srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                if(global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                    dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
            }
            
            for(int i = 0; i < global_pairwise_matches.size(); ++i)
            {
                if(global_pairwise_matches[i].src_img_idx == srcMaxIdx || global_pairwise_matches[i].dst_img_idx == dstMaxIdx)
                {
                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                    i = -1;
                }
            }
            
            //            for(int i = 0; i < global_pairwise_matches.size(); ++i)
            //            {
            //                if(global_pairwise_matches[i].src_img_idx == InputImgNums - 1 || global_pairwise_matches[i].dst_img_idx == InputImgNums - 1)
            //                {
            //                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
            //                    i = -1;
            //                }
            //            }
        } else  //大于最大匹配数
        {
            
            for(int i = 0; i < global_pairwise_matches.size(); ++i)
            {
                if(global_pairwise_matches[i].src_img_idx == MaxImgNums_ - 1 || global_pairwise_matches[i].dst_img_idx == MaxImgNums_ - 1)
                {
                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                    i = -1;
                }
            }
        }
        
    }
    // 当前图片数量 > MaxImgNums_ 并且前面拼接成功,删除保存向量的首个元素, 后面的元素坐标向前退1, 删除当前最大匹配图片的首张图片和特征
    if(stitchflag && InputImgNums > MaxImgNums_)
    {
        //erase 图片和特征
        global_img_feature.erase(global_img_feature.begin());
        global_images.erase(global_images.begin());
        global_input_images.erase(global_input_images.begin());
        //删除匹配信息
        global_pairwise_matches.erase(global_pairwise_matches.begin());  //删除第一个元素
        int matchSize = global_pairwise_matches.size();
        //删除保存向量的第一张图片的匹配信息
        for(int i = 0; i < matchSize; ++i)
        {
            if(global_pairwise_matches[i].src_img_idx == 0 || global_pairwise_matches[i].dst_img_idx == 0)
            {
                global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                i = -1; //执行 ++i将 i 重置为0
            }
        }
        int matchSizeModi = global_pairwise_matches.size();
        //后面的元素坐标向前退1
        for(int i = 0; i < matchSizeModi; ++i)
        {
            if(global_pairwise_matches[i].src_img_idx != -1 )//|| global_pairwise_matches[i].dst_img_idx != -1)
            {
                global_pairwise_matches[i].src_img_idx = global_pairwise_matches[i].src_img_idx - 1;
                global_pairwise_matches[i].dst_img_idx = global_pairwise_matches[i].dst_img_idx - 1;
            }
        }
    }
    
    Mat img;
    resize(imglist[0], input, Size(), input_scale, input_scale, INTER_NEAREST);
    
    if (work_megapix < 0)
    {
        img = input.clone();
        work_scale = 1;
        is_work_scale_set = true;
    }
    else {
        if (!is_work_scale_set) {
            if (work_pixel == 0){
                work_scale = min(1.0, sqrt(work_megapix * MaxPix_ / input.size().area()));
                is_work_scale_set = true;
            }
            else{
                if (min(imglist[0].rows, imglist[0].cols) <= work_pixel){
                    work_scale = 1;
                    is_work_scale_set = true;
                }
                else{
                    work_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                    is_work_scale_set = true;
                }
            }
        }
        resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
    }
    if (!is_seam_scale_set)
    {
        seam_scale = min(1.0, sqrt(seam_megapix * MaxPix_ / input.size().area()));
        seam_work_aspect = seam_scale / work_scale;
        is_seam_scale_set = true;
    }
    
    ImageFeatures feature;
    //find feature
    struct timeval find_start, find_end;
    gettimeofday( &find_start, NULL );
    (*finder)(img, feature);
    
    gettimeofday( &find_end, NULL );
    //求出两次时间的差值，单位为us
    int find_timeuse = 1000000 * ( find_end.tv_sec - find_start.tv_sec ) + find_end.tv_usec - find_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "LOGINFO: Finder time is " << find_timeuse << " us."<< std::endl;
    else
        printf("Finder time: %d us\n", find_timeuse);
    
    feature.img_idx = InputImgNums - 1;
    
    if (try_compose == true)
        resize(input, img, Size(), seam_scale, seam_scale, INTER_NEAREST);
    else {
        if (!is_compose_scale_set) {
            if (work_pixel == 0) {
                compose_scale = min(1.0, sqrt(compose_scale * MaxPix_ / input.size().area()));
                is_compose_scale_set = true;
            } else {
                if (min(imglist[0].rows, imglist[0].cols) <= work_pixel) {
                    compose_scale = 1;
                    is_compose_scale_set = true;
                } else {
                    compose_scale = work_pixel * 1. / min(imglist[0].rows, imglist[0].cols);
                    is_compose_scale_set = true;
                }
            }
        }
        resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
    }
    
    global_input_images.push_back(input.clone());
    global_images.push_back(img.clone());
    global_img_feature.push_back(feature);
    finder->collectGarbage();
    
    // 将matcher 分拆 ///////////////////////////////////////
    vector<MatchesInfo> pairwise_matches;
    pairwise_matches.clear();
    
    struct timeval match_start, match_end;
    gettimeofday( &match_start, NULL );
    
    int num_images = global_img_feature.size();
    
    // 创建匹配 pair, 每次新进来的要clear
    std::vector<std::pair<int,int> > near_pairs;
    near_pairs.clear();
    
    for(int idx = 0; idx < global_img_feature.size() - 1; ++idx)
        if (global_img_feature[global_img_feature.size() - 1].keypoints.size() > 0 && global_img_feature[0].keypoints.size() > 0)
            near_pairs.push_back(std::make_pair(idx,global_img_feature.size() - 1));
    
    pairwise_matches.resize(num_images * num_images);
    
    cv::RNG rng = cv::theRNG(); // save entry rng state
    Range r = Range(0, static_cast<int>(near_pairs.size()));
    
    vector <MatchesInfo> tmp_pairwise_matches;
    tmp_pairwise_matches.clear();// zt
    int MatchError = 0; //定义新进来的图片与之前图片匹配的错误数量，根据此来作为后续判断当前图片是否可以与之前图片拼接
    for (int i = r.start; i < r.end; ++i) {
        struct timeval submatch_start, submatch_end;
        gettimeofday( &submatch_start, NULL );
        // std::cout << "r size is " << r.size() << ",   rng is " << rng.state + i << std::endl;
        //cv::theRNG() = cv::RNG(rng.state + i); // force "stable" RNG seed for each processed pair
        //std::cout << "MatchPairsBody -> rng state = " << rng.state << std::endl;
        cv::theRNG() = cv::RNG(4294967295); //4294967295
        
        int from = near_pairs[i].first;
        int to = near_pairs[i].second;
        int pair_idx = from * num_images + to;
        //        std::cout << "MatchPairsBody -> from = " << from << std::endl;
        //        std::cout << "MatchPairsBody -> to = " << to << std::endl;
        //        std::cout << "MatchPairsBody -> pair_idx " << pair_idx << std::endl;
        
        MatchesInfo matches_info;
        matches_info.matches.clear();//zt
        //matcher(features[from], features[to], pairwise_matches[pair_idx]); // 先进BestOf2NearestMatcher::match ，再调用CpuMatcher match
        CV_Assert(global_img_feature[from].descriptors.type() == global_img_feature[to].descriptors.type());
        CV_Assert(global_img_feature[to].descriptors.depth() == CV_8U || global_img_feature[to].descriptors.depth() == CV_32F);
        Ptr<cv::DescriptorMatcher> Dmatcher;
        {
            Ptr<flann::IndexParams> indexParams = makePtr<flann::KDTreeIndexParams>();
            Ptr<flann::SearchParams> searchParams = makePtr<flann::SearchParams>();
            
            if (global_img_feature[to].descriptors.depth() == CV_8U) {
                indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
                searchParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
            }
            
            Dmatcher = makePtr<FlannBasedMatcher>(indexParams, searchParams);
        }
        std::vector<std::vector<DMatch> > pair_matches;
        pair_matches.clear();
        MatchesSet matches;
        
        // Find 1->2 matches
        Dmatcher->knnMatch(global_img_feature[from].descriptors, global_img_feature[to].descriptors, pair_matches, 2);
        //
        //        for(int j = 0; j < pair_matches.size(); ++j)
        //        {
        //            for(int k = 0; k < pair_matches[j].size(); ++k) {
        //                std::cout << "pair_matches trainTdx = " << pair_matches[j][k].trainIdx << std::endl;
        //                std::cout << "pair_matches queryIdx = " << pair_matches[j][k].queryIdx << std::endl;
        //                std::cout << "pair_matches distance = " << pair_matches[j][k].distance << std::endl;
        //                std::cout << "pair_matches imgIdx = " << pair_matches[j][k].imgIdx << std::endl;
        //            }
        //        }
        
        for (size_t i = 0; i < pair_matches.size(); ++i) {
            if (pair_matches[i].size() < 2)
                continue;
            const DMatch &m0 = pair_matches[i][0];
            const DMatch &m1 = pair_matches[i][1];
            if (m0.distance < (1.f - match_conf) * m1.distance) {
                matches_info.matches.push_back(m0);
                matches.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
            }
        }
        // LOG("\n1->2 matches: " << matches_info.matches.size() << endl);
        // Find 2->1 matches
        pair_matches.clear();
        Dmatcher->knnMatch(global_img_feature[to].descriptors, global_img_feature[from].descriptors, pair_matches, 2);
        for (size_t i = 0; i < pair_matches.size(); ++i) {
            if (pair_matches[i].size() < 2)
                continue;
            const DMatch &m0 = pair_matches[i][0];
            const DMatch &m1 = pair_matches[i][1];
            if (m0.distance < (1.f - match_conf) * m1.distance)
                if (matches.find(std::make_pair(m0.trainIdx, m0.queryIdx)) == matches.end())
                    matches_info.matches.push_back(DMatch(m0.trainIdx, m0.queryIdx, m0.distance));
        }
        // LOG("1->2 & 2->1 matches: " << matches_info.matches.size() << endl);
        
        
        // Check if it makes sense to find homography
        if (matches_info.matches.size() >= static_cast<size_t>(6))
        {
            ////
            // Construct point-point correspondences for homography estimation
            Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
            Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
            // std::cout << "BestOf2NearestMatcher::match-> matches_info.matches.size is " << matches_info.matches.size() << std::endl;
            //TODO 取前五百对最小距离的
            for (size_t i = 0; i < matches_info.matches.size(); ++i)
            {
                const DMatch& m = matches_info.matches[i];
                
                Point2f p = global_img_feature[from].keypoints[m.queryIdx].pt;
                p.x -= global_img_feature[from].img_size.width * 0.5f;
                p.y -= global_img_feature[from].img_size.height * 0.5f;
                src_points.at<Point2f>(0, static_cast<int>(i)) = p;
                
                p = global_img_feature[to].keypoints[m.trainIdx].pt;
                p.x -= global_img_feature[to].img_size.width * 0.5f;
                p.y -= global_img_feature[to].img_size.height * 0.5f;
                dst_points.at<Point2f>(0, static_cast<int>(i)) = p;
            }
            
            // Find pair-wise motion
            matches_info.H = findHomography(src_points, dst_points, matches_info.inliers_mask, RANSAC);
            //std::cout << "matches_info pair-wise motion H is " << matches_info.H << std::endl;
            //下面的判断语句是原始源码的,在这需要改成下面的判断并加上符合修改后的判断里面的执行语句,否则在H为空的时候直接用原始的会报错
            //        if (matches_info.H.empty() || std::abs(determinant(matches_info.H)) < std::numeric_limits<double>::epsilon())
            //        {
            //            std::cout << "matches_info.H is empty." << std::endl;
            //            continue;
            //        }
            if (!matches_info.H.empty() && std::abs(determinant(matches_info.H)) >= std::numeric_limits<double>::epsilon())
                // Find number of inliers
            {
                
                matches_info.num_inliers = 0;
                for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
                    if (matches_info.inliers_mask[i])
                        matches_info.num_inliers++;
                //std::cout << "BestOf2NearestMatcher::match-> matches_info.num_inliers is " << matches_info.num_inliers<< std::endl;
                // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic Image Stitching
                // using Invariant Features"
                matches_info.confidence = matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());
                //  std::cout << "BestOf2NearestMatcher::match-> matches_info.confidence is " << matches_info.confidence  << std::endl;
                // Set zero confidence to remove matches between too close global_images, as they don't provide
                // additional information anyway. The threshold was set experimentally.
                if(matches_info.confidence > 3) //zt
                {
                    if(!strLogPath.empty())
                    {
                        output_file << "LOGINFO: two pictures matches is too close." << std::endl;
                    }
                    else
                        std::cout << "LOGINFO: two pictures matches is too close." << std::endl;
                }
                matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;
                
                ///////zt
                // Check if we should try to refine motion
                if (matches_info.num_inliers >= 6)
                {
                    //zt add 新加判断匹配是否符合拼接条件
                    //如果当前图片匹配失败, 需要删掉当前图片以及当前图片的特征和匹配的信息,并返回
                    if(matches_info.num_inliers < inliner_num || matches_info.confidence < conf_thresh)
                    {
                        MatchError++; //当前图片与之前每一次的匹配是否满足匹配条件，
                    }
                    
                    // Construct point-point correspondences for inliers only
                    src_points.create(1, matches_info.num_inliers, CV_32FC2);
                    dst_points.create(1, matches_info.num_inliers, CV_32FC2);
                    int inlier_idx = 0;
                    for (size_t i = 0; i < matches_info.matches.size(); ++i)
                    {
                        if (!matches_info.inliers_mask[i])
                            continue;
                        
                        const DMatch& m = matches_info.matches[i];
                        
                        Point2f p = global_img_feature[from].keypoints[m.queryIdx].pt;
                        
                        p.x -= global_img_feature[from].img_size.width * 0.5f;
                        p.y -= global_img_feature[from].img_size.height * 0.5f;
                        src_points.at<Point2f>(0, inlier_idx) = p;
                        
                        p = global_img_feature[to].keypoints[m.trainIdx].pt;
                        p.x -= global_img_feature[to].img_size.width * 0.5f;
                        p.y -= global_img_feature[to].img_size.height * 0.5f;
                        dst_points.at<Point2f>(0, inlier_idx) = p;
                        
                        inlier_idx++;
                    }
                    
                    // Rerun motion estimation on inliers only
                    matches_info.H = findHomography(src_points, dst_points, RANSAC);
                    pairwise_matches[pair_idx] = matches_info; //zt
                }
                else
                {
                    MatchError++;
                }
            }
            else
            {
                MatchError++;
            }
        } else
            MatchError++;
        
        pairwise_matches[pair_idx].src_img_idx = from;
        pairwise_matches[pair_idx].dst_img_idx = to;
        
        size_t dual_pair_idx = to * num_images + from;
        // std::cout << "MatchPairsBody -> dual_pair_idx is " << dual_pair_idx << std::endl;
        pairwise_matches[dual_pair_idx] = pairwise_matches[pair_idx];
        pairwise_matches[dual_pair_idx].src_img_idx = to;
        pairwise_matches[dual_pair_idx].dst_img_idx = from;
        
        if (!pairwise_matches[pair_idx].H.empty())
            pairwise_matches[dual_pair_idx].H = pairwise_matches[pair_idx].H.inv();
        
        for (size_t j = 0; j < pairwise_matches[dual_pair_idx].matches.size(); ++j)
            std::swap(pairwise_matches[dual_pair_idx].matches[j].queryIdx,
                      pairwise_matches[dual_pair_idx].matches[j].trainIdx);
        
        Dmatcher.release(); //zt
        //zt
        if(pairwise_matches[pair_idx].src_img_idx != -1)
        {
            tmp_pairwise_matches.push_back(pairwise_matches[pair_idx]);
            tmp_pairwise_matches.push_back(pairwise_matches[dual_pair_idx]);
        }
        
        gettimeofday( &submatch_end, NULL );
        //求出两次时间的差值，单位为us
        int submatch_timeuse = 1000000 * ( submatch_end.tv_sec - submatch_start.tv_sec ) + submatch_end.tv_usec - submatch_start.tv_usec;
        //printf("submatch time: %d us\n", submatch_timeuse);
    }
    
    if(num_images == 2)  // 直接返回结果
    {
        global_pairwise_matches.push_back(pairwise_matches[0]);
        for(int i = 0; i < num_images; ++i)
            global_pairwise_matches.push_back(tmp_pairwise_matches[i]);
        global_pairwise_matches.push_back(pairwise_matches[0]);
    }
    // 因为 temp 里面的都是成对出现, 如0-1 和 1-0, 0-2 和 2-0, 1-2 和 2-1, 0-3 和 3-0 , 所以下面的直接取除temp元素,不需要再取排序
    // 下面每次insert 完可以将temp里面的erase 掉, 这里我没做这个
    else if(num_images == 3)  // 先将0-2 插进去, 再将1-2 插到end(), 再 2-0 和 2-1
    {
        global_pairwise_matches.insert(global_pairwise_matches.begin() + num_images - 1,tmp_pairwise_matches[0]);// 将0-2 插进去
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            //先将 1-2 插进去,
            if(tmp_pairwise_matches[i].src_img_idx != 0 && tmp_pairwise_matches[i].dst_img_idx == num_images - 1)
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
        }
        //再插 2-0 和 2-1
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            if (tmp_pairwise_matches[i].src_img_idx == num_images - 1)
                global_pairwise_matches.insert(global_pairwise_matches.end(), tmp_pairwise_matches[i]);
        }
        global_pairwise_matches.push_back(global_pairwise_matches[0]);
    }
    else  //num_images > 3
    {
        // global_pairwise_matches.insert(global_pairwise_matches.begin() + num_images - 1,tmp_pairwise_matches[0]);
        //查找每次新的 (0-(num_images -1)) ,(0-(num_image - 2)),(1-(num_images - 1)),...,插入到对应位置
        for (int i = 0; i < tmp_pairwise_matches.size(); i++ ){
            for(int j = 0; j < global_pairwise_matches.size(); ++j)
            {
                if(global_pairwise_matches[j].src_img_idx == tmp_pairwise_matches[i].src_img_idx && global_pairwise_matches[j].dst_img_idx == num_images - 2)
                {
                    global_pairwise_matches.insert(global_pairwise_matches.begin() + j + 1,tmp_pairwise_matches[i]);
                    tmp_pairwise_matches.erase(tmp_pairwise_matches.begin() + i);
                }
            }
        }
        //最后几个上面没有对应的位置,直接插到末尾
        for(int i = 0; i < tmp_pairwise_matches.size(); ++i)
        {
            if(tmp_pairwise_matches[i].src_img_idx != num_images - 1)
            {
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
                tmp_pairwise_matches.erase(tmp_pairwise_matches.begin() + i);
            }
        }
        //从 (num_imagse -1 )-0 , (num_imagse -1 )-1, (num_imagse -1 )-2, ... 插入末尾
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            
            if(tmp_pairwise_matches[i].src_img_idx == num_images - 1 )
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
        }
        //最后一个跟自己匹配的结果(num_images - 1)-(num_images - 1), 内容和第一个一样,直接插入
        global_pairwise_matches.insert(global_pairwise_matches.end(),pairwise_matches[0]);
    }
    gettimeofday( &match_end, NULL );
    //求出两次时间的差值，单位为us
    int match_timeuse = 1000000 * ( match_end.tv_sec - match_start.tv_sec ) + match_end.tv_usec - match_start.tv_usec;
    
    if(!strLogPath.empty())
        output_file << "LOGINFO: Matcher Feature time is " << match_timeuse << "  us."<< std::endl;
    else
    {
        printf("match time: %d us\n", match_timeuse);
    }
    
    //当前图片与之前匹配不满足条件的个数如果等于所有匹配次数，则此次匹配失败，删除保存的当前图片和特征, 返回拼接失败的结果
    //    if(MatchError == num_images - 1)
    //    {
    //        global_images.pop_back();
    //        global_input_images.pop_back();
    //        global_img_feature.pop_back();
    //        stitchflag = false;
    //
    //        if(!strLogPath.empty())
    //            output_file << "Match error = num_images - 1 ." << std::endl;
    //        else
    //            std::cout << "Match error == num_images - 1 ." << std::endl;
    //        return false;
    //    }
    
    //判断哪些是不能拼接的, 在matcher里面我已经通过设置匹配置信度和内点数量进行了判断
    vector<int> indices;
    indices.clear();
    vector <ImageFeatures> img_feature_tmp;
    img_feature_tmp.clear();
    img_feature_tmp = global_img_feature;
    vector <MatchesInfo> pairwise_matches_tmp;
    pairwise_matches_tmp.clear();
    pairwise_matches_tmp = global_pairwise_matches;
    indices = leaveBiggestComponent(img_feature_tmp, pairwise_matches_tmp, conf_thresh);
    if(indices.size() < num_images)
    {
        if(!strLogPath.empty())
        {
            output_file << "indice size < num_images." << std::endl;
            output_file.close();
        }
        else
            std::cout << "indice size < num_images." << std::endl;
        global_images.pop_back();
        global_input_images.pop_back();
        global_img_feature.pop_back();
        
        stitchflag = false;
        
        return false;
    }
    
    //寻找最大匹配
    vector<int> pair_idx;
    vector<double> pair_confidence;
    vector<int> pair_points;
    for (int i = 0; i < global_img_feature.size(); ++i) {
        pair_idx.push_back(-1);
        pair_confidence.push_back(0.0);
        pair_points.push_back(0);
    }
    int src_img_idx;
    int dst_img_idx;
    double confidence;
    int num_inliers;
    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
        src_img_idx = global_pairwise_matches[i].src_img_idx;
        dst_img_idx = global_pairwise_matches[i].dst_img_idx;
        confidence = global_pairwise_matches[i].confidence;
        num_inliers = global_pairwise_matches[i].num_inliers;
        if (pair_idx[src_img_idx] == -1) {
            pair_idx[src_img_idx] = dst_img_idx;
            pair_confidence[src_img_idx] = confidence;
            pair_points[src_img_idx] = num_inliers;
        } else if (confidence > pair_confidence[src_img_idx]) {
            pair_idx[src_img_idx] = dst_img_idx;
            pair_confidence[src_img_idx] = confidence;
            pair_points[src_img_idx] = num_inliers;
        }
    }
    
    bool bflag = false;//zt
    int ImgNum = global_img_feature.size(); //这里的ImgNum包括当前传进来的图片
    for (int i = 0; i < ImgNum; ++i) {
        if (pair_points[i] <= inliner_num) {
            bflag = true;
        }
    }
    
    if(!strLogPath.empty())
    {
        if(bflag)
        {
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            stitchflag = false;
            output_file << "LOGINFO: inliner_num < 16" << std::endl;
            output_file.close(); //zt
            return false; //zt
        }
        
    } else
    {
        if(bflag)
        {
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            std::cout << "LOGINFO: inliner_num < 16" << std::endl;
            stitchflag = false;
            return false; //zt
        }
    }
    
    vector<CameraParams> cameras;
    struct timeval eatimator_start, estimator_end;
    gettimeofday( &eatimator_start, NULL );
    
    if(!strLogPath.empty())
    {
        if (!(*estimator)(global_img_feature, global_pairwise_matches, cameras)){
            output_file << "LOGINFO: camera estimat failed." << std::endl; //zt
            output_file.close(); //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            stitchflag = false;
            return false;
        }
    }
    else
    {
        if (!(*estimator)(global_img_feature, global_pairwise_matches, cameras)){
            std::cout << "LOGINFO: camera estimat failed." << std::endl; //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            stitchflag = false;
            return false;
        }
    }
    gettimeofday( &estimator_end, NULL );
    //求出两次时间的差值，单位为us
    int estimator_timeuse = 1000000 * ( estimator_end.tv_sec - eatimator_start.tv_sec ) + estimator_end.tv_usec - eatimator_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "estimator time is " << estimator_timeuse  << "  us."<< std::endl;
    else
        printf("estimator_ time: %d us\n", estimator_timeuse);
    
    for (size_t i = 0; i < cameras.size(); ++i){
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
    }
    
    struct timeval adjust_start, adjust_end;
    gettimeofday( &adjust_start, NULL);
    
    if(!strLogPath.empty())
    {
        if (!(*adjuster)(global_img_feature, global_pairwise_matches, cameras)){
            output_file << "LOGERROR: camera adjuster failed." << std::endl; //zt
            output_file.close(); //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            stitchflag = false;
            return false;
        }
        // check camera parameter R
        CameraParams camera;
        for (int i = 0; i < cameras.size(); ++i) {
            camera = cameras[i];
            for (int row = 0; row < camera.R.rows; row++) {
                for (int col = 0; col < camera.R.cols; col++) {
                    if (isnan(camera.R.at<int>(row,col))) {
                        output_file << "LOGINFO: camera check failed." << std::endl; //zt
                        output_file.close(); //zt
                        global_images.pop_back();
                        global_input_images.pop_back();
                        global_img_feature.pop_back();
                        
                        stitchflag = false;
                        return false;
                    }
                }
            }
        }
        
    }
    else
    {
        if (!(*adjuster)(global_img_feature, global_pairwise_matches, cameras)){
            std::cout << "LOGERROR: camera adjuster failed." << std::endl; //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            stitchflag = false;
            return false;
        }
        // check camera parameter R
        CameraParams camera;
        for (int i = 0; i < cameras.size(); ++i) {
            camera = cameras[i];
            for (int row = 0; row < camera.R.rows; row++) {
                for (int col = 0; col < camera.R.cols; col++) {
                    if (isnan(camera.R.at<int>(row,col))) {
                        std::cout << "LOGINFO: camera check failed." << std::endl; //zt
                        global_images.pop_back();
                        global_input_images.pop_back();
                        global_img_feature.pop_back();
                        
                        stitchflag = false;
                        return false;
                    }
                }
            }
        }
    }
    
    gettimeofday( &adjust_end, NULL );
    //求出两次时间的差值，单位为us
    int adjust_timeuse = 1000000 * ( adjust_end.tv_sec - adjust_start.tv_sec ) + adjust_end.tv_usec - adjust_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "adjust time is " << adjust_timeuse << "  us." << std::endl;
    else
        printf("adjust time: %d us\n", adjust_timeuse);
    
    camera_params = cameras;
    //大于最大匹配数保存当前图片的相机参数  //zt
    if(InputImgNums <= MaxImgNums_)
        global_camera_params = cameras;
    else
        global_camera_params.push_back(cameras[MaxImgNums_ - 1]);
    
    vector<double> focals;
    focals.clear(); //zt
    for (size_t i = 0; i < cameras.size(); ++i){
        focals.push_back(cameras[i].focal);
    }
    
    sort(focals.begin(), focals.end());
    float warped_image_scale;
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;
    
    vector<Point> corners(global_input_images.size());
    vector<UMat> masks_warped(global_input_images.size());
    vector<UMat> images_warped(global_input_images.size());
    vector<Size> sizes(global_input_images.size());
    vector<UMat> masks(global_input_images.size());
    
    
    struct timeval warp_start, warp_end;
    gettimeofday( &warp_start, NULL );
    
#ifndef WARP
    // Preapre images masks
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(global_images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }
    
    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
    
    for (int i = 0; i < num_images; ++i)
    {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;
        
        //check
        Rect roi = warper->warpRoi(global_images[i].size(), K, cameras[i].R);
        
        if (roi.width > pano_max_edge || roi.height > pano_max_edge){
            if(!strLogPath.empty())
            {   output_file << "LOGERROR： roi.width or roi.height > pano_max_edge. "<< std::endl;
                output_file.close();
                global_images.pop_back();
                global_input_images.pop_back();
                global_img_feature.pop_back();
                
                stitchflag = false;
                return false;
            }
            else
            {
                std::cout << "LOGERROR： roi.width or roi.height > pano_max_edge. "<< std::endl;
                global_images.pop_back();
                global_input_images.pop_back();
                global_img_feature.pop_back();
                
                stitchflag = false;
                return false;
            }
        }
        
        corners[i] = warper->warp(global_images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();
        
        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
    }
    
    vector<UMat> images_warped_f(num_images);
    for (int i = 0; i < num_images; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);
    
    gettimeofday( &warp_end, NULL );
    //求出两次时间的差值，单位为us
    int warp_timeuse = 1000000 * ( warp_end.tv_sec - warp_start.tv_sec ) + warp_end.tv_usec - warp_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "warp time is " << warp_timeuse << "  us." << std::endl;
    else
        printf("warp time: %d us\n", warp_timeuse);
    //seam time
    struct timeval seam_start, seam_end;
    gettimeofday( &seam_start, NULL );
    
    seam_finder->find(images_warped_f, corners, masks_warped);
    gettimeofday( &seam_end, NULL );
    //求出两次时间的差值，单位为us
    int seam_timeuse = 1000000 * ( seam_end.tv_sec - seam_start.tv_sec ) + seam_end.tv_usec - seam_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "seam time is " << seam_timeuse << "  us." << std::endl;
    else
        printf("seam time: %d us\n", seam_timeuse);
    
    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    Mat input_img;
    double compose_seam_aspect = 1;
    double compose_work_aspect = 1;
    
    struct timeval compose_start, compose_end;
    gettimeofday( &compose_start, NULL );
    
    if (!is_compose_scale_set ) {
        compose_scale = min(1.0, sqrt(compose_megapix * MaxPix_ / input.size().area()));
        is_compose_scale_set = true;
    }
    
    // Compute relative scales
    compose_seam_aspect = compose_scale / seam_scale;
    compose_work_aspect = compose_scale / work_scale;
    
    // Update warped image scale
    warped_image_scale *= static_cast<float>(compose_work_aspect);
    warper = warper_creator->create(warped_image_scale);
    
    // Update corners and sizes
    for (int i = 0; i < num_images; ++i)
    {
        // Update intrinsics
        cameras[i].focal *= compose_work_aspect;
        cameras[i].ppx *= compose_work_aspect;
        cameras[i].ppy *= compose_work_aspect;
        
        // Update corner and size
        Size sz = input.size();
        if (std::abs(compose_scale - 1) > 1e-1)
        {
            sz.width = cvRound(input.size().width * compose_scale);
            sz.height = cvRound(input.size().height * compose_scale);
        }
        
        Mat K;
        cameras[i].K().convertTo(K, CV_32F);
        Rect roi = warper->warpRoi(sz, K, cameras[i].R);
        corners[i] = roi.tl();
        sizes[i] = roi.size();
    }
    
    for (int img_idx = 0; img_idx < num_images; ++img_idx)
    {
        // Read image and resize it if necessary
        input_img = global_input_images[img_idx].clone();
        
        if (abs(compose_scale - 1) > 1e-1)
            cv::resize(input_img, img, Size(), compose_scale, compose_scale, INTER_LINEAR_EXACT);
        else
            img = input_img.clone();
        
        input_img.release();
        Size img_size = img.size();
        
        Mat K;
        cameras[img_idx].K().convertTo(K, CV_32F);
        
        // Warp the current image
        warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);
        
        // Warp the current image mask
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);
        
        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();
        
        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
        mask_warped = seam_mask & mask_warped;
        
        if (!blender && !timelapse)
        {
            blender = Blender::createDefault(blend_type, try_cuda);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, try_cuda);
            else if (blend_type == Blender::MULTI_BAND)
            {
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
            }
            blender->prepare(corners, sizes);
        }
        
        // Blend the current image
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }
    
    Mat result, result_mask;
    blender->blend(result, result_mask);
    
    gettimeofday( &compose_end, NULL );
    //求出两次时间的差值，单位为us
    int compose_timeuse = 1000000 * ( compose_end.tv_sec - compose_start.tv_sec ) + compose_end.tv_usec - compose_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "compose time is " << compose_timeuse << "  us." << std::endl;
    else
        printf("compose time: %d us\n", compose_timeuse);
    
    result.copyTo(dst);
    result_size = result.size();
    //zt log camera parameters
    if(!strLogPath.empty())
    {
        for (size_t tt = 0; tt < cameras.size(); ++tt)
        {
            output_file << " LOGINFO: camera information: " << std::endl;  //zt
            output_file << "    aspect is " << cameras[tt].aspect << endl;
            output_file << "    focal is " << cameras[tt].focal << endl;
            output_file << "    Principal point X is " << cameras[tt].ppx  << ", Principal point Y is " << cameras[tt].ppy << std::endl;
            output_file << "    R matrix is " << cameras[tt].R << std::endl;
            output_file << "    t matrix is " << cameras[tt].t << std::endl;
        }
        // zt log pano size
        output_file << " LOGINFO: pano size is " << result_size << std::endl; //zt
    }
    ///MAP compute
    
    //zt 保存全局 corners
    if(InputImgNums <= MaxImgNums_)
    {
        global_corners = corners;
        corner_tmp = corners[corners.size() - 1];
    }
    else
    {
        //将大于最大匹配的corner都转换到第一次的corner上
        corner_tmp.x = corners[corners.size() - 1].x + corner_tmp.x - corners[corners.size() - 2].x;
        corner_tmp.y = corners[corners.size() - 1].y + corner_tmp.y - corners[corners.size() - 2].y;
        global_corners.push_back(corner_tmp);
    }
    //MAP
    Point left_top;
    Point right_bottom;
    int stitch_x = 0;
    int stitch_y = 0;
    Size blend_image_size;
    vector<Point> corner_points;
    corner_points.clear();
    Point map_idx;
    vector<int> left_top_x;
    left_top_x.clear();
    vector<int> left_top_y;
    left_top_y.clear();
    //sum_corner_points.clear(); //zt 20190404 没有调用save_param 需clear
    //convert left_top points and size into corner points
    for (int img_idx = 0; img_idx < InputImgNums; ++img_idx) {
        // calculate blend global_images 4 corner points
        blend_image_size = sizes[img_idx];
        left_top = global_corners[img_idx];
        if (left_top.x < stitch_x) {
            stitch_x = left_top.x;
        }
        if (left_top.y < stitch_y) {
            stitch_y = left_top.y;
        }
        right_bottom.x = left_top.x + blend_image_size.width;
        right_bottom.y = left_top.y + blend_image_size.height;
        //std::cout << "left_top: " << left_top << std::endl;
        //std::cout << "right_bottom: " << right_bottom << std::endl;
        corner_points.push_back(left_top);
        corner_points.push_back(right_bottom);
        sum_corner_points.push_back(corner_points);
        corner_points.clear();
    }
    if (!strLogPath.empty())
        output_file << " LOGINFO: stitch_left_topx is " << stitch_x << " ,stitch_left_topy is " << stitch_y
        << std::endl;
    for (int i = 0; i < InputImgNums; ++i) {
        sum_corner_points[i][0].x = sum_corner_points[i][0].x - stitch_x;
        sum_corner_points[i][0].y = sum_corner_points[i][0].y - stitch_y;
        sum_corner_points[i][1].x = sum_corner_points[i][1].x - stitch_x;
        sum_corner_points[i][1].y = sum_corner_points[i][1].y - stitch_y;
        Point tmp_lt_pnt = sum_corner_points[i][0];
        Point tmp_rb_pnt = sum_corner_points[i][1];
        if (!strLogPath.empty())
            output_file << " LOGINFO: cornetPoints Info " << i << " image corner points in pano: " << tmp_lt_pnt << ", "
            << tmp_rb_pnt << std::endl;
    }
    //image map index
    for (int img_idx = 0; img_idx < InputImgNums; ++img_idx) {
        left_top_x.push_back(global_corners[img_idx].x);
        left_top_y.push_back(global_corners[img_idx].y);
    }
    vector<long unsigned int> sorted_x_idx;
    sorted_x_idx.clear();
    vector<long unsigned int> sorted_y_idx;
    sorted_y_idx.clear();
    sorted_x_idx = sort_indexes_e(left_top_x);
    sorted_y_idx = sort_indexes_e(left_top_y);
    int x;
    int y;
    int w_diff;
    int h_diff;
    vector<int> vec_w_diff;
    vector<int> vec_h_diff;
    vector<int> corner_dist;
    vector<long unsigned int> sorted_dist_idx;
    vec_w_diff.clear();
    vec_h_diff.clear();
    corner_dist.clear();
    sorted_dist_idx.clear();
    int cloest_idx;
    Point cloest_point;
    int step;
    int step_direction; // 0: x; 1: y
    for (int img_idx = 0; img_idx < InputImgNums; ++img_idx) {
        if (img_idx == 0) {
            map_idx.x = 0;
            map_idx.y = 0;
        } else {
            x = left_top_x[img_idx];
            y = left_top_y[img_idx];
            // std::cout << "image_map_index " << image_map_index.size() << std::endl;
            for (int i = 0; i < image_map_index.size(); ++i) {
                w_diff = x - left_top_x[i];
                h_diff = y - left_top_y[i];
                vec_w_diff.push_back(w_diff);
                vec_h_diff.push_back(h_diff);
                corner_dist.push_back(abs(w_diff) + abs(h_diff));
            }
            sorted_dist_idx = sort_indexes_e(corner_dist);
            cloest_idx = sorted_dist_idx[0];
            cloest_point = image_map_index[cloest_idx];
            
            w_diff = vec_w_diff[cloest_idx];
            h_diff = vec_h_diff[cloest_idx];
            //            if(!strLogPath.empty())
            //            {
            //                output_file << " LOGINFO: cloest_idx: " << cloest_idx << std::endl;
            //                output_file << " LOGINFO: cloest_point: " << cloest_point << std::endl;
            //
            //                output_file << " LOGINFO: w_diff: " << w_diff << std::endl;
            //                output_file << " LOGINFO: h_diff: " << h_diff << std::endl;
            //            }
            
            //            std::cout << " LOGINFO: cloest_idx: " << cloest_idx << std::endl;
            //            std::cout << " LOGINFO: cloest_point: " << cloest_point << std::endl;
            //
            //            std::cout << " LOGINFO: w_diff: " << w_diff << std::endl;
            //            std::cout << " LOGINFO: h_diff: " << h_diff << std::endl;
            if (abs(w_diff) >= abs(h_diff)) {
                step_direction = 0;
                if (w_diff >= 0) {
                    step = 1;
                } else {
                    step = -1;
                }
                map_idx.x = cloest_point.x + step;
                map_idx.y = cloest_point.y;
            } else {
                step_direction = 1;
                if (h_diff >= 0) {
                    step = 1;
                } else {
                    step = -1;
                }
                map_idx.y = cloest_point.y + step;
                map_idx.x = cloest_point.x;
            }
            for (int i = 0; i < image_map_index.size(); ++i) {
                Point tmp_point = image_map_index[i];
                if (step_direction == 0 && tmp_point.y == map_idx.y && tmp_point.x >= map_idx.x) {
                    ++image_map_index[i].x;
                } else if (step_direction == 1 && tmp_point.x == map_idx.x && tmp_point.y >= map_idx.y) {
                    ++image_map_index[i].y;
                }
            }
        }
        image_map_index.push_back(map_idx);
        vec_w_diff.clear();
        vec_h_diff.clear();
        corner_dist.clear();
        sorted_dist_idx.clear();
    }
    //image map index reset
    vector<int> map_idx_x;
    map_idx_x.clear();
    vector<int> map_idx_y;
    map_idx_y.clear();
    vector<int> map_idx_first_row;
    map_idx_first_row.clear();
    for (int i = 0; i < InputImgNums; ++i) {
        map_idx_x.push_back(image_map_index[i].x);
        map_idx_y.push_back(image_map_index[i].y);
        if (image_map_index[i].y == 0) {
            map_idx_first_row.push_back(image_map_index[i].x);
        }
    }
    //convert negative index to postive
    int min_x = *min_element(map_idx_x.begin(), map_idx_x.end());
    int min_y = *min_element(map_idx_y.begin(), map_idx_y.end());
    if (min_x < 0) {
        for (int i = 0; i < map_idx_x.size(); ++i) {
            map_idx_x[i] = map_idx_x[i] + abs(min_x);
        }
        for (int i = 0; i < map_idx_first_row.size(); ++i) {
            map_idx_first_row[i] = map_idx_first_row[i] + abs(min_x);
        }
    }
    if (min_y < 0) {
        for (int i = 0; i < InputImgNums; ++i) {
            map_idx_y[i] = map_idx_y[i] + abs(min_y);
        }
    }
    //find the miss first row col after positive convert
    map_idx_first_row.clear();
    for (int i = 0; i < InputImgNums; ++i) {
        if (map_idx_y[i] == 0) {
            map_idx_first_row.push_back(map_idx_x[i]);
        }
    }
    //find the col index of missing point in first row
    vector<int>::iterator ret;
    vector<int> miss_x;
    miss_x.clear();
    for (int i = 0; i < InputImgNums; ++i) {
        if (map_idx_y[i] > 0) {
            ret = find(map_idx_first_row.begin(), map_idx_first_row.end(), map_idx_x[i]);
            if (ret == map_idx_first_row.end()) {
                miss_x.push_back(map_idx_x[i]);
            }
        }
    }
    //pop up the missing column until touch the first row
    vector<vector<int>> miss_columns;
    miss_columns.clear();
    vector<vector<int>> tmp_idxes;
    tmp_idxes.clear();
    vector<int> col;
    col.clear();
    for (int i = 0; i < miss_x.size(); ++i) {
        miss_columns.push_back(col);
        tmp_idxes.push_back(col);
    }
    int col_idx;
    for (int i = 0; i < InputImgNums; ++i) {
        if (map_idx_y[i] > 0) {
            ret = find(miss_x.begin(), miss_x.end(), map_idx_x[i]);
            if (ret != miss_x.end()) {
                col_idx = distance(miss_x.begin(), ret);
                miss_columns[col_idx].push_back(map_idx_y[i]);
                tmp_idxes[col_idx].push_back(i);
            }
        }
    }
    for (int i = 0; i < miss_columns.size(); ++i) {
        col = miss_columns[i];
        int min_y_in_col = *min_element(col.begin(), col.end());
        for (int j = 0; j < col.size(); ++j) {
            map_idx_y[tmp_idxes[i][j]] = map_idx_y[tmp_idxes[i][j]] - min_y_in_col;
        }
    }
    //connect the break point in col
    col.clear();
    int n_col = *max_element(map_idx_x.begin(), map_idx_x.end()) + 1;
    vector<int> idx_in_col;
    idx_in_col.clear();
    int src_idx;
    int prev_idx;
    for (int i = 0; i <= n_col; ++i) {
        for (int j = 0; j < map_idx_y.size(); ++j) {
            if (map_idx_x[j] == i) {
                col.push_back(map_idx_y[j]);
                idx_in_col.push_back(j);
            }
        }
        vector<long unsigned int> sorted_col_idx = sort_indexes_e(col);
        for (int j = 1; j < sorted_col_idx.size(); ++j) {
            src_idx = idx_in_col[sorted_col_idx[j]];
            prev_idx = idx_in_col[sorted_col_idx[j - 1]];
            if (map_idx_y[src_idx] != map_idx_y[prev_idx] + 1) {
                map_idx_y[src_idx] = map_idx_y[prev_idx] + 1;
            }
        }
        col.clear();
        idx_in_col.clear();
        sorted_col_idx.clear();
    }
    //connect the break point in row
    vector<int> row;
    row.clear();
    int n_row = *max_element(map_idx_y.begin(), map_idx_y.end()) + 1;
    vector<int> idx_in_row;
    idx_in_row.clear();
    src_idx = 0;
    prev_idx = 0;
    for (int i = 0; i <= n_row; ++i) {
        for (int j = 0; j < map_idx_x.size(); ++j) {
            if (map_idx_y[j] == i) {
                row.push_back(map_idx_x[j]);
                idx_in_row.push_back(j);
            }
        }
        vector<long unsigned int> sorted_row_idx = sort_indexes_e(row);
        for (int j = 1; j < sorted_row_idx.size(); ++j) {
            src_idx = idx_in_row[sorted_row_idx[j]];
            prev_idx = idx_in_row[sorted_row_idx[j - 1]];
            if (map_idx_x[src_idx] != map_idx_x[prev_idx] + 1) {
                map_idx_x[src_idx] = map_idx_x[prev_idx] + 1;
            }
        }
        row.clear();
        idx_in_row.clear();
        sorted_row_idx.clear();
    }
    //find the col index of missing point in first row
    miss_x.clear();
    for (int i = 0; i < InputImgNums; ++i) {
        if (map_idx_y[i] > 0) {
            ret = find(map_idx_first_row.begin(), map_idx_first_row.end(), map_idx_x[i]);
            if (ret == map_idx_first_row.end()) {
                miss_x.push_back(map_idx_x[i]);
            }
        }
    }
    //pop up the missing column until touch the first row
    miss_columns.clear();
    tmp_idxes.clear();
    col.clear();
    for (int i = 0; i < miss_x.size(); ++i) {
        miss_columns.push_back(col);
        tmp_idxes.push_back(col);
    }
    col_idx = 0;
    for (int i = 0; i < InputImgNums; ++i) {
        if (map_idx_y[i] > 0) {
            ret = find(miss_x.begin(), miss_x.end(), map_idx_x[i]);
            if (ret != miss_x.end()) {
                col_idx = distance(miss_x.begin(), ret);
                miss_columns[col_idx].push_back(map_idx_y[i]);
                tmp_idxes[col_idx].push_back(i);
            }
        }
    }
    for (int i = 0; i < miss_columns.size(); ++i) {
        col = miss_columns[i];
        int min_y_in_col = *min_element(col.begin(), col.end());
        for (int j = 0; j < col.size(); ++j) {
            map_idx_y[tmp_idxes[i][j]] = map_idx_y[tmp_idxes[i][j]] - min_y_in_col;
        }
    }
    
    //restore map index, start from (1, 1)
    for (int i = 0; i < InputImgNums; ++i) {
        image_map_index[i].x = map_idx_x[i] + 1;
        image_map_index[i].y = map_idx_y[i] + 1;
        // std::cout << "map_idx_x = " << map_idx_x[i] << ",  map_idx_y = " << map_idx_y[i] << std::endl;
    }
    
    // Release unused memory
    //global_images.clear();
    //global_input_images.clear();
    images_warped.clear();
    images_warped_f.clear();
    masks.clear();
    cameras.clear();
    corners.clear();
    sizes.clear();
    
    img_warped_s.release();
    dilated_mask.release();
    seam_mask.release();
    mask_warped.release();
    result.release();
    result_mask.release();
    
    corner_points.clear();
    left_top_x.clear();
    left_top_y.clear();
    sorted_x_idx.clear();
    sorted_y_idx.clear();
    map_idx_x.clear();
    map_idx_y.clear();
    map_idx_first_row.clear();
    miss_x.clear();
    miss_columns.clear();
    tmp_idxes.clear();
    col.clear();
#endif
    
    gettimeofday( &stitch_end, NULL );
    //求出两次时间的差值，单位为us
    int stitch_timeuse = 1000000 * ( stitch_end.tv_sec - stitch_start.tv_sec ) + stitch_end.tv_usec - stitch_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "total time is " << stitch_timeuse << "  us." << std::endl;
    else
        printf("total time: %d us\n", stitch_timeuse);
    
    if(!strLogPath.empty())
    {
        output_file.close(); //zt
    }
    InputImgNums ++;
    stitchflag = true;
    return true;
}
#endif

#ifdef NEWMETHOD_RETAKE //20190416 没有相机参数和坐标
bool stitching(std::vector <cv::Mat> &imglist, Mat &dst, int mode, std::string & strLogPath){
    if (mode == 1 ){
        if (all_images.size() > 0){
            delete_pic(all_images.size());
            if (pano_vec.size() > 0){
                pano_vec[pano_vec.size() - 1].copyTo(dst);
            }
            return true;
        }
        else
            return false;
    }
    
    struct timeval stitch_start, stitch_end;
    gettimeofday( &stitch_start, NULL );
    
    ofstream output_file;
    if(!strLogPath.empty())
    {
        output_file.open(strLogPath, ios::out); //zt
        if(imglist.size() == 0)
        {
            output_file << "LOGERROR: input images = 0." << std::endl;
            output_file.close();
            return false;
        }
        if(imglist.size() > 1)
        {
            output_file << "LOGERROR: input images > 1, please input just one img." << std::endl;
            output_file.close();
            return false;
        }
        
        if(imglist[0].empty())
        {
            output_file << "LOGERROR: input img is empty." << std::endl;
            output_file.close();
            return false;
        }
    }
    else
    {
        if(imglist.size() == 0)
        {
            printf("input images = 0.");
            return false;
        }
        if(imglist.size() > 1)
        {
            printf("input images > 1, please input just one img");
            return false;
        }
        
        if(imglist[0].empty())
        {
            printf("input img is empty.");
            return false;
        }
    }
    
    if (!is_input_scale_set) {
        if (work_pixel == 0){
            input_scale = min(1.0, sqrt(input_megapix * MaxPix_ / imglist[0].size().area()));
            is_input_scale_set = true;
        }
        else{
            if (min(imglist[0].rows, imglist[0].cols)<= work_pixel){
                input_scale = 1;
                is_input_scale_set = true;
            }
            else{
                input_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                is_input_scale_set = true;
            }
        }
    }
    //每次只能送一张进行stitching
    if(InputImgNums == 0)
        InputImgNums = imglist.size(); // 当前图片就1张
    
    if(InputImgNums == 1)  //first image , return current img as pino.jpg
    {
        dst = imglist[0];
        pano_vec.push_back(dst); //zt
        InputImgNums++;
        ImageFeatures featureTemp;
        Mat img;
        resize(imglist[0], input, Size(), input_scale, input_scale, INTER_NEAREST);
        
        if (work_megapix < 0)
        {
            img = input.clone();
            work_scale = 1;
            is_work_scale_set = true;
        }
        else {
            if (!is_work_scale_set) {
                if (work_pixel == 0){
                    work_scale = min(1.0, sqrt(work_megapix * MaxPix_ / input.size().area()));
                    is_work_scale_set = true;
                }
                else{
                    if (min(imglist[0].rows, imglist[0].cols)<= work_pixel){
                        work_scale = 1;
                        is_work_scale_set = true;
                    }
                    else{
                        work_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                        is_work_scale_set = true;
                    }
                }
            }
            resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
        }
        if (!is_seam_scale_set)
        {
            seam_scale = min(1.0, sqrt(seam_megapix * MaxPix_ / input.size().area()));
            seam_work_aspect = seam_scale / work_scale;
            is_seam_scale_set = true;
        }
        //find feature
        struct timeval first_find_start, first_find_end;
        gettimeofday( &first_find_start, NULL );
        (*finder)(img, featureTemp);
        gettimeofday( &first_find_end, NULL );
        //求出两次时间的差值，单位为us
        int firstfindtimeuse = 1000000 * ( first_find_end.tv_sec - first_find_start.tv_sec ) + first_find_end.tv_usec - first_find_start.tv_usec;
        if(!strLogPath.empty())
            output_file << "first img find time is " << firstfindtimeuse << "  us."<< std::endl;
        else
            printf("first img find time: %d us\n", firstfindtimeuse);
        
        featureTemp.img_idx = 0;
        global_img_feature.push_back(featureTemp); //debug
        all_img_feature.push_back(featureTemp);
        finder->collectGarbage();
        
        if (try_compose == true)
            resize(input, img, Size(), seam_scale, seam_scale, INTER_NEAREST);
        else {
            if (!is_compose_scale_set) {
                if (work_pixel == 0) {
                    compose_scale = min(1.0, sqrt(compose_scale * MaxPix_ / input.size().area()));
                    is_compose_scale_set = true;
                } else {
                    if (min(imglist[0].rows, imglist[0].cols) <= work_pixel) {
                        compose_scale = 1;
                        is_compose_scale_set = true;
                    } else {
                        compose_scale = work_pixel * 1. / min(imglist[0].rows, imglist[0].cols);
                        is_compose_scale_set = true;
                    }
                }
            }
            resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
        }
        global_input_images.push_back(input.clone());//debug
        all_input_images.push_back(input.clone());
        global_images.push_back(img.clone());  //debug
        all_images.push_back(img.clone());
        if(!strLogPath.empty())
            output_file.close();
        return true;
    }
    
    std::cout << "global_input_images size is " << global_input_images.size() << std::endl;
    std::cout << "global_images size is " << global_images.size() << std::endl;
    std::cout << "global_img_feature size is " << global_img_feature.size() << std::endl;
#ifdef ORI0416
    if(!stitchflag)  //前面拼接失败,需要删除前面最近一次的匹配信息
    {
        global_pairwise_matches.pop_back();
        
        if(InputImgNums == 2)
            global_pairwise_matches.clear();
        else if(InputImgNums <= MaxImgNums_ && InputImgNums > 2)
        {
            
            int srcMaxIdx = 0, dstMaxIdx = 0;
            global_pairwise_matches.pop_back();
            for(int i= 0; i< global_pairwise_matches.size(); ++i)
            {
                if(global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                    srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                if(global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                    dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
            }
            
            for(int i = 0; i < global_pairwise_matches.size(); ++i)
            {
                if(global_pairwise_matches[i].src_img_idx == srcMaxIdx || global_pairwise_matches[i].dst_img_idx == dstMaxIdx)
                {
                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                    i = -1;
                }
            }
            
            //            for(int i = 0; i < global_pairwise_matches.size(); ++i)
            //            {
            //                if(global_pairwise_matches[i].src_img_idx == InputImgNums - 1 || global_pairwise_matches[i].dst_img_idx == InputImgNums - 1)
            //                {
            //                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
            //                    i = -1;
            //                }
            //            }
        } else  //大于最大匹配数
        {
            
            for(int i = 0; i < global_pairwise_matches.size(); ++i)
            {
                if(global_pairwise_matches[i].src_img_idx == MaxImgNums_ - 1 || global_pairwise_matches[i].dst_img_idx == MaxImgNums_ - 1)
                {
                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                    i = -1;
                }
            }
        }
        
    }
#endif
    
    // 当前图片数量 > MaxImgNums_ ,当前global只保存了输入图片数量 - 1 的图片数,并且前面拼接成功,删除保存向量的首个元素, 后面的元素坐标向前退1, 删除当前最大匹配图片的首张图片和特征
    if(global_img_feature.size() >= MaxImgNums_)
    {
        //erase 图片和特征
        global_img_feature.erase(global_img_feature.begin());
        global_images.erase(global_images.begin());
        global_input_images.erase(global_input_images.begin());
        
        //删除匹配信息
        global_pairwise_matches.erase(global_pairwise_matches.begin());  //删除第一个元素
        int matchSize = global_pairwise_matches.size();
        //删除保存向量的第一张图片的匹配信息
        for(int i = 0; i < matchSize; ++i)
        {
            if(global_pairwise_matches[i].src_img_idx == 0 || global_pairwise_matches[i].dst_img_idx == 0)
            {
                global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                i = -1; //执行 ++i将 i 重置为0
            }
        }
        int matchSizeModi = global_pairwise_matches.size();
        //后面的元素坐标向前退1
        for(int i = 0; i < matchSizeModi; ++i)
        {
            if(global_pairwise_matches[i].src_img_idx != -1 )//|| global_pairwise_matches[i].dst_img_idx != -1)
            {
                global_pairwise_matches[i].src_img_idx = global_pairwise_matches[i].src_img_idx - 1;
                global_pairwise_matches[i].dst_img_idx = global_pairwise_matches[i].dst_img_idx - 1;
            }
        }
    }
    
    Mat img;
    resize(imglist[0], input, Size(), input_scale, input_scale, INTER_NEAREST);
    
    if (work_megapix < 0)
    {
        img = input.clone();
        work_scale = 1;
        is_work_scale_set = true;
    }
    else {
        if (!is_work_scale_set) {
            if (work_pixel == 0){
                work_scale = min(1.0, sqrt(work_megapix * MaxPix_ / input.size().area()));
                is_work_scale_set = true;
            }
            else{
                if (min(imglist[0].rows, imglist[0].cols) <= work_pixel){
                    work_scale = 1;
                    is_work_scale_set = true;
                }
                else{
                    work_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                    is_work_scale_set = true;
                }
            }
        }
        resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
    }
    if (!is_seam_scale_set)
    {
        seam_scale = min(1.0, sqrt(seam_megapix * MaxPix_ / input.size().area()));
        seam_work_aspect = seam_scale / work_scale;
        is_seam_scale_set = true;
    }
    
    ImageFeatures feature;
    //find feature
    struct timeval find_start, find_end;
    gettimeofday( &find_start, NULL );
    (*finder)(img, feature);
    
    gettimeofday( &find_end, NULL );
    //求出两次时间的差值，单位为us
    int find_timeuse = 1000000 * ( find_end.tv_sec - find_start.tv_sec ) + find_end.tv_usec - find_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "LOGINFO: Finder time is " << find_timeuse << " us."<< std::endl;
    else
        printf("Finder time: %d us\n", find_timeuse);
    
    feature.img_idx = InputImgNums - 1;
    
    if (try_compose == true)
        resize(input, img, Size(), seam_scale, seam_scale, INTER_NEAREST);
    else {
        if (!is_compose_scale_set) {
            if (work_pixel == 0) {
                compose_scale = min(1.0, sqrt(compose_scale * MaxPix_ / input.size().area()));
                is_compose_scale_set = true;
            } else {
                if (min(imglist[0].rows, imglist[0].cols) <= work_pixel) {
                    compose_scale = 1;
                    is_compose_scale_set = true;
                } else {
                    compose_scale = work_pixel * 1. / min(imglist[0].rows, imglist[0].cols);
                    is_compose_scale_set = true;
                }
            }
        }
        resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
    }
    
    global_input_images.push_back(input.clone());
    global_images.push_back(img.clone());
    global_img_feature.push_back(feature);
    
    
    all_input_images.push_back(input.clone());
    all_images.push_back(img.clone());
    all_img_feature.push_back(feature);
    
    finder->collectGarbage();
    
    // 将matcher 分拆 ///////////////////////////////////////
    vector<MatchesInfo> pairwise_matches;
    pairwise_matches.clear();
    
    struct timeval match_start, match_end;
    gettimeofday( &match_start, NULL );
    
    int num_images = global_img_feature.size();
    
    // 创建匹配 pair, 每次新进来的要clear
    std::vector<std::pair<int,int> > near_pairs;
    near_pairs.clear();
    
    for(int idx = 0; idx < global_img_feature.size() - 1; ++idx)
        if (global_img_feature[global_img_feature.size() - 1].keypoints.size() > 0 && global_img_feature[0].keypoints.size() > 0)
            near_pairs.push_back(std::make_pair(idx,global_img_feature.size() - 1));
    
    pairwise_matches.resize(num_images * num_images);
    
    cv::RNG rng = cv::theRNG(); // save entry rng state
    Range r = Range(0, static_cast<int>(near_pairs.size()));
    
    vector <MatchesInfo> tmp_pairwise_matches;
    tmp_pairwise_matches.clear();// zt
    int MatchError = 0; //定义新进来的图片与之前图片匹配的错误数量，根据此来作为后续判断当前图片是否可以与之前图片拼接
    for (int i = r.start; i < r.end; ++i) {
        struct timeval submatch_start, submatch_end;
        gettimeofday( &submatch_start, NULL );
        // std::cout << "r size is " << r.size() << ",   rng is " << rng.state + i << std::endl;
        //cv::theRNG() = cv::RNG(rng.state + i); // force "stable" RNG seed for each processed pair
        //std::cout << "MatchPairsBody -> rng state = " << rng.state << std::endl;
        cv::theRNG() = cv::RNG(4294967295); //4294967295
        
        int from = near_pairs[i].first;
        int to = near_pairs[i].second;
        int pair_idx = from * num_images + to;
        
        MatchesInfo matches_info;
        matches_info.matches.clear();//zt
        //matcher(features[from], features[to], pairwise_matches[pair_idx]); // 先进BestOf2NearestMatcher::match ，再调用CpuMatcher match
        CV_Assert(global_img_feature[from].descriptors.type() == global_img_feature[to].descriptors.type());
        CV_Assert(global_img_feature[to].descriptors.depth() == CV_8U || global_img_feature[to].descriptors.depth() == CV_32F);
        Ptr<cv::DescriptorMatcher> Dmatcher;
        {
            Ptr<flann::IndexParams> indexParams = makePtr<flann::KDTreeIndexParams>();
            Ptr<flann::SearchParams> searchParams = makePtr<flann::SearchParams>();
            
            if (global_img_feature[to].descriptors.depth() == CV_8U) {
                indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
                searchParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
            }
            
            Dmatcher = makePtr<FlannBasedMatcher>(indexParams, searchParams);
        }
        std::vector<std::vector<DMatch> > pair_matches;
        pair_matches.clear();
        MatchesSet matches;
        
        // Find 1->2 matches
        Dmatcher->knnMatch(global_img_feature[from].descriptors, global_img_feature[to].descriptors, pair_matches, 2);
        //
        for (size_t i = 0; i < pair_matches.size(); ++i) {
            if (pair_matches[i].size() < 2)
                continue;
            const DMatch &m0 = pair_matches[i][0];
            const DMatch &m1 = pair_matches[i][1];
            if (m0.distance < (1.f - match_conf) * m1.distance) {
                matches_info.matches.push_back(m0);
                matches.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
            }
        }
        // LOG("\n1->2 matches: " << matches_info.matches.size() << endl);
        // Find 2->1 matches
        pair_matches.clear();
        Dmatcher->knnMatch(global_img_feature[to].descriptors, global_img_feature[from].descriptors, pair_matches, 2);
        for (size_t i = 0; i < pair_matches.size(); ++i) {
            if (pair_matches[i].size() < 2)
                continue;
            const DMatch &m0 = pair_matches[i][0];
            const DMatch &m1 = pair_matches[i][1];
            if (m0.distance < (1.f - match_conf) * m1.distance)
                if (matches.find(std::make_pair(m0.trainIdx, m0.queryIdx)) == matches.end())
                    matches_info.matches.push_back(DMatch(m0.trainIdx, m0.queryIdx, m0.distance));
        }
        // LOG("1->2 & 2->1 matches: " << matches_info.matches.size() << endl);
        
        // Check if it makes sense to find homography
        if (matches_info.matches.size() >= static_cast<size_t>(6))
        {
            ////
            // Construct point-point correspondences for homography estimation
            Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
            Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
            // std::cout << "BestOf2NearestMatcher::match-> matches_info.matches.size is " << matches_info.matches.size() << std::endl;
            //TODO 取前五百对最小距离的
            for (size_t i = 0; i < matches_info.matches.size(); ++i)
            {
                const DMatch& m = matches_info.matches[i];
                
                Point2f p = global_img_feature[from].keypoints[m.queryIdx].pt;
                p.x -= global_img_feature[from].img_size.width * 0.5f;
                p.y -= global_img_feature[from].img_size.height * 0.5f;
                src_points.at<Point2f>(0, static_cast<int>(i)) = p;
                
                p = global_img_feature[to].keypoints[m.trainIdx].pt;
                p.x -= global_img_feature[to].img_size.width * 0.5f;
                p.y -= global_img_feature[to].img_size.height * 0.5f;
                dst_points.at<Point2f>(0, static_cast<int>(i)) = p;
            }
            
            // Find pair-wise motion
            matches_info.H = findHomography(src_points, dst_points, matches_info.inliers_mask, RANSAC);
            //std::cout << "matches_info pair-wise motion H is " << matches_info.H << std::endl;
            //下面的判断语句是原始源码的,在这需要改成下面的判断并加上符合修改后的判断里面的执行语句,否则在H为空的时候直接用原始的会报错
            //        if (matches_info.H.empty() || std::abs(determinant(matches_info.H)) < std::numeric_limits<double>::epsilon())
            //        {
            //            std::cout << "matches_info.H is empty." << std::endl;
            //            continue;
            //        }
            if (!matches_info.H.empty() && std::abs(determinant(matches_info.H)) >= std::numeric_limits<double>::epsilon())
                // Find number of inliers
            {
                
                matches_info.num_inliers = 0;
                for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
                    if (matches_info.inliers_mask[i])
                        matches_info.num_inliers++;
                //std::cout << "BestOf2NearestMatcher::match-> matches_info.num_inliers is " << matches_info.num_inliers<< std::endl;
                // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic Image Stitching
                // using Invariant Features"
                matches_info.confidence = matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());
                //  std::cout << "BestOf2NearestMatcher::match-> matches_info.confidence is " << matches_info.confidence  << std::endl;
                if(matches_info.confidence > 3) //zt
                {
                    if(!strLogPath.empty())
                    {
                        output_file << "LOGINFO: two pictures matches is too close." << std::endl;
                    }
                    else
                        std::cout << "LOGINFO: two pictures matches is too close." << std::endl;
                }
                // Set zero confidence to remove matches between too close global_images, as they don't provide
                // additional information anyway. The threshold was set experimentally.
                matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;
                
                ///////zt
                // Check if we should try to refine motion
                if (matches_info.num_inliers >= 6)
                {
                    //zt add 新加判断匹配是否符合拼接条件
                    //如果当前图片匹配失败, 需要删掉当前图片以及当前图片的特征和匹配的信息,并返回
                    if(matches_info.num_inliers < inliner_num || matches_info.confidence < conf_thresh)
                    {
                        MatchError++; //当前图片与之前每一次的匹配是否满足匹配条件，
                    }
                    
                    // Construct point-point correspondences for inliers only
                    src_points.create(1, matches_info.num_inliers, CV_32FC2);
                    dst_points.create(1, matches_info.num_inliers, CV_32FC2);
                    int inlier_idx = 0;
                    for (size_t i = 0; i < matches_info.matches.size(); ++i)
                    {
                        if (!matches_info.inliers_mask[i])
                            continue;
                        
                        const DMatch& m = matches_info.matches[i];
                        
                        Point2f p = global_img_feature[from].keypoints[m.queryIdx].pt;
                        
                        p.x -= global_img_feature[from].img_size.width * 0.5f;
                        p.y -= global_img_feature[from].img_size.height * 0.5f;
                        src_points.at<Point2f>(0, inlier_idx) = p;
                        
                        p = global_img_feature[to].keypoints[m.trainIdx].pt;
                        p.x -= global_img_feature[to].img_size.width * 0.5f;
                        p.y -= global_img_feature[to].img_size.height * 0.5f;
                        dst_points.at<Point2f>(0, inlier_idx) = p;
                        
                        inlier_idx++;
                    }
                    
                    // Rerun motion estimation on inliers only
                    matches_info.H = findHomography(src_points, dst_points, RANSAC);
                    pairwise_matches[pair_idx] = matches_info; //zt
                }
                else
                {
                    MatchError++;
                }
            }
            else
            {
                MatchError++;
            }
        } else
            MatchError++;
        
        pairwise_matches[pair_idx].src_img_idx = from;
        pairwise_matches[pair_idx].dst_img_idx = to;
        
        size_t dual_pair_idx = to * num_images + from;
        // std::cout << "MatchPairsBody -> dual_pair_idx is " << dual_pair_idx << std::endl;
        pairwise_matches[dual_pair_idx] = pairwise_matches[pair_idx];
        pairwise_matches[dual_pair_idx].src_img_idx = to;
        pairwise_matches[dual_pair_idx].dst_img_idx = from;
        
        if (!pairwise_matches[pair_idx].H.empty())
            pairwise_matches[dual_pair_idx].H = pairwise_matches[pair_idx].H.inv();
        
        for (size_t j = 0; j < pairwise_matches[dual_pair_idx].matches.size(); ++j)
            std::swap(pairwise_matches[dual_pair_idx].matches[j].queryIdx,
                      pairwise_matches[dual_pair_idx].matches[j].trainIdx);
        
        Dmatcher.release(); //zt
        //zt
        if(pairwise_matches[pair_idx].src_img_idx != -1)
        {
            tmp_pairwise_matches.push_back(pairwise_matches[pair_idx]);
            tmp_pairwise_matches.push_back(pairwise_matches[dual_pair_idx]);
        }
        
        gettimeofday( &submatch_end, NULL );
        //求出两次时间的差值，单位为us
        int submatch_timeuse = 1000000 * ( submatch_end.tv_sec - submatch_start.tv_sec ) + submatch_end.tv_usec - submatch_start.tv_usec;
        //printf("submatch time: %d us\n", submatch_timeuse);
    }
    
    if(num_images == 2)  // 直接返回结果
    {
        global_pairwise_matches.push_back(pairwise_matches[0]);
        for(int i = 0; i < num_images; ++i)
            global_pairwise_matches.push_back(tmp_pairwise_matches[i]);
        global_pairwise_matches.push_back(pairwise_matches[0]);
    }
    // 因为 temp 里面的都是成对出现, 如0-1 和 1-0, 0-2 和 2-0, 1-2 和 2-1, 0-3 和 3-0 , 所以下面的直接取除temp元素,不需要再取排序
    // 下面每次insert 完可以将temp里面的erase 掉, 这里我没做这个
    else if(num_images == 3)  // 先将0-2 插进去, 再将1-2 插到end(), 再 2-0 和 2-1
    {
        global_pairwise_matches.insert(global_pairwise_matches.begin() + num_images - 1,tmp_pairwise_matches[0]);// 将0-2 插进去
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            //先将 1-2 插进去,
            if(tmp_pairwise_matches[i].src_img_idx != 0 && tmp_pairwise_matches[i].dst_img_idx == num_images - 1)
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
        }
        //再插 2-0 和 2-1
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            if (tmp_pairwise_matches[i].src_img_idx == num_images - 1)
                global_pairwise_matches.insert(global_pairwise_matches.end(), tmp_pairwise_matches[i]);
        }
        global_pairwise_matches.push_back(global_pairwise_matches[0]);
    }
    else  //num_images > 3
    {
        // global_pairwise_matches.insert(global_pairwise_matches.begin() + num_images - 1,tmp_pairwise_matches[0]);
        //查找每次新的 (0-(num_images -1)) ,(0-(num_image - 2)),(1-(num_images - 1)),...,插入到对应位置
        for (int i = 0; i < tmp_pairwise_matches.size(); i++ ){
            for(int j = 0; j < global_pairwise_matches.size(); ++j)
            {
                if(global_pairwise_matches[j].src_img_idx == tmp_pairwise_matches[i].src_img_idx && global_pairwise_matches[j].dst_img_idx == num_images - 2)
                {
                    global_pairwise_matches.insert(global_pairwise_matches.begin() + j + 1,tmp_pairwise_matches[i]);
                    tmp_pairwise_matches.erase(tmp_pairwise_matches.begin() + i);
                }
            }
        }
        //最后几个上面没有对应的位置,直接插到末尾
        for(int i = 0; i < tmp_pairwise_matches.size(); ++i)
        {
            if(tmp_pairwise_matches[i].src_img_idx != num_images - 1)
            {
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
                tmp_pairwise_matches.erase(tmp_pairwise_matches.begin() + i);
            }
        }
        //从 (num_imagse -1 )-0 , (num_imagse -1 )-1, (num_imagse -1 )-2, ... 插入末尾
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            
            if(tmp_pairwise_matches[i].src_img_idx == num_images - 1 )
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
        }
        //最后一个跟自己匹配的结果(num_images - 1)-(num_images - 1), 内容和第一个一样,直接插入
        global_pairwise_matches.insert(global_pairwise_matches.end(),pairwise_matches[0]);
    }
    gettimeofday( &match_end, NULL );
    //求出两次时间的差值，单位为us
    int match_timeuse = 1000000 * ( match_end.tv_sec - match_start.tv_sec ) + match_end.tv_usec - match_start.tv_usec;
    
    if(!strLogPath.empty())
        output_file << "LOGINFO: Matcher Feature time is " << match_timeuse << "  us."<< std::endl;
    else
    {
        printf("match time: %d us\n", match_timeuse);
    }
    
    
    for(int i = 0; i < global_pairwise_matches.size(); ++i)
    {
        if(!strLogPath.empty()) {
            output_file << "src = " << global_pairwise_matches[i].src_img_idx << " , dst = "
            << global_pairwise_matches[i].dst_img_idx;
            output_file << " , num_inliers = " << global_pairwise_matches[i].num_inliers << " , confidence = "
            << global_pairwise_matches[i].confidence << std::endl;
        }
    }
    
    //当前图片与之前匹配不满足条件的个数如果等于所有匹配次数，则此次匹配失败，删除保存的当前图片和特征, 返回拼接失败的结果
    //    if(MatchError == num_images - 1)
    //    {
    //        global_images.pop_back();
    //        global_input_images.pop_back();
    //        global_img_feature.pop_back();
    //        stitchflag = false;
    //
    //        if(!strLogPath.empty())
    //            output_file << "Match error = num_images - 1 ." << std::endl;
    //        else
    //            std::cout << "Match error == num_images - 1 ." << std::endl;
    //        return false;
    //    }
    
    //判断哪些是不能拼接的, 在matcher里面我已经通过设置匹配置信度和内点数量进行了判断
    vector<int> indices;
    indices.clear();
    vector <ImageFeatures> img_feature_tmp;
    img_feature_tmp.clear();
    img_feature_tmp = global_img_feature;
    vector <MatchesInfo> pairwise_matches_tmp;
    pairwise_matches_tmp.clear();
    pairwise_matches_tmp = global_pairwise_matches;
    indices = leaveBiggestComponent(img_feature_tmp, pairwise_matches_tmp, conf_thresh);
    if(indices.size() < num_images)
    {
        if(!strLogPath.empty())
        {
            output_file << "indice size < num_images." << std::endl;
            output_file.close();
        }
        else
            std::cout << "indice size < num_images." << std::endl;
        global_images.pop_back();
        global_input_images.pop_back();
        global_img_feature.pop_back();
        
        all_images.pop_back();
        all_input_images.pop_back();
        all_img_feature.pop_back();
        
        //前面拼接失败,需要删除前面最近一次的匹配信息
        global_pairwise_matches.pop_back();
        if(InputImgNums == 2)
            global_pairwise_matches.clear();
        else {
            int srcMaxIdx = 0, dstMaxIdx = 0;
            global_pairwise_matches.pop_back();
            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                    srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                    dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
            }
            
            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                    global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                    i = -1;
                }
            }
        }
        
        return false;
    }
    
    //寻找最大匹配
    vector<int> pair_idx;
    vector<double> pair_confidence;
    vector<int> pair_points;
    for (int i = 0; i < global_img_feature.size(); ++i) {
        pair_idx.push_back(-1);
        pair_confidence.push_back(0.0);
        pair_points.push_back(0);
    }
    int src_img_idx;
    int dst_img_idx;
    double confidence;
    int num_inliers;
    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
        src_img_idx = global_pairwise_matches[i].src_img_idx;
        dst_img_idx = global_pairwise_matches[i].dst_img_idx;
        confidence = global_pairwise_matches[i].confidence;
        num_inliers = global_pairwise_matches[i].num_inliers;
        if (pair_idx[src_img_idx] == -1) {
            pair_idx[src_img_idx] = dst_img_idx;
            pair_confidence[src_img_idx] = confidence;
            pair_points[src_img_idx] = num_inliers;
        } else if (confidence > pair_confidence[src_img_idx]) {
            pair_idx[src_img_idx] = dst_img_idx;
            pair_confidence[src_img_idx] = confidence;
            pair_points[src_img_idx] = num_inliers;
        }
    }
    
    bool bflag = false;//zt
    int ImgNum = global_img_feature.size(); //这里的ImgNum包括当前传进来的图片
    for (int i = 0; i < ImgNum; ++i) {
        if (pair_points[i] <= inliner_num) {
            bflag = true;
        }
    }
    
    if(!strLogPath.empty())
    {
        if(bflag)
        {
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if(InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            
            output_file << "LOGINFO: inliner_num < 16" << std::endl;
            output_file.close(); //zt
            return false; //zt
        }
        
    } else
    {
        if(bflag)
        {
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            std::cout << "LOGINFO: inliner_num < 16" << std::endl;
            
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if(InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            
            return false; //zt
        }
    }
    
    vector<CameraParams> cameras;
    struct timeval eatimator_start, estimator_end;
    gettimeofday( &eatimator_start, NULL );
    
    if(!strLogPath.empty())
    {
        if (!(*estimator)(global_img_feature, global_pairwise_matches, cameras)){
            output_file << "LOGINFO: camera estimat failed." << std::endl; //zt
            output_file.close(); //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if(InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            
            return false;
        }
    }
    else
    {
        if (!(*estimator)(global_img_feature, global_pairwise_matches, cameras)){
            std::cout << "LOGINFO: camera estimat failed." << std::endl; //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if(InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            
            return false;
        }
    }
    gettimeofday( &estimator_end, NULL );
    //求出两次时间的差值，单位为us
    int estimator_timeuse = 1000000 * ( estimator_end.tv_sec - eatimator_start.tv_sec ) + estimator_end.tv_usec - eatimator_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "estimator time is " << estimator_timeuse  << "  us."<< std::endl;
    else
        printf("estimator_ time: %d us\n", estimator_timeuse);
    
    for (size_t i = 0; i < cameras.size(); ++i){
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
    }
    
    ///log
    for (size_t i = 0; i < cameras.size(); ++i) {
        std::cout << "R is " << cameras[i].R << std::endl;
        std::cout << "t is " << cameras[i].t << std::endl;
        std::cout << "ppx = " << cameras[i].ppx << ", ppy = " << cameras[i].ppy << std::endl;
        std::cout << "focal = " << cameras[i].focal << std::endl;
        std::cout << "aspect = " << cameras[i].aspect << std::endl;
    }
    ///log
    struct timeval adjust_start, adjust_end;
    gettimeofday( &adjust_start, NULL);
    
    if(!strLogPath.empty())
    {
        if (!(*adjuster)(global_img_feature, global_pairwise_matches, cameras)){
            //if(!bundleadjust(global_img_feature, global_pairwise_matches, cameras)){
            output_file << "LOGERROR: camera adjuster failed." << std::endl; //zt
            output_file.close(); //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if(InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            
            return false;
        }
        // check camera parameter R
        CameraParams camera;
        for (int i = 0; i < cameras.size(); ++i) {
            camera = cameras[i];
            for (int row = 0; row < camera.R.rows; row++) {
                for (int col = 0; col < camera.R.cols; col++) {
                    if (isnan(camera.R.at<int>(row,col))) {
                        output_file << "LOGINFO: camera check failed." << std::endl; //zt
                        output_file.close(); //zt
                        global_images.pop_back();
                        global_input_images.pop_back();
                        global_img_feature.pop_back();
                        
                        all_images.pop_back();
                        all_input_images.pop_back();
                        all_img_feature.pop_back();
                        
                        //前面拼接失败,需要删除前面最近一次的匹配信息
                        global_pairwise_matches.pop_back();
                        if(InputImgNums == 2)
                            global_pairwise_matches.clear();
                        else {
                            int srcMaxIdx = 0, dstMaxIdx = 0;
                            global_pairwise_matches.pop_back();
                            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                                if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                                    srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                                if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                                    dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                            }
                            
                            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                                if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                                    global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                                    i = -1;
                                }
                            }
                        }
                        
                        return false;
                    }
                }
            }
        }
        
    }
    else
    {
        if (!(*adjuster)(global_img_feature, global_pairwise_matches, cameras)){
            std::cout << "LOGERROR: camera adjuster failed." << std::endl; //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if(InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            return false;
        }
        // check camera parameter R
        CameraParams camera;
        for (int i = 0; i < cameras.size(); ++i) {
            camera = cameras[i];
            for (int row = 0; row < camera.R.rows; row++) {
                for (int col = 0; col < camera.R.cols; col++) {
                    if (isnan(camera.R.at<int>(row,col))) {
                        std::cout << "LOGINFO: camera check failed." << std::endl; //zt
                        global_images.pop_back();
                        global_input_images.pop_back();
                        global_img_feature.pop_back();
                        
                        all_images.pop_back();
                        all_input_images.pop_back();
                        all_img_feature.pop_back();
                        
                        //前面拼接失败,需要删除前面最近一次的匹配信息
                        global_pairwise_matches.pop_back();
                        if(InputImgNums == 2)
                            global_pairwise_matches.clear();
                        else {
                            int srcMaxIdx = 0, dstMaxIdx = 0;
                            global_pairwise_matches.pop_back();
                            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                                if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                                    srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                                if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                                    dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                            }
                            
                            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                                if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                                    global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                                    i = -1;
                                }
                            }
                        }
                        return false;
                    }
                }
            }
        }
    }
    
    gettimeofday( &adjust_end, NULL );
    //求出两次时间的差值，单位为us
    int adjust_timeuse = 1000000 * ( adjust_end.tv_sec - adjust_start.tv_sec ) + adjust_end.tv_usec - adjust_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "adjust time is " << adjust_timeuse << "  us." << std::endl;
    else
        printf("adjust time: %d us\n", adjust_timeuse);
    
    camera_params = cameras;
    //大于最大匹配数保存当前图片的相机参数  //zt
    //    if(InputImgNums <= MaxImgNums_)
    //        global_camera_params = cameras;
    //    else
    //        global_camera_params.push_back(cameras[MaxImgNums_ - 1]);
    
    vector<double> focals;
    focals.clear(); //zt
    for (size_t i = 0; i < cameras.size(); ++i){
        focals.push_back(cameras[i].focal);
    }
    
    sort(focals.begin(), focals.end());
    float warped_image_scale;
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;
    
    vector<Point> corners(global_input_images.size());
    vector<UMat> masks_warped(global_input_images.size());
    vector<UMat> images_warped(global_input_images.size());
    vector<Size> sizes(global_input_images.size());
    vector<UMat> masks(global_input_images.size());
    
    
    struct timeval warp_start, warp_end;
    gettimeofday( &warp_start, NULL );
    
#ifndef WARP
    // Preapre images masks
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(global_images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }
    
    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
    
    for (int i = 0; i < num_images; ++i)
    {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;
        
        //check
        Rect roi = warper->warpRoi(global_images[i].size(), K, cameras[i].R);
        
        if (roi.width > pano_max_edge || roi.height > pano_max_edge){
            if(!strLogPath.empty())
            {   output_file << "LOGERROR： roi.width or roi.height > pano_max_edge. "<< std::endl;
                output_file.close();
                global_images.pop_back();
                global_input_images.pop_back();
                global_img_feature.pop_back();
                
                all_images.pop_back();
                all_input_images.pop_back();
                all_img_feature.pop_back();
                
                //前面拼接失败,需要删除前面最近一次的匹配信息
                global_pairwise_matches.pop_back();
                if(InputImgNums == 2)
                    global_pairwise_matches.clear();
                else {
                    int srcMaxIdx = 0, dstMaxIdx = 0;
                    global_pairwise_matches.pop_back();
                    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                        if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                            srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                        if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                            dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                    }
                    
                    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                        if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                            global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                            global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                            i = -1;
                        }
                    }
                }
                return false;
            }
            else
            {
                std::cout << "LOGERROR： roi.width or roi.height > pano_max_edge. "<< std::endl;
                global_images.pop_back();
                global_input_images.pop_back();
                global_img_feature.pop_back();
                
                all_images.pop_back();
                all_input_images.pop_back();
                all_img_feature.pop_back();
                
                //前面拼接失败,需要删除前面最近一次的匹配信息
                global_pairwise_matches.pop_back();
                if(InputImgNums == 2)
                    global_pairwise_matches.clear();
                else {
                    int srcMaxIdx = 0, dstMaxIdx = 0;
                    global_pairwise_matches.pop_back();
                    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                        if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                            srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                        if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                            dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                    }
                    
                    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                        if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                            global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                            global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                            i = -1;
                        }
                    }
                }
                return false;
            }
        }
        
        corners[i] = warper->warp(global_images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();
        
        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
    }
    
    vector<UMat> images_warped_f(num_images);
    for (int i = 0; i < num_images; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);
    
    gettimeofday( &warp_end, NULL );
    //求出两次时间的差值，单位为us
    int warp_timeuse = 1000000 * ( warp_end.tv_sec - warp_start.tv_sec ) + warp_end.tv_usec - warp_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "warp time is " << warp_timeuse << "  us." << std::endl;
    else
        printf("warp time: %d us\n", warp_timeuse);
    //seam time
    struct timeval seam_start, seam_end;
    gettimeofday( &seam_start, NULL );
    
    seam_finder->find(images_warped_f, corners, masks_warped);
    gettimeofday( &seam_end, NULL );
    //求出两次时间的差值，单位为us
    int seam_timeuse = 1000000 * ( seam_end.tv_sec - seam_start.tv_sec ) + seam_end.tv_usec - seam_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "seam time is " << seam_timeuse << "  us." << std::endl;
    else
        printf("seam time: %d us\n", seam_timeuse);
    
    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    Mat input_img;
    double compose_seam_aspect = 1;
    double compose_work_aspect = 1;
    
    struct timeval compose_start, compose_end;
    gettimeofday( &compose_start, NULL );
    
    if (!is_compose_scale_set ) {
        compose_scale = min(1.0, sqrt(compose_megapix * MaxPix_ / input.size().area()));
        is_compose_scale_set = true;
    }
    
    // Compute relative scales
    compose_seam_aspect = compose_scale / seam_scale;
    compose_work_aspect = compose_scale / work_scale;
    
    // Update warped image scale
    warped_image_scale *= static_cast<float>(compose_work_aspect);
    warper = warper_creator->create(warped_image_scale);
    
    // Update corners and sizes
    for (int i = 0; i < num_images; ++i)
    {
        // Update intrinsics
        cameras[i].focal *= compose_work_aspect;
        cameras[i].ppx *= compose_work_aspect;
        cameras[i].ppy *= compose_work_aspect;
        
        // Update corner and size
        Size sz = input.size();
        if (std::abs(compose_scale - 1) > 1e-1)
        {
            sz.width = cvRound(input.size().width * compose_scale);
            sz.height = cvRound(input.size().height * compose_scale);
        }
        
        Mat K;
        cameras[i].K().convertTo(K, CV_32F);
        Rect roi = warper->warpRoi(sz, K, cameras[i].R);
        corners[i] = roi.tl();
        sizes[i] = roi.size();
    }
    
    for (int img_idx = 0; img_idx < num_images; ++img_idx)
    {
        // Read image and resize it if necessary
        input_img = global_input_images[img_idx].clone();
        
        if (abs(compose_scale - 1) > 1e-1)
            cv::resize(input_img, img, Size(), compose_scale, compose_scale, INTER_LINEAR_EXACT);
        else
            img = input_img.clone();
        
        input_img.release();
        Size img_size = img.size();
        
        Mat K;
        cameras[img_idx].K().convertTo(K, CV_32F);
        
        // Warp the current image
        warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);
        
        // Warp the current image mask
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);
        
        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();
        
        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
        mask_warped = seam_mask & mask_warped;
        
        if (!blender && !timelapse)
        {
            blender = Blender::createDefault(blend_type, try_cuda);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, try_cuda);
            else if (blend_type == Blender::MULTI_BAND)
            {
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
            }
            blender->prepare(corners, sizes);
        }
        // Blend the current image
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }
    
    Mat result, result_mask;
    blender->blend(result, result_mask);
    
    gettimeofday( &compose_end, NULL );
    //求出两次时间的差值，单位为us
    int compose_timeuse = 1000000 * ( compose_end.tv_sec - compose_start.tv_sec ) + compose_end.tv_usec - compose_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "compose time is " << compose_timeuse << "  us." << std::endl;
    else
        printf("compose time: %d us\n", compose_timeuse);
    
    result.copyTo(dst);
    pano_vec.push_back(dst); //save pano ,by //zt
    result_size = result.size();
    //zt log camera parameters
    if(!strLogPath.empty())
    {
        for (size_t tt = 0; tt < cameras.size(); ++tt)
        {
            output_file << " LOGINFO: camera information: " << std::endl;  //zt
            output_file << "    aspect is " << cameras[tt].aspect << endl;
            output_file << "    focal is " << cameras[tt].focal << endl;
            output_file << "    Principal point X is " << cameras[tt].ppx  << ", Principal point Y is " << cameras[tt].ppy << std::endl;
            output_file << "    R matrix is " << cameras[tt].R << std::endl;
            output_file << "    t matrix is " << cameras[tt].t << std::endl;
        }
        // zt log pano size
        output_file << " LOGINFO: pano size is " << result_size << std::endl; //zt
    }
    
#endif
    gettimeofday( &stitch_end, NULL );
    //求出两次时间的差值，单位为us
    int stitch_timeuse = 1000000 * ( stitch_end.tv_sec - stitch_start.tv_sec ) + stitch_end.tv_usec - stitch_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "total time is " << stitch_timeuse << "  us." << std::endl;
    else
        printf("total time: %d us\n", stitch_timeuse);
    
    if(!strLogPath.empty())
    {
        output_file.close(); //zt
    }
    InputImgNums ++;
    return true;
}
#endif

#ifndef NEWMETHOD_RETAKE //20190509 当前图片不在全景图认为拼接失败,没有相机参数和坐标,
bool stitching(std::vector <cv::Mat> &imglist, Mat &dst, int mode, std::string & strLogPath){
    if (mode == 1 ){
        if (all_images.size() > 0){
            delete_pic(all_images.size());
            if (pano_vec.size() > 0){
                pano_vec[pano_vec.size() - 1].copyTo(dst);
            }
            return true;
        }
        else
            return false;
    }
    
    struct timeval stitch_start, stitch_end;
    gettimeofday( &stitch_start, NULL );
    
    ofstream output_file;
    if(!strLogPath.empty())
    {
        output_file.open(strLogPath, ios::out); //zt
        if(imglist.size() == 0)
        {
            output_file << "LOGERROR: input images = 0." << std::endl;
            output_file.close();
            return false;
        }
        if(imglist.size() > 1)
        {
            output_file << "LOGERROR: input images > 1, please input just one img." << std::endl;
            output_file.close();
            return false;
        }
        
        if(imglist[0].empty())
        {
            output_file << "LOGERROR: input img is empty." << std::endl;
            output_file.close();
            return false;
        }
    }
    else
    {
        if(imglist.size() == 0)
        {
            printf("input images = 0.");
            return false;
        }
        if(imglist.size() > 1)
        {
            printf("input images > 1, please input just one img");
            return false;
        }
        
        if(imglist[0].empty())
        {
            printf("input img is empty.");
            return false;
        }
    }
    
    if (!is_input_scale_set) {
        if (work_pixel == 0){
            input_scale = min(1.0, sqrt(input_megapix * MaxPix_ / imglist[0].size().area()));
            is_input_scale_set = true;
        }
        else{
            if (min(imglist[0].rows, imglist[0].cols)<= work_pixel){
                input_scale = 1;
                is_input_scale_set = true;
            }
            else{
                input_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                is_input_scale_set = true;
            }
        }
    }
    //每次只能送一张进行stitching
    if(InputImgNums == 0)
        InputImgNums = imglist.size(); // 当前图片就1张
    
    if(InputImgNums == 1)  //first image , return current img as pino.jpg
    {
        dst = imglist[0];
        pano_vec.push_back(dst); //zt
        InputImgNums++;
        ImageFeatures featureTemp;
        Mat img;
        resize(imglist[0], input, Size(), input_scale, input_scale, INTER_NEAREST);
        
        if (work_megapix < 0)
        {
            img = input.clone();
            work_scale = 1;
            is_work_scale_set = true;
        }
        else {
            if (!is_work_scale_set) {
                if (work_pixel == 0){
                    work_scale = min(1.0, sqrt(work_megapix * MaxPix_ / input.size().area()));
                    is_work_scale_set = true;
                }
                else{
                    if (min(imglist[0].rows, imglist[0].cols)<= work_pixel){
                        work_scale = 1;
                        is_work_scale_set = true;
                    }
                    else{
                        work_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                        is_work_scale_set = true;
                    }
                }
            }
            resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
        }
        if (!is_seam_scale_set)
        {
            seam_scale = min(1.0, sqrt(seam_megapix * MaxPix_ / input.size().area()));
            seam_work_aspect = seam_scale / work_scale;
            is_seam_scale_set = true;
        }
        //find feature
        struct timeval first_find_start, first_find_end;
        gettimeofday( &first_find_start, NULL );
        (*finder)(img, featureTemp);
        gettimeofday( &first_find_end, NULL );
        //求出两次时间的差值，单位为us
        int firstfindtimeuse = 1000000 * ( first_find_end.tv_sec - first_find_start.tv_sec ) + first_find_end.tv_usec - first_find_start.tv_usec;
        if(!strLogPath.empty())
            output_file << "first img find time is " << firstfindtimeuse << "  us."<< std::endl;
        else
            printf("first img find time: %d us\n", firstfindtimeuse);
        
        featureTemp.img_idx = 0;
        global_img_feature.push_back(featureTemp); //debug
        all_img_feature.push_back(featureTemp);
        finder->collectGarbage();
        
        if (try_compose == true)
            resize(input, img, Size(), seam_scale, seam_scale, INTER_NEAREST);
        else {
            if (!is_compose_scale_set) {
                if (work_pixel == 0) {
                    compose_scale = min(1.0, sqrt(compose_scale * MaxPix_ / input.size().area()));
                    is_compose_scale_set = true;
                } else {
                    if (min(imglist[0].rows, imglist[0].cols) <= work_pixel) {
                        compose_scale = 1;
                        is_compose_scale_set = true;
                    } else {
                        compose_scale = work_pixel * 1. / min(imglist[0].rows, imglist[0].cols);
                        is_compose_scale_set = true;
                    }
                }
            }
            resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
        }
        global_input_images.push_back(input.clone());//debug
        all_input_images.push_back(input.clone());
        global_images.push_back(img.clone());  //debug
        all_images.push_back(img.clone());
        if(!strLogPath.empty())
            output_file.close();
        return true;
    }
    
    std::cout << "global_input_images size is " << global_input_images.size() << std::endl;
    std::cout << "global_images size is " << global_images.size() << std::endl;
    std::cout << "global_img_feature size is " << global_img_feature.size() << std::endl;
#ifdef ORI0416
    if(!stitchflag)  //前面拼接失败,需要删除前面最近一次的匹配信息
    {
        global_pairwise_matches.pop_back();
        
        if(InputImgNums == 2)
            global_pairwise_matches.clear();
        else if(InputImgNums <= MaxImgNums_ && InputImgNums > 2)
        {
            
            int srcMaxIdx = 0, dstMaxIdx = 0;
            global_pairwise_matches.pop_back();
            for(int i= 0; i< global_pairwise_matches.size(); ++i)
            {
                if(global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                    srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                if(global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                    dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
            }
            
            for(int i = 0; i < global_pairwise_matches.size(); ++i)
            {
                if(global_pairwise_matches[i].src_img_idx == srcMaxIdx || global_pairwise_matches[i].dst_img_idx == dstMaxIdx)
                {
                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                    i = -1;
                }
            }
            
            //            for(int i = 0; i < global_pairwise_matches.size(); ++i)
            //            {
            //                if(global_pairwise_matches[i].src_img_idx == InputImgNums - 1 || global_pairwise_matches[i].dst_img_idx == InputImgNums - 1)
            //                {
            //                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
            //                    i = -1;
            //                }
            //            }
        } else  //大于最大匹配数
        {
            
            for(int i = 0; i < global_pairwise_matches.size(); ++i)
            {
                if(global_pairwise_matches[i].src_img_idx == MaxImgNums_ - 1 || global_pairwise_matches[i].dst_img_idx == MaxImgNums_ - 1)
                {
                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                    i = -1;
                }
            }
        }
        
    }
#endif
    
    // 当前图片数量 > MaxImgNums_ ,当前global只保存了输入图片数量 - 1 的图片数,并且前面拼接成功,删除保存向量的首个元素, 后面的元素坐标向前退1, 删除当前最大匹配图片的首张图片和特征
    if(global_img_feature.size() >= MaxImgNums_)
    {
        //erase 图片和特征
        global_img_feature.erase(global_img_feature.begin());
        global_images.erase(global_images.begin());
        global_input_images.erase(global_input_images.begin());
        
        //删除匹配信息
        global_pairwise_matches.erase(global_pairwise_matches.begin());  //删除第一个元素
        int matchSize = global_pairwise_matches.size();
        //删除保存向量的第一张图片的匹配信息
        for(int i = 0; i < matchSize; ++i)
        {
            if(global_pairwise_matches[i].src_img_idx == 0 || global_pairwise_matches[i].dst_img_idx == 0)
            {
                global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                i = -1; //执行 ++i将 i 重置为0
            }
        }
        int matchSizeModi = global_pairwise_matches.size();
        //后面的元素坐标向前退1
        for(int i = 0; i < matchSizeModi; ++i)
        {
            if(global_pairwise_matches[i].src_img_idx != -1 )//|| global_pairwise_matches[i].dst_img_idx != -1)
            {
                global_pairwise_matches[i].src_img_idx = global_pairwise_matches[i].src_img_idx - 1;
                global_pairwise_matches[i].dst_img_idx = global_pairwise_matches[i].dst_img_idx - 1;
            }
        }
    }
    
    Mat img;
    resize(imglist[0], input, Size(), input_scale, input_scale, INTER_NEAREST);
    
    if (work_megapix < 0)
    {
        img = input.clone();
        work_scale = 1;
        is_work_scale_set = true;
    }
    else {
        if (!is_work_scale_set) {
            if (work_pixel == 0){
                work_scale = min(1.0, sqrt(work_megapix * MaxPix_ / input.size().area()));
                is_work_scale_set = true;
            }
            else{
                if (min(imglist[0].rows, imglist[0].cols) <= work_pixel){
                    work_scale = 1;
                    is_work_scale_set = true;
                }
                else{
                    work_scale = work_pixel*1./min(imglist[0].rows, imglist[0].cols);
                    is_work_scale_set = true;
                }
            }
        }
        resize(input, img, Size(), work_scale, work_scale, INTER_NEAREST);
    }
    if (!is_seam_scale_set)
    {
        seam_scale = min(1.0, sqrt(seam_megapix * MaxPix_ / input.size().area()));
        seam_work_aspect = seam_scale / work_scale;
        is_seam_scale_set = true;
    }
    
    ImageFeatures feature;
    //find feature
    struct timeval find_start, find_end;
    gettimeofday( &find_start, NULL );
    (*finder)(img, feature);
    
    gettimeofday( &find_end, NULL );
    //求出两次时间的差值，单位为us
    int find_timeuse = 1000000 * ( find_end.tv_sec - find_start.tv_sec ) + find_end.tv_usec - find_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "LOGINFO: Finder time is " << find_timeuse << " us."<< std::endl;
    else
        printf("Finder time: %d us\n", find_timeuse);
    
    feature.img_idx = InputImgNums - 1;
    
    if (try_compose == true)
        resize(input, img, Size(), seam_scale, seam_scale, INTER_NEAREST);
    else {
        if (!is_compose_scale_set) {
            if (work_pixel == 0) {
                compose_scale = min(1.0, sqrt(compose_scale * MaxPix_ / input.size().area()));
                is_compose_scale_set = true;
            } else {
                if (min(imglist[0].rows, imglist[0].cols) <= work_pixel) {
                    compose_scale = 1;
                    is_compose_scale_set = true;
                } else {
                    compose_scale = work_pixel * 1. / min(imglist[0].rows, imglist[0].cols);
                    is_compose_scale_set = true;
                }
            }
        }
        resize(input, img, Size(), compose_scale, compose_scale, INTER_NEAREST);
    }
    
    global_input_images.push_back(input.clone());
    global_images.push_back(img.clone());
    global_img_feature.push_back(feature);
    
    
    all_input_images.push_back(input.clone());
    all_images.push_back(img.clone());
    all_img_feature.push_back(feature);
    
    finder->collectGarbage();
    
    // 将matcher 分拆 ///////////////////////////////////////
    vector<MatchesInfo> pairwise_matches;
    pairwise_matches.clear();
    
    struct timeval match_start, match_end;
    gettimeofday( &match_start, NULL );
    
    int num_images = global_img_feature.size();
    
    // 创建匹配 pair, 每次新进来的要clear
    std::vector<std::pair<int,int> > near_pairs;
    near_pairs.clear();
    
    for(int idx = 0; idx < global_img_feature.size() - 1; ++idx)
        if (global_img_feature[global_img_feature.size() - 1].keypoints.size() > 0 && global_img_feature[0].keypoints.size() > 0)
            near_pairs.push_back(std::make_pair(idx,global_img_feature.size() - 1));
    
    pairwise_matches.resize(num_images * num_images);
    
    cv::RNG rng = cv::theRNG(); // save entry rng state
    Range r = Range(0, static_cast<int>(near_pairs.size()));
    
    vector <MatchesInfo> tmp_pairwise_matches;
    tmp_pairwise_matches.clear();// zt
    int MatchError = 0; //定义新进来的图片与之前图片匹配的错误数量，根据此来作为后续判断当前图片是否可以与之前图片拼接
    for (int i = r.start; i < r.end; ++i) {
        struct timeval submatch_start, submatch_end;
        gettimeofday( &submatch_start, NULL );
        // std::cout << "r size is " << r.size() << ",   rng is " << rng.state + i << std::endl;
        //cv::theRNG() = cv::RNG(rng.state + i); // force "stable" RNG seed for each processed pair
        //std::cout << "MatchPairsBody -> rng state = " << rng.state << std::endl;
        cv::theRNG() = cv::RNG(4294967295); //4294967295
        
        int from = near_pairs[i].first;
        int to = near_pairs[i].second;
        int pair_idx = from * num_images + to;
        
        MatchesInfo matches_info;
        matches_info.matches.clear();//zt
        //matcher(features[from], features[to], pairwise_matches[pair_idx]); // 先进BestOf2NearestMatcher::match ，再调用CpuMatcher match
        CV_Assert(global_img_feature[from].descriptors.type() == global_img_feature[to].descriptors.type());
        CV_Assert(global_img_feature[to].descriptors.depth() == CV_8U || global_img_feature[to].descriptors.depth() == CV_32F);
        Ptr<cv::DescriptorMatcher> Dmatcher;
        {
            Ptr<flann::IndexParams> indexParams = makePtr<flann::KDTreeIndexParams>();
            Ptr<flann::SearchParams> searchParams = makePtr<flann::SearchParams>();
            
            if (global_img_feature[to].descriptors.depth() == CV_8U) {
                indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
                searchParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
            }
            
            Dmatcher = makePtr<FlannBasedMatcher>(indexParams, searchParams);
        }
        std::vector<std::vector<DMatch> > pair_matches;
        pair_matches.clear();
        MatchesSet matches;
        
        // Find 1->2 matches
        Dmatcher->knnMatch(global_img_feature[from].descriptors, global_img_feature[to].descriptors, pair_matches, 2);
        //
        for (size_t i = 0; i < pair_matches.size(); ++i) {
            if (pair_matches[i].size() < 2)
                continue;
            const DMatch &m0 = pair_matches[i][0];
            const DMatch &m1 = pair_matches[i][1];
            if (m0.distance < (1.f - match_conf) * m1.distance) {
                matches_info.matches.push_back(m0);
                matches.insert(std::make_pair(m0.queryIdx, m0.trainIdx));
            }
        }
        // LOG("\n1->2 matches: " << matches_info.matches.size() << endl);
        // Find 2->1 matches
        pair_matches.clear();
        Dmatcher->knnMatch(global_img_feature[to].descriptors, global_img_feature[from].descriptors, pair_matches, 2);
        for (size_t i = 0; i < pair_matches.size(); ++i) {
            if (pair_matches[i].size() < 2)
                continue;
            const DMatch &m0 = pair_matches[i][0];
            const DMatch &m1 = pair_matches[i][1];
            if (m0.distance < (1.f - match_conf) * m1.distance)
                if (matches.find(std::make_pair(m0.trainIdx, m0.queryIdx)) == matches.end())
                    matches_info.matches.push_back(DMatch(m0.trainIdx, m0.queryIdx, m0.distance));
        }
        // LOG("1->2 & 2->1 matches: " << matches_info.matches.size() << endl);
        
        // Check if it makes sense to find homography
        if (matches_info.matches.size() >= static_cast<size_t>(6))
        {
            ////
            // Construct point-point correspondences for homography estimation
            Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
            Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
            // std::cout << "BestOf2NearestMatcher::match-> matches_info.matches.size is " << matches_info.matches.size() << std::endl;
            //TODO 取前五百对最小距离的
            for (size_t i = 0; i < matches_info.matches.size(); ++i)
            {
                const DMatch& m = matches_info.matches[i];
                
                Point2f p = global_img_feature[from].keypoints[m.queryIdx].pt;
                p.x -= global_img_feature[from].img_size.width * 0.5f;
                p.y -= global_img_feature[from].img_size.height * 0.5f;
                src_points.at<Point2f>(0, static_cast<int>(i)) = p;
                
                p = global_img_feature[to].keypoints[m.trainIdx].pt;
                p.x -= global_img_feature[to].img_size.width * 0.5f;
                p.y -= global_img_feature[to].img_size.height * 0.5f;
                dst_points.at<Point2f>(0, static_cast<int>(i)) = p;
            }
            
            // Find pair-wise motion
            matches_info.H = findHomography(src_points, dst_points, matches_info.inliers_mask, RANSAC);
            //std::cout << "matches_info pair-wise motion H is " << matches_info.H << std::endl;
            //下面的判断语句是原始源码的,在这需要改成下面的判断并加上符合修改后的判断里面的执行语句,否则在H为空的时候直接用原始的会报错
            //        if (matches_info.H.empty() || std::abs(determinant(matches_info.H)) < std::numeric_limits<double>::epsilon())
            //        {
            //            std::cout << "matches_info.H is empty." << std::endl;
            //            continue;
            //        }
            if (!matches_info.H.empty() && std::abs(determinant(matches_info.H)) >= std::numeric_limits<double>::epsilon())
                // Find number of inliers
            {
                
                matches_info.num_inliers = 0;
                for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
                    if (matches_info.inliers_mask[i])
                        matches_info.num_inliers++;
                //std::cout << "BestOf2NearestMatcher::match-> matches_info.num_inliers is " << matches_info.num_inliers<< std::endl;
                // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic Image Stitching
                // using Invariant Features"
                matches_info.confidence = matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());
                //  std::cout << "BestOf2NearestMatcher::match-> matches_info.confidence is " << matches_info.confidence  << std::endl;
                if(matches_info.confidence > 3) //zt
                {
                    if(!strLogPath.empty())
                    {
                        output_file << "LOGINFO: two pictures matches is too close." << std::endl;
                    }
                    else
                        std::cout << "LOGINFO: two pictures matches is too close." << std::endl;
                }
                // Set zero confidence to remove matches between too close images, as they don't provide
                // additional information anyway. The threshold was set experimentally.
                
                //matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;  //zt
                
                ///////zt
                // Check if we should try to refine motion
                if (matches_info.num_inliers >= 6)
                {
                    //zt add 新加判断匹配是否符合拼接条件
                    //如果当前图片匹配失败, 需要删掉当前图片以及当前图片的特征和匹配的信息,并返回
                    //                    if(matches_info.num_inliers < inliner_num || matches_info.confidence < conf_thresh)
                    //                    {
                    //                        MatchError++; //当前图片与之前每一次的匹配是否满足匹配条件，
                    //                    }
                    
                    // Construct point-point correspondences for inliers only
                    src_points.create(1, matches_info.num_inliers, CV_32FC2);
                    dst_points.create(1, matches_info.num_inliers, CV_32FC2);
                    int inlier_idx = 0;
                    for (size_t i = 0; i < matches_info.matches.size(); ++i)
                    {
                        if (!matches_info.inliers_mask[i])
                            continue;
                        
                        const DMatch& m = matches_info.matches[i];
                        
                        Point2f p = global_img_feature[from].keypoints[m.queryIdx].pt;
                        
                        p.x -= global_img_feature[from].img_size.width * 0.5f;
                        p.y -= global_img_feature[from].img_size.height * 0.5f;
                        src_points.at<Point2f>(0, inlier_idx) = p;
                        
                        p = global_img_feature[to].keypoints[m.trainIdx].pt;
                        p.x -= global_img_feature[to].img_size.width * 0.5f;
                        p.y -= global_img_feature[to].img_size.height * 0.5f;
                        dst_points.at<Point2f>(0, inlier_idx) = p;
                        
                        inlier_idx++;
                    }
                    
                    // Rerun motion estimation on inliers only
                    matches_info.H = findHomography(src_points, dst_points, RANSAC);
                    pairwise_matches[pair_idx] = matches_info; //zt
                }
                //                else
                //                {
                //                    MatchError++;
                //                }
            }
            //            else
            //            {
            //                MatchError++;
            //            }
        }
        //        else
        //            MatchError++;
        
        pairwise_matches[pair_idx].src_img_idx = from;
        pairwise_matches[pair_idx].dst_img_idx = to;
        
        size_t dual_pair_idx = to * num_images + from;
        // std::cout << "MatchPairsBody -> dual_pair_idx is " << dual_pair_idx << std::endl;
        pairwise_matches[dual_pair_idx] = pairwise_matches[pair_idx];
        pairwise_matches[dual_pair_idx].src_img_idx = to;
        pairwise_matches[dual_pair_idx].dst_img_idx = from;
        
        if (!pairwise_matches[pair_idx].H.empty())
            pairwise_matches[dual_pair_idx].H = pairwise_matches[pair_idx].H.inv();
        
        for (size_t j = 0; j < pairwise_matches[dual_pair_idx].matches.size(); ++j)
            std::swap(pairwise_matches[dual_pair_idx].matches[j].queryIdx,
                      pairwise_matches[dual_pair_idx].matches[j].trainIdx);
        
        Dmatcher.release(); //zt
        //zt
        if(pairwise_matches[pair_idx].src_img_idx != -1)
        {
            tmp_pairwise_matches.push_back(pairwise_matches[pair_idx]);
            tmp_pairwise_matches.push_back(pairwise_matches[dual_pair_idx]);
        }
        
        gettimeofday( &submatch_end, NULL );
        //求出两次时间的差值，单位为us
        int submatch_timeuse = 1000000 * ( submatch_end.tv_sec - submatch_start.tv_sec ) + submatch_end.tv_usec - submatch_start.tv_usec;
        //printf("submatch time: %d us\n", submatch_timeuse);
    }
    
    if(num_images == 2)  // 直接返回结果
    {
        global_pairwise_matches.push_back(pairwise_matches[0]);
        for(int i = 0; i < num_images; ++i)
            global_pairwise_matches.push_back(tmp_pairwise_matches[i]);
        global_pairwise_matches.push_back(pairwise_matches[0]);
    }
    // 因为 temp 里面的都是成对出现, 如0-1 和 1-0, 0-2 和 2-0, 1-2 和 2-1, 0-3 和 3-0 , 所以下面的直接取除temp元素,不需要再取排序
    // 下面每次insert 完可以将temp里面的erase 掉, 这里我没做这个
    else if(num_images == 3)  // 先将0-2 插进去, 再将1-2 插到end(), 再 2-0 和 2-1
    {
        global_pairwise_matches.insert(global_pairwise_matches.begin() + num_images - 1,tmp_pairwise_matches[0]);// 将0-2 插进去
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            //先将 1-2 插进去,
            if(tmp_pairwise_matches[i].src_img_idx != 0 && tmp_pairwise_matches[i].dst_img_idx == num_images - 1)
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
        }
        //再插 2-0 和 2-1
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            if (tmp_pairwise_matches[i].src_img_idx == num_images - 1)
                global_pairwise_matches.insert(global_pairwise_matches.end(), tmp_pairwise_matches[i]);
        }
        global_pairwise_matches.push_back(global_pairwise_matches[0]);
    }
    else  //num_images > 3
    {
        // global_pairwise_matches.insert(global_pairwise_matches.begin() + num_images - 1,tmp_pairwise_matches[0]);
        //查找每次新的 (0-(num_images -1)) ,(0-(num_image - 2)),(1-(num_images - 1)),...,插入到对应位置
        for (int i = 0; i < tmp_pairwise_matches.size(); i++ ){
            for(int j = 0; j < global_pairwise_matches.size(); ++j)
            {
                if(global_pairwise_matches[j].src_img_idx == tmp_pairwise_matches[i].src_img_idx && global_pairwise_matches[j].dst_img_idx == num_images - 2)
                {
                    global_pairwise_matches.insert(global_pairwise_matches.begin() + j + 1,tmp_pairwise_matches[i]);
                    tmp_pairwise_matches.erase(tmp_pairwise_matches.begin() + i);
                }
            }
        }
        //最后几个上面没有对应的位置,直接插到末尾
        for(int i = 0; i < tmp_pairwise_matches.size(); ++i)
        {
            if(tmp_pairwise_matches[i].src_img_idx != num_images - 1)
            {
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
                tmp_pairwise_matches.erase(tmp_pairwise_matches.begin() + i);
            }
        }
        //从 (num_imagse -1 )-0 , (num_imagse -1 )-1, (num_imagse -1 )-2, ... 插入末尾
        for (int i = 0; i < tmp_pairwise_matches.size(); i++) {
            
            if(tmp_pairwise_matches[i].src_img_idx == num_images - 1 )
                global_pairwise_matches.insert(global_pairwise_matches.end(),tmp_pairwise_matches[i]);
        }
        //最后一个跟自己匹配的结果(num_images - 1)-(num_images - 1), 内容和第一个一样,直接插入
        global_pairwise_matches.insert(global_pairwise_matches.end(),pairwise_matches[0]);
    }
    gettimeofday( &match_end, NULL );
    //求出两次时间的差值，单位为us
    int match_timeuse = 1000000 * ( match_end.tv_sec - match_start.tv_sec ) + match_end.tv_usec - match_start.tv_usec;
    
    if(!strLogPath.empty())
        output_file << "LOGINFO: Matcher Feature time is " << match_timeuse << "  us."<< std::endl;
    else
    {
        printf("match time: %d us\n", match_timeuse);
    }
    
    
    for(int i = 0; i < global_pairwise_matches.size(); ++i)
    {
        if(!strLogPath.empty()) {
            output_file << "src = " << global_pairwise_matches[i].src_img_idx << " , dst = "
            << global_pairwise_matches[i].dst_img_idx;
            output_file << " , num_inliers = " << global_pairwise_matches[i].num_inliers << " , confidence = "
            << global_pairwise_matches[i].confidence << std::endl;
        }
    }
    
    //当前图片与之前匹配不满足条件的个数如果等于所有匹配次数，则此次匹配失败，删除保存的当前图片和特征, 返回拼接失败的结果
    //    if(MatchError == num_images - 1)
    //    {
    //        global_images.pop_back();
    //        global_input_images.pop_back();
    //        global_img_feature.pop_back();
    //        stitchflag = false;
    //
    //        if(!strLogPath.empty())
    //            output_file << "Match error = num_images - 1 ." << std::endl;
    //        else
    //            std::cout << "Match error == num_images - 1 ." << std::endl;
    //        return false;
    //    }
    
    //判断哪些是不能拼接的, 在matcher里面我已经通过设置匹配置信度和内点数量进行了判断
    vector<int> indices;
    indices.clear();
    vector <ImageFeatures> img_feature_tmp;
    img_feature_tmp.clear();
    img_feature_tmp = global_img_feature;
    vector <MatchesInfo> pairwise_matches_tmp;
    pairwise_matches_tmp.clear();
    pairwise_matches_tmp = global_pairwise_matches;
    indices = leaveBiggestComponent(img_feature_tmp, pairwise_matches_tmp, conf_thresh);
    //zt 20190509
    if(indices.size() < num_images) {
        if (indices[indices.size() - 1] != num_images - 1 || indices.size() < 2) {
            if (!strLogPath.empty()) {
                output_file << "indice size != num_images." << std::endl;
                output_file.close();
            } else
                std::cout << "indice size !=  num_images." << std::endl;
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if (InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            
            return false;
        } else {
            global_pairwise_matches.clear();
            global_pairwise_matches = pairwise_matches_tmp;
            global_img_feature.clear();
            global_img_feature = img_feature_tmp;
            
            std::vector<cv::Mat> images_tmp;
            std::vector<cv::Mat> input_images_tmp;
            
            
            for (int i = 0; i < indices.size(); ++i) {
                images_tmp.push_back(global_images[indices[i]]);
                input_images_tmp.push_back(global_input_images[indices[i]]);
            }
            global_images.clear();
            global_input_images.clear();
            global_images = images_tmp;
            global_input_images = input_images_tmp;
            num_images = global_input_images.size();
        }
    }
    //    if(indices.size() < num_images)
    //    {
    //        if(!strLogPath.empty())
    //        {
    //            output_file << "indice size < num_images." << std::endl;
    //            output_file.close();
    //        }
    //        else
    //            std::cout << "indice size < num_images." << std::endl;
    //        global_images.pop_back();
    //        global_input_images.pop_back();
    //        global_img_feature.pop_back();
    //
    //        all_images.pop_back();
    //        all_input_images.pop_back();
    //        all_img_feature.pop_back();
    //
    //        //前面拼接失败,需要删除前面最近一次的匹配信息
    //        global_pairwise_matches.pop_back();
    //        if(InputImgNums == 2)
    //            global_pairwise_matches.clear();
    //        else {
    //            int srcMaxIdx = 0, dstMaxIdx = 0;
    //            global_pairwise_matches.pop_back();
    //            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
    //                if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
    //                    srcMaxIdx = global_pairwise_matches[i].src_img_idx;
    //                if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
    //                    dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
    //            }
    //
    //            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
    //                if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
    //                    global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
    //                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
    //                    i = -1;
    //                }
    //            }
    //        }
    //
    //        return false;
    //    }
    
    //    //寻找最大匹配
    //    vector<int> pair_idx;
    //    vector<double> pair_confidence;
    //    vector<int> pair_points;
    //    for (int i = 0; i < global_img_feature.size(); ++i) {
    //        pair_idx.push_back(-1);
    //        pair_confidence.push_back(0.0);
    //        pair_points.push_back(0);
    //    }
    //    int src_img_idx;
    //    int dst_img_idx;
    //    double confidence;
    //    int num_inliers;
    //    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
    //        src_img_idx = global_pairwise_matches[i].src_img_idx;
    //        dst_img_idx = global_pairwise_matches[i].dst_img_idx;
    //        confidence = global_pairwise_matches[i].confidence;
    //        num_inliers = global_pairwise_matches[i].num_inliers;
    //        if (pair_idx[src_img_idx] == -1) {
    //            pair_idx[src_img_idx] = dst_img_idx;
    //            pair_confidence[src_img_idx] = confidence;
    //            pair_points[src_img_idx] = num_inliers;
    //        } else if (confidence > pair_confidence[src_img_idx]) {
    //            pair_idx[src_img_idx] = dst_img_idx;
    //            pair_confidence[src_img_idx] = confidence;
    //            pair_points[src_img_idx] = num_inliers;
    //        }
    //    }
    //
    //    bool bflag = false;//zt
    //    int ImgNum = global_img_feature.size(); //这里的ImgNum包括当前传进来的图片
    //    for (int i = 0; i < ImgNum; ++i) {
    //        if (pair_points[i] <= inliner_num) {
    //            bflag = true;
    //        }
    //    }
    //
    //    if(!strLogPath.empty())
    //    {
    //        if(bflag)
    //        {
    //            global_images.pop_back();
    //            global_input_images.pop_back();
    //            global_img_feature.pop_back();
    //
    //
    //            all_images.pop_back();
    //            all_input_images.pop_back();
    //            all_img_feature.pop_back();
    //
    //            //前面拼接失败,需要删除前面最近一次的匹配信息
    //            global_pairwise_matches.pop_back();
    //            if(InputImgNums == 2)
    //                global_pairwise_matches.clear();
    //            else {
    //                int srcMaxIdx = 0, dstMaxIdx = 0;
    //                global_pairwise_matches.pop_back();
    //                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
    //                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
    //                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
    //                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
    //                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
    //                }
    //
    //                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
    //                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
    //                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
    //                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
    //                        i = -1;
    //                    }
    //                }
    //            }
    //
    //            output_file << "LOGINFO: inliner_num < 16" << std::endl;
    //            output_file.close(); //zt
    //            return false; //zt
    //        }
    //
    //    } else
    //    {
    //        if(bflag)
    //        {
    //            global_images.pop_back();
    //            global_input_images.pop_back();
    //            global_img_feature.pop_back();
    //
    //            all_images.pop_back();
    //            all_input_images.pop_back();
    //            all_img_feature.pop_back();
    //            std::cout << "LOGINFO: inliner_num < 16" << std::endl;
    //
    //            //前面拼接失败,需要删除前面最近一次的匹配信息
    //            global_pairwise_matches.pop_back();
    //            if(InputImgNums == 2)
    //                global_pairwise_matches.clear();
    //            else {
    //                int srcMaxIdx = 0, dstMaxIdx = 0;
    //                global_pairwise_matches.pop_back();
    //                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
    //                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
    //                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
    //                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
    //                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
    //                }
    //
    //                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
    //                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
    //                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
    //                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
    //                        i = -1;
    //                    }
    //                }
    //            }
    //
    //            return false; //zt
    //        }
    //    }
    
    vector<CameraParams> cameras;
    struct timeval eatimator_start, estimator_end;
    gettimeofday( &eatimator_start, NULL );
    
    if(!strLogPath.empty())
    {
        if (!(*estimator)(global_img_feature, global_pairwise_matches, cameras)){
            output_file << "LOGINFO: camera estimat failed." << std::endl; //zt
            output_file.close(); //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if(InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            
            return false;
        }
    }
    else
    {
        if (!(*estimator)(global_img_feature, global_pairwise_matches, cameras)){
            std::cout << "LOGINFO: camera estimat failed." << std::endl; //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if(InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            
            return false;
        }
    }
    gettimeofday( &estimator_end, NULL );
    //求出两次时间的差值，单位为us
    int estimator_timeuse = 1000000 * ( estimator_end.tv_sec - eatimator_start.tv_sec ) + estimator_end.tv_usec - eatimator_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "estimator time is " << estimator_timeuse  << "  us."<< std::endl;
    else
        printf("estimator_ time: %d us\n", estimator_timeuse);
    
    for (size_t i = 0; i < cameras.size(); ++i){
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
    }
    
    ///log
    for (size_t i = 0; i < cameras.size(); ++i) {
        std::cout << "R is " << cameras[i].R << std::endl;
        std::cout << "t is " << cameras[i].t << std::endl;
        std::cout << "ppx = " << cameras[i].ppx << ", ppy = " << cameras[i].ppy << std::endl;
        std::cout << "focal = " << cameras[i].focal << std::endl;
        std::cout << "aspect = " << cameras[i].aspect << std::endl;
    }
    ///log
    struct timeval adjust_start, adjust_end;
    gettimeofday( &adjust_start, NULL);
    
    if(!strLogPath.empty())
    {
        if (!(*adjuster)(global_img_feature, global_pairwise_matches, cameras)){
            //if(!bundleadjust(global_img_feature, global_pairwise_matches, cameras)){
            output_file << "LOGERROR: camera adjuster failed." << std::endl; //zt
            output_file.close(); //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if(InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            
            return false;
        }
        // check camera parameter R
        CameraParams camera;
        for (int i = 0; i < cameras.size(); ++i) {
            camera = cameras[i];
            for (int row = 0; row < camera.R.rows; row++) {
                for (int col = 0; col < camera.R.cols; col++) {
                    if (isnan(camera.R.at<int>(row,col))) {
                        output_file << "LOGINFO: camera check failed." << std::endl; //zt
                        output_file.close(); //zt
                        global_images.pop_back();
                        global_input_images.pop_back();
                        global_img_feature.pop_back();
                        
                        all_images.pop_back();
                        all_input_images.pop_back();
                        all_img_feature.pop_back();
                        
                        //前面拼接失败,需要删除前面最近一次的匹配信息
                        global_pairwise_matches.pop_back();
                        if(InputImgNums == 2)
                            global_pairwise_matches.clear();
                        else {
                            int srcMaxIdx = 0, dstMaxIdx = 0;
                            global_pairwise_matches.pop_back();
                            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                                if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                                    srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                                if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                                    dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                            }
                            
                            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                                if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                                    global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                                    i = -1;
                                }
                            }
                        }
                        
                        return false;
                    }
                }
            }
        }
        
    }
    else
    {
        if (!(*adjuster)(global_img_feature, global_pairwise_matches, cameras)){
            std::cout << "LOGERROR: camera adjuster failed." << std::endl; //zt
            global_images.pop_back();
            global_input_images.pop_back();
            global_img_feature.pop_back();
            
            all_images.pop_back();
            all_input_images.pop_back();
            all_img_feature.pop_back();
            //前面拼接失败,需要删除前面最近一次的匹配信息
            global_pairwise_matches.pop_back();
            if(InputImgNums == 2)
                global_pairwise_matches.clear();
            else {
                int srcMaxIdx = 0, dstMaxIdx = 0;
                global_pairwise_matches.pop_back();
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                        srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                    if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                        dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                }
                
                for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                    if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                        global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                        global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                        i = -1;
                    }
                }
            }
            return false;
        }
        // check camera parameter R
        CameraParams camera;
        for (int i = 0; i < cameras.size(); ++i) {
            camera = cameras[i];
            for (int row = 0; row < camera.R.rows; row++) {
                for (int col = 0; col < camera.R.cols; col++) {
                    if (isnan(camera.R.at<int>(row,col))) {
                        std::cout << "LOGINFO: camera check failed." << std::endl; //zt
                        global_images.pop_back();
                        global_input_images.pop_back();
                        global_img_feature.pop_back();
                        
                        all_images.pop_back();
                        all_input_images.pop_back();
                        all_img_feature.pop_back();
                        
                        //前面拼接失败,需要删除前面最近一次的匹配信息
                        global_pairwise_matches.pop_back();
                        if(InputImgNums == 2)
                            global_pairwise_matches.clear();
                        else {
                            int srcMaxIdx = 0, dstMaxIdx = 0;
                            global_pairwise_matches.pop_back();
                            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                                if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                                    srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                                if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                                    dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                            }
                            
                            for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                                if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                                    global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                                    global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                                    i = -1;
                                }
                            }
                        }
                        return false;
                    }
                }
            }
        }
    }
    
    gettimeofday( &adjust_end, NULL );
    //求出两次时间的差值，单位为us
    int adjust_timeuse = 1000000 * ( adjust_end.tv_sec - adjust_start.tv_sec ) + adjust_end.tv_usec - adjust_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "adjust time is " << adjust_timeuse << "  us." << std::endl;
    else
        printf("adjust time: %d us\n", adjust_timeuse);
    
    camera_params = cameras;
    //大于最大匹配数保存当前图片的相机参数  //zt
    //    if(InputImgNums <= MaxImgNums_)
    //        global_camera_params = cameras;
    //    else
    //        global_camera_params.push_back(cameras[MaxImgNums_ - 1]);
    
    vector<double> focals;
    focals.clear(); //zt
    for (size_t i = 0; i < cameras.size(); ++i){
        focals.push_back(cameras[i].focal);
    }
    
    sort(focals.begin(), focals.end());
    float warped_image_scale;
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;
    
    vector<Point> corners(global_input_images.size());
    vector<UMat> masks_warped(global_input_images.size());
    vector<UMat> images_warped(global_input_images.size());
    vector<Size> sizes(global_input_images.size());
    vector<UMat> masks(global_input_images.size());
    
    
    struct timeval warp_start, warp_end;
    gettimeofday( &warp_start, NULL );
    
#ifndef WARP
    // Preapre images masks
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(global_images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }
    
    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
    
    for (int i = 0; i < num_images; ++i)
    {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;
        
        //check
        Rect roi = warper->warpRoi(global_images[i].size(), K, cameras[i].R);
        
        if (roi.width > pano_max_edge || roi.height > pano_max_edge){
            if(!strLogPath.empty())
            {   output_file << "LOGERROR： roi.width or roi.height > pano_max_edge. "<< std::endl;
                output_file.close();
                global_images.pop_back();
                global_input_images.pop_back();
                global_img_feature.pop_back();
                
                all_images.pop_back();
                all_input_images.pop_back();
                all_img_feature.pop_back();
                
                //前面拼接失败,需要删除前面最近一次的匹配信息
                global_pairwise_matches.pop_back();
                if(InputImgNums == 2)
                    global_pairwise_matches.clear();
                else {
                    int srcMaxIdx = 0, dstMaxIdx = 0;
                    global_pairwise_matches.pop_back();
                    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                        if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                            srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                        if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                            dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                    }
                    
                    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                        if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                            global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                            global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                            i = -1;
                        }
                    }
                }
                return false;
            }
            else
            {
                std::cout << "LOGERROR： roi.width or roi.height > pano_max_edge. "<< std::endl;
                global_images.pop_back();
                global_input_images.pop_back();
                global_img_feature.pop_back();
                
                all_images.pop_back();
                all_input_images.pop_back();
                all_img_feature.pop_back();
                
                //前面拼接失败,需要删除前面最近一次的匹配信息
                global_pairwise_matches.pop_back();
                if(InputImgNums == 2)
                    global_pairwise_matches.clear();
                else {
                    int srcMaxIdx = 0, dstMaxIdx = 0;
                    global_pairwise_matches.pop_back();
                    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                        if (global_pairwise_matches[i].src_img_idx > srcMaxIdx)
                            srcMaxIdx = global_pairwise_matches[i].src_img_idx;
                        if (global_pairwise_matches[i].dst_img_idx > dstMaxIdx)
                            dstMaxIdx = global_pairwise_matches[i].dst_img_idx;
                    }
                    
                    for (int i = 0; i < global_pairwise_matches.size(); ++i) {
                        if (global_pairwise_matches[i].src_img_idx == srcMaxIdx ||
                            global_pairwise_matches[i].dst_img_idx == dstMaxIdx) {
                            global_pairwise_matches.erase(global_pairwise_matches.begin() + i);
                            i = -1;
                        }
                    }
                }
                return false;
            }
        }
        
        corners[i] = warper->warp(global_images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();
        
        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
    }
    
    vector<UMat> images_warped_f(num_images);
    for (int i = 0; i < num_images; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);
    
    gettimeofday( &warp_end, NULL );
    //求出两次时间的差值，单位为us
    int warp_timeuse = 1000000 * ( warp_end.tv_sec - warp_start.tv_sec ) + warp_end.tv_usec - warp_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "warp time is " << warp_timeuse << "  us." << std::endl;
    else
        printf("warp time: %d us\n", warp_timeuse);
    //seam time
    struct timeval seam_start, seam_end;
    gettimeofday( &seam_start, NULL );
    
    seam_finder->find(images_warped_f, corners, masks_warped);
    gettimeofday( &seam_end, NULL );
    //求出两次时间的差值，单位为us
    int seam_timeuse = 1000000 * ( seam_end.tv_sec - seam_start.tv_sec ) + seam_end.tv_usec - seam_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "seam time is " << seam_timeuse << "  us." << std::endl;
    else
        printf("seam time: %d us\n", seam_timeuse);
    
    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    Mat input_img;
    double compose_seam_aspect = 1;
    double compose_work_aspect = 1;
    
    struct timeval compose_start, compose_end;
    gettimeofday( &compose_start, NULL );
    
    if (!is_compose_scale_set ) {
        compose_scale = min(1.0, sqrt(compose_megapix * MaxPix_ / input.size().area()));
        is_compose_scale_set = true;
    }
    
    // Compute relative scales
    compose_seam_aspect = compose_scale / seam_scale;
    compose_work_aspect = compose_scale / work_scale;
    
    // Update warped image scale
    warped_image_scale *= static_cast<float>(compose_work_aspect);
    warper = warper_creator->create(warped_image_scale);
    
    // Update corners and sizes
    for (int i = 0; i < num_images; ++i)
    {
        // Update intrinsics
        cameras[i].focal *= compose_work_aspect;
        cameras[i].ppx *= compose_work_aspect;
        cameras[i].ppy *= compose_work_aspect;
        
        // Update corner and size
        Size sz = input.size();
        if (std::abs(compose_scale - 1) > 1e-1)
        {
            sz.width = cvRound(input.size().width * compose_scale);
            sz.height = cvRound(input.size().height * compose_scale);
        }
        
        Mat K;
        cameras[i].K().convertTo(K, CV_32F);
        Rect roi = warper->warpRoi(sz, K, cameras[i].R);
        corners[i] = roi.tl();
        sizes[i] = roi.size();
    }
    
    for (int img_idx = 0; img_idx < num_images; ++img_idx)
    {
        // Read image and resize it if necessary
        input_img = global_input_images[img_idx].clone();
        
        if (abs(compose_scale - 1) > 1e-1)
            cv::resize(input_img, img, Size(), compose_scale, compose_scale, INTER_LINEAR_EXACT);
        else
            img = input_img.clone();
        
        input_img.release();
        Size img_size = img.size();
        
        Mat K;
        cameras[img_idx].K().convertTo(K, CV_32F);
        
        // Warp the current image
        warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);
        
        // Warp the current image mask
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);
        
        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();
        
        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
        mask_warped = seam_mask & mask_warped;
        
        if (!blender && !timelapse)
        {
            blender = Blender::createDefault(blend_type, try_cuda);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, try_cuda);
            else if (blend_type == Blender::MULTI_BAND)
            {
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
            }
            blender->prepare(corners, sizes);
        }
        // Blend the current image
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }
    
    Mat result, result_mask;
    blender->blend(result, result_mask);
    
    gettimeofday( &compose_end, NULL );
    //求出两次时间的差值，单位为us
    int compose_timeuse = 1000000 * ( compose_end.tv_sec - compose_start.tv_sec ) + compose_end.tv_usec - compose_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "compose time is " << compose_timeuse << "  us." << std::endl;
    else
        printf("compose time: %d us\n", compose_timeuse);
    
    result.copyTo(dst);
    pano_vec.push_back(dst); //save pano ,by //zt
    result_size = result.size();
    //zt log camera parameters
    if(!strLogPath.empty())
    {
        for (size_t tt = 0; tt < cameras.size(); ++tt)
        {
            output_file << " LOGINFO: camera information: " << std::endl;  //zt
            output_file << "    aspect is " << cameras[tt].aspect << endl;
            output_file << "    focal is " << cameras[tt].focal << endl;
            output_file << "    Principal point X is " << cameras[tt].ppx  << ", Principal point Y is " << cameras[tt].ppy << std::endl;
            output_file << "    R matrix is " << cameras[tt].R << std::endl;
            output_file << "    t matrix is " << cameras[tt].t << std::endl;
        }
        // zt log pano size
        output_file << " LOGINFO: pano size is " << result_size << std::endl; //zt
    }
    
#endif
    gettimeofday( &stitch_end, NULL );
    //求出两次时间的差值，单位为us
    int stitch_timeuse = 1000000 * ( stitch_end.tv_sec - stitch_start.tv_sec ) + stitch_end.tv_usec - stitch_start.tv_usec;
    if(!strLogPath.empty())
        output_file << "total time is " << stitch_timeuse << "  us." << std::endl;
    else
        printf("total time: %d us\n", stitch_timeuse);
    
    if(!strLogPath.empty())
    {
        output_file.close(); //zt
    }
    InputImgNums ++;
    return true;
}
#endif

#ifdef TEST20190506
bool stitching(std::vector <cv::Mat> &imglist, Mat &dst, int mode, std::string & strLogPath)
{
    std::vector<KeyPoint> pts;
    pts.clear();
    cv::Mat img;
    imglist[0].copyTo(img);
    imglist[0].copyTo(dst);
    if(img.channels() > 1)
        cvtColor(img, img, CV_BGR2GRAY);
    HarrisResponses(img, pts, 3, 0.04);
    return true;
}
#endif

void stitching_params(std::vector<vector<Point>> &sum_corner_points_, Size &result_size_, std::vector<CameraParams> &camera_params_, double &input_scale_, vector<Point> &image_map_index_)
{
    sum_corner_points_ = sum_corner_points;
    result_size_ = result_size;
    //camera_params_ = camera_params;
    // camera_params_ = global_camera_params;  //zt
    input_scale_ = input_scale;
    image_map_index_ = image_map_index;
    
    //std::cout << " camera_params: size: " << camera_params_.size() << std::endl;
    //for (int i = 0; i < image_map_index_.size(); ++i) {
    //    std::cout << i << " image_map_index: " << image_map_index_[i] << std::endl;
    //    std::cout << i << " sum_corner_points: left_top: " << sum_corner_points_[i][0] << std::endl;
    //    std::cout << i << " sum_corner_points: right_bottom: " << sum_corner_points_[i][1] << std::endl;
    //    std::cout << i << " camera_params: aspect: " << camera_params_[i].aspect << std::endl;
    //    std::cout << i << " camera_params: focal: " << camera_params_[i].focal << std::endl;
    //    std::cout << i << " camera_params: ppx: " << camera_params_[i].ppx << std::endl;
    //    std::cout << i << " camera_params: ppy: " << camera_params_[i].ppy << std::endl;
    //    std::cout << i << " camera_params: R: " << camera_params_[i].R << std::endl;
    //    std::cout << i << " camera_params: t: " << camera_params_[i].t << std::endl;
    //}
    //std::cout << "result_size: " << result_size_ << std::endl;
    //std::cout << "input_scale: " << input_scale_ << std::endl;
    
    //release resource
    sum_corner_points.clear();
    camera_params.clear();
    image_map_index.clear();
}

#ifdef ORI
void stitching_params(std::vector<vector<Point>> &sum_corner_points_, Size &result_size_, std::vector<CameraParams> &camera_params_, double &input_scale_, vector<Point> &image_map_index_)
{
    sum_corner_points_ = sum_corner_points;
    result_size_ = result_size;
    camera_params_ = camera_params;
    input_scale_ = input_scale;
    image_map_index_ = image_map_index;
    
    //std::cout << " camera_params: size: " << camera_params_.size() << std::endl;
    //for (int i = 0; i < image_map_index_.size(); ++i) {
    //    std::cout << i << " image_map_index: " << image_map_index_[i] << std::endl;
    //    std::cout << i << " sum_corner_points: left_top: " << sum_corner_points_[i][0] << std::endl;
    //    std::cout << i << " sum_corner_points: right_bottom: " << sum_corner_points_[i][1] << std::endl;
    //    std::cout << i << " camera_params: aspect: " << camera_params_[i].aspect << std::endl;
    //    std::cout << i << " camera_params: focal: " << camera_params_[i].focal << std::endl;
    //    std::cout << i << " camera_params: ppx: " << camera_params_[i].ppx << std::endl;
    //    std::cout << i << " camera_params: ppy: " << camera_params_[i].ppy << std::endl;
    //    std::cout << i << " camera_params: R: " << camera_params_[i].R << std::endl;
    //    std::cout << i << " camera_params: t: " << camera_params_[i].t << std::endl;
    //}
    //std::cout << "result_size: " << result_size_ << std::endl;
    //std::cout << "input_scale: " << input_scale_ << std::endl;
    
    //release resource
    sum_corner_points.clear();
    camera_params.clear();
    image_map_index.clear();
}
#endif
