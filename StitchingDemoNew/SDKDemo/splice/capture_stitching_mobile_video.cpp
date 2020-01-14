
#include "capture_stitching_mobile_video.hpp"

#include <iostream>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching/detail/util.hpp"

#include <numeric>
#include <time.h> //zt
#include <sys/time.h> //zt
#include <math.h>


#include <list>
#include <vector>
#include <opencv2/video/tracking.hpp>
#include <iomanip>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv_modules.hpp"

#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/xfeatures2d/nonfree.hpp"
#endif

using namespace std;
using namespace cv;
using namespace cv::detail;

static float g_inlierConfThresh = 0.5;//0.9;  //匹配置信度
static int g_inlinerNum = 6;  //最小匹配内点数

static float g_matchConfThresh = 0.25f;  //匹配最低置信度
static double g_scale = 0.25;
static double g_inputScale = 1.0;//0.5;  //合成图片缩放尺度 ，1为原图合成
static vector<Mat> g_panoVec; //全景图list

static int g_inputImgNums = 0;
static std::vector<int> g_bestPovVecs;
static bool isInited = false;

cv::detail::ImageFeatures src_feature;
std::vector<cv::detail::ImageFeatures> src_featureVec;

cv::Point3d target_Angle;
std::vector<cv::Point3d> targetAngleVecs; //zt

typedef struct
{
    Mat inputImg;
    Mat img;
    ImageFeatures feature;
    float pitch;
    float yaw;
    float roll;
    Mat H;
}AngleImg;

std::vector <AngleImg> g_angleImg;

ofstream output_file;
std::string g_strLogPath;

//zongyue
list<cv::Point2f> origin_keypoints; //只是一个list存储所有的初始点
list< cv::Point2f > keypoints;      // 因为要删除跟踪失败的点，使用list

cv::Mat templateImg;
cv::Mat H_last_ = Mat::eye(3,3,CV_64FC1);
Mat H_optflow_ = Mat::eye(3,3,CV_64FC1);
int Max_origin_Size = 100;

#define Rsize_scale 0.25
#define Trans_rate 0.75
#define Max_origin_  100
#define Detail_Max_  60
#define Resize_gool 500.

bool find_back_tracking = false;

vector<KeyPoint> startKeypoints;
Mat start_Descriptor;

std::vector<vector<KeyPoint> > startKeypointsVecs;
std::vector<Mat> startDescriptorVecs;
bool IdtSetTemplate(cv::Mat & frame);
void KnnMatcher(Mat descriptors_1,Mat descriptors_2,vector<DMatch>& Dmatchinfo,float match_conf_);
float reprojError(vector<Point2f> pt_org, vector<Point2f> pt_prj, const Mat& H_front);
Size Get_Overlap_Area(vector<Point2f> corners);
bool Calc_Overlap(Mat& dst,int &maxOriginSize);
bool Start_Overlap(int &Max_origin_size);
void updateKeyPoint(const Mat& frame,int &MOS);
bool Retrieve_Overlap(Mat &dst,int Max_origin_Size);
bool Check_rads_bias(float size,const Mat& pic_,const Point2f& center);
void Overlap_calculate_ket(int& Max_origin_size);
void cal_area_corner(const Mat& pic,const Mat& H_final, vector<Point>&);
bool Check_center_crossborder(const Mat& pic,float border_rate,Mat H);
bool Check_Area_Correct(vector<Point>dst_pnts,cv::Mat &dst);
float Cal_rads_bias(float size,const Mat& pic_,const Point2f& center);

int Max_origin_size = Max_origin_;

typedef std::set<std::pair<int,int> > MatchesSet;

inline Mat resize_input(Mat &frame);
inline void resize_output(Mat &frame,vector<Point>& dst_pt);

bool Check_Area_Correct(vector<Point2f>dst_pnts,cv::Mat &dst);

float rad(float x){
    return x * CV_PI / 180;
}

inline float getSideLength(const Point2f &p1, const Point2f &p2);
inline float getSideVec(const Point2f &p1, const Point2f &p2, const Point2f & p3);

float getSideLength(const Point2f &p1, const Point2f &p2)
{
    float sideLength = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
    return sideLength;
}

float getSideVec(const Point2f &p1, const Point2f &p2, const Point2f & p3)
{
    float sideVec = (p2.x - p1.x) * (p3.x - p1.x) + (p2.y - p1.y) * (p3.y - p1.y);
    return sideVec;
}

//删除最后一张图片返回倒数第二次的拼接图
void IdtRetry(cv::Mat & pano){

    if (g_panoVec.size() > 0) {
        g_angleImg.pop_back();
        g_panoVec.pop_back();
        g_inputImgNums--;
        if (g_panoVec.size() > 0) {
            g_panoVec[g_panoVec.size() - 1].copyTo(pano);
        }
    }

    if(src_featureVec.size() > 0)
    {
        src_featureVec.pop_back();
    }
    if(startKeypointsVecs.size() > 0)
    {
        startKeypointsVecs.pop_back();
        startDescriptorVecs.pop_back();
        if(startKeypointsVecs.size() > 0)
        {
            startKeypoints.clear();
            start_Descriptor.release();

            startKeypoints.assign(startKeypointsVecs[startKeypointsVecs.size() - 1].begin(), startKeypointsVecs[startKeypointsVecs.size() - 1].end());
            startDescriptorVecs[startDescriptorVecs.size() - 1].copyTo(start_Descriptor);
            find_back_tracking = true;
        }
    }
    if(g_bestPovVecs.size() > 0)
        g_bestPovVecs.pop_back();
    if(targetAngleVecs.size() > 0)
    {
        targetAngleVecs.pop_back();
        if(targetAngleVecs.size() > 0)
            target_Angle = targetAngleVecs[targetAngleVecs.size() - 1];
    }

}

bool IdtStitchInit(const std::string &strLogPath)
{
    g_strLogPath = strLogPath;
    g_inputImgNums = 0;
    if(g_panoVec.size() > 0)
        g_panoVec.clear();
    if(g_angleImg.size() > 0)
        g_angleImg.clear();

    if(!g_strLogPath.empty()) {
        output_file.open(g_strLogPath, ios::out);
    }

    target_Angle.x = 0;
    target_Angle.y = 0;
    target_Angle.z = 0;
    if(targetAngleVecs.size() > 0)
        targetAngleVecs.clear();

    return true;
}

void CalcCorners(const Mat& H, const Mat& src, vector<Point2f> &corners)
{
    Point2f left_top, left_bottom, right_top, right_bottom;

    double v2[] = { 0, 0, 1 };//左上角
    double v1[3];//变换后的坐标值
    Mat V2 = Mat(3, 1, CV_64FC1, v2); //列向量
    Mat V1 = Mat(3, 1, CV_64FC1, v1); //列向量

    V1 = H * V2;
//左上角(0,0,1)
//    cout << "V2: " << V2 << endl;
    // cout << "V1: " << v1[2] << endl;
    left_top.x = v1[0] / v1[2];
    left_top.y = v1[1] / v1[2];
    corners.push_back(left_top);

//左下角(0,src.rows,1)
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2); //列向量
    V1 = Mat(3, 1, CV_64FC1, v1); //列向量
    V1 = H * V2;
    left_bottom.x = v1[0] / v1[2];
    left_bottom.y = v1[1] / v1[2];
    corners.push_back(left_bottom);

//右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2); //列向量
    V1 = Mat(3, 1, CV_64FC1, v1); //列向量
    V1 = H * V2;
    right_top.x = v1[0] / v1[2];
    right_top.y = v1[1] / v1[2];
    corners.push_back(right_top);

//右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2); //列向量
    V1 = Mat(3, 1, CV_64FC1, v1); //列向量
    V1 = H * V2;
    right_bottom.x = v1[0] / v1[2];
    right_bottom.y = v1[1] / v1[2];
    corners.push_back(right_bottom);
}

void calwarpImgSizeAndCorners(const Mat& src, const Mat& H,  cv::Size & dstSize, vector<Point2f> &corners)
{
    Mat tmp(3, 4, CV_64FC1, 1);
    tmp.at < double >(0, 0) = 0;
    tmp.at < double >(1, 0) = 0;
    tmp.at < double >(0, 1) = src.cols;
    tmp.at < double >(1, 1) = 0;
    tmp.at < double >(0, 2) = 0;
    tmp.at < double >(1, 2) = src.rows;
    tmp.at < double >(0, 3) = src.cols;
    tmp.at < double >(1, 3) = src.rows;

    //获得原图四个顶点变换后的坐标，计算变换后的图像尺寸
    Mat corner = H * tmp;  //corner=(x,y)=(cols,rows)

    float lt_x = (float)corner.at < double >(0, 0) / corner.at < double >(2,0);
    float lt_y = (float)corner.at < double >(1, 0) / corner.at < double >(2,0);
    float rt_x = (float)corner.at < double >(0, 1) / corner.at < double >(2,1);
    float rt_y = (float)corner.at < double >(1, 1) / corner.at < double >(2,1);
    float lb_x = (float)corner.at < double >(0, 2) / corner.at < double >(2,2);
    float lb_y = (float)corner.at < double >(1, 2) / corner.at < double >(2,2);
    float rb_x = (float)corner.at < double >(0, 3) / corner.at < double >(2,3);
    float rb_y = (float)corner.at < double >(1, 3) / corner.at < double >(2,3);

    double maxw = corner.at < double >(0, 0)/ corner.at < double >(2,0);
    double minw = corner.at < double >(0, 0)/ corner.at < double >(2,0);
    double maxh = corner.at < double >(1, 0)/ corner.at < double >(2,0);
    double minh = corner.at < double >(1, 0)/ corner.at < double >(2,0);

    //std::cout << "maxw = " << maxw << ", minw = " << minw << ", maxh = " << maxh << ",minh = " << minh << std::endl;
    for (int i = 1; i < 4; i++) {

        maxw = max(maxw, corner.at < double >(0, i) / corner.at < double >(2, i));
        minw = min(minw, corner.at < double >(0, i) / corner.at < double >(2, i));
        maxh = max(maxh, corner.at < double >(1, i) / corner.at < double >(2, i));
        minh = min(minh, corner.at < double >(1, i) / corner.at < double >(2, i));
    }

    int width = 0, height = 0;
    //创建向前映射矩阵 map_x, map_y
    width = int(maxw - minw);
    height = int(maxh - minh);


    if(width < src.cols) width = src.cols;
    if(height < src.rows) height = src.rows;

    dstSize.width = width;
    dstSize.height = height;

    Point2f point0, point1, point2, point3;

    point0.x = lt_x;
    point0.y = lt_y;
    point1.x = lb_x;
    point1.y = lb_y;
    point2.x = rt_x;
    point2.y = rt_y;
    point3.x = rb_x;
    point3.y = rb_y;

    corners.push_back(point0);
    corners.push_back(point1);
    corners.push_back(point2);
    corners.push_back(point3);
}

bool mywarpPerspective(Mat src, Mat &dst, Mat T, vector<Point2f> &corners) {

    //此处注意计算模型的坐标系与Mat的不同

    //图像以左上点为（0,0），向左为x轴，向下为y轴，所以前期搜索到的特征点 存的格式是（图像x，图像y）---（rows，cols）

    //而Mat矩阵的是向下为x轴，向左为y轴，所以存的方向为（图像y，图像x）----（cols，rows）----（width，height）

    //这个是计算的时候容易弄混的

    //创建原图的四个顶点的3*4矩阵（此处我的顺序为左上，右上，左下，右下）

    Mat tmp(3, 4, CV_64FC1, 1);
    tmp.at < double >(0, 0) = 0;
    tmp.at < double >(1, 0) = 0;
    tmp.at < double >(0, 1) = src.cols;
    tmp.at < double >(1, 1) = 0;
    tmp.at < double >(0, 2) = 0;
    tmp.at < double >(1, 2) = src.rows;
    tmp.at < double >(0, 3) = src.cols;
    tmp.at < double >(1, 3) = src.rows;

    //获得原图四个顶点变换后的坐标，计算变换后的图像尺寸
    Mat corner = T * tmp;  //corner=(x,y)=(cols,rows)

    float lt_x = (float)corner.at < double >(0, 0) / corner.at < double >(2,0);
    float lt_y = (float)corner.at < double >(1, 0) / corner.at < double >(2,0);
    float rt_x = (float)corner.at < double >(0, 1) / corner.at < double >(2,1);
    float rt_y = (float)corner.at < double >(1, 1) / corner.at < double >(2,1);
    float lb_x = (float)corner.at < double >(0, 2) / corner.at < double >(2,2);
    float lb_y = (float)corner.at < double >(1, 2) / corner.at < double >(2,2);
    float rb_x = (float)corner.at < double >(0, 3) / corner.at < double >(2,3);
    float rb_y = (float)corner.at < double >(1, 3) / corner.at < double >(2,3);

//    std::cout << "lt_x = " << lt_x << ", lt_y = " << lt_y << std::endl;
//    std::cout << "lb_x = " << lb_x << ", lb_y = " << lb_y << std::endl;
//    std::cout << "rt_x = " << rt_x << ", rt_y = " << rt_y << std::endl;
//    std::cout << "rb_x = " << rb_x << ", rb_y = " << rb_y << std::endl;


    double maxw = corner.at < double >(0, 0)/ corner.at < double >(2,0);
    double minw = corner.at < double >(0, 0)/ corner.at < double >(2,0);
    double maxh = corner.at < double >(1, 0)/ corner.at < double >(2,0);
    double minh = corner.at < double >(1, 0)/ corner.at < double >(2,0);

    //std::cout << "maxw = " << maxw << ", minw = " << minw << ", maxh = " << maxh << ",minh = " << minh << std::endl;
    for (int i = 1; i < 4; i++) {

        maxw = max(maxw, corner.at < double >(0, i) / corner.at < double >(2, i));
        minw = min(minw, corner.at < double >(0, i) / corner.at < double >(2, i));
        maxh = max(maxh, corner.at < double >(1, i) / corner.at < double >(2, i));
        minh = min(minh, corner.at < double >(1, i) / corner.at < double >(2, i));
    }

    int width = 0, height = 0;
    //创建向前映射矩阵 map_x, map_y
    width = int(maxw - minw);
    height = int(maxh - minh);

    if(width < src.cols) width = src.cols;
    if(height < src.rows) height = src.rows;

    if(width > 5 * src.cols || height > 5 * src.rows)
    {
        return false;
    }

    cv::Mat tmpImg;
    tmpImg.create(height, width, src.type());
    tmpImg.setTo(0);

    src.copyTo(tmpImg(Rect(0,0,src.cols, src.rows)));

    vector<Point2f> src_point;
    src_point.push_back(Point2f(0, 0));
    src_point.push_back(Point2f(0, src.rows));
    src_point.push_back(Point2f(src.cols, 0));
    src_point.push_back(Point2f(src.cols, src.rows));

    Point2f point0, point1, point2, point3;

    point0.x = lt_x;
    point0.y = lt_y;
    point1.x = lb_x;
    point1.y = lb_y;
    point2.x = rt_x;
    point2.y = rt_y;
    point3.x = rb_x;
    point3.y = rb_y;

    corners.push_back(point0);
    corners.push_back(point1);
    corners.push_back(point2);
    corners.push_back(point3);

    Point tmp0, tmp1, tl;
    tmp0.x = min(point0.x, point1.x);
    tmp1.x = min(point2.x, point3.x);
    tl.x = min(tmp0.x, tmp1.x);

    tmp0.y = min(point0.y, point1.y);
    tmp1.y = min(point2.y, point3.y);
    tl.y = min(tmp0.y, tmp1.y);

    point0.x = point0.x - tl.x;
    point0.y = point0.y - tl.y;
    point1.x = point1.x - tl.x;
    point1.y = point1.y - tl.y;
    point2.x = point2.x - tl.x;
    point2.y = point2.y - tl.y;
    point3.x = point3.x - tl.x;
    point3.y = point3.y - tl.y;

    vector<Point2f> obj_point;
    obj_point.push_back(point0);  //左上
    obj_point.push_back(point1);  //左下
    obj_point.push_back(point2);  //右上
    obj_point.push_back(point3);  //右下
    Mat H_ = findHomography(src_point, obj_point, CV_RANSAC);
    warpPerspective(tmpImg, dst, H_, cv::Size(width, height));

    return true;
}

bool getBestPOV(std::vector <AngleImg> g_angleImg, int & bestPOV){
    vector<Mat> Homos;
    Homos.clear();

    for(int i = 0; i < g_angleImg.size(); ++i)
    {
        Homos.push_back(g_angleImg[i].H);
    }
    cv::Mat img0;
    g_angleImg[g_angleImg.size() - 1].inputImg.copyTo(img0);

    int PanoSize_area;
    double app_start_time = getTickCount();
    int Img_Numbers = Homos.size();
    int PicNums_ = Img_Numbers;

    for (int bestp = 0; bestp < Img_Numbers; ++bestp) {

        Mat tempEye = Mat::eye(3,3,CV_64F);
        vector<Mat> Homo_Projected;

        int BestPOV = bestp;
        for (int picnum = 0; picnum < PicNums_;) {
            //        cout <<  "turn = " << picnum << endl;
            if(picnum < BestPOV){
                Mat tempHomo;
                tempEye.copyTo(tempHomo);
                for (int step = picnum+1; step <= BestPOV ;) {
                    tempHomo = Homos[step].inv() * tempHomo;
                    step++;
                }
                Homo_Projected.push_back(tempHomo);
            }
            else if (picnum > BestPOV){

                Mat tempHomo;
                tempEye.copyTo(tempHomo);
                for (int step = BestPOV+1; step <= (picnum);) {
                    tempHomo =  tempHomo * Homos[step];
                    step++;
                }
                Homo_Projected.push_back(tempHomo);
            }
            else if (picnum == BestPOV){
                Mat tempHomo;//深拷贝问题
                tempEye.copyTo(tempHomo);
                Homo_Projected.push_back(tempHomo);
            }
            picnum++;
        }

        vector<Size> SinglePrjSize;//
        vector<Point> CornersInPano;
        vector<vector<Point2f> > corners(PicNums_);

        for (int i = 0; i < PicNums_; ++i) {

            vector<Point2f> tempcorners;
            CalcCorners(Homo_Projected[i], img0, corners[i]);

            //TODO CalcCorners 计算的是原始图片四个顶点在warp后的图像位置
            // 这里需要改成四个顶点的外接矩形坐标
            Point tmp0, tmp1, LT_position, RB_position;
            //LT_position 是该图片在全景图中的左上角位置坐标
            tmp0.x = min(corners[i][0].x, corners[i][1].x);
            tmp1.x = min(corners[i][2].x, corners[i][3].x);
            LT_position.x = min(tmp0.x, tmp1.x);

            tmp0.y = min(corners[i][0].y, corners[i][1].y);
            tmp1.y = min(corners[i][2].y, corners[i][3].y);
            LT_position.y = min(tmp0.y, tmp1.y);
            //SinglePrjSize 尺寸
            tmp0.x = max(corners[i][0].x, corners[i][1].x);
            tmp1.x = max(corners[i][2].x, corners[i][3].x);
            RB_position.x = max(tmp0.x, tmp1.x) - LT_position.x;

            tmp0.y = max(corners[i][0].y, corners[i][1].y);
            tmp1.y = max(corners[i][2].y, corners[i][3].y);
            RB_position.y = max(tmp0.y, tmp1.y) - LT_position.y;
            //CornersInPano 全局位置

            SinglePrjSize.push_back(Size(RB_position.x, RB_position.y));
            CornersInPano.push_back(LT_position);//corners.left_top;
        }

        Rect dst_roi = resultRoi(CornersInPano, SinglePrjSize);

        if (bestp == 0){
            bestPOV = 0;
            PanoSize_area = dst_roi.height * dst_roi.width;
        }
        else{
            int PanoSize_area_tmp = dst_roi.height*2 + dst_roi.width;
            if(PanoSize_area > PanoSize_area_tmp){
                PanoSize_area = PanoSize_area_tmp;
                bestPOV = bestp;
            }
        }
    }
    return true;
}

bool findHomo(ImageFeatures &src_features, ImageFeatures &dst_features, Mat & Homo){

    //FlannBasedMatcher matcher;
    BFMatcher matcher;
    vector<vector<DMatch> > matchePoints;
    vector<DMatch> GoodMatchePoints;

    cv::Mat src_des;
    src_features.descriptors.copyTo(src_des);
    vector<Mat> train_desc(1,src_des);
    matcher.add(train_desc);
    matcher.train();

    cv::Mat dst_des;
    dst_features.descriptors.copyTo(dst_des);
    matcher.knnMatch(dst_des, matchePoints, 2);

    cout << "total match points: " << matchePoints.size() << endl;

    // overlap comput
    std::vector<Point2f> obj_pt, scene_pt;  //重复区域匹配点list
    obj_pt.clear();
    scene_pt.clear();

    // Lowe's algorithm,获取优秀匹配点
    for (int i = 0; i < matchePoints.size(); i++)
    {
        if (matchePoints[i][0].distance < (1.0 - g_matchConfThresh) * matchePoints[i][1].distance)
        {
            GoodMatchePoints.push_back(matchePoints[i][0]);
        }
    }

    if(GoodMatchePoints.size() < g_inlinerNum)
    {
        if(!g_strLogPath.empty())
            output_file << "GoodMatchePoints size  < g_inlinerNum" << std::endl;
        std::cout << "GoodMatchePoints size  < g_inlinerNum" << std::endl;
        return false;
    }

    cout << "Good match points: " << GoodMatchePoints.size() << std::endl;
    std::vector<KeyPoint> match_src_keypoints;
    match_src_keypoints.clear();
    std::vector<KeyPoint> match_dst_keypoints;
    match_dst_keypoints.clear();

    for(int k = 0; k < GoodMatchePoints.size(); ++k)
    {
        obj_pt.push_back(src_features.keypoints[GoodMatchePoints[k].trainIdx].pt );
        match_src_keypoints.push_back(src_features.keypoints[GoodMatchePoints[k].trainIdx]);
        scene_pt.push_back(dst_features.keypoints[GoodMatchePoints[k].queryIdx].pt);
        match_dst_keypoints.push_back(dst_features.keypoints[GoodMatchePoints[k].queryIdx]);
    }

//    if (obj_pt.size() < 50)
//    {
//        std::cout << "obj_pt.size() < 50. " << std::endl;
//        return false;
//    }

    std::vector <uchar> inliers_mask;
    Homo = findHomography(scene_pt, obj_pt, RANSAC, 4.0, inliers_mask, 500, 0.9999);
    std::cout << "Homo = " << Homo << std::endl;
    int good_num = 0;
    for (int i = 0; i < inliers_mask.size();++i){
        if (inliers_mask[i] != '\0')
            good_num++;
    }

    float conf = good_num /(8 + 0.3 * (obj_pt.size()));
    if (good_num < g_inlinerNum || conf < g_inlierConfThresh)
    {
        if(!g_strLogPath.empty())
            output_file << "Stitch: good_num < g_inlinerNum or conf < g_inlierConfThresh" << std::endl;
        std::cout << "Stitch: good_num < g_inlinerNum or conf < g_inlierConfThresh" << std::endl;
        return false;
    }

    return true;
}

//针对bestPOV是否改变来调整是不是将所有图片都重新合成
bool IdtStitch(const cv::Mat &src, const Point3f angle, cv::Mat &result, int & bestPOV, double Homo[9])
{
    struct timeval stitch_start, stitch_end;
    gettimeofday( &stitch_start, NULL );

    if(!g_strLogPath.empty())
    {
        if (src.empty())
        {
            output_file << "Stitch: input img is empty." << std::endl;
            return false;
        }
    }
    else
    {
        if (src.empty())
        {
            std::cout << "Stitch: input img is empty." << std::endl;
            return false;
        }
    }

    if(!g_strLogPath.empty()) {
        output_file << "Stitch: pitch = " << angle.x << ", roll = " << angle.y << ", yaw = " << angle.z << std::endl;
    }

        //每次只能送一张进行stitching
    if(g_inputImgNums == 0)
        g_inputImgNums = 1; // 当前图片就1张

    if(g_inputImgNums == 1)  //first image , return current img
    {
        Mat img;
        resize(src, img, Size(), g_scale, g_scale, INTER_AREA);//INTER_AREA
        AngleImg firstAngleImg;
        firstAngleImg.pitch = angle.x;
        firstAngleImg.yaw = angle.y;
        firstAngleImg.roll = angle.z;

        src.copyTo(firstAngleImg.inputImg);
        cv::resize(firstAngleImg.inputImg, firstAngleImg.inputImg, cv::Size(), g_inputScale, g_inputScale, cv::INTER_AREA); //zt
        img.copyTo(firstAngleImg.img);

        firstAngleImg.H = Mat::eye(3, 3, CV_64FC1);
        //find feature
        struct timeval first_find_start, first_find_end;
        gettimeofday( &first_find_start, NULL);

        //Ptr <Feature2D> sift = xfeatures2d::SIFT::create();
        Ptr<xfeatures2d::SIFT> sift = xfeatures2d::SIFT::create();
        sift-> detectAndCompute(firstAngleImg.img, Mat(), firstAngleImg.feature.keypoints, firstAngleImg.feature.descriptors); //得到特征点和特征点描述
        sift.release();
        //(*g_finder)(firstAngleImg.img, firstAngleImg.feature); //origin
        gettimeofday( &first_find_end, NULL);

        //求出两次时间的差值，单位为us
        int firstfindtimeuse = 1000000 * ( first_find_end.tv_sec - first_find_start.tv_sec ) + first_find_end.tv_usec - first_find_start.tv_usec;
        if(!g_strLogPath.empty())
            output_file << "Stitch: first img find time is " << firstfindtimeuse << "  us."<< std::endl;
        printf("Stitch: first img find time: %d us\n", firstfindtimeuse);

        g_panoVec.push_back(src); //save first img ,zt
        g_inputImgNums ++;
        g_angleImg.push_back(firstAngleImg);
        g_panoVec[g_panoVec.size() - 1].copyTo(result);


        Homo[0] = 1;
        Homo[1] = 0;
        Homo[2] = 0;
        Homo[3] = 0;
        Homo[4] = 1;
        Homo[5] = 0;
        Homo[6] = 0;
        Homo[7] = 0;
        Homo[8] = 1;
        bestPOV = 0;

        g_bestPovVecs.push_back(bestPOV); //zt
        cv::Mat temp;
        src.copyTo(temp);
        IdtSetTemplate(temp);

        return true;
    }

    //find feature
    struct timeval find_start, find_end;
    gettimeofday( &find_start, NULL );

    Mat img;
    resize(src, img, Size(), g_scale, g_scale, INTER_AREA);

    AngleImg angleImg;
    angleImg.pitch = angle.x;
    angleImg.yaw = angle.y;
    angleImg.roll = angle.z;

    src.copyTo(angleImg.inputImg);
    cv::resize(angleImg.inputImg, angleImg.inputImg, cv::Size(), g_inputScale, g_inputScale, cv::INTER_AREA); //zt
    img.copyTo(angleImg.img);
    //(*g_finder)(angleImg.img, angleImg.feature); //origin
    //Ptr <Feature2D> sift = xfeatures2d::SIFT::create();
    Ptr<xfeatures2d::SIFT> sift = xfeatures2d::SIFT::create();
    sift-> detectAndCompute(angleImg.img, Mat(), angleImg.feature.keypoints, angleImg.feature.descriptors); //得到特征点和特征点描述
    sift.release();

    gettimeofday( &find_end, NULL );
    //求出两次时间的差值，单位为us
    int find_timeuse = 1000000 * ( find_end.tv_sec - find_start.tv_sec ) + find_end.tv_usec - find_start.tv_usec;
    if(!g_strLogPath.empty())
        output_file << "Stitch: Finder time is " << find_timeuse << " us."<< std::endl;
    printf("Stitch: Finder time is %d us\n", find_timeuse);

    //TODO 求采集区域并删除非采集区域的特征点
    // offset

    struct timeval homo_start, homo_end;
    gettimeofday( &homo_start, NULL );

    // 单应矩阵求解
    if(findHomo(g_angleImg[g_inputImgNums - 2].feature, angleImg.feature, angleImg.H)){
        //对单应矩阵里面的尺度因子做缩放
        angleImg.H.at<double>(0, 2) = angleImg.H.at<double>(0, 2) / (g_scale / g_inputScale);
        angleImg.H.at<double>(1, 2) = angleImg.H.at<double>(1, 2) / (g_scale / g_inputScale);
        angleImg.H.at<double>(2, 0) = angleImg.H.at<double>(2, 0) * (g_scale / g_inputScale);
        angleImg.H.at<double>(2, 1) = angleImg.H.at<double>(2, 1) * (g_scale / g_inputScale);

        Homo[0] = angleImg.H.at<double>(0, 0);
        Homo[1] = angleImg.H.at<double>(0, 1);
        Homo[2] = angleImg.H.at<double>(0, 2);
        Homo[3] = angleImg.H.at<double>(1, 0);
        Homo[4] = angleImg.H.at<double>(1, 1);
        Homo[5] = angleImg.H.at<double>(1, 2);
        Homo[6] = angleImg.H.at<double>(2, 0);
        Homo[7] = angleImg.H.at<double>(2, 1);
        Homo[8] = angleImg.H.at<double>(2, 2);

        if(Homo[0] >= 2 || Homo[0] < 0.2 || Homo[4] >= 2 || Homo[4] < 0.2)
        {
            if(!g_strLogPath.empty())
            {
                output_file << "Stitch: homo abnormal. "<< ",  H = " << angleImg.H << std::endl;
            }
            std::cout << "Stitch: homo abnormal." << std::endl;
            return false;
        }

        if(!g_strLogPath.empty())
        {
            output_file << "Stitch: Homo = ";

            for(int i = 0;i < 9; ++i)
            {
                if(i < 8)
                    output_file << Homo[i] << ",";
                else
                    output_file << Homo[i];
            }
            output_file << std::endl;
        }

        g_angleImg.push_back(angleImg);

    } else
    {
        if(!g_strLogPath.empty())
            output_file << "Stitch: Match failed." << std::endl;
        std::cout << "Stitch: Match failed." << std::endl;
        return false;
    }

    vector<Point> resultCornersPre(g_inputImgNums - 1);
    vector<Point> resultCorners(g_inputImgNums);
    vector<Size> sizesPre(g_inputImgNums - 1);
    vector<Size> sizes(g_inputImgNums);
    vector<vector <Point2f> > corners(g_inputImgNums);
    std::vector <cv::Mat> HomoVecs;
    std::vector <cv::Mat> NewHomoVecs;
    std::vector <cv::Mat> warped(g_inputImgNums);

    for(int i = 0; i < g_inputImgNums; ++i)
    {
        Mat H = Mat::eye(3, 3, CV_64FC1);
        NewHomoVecs.push_back(H);
    }

    // ref pic , 第 refIndx + 1 图 作为参考图像
    int refIndx = 0;

#ifndef BESTPOV
    getBestPOV(g_angleImg,refIndx);
#else
    if(g_inputImgNums % 2 == 0)
        refIndx = g_inputImgNums / 2 - 1;
    else
        refIndx = g_inputImgNums / 2;
#endif
    //refIndx = 1;  //zt
    bestPOV = refIndx;

    if(!g_strLogPath.empty()) {
        output_file << "Stitch: bestPOV = " << bestPOV << std::endl;
    }

    Mat dst;
    //bestPOV不变的情况下只warp当前图片
    if(bestPOV == g_bestPovVecs[g_bestPovVecs.size() - 1])
    {
        //std::cout << "into bestPOV. bestPOV = " << bestPOV << std::endl;
        std::vector <cv::Mat> newWarped(2);
        for(int i = 0; i < g_inputImgNums; ++i)
        {
            if(i < refIndx)
            {
                Mat H_tmp = Mat::eye(3, 3, CV_64FC1);
                for(int k = i; k < refIndx; ++k){
                    H_tmp = H_tmp * g_angleImg[k+1].H;
                }
                NewHomoVecs[i] = H_tmp.inv();
            }
            else if(i > refIndx)
            {
                Mat H_tmp = Mat::eye(3, 3, CV_64FC1);
                for(int k = refIndx; k < i; ++k){
                    H_tmp = H_tmp * g_angleImg[k+1].H;
                }
                NewHomoVecs[i] = H_tmp;
            } else
            {
                NewHomoVecs[i] = Mat::eye(3, 3, CV_64FC1);
            }

            calwarpImgSizeAndCorners(g_angleImg[i].inputImg, NewHomoVecs[i], sizes[i], corners[i]);
//            std::cout << "corners[i] = " << corners[i] << std::endl;
            Point tmp0, tmp1, tl;
            tmp0.x = min(corners[i][0].x, corners[i][1].x);
            tmp1.x = min(corners[i][2].x, corners[i][3].x);
            tl.x = min(tmp0.x, tmp1.x);

            tmp0.y = min(corners[i][0].y, corners[i][1].y);
            tmp1.y = min(corners[i][2].y, corners[i][3].y);
            tl.y = min(tmp0.y, tmp1.y);

            resultCorners[i] = tl;

            if(i == g_inputImgNums - 1)  //只warp当前图片
            {
                vector <Point2f> corner;
                corner.clear();
                if(!mywarpPerspective(g_angleImg[i].inputImg, newWarped[1], NewHomoVecs[i], corner))
                {
                    if(!g_strLogPath.empty())
                    {
                        output_file << "Stitch: warp failed. " << std::endl;
                    }
                    std::cout << "Stitch: warp failed. " << std::endl;

                    if(g_angleImg.size() > 0)
                        g_angleImg.pop_back();

                    return false;
                }
            }
        }

        for(int idx = 0; idx < resultCorners.size() - 1; ++idx)
        {
            sizesPre.push_back(sizes[idx]);
            resultCornersPre.push_back(resultCorners[idx]);
        }

        Rect dst_roi = resultRoi(resultCorners, sizes);
        Rect dst_roiPre = resultRoi(resultCornersPre, sizesPre);
        std::cout << "my dst_roi size = " << dst_roi.size() << std::endl;
        std::cout << "dst_roi X = " << dst_roi.x << ", dst_roi Y = " << dst_roi.y << std::endl;
        dst.create(dst_roi.size(), CV_8UC3);
        dst.setTo(cv::Scalar::all(0));

        for(int i = 0; i < 2; ++i)
        {
            Mat gray;
            int dx = 0, dy = 0;
            if(i == 0)
            {
                g_panoVec[g_panoVec.size() - 1].copyTo(newWarped[i]);
                dx = dst_roiPre.x - dst_roi.x;//resultCorners[i].x - dst_roi.x;
                dy = dst_roiPre.y - dst_roi.y;//resultCorners[i].y - dst_roi.y;
            } else
            {
                dx = resultCorners[resultCorners.size() - 1].x - dst_roi.x;//resultCorners[0].x
                dy = resultCorners[resultCorners.size() - 1].y - dst_roi.y;//- resultCorners[0].y
            }
            cvtColor(newWarped[i], gray, CV_BGR2GRAY);
            //std::cout << "dx = " << dx << ", dy = " << dy << std::endl;

            struct timeval copy_start, copy_end;
            gettimeofday( &copy_start, NULL );

//            //debug
//            std::cout << "i = " << i << std::endl;
//            std::cout << "dst_roi = " << dst_roi.size() << std::endl;
//            std::cout << "dx = " << dx << ", dy = " << dy << std::endl;

            newWarped[i].copyTo(dst(Rect(dx, dy, newWarped[i].cols, newWarped[i].rows)), gray);

            gettimeofday( &copy_end, NULL );
            //求出两次时间的差值，单位为us
            int copy_timeuse = 1000000 * (copy_end.tv_sec - copy_start.tv_sec ) + copy_end.tv_usec - copy_start.tv_usec;
//            printf("copy time: %d us\n", copy_timeuse);

//            std::cout << "corners[i][0] = " << corners[corners.size() - 2] << std::endl;
//            std::cout << "corners[i][1] = " << corners[corners.size() - 1] << std::endl;
            if(i == 0)
            {
                corners[corners.size() - 2][0].x = corners[corners.size() - 2][0].x - dst_roi.x;
                corners[corners.size() - 2][0].y = corners[corners.size() - 2][0].y - dst_roi.y;
                corners[corners.size() - 2][1].x = corners[corners.size() - 2][1].x - dst_roi.x;
                corners[corners.size() - 2][1].y = corners[corners.size() - 2][1].y - dst_roi.y;
                corners[corners.size() - 2][2].x = corners[corners.size() - 2][2].x - dst_roi.x;
                corners[corners.size() - 2][2].y = corners[corners.size() - 2][2].y - dst_roi.y;
                corners[corners.size() - 2][3].x = corners[corners.size() - 2][3].x - dst_roi.x;
                corners[corners.size() - 2][3].y = corners[corners.size() - 2][3].y - dst_roi.y;

            } else
            {
                corners[corners.size() - 1][0].x = corners[corners.size() - 1][0].x - dst_roi.x;
                corners[corners.size() - 1][0].y = corners[corners.size() - 1][0].y - dst_roi.y;
                corners[corners.size() - 1][1].x = corners[corners.size() - 1][1].x - dst_roi.x;
                corners[corners.size() - 1][1].y = corners[corners.size() - 1][1].y - dst_roi.y;
                corners[corners.size() - 1][2].x = corners[corners.size() - 1][2].x - dst_roi.x;
                corners[corners.size() - 1][2].y = corners[corners.size() - 1][2].y - dst_roi.y;
                corners[corners.size() - 1][3].x = corners[corners.size() - 1][3].x - dst_roi.x;
                corners[corners.size() - 1][3].y = corners[corners.size() - 1][3].y - dst_roi.y;
            }
            //std::cout << "corners[i][0] X = " << corners[i][0].x << ", Y = " << corners[i][0].y << std::endl;
            if(i == 1)
            {
                line(dst, Point(corners[corners.size() - 1][0]),Point(corners[corners.size() - 1][2]), cv::Scalar(0, 0, 255), 2);
                line(dst, Point(corners[corners.size() - 1][2]),Point(corners[corners.size() - 1][3]), cv::Scalar(0, 0, 255), 2);
                line(dst, Point(corners[corners.size() - 1][3]),Point(corners[corners.size() - 1][1]), cv::Scalar(0, 0, 255), 2);
                line(dst, Point(corners[corners.size() - 1][0]),Point(corners[corners.size() - 1][1]), cv::Scalar(0, 0, 255), 2);
            } else
            {
                line(dst, Point(corners[corners.size() - 2][0]),Point(corners[corners.size() - 2][2]), cv::Scalar(255, 255, 255), 2);
                line(dst, Point(corners[corners.size() - 2][2]),Point(corners[corners.size() - 2][3]), cv::Scalar(255, 255, 255), 2);
                line(dst, Point(corners[corners.size() - 2][3]),Point(corners[corners.size() - 2][1]), cv::Scalar(255, 255, 255), 2);
                line(dst, Point(corners[corners.size() - 2][0]),Point(corners[corners.size() - 2][1]), cv::Scalar(255, 255, 255), 2);
            }
            //debug
//            cv::namedWindow("dst", 0);
//            cv::imshow("dst", dst);
//            cv::waitKey();
        }

        newWarped.clear();
    }
    else{

        struct timeval warp_start, warp_end;
        gettimeofday( &warp_start, NULL );

        for(int i = 0; i < g_inputImgNums; ++i)
        {
            if(i < refIndx)
            {
                Mat H_tmp = Mat::eye(3, 3, CV_64FC1);
                for(int k = i; k < refIndx; ++k){
                    H_tmp = H_tmp * g_angleImg[k+1].H;
                }
                NewHomoVecs[i] = H_tmp.inv();
            }
            else if(i > refIndx)
            {
                Mat H_tmp = Mat::eye(3, 3, CV_64FC1);
                for(int k = refIndx; k < i; ++k){
                    H_tmp = H_tmp * g_angleImg[k+1].H;
                }
                NewHomoVecs[i] = H_tmp;
            } else
            {
                NewHomoVecs[i] = Mat::eye(3, 3, CV_64FC1);
            }

            if(!mywarpPerspective(g_angleImg[i].inputImg, warped[i], NewHomoVecs[i], corners[i]))
            {
                if(!g_strLogPath.empty())
                {
                    output_file << "Stitch: warp failed. " << std::endl;
                }
                std::cout << "Stitch: warp failed. " << std::endl;

                if(g_angleImg.size() > 0)
                    g_angleImg.pop_back();
                return false;
            }

            //TODO CalcCorners 计算的是原始图片四个顶点在warp后的图像位置， 这里需要改成四个顶点的外接矩形坐标
            Point tmp0, tmp1, tl;
            tmp0.x = min(corners[i][0].x, corners[i][1].x);
            tmp1.x = min(corners[i][2].x, corners[i][3].x);
            tl.x = min(tmp0.x, tmp1.x);

            tmp0.y = min(corners[i][0].y, corners[i][1].y);
            tmp1.y = min(corners[i][2].y, corners[i][3].y);
            tl.y = min(tmp0.y, tmp1.y);

            sizes[i] = warped[i].size();
            resultCorners[i] = tl;
        }

        gettimeofday( &warp_end, NULL );
        //求出两次时间的差值，单位为us
        int warp_timeuse = 1000000 * (warp_end.tv_sec - warp_start.tv_sec ) + warp_end.tv_usec - warp_start.tv_usec;
        // printf("warp time: %d us\n", warp_timeuse);

        Rect dst_roi = resultRoi(resultCorners, sizes);

        //    Mat dst;
        dst.create(dst_roi.size(), CV_8UC3);
        dst.setTo(cv::Scalar::all(0));

        for(int i = 0; i < g_inputImgNums; ++i)
        {
            Mat gray;
            cvtColor(warped[i], gray, CV_BGR2GRAY);

            int dx = resultCorners[i].x - dst_roi.x;
            int dy = resultCorners[i].y - dst_roi.y;
            //std::cout << "dx = " << dx << ", dy = " << dy << std::endl;

            struct timeval copy_start, copy_end;
            gettimeofday( &copy_start, NULL );

            warped[i].copyTo(dst(Rect(dx, dy, warped[i].cols, warped[i].rows)), gray);
            gettimeofday( &copy_end, NULL );
            //求出两次时间的差值，单位为us
            int copy_timeuse = 1000000 * (copy_end.tv_sec - copy_start.tv_sec ) + copy_end.tv_usec - copy_start.tv_usec;
//            printf("copy time: %d us\n", copy_timeuse);
            //std::cout << "dx = " << dx << ", dy = " << dy << std::endl;
            corners[i][0].x = corners[i][0].x - dst_roi.x;
            corners[i][0].y = corners[i][0].y - dst_roi.y;
            corners[i][1].x = corners[i][1].x - dst_roi.x;
            corners[i][1].y = corners[i][1].y - dst_roi.y;
            corners[i][2].x = corners[i][2].x - dst_roi.x;
            corners[i][2].y = corners[i][2].y - dst_roi.y;
            corners[i][3].x = corners[i][3].x - dst_roi.x;
            corners[i][3].y = corners[i][3].y - dst_roi.y;

            //std::cout << "after corners[i] ="  << corners[i] << std::endl;

            if(i == g_inputImgNums - 1)
            {
                line(dst, Point(corners[i][0]),Point(corners[i][2]), cv::Scalar(0, 0, 255), 2);
                line(dst, Point(corners[i][2]),Point(corners[i][3]), cv::Scalar(0, 0, 255), 2);
                line(dst, Point(corners[i][3]),Point(corners[i][1]), cv::Scalar(0, 0, 255), 2);
                line(dst, Point(corners[i][0]),Point(corners[i][1]), cv::Scalar(0, 0, 255), 2);
            } else
            {
                line(dst, Point(corners[i][0]),Point(corners[i][2]), cv::Scalar(255, 255, 255), 2);
                line(dst, Point(corners[i][2]),Point(corners[i][3]), cv::Scalar(255, 255, 255), 2);
                line(dst, Point(corners[i][3]),Point(corners[i][1]), cv::Scalar(255, 255, 255), 2);
                line(dst, Point(corners[i][0]),Point(corners[i][1]), cv::Scalar(255, 255, 255), 2);
            }
        }
    }

    resultCornersPre.clear();
    resultCorners.clear();
    sizesPre.clear();
    sizes.clear();
    corners.clear();
    HomoVecs.clear();
    NewHomoVecs.clear();
    warped.clear();

    //debug
//    cv::rectangle(dst, Point(0,0),Point(dst.cols, dst.rows),cv::Scalar(0,255,0), 2);
    dst.copyTo(result);
    g_panoVec.push_back(result); //save pano ,by //zt
    std::cout << "result size = " << result.size() << std::endl;


    g_inputImgNums++;
    g_bestPovVecs.push_back(bestPOV); //保存bestPOV

    gettimeofday( &stitch_end, NULL );
    //求出两次时间的差值，单位为us
    int stitch_timeuse = 1000000 * ( stitch_end.tv_sec - stitch_start.tv_sec ) + stitch_end.tv_usec - stitch_start.tv_usec;
    if(!g_strLogPath.empty())
        output_file << "Stitch: total time is " << stitch_timeuse << "  us." << std::endl;
    printf("Stitch: total time: %d us\n", stitch_timeuse);

//    cv::namedWindow("result", 0);
//    cv::imshow("result", result);
//    cv::waitKey();
    cv::Mat temp;
    src.copyTo(temp);
    IdtSetTemplate(temp);
    return true;
}

void IdtStitchClean()
{
    g_inputImgNums = 0;

    if(g_panoVec.size() > 0)
        g_panoVec.clear();
    if(g_angleImg.size() > 0)
        g_angleImg.clear();

    if(!g_strLogPath.empty())
        output_file.close();
    if(src_feature.keypoints.size() > 0)
    {
        src_feature.keypoints.clear();
        src_feature.descriptors.release();
    }
    if(src_featureVec.size() > 0)
        src_featureVec.clear();

    isInited = false;
    if(g_bestPovVecs.size() > 0)
        g_bestPovVecs.clear();

    target_Angle.x = 0;
    target_Angle.y = 0;
    target_Angle.z = 0;
    if(targetAngleVecs.size() > 0)
        targetAngleVecs.clear();

    origin_keypoints.clear();
    keypoints.clear();
    startKeypoints.clear();
    startKeypointsVecs.clear();
    startDescriptorVecs.clear();
    find_back_tracking = false;
}

//
#ifndef OPTICALFLOW

//
Mat resize_input(Mat &frame){
    Mat output;
//    float scale = Rsize_scale;
    float scale = 0.25;
//    scale = Resize_gool/frame.cols;
    resize(frame,output,Size(),scale,scale,INTER_LINEAR);
    return output;
}

void resize_output(Mat &frame,vector<Point>& dst_pt){

    float scale = 4;
    for (int i = 0; i < 4; ++i) {
        dst_pt[i] =dst_pt[i]*scale;
    }
}

bool Check_Area_Correct(vector<Point>dst_pnts,cv::Mat &dst){

    //判断重叠区域是否为四边形
    if(dst_pnts.size() != 4 || dst_pnts[0].x >= dst_pnts[1].x || dst_pnts[3].x >= dst_pnts[2].x || dst_pnts[0].y >= dst_pnts[3].y || dst_pnts[1].y >= dst_pnts[2].y)
    {
        std::cout << "OverLap: overlap area is not Quadrilateral. " << std::endl;
        return false;
    }

    //判断四边形内角
    float topSide = getSideLength(dst_pnts[0], dst_pnts[1]); //四边形的上边
    float bottomSide = getSideLength(dst_pnts[2], dst_pnts[3]); //四边形的下边
    float leftSide = getSideLength(dst_pnts[0], dst_pnts[3]); //四边形的左边
    float rightSide = getSideLength(dst_pnts[1], dst_pnts[2]); //四边形的右边
    std::cout << "topSide = " << topSide << ", bottomSide = " << bottomSide << ", leftSide = " << leftSide << ", rightSide = " << rightSide << std::endl;


    float lefttopVec = getSideVec(dst_pnts[0], dst_pnts[1], dst_pnts[3]);
    float righttopVec = getSideVec(dst_pnts[1], dst_pnts[0], dst_pnts[2]);
    float rightbottomVec = getSideVec(dst_pnts[2], dst_pnts[1], dst_pnts[3]);
    float leftbottomVec = getSideVec(dst_pnts[3], dst_pnts[0], dst_pnts[2]);


    int angle0 = acos(lefttopVec / (topSide * leftSide)) * 180 / 3.1416 + 0.5;
    int angle1 = acos(righttopVec / (topSide * rightSide)) * 180 / 3.1416 + 0.5;
    int angle2 = acos(leftbottomVec / (bottomSide * leftSide)) * 180 / 3.1416 + 0.5;
    int angle3 = acos(rightbottomVec / (bottomSide * rightSide)) * 180 / 3.1416 + 0.5;

    std::cout << "angle0 = " << angle0 << ", angle1 = " << angle1 << ", angle2 = " << angle2 << ", angle3 = " << angle3 << std::endl;
    if(abs(angle0) < 60 || abs(angle1) < 60 || abs(angle2) < 60 || abs(angle3) < 60)
    {
        //else
        std::cout << "OverLap: inline angle < 30. " << std::endl;
        return false;
    }
    return true;
}

bool IdtSetTemplate(cv::Mat & frame){

//    target_Angle.x = angle.x;
//    target_Angle.y = angle.y;
//    target_Angle.z = angle.z;
//    targetAngleVecs.push_back(target_Angle);

    H_last_ = Mat::eye(3,3,CV_64FC1);
    H_optflow_ = Mat::eye(3,3,CV_64FC1);
    find_back_tracking = false;
    origin_keypoints.clear(); //只是一个list存储所有的初始点
    keypoints.clear();      // 因为要删除跟踪失败的点，使用list
    startKeypoints.clear();
    start_Descriptor.release();
    templateImg.release();
    Max_origin_size = Max_origin_;

    frame = resize_input(frame);

//    cv::Mat tempGray;
//    if(frame.channels() == 4)
//        cv::cvtColor(frame, tempGray, CV_BGRA2GRAY);
//    else if(frame.channels() == 3)
//        cv::cvtColor(frame, tempGray, CV_BGR2GRAY);
//    else
//        frame.copyTo(tempGray);

    keypoints.clear();
    origin_keypoints.clear();

    // 对第一帧提取FAST特征点
    vector<cv::KeyPoint> kps;
    Ptr<GFTTDetector> gftt = GFTTDetector::create(Max_origin_);
    gftt->detect(frame,kps);
    //    KeyPointsFilter::retainBest(kps, Max_origin_);
    for ( const auto& kp:kps ){
        keypoints.push_back( kp.pt );
        origin_keypoints.push_back( kp.pt);
    }

    cout  << "keypoints? = " << keypoints.size() <<endl;
    Ptr<xfeatures2d::SIFT> detail_detector = xfeatures2d::SIFT::create(Max_origin_,2);
    //    Ptr<DescriptorExtractor> detail_extractor= xfeatures2d::SIFT::create(Max_origin_);
    detail_detector->detectAndCompute(frame, Mat(), startKeypoints, start_Descriptor);
    //    cout << start_Descriptor.size <<  "? = " << startKeypoints.size() <<endl;
    gftt.release();
    detail_detector.release();
    //    detail_extractor.release();
    kps.clear();
    frame.copyTo(templateImg);

    //保存模板特征  zt
    startKeypointsVecs.push_back(startKeypoints);
    startDescriptorVecs.push_back(start_Descriptor);

    return true;
}

bool IdtCalOverLap(cv::Mat &dst, std::vector<cv::Point> &dst_pnts, float &Score){

    //zt add 20200109
//    if(abs(angle.x - target_Angle.x) > 25 || abs(angle.y - target_Angle.y) > 30 || abs(angle.z - target_Angle.z) > 20)
//    {
//        std::cout << "stitchInterFace: angle is invalid. " << angle << std::endl;
//        for(int i = 0; i < 4; ++i)
//            dst_pnts.push_back(cv::Point(0,0));
//
//        return false;
//    }
    dst_pnts.clear();
    if(startKeypointsVecs.size() <= 0 || startKeypointsVecs[startKeypointsVecs.size() - 1].size() <= 0)
    {
        std::cout << "OverLap: template kp size is 0 or no template. " << std::endl;
        return false;
    }

    //找回机制
    dst = resize_input(dst);
    if(find_back_tracking){
        cout<<"find_back_tracking" <<endl;
        if(Retrieve_Overlap(dst,Max_origin_))
            find_back_tracking  = !find_back_tracking;
        if (!find_back_tracking){
            Max_origin_size = Max_origin_;
            updateKeyPoint(dst,Max_origin_size);
        }
    }
    else{
        if(!Calc_Overlap(dst,Max_origin_size))
            find_back_tracking  = true;
    }
    //测量中心点的偏移
    vector<Point2f> Pic_centre;
    Pic_centre.push_back(Point2f((float)(dst.cols/2),(float)(dst.rows/2)));
    perspectiveTransform(Pic_centre,Pic_centre,H_optflow_);
    Score = Cal_rads_bias(1,dst,Pic_centre[0]);
    cout <<"cal rads " << (float)Score <<endl;
//    if(!Check_rads_bias(-0.7,dst,Pic_centre[0])) {
//        find_back_tracking = true;
//        return false;
//    }
    vector<Point> dst_(4);
    
    cal_area_corner(dst,H_optflow_,dst_pnts);
    cout << dst_pnts <<endl;
    if(!Check_Area_Correct(dst_pnts,dst))
        find_back_tracking = true;
    if (find_back_tracking)
        return false;
    resize_output(dst,dst_pnts);
//    for(const auto& pp:keypoints)
//        circle(dst,pp,2,Scalar(0,200,0),2);
    return true;
}

float Cal_rads_bias(float size,const Mat& pic_,const Point2f& center){

    float rad_difference = ((float)pic_.cols/pic_.rows);
    float rad_least = min(pic_.rows/2,pic_.cols/2);
    float centre_x = center.x - pic_.cols/2;
    float centre_y = rad_difference*(center.y - pic_.rows/2);
    float rad_dis = sqrtf(centre_x*centre_x+centre_y*centre_y );
    return rad_dis/rad_least;
}

bool Retrieve_Overlap(Mat& dst,int Max_origin_Size){
    vector<KeyPoint> frame_keypoints;
    Mat frame_descriptor;

//    cv::Mat dstGray;
//    if(dst.channels() == 4)
//        cv::cvtColor(dst, dstGray, CV_BGRA2GRAY);
//    else if(dst.channels() == 3)
//        cv::cvtColor(dst, dstGray, CV_BGR2GRAY);
//    else
//        dst.copyTo(dstGray);
    //先不断地去寻找 SIFT特征匹配
    Ptr<xfeatures2d::SIFT> detail_detector = xfeatures2d::SIFT::create(Detail_Max_,2);
    detail_detector->detectAndCompute(dst,Mat(),frame_keypoints,frame_descriptor);


    if(frame_keypoints.size() < startKeypoints.size() / 20)
        return false;
    vector<DMatch> matches;
    std::vector<Point2f> start_pt, frame_pt;  //重复区域匹配点list
    KnnMatcher(start_Descriptor, frame_descriptor, matches, 0.25);
    //将kNN的配对填入两个缓冲 vector
    for (auto m : matches) {
        //得到第一幅图像的当前匹配点对的特征点坐标
        Point2f p = startKeypoints[m.queryIdx].pt;
        start_pt.push_back(p);    //特征点坐标赋值
        p = frame_keypoints[m.trainIdx].pt;
        frame_pt.push_back(p);
    }

    if( start_pt.size()<6||frame_pt.size()<6)
        return false;

    std::vector<uchar> inliers_mask;
    cv::Mat H = findHomography(start_pt, frame_pt,inliers_mask,LMEDS);
    int num_inliers = 0;    //匹配点对的内点数先清零
    //由内点掩码得到内点数
    //在KNN基础上再得到内点 vector
    vector<Point2f>inlier_org,inlier_prj;
    for (size_t i = 0; i < inliers_mask.size(); ++i)    //遍历匹配点对，得到内点
    {
        if (!inliers_mask[i])    //不是内点
            continue;
        Point2f p = start_pt[i];    //第一幅图像的内点坐标
        inlier_org.push_back(p);
        p = frame_pt[i];    //第一幅图像的内点坐标
        inlier_prj.push_back(p);
        num_inliers++;
    }

    float confidence = num_inliers / (8 + 0.3 * start_pt.size());
    cout << "confidence = " << confidence << endl;
    float err = reprojError(inlier_org, inlier_prj, H);
    cout << "err = " << err <<endl;
    if (err>20){
        return false;
    }
    if(confidence<1.0)
        return false;
    if(!Check_center_crossborder(dst,-0.2,H.inv())){
        cout << "check false;" <<endl;
        return false;
    }
    //再次更新 两个H矩阵
    H.copyTo(H_optflow_);
    H.copyTo(H_last_);
//    cout << "auto_retrieve" <<endl;
    return true;
}

void cal_area_corner(const Mat& pic,const Mat& H_final,vector<Point>& output_pt) {
    std::vector<Point2f> template_pt(4);
    std::vector<Point2f> output_temp_corner(4);
    template_pt[0] = Point2f(0, 0);
    template_pt[1] = Point2f((float) pic.cols, 0);
    template_pt[2] = Point2f((float) pic.cols, (float) pic.rows);
    template_pt[3] = Point2f(0, (float) pic.rows);
    perspectiveTransform(template_pt, output_temp_corner, H_final);

    for (int j = 0; j < 4; ++j) {

#ifdef __linux__
        output_temp_corner[j].x = (int) (output_temp_corner[j].x * 1.0);
        output_temp_corner[j].y = (int) (output_temp_corner[j].y * 1.0);
        output_pt.push_back(output_temp_corner[j]);
#elif  __ANDROID__
        output_temp_corner[j].x = (int)(output_temp_corner[j].x * 1.0 );
    output_temp_corner[j].y = (int)(output_temp_corner[j].y * 1.0 );
    output_pt.push_back(output_temp_corner[j]);
#elif __APPLE__
    output_temp_corner[j].x = (int)(output_temp_corner[j].x * 1.0 );
    output_temp_corner[j].y = (int)(output_temp_corner[j].y * 1.0 );
    output_pt.push_back(output_temp_corner[j]);
//    output_temp_corner[j].x = output_temp_corner[j].x * 1.0 ;
//    output_temp_corner[j].y = output_temp_corner[j].y * 1.0 ;
//    output_pt[j] = Point((int)output_temp_corner[j].x,(int)output_temp_corner[j].y);
#endif
        std::cout << "output_temp_corner = " << output_temp_corner[j].x << ", y = " << output_temp_corner[j].y << std::endl;
        
        //output_file << "output_pt = " << output_pt[j].x << ", y = "<< output_pt[j].y << std::endl;
        std::cout << "output_pt = " << output_pt[j].x << ", y = " << output_pt[j].y << std::endl;
    }
}

void KnnMatcher(const Mat descriptors_1,const Mat descriptors_2,vector<DMatch>& Dmatchinfo,const float match_conf_) {
    Dmatchinfo.clear();    //清空
    //定义K-D树形式的索引
    Ptr<flann::IndexParams> indexParams = new flann::KDTreeIndexParams();
    //定义搜索参数
    Ptr<flann::SearchParams> searchParams = new flann::SearchParams();

    if (descriptors_1.depth() == CV_8U) {
        indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
        searchParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
    }
    //使用FLANN方法匹配，定义matcher变量
    FlannBasedMatcher matcher(indexParams, searchParams);
    vector<vector<DMatch> > pair_matches;    //表示邻域特征点
    MatchesSet matches;    //表示匹配点对

    // Find 1->2 matches
    //在第二幅图像中，找到与第一幅图像的特征点最相近的两个特征点
    matcher.knnMatch(descriptors_1, descriptors_2, pair_matches, 2);
    for (auto & pair_matche : pair_matches)    //遍历这两次匹配结果
    {
        //如果相近的特征点少于2个，则继续下个匹配
        if (pair_matche.size() < 2)
            continue;
        //得到两个最相近的特征点
        const DMatch &m0 = pair_matche[0];
        const DMatch &m1 = pair_matche[1];
        //比较这两个最相近的特征点的相似程度，当满足一定条件时（用match_conf_变量来衡量），才能认为匹配成功
        //TODO match_conf_越大说明第二相似的匹配点，要与第一相似的匹配点差距越大，也就是匹配的要求
        // 特例性，match_conf_ 越大 粗匹配点数越少
        if (m0.distance < (1.f - match_conf_) * m1.distance)    //式1
        {
            //把匹配点对分别保存在matches_info和matches中
            Dmatchinfo.push_back(m0);
            matches.insert(make_pair(m0.queryIdx, m0.trainIdx));
        }
    }

    pair_matches.clear();    //变量清零
}

bool Check_center_crossborder(const Mat& pic,float border_rate,Mat H){
    vector<Point2f> Pic_centre;
    Pic_centre.push_back(Point((pic.cols/2),(pic.rows/2)));
    perspectiveTransform(Pic_centre,Pic_centre,H);
    return Check_rads_bias(border_rate, pic, Pic_centre[0]);
}

bool Calc_Overlap(Mat &dst,int &maxOriginSize){
    // 对其他帧用LK跟踪特征点

    //填入 buffer
    vector<cv::Point2f> next_keypoints;
    vector<cv::Point2f> prev_keypoints;
    for ( const auto& kp:keypoints )
        prev_keypoints.push_back(kp);
    vector<unsigned char> status;
    vector<float> error;
    if (prev_keypoints.empty())
        return false;
//    cvtColo®gray, cv::COLOR_BGR2GRAY);
    //计算光流跟踪点
//    cv::Mat tempGray, dstGray;
//    if(templateImg.channels() == 4)
//        cv::cvtColor(templateImg, tempGray, CV_BGRA2GRAY);
//    else if(templateImg.channels() == 3)
//        cv::cvtColor(templateImg, tempGray, CV_BGR2GRAY);
//    else
//        templateImg.copyTo(tempGray);
//
//    if(dst.channels() == 4)
//        cv::cvtColor(dst, dstGray, CV_BGRA2GRAY);
//    else if(dst.channels() == 3)
//        cv::cvtColor(dst, dstGray, CV_BGR2GRAY);
//    else
//        dst.copyTo(dstGray);

    cv::calcOpticalFlowPyrLK( templateImg, dst, prev_keypoints, next_keypoints, status, error );

    // 把跟丢的点删掉
    int i=0;int number_stable = 0;
    auto iter_tmp = origin_keypoints.begin();
    for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
    {
        if (status[i] == 0 ){
            //同步删除那些origin_keypoints 中的对应点
            iter = keypoints.erase(iter);
            iter_tmp = origin_keypoints.erase(iter_tmp);
            continue;
        }
        iter_tmp++;
        *iter = next_keypoints[i];
        iter++;
    }

    cout << "tracked keypoints: " << keypoints.size()<<endl;
    cout << "origin_keypoints keypoints: " << origin_keypoints.size()<<endl;
    //如果光流跟踪到的点特别少直接退出
    if(keypoints.size()<Max_origin_/10)
        return false;

    Mat src_points(1, static_cast<int>(prev_keypoints.size()), CV_32FC2);
    Mat dst_points(1, static_cast<int>(prev_keypoints.size()), CV_32FC2);
    vector<KeyPoint> srcKeypoint,dstKeypoint;
    //遍历所有匹配点对，得到匹配点对的特征点坐标
    auto psrc =origin_keypoints.begin();
    auto pdst =keypoints.begin();
    for (size_t step = 0; step < prev_keypoints.size(); ++step) {
        src_points.at<Point2f>(0, static_cast<int>(step)) = *psrc;    //特征点坐标赋值
        dst_points.at<Point2f>(0, static_cast<int>(step)) = *pdst;    //特征点坐标赋值
        psrc++;
        pdst++;
    }
    vector<uchar> origin_kp_mask;
    Mat H_temp = findHomography(src_points,dst_points,LMEDS,5,
                                origin_kp_mask,1000,0.99);
    H_optflow_ = H_temp*H_last_;

    //zt
    if(abs(H_optflow_.at<double>(0,0)) > 2 || abs(H_optflow_.at<double>(0,0)) <= 0.2 || abs(H_optflow_.at<double>(1,1)) > 2 || abs(H_optflow_.at<double>(1,1)) <= 0.2)
    {
        return false;
    }

    //cout << H_optflow_ <<endl;
    //todo 如果光流跟踪点数量少于0.9
//    if(Max_origin_size < Max_origin_/3)
//        return false;
    if (keypoints.size() < maxOriginSize * Trans_rate)
    {
//        return false;
        updateKeyPoint(dst, maxOriginSize);
        H_optflow_.copyTo(H_last_);
    }
    dst.copyTo(templateImg);

    return true;
}

void updateKeyPoint(const Mat& frame,int &MOS){
    //先清空 vector point buffer
    keypoints.clear();
    origin_keypoints.clear();
    vector<cv::KeyPoint> kps;
    Ptr<GFTTDetector> gftt =
            GFTTDetector::create(Max_origin_,
                                 0.01,
                                 1,
                                 3,
                                 true);
    gftt->detect(frame,kps);
    MOS = kps.size();

    for ( const auto& kp:kps ){
        keypoints.push_back( kp.pt );
        origin_keypoints.push_back( kp.pt);
    }
    frame.copyTo(templateImg);
}

Size Get_Overlap_Area(vector<Point2f> corners){
    // 这里需要改成四个顶点的外接矩形坐标
    Point tmp0, tmp1, LT_position,RB_position;
    //LT_position 是该图片在全景图中的左上角位置坐标
    tmp0.x = min(corners[0].x, corners[1].x);
    tmp1.x = min(corners[2].x, corners[3].x);
    LT_position.x = min(tmp0.x, tmp1.x);

    tmp0.y = min(corners[0].y, corners[1].y);
    tmp1.y = min(corners[2].y, corners[3].y);
    LT_position.y = min(tmp0.y, tmp1.y);

    tmp0.x = max(corners[0].x, corners[1].x);
    tmp1.x = max(corners[2].x, corners[3].x);
    RB_position.x = max(tmp0.x, tmp1.x);

    tmp0.y = max(corners[0].y, corners[1].y);
    tmp1.y = max(corners[2].y, corners[3].y);
    RB_position.y = max(tmp0.y, tmp1.y);
    Size returnout(-LT_position.x+RB_position.x,
                   -LT_position.y+RB_position.y);
    return returnout;
}

bool Check_rads_bias(float size,const Mat& pic_,const Point2f& center){

    float rad_least = min(pic_.rows/2,pic_.cols/2);
    float centre_x = center.x - pic_.cols/2;
    float centre_y = center.y - pic_.rows/2;
//    cout << centre_x << " " <<centre_y << endl;
//    float centre_y;
    float rad_dis = sqrtf(centre_x*centre_x+centre_y*centre_y);
//    cout << "rand_dis" << rad_dis <<endl;
    return (rad_dis < rad_least*(1-size));

}

float reprojError(vector<Point2f> pt_org, vector<Point2f> pt_prj, const Mat& H_front)
{
    //m1和m2为匹配点对
//model表示单应矩阵H
//_err表示所有匹配点对的重映射误差，即式21几何距离的平方
    Mat H_inerr;
    H_front.copyTo(H_inerr);
//    cout << "H_inerr : " <<H_inerr <<endl;
    vector<double> err;
    err.clear();
//    int count = pt_org.size();
//    const double* H = reinterpret_cast<const double*>(H_front.data);
    for(int i = 0; i < pt_org.size(); i++ )    //遍历所有特征点，计算重映射误差
    {
        double ww = 1./(H_inerr.at<double>(2,0)*pt_org[i].x + H_inerr.at<double>(2,1)*pt_org[i].y + 1.);    //式21中分式的分母部分
        double dx = (H_inerr.at<double>(0,0)*pt_org[i].x +
                     H_inerr.at<double>(0,1)*pt_org[i].y +
                     H_inerr.at<double>(0,2))*
                    ww - pt_prj[i].x;    //式21中x坐标之差
        double dy = (H_inerr.at<double>(1,0)*pt_org[i].x +
                     H_inerr.at<double>(1,1)*pt_org[i].y +
                     H_inerr.at<double>(1,2))*
                    ww - pt_prj[i].y;    //式21中y坐标之差
        err.push_back((double)dx*dx + (double)dy*dy);    //得到当前匹配点对的重映射误差
    }
    double err_mean =  (accumulate(err.begin(),err.end(),0.0))/(double)pt_org.size() ;
    return (float)err_mean;
}

#endif
