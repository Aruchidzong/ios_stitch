//
//  StitchManager.m
//  SDKDemo
//
//  Created by KanDao on 2018/11/25.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import "StitchManager.h"
#import "capture_stitching_mobile_video.hpp"
#import <opencv2/core/utility.hpp>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/imgcodecs/imgcodecs.hpp>
#import "FileManager.h"

@interface StitchManager ()

@property (atomic, assign) bool overlapSrcSet;

@end

@implementation StitchManager

+ (instancetype)sharedInstance
{
    static dispatch_once_t p = 0;
    __strong static id _sharedObject = nil;
    
    dispatch_once(&p, ^{
        _sharedObject = [[self alloc] init];
    });
    
    return _sharedObject;
}

-(id)init{
    if(self = [super init]){
        self.overlapSrcSet = NO;
    }
    return self;
}

/**
 * initialize stitcher with a given quality
 */
-(void)start:(int)quality warpType:(int)warpType maxEdge:(int)maxEdge{
    capture_stitching_init(quality, warpType);
}

/**
 * re-initialize stitcher with a given quality
 */
-(void)restart:(int)quality warpType:(int)warpType maxEdge:(int)maxEdge{
    [self stop];
    [self start:quality warpType:warpType maxEdge:maxEdge];
}

/**
 * stop stitcher
 */
-(void)stop{
    capture_stitching_release();
}

/**
 * stitch a new image and return the stitched image.
 */
-(void)stitch:(nullable NSString*)imagePath outputPath:(nullable NSString*)outputPath retake:(BOOL)retake completionHandler:(void(^)(NSError*))handler{
    dispatch_async(dispatch_get_global_queue(0, 0), ^{
        cv::Mat src, dst;
        if(imagePath != nil){
            // load input image
            src = cv::imread(imagePath.UTF8String);
            
        }

        // run stitch and return result
        NSError* error = nil;
        if(capture_pic(src, dst, retake ? 1 : 0)){
            if(outputPath != nil){
                NSLog(@"xdf out %d x %d", dst.cols, dst.rows);
                cv::imwrite(outputPath.UTF8String, dst);
            }
        }else{
            error = [NSError errorWithDomain:@"com.imagedt.stitch" code:101 userInfo:@{NSLocalizedDescriptionKey:@"拼接失败"}];
        }
        if(handler){
            handler(error);
        }
    });
}

-(BOOL)setOverlapSrc:(nullable NSString*)imagePath{
    if(imagePath != nil){
        cv::Mat src = cv::imread(imagePath.UTF8String);
        bool res = set_src_feature(src);
        if(res){
            self.overlapSrcSet = YES;
            return YES;
        }
    }
    return NO;
}

-(BOOL)calcOverlap:(uint8_t*)imageData width:(int)width height:(int)height points:(float*)points{
    
    if(!self.overlapSrcSet){
        return NO;
    }

    cv::Mat dst(cv::Size(width, height), CV_8UC1, imageData, cv::Mat::AUTO_STEP);
    
    // rotate dst to portraint
    cv::rotate(dst, dst, cv::ROTATE_90_CLOCKWISE);
    
    // call overlap_points
    std::vector<cv::Point> srcPnts(4);
    std::vector<cv::Point> dstPnts(4);
    
    bool res = overlap_point(dst, srcPnts, dstPnts);
    if(res){
        for(int i = 0 ; i < 4; i ++){
            points[i*2] = (float)srcPnts[i].x/height;
            points[i*2+1] = (float)srcPnts[i].y/width;
            points[i*2+8] = (float)dstPnts[i].x/height;
            points[i*2+9] = (float)dstPnts[i].y/width;
        }
        return YES;
    }
    
    return NO;
}

-(void)saveImage:(uint8_t*)data width:(int)width height:(int)height{
    
    cv::Mat mat(cv::Size(width, height), CV_8UC1, data, cv::Mat::AUTO_STEP);
    NSString* path = [FileManager.sharedInstance pathForImageAtIndex:1000];
    cv::imwrite(path.UTF8String, mat);
}

@end
