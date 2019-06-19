//
//  StitchManager.m
//  SDKDemo
//
//  Created by KanDao on 2018/11/25.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import "StitchManager.h"
#import "capture_stitching_mobile.hpp"
#import <opencv2/core/utility.hpp>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/imgcodecs/imgcodecs.hpp>
#import "FileManager.h"

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

/**
 * initialize stitcher with a given quality
 */
-(void)start:(int)quality{
    capture_stitching_init(quality, 1);
}

/**
 * re-initialize stitcher with a given quality
 */
-(void)restart:(int)quality{
    [self stop];
    [self start:quality];
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
-(void)stitch:(nullable NSString*)imagePath outputPath:(nullable NSString*)outputPath logPath:(NSString*)logPath retake:(BOOL)retake completionHandler:(void(^)(NSError*))handler{
    dispatch_async(dispatch_get_global_queue(0, 0), ^{
        cv::Mat src, dst;
        std::vector<cv::Mat> imgs;
        std::string log = [logPath UTF8String];

        if(imagePath != nil){
            // load input image
            src = cv::imread(imagePath.UTF8String);
            if (src.empty()) {
                std::cout << "src is empty." <<std::endl;
            }else{
                imgs.emplace_back(src);
            }
        }

        // run stitch and return result
        NSError* error = nil;
        if(stitching(imgs, dst,  retake ? 1 : 0, log)){
            if(outputPath != nil){
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

@end
