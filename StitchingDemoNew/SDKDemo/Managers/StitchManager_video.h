//
//  StitchManager.h
//  SDKDemo
//
//  Created by KanDao on 2018/11/25.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>
#import <CoreServices/CoreServices.h>

NS_ASSUME_NONNULL_BEGIN

@interface StitchManager : NSObject

+ (instancetype)sharedInstance;
- (UIImage *)imageConvert:(CMSampleBufferRef)sampleBuffer scale:(float)scale orientation:(UIImageOrientation)orientation;
//planA
-(void)overlapOfImage:(UIImage*)image0
       frameIndex:(int)frameIndex
      angle_pitch:(float)pitch
       angle_roll:(float)roll
        angle_yaw:(float)yaw
completionHandler:(void(^)(bool success,
                           NSString* msg,
                           NSArray<NSNumber*> *overPoint,
                           float currentHintLinesWidth,
                           float currentHintLinesHeight,
                           float score))handler;
-(void)overlapOfBuffer:(CMSampleBufferRef)sampleBuffer
       frameIndex:(int)frameIndex
      angle_pitch:(float)pitch
       angle_roll:(float)roll
        angle_yaw:(float)yaw
completionHandler:(void(^)(bool success,
                           NSString* msg,
                           NSArray<NSNumber*> *overPoint,
                           float currentHintLinesWidth,
                           float currentHintLinesHeight,
                           float score))handler;

-(void)stitchOfImage:(UIImage*)image
         angle_pitch:(float)pitch
          angle_roll:(float)roll
           angle_yaw:(float)yaw
             logPath:(NSString*)log
            panoPath:(NSString*)pano
   completionHandler:(void(^)(bool success,
                              NSString* msg,
                              UIImage *panoImg,
                              int bestPOV,
                              NSArray *homography))handler;
-(void)delLastOfCompletionHandler:(void(^)(bool success,
                                           NSString* msg))handler;
-(void)resetOfLogPath:(NSString*)log
    completionHandler:(void(^)(bool success,
                               NSString* msg))handler;
-(void)stitchClean;
@end


NS_ASSUME_NONNULL_END
