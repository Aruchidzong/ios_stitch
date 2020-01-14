//
//  StitchManager.m
//  SDKDemo
//
//  Created by KanDao on 2018/11/25.
//  Copyright © 2018 KandaoVR. All rights reserved.
//
#include "opencv2/opencv.hpp"

#import "StitchManager_video.h"
#include <iostream>
#import "capture_stitching_mobile_video.hpp"
#import <opencv2/core/utility.hpp>
#import <opencv2/imgcodecs/imgcodecs.hpp>
//#import <opencv2/imgcodecs/ios.h>
//#import <StitchCameraSDK/ImageStitchManager.h>
//#import <StitchCameraSDK/ImageStitchMangerDelegate.h>

#import "FileManager.h"
//#import <IdtRes/IdtRes.h>
#import <objc/runtime.h>
/*
@implementation ImageStitchManager (Hook)
+ (void)load {
//    return;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
//        Class class = [self class];
        SEL originalSelector = @selector(imageFromSampleBuffer:);
        SEL swizzledSelector = @selector(idt_imageFromSampleBuffer:);
      
        Method originalMethod = class_getInstanceMethod( [self class], originalSelector);
        Method swizzledMethod = class_getInstanceMethod( [self class], swizzledSelector);
      
        BOOL success = class_addMethod([self class],
                                     originalSelector,
                                     method_getImplementation(swizzledMethod),
                                     method_getTypeEncoding(swizzledMethod));
        if (success) {
           class_replaceMethod([self class],
                              swizzledSelector,
                              method_getImplementation(originalMethod),
                              method_getTypeEncoding(originalMethod));
        } else {
           method_exchangeImplementations(originalMethod, swizzledMethod);
        }
              
      });
}

- (id)idt_imageFromSampleBuffer:(struct opaqueCMSampleBuffer *)arg1{
//    UIImage *img = [[StitchManager sharedInstance] imageConvert:arg1 scale:1.0 orientation:UIImageOrientationRight];
    UIImage *img = [[StitchManager sharedInstance] imageConvert:arg1 scale:1.0 orientation:UIImageOrientationUp];

    return img;
}

- (id)idt_UIImageFromCVMat:(struct Mat)arg1{
//    UIImage *img = [[StitchManager sharedInstance] UIImageFromCVMat:arg1];
    return nil;

}
@end


 */

@interface StitchManager ()//<ImageStitchMangerDelegate>
{
    cv::detail::ImageFeatures points;
    CGSize lastSize;
    
}

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

- (instancetype)init
{
    self = [super init];
    if (self) {
 
    }
    return self;
}

-(UIImage *)UIImageFromCVMat:(cv::Mat)cvMat changeColor:(BOOL)change{
    
    if (cvMat.empty()) {
        std::cout << "cvMat is empty " << std::endl;

        return nil;
    }
     if(change && cvMat.type() == CV_8UC3)
        {
            cvtColor(cvMat,cvMat, CV_BGR2RGB);
//            std::cout << "m type is CV_8UC4. " << std::endl;
    
        }
    
    NSData *data = [NSData dataWithBytes:cvMat.data length:cvMat.elemSize()*cvMat.total()];
    CGColorSpaceRef colorSpace;
    if (cvMat.elemSize() == 1) {
        colorSpace = CGColorSpaceCreateDeviceGray();
    } else {
        colorSpace = CGColorSpaceCreateDeviceRGB();
    }
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
    // Creating CGImage from cv::Mat
    CGImageRef imageRef = CGImageCreate(cvMat.cols,                                 //width
                                        cvMat.rows,                                 //height
                                        8,                                          //bits per component
                                        8 * cvMat.elemSize(),                       //bits per pixel
                                        cvMat.step[0],                            //bytesPerRow
                                        colorSpace,                                 //colorspace
                                        kCGImageAlphaNone|kCGBitmapByteOrderDefault,// bitmap info
                                        provider,                                   //CGDataProviderRef
                                        NULL,                                       //decode
                                        false,                                      //should interpolate
                                        kCGRenderingIntentDefault                   //intent
                                        );
    // Getting UIImage from CGImage
    UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    return finalImage;
}
- (cv::Mat)cvMatFromUIImage:(UIImage *)image {
    
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat cols = image.size.width;
    CGFloat rows = image.size.height;
    
    cv::Mat cvMat(rows, cols, CV_8UC4); // 8 bits per component, 4 channels (color channels + alpha)
    
    CGContextRef contextRef = CGBitmapContextCreate(cvMat.data,                 // Pointer to  data
                                                    cols,                       // Width of bitmap
                                                    rows,                       // Height of bitmap
                                                    8,                          // Bits per component
                                                    cvMat.step[0],              // Bytes per row
                                                    colorSpace,                 // Colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault); // Bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows), image.CGImage);
//    CGColorSpaceRelease(colorSpace);
    CGContextRelease(contextRef);
    if(cvMat.type() == CV_8UC4)
    {
        cvtColor(cvMat,cvMat, CV_RGBA2BGR);
//        std::cout << "m type is CV_8UC4. " << std::endl;

    }

      return cvMat;
}
void UIImageToMat(const UIImage* image,
                  cv::Mat& m, bool alphaExist) {
    
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat cols = CGImageGetWidth(image.CGImage), rows = CGImageGetHeight(image.CGImage);
    CGContextRef contextRef;
    CGBitmapInfo bitmapInfo = kCGImageAlphaPremultipliedLast;
    if (CGColorSpaceGetModel(colorSpace) == kCGColorSpaceModelMonochrome)
    {
        m.create(rows, cols, CV_8UC1); // 8 bits per component, 1 channel
        bitmapInfo = kCGImageAlphaNone;
        if (!alphaExist)
            bitmapInfo = kCGImageAlphaNone;
        else
            m = cv::Scalar(0);
        contextRef = CGBitmapContextCreate(m.data, m.cols, m.rows, 8,
                                           m.step[0], colorSpace,
                                           bitmapInfo);
    }
    else
    {
        m.create(rows, cols, CV_8UC4); // 8 bits per component, 4 channels
        if (!alphaExist)
            bitmapInfo = kCGImageAlphaNoneSkipLast |
            kCGBitmapByteOrderDefault;
        else
            m = cv::Scalar(0);
        
        contextRef = CGBitmapContextCreate(m.data, m.cols, m.rows, 8,
                                           m.step[0], colorSpace,
                                           bitmapInfo);
    }
    

    CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows),
                       image.CGImage);

    CGContextRelease(contextRef);
//    CGColorSpaceRelease(colorSpace);
//    UIImage *image2 = [self UIImageFromCVMat:src];
//    NSLog(@"%@",image2);

    if(m.type() == CV_8UC4)
    {
//        NSString * path = [FileManager.sharedInstance pathForStitchingErrorImageAtItemId:@"m"];
//        cv::imwrite([path UTF8String],m);

        cvtColor(m,m, CV_RGBA2BGR);
//        NSString * path1 = [FileManager.sharedInstance pathForStitchingErrorImageAtItemId:@"m1"];
//        cv::imwrite([path1 UTF8String],m);
//        std::cout << "m type is CV_8UC4. " << std::endl;
        
    }
    
}
//urlEncode解码
- (NSString *)decoderUrlEncodeStr: (NSString *) input{
    NSMutableString *outputStr = [NSMutableString stringWithString:input];
    [outputStr replaceOccurrencesOfString:@"+" withString:@"" options:NSLiteralSearch range:NSMakeRange(0,[outputStr length])];
    return [outputStr stringByRemovingPercentEncoding];
}
- (cv::Mat)matFromImageBuffer: (CVPixelBufferRef)pixelBuffer {
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
    void *baseaddress = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
    int bufferWidth = (int)CVPixelBufferGetWidthOfPlane(pixelBuffer,0);
    int bufferHeight = (int)CVPixelBufferGetHeightOfPlane(pixelBuffer, 0);
    
    cv::Mat converted;
    
    // Get the yPlane (Luma values)
    cv::Mat yPlane = cv::Mat(bufferHeight, bufferWidth, CV_8UC1, baseaddress);
    
    // Get cbcrPlane (Chroma values)
    int cbcrWidth = (int)CVPixelBufferGetWidthOfPlane(pixelBuffer,1);
    int cbcrHeight = (int)CVPixelBufferGetHeightOfPlane(pixelBuffer, 1);
    void *cbcrAddress = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1);
    // Since the CbCr Values are alternating we have 2 channels: Cb and Cr. Thus we need to use CV_8UC2 here.
    cv::Mat cbcrPlane = cv::Mat(cbcrHeight, cbcrWidth, CV_8UC2, cbcrAddress);
    
    // Split them apart so we can merge them with the luma values
    std::vector<cv::Mat> cbcrPlanes;
    cv::split(cbcrPlane, cbcrPlanes);
    
    cv::Mat cbPlane;
    cv::Mat crPlane;
    
    // Since we have a 4:2:0 format, cb and cr values are only present for each 2x2 luma pixels. Thus we need to enlargen them (by a factor of 2).
    cv::resize(cbcrPlanes[0], cbPlane, yPlane.size(), 0, 0, cv::INTER_NEAREST);
    cv::resize(cbcrPlanes[1], crPlane, yPlane.size(), 0, 0, cv::INTER_NEAREST);
    
    cv::Mat ycbcr;
    std::vector<cv::Mat> allPlanes = {yPlane, cbPlane, crPlane};
    cv::merge(allPlanes, ycbcr);
    
    // ycbcr now contains all three planes. We need to convert it from YCbCr to BGR so OpenCV can work with it
    cv::cvtColor(ycbcr, converted, cv::COLOR_YCrCb2BGR);
    
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
    
    return converted;
}
bool YV12ToBGR24_OpenCV(unsigned char* pYUV,unsigned char* pBGR24,int width,int height)
{
    if (width < 1 || height < 1 || pYUV == NULL || pBGR24 == NULL)
        return false;
    cv::Mat dst(height,width,CV_8UC3,pBGR24);
    cv::Mat src(height + height/2,width,CV_8UC1,pYUV);
    cvtColor(src,dst,CV_YUV2BGR_YV12);
    return true;
}

#pragma mark - ImageStitchManger
- (UIImage*)imageFromSampleBuffer:(struct opaqueCMSampleBuffer *)buffer{
    return nil; // [[ImageStitchManager shared] imageFromSampleBuffer:buffer];
}
- (UIImage *)imageConvert:(CMSampleBufferRef)sampleBuffer scale:(float)scale orientation:(UIImageOrientation)orientation
{
    
//    @autoreleasepool{
//        CFRetain(sampleBuffer);
//        CVPixelBufferRef pixelBuffer = (CVPixelBufferRef)CMSampleBufferGetImageBuffer(sampleBuffer);
        CVPixelBufferRef pixelBuffer = CVPixelBufferRetain((CVPixelBufferRef)CMSampleBufferGetImageBuffer(sampleBuffer));
        CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
        CIImage *ciImage = [CIImage imageWithCVPixelBuffer:pixelBuffer];
        
        CIContext *temporaryContext = [CIContext contextWithOptions:nil];
        CGImageRef videoImage = [temporaryContext
                                 createCGImage:ciImage
                                 fromRect:CGRectMake(0, 0,
                                                     CVPixelBufferGetWidth(pixelBuffer),
                                                     CVPixelBufferGetHeight(pixelBuffer))];
        UIImage *uiImage = [[UIImage alloc] initWithCGImage:videoImage scale:1.0 orientation:orientation];
        
        CGImageRelease(videoImage);
        CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
        CVPixelBufferRelease(pixelBuffer);
//        CFRelease(sampleBuffer);
        return uiImage;

//    }
    
}
- (UIImage*)sendStitching:(CMSampleBufferRef)buffer frameIndex:(int)index isUnitImage:(int)isUnit
{
//    [[ImageStitchManager shared] sendStitchParameters:buffer frameIndex:index isUnitImage:isUnit];
//    if (isUnit) {
//        return [[ImageStitchManager shared] imageFromSampleBuffer: buffer];
//    }else{
//        return nil;
//    }
    return nil;

}
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
                              NSArray *homography))handler{
    
    cv::Mat src = [self cvMatFromUIImage:image];
    if (src.empty() || src.rows < 1){
        //        mode == 3 重置
        NSLog(@"空图片");
        handler(false,@"空图片",NULL,0,{});
        
        //            [self didCatchError:-1011 frameIndex:frameIndex];
        
        return ;
    }
    cv::Mat dsc;
    cv::Point3f angle = cvPoint3D32f(pitch,roll,yaw);
    int basePOV = 0;
    double homo[9] = {0,0,0,0,0,0,0,0,0};
    std::string logPath = [log UTF8String];
    
    // call overlap_points
    std::vector<cv::Point> dstPnts(4);

    if (IdtStitch(src, angle, dsc, basePOV, homo)) {
        
        NSMutableArray<NSNumber*> *list = [[NSMutableArray alloc] init];
        for (int i = 0; i < 9 ; i++) {
            [list addObject:[[NSNumber alloc] initWithDouble:homo[i]]];
        }
//        if (cv::imwrite([pano UTF8String],dsc)) {
//            handler(true,@"",nil,basePOV,list);
//        }else{
            UIImage *image = [self UIImageFromCVMat:dsc changeColor:true];
                //        UIImage *image2 = [self UIImageFromCVMat:dsc changeColor:false];
            NSLog(@"image:%@",image);
            if (image == nil) {
                image = [[UIImage alloc] init];
            }else{
                
            }
            handler(true,@"",image,basePOV,list);
//        }

        
    }else{
        
        handler(false,@"拼接失败",NULL,0,{});
        
    }
}

-(void)delLastOfCompletionHandler:(void(^)(bool success,
                           NSString* msg))handler{
    cv::Mat src;
    IdtRetry(src);
    handler(true,@"");
}

-(void)stitchClean{
    IdtStitchClean();
}

-(void)resetOfLogPath:(NSString*)log
completionHandler:(void(^)(bool success,
                           NSString* msg))handler{
    cv::Mat src,dsc;
    std::string logPath = [log UTF8String];
    
    // call overlap_points
    IdtStitchClean();
    if (IdtStitchInit(logPath)) {
        handler(true,@"");
    }else{
        handler(false,@"");
    }

}

-(void)overlapOfBuffer:(CMSampleBufferRef)sampleBuffer
       frameIndex:(int)frameIndex
             mode:(int)mode
      angle_pitch:(float)pitch
       angle_roll:(float)roll
        angle_yaw:(float)yaw
completionHandler:(void(^)(bool success,
                           NSString* msg,
                           NSArray<NSNumber*> *overPoint,
                           float currentHintLinesWidth,
                           float currentHintLinesHeight,
                           float score))handler{
    @autoreleasepool {
        CFRetain(sampleBuffer);
        CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
        CVPixelBufferLockBaseAddress(imageBuffer, 0);
        void *imageData = CVPixelBufferGetBaseAddressOfPlane(imageBuffer,0);
        
        cv::Mat src(cv::Size((int)CVPixelBufferGetWidth(imageBuffer), (int)CVPixelBufferGetHeight(imageBuffer)), CV_8UC1, imageData);
        
        //        std::string imgPath = [srcPath UTF8String];
        //            cv::rotate(src, src, cv::ROTATE_90_CLOCKWISE);
        
        if (src.empty() || src.rows < 1) {
            NSLog(@"空图片");
            return ;
        }
        
        cv::Mat dsc;
//        cv::Point3f angle = cvPoint3D32f(pitch,roll,yaw);
//        int basePOV = 0;
//        double homo[9] = {0,0,0,0,0,0,0,0,0};
        
        std::string logPath = [@"" UTF8String];
        float score = 0;
        // call overlap_points
        std::vector<cv::Point> dstPnts(4);
        
        if( IdtCalOverLap(src, dstPnts, score)) {
            float width = src.cols;
            float height = src.rows;
            NSMutableArray<NSNumber*> *list = [[NSMutableArray alloc] init];
            [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[0].x]];
            [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[0].y]];
            [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[1].x]];
            [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[1].y]];
            [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[2].x]];
            [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[2].y]];
            [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[3].x]];
            [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[3].y]];
            
            UIImage *image = [[UIImage alloc] init];
            if (mode != 0) {
                image = [self UIImageFromCVMat:dsc changeColor:true];
                //        UIImage *image2 = [self UIImageFromCVMat:dsc changeColor:false];
                NSLog(@"image:%@",image);
                if (image == nil) {
                    image = [[UIImage alloc] init];
                }else{
                    
                }
            }
            CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
            CFRelease(sampleBuffer);
            
            handler(true,@"",list,width,height,score);
        }else{
            CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
            CFRelease(sampleBuffer);
            
            handler(false,@"",nil,1,1,0);
        }
        
        
        
    }

}
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
                           float score))handler{
    cv::Mat src = [self cvMatFromUIImage:image0];
    if (src.empty() || src.rows < 1){
        //        mode == 3 重置
        NSLog(@"空图片");
        handler(false,@"",nil,1,1,0);

        return ;
    }
    cv::Mat dsc;
    cv::Point3f angle = cvPoint3D32f(pitch,roll,yaw);
    int basePOV = 0;
    double homo[9] = {0,0,0,0,0,0,0,0,0};
    std::string logPath = [@"" UTF8String];
    float score = 0;

    // call overlap_points
    std::vector<cv::Point> dstPnts(4);
    
    float width = src.cols;
    float height = src.rows;
    if (IdtCalOverLap(src, dstPnts, score)) {
        //(dstPnts[1].x - dstPnts[0].x)/4 + dstPnts[0].x
        NSMutableArray<NSNumber*> *list = [[NSMutableArray alloc] init];
        [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[0].x]];
        [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[0].y]];
        [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[1].x]];
        [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[1].y]];
        [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[2].x]];
        [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[2].y]];
        [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[3].x]];
        [list addObject:[[NSNumber alloc] initWithDouble:dstPnts[3].y]];
        handler(true,@"",list,width,height,score);
    }else{
        handler(false,@"",nil,1,1,0);
    }
}

-(void)stitchOfBuffer:(CMSampleBufferRef)sampleBuffer
           frameIndex:(int)frameIndex
                 mode:(int)mode
          angle_pitch:(float)pitch
           angle_roll:(float)roll
            angle_yaw:(float)yaw{
    /*
        @autoreleasepool {
            CFRetain(sampleBuffer);
            CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
            CVPixelBufferLockBaseAddress(imageBuffer, 0);
            void *imageData = CVPixelBufferGetBaseAddressOfPlane(imageBuffer,0);
    //        void *imageData = CVPixelBufferGetBaseAddress(imageBuffer);
            
//            cv::Mat src(cv::Size((int)CVPixelBufferGetWidth(imageBuffer), (int)CVPixelBufferGetHeight(imageBuffer)), CV_8UC1, imageData);

    //        std::string imgPath = [srcPath UTF8String];
//            cv::rotate(src, src, cv::ROTATE_90_CLOCKWISE);

//            if (src.empty() || src.rows < 1) {
//                NSLog(@"空图片");
//                return ;
//            }

            cv::Mat dsc;
            cv::Point3f angle = cvPoint3D32f(pitch,roll,yaw);
            int basePOV = 0;
            double homo[9] = {0,0,0,0,0,0,0,0,0};

            // call overlap_points
            std::vector<cv::Point> srcPnts(4);
            std::vector<cv::Point> dstPnts(4);

//            UIImage *image0 = [self UIImageFromCVMat:src];
//            NSLog(@"%@",image0);

            float width = CVPixelBufferGetWidth(imageBuffer);
            float height = (int)CVPixelBufferGetHeight(imageBuffer);

            
            if (IdtStitchInterFaceBuffer(imageData, width,height , mode, angle, dsc, basePOV, homo, srcPnts, dstPnts)) {
                

                NSMutableArray<NSNumber*> *list = [[NSMutableArray alloc] init];
                for (int i = 0; i < 9 ; i++) {
                    [list addObject:[[NSNumber alloc] initWithDouble:homo[i]]];
                }

                UIImage *image = [self UIImageFromCVMat:dsc  changeColor:true];
                NSLog(@"image:%@",image);
                if (image == nil) {
                    image = [[UIImage alloc] init];
                }
                    
                [self didGetNewPreviewImage:image
                                isUnitImage:mode
                           isValidHintLines:0
                      currentHintLinesWidth:width
                     currentHintLinesHeight:height
                             currentHintP1x:dstPnts[0].x
                             currentHintP1y:dstPnts[0].y
                             currentHintP2x:dstPnts[1].x
                             currentHintP2y:dstPnts[1].y
                             currentHintP3x:dstPnts[2].x
                             currentHintP3y:dstPnts[2].y
                             currentHintP4x:dstPnts[3].x
                             currentHintP4y:dstPnts[3].y
                         currentHintOverlap:0
                          currentHintStatus:0
                     currentImageFrameIndex:frameIndex
                                 homography:list
                                   pixelCor:list
                                    bestPOV:basePOV
                               previewImage:image];
                
                            //        NSLog(@"%@",points);
                //            self.overPointStatus = 0;

                CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
                CFRelease(sampleBuffer);
            }else{
                CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
                CFRelease(sampleBuffer);

                if (mode == 0) {
                    [self didCatchError:-1001];
                }else if (mode == 2){
                    [self didCatchError:-1005];
                }else{
                    [self didCatchError:-1010];
                }
                
            }
    //12-20palnA植入planB改动            bool res = overlap_point(dst, points, srcPnts, dstPnts);// overlap_point(dst, srcPnts, dstPnts);
    //        if(res){
    //            float width = dst.cols;
    //            float height = dst.rows;
    //            NSMutableArray * points = [[NSMutableArray alloc] init];
    //
    //            for (int i = 0; i < 16; i++) {
    //                [points addObject:[NSNumber numberWithFloat:0]];
    //            }
    //            for(int i = 0 ; i < 4; i ++){
    //                points[i*2] = [NSNumber numberWithFloat:(float)srcPnts[i].x/width] ;
    //                points[i*2+1] = [NSNumber numberWithFloat:(float)srcPnts[i].y/height] ;
    //                points[i*2+8] = [NSNumber numberWithFloat:(float)dstPnts[i].x/width] ;
    //                points[i*2+9] = [NSNumber numberWithFloat:(float)dstPnts[i].y/height] ;
    //            }
    //            CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
    //            CFRelease(sampleBuffer);
    //
    //            //        NSLog(@"%@",points);
    //            self.overPointStatus = 0;
    //            return points;
    //        }
//            CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
//            CFRelease(sampleBuffer);

        }
     */
}

@end
