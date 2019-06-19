//
//  StitchManager.h
//  SDKDemo
//
//  Created by KanDao on 2018/11/25.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

NS_ASSUME_NONNULL_BEGIN

@interface StitchManager : NSObject

+ (instancetype)sharedInstance;

-(void)start:(int)quality warpType:(int)warpType maxEdge:(int)maxEdge;
-(void)restart:(int)quality warpType:(int)warpType maxEdge:(int)maxEdge;
-(void)stitch:(nullable NSString*)imagePath outputPath:(nullable NSString*)outputPath retake:(BOOL)retake completionHandler:(void(^)(NSError*))handler;
-(void)stop;

-(BOOL)setOverlapSrc:(nullable NSString*)imagePath;
-(BOOL)calcOverlap:(uint8_t*)imageData width:(int)width height:(int)height points:(float*)points;

-(void)saveImage:(uint8_t*)data width:(int)width height:(int)height;

@end

NS_ASSUME_NONNULL_END
