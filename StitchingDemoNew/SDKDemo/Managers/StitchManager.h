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

-(void)start:(int)quality;
-(void)restart:(int)quality;
-(void)stitch:(nullable NSString*)imagePath outputPath:(nullable NSString*)outputPath logPath:(NSString*)logPath retake:(BOOL)retake completionHandler:(void(^)(NSError*))handler;
-(void)stop;

@end

NS_ASSUME_NONNULL_END
