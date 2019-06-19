//
//  PoseManager.h
//  SDKDemo
//
//  Created by KanDao on 2018/11/25.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@interface PoseManager : NSObject

@property (nonatomic, copy) void(^motionUpdate)(double acc);

@property (nonatomic, assign) int currentSlope;
@property (nonatomic, assign) int currentOffset;

-(void)start;
-(void)stop;
-(BOOL)isValidPose;
-(BOOL)isStationary;

- (void)firstShoot;
- (void)removeShoot;
- (float)headingGap;

@end

NS_ASSUME_NONNULL_END
