//
//  PoseManager.m
//  SDKDemo
//
//  Created by KanDao on 2018/11/25.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import "PoseManager.h"
#import <CoreMotion/CoreMotion.h>
#import <CoreLocation/CoreLocation.h>

#import <UIKit/UIKit.h>

#define ORIENTATION_UNKNOWN 1000
#define ACC_QUEUE_SIZE 10
#define MOVE_THRESHOLD 0.02

@interface PoseManager () <CLLocationManagerDelegate>

@property (nonatomic, strong) CMMotionManager* motionManager;

@property (nonatomic, assign) CMAcceleration userAcc;

@property (nonatomic, strong) CLLocationManager *locationManager;
@property (nonatomic, assign) float currentHeading;
@property (nonatomic, assign) float firstHeading;
@end

@implementation PoseManager

-(void)start{
    __weak PoseManager* weakSelf = self;
    self.firstHeading = -361;
    self.locationManager = [[CLLocationManager alloc] init];
    self.locationManager.delegate = self;
    if ([CLLocationManager headingAvailable]) {
        [self.locationManager startUpdatingHeading];
    }
    self.motionManager = [[CMMotionManager alloc] init];
    self.motionManager.deviceMotionUpdateInterval = 1.0/60.0;
    [self.motionManager startDeviceMotionUpdatesToQueue:NSOperationQueue.currentQueue withHandler:^(CMDeviceMotion * _Nullable motion, NSError * _Nullable error) {
        CMAcceleration acc;
        acc.x = motion.gravity.x + motion.userAcceleration.x;
        acc.y = motion.gravity.y + motion.userAcceleration.y;
        acc.z = motion.gravity.z + motion.userAcceleration.z;
        
        weakSelf.currentSlope = [weakSelf slope:motion.gravity];
        weakSelf.currentOffset = [weakSelf offset:[weakSelf screenOrientation:acc]];
        weakSelf.userAcc = motion.userAcceleration;
        
        if(weakSelf.motionUpdate){
            // notify listener the current user acc strength
            double x = motion.userAcceleration.x;
            double y = motion.userAcceleration.y;
            double z = motion.userAcceleration.z;
            weakSelf.motionUpdate(sqrt(x*x + y*y + z*z));
        }
    }];
}

//获得地理和地磁航向数据，从而转动地理刻度表
-(void)locationManager:(CLLocationManager *)manager didUpdateHeading:(CLHeading *)newHeading{
    self.currentHeading = newHeading.magneticHeading;
}

-(void)stop{
    [self.motionManager stopDeviceMotionUpdates];
}

- (void)firstShoot {
    self.firstHeading = self.currentHeading;
}

- (void)removeShoot {
    self.firstHeading = -361;
}

- (float)headingGap {
    if (self.firstHeading < -360) {
        return 0.0f;
    }
    
    float gap = fabsf(self.currentHeading - self.firstHeading);
    if (gap > 180) {
        gap = 360 - gap;
    }
    return gap;
}

//
-(BOOL)isValidPose{
    return self.currentSlope <= 10 && self.currentOffset <= 10;
}

// 获取手机的前后倾斜角度
-(int)slope:(CMAcceleration)gravity{
    float x = gravity.x;
    float y = gravity.y;
    float z = gravity.z;
    double zTheta = atan2(z, sqrt(x*x+y*y)) / M_PI * 180;
    return (int)zTheta * -1;
}

-(int)screenOrientation:(CMAcceleration)accValues{
    int orientation = ORIENTATION_UNKNOWN;
    float x = -accValues.x;
    float y = -accValues.y;
    float z = -accValues.z;
    float magnitude = x * x + y * y;
    if(magnitude * 4 >= z * z){
        float oneEightyOverPi = 57.29577957855f;
        float angle = (float) atan2(-y, x) * oneEightyOverPi;
        orientation = 90 - round(angle);
        while(orientation >= 360){
            orientation -= 360;
        }
        while(orientation < 0){
            orientation += 360;
        }
    }
    return orientation;
}

// 左右倾斜范围为 (-45 , 45)
-(int)offset:(int)orientation{
    int offset = 0;
    if(orientation > 315 && orientation < 360){
        offset = orientation - 360;
    }
    else if(orientation > 0 && orientation < 45){
        offset = orientation;
    }
    else if(orientation > 45 && orientation < 135){
        offset = orientation - 90;
    }
    else if(orientation > 135 && orientation < 225){
        offset = orientation - 180;
    }
    else if(orientation > 225 && orientation < 315){
        offset = orientation - 270;
    }
    return offset;
}

-(BOOL)isStationary{
    return fabs(self.userAcc.x) < MOVE_THRESHOLD && fabs(self.userAcc.y) < MOVE_THRESHOLD && fabs(self.userAcc.z) < MOVE_THRESHOLD;
}


@end
