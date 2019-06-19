//
//  SettingManager.h
//  SDKDemo
//
//  Created by KanDao on 2018/11/27.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN
#define SETTING_CHECK_POSE @"checkPose"
#define SETTING_QUALITY @"quality"
#define SETTING_IMG @"img"
#define SETTING_WARP @"warp"
#define SETTING_EDGE @"edge"

@interface SettingManager : NSObject

@property (nonatomic, assign) BOOL checkPose;
@property (nonatomic, assign) int quality;
@property (nonatomic, readonly) NSString* qualityText;
@property (nonatomic, readonly) NSArray* qualityOptions;

+ (instancetype)sharedInstance;
-(NSArray*)textsForKey:(NSString*)key;
-(NSArray*)valuesForKey:(NSString*)key;
-(int)valueForKey:(NSString*)key;
-(NSString*)textForKey:(NSString*)key;
-(void)setValue:(int)value forKey:(NSString*)key;

@end

NS_ASSUME_NONNULL_END
