//
//  SettingManager.m
//  SDKDemo
//
//  Created by KanDao on 2018/11/27.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import "SettingManager.h"


@implementation SettingManager

+ (instancetype)sharedInstance
{
    static dispatch_once_t p = 0;
    __strong static id _sharedObject = nil;
    
    dispatch_once(&p, ^{
        _sharedObject = [[self alloc] init];
    });
    
    return _sharedObject;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        [self initCheckPose];
    }
    return self;
}

- (void)initCheckPose{
    if (![NSUserDefaults.standardUserDefaults objectIsForcedForKey:SETTING_CHECK_POSE]) {
        [self setCheckPose:YES];
    }
}

-(BOOL)checkPose{
    return [NSUserDefaults.standardUserDefaults boolForKey:SETTING_CHECK_POSE];
}

-(void)setCheckPose:(BOOL)checkPose{
    [NSUserDefaults.standardUserDefaults setBool:checkPose forKey:SETTING_CHECK_POSE];
}

-(int)quality{
    return (int)[NSUserDefaults.standardUserDefaults integerForKey:SETTING_QUALITY];
}

-(void)setQuality:(int)quality{
    [NSUserDefaults.standardUserDefaults setInteger:quality forKey:SETTING_QUALITY];
}

-(NSString*)qualityText{
    NSArray* options = self.qualityOptions;
    int quality = self.quality;
    if(quality < 0 || quality >= options.count){
        return options[0];
    }
    return options[quality];
}

-(NSArray*)qualityOptions{
    return [NSArray arrayWithObjects:@"速度最快", @"优先速度", @"优先质量", @" 质量最高", nil];
}

-(NSArray*)textsForKey:(NSString*)key{
    if([key isEqualToString:SETTING_IMG]){
        return [NSArray arrayWithObjects:@"0", @"1", @"2", @"3", nil];
    }
    else if([key isEqualToString:SETTING_WARP]){
        return [NSArray arrayWithObjects:@"0", @"1", nil];
    }
    else if([key isEqualToString:SETTING_EDGE]){
        return [NSArray arrayWithObjects:@"1000", @"1500", @"2000", nil];
    }
    return nil;
}

-(NSArray*)valuesForKey:(NSString*)key{
    if([key isEqualToString:SETTING_IMG]){
        return [NSArray arrayWithObjects:@(0), @(1), @(2), @(3), nil];
    }
    else if([key isEqualToString:SETTING_WARP]){
        return [NSArray arrayWithObjects:@(0), @(1), nil];
    }
    else if([key isEqualToString:SETTING_EDGE]){
        return [NSArray arrayWithObjects:@(1000), @(1500), @(2000), nil];
    }
    return nil;
}

-(int)defaultValueForKey:(NSString*)key{
    if([key isEqualToString:SETTING_IMG]){
        return 0;
    }
    if([key isEqualToString:SETTING_WARP]){
        return 1;
    }
    if([key isEqualToString:SETTING_EDGE]){
        return 2000;
    }
    return 0;
}

-(int)valueForKey:(NSString*)key{
    
    if([NSUserDefaults.standardUserDefaults objectForKey:key] == nil){
        return [self defaultValueForKey:key];
    }
    return (int)[NSUserDefaults.standardUserDefaults integerForKey:key];
}

-(void)setValue:(int)value forKey:(NSString*)key{
    [NSUserDefaults.standardUserDefaults setInteger:value forKey:key];
}

-(NSString*)textForKey:(NSString*)key{
    NSArray* texts = [self textsForKey:key];
    NSArray* values = [self valuesForKey:key];
    int value = [self valueForKey:key];
    for(int i = 0 ; i < values.count; i ++){
        if([values[i] intValue] == value){
            return texts[i];
        }
    }
    return nil;
}

@end
