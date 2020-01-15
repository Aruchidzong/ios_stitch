//
//  FileManager.h
//  SDKDemo
//
//  Created by KanDao on 2018/11/24.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

NS_ASSUME_NONNULL_BEGIN

@interface FileManager : NSObject

+ (instancetype)sharedInstance;

-(BOOL)saveImage:(UIImage*)image atIndex:(int)index;
-(BOOL)saveImage:(UIImage*)image atItemId:(NSString*)_id atIndex:(int)index;
-(BOOL)saveImageData:(NSData*)data atIndex:(int)index;
-(BOOL)savePanoImage:(UIImage*)image atIndex:(int)index;
-(UIImage*)imageAtIndex:(int)index;
-(UIImage*)panoImageAtIndex:(int)index;
-(NSString*)pathForLogAtIndex:(int)index;

-(NSString*)pathForLogWithFolder:(NSString*)folder itemId:(NSString*)_id;
-(NSString*)pathForImageAtIndex:(int)index;
-(NSString*)pathForPanoImageAtIndex:(int)index;
-(NSString*)pathForImageAtItemId:(NSString*)_id atIndex:(int)index;
-(NSString*)pathForPanoImageAtItemId:(NSString*)_id atIndex:(int)index;

@end

NS_ASSUME_NONNULL_END
