//
//  UIImage+Ext.m
//  SDKDemo
//
//  Created by luck chen on 2020/1/14.
//  Copyright © 2020 KandaoVR. All rights reserved.
//

#import "UIImage+Ext.h"


@implementation UIImage (Ext)

- (UIImage*)scaleTo:(CGFloat)maxLength{
    CGSize realSize = CGSizeMake(self.size.width*self.scale, self.size.height*self.scale);

    CGSize targetSize ;
    if (realSize.width < maxLength && realSize.height < maxLength) {
        targetSize = self.size;
    } else{
        if (realSize.width > realSize.height) {
            targetSize = CGSizeMake((int)maxLength, (int)(realSize.height / (realSize.width / maxLength)));
        } else{
            targetSize = CGSizeMake((int)(realSize.width / (realSize.height / maxLength)), (int)maxLength);
        }
    }
    
     // 设置成为当前正在使用的context

    UIGraphicsBeginImageContext(targetSize);

    // 绘制改变大小的图片

    [self drawInRect:CGRectMake(0, 0, targetSize.width, targetSize.height)];

    // 从当前context中创建一个改变大小后的图片

    UIImage* scaledImage = UIGraphicsGetImageFromCurrentImageContext();

    // 使当前的context出堆栈
    UIGraphicsEndImageContext();
    
    return scaledImage;

}

@end
