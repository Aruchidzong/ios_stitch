//
//  SDKPhoto.h
//  SDKDemo
//
//  Created by KanDao on 2018/11/24.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <NYTPhotoViewer/NYTPhotoViewer.h>

NS_ASSUME_NONNULL_BEGIN

@interface SDKPhoto : NSObject<NYTPhoto>

@property (nonatomic) UIImage *image;
@property (nonatomic) NSData *imageData;
@property (nonatomic) UIImage *placeholderImage;
@property (nonatomic) NSAttributedString *attributedCaptionTitle;
@property (nonatomic) NSAttributedString *attributedCaptionSummary;
@property (nonatomic) NSAttributedString *attributedCaptionCredit;

@end

NS_ASSUME_NONNULL_END
