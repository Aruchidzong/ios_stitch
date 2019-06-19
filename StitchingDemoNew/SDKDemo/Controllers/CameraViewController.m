//
//  CamereViewController.m
//  SDKDemo
//
//  Created by KanDao on 2018/11/19.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import "CameraViewController.h"
@import Toast;
#import <AVFoundation/AVFoundation.h>
#import <NYTPhotoViewer/NYTPhotoViewer.h>
#import "SDKPhoto.h"
#import "FileManager.h"
#import "StitchManager.h"
#import "PoseManager.h"
#import "SettingManager.h"
#import <FLEX/FLEX.h>

@interface CameraViewController ()<AVCapturePhotoCaptureDelegate>

@property (nonatomic, weak) IBOutlet UIButton* captureButton;
@property (nonatomic, weak) IBOutlet UIButton* retakeButton;
@property (nonatomic, weak) IBOutlet UIButton* clearButton;

@property (nonatomic, weak) IBOutlet UIScrollView* scrollView;
@property (nonatomic, strong) UIImageView* panoView;
@property (nonatomic, strong) UIImage* panoImage;

@property (nonatomic, strong) UILabel *slope;
@property (nonatomic, strong) UILabel *offset;
@property (nonatomic, strong) UILabel *heading;

@property (nonatomic, assign) int currentIndex;
@property (nonatomic, assign) BOOL isCapturing;
@property (nonatomic, assign) int stitchQuality;

@property (nonatomic, strong) UIActivityIndicatorView * activityIndicator;

@property (nonatomic, strong) PoseManager* poseManager;

@property (nonatomic, strong)       AVCaptureSession            * session;
@property (nonatomic, strong)       AVCaptureDeviceInput        * videoInput;
@property (nonatomic, strong)       AVCapturePhotoOutput        * photoOutput;
@property (nonatomic, strong)       AVCaptureVideoPreviewLayer  * previewLayer;

@end

@implementation CameraViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
    [self customizeView];
    [[FLEXManager sharedManager] showExplorer];

    [self moreTipView];
    self.currentIndex = 0;
    self.stitchQuality = SettingManager.sharedInstance.quality;
    [StitchManager.sharedInstance start:self.stitchQuality];
//    __weak CameraViewController* weakSelf = self;
    self.poseManager = [[PoseManager alloc] init];
//    self.poseManager.motionUpdate = ^(double acc) {
//        if(!weakSelf.isCapturing){
//            weakSelf.captureButton.enabled = acc < 0.03;
//        }
//    };
    
    self.activityIndicator = [[UIActivityIndicatorView alloc] initWithActivityIndicatorStyle:(UIActivityIndicatorViewStyleGray)];
    [self.view addSubview:self.activityIndicator];
    self.activityIndicator.frame= CGRectMake(100, 100, 100, 100);
    self.activityIndicator.center = self.view.center;
    //设置小菊花颜色
    self.activityIndicator.color = [UIColor grayColor];
    //设置背景颜色
    self.activityIndicator.backgroundColor = [UIColor clearColor];
    //刚进入这个界面会显示控件，并且停止旋转也会显示，只是没有在转动而已，没有设置或者设置为YES的时候，刚进入页面不会显示
    self.activityIndicator.hidesWhenStopped = NO;
    self.activityIndicator.hidden = YES;
}

-(void)viewWillAppear:(BOOL)animated{
    [super viewWillAppear:animated];
    self.navigationController.navigationBarHidden = YES;
    [self initialSession];
    [self.poseManager start];
    if(self.stitchQuality != SettingManager.sharedInstance.quality){
        self.stitchQuality = SettingManager.sharedInstance.quality;
        [self clearClicked:nil];
    }
}

-(void)viewDidAppear:(BOOL)animated{
    [super viewDidAppear:animated];
    [self setUpCameraLayer];
    [self.session startRunning];
}

-(void)viewWillDisappear:(BOOL)animated{
    [self.session stopRunning];
    [self.poseManager stop];
    [super viewWillDisappear:animated];
}

-(void)dealloc{
    [StitchManager.sharedInstance stop];
}

-(BOOL)prefersStatusBarHidden{
    return YES;
}

-(void)customizeView{
    self.captureButton.layer.cornerRadius = 27.0f;
    self.clearButton.layer.cornerRadius = 4.0f;
    self.retakeButton.layer.cornerRadius = 4.0f;
}

- (void)moreTipView {
    self.slope = [[UILabel alloc] initWithFrame:CGRectMake(self.view.bounds.size.width-112, 20, 100, 20)];
    self.slope.textColor = [UIColor blackColor];
    self.slope.textAlignment = NSTextAlignmentRight;
    self.slope.shadowColor = [UIColor whiteColor];
    self.slope.shadowOffset = CGSizeMake(2, 1);
    [self.view addSubview:self.slope];
    self.offset = [[UILabel alloc] initWithFrame:CGRectMake(self.view.bounds.size.width-112, 40, 100, 20)];
    self.offset.shadowColor = [UIColor whiteColor];
    self.offset.textAlignment = NSTextAlignmentRight;
    self.offset.shadowOffset = CGSizeMake(2, 1);
    self.offset.textColor = [UIColor blackColor];
    [self.view addSubview:self.offset];
    self.heading = [[UILabel alloc] initWithFrame:CGRectMake(self.view.bounds.size.width-112, 50, 100, 20)];
    self.heading.shadowColor = [UIColor whiteColor];
    self.heading.textAlignment = NSTextAlignmentRight;
//    self.heading.shadowOffset = CGSizeMake(2, 1);
//    self.heading.textColor = [UIColor blackColor];
//    [self.view addSubview:self.heading];
//    UILabel *slope = [[UILabel alloc] initWithFrame:CGRectMake(300, 300, 90, 50)];
//    slope.textColor = [UIColor blackColor];
//    [self.view addSubview:slope];
    // 创建CADisplayLink
    CADisplayLink *disLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(linkMethod:)];
    // 添加至RunLoop中
    [disLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSRunLoopCommonModes];
}

- (void)linkMethod:(CADisplayLink *)sender {
    self.slope.text = [NSString stringWithFormat:@"(%@)", @(self.poseManager.currentSlope)];
    self.offset.text = [NSString stringWithFormat:@"(%@)", @(self.poseManager.currentOffset)];
    self.heading.text = [NSString stringWithFormat:@"(%.2f)", self.poseManager.headingGap];
}

-(UIImageView*)panoView{
    if(_panoView == nil){
        _panoView = [[UIImageView alloc] initWithFrame:CGRectZero];
        _panoView.userInteractionEnabled = YES;
        [_panoView addGestureRecognizer:[[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(showPanoImage)]];
        [self.scrollView addSubview:_panoView];
    }
    return _panoView;
}

-(void)setPanoImage:(UIImage *)panoImage{
    _panoImage = panoImage;
    
    if(panoImage == nil){
        self.panoView.hidden = YES;
    }
    else{
        CGFloat height = self.scrollView.frame.size.height;
        CGFloat width = panoImage.size.width/panoImage.size.height*height;
        self.panoView.frame = CGRectMake(0, 0, width, height);
        self.panoView.image = panoImage;
        self.panoView.hidden = NO;
        self.scrollView.contentSize = self.panoView.frame.size;
        self.scrollView.contentOffset = CGPointZero;
    }
}
//读取Log路径
- (NSString*)getLogPath:(int)index{
    
    return [FileManager.sharedInstance pathForLogWithFolder:@"pano" itemId:[NSString stringWithFormat:@"%d",index]];
}


-(IBAction)retakeClicked:(id)sender{
    if(self.currentIndex == 0){
        // no picture taken, do nothing
        return;
    }
    
    __weak CameraViewController* weakSelf = self;
    [StitchManager.sharedInstance stitch:nil outputPath:nil logPath:[self getLogPath:-1] retake:YES completionHandler:^(NSError * _Nonnull error) {
        dispatch_async(dispatch_get_main_queue(), ^{
            if(error){
                self.captureButton.enabled = YES;
                self.activityIndicator.hidden = YES;
                [self.activityIndicator stopAnimating];
                self.isCapturing = NO;
                [weakSelf showError:error.localizedDescription];
                return;
            }
            
            self.currentIndex --;
            
            // change pano image to the previous image
            if(self.currentIndex == 0){
                self.panoImage = nil;
                return;
            }
            
            self.panoImage = [FileManager.sharedInstance panoImageAtIndex:self.currentIndex-1];
        });
    }];
}

-(IBAction)clearClicked:(id)sender{
    // remove existing image and restart stitcher
    self.panoImage = nil;
    
    self.currentIndex = 0;
    [self.poseManager removeShoot];
    [StitchManager.sharedInstance restart:self.stitchQuality];
}

-(void)showPanoImage{
    [self showImage:self.panoImage];
}

-(void)showImage:(UIImage*)image{
    SDKPhoto* photo = [SDKPhoto new];
    photo.image = image;
    NYTPhotoViewerSinglePhotoDataSource* dataSource = [NYTPhotoViewerSinglePhotoDataSource dataSourceWithPhoto:photo];
    NYTPhotosViewController* viewController = [[NYTPhotosViewController alloc] initWithDataSource:dataSource];
    viewController.rightBarButtonItem = nil;
    [self presentViewController:viewController animated:YES completion:nil];
}

- (void) initialSession{
    __weak CameraViewController* weakSelf = self;
    [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted) {
        if(!granted){
            [weakSelf showError:@"Please enable camera."];
            return;
        }
    }];
    AVCaptureDevice* device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    
    self.session = [[AVCaptureSession alloc] init];
    [self.session beginConfiguration];
    
    self.session.sessionPreset = AVCaptureSessionPresetHigh;
    self.videoInput = [[AVCaptureDeviceInput alloc] initWithDevice:device error:nil];
    if ([self.session canAddInput:self.videoInput]) {
        [self.session addInput:self.videoInput];
    }
    
    self.photoOutput = [[AVCapturePhotoOutput alloc] init];
    if([self.session canAddOutput:self.photoOutput]){
        [self.session addOutput:self.photoOutput];
    }
    [self.session commitConfiguration];
}

- (void) setUpCameraLayer{
    if(self.previewLayer != nil){
        [self.previewLayer removeFromSuperlayer];
    }
    
    self.previewLayer = [[AVCaptureVideoPreviewLayer alloc] initWithSession:self.session];
    UIView * view = self.view;
    CALayer * viewLayer = [view layer];
    [viewLayer setMasksToBounds:YES];
    
    [self.previewLayer connection].videoOrientation = AVCaptureVideoOrientationPortrait;// [self videoOrientation];
    self.previewLayer.backgroundColor = [[UIColor blackColor] CGColor];
    
    CGRect bounds = [view bounds];
    [self.previewLayer setFrame:bounds];
    
    [viewLayer insertSublayer:self.previewLayer below:[[viewLayer sublayers] objectAtIndex:0]];
}

-(IBAction)shot:(id)sender{
    if(self.currentIndex == 0){
        [self.poseManager firstShoot];
    }
//    int maxSlope = 20;
//    int maxOffset = 20;
//    if(SettingManager.sharedInstance.checkPose){
//        if (self.poseManager.currentSlope > maxSlope) {
//            [self showError:[NSString stringWithFormat:@"前后倾斜角度不允许超过%@度哦", @(maxSlope)]];
//            return;
//        } else if (self.poseManager.currentOffset > maxOffset) {
//            [self showError:[NSString stringWithFormat:@"左右倾斜角度不允许超过%@度哦", @(maxOffset)]];
//            return;
//        }
//    }
    // start photo capture
    self.isCapturing = YES;
    self.captureButton.enabled = NO;
    self.activityIndicator.hidden = NO;
    [self.activityIndicator startAnimating];
    AVCapturePhotoSettings* photoSettings = AVCapturePhotoSettings.photoSettings;
    [self.photoOutput capturePhotoWithSettings:photoSettings delegate:self];
}


- (void)captureOutput:(AVCapturePhotoOutput *)output didFinishProcessingPhotoSampleBuffer:(CMSampleBufferRef)photoSampleBuffer previewPhotoSampleBuffer:(CMSampleBufferRef)previewPhotoSampleBuffer resolvedSettings:(AVCaptureResolvedPhotoSettings *)resolvedSettings bracketSettings:(AVCaptureBracketedStillImageSettings *)bracketSettings error:(NSError *)error
{
    if (photoSampleBuffer) {
        NSData *data = [AVCaptureStillImageOutput jpegStillImageNSDataRepresentation:photoSampleBuffer];
        int index = self.currentIndex;
        
        // save image
        [FileManager.sharedInstance saveImageData:data atIndex:index];
        
        NSString* imagePath = [FileManager.sharedInstance pathForImageAtIndex:index];
        NSString* outputPath = [FileManager.sharedInstance pathForPanoImageAtIndex:index];
        
        // stitch image
        __weak CameraViewController* weakSelf = self;
        [StitchManager.sharedInstance stitch:imagePath outputPath:outputPath logPath:[self getLogPath:index] retake:NO completionHandler:^(NSError * _Nonnull error) {
            if(error != nil){
                dispatch_async(dispatch_get_main_queue(), ^{
                    self.captureButton.enabled = YES;
                    self.activityIndicator.hidden = YES;
                    [self.activityIndicator stopAnimating];
                    self.isCapturing = NO;
                    [weakSelf showError:error.localizedDescription];
                });
                return;
            }
            
            self.currentIndex ++;
            dispatch_async(dispatch_get_main_queue(), ^{
                self.panoImage = [UIImage imageWithContentsOfFile:outputPath];
                self.captureButton.enabled = YES;
                self.activityIndicator.hidden = YES;
                [self.activityIndicator stopAnimating];
                self.isCapturing = NO;
            });
            NSLog(@"currentIndex = %d", self.currentIndex);
        }];

    }
}
-(void)captureOutput:(AVCapturePhotoOutput *)output didFinishProcessingPhoto:(AVCapturePhoto *)photo error:(NSError *)error{
    int index = self.currentIndex;

    // save image
    [FileManager.sharedInstance saveImageData:photo.fileDataRepresentation atIndex:index];
    
    NSString* imagePath = [FileManager.sharedInstance pathForImageAtIndex:index];
    NSString* outputPath = [FileManager.sharedInstance pathForPanoImageAtIndex:index];
    
    // stitch image
    __weak CameraViewController* weakSelf = self;
    [StitchManager.sharedInstance stitch:imagePath outputPath:outputPath logPath:[self getLogPath:index] retake:NO completionHandler:^(NSError * _Nonnull error) {
        if(error != nil){
            dispatch_async(dispatch_get_main_queue(), ^{
                self.captureButton.enabled = YES;
                self.activityIndicator.hidden = YES;
                [self.activityIndicator stopAnimating];
                self.isCapturing = NO;
                [weakSelf showError:error.localizedDescription];
            });
            return;
        }
        
        self.currentIndex ++;
        dispatch_async(dispatch_get_main_queue(), ^{
            self.panoImage = [UIImage imageWithContentsOfFile:outputPath];
            self.captureButton.enabled = YES;
            self.activityIndicator.hidden = YES;
            [self.activityIndicator stopAnimating];
            self.isCapturing = NO;
        });
        NSLog(@"currentIndex = %d", self.currentIndex);
    }];
}

-(void)showError:(NSString*)message{
    [self.view makeToast:message
                duration:3
                position:CSToastPositionCenter];
}

@end
