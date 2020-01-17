//
//  CamereViewController.m
//  SDKDemo
//
//  Created by KanDao on 2018/11/19.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import "CameraViewController_video.h"
//@import Toast;
#import <AVFoundation/AVFoundation.h>
#import <NYTPhotoViewer/NYTPhotoViewer.h>
#import "SDKPhoto.h"
#import "FileManager.h"
#import "StitchManager_video.h"
#import "PoseManager.h"
#import "SettingManager.h"
#import <CoreServices/CoreServices.h>
#import "UIImage+Ext.h"
#import "math.h"

@interface CameraViewController_video ()<AVCapturePhotoCaptureDelegate, AVCaptureVideoDataOutputSampleBufferDelegate>{
    dispatch_queue_t _dataQueue;
    dispatch_queue_t _overlapQueue;
    float _points[16];
}

@property (nonatomic, weak) IBOutlet UIButton* captureButton;
@property (nonatomic, weak) IBOutlet UIButton* retakeButton;
@property (nonatomic, weak) IBOutlet UIButton* clearButton;

//@property (nonatomic, weak) IBOutlet UIImageView* overlayView;
@property (nonatomic, weak) IBOutlet UIImageView* srcImageView;
@property (nonatomic, strong) UIImageView* panoView;
@property (nonatomic, strong) UIImage* panoImage;

@property (nonatomic, strong) CAShapeLayer* prevOverlayLayer;
@property (nonatomic, strong) CAShapeLayer* currOverlayLayer;

@property (nonatomic, strong) UILabel *slope;
@property (nonatomic, strong) UILabel *offset;
@property (nonatomic, strong) UILabel *heading;

@property (nonatomic, assign) int currentIndex;
@property (nonatomic, assign) BOOL isCapturing;
@property (nonatomic, assign) int stitchQuality;
@property (nonatomic, assign) int stitchWarpType;
@property (nonatomic, assign) int stitchMaxEdge;
@property (weak, nonatomic) IBOutlet UILabel *tostLabel;

@property (nonatomic, strong) UIActivityIndicatorView * activityIndicator;

@property (nonatomic, strong) PoseManager* poseManager;

@property (nonatomic, strong)       AVCaptureSession            * session;
@property (nonatomic, strong)       AVCaptureDeviceInput        * videoInput;
@property (nonatomic, strong)       AVCapturePhotoOutput        * photoOutput;
@property (nonatomic, strong)       AVCaptureVideoPreviewLayer  * previewLayer;
@property (nonatomic, strong)       AVCaptureVideoDataOutput    * dataOutput;

@property (atomic, assign) BOOL isCalcOverlap;
@property (atomic, assign) BOOL shouldCalcOverlap;

@end

@implementation CameraViewController_video

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
    [self customizeView];
    
    [self moreTipView];
    self.currentIndex = 0;
    self.stitchQuality = [SettingManager.sharedInstance valueForKey:SETTING_IMG];
    self.stitchWarpType = [SettingManager.sharedInstance valueForKey:SETTING_WARP];
    self.stitchMaxEdge = [SettingManager.sharedInstance valueForKey:SETTING_EDGE];
    NSString* logPath = [FileManager.sharedInstance pathForLogAtIndex:123];

    [StitchManager.sharedInstance resetOfLogPath:logPath completionHandler:^(bool success, NSString * _Nonnull msg) {
        
    }];
//    [StitchManager.sharedInstance start:self.stitchQuality warpType:self.stitchWarpType maxEdge:self.stitchMaxEdge];
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
    
//    UISwipeGestureRecognizer* swipeLeftRecognizer = [[UISwipeGestureRecognizer alloc] initWithTarget:self action:@selector(swipeHandler:)];
//    swipeLeftRecognizer.direction = UISwipeGestureRecognizerDirectionLeft;
//    [self.view addGestureRecognizer:swipeLeftRecognizer];
//
//    UISwipeGestureRecognizer* swipeDownRecognizer = [[UISwipeGestureRecognizer alloc] initWithTarget:self action:@selector(swipeHandler:)];
//    swipeDownRecognizer.direction = UISwipeGestureRecognizerDirectionDown;
//    [self.view addGestureRecognizer:swipeDownRecognizer];
//
//    UISwipeGestureRecognizer* swipeRightRecognizer = [[UISwipeGestureRecognizer alloc] initWithTarget:self action:@selector(swipeHandler:)];
//    swipeRightRecognizer.direction = UISwipeGestureRecognizerDirectionRight;
//    [self.view addGestureRecognizer:swipeRightRecognizer];
//
//    UISwipeGestureRecognizer* swipeUpRecognizer = [[UISwipeGestureRecognizer alloc] initWithTarget:self action:@selector(swipeHandler:)];
//    swipeUpRecognizer.direction = UISwipeGestureRecognizerDirectionUp;
//    [self.view addGestureRecognizer:swipeUpRecognizer];

//    self.overlayView.hidden = YES;
//    self.overlayView.opaque = NO;
//    self.overlayView.alpha = 0.5f;
    
    self.isCalcOverlap = false;
    self.shouldCalcOverlap = false;
    
    NSTimer *timer = [NSTimer scheduledTimerWithTimeInterval:3 target:self selector:@selector(restart) userInfo:nil repeats:true];
    [[NSRunLoop mainRunLoop] addTimer:timer forMode:NSRunLoopCommonModes];
}

- (void)restart{
    dispatch_async(dispatch_get_main_queue(), ^{
    });
}

-(void)viewWillAppear:(BOOL)animated{
    [super viewWillAppear:animated];
    self.navigationController.navigationBarHidden = YES;
    [self initialSession];
    [self.poseManager start];
    if(self.stitchQuality != [SettingManager.sharedInstance valueForKey:SETTING_IMG]
       || self.stitchWarpType != [SettingManager.sharedInstance valueForKey:SETTING_WARP]
       || self.stitchMaxEdge != [SettingManager.sharedInstance valueForKey:SETTING_EDGE]){
        self.stitchQuality = [SettingManager.sharedInstance valueForKey:SETTING_IMG];
        self.stitchWarpType = [SettingManager.sharedInstance valueForKey:SETTING_WARP];
        self.stitchMaxEdge = [SettingManager.sharedInstance valueForKey:SETTING_EDGE];
        [self clearClicked:nil];
    }
}

-(void)viewDidAppear:(BOOL)animated{
    [super viewDidAppear:animated];
    [self setUpCameraLayer];
    [self setupPreview];
    [self.session startRunning];
}

-(void)viewWillDisappear:(BOOL)animated{
    [self.session stopRunning];
    [self.poseManager stop];
    [super viewWillDisappear:animated];
}

-(void)swipeHandler:(UISwipeGestureRecognizer*)recognizer{
    if(self.currentIndex == 0){
        return;
    }
    
    UIImage* image = [FileManager.sharedInstance imageAtIndex:self.currentIndex-1];
    CGSize imageViewSize = CGSizeMake(image.size.width/image.size.height*self.view.frame.size.height, self.view.frame.size.height);
//    self.overlayView.image = image;
//    self.overlayView.hidden = NO;
//    if(recognizer.direction == UISwipeGestureRecognizerDirectionLeft){
//        self.overlayView.frame = CGRectMake(-imageViewSize.width*2/3, 0, imageViewSize.width, imageViewSize.height);
//    }
//    else if(recognizer.direction == UISwipeGestureRecognizerDirectionRight){
//        self.overlayView.frame = CGRectMake(imageViewSize.width*2/3, 0, imageViewSize.width, imageViewSize.height);
//    }
//    else if(recognizer.direction == UISwipeGestureRecognizerDirectionUp){
//        self.overlayView.frame = CGRectMake(0, -imageViewSize.height*2/3, imageViewSize.width, imageViewSize.height);
//    }
//    else{
//        self.overlayView.frame = CGRectMake(0, imageViewSize.height*2/3, imageViewSize.width, imageViewSize.height);
//    }
}

-(void)dealloc{
    [StitchManager.sharedInstance stitchClean];
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
    self.slope = [[UILabel alloc] initWithFrame:CGRectMake(self.view.bounds.size.width-112, 60, 100, 20)];
    self.slope.textColor = [UIColor blackColor];
    self.slope.textAlignment = NSTextAlignmentRight;
    self.slope.shadowColor = [UIColor whiteColor];
    self.slope.shadowOffset = CGSizeMake(2, 1);
    [self.view addSubview:self.slope];
    self.offset = [[UILabel alloc] initWithFrame:CGRectMake(self.view.bounds.size.width-112, 80, 100, 20)];
    self.offset.shadowColor = [UIColor whiteColor];
    self.offset.textAlignment = NSTextAlignmentRight;
    self.offset.shadowOffset = CGSizeMake(2, 1);
    self.offset.textColor = [UIColor blackColor];
    [self.view addSubview:self.offset];
    self.heading = [[UILabel alloc] initWithFrame:CGRectMake(self.view.bounds.size.width-112, 90, 100, 20)];
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
        [self.view addSubview:_panoView];
    }
    return _panoView;
}

-(void)setPanoImage:(UIImage *)panoImage{
    _panoImage = panoImage;
    
    if(panoImage == nil){
        self.panoView.hidden = YES;
    }
    else{
        self.panoView.frame = CGRectMake(20, 100, 200, 150);
        self.panoView.image = panoImage;
        self.panoView.hidden = NO;
    }
}

-(IBAction)retakeClicked:(id)sender{
    if(self.currentIndex == 0 || self.currentIndex == 1){
        // no picture taken, do nothing
        [self clearClicked:nil];
        self.currentIndex = 0;
        return;
    }
    self.captureButton.enabled = NO;
    self.activityIndicator.hidden = false;
    [self.activityIndicator startAnimating];

    [StitchManager.sharedInstance delLastOfCompletionHandler:^(bool success, NSString * _Nonnull msg) {
        dispatch_async(dispatch_get_main_queue(), ^{
            if (success) {
                self.currentIndex --;
                self.panoImage = [FileManager.sharedInstance panoImageAtIndex:self.currentIndex-1];
                self.panoView.image = self.panoImage ;

            }
            self.captureButton.enabled = YES;
            self.activityIndicator.hidden = YES;
            [self.activityIndicator stopAnimating];
        });
    }];
    /*
    __weak CameraViewController_video* weakSelf = self;
    [StitchManager.sharedInstance stitch:nil outputPath:nil retake:YES completionHandler:^(NSError * _Nonnull error) {
        dispatch_async(dispatch_get_main_queue(), ^{
            if(error){
                self.isCapturing = NO;
                [weakSelf showError:error.localizedDescription];
                return;
            }
            
            self.currentIndex --;
            
            // change pano image to the previous image
            [self setupPreview];
            if(self.currentIndex == 0){
                self.panoImage = nil;
                return;
            }
            
            self.panoImage = [FileManager.sharedInstance panoImageAtIndex:self.currentIndex-1];
            self.overlayView.hidden = YES;
        });
    }];
     */
}

-(IBAction)clearClicked:(id)sender{
    // remove existing image and restart stitcher
    self.panoImage = nil;
//    self.overlayView.hidden = YES;
    self.shouldCalcOverlap = NO;
    
    self.currentIndex = 0;
    [self.poseManager removeShoot];
    [StitchManager.sharedInstance stitchClean];
//    [StitchManager.sharedInstance restart:self.stitchQuality warpType:self.stitchWarpType maxEdge:self.stitchMaxEdge];
    [self setupPreview];
    dispatch_async(dispatch_get_main_queue(), ^{

        self.prevOverlayLayer.path = nil;
        self.prevOverlayLayer.hidden = true;
    });

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
    __weak CameraViewController_video* weakSelf = self;
    [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted) {
        if(!granted){
            [weakSelf showError:@"Please enable camera."];
            return;
        }
    }];
    AVCaptureDevice* device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];

    self.session = [[AVCaptureSession alloc] init];
    [self.session beginConfiguration];
    
    self.session.sessionPreset = AVCaptureSessionPreset640x480;
    self.videoInput = [[AVCaptureDeviceInput alloc] initWithDevice:device error:nil];
    if ([self.session canAddInput:self.videoInput]) {
        [self.session addInput:self.videoInput];
    }
    
    self.photoOutput = [[AVCapturePhotoOutput alloc] init];
    if([self.session canAddOutput:self.photoOutput]){
        [self.session addOutput:self.photoOutput];
    }
    self.photoOutput.highResolutionCaptureEnabled = true;
    
    self.dataOutput = [[AVCaptureVideoDataOutput alloc] init];
    self.dataOutput.videoSettings = @{(id)kCVPixelBufferPixelFormatTypeKey: @(kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange)};
    NSError *error;
    [device lockForConfiguration:&error];
//    [self.session setSessionPreset:AVCaptureSessionPresetPhoto];
    self.session.sessionPreset = AVCaptureSessionPreset640x480;
    device.activeVideoMaxFrameDuration = CMTimeMake(1, 30);
    device.activeVideoMinFrameDuration = CMTimeMake(1, 30);
    [device unlockForConfiguration];

    _dataQueue = dispatch_queue_create("DataQueue", NULL);
    [self.dataOutput setSampleBufferDelegate:self queue:_dataQueue];
    if([self.session canAddOutput:self.dataOutput]){
        [self.session addOutput:self.dataOutput];
    }
            // 获取输入与输出之间的连接
    AVCaptureConnection *connection = [self.dataOutput connectionWithMediaType: AVMediaTypeVideo];
    connection.videoScaleAndCropFactor = connection.videoMaxScaleAndCropFactor;
    //         [videoOutput connectionWithMediaType:AVMediaTypeVideo];
    //         设置采集数据的方向
    connection.videoOrientation = AVCaptureVideoOrientationPortrait;

    
    _overlapQueue = dispatch_queue_create("OverlapQueue", NULL);

    [self.session commitConfiguration];
}

- (void)captureOutput:(AVCaptureOutput *)output didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection{
    
    if(self.isCapturing){
        // don't need to calculate overlap or it is already calculating
        return;
    }
    UIImage *img = [StitchManager.sharedInstance imageConvert:sampleBuffer scale:1.0 orientation:UIImageOrientationUp];
    [StitchManager.sharedInstance overlapOfImage:img frameIndex:0 angle_pitch:0 angle_roll:0 angle_yaw:0 completionHandler:^(bool success, NSString * _Nonnull msg, NSArray<NSNumber *> * _Nonnull overPoint, float currentHintLinesWidth, float currentHintLinesHeight, float score) {
        if (success && overPoint.count >= 8) {
            
            dispatch_async(dispatch_get_main_queue(), ^{
                CGFloat scale_x = self.previewLayer.frame.size.width / currentHintLinesWidth;
                CGFloat scale_y = self.previewLayer.frame.size.height / currentHintLinesHeight;
                UIBezierPath* path = [[UIBezierPath alloc] init];
                [path moveToPoint:CGPointMake(scale_x * overPoint[0].floatValue, scale_y * overPoint[1].floatValue)];
                [path addLineToPoint:CGPointMake(scale_x *overPoint[2].floatValue, scale_y * overPoint[3].floatValue)];
                [path addLineToPoint:CGPointMake(scale_x *overPoint[4].floatValue, scale_y * overPoint[5].floatValue)];
                [path addLineToPoint:CGPointMake(scale_x *overPoint[6].floatValue, scale_y * overPoint[7].floatValue)];
                [path closePath];
                if (score > 1.3) {
                    self.prevOverlayLayer.path = nil;
                    self.prevOverlayLayer.hidden = true;
                    [self.captureButton setEnabled:false];
                }else if (score > 1.0) {
                    self.prevOverlayLayer.path = path.CGPath;
                    self.prevOverlayLayer.hidden = false;
                    [self.captureButton setEnabled:false];

                    self.prevOverlayLayer.fillColor = UIColor.redColor.CGColor;
                    self.prevOverlayLayer.fillColor = [UIColor colorWithRed:255/255.0 green:0/255.0 blue:0/255.0 alpha:(score - 0.5)/1.0*0.9].CGColor;

                }else{
                    self.prevOverlayLayer.path = path.CGPath;
                    self.prevOverlayLayer.hidden = false;
                    [self.captureButton setEnabled:true];

                    self.prevOverlayLayer.fillColor = [UIColor colorWithRed:61/255.0 green:251/255.0 blue:51/255.0 alpha:MAX(0.4,score/1.0*0.9)].CGColor;
                }

            });
        }else{
            dispatch_async(dispatch_get_main_queue(), ^{

                self.prevOverlayLayer.path = nil;
                self.prevOverlayLayer.hidden = true;
                if (self.currentIndex > 0) {
                    [self.captureButton setEnabled:false];
                }else{
                    [self.captureButton setEnabled:true];
                }
            });

        }
    }];
    /*
    self.isCalcOverlap = YES;
    CFRetain(sampleBuffer);
    dispatch_async(_overlapQueue, ^{
        CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
        CVPixelBufferLockBaseAddress(imageBuffer, 0);
        uint8_t* address = CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 0);
        int width = (int)CVPixelBufferGetWidth(imageBuffer);
        int height = (int)CVPixelBufferGetHeight(imageBuffer);
        if([StitchManager.sharedInstance calcOverlap:address width:width height:height points:self->_points]){
            dispatch_async(dispatch_get_main_queue(), ^{
                CGFloat width = self.view.frame.size.width/2;
                CGFloat height = self.view.frame.size.height/2;
                UIBezierPath* path = [[UIBezierPath alloc] init];
                [path moveToPoint:CGPointMake(self->_points[0]*width, self->_points[1]*height)];
                for(int i = 1; i < 4; i ++){
                    [path addLineToPoint:CGPointMake(self->_points[i*2]*width, self->_points[i*2+1]*height)];
                }
                [path addLineToPoint:CGPointMake(self->_points[0]*width, self->_points[1]*height)];
                [path closePath];
                self.prevOverlayLayer.path = path.CGPath;
                self.prevOverlayLayer.hidden = NO;
                
                path = [[UIBezierPath alloc] init];
                [path moveToPoint:CGPointMake(self->_points[8]*width, self->_points[9]*height)];
                for(int i = 1; i < 4; i ++){
                    [path addLineToPoint:CGPointMake(self->_points[i*2+8]*width, self->_points[i*2+9]*height)];
                }
                [path addLineToPoint:CGPointMake(self->_points[8]*width, self->_points[9]*height)];
                [path closePath];
                self.currOverlayLayer.path = path.CGPath;
                self.currOverlayLayer.hidden = NO;
            });
        }
        else{
            // no matching points
            NSLog(@"no matching points");
            dispatch_async(dispatch_get_main_queue(), ^{
                self.prevOverlayLayer.hidden = YES;
                self.currOverlayLayer.hidden = YES;
            });
        }
        CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
        CFRelease(sampleBuffer);
        self.isCalcOverlap = NO;
    });*/
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

-(CAShapeLayer*)prevOverlayLayer{
    if(_prevOverlayLayer == nil){
        _prevOverlayLayer = [[CAShapeLayer alloc] init];
//        _prevOverlayLayer.opacity = 0.5f;
        _prevOverlayLayer.fillColor = UIColor.blueColor.CGColor;
        _prevOverlayLayer.masksToBounds = YES;
        _prevOverlayLayer.frame = CGRectMake(0, 0, self.previewLayer.frame.size.width, self.previewLayer.frame.size.height);
        _prevOverlayLayer.hidden = true;

        [self.previewLayer addSublayer:_prevOverlayLayer];
    }
    return _prevOverlayLayer;
}

-(CAShapeLayer*)currOverlayLayer{
    if(_currOverlayLayer == nil){
        _currOverlayLayer = [[CAShapeLayer alloc] init];
        _currOverlayLayer.opacity = 0.5f;
        _currOverlayLayer.fillColor = UIColor.blueColor.CGColor;
        _currOverlayLayer.masksToBounds = YES;
        [self.previewLayer addSublayer:_currOverlayLayer];
    }
    return _currOverlayLayer;
}

-(void)setupPreview{
    if(self.currentIndex > 0){
        UIImage *img = [FileManager.sharedInstance imageAtIndex:self.currentIndex-1];
        CGFloat top = self.view.frame.size.height * 0.15;
        CGFloat subwidth = self.view.frame.size.width/2;
        CGFloat subheight = subwidth * img.size.height / img.size.width;
//        self.previewLayer.frame = CGRectMake(subwidth, top, subwidth, subheight);
        
        self.srcImageView.frame = CGRectMake(0, top, subwidth, subheight);
        self.srcImageView.image = img;
        self.srcImageView.hidden = YES;
        self.srcImageView.clipsToBounds = YES;
        self.shouldCalcOverlap = YES;
        
//        dispatch_async(_overlapQueue, ^{
//            NSString* imgPath = [FileManager.sharedInstance pathForImageAtIndex:self.currentIndex-1];
//            if(![StitchManager.sharedInstance setOverlapSrc:imgPath]){
//                NSLog(@"setOverlapSrc failed.");
//            }
//        });
        
//        _prevOverlayLayer.frame = CGRectMake(0, 0, subwidth, subheight);
        _currOverlayLayer.frame = CGRectMake(0, 0, subwidth, subheight);
    }
    else{
        self.previewLayer.frame = CGRectMake(0, 0, self.view.frame.size.width, self.view.frame.size.width*4/3);
//        self.previewLayer.frame = self.view.bounds;
        self.srcImageView.hidden = YES;
        self.shouldCalcOverlap = NO;
    }
    
    self.currOverlayLayer.hidden = YES;
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
    self.captureButton.enabled = NO;
    self.activityIndicator.hidden = NO;
    [self.activityIndicator startAnimating];
//    AVCapturePhotoSettings* photoSettings = AVCapturePhotoSettings.photoSettings;
    AVCapturePhotoSettings* photoSettings = [AVCapturePhotoSettings photoSettingsWithFormat:@{AVVideoCodecKey:AVVideoCodecTypeJPEG}];
    [photoSettings setHighResolutionPhotoEnabled:true];
    [self.photoOutput capturePhotoWithSettings:photoSettings delegate:self];
}

-(void)captureOutput:(AVCapturePhotoOutput *)output didFinishProcessingPhoto:(AVCapturePhoto *)photo error:(NSError *)error{
    int index = self.currentIndex;
    
    
    UIImage *image = [[UIImage alloc] initWithData:photo.fileDataRepresentation];
    UIImage *realImg = [image scaleTo:640];

    // save image
    [FileManager.sharedInstance saveImageData:photo.fileDataRepresentation atIndex:index];
    
//    NSString* imagePath = [FileManager.sharedInstance pathForImageAtIndex:index];
    NSString* outputPath = [FileManager.sharedInstance pathForPanoImageAtIndex:index];
    self.isCapturing = YES;

    __weak CameraViewController_video* weakSelf = self;
    [StitchManager.sharedInstance stitchOfImage:realImg angle_pitch:0 angle_roll:0 angle_yaw:0 logPath:@"" panoPath:outputPath completionHandler:^(bool success, NSString * _Nonnull msg, UIImage * _Nonnull panoImg, int bestPOV, NSArray * _Nonnull homography) {
        dispatch_async(dispatch_get_main_queue(), ^{
            if (success) {
                self.currentIndex ++;
                
                weakSelf.panoImage = [UIImage imageWithContentsOfFile:outputPath];
                self.panoView.image = weakSelf.panoImage ;
                self.panoView.hidden = NO;
//                weakSelf.overlayView.hidden = YES;
                //                [weakSelf setupPreview];
            }else{
                self.tostLabel.text = @"拼接失败";
                self.tostLabel.hidden = false;
                [self performSelector:@selector(hiddedTost) withObject:nil afterDelay:2];
            }
            weakSelf.captureButton.enabled = YES;
            weakSelf.activityIndicator.hidden = YES;
            [weakSelf.activityIndicator stopAnimating];
        });
        weakSelf.isCapturing = NO;

    }];
    // stitch image
//    [StitchManager.sharedInstance stitch:imagePath outputPath:outputPath retake:NO completionHandler:^(NSError * _Nonnull error) {
//        if(error != nil){
//            dispatch_async(dispatch_get_main_queue(), ^{
//                self.captureButton.enabled = YES;
//                self.activityIndicator.hidden = YES;
//                [self.activityIndicator stopAnimating];
//                self.isCapturing = NO;
//                [weakSelf showError:error.localizedDescription];
//            });
//            return;
//        }
//
//        self.currentIndex ++;
//        dispatch_async(dispatch_get_main_queue(), ^{
//            self.panoImage = [UIImage imageWithContentsOfFile:outputPath];
//            self.captureButton.enabled = YES;
//            self.activityIndicator.hidden = YES;
//            [self.activityIndicator stopAnimating];
//            self.isCapturing = NO;
//            self.overlayView.hidden = YES;
//            [self setupPreview];
//        });
//    }];
}

- (void)hiddedTost{
    self.tostLabel.hidden = true;
}
-(void)showError:(NSString*)message{
//    [self.view makeToast:message
//                duration:3
//                position:CSToastPositionCenter];
}

@end
