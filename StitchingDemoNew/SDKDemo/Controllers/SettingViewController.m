//
//  SettingViewController.m
//  SDKDemo
//
//  Created by KanDao on 2018/11/24.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import "SettingViewController.h"
#import "SettingManager.h"

@interface SettingViewController ()

@property (nonatomic, weak) IBOutlet UISwitch* poseSwitch;

@property (nonatomic, weak) IBOutlet UIView* qualityView;
@property (nonatomic, weak) IBOutlet UILabel* qualityLabel;

@end

@implementation SettingViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
    self.navigationItem.title = NSLocalizedString(@"设置", nil);
    
    self.qualityView.userInteractionEnabled = YES;
    [self.qualityView addGestureRecognizer:[[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(qualityClicked)]];
}

-(void)viewWillAppear:(BOOL)animated{
    [super viewWillAppear:animated];
    self.navigationController.navigationBarHidden = NO;
    self.poseSwitch.on = SettingManager.sharedInstance.checkPose;
    self.qualityLabel.text = SettingManager.sharedInstance.qualityText;
}

-(IBAction)poseSwitchChanged:(id)sender{
    SettingManager.sharedInstance.checkPose = self.poseSwitch.on;
}

-(IBAction)qualityClicked{
    [self performSegueWithIdentifier:@"showQuality" sender:self];
}

@end
