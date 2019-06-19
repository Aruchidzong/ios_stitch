//
//  QualityViewController.m
//  SDKDemo
//
//  Created by KanDao on 2018/12/6.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import "QualityViewController.h"
#import "SettingManager.h"

@interface QualityViewController ()<UITableViewDataSource, UITableViewDelegate>

@property (nonatomic, weak) IBOutlet UITableView* tableView;
@property (nonatomic, strong) NSArray* options;
@property (nonatomic, assign) int selected;

@end

@implementation QualityViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
    self.title = @"图片分辨率";
    self.options = SettingManager.sharedInstance.qualityOptions;
    self.selected = SettingManager.sharedInstance.quality;
    
    self.tableView.tableFooterView = [[UIView alloc] init];
}

#pragma mark - UITableView
- (NSInteger)tableView:(nonnull UITableView *)tableView numberOfRowsInSection:(NSInteger)section {
    return self.options.count;
}

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView{
    return 1;
}

- (nonnull UITableViewCell *)tableView:(nonnull UITableView *)tableView cellForRowAtIndexPath:(nonnull NSIndexPath *)indexPath {
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:@"Cell" forIndexPath:indexPath];
    cell.separatorInset = UIEdgeInsetsZero;
    
    UILabel* label = (UILabel*)[cell viewWithTag:1];
    label.text = self.options[indexPath.row];
    
    UIImageView* tickView = (UIImageView*)[cell viewWithTag:2];
    tickView.hidden = indexPath.row != self.selected;
    
    return cell;
}

-(void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath{
    [tableView deselectRowAtIndexPath:indexPath animated:YES];
    self.selected = (int)indexPath.row;
    SettingManager.sharedInstance.quality = self.selected;
    [tableView reloadData];
}

@end
