//
//  FileManager.m
//  SDKDemo
//
//  Created by KanDao on 2018/11/24.
//  Copyright © 2018 KandaoVR. All rights reserved.
//

#import "FileManager.h"

@interface FileManager ()

@property (nonatomic, strong) NSFileManager* fileManager;
@property (nonatomic, strong) NSURL* folder;
@property (nonatomic, strong) NSURL* panoFolder;
@property (nonatomic, strong) NSURL* logFolder;
//@property (nonatomic, strong) NSURL* logFolder;

@end

@implementation FileManager

+ (instancetype)sharedInstance
{
    static dispatch_once_t p = 0;
    __strong static id _sharedObject = nil;
    
    dispatch_once(&p, ^{
        _sharedObject = [[self alloc] init];
    });
    
    return _sharedObject;
}

-(instancetype)init{
    if(self = [super init]){
        self.fileManager = NSFileManager.defaultManager;
        
        
        NSString *documents = [NSHomeDirectory() stringByAppendingPathComponent:@"Documents"];
        NSString *folderPath = [documents stringByAppendingPathComponent:@"stitching/imgs"];
        NSString *panoFolderPath = [documents stringByAppendingPathComponent:@"stitching/pano"];
        NSString *logFolderPath = [documents stringByAppendingPathComponent:@"stitching/log"];

        self.panoFolder = [self checkPath:panoFolderPath];
        self.logFolder = [self checkPath:logFolderPath];
        self.folder = [self checkPath:folderPath];
        //        NSString *logFolderPath = [documents stringByAppendingPathComponent:@"stitching/log"];
        //        self.logFolder = [self checkPath:logFolderPath];
        
    }
    return self;
}

- (NSURL*)checkPath:(NSString*)path {
    if(![self.fileManager fileExistsAtPath:path]){//如果不存在,则说明是第一次运行这个程序，那么建立这个文件夹
        [self.fileManager createDirectoryAtPath:path withIntermediateDirectories:YES attributes:nil error:nil];
    }
    return [NSURL URLWithString:path];
}
-(NSString*)pathForLogWithFolder:(NSString*)folder itemId:(NSString*)_id{
    
    NSString *logFolderPath = [[NSHomeDirectory() stringByAppendingPathComponent:@"Documents"] stringByAppendingPathComponent:[NSString stringWithFormat:@"stitching/log/%@",folder]];
    NSURL *logFolder = [self checkPath:logFolderPath];
    
    NSString* filename = [NSString stringWithFormat:@"log%@.txt", _id];
    NSURL* url = [logFolder URLByAppendingPathComponent:filename];
    return url.path;
    
}
-(NSString*)pathForImageAtIndex:(int)index{
    NSString* filename = [NSString stringWithFormat:@"img%d.jpg", index];
    NSURL* url = [self.folder URLByAppendingPathComponent:filename];
    return url.path;
}

-(NSString*)pathForPanoImageAtIndex:(int)index{
    NSString* filename = [NSString stringWithFormat:@"pano%d.jpg", index];
    NSURL* url = [self.panoFolder URLByAppendingPathComponent:filename];
    return url.path;
}
-(NSString*)pathForLogAtIndex:(int)index{
    NSString* filename = [NSString stringWithFormat:@"log%d.txt", index];
    NSURL* url = [self.logFolder URLByAppendingPathComponent:filename];
    return url.path;
}
-(NSString*)pathForImageAtItemId:(NSString*)_id atIndex:(int)index{
    NSString* filename = [NSString stringWithFormat:@"imgItemId%@index%d.jpg",_id, index];
    NSURL* url = [self.folder URLByAppendingPathComponent:filename];
    return url.path;
}
-(NSString*)pathForPanoImageAtItemId:(NSString*)_id atIndex:(int)index{
    NSString* filename = [NSString stringWithFormat:@"panoItemId%@index%d.jpg", _id,index];
    NSURL* url = [self.panoFolder URLByAppendingPathComponent:filename];
    return url.path;
}


-(BOOL)saveImage:(UIImage*)image atIndex:(int)index{
    return [self saveImageData:UIImageJPEGRepresentation(image, 1.0) atIndex:index];
}

-(BOOL)saveImage:(UIImage*)image atItemId:(NSString*)_id atIndex:(int)index{
    return [self saveImageData:UIImageJPEGRepresentation(image, 1.0) atItemId:_id atIndex:index];
}
-(BOOL)saveImageData:(NSData*)data atItemId:(NSString*)_id atIndex:(int)index{
    NSString* path = [self pathForImageAtItemId:_id atIndex:index];
    return [self saveData:data toPath:path];
}
-(BOOL)saveImageData:(NSData*)data atIndex:(int)index{
    NSString* path = [self pathForImageAtIndex:index];
    return [self saveData:data toPath:path];
}

-(BOOL)savePanoImage:(UIImage*)image atIndex:(int)index{
    NSString* path = [self pathForPanoImageAtIndex:index];
    return [self saveData:UIImageJPEGRepresentation(image, 1.0) toPath:path];
}

-(BOOL)saveData:(NSData*)data toPath:(NSString*)path{
    NSError* error;
    BOOL success = [data writeToFile:path options:NSDataWritingAtomic error:&error];
    if(!success){
        NSLog(@"fail write error %@ for path = %@", error, path);
    }
    return success;
}

-(UIImage*)imageAtIndex:(int)index{
    NSString* path = [self pathForImageAtIndex:index];
    if(![self.fileManager fileExistsAtPath:path]){
        NSLog(@"image %d does not exist.", index);
        return nil;
    }
    return [UIImage imageWithContentsOfFile:path];
}

-(UIImage*)panoImageAtIndex:(int)index{
    NSString* path = [self pathForPanoImageAtIndex:index];
    if(![self.fileManager fileExistsAtPath:path]){
        NSLog(@"pano image %d does not exist.", index);
        return nil;
    }
    return [UIImage imageWithContentsOfFile:path];
}

@end
