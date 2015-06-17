//
//  ImageViewController.m
//  BinBots2.0
//
//  Created by Tsvetan Zhivkov on 27/04/2015.
//  Copyright (c) 2015 Tsvetan Zhivkov. All rights reserved.
//

#import "ImageViewController.h"
#import "SimpleSocket.h"

@interface ImageViewController ()
@property (nonatomic, strong) SimpleSocket *simplesock;
@property (weak, nonatomic) IBOutlet UIImageView *objectImage;
- (IBAction)wrongImageButton:(id)sender;
- (IBAction)correctImageButton:(id)sender;
@property (nonatomic) NSString* pictureStr;

@end

@implementation ImageViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    self.simplesock = [[SimpleSocket alloc] init];
    [[self simplesock] sendObjectImage];
    
    NSTimer * loadImageTimer = [NSTimer
                      scheduledTimerWithTimeInterval:(1.5f)
                      target:self
                      selector:@selector(loadImage:)
                      userInfo:nil
                      repeats:NO];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
}

- (void)loadImage:(NSTimer *)loadImageTimer {
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    NSString *picPath = [documentsDirectory stringByAppendingPathComponent:@"objImage.png"];
    if ([[NSFileManager defaultManager] fileExistsAtPath:picPath]) {
        UIImage *objectImage = [UIImage imageWithContentsOfFile:picPath];
        _objectImage.image = objectImage;
        NSLog(@"Done loading");
    }
}

- (IBAction)wrongImageButton:(id)sender {
    _pickUp = NO;
}

- (IBAction)correctImageButton:(id)sender {
    _pickUp = YES;
}
@end
