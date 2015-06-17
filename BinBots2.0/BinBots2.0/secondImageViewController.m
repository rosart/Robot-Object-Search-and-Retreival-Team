//
//  secondImageViewController.m
//  BinBots2.0
//
//  Created by Tsvetan Zhivkov on 27/04/2015.
//  Copyright (c) 2015 Tsvetan Zhivkov. All rights reserved.
//

#import "secondImageViewController.h"
#import "SimpleSocket.h"

@interface secondImageViewController ()
@property (nonatomic, strong) SimpleSocket *simplesock;
- (IBAction)confirmButton:(id)sender;
- (IBAction)wrongButton:(id)sender;
@property (weak, nonatomic) IBOutlet UIImageView *objectImage;


@end

@implementation secondImageViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    self.simplesock = [[SimpleSocket alloc] init];
    [[self simplesock] sendObjectSecondImage];
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
        //Get the image from the correct directory/png file
        UIImage *objectImage = [UIImage imageWithContentsOfFile:picPath];
        NSLog(@"size of image: %lu", (unsigned long)picPath.length);
        _objectImage.image = objectImage;
    }
}

- (IBAction)confirmButton:(id)sender {
    _confirm = YES;
}

- (IBAction)wrongButton:(id)sender {
    _confirm = NO;
}
@end
