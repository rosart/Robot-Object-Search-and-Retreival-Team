//
//  mapView.m
//  BinBots2.0
//
//  Created by Tsvetan Zhivkov on 24/04/2015.
//  Copyright (c) 2015 Tsvetan Zhivkov. All rights reserved.
//

#import "mapView.h"

@implementation mapView

// An empty implementation adversely affects performance during animation.
- (void)drawRect:(CGRect)rect {
    // Drawing code
    CGRect bounds = [self bounds];
    [[UIColor lightGrayColor] set];
    UIRectFill(bounds);

}

@end
