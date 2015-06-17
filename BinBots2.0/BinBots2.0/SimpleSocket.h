//
//  SimpleSocket.h
//  BinBots2.0
//
//  Created by Tsvetan Zhivkov on 26/04/2015.
//  Copyright (c) 2015 Tsvetan Zhivkov. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface SimpleSocket : NSObject <NSStreamDelegate> {
    NSInputStream *inputStream;
    NSOutputStream *outputStream;
}

- (void)sendObjectImage;
- (void)sendObjectSecondImage;
- (void)disconnect;
- (void)saveData;

@end