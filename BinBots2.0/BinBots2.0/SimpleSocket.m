//
//  SimpleSocket.m
//  BinBots2.0
//
//  Created by Tsvetan Zhivkov on 26/04/2015.
//  Copyright (c) 2015 Tsvetan Zhivkov. All rights reserved.
//

#import "SimpleSocket.h"

@interface SimpleSocket ()


- (void)stream:(NSStream *)theStream handleEvent:(NSStreamEvent)streamEvent;
//- (void)unscheduleFromRunLoop:(NSRunLoop *)aRunLoop forMode:(NSString *)mode;
- (void)initNetworkCommunication;
@property (nonatomic) NSMutableData *output;

//@property (nonatomic) NSMutableData *data;

@end

@implementation SimpleSocket

//====================================================
#pragma mark - Initialise socket
//====================================================
- (void)initNetworkCommunication {
    CFReadStreamRef readStream;
    CFWriteStreamRef writeStream;
    CFStreamCreatePairWithSocketToHost(NULL, (CFStringRef)@"192.168.0.104", 1111, &readStream, &writeStream);
    inputStream = (__bridge NSInputStream *)readStream;
    outputStream = (__bridge NSOutputStream *)writeStream;
    [inputStream setDelegate:self];
    [outputStream setDelegate:self];
    [inputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [outputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [inputStream open];
    [outputStream open];
}

- (void)stream:(NSStream *)theStream handleEvent:(NSStreamEvent)streamEvent {
    NSMutableData *data;
    _output = [NSMutableData dataWithData:_output];
    typedef enum {
        NSStreamEventNone = 0,
        NSStreamEventOpenCompleted = 1 << 0,
        NSStreamEventHasBytesAvailable = 1 << 1,
        NSStreamEventHasSpaceAvailable = 1 << 2,
        NSStreamEventErrorOccurred = 1 << 3,
        NSStreamEventEndEncountered = 1 << 4
    };
    switch (streamEvent) {
            
        case NSStreamEventOpenCompleted:
            NSLog(@"Stream opened");
            break;
            
        case NSStreamEventHasBytesAvailable:
            if (theStream == inputStream) {
                
                uint8_t buffer[1024];
                int len;
                while ([inputStream hasBytesAvailable]) {
                    len = [inputStream read:buffer maxLength:sizeof(buffer)];
                    if (len > 0) {
                        data = [[NSMutableData alloc] initWithBytes:buffer length:len];
                        [_output appendData:data];
                        NSLog(@"output: %lu", (unsigned long)data.length);
 
                    }
                }
            }
            break;
            
        case NSStreamEventErrorOccurred:
            NSLog(@"Can not connect to the host!");
            break;
            
        case NSStreamEventEndEncountered:
            [theStream close];
            [theStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
            NSLog(@"Event end encountered");
            break;
            
        default:
            NSLog(@"Unknown event");
    }
    if(nil != _output){
        [self saveData];
    }
}

- (void)sendObjectImage {
    [self initNetworkCommunication];
    NSString *response  = [NSString stringWithFormat:@"SEND_OBJ_PNG#"];
    NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
    [outputStream write:[data bytes] maxLength:[data length]];
}

- (void)sendObjectSecondImage {
    [self initNetworkCommunication];
    NSString *response  = [NSString stringWithFormat:@"SEND_VER_PNG#"];
    NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
    [outputStream write:[data bytes] maxLength:[data length]];
}

- (void)disconnect {
    [inputStream close];
    [outputStream close];
    [inputStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [outputStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
}

- (void)saveData {
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    NSString *picPath = [documentsDirectory stringByAppendingPathComponent:@"objImage.png"];
    //NSLog(@"Location: %@", picPath);
    [_output writeToFile:picPath atomically:YES];
    //NSLog(@"image saved");
}

@end