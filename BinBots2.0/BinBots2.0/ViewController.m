//
//  ViewController.m
//  BinBots2.0
//
//  Created by Tsvetan Zhivkov on 24/04/2015.
//  Copyright (c) 2015 Tsvetan Zhivkov. All rights reserved.
//
//Import h
#import "ViewController.h"
#import "ImageViewController.h"
#import "secondImageViewController.h"
#import "GCDAsyncSocket.h"
#import "mapView.h"
#include <math.h>
#include <stdio.h>
//global static definition
#define DEGREES_RADIANS(angle) ((angle) / 180.0 * M_PI)

@interface ViewController ()
//Add private instance declaration for GCDAsyncSocket library
@property (nonatomic, strong) GCDAsyncSocket *asyncSocket;
//Add an instance declaration for the map view of type IBOutlet
@property (weak, nonatomic) IBOutlet mapView *mapview;
//Create private mutable data variable to hold server buffer data
@property (nonatomic, retain) NSMutableData *bufferData;
//Create private mutable string variable to store buffer data in a string format
@property (nonatomic, retain) NSMutableString *theMsg;
@property (nonatomic) BOOL insideTheMap;

//integer value to easily change port
@property (nonatomic, assign) int port;
//String value to easily change IP
@property (nonatomic, assign) NSString* host;
//All declared Image views linked to the app interface
@property (weak, nonatomic) IBOutlet UIImageView *mapBotImage;
@property (weak, nonatomic) IBOutlet UIImageView *armBotImage;
@property (weak, nonatomic) IBOutlet UIImageView *binBotImage;
@property (weak, nonatomic) IBOutlet UIImageView *flyBotImage;
@property (weak, nonatomic) IBOutlet UIImageView *mapViewImage;
//All declared Buttons linked to the app interface
@property (weak, nonatomic) IBOutlet UIButton *mapBotButton;
@property (weak, nonatomic) IBOutlet UIButton *armBotButton;
@property (weak, nonatomic) IBOutlet UIButton *binBotButton;
@property (weak, nonatomic) IBOutlet UIButton *flyBotButton;
//All declared Labels linked to the app interface
@property (weak, nonatomic) IBOutlet UILabel *touchPoseLabel;
@property (weak, nonatomic) IBOutlet UILabel *statusLabel;
@property (weak, nonatomic) IBOutlet UILabel *mapStatusLabel;
@property (weak, nonatomic) IBOutlet UILabel *armStatusLabel;
@property (weak, nonatomic) IBOutlet UILabel *baseStatusLabel;
@property (weak, nonatomic) IBOutlet UILabel *binStatusLabel;
@property (weak, nonatomic) IBOutlet UILabel *flyStatusLabel;
@property (weak, nonatomic) IBOutlet UILabel *mapXpos;
@property (weak, nonatomic) IBOutlet UILabel *mapYpos;
@property (weak, nonatomic) IBOutlet UILabel *mapZpos;
@property (weak, nonatomic) IBOutlet UILabel *armXpos;
@property (weak, nonatomic) IBOutlet UILabel *armYpos;
@property (weak, nonatomic) IBOutlet UILabel *armZpos;
@property (weak, nonatomic) IBOutlet UILabel *binXpos;
@property (weak, nonatomic) IBOutlet UILabel *binYpos;
@property (weak, nonatomic) IBOutlet UILabel *binZpos;
@property (weak, nonatomic) IBOutlet UILabel *flyXpos;
@property (weak, nonatomic) IBOutlet UILabel *flyYpos;
@property (weak, nonatomic) IBOutlet UILabel *flyZpos;
//Declaration of all variables of NSNumber type
@property (strong, nonatomic) NSNumber * x;
@property (strong, nonatomic) NSNumber * y;
@property (strong, nonatomic) NSNumber * z;
@property (strong, nonatomic) NSNumber * mapX;
@property (strong, nonatomic) NSNumber * mapY;
@property (strong, nonatomic) NSNumber * mapZ;
@property (strong, nonatomic) NSNumber * armX;
@property (strong, nonatomic) NSNumber * armY;
@property (strong, nonatomic) NSNumber * armZ;
@property (strong, nonatomic) NSNumber * binX;
@property (strong, nonatomic) NSNumber * binY;
@property (strong, nonatomic) NSNumber * binZ;
//All button action events
- (IBAction)resetButton:(id)sender;
- (IBAction)selectRobotButton:(id)sender;
- (IBAction)setPoseButton:(id)sender;
- (IBAction)mapDoneButton:(id)sender;

@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    //Add the map view as a subview of the main app view controller
    [[self view] addSubview:[self mapview]];
    //Load the map on app load
    [self loadImage];
    /*testing code
    _x = [NSNumber numberWithFloat:-1.010];
    _y = [NSNumber numberWithFloat:0.739];
    _z = [NSNumber numberWithInt:0];
     */
    //Setting the port for the view controller
    //Followed by the setting of the host IP
    _port = 1111;
    _host = @"192.168.0.104";
    //Initialise and allocate buffer data
    _bufferData = [[NSMutableData alloc] init];
    //Initialise and allocate an instance of async socket
    _asyncSocket = [[GCDAsyncSocket alloc] initWithDelegate:self delegateQueue:dispatch_get_main_queue()];
    //Create a periodic timer which asks server for state update
    NSTimer * state = [NSTimer
                           scheduledTimerWithTimeInterval:(1.0f)
                           target:self
                           selector:@selector(sendStateMessage:)
                           userInfo:nil
                           repeats:YES];
    //Create a periodic timer which asks server for pose update
    NSTimer * pose = [NSTimer
                           scheduledTimerWithTimeInterval:(0.4f)
                           target:self
                           selector:@selector(sendPoseMessage:)
                           userInfo:nil
                           repeats:YES];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
}

//======================================================================
#pragma mark - SocketLib Methods
//======================================================================
//The methods below are needed to send and receive messages from the server-
//using the GCDAsyncSocket Library. This includes sending different messages-
//to the server, receiveing string data from server and closing socket when-
//socket is finished sending/receiving data.

//Message method sent to ask for states
- (void)sendStateMessage:(NSTimer *)state {
    if([_asyncSocket connectToHost:_host onPort:_port error:nil]) {
        NSString *response = [NSString stringWithFormat:@"SEND_STATES#"];
        NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
        [[self asyncSocket] writeData:data withTimeout:-1 tag:0];
    }
    else if(![_asyncSocket connectToHost:_host onPort:_port error:nil]) {
        [self clearAll];
    }
}

//Message method sent to ask for poses
- (void)sendPoseMessage:(NSTimer *)pose {
    if([_asyncSocket connectToHost:_host onPort:_port error:nil]) {
        NSString *response = [NSString stringWithFormat:@"SEND_POSES#"];
        NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
        [[self asyncSocket] writeData:data withTimeout:-1 tag:0];
    }
    else {
        //NSLog(@"No POSES");
    }
}

//The method used to receive data from the server
- (void)socket:(GCDAsyncSocket *)sock didReadData:(NSData *)data withTag:(long)tag {
    [sock readDataWithTimeout:-1 tag:0];
    [_bufferData setLength:0];
    _theMsg = [[NSMutableString alloc] initWithData:data encoding:NSASCIIStringEncoding];
    //Put the buffered data into _theMsg string and separate with "#" character
    NSArray * StrArray = [_theMsg componentsSeparatedByString:@"#"];
    //If the first part of the string is "STATE" then getStates from the string
    if([[StrArray objectAtIndex:0]  isEqual: @"STATE"]){
        [self getStates:_theMsg];
        
    }
    //else if first part of string is "POSE" getPoses from the string
    else if([[StrArray objectAtIndex:0] isEqual: @"POSE"]){
        [self getPose:_theMsg];
    }
    //Read data from server with no timeout (-1) and no buffer off-set (set to 0) and tag "tag"
    [_asyncSocket readDataWithTimeout:-1 buffer:_bufferData bufferOffset:[_bufferData length] tag:tag];
}

//Checks that the socket did connect to the specified host and port
- (void)socket:(GCDAsyncSocket *)sock didConnectToHost:(NSString *)host port:(UInt16)port {
    //NSLog(@"Socket:DidConnectToHost: %@ Port: %hu", host, port);
    [_asyncSocket readDataWithTimeout:-1 tag:0];
}

//Check when the socket disconnected (make sure it did, and check for error)
- (void)socketDidDisconnect:(GCDAsyncSocket *)sock withError:(NSError *)err {
    //NSLog(@"Socket disconnected from host");
}

//Message method sent to tell arm-robot to pick-up object
- (void)sendPickupYes {
    if([_asyncSocket connectToHost:_host onPort:_port error:nil]) {
        NSString *response = [NSString stringWithFormat:@"INPUT#PICKUP#"];
        NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
        [[self asyncSocket] writeData:data withTimeout:-1 tag:0];
    }
    else {
        NSLog(@"No connection");
    }
}

//Message method sent to tell arm-robot to leave object
- (void)sendPickupNo {
    if([_asyncSocket connectToHost:_host onPort:_port error:nil]) {
        NSString *response = [NSString stringWithFormat:@"INPUT#LEAVE#"];
        NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
        [[self asyncSocket] writeData:data withTimeout:-1 tag:0];
    }
    else {
        NSLog(@"No connection");
    }
}

//Message method sent to tell server to confirm a successful pick-up
- (void)sendPickupCheckYes {
    if([_asyncSocket connectToHost:_host onPort:_port error:nil]) {
        NSString *response = [NSString stringWithFormat:@"INPUT#SUCC#"];
        NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
        [[self asyncSocket] writeData:data withTimeout:-1 tag:0];
    }
    else {
        NSLog(@"No connection");
    }
}

//Message method sent to tell server to confirm a failed pick-up
- (void)sendPickupCheckNo {
    if([_asyncSocket connectToHost:_host onPort:_port error:nil]) {
        NSString *response = [NSString stringWithFormat:@"INPUT#FAIL#"];
        NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
        [[self asyncSocket] writeData:data withTimeout:-1 tag:0];
    }
    else {
        NSLog(@"No connection");
    }
}

//Message method sent to start robots
- (void)sendStart {
    if([_asyncSocket connectToHost:_host onPort:_port error:nil]) {
        NSString *response = [NSString stringWithFormat:@"START#"];
        NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
        [[self asyncSocket] writeData:data withTimeout:-1 tag:0];
    }
    else {
        NSLog(@"No connection");
    }
}

//Message method sent to reset robots and server
- (void)sendReset {
    if([_asyncSocket connectToHost:_host onPort:_port error:nil]) {
        NSString *response = [NSString stringWithFormat:@"RESET#"];
        NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
        [[self asyncSocket] writeData:data withTimeout:-1 tag:0];
    }
    else {
        NSLog(@"No connection");
    }
}

//Message method sent to tell server and robots the map-robot is finishing mapping
- (void)sendMapDone {
    if([_asyncSocket connectToHost:_host onPort:_port error:nil]) {
        NSString *response = [NSString stringWithFormat:@"INPUT#DONE#"];
        NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
        [[self asyncSocket] writeData:data withTimeout:-1 tag:0];
    }
    else {
        NSLog(@"No connection");
    }
}
//======================================================================
#pragma mark - Touch Event
//======================================================================
//Touch events were created to allow the app to send poses, but this was never-
//implemented in the final iOS app as the critical project goals needed to be-
//achieved. It was also used to help get estimated x/y co-ordinates of the map-
//view space

//Method which reacts to when a touch first began within the view
- (void) touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event {
    //single touch call any object to retrieve from touches NSSet
    //commented out method used to allow "tap" away keyboard/keypad
    //[self.view endEditing:YES];
    UITouch *touch = [touches anyObject];
    if ([touch view] == [self mapview]) {
        self.touchPoseLabel.hidden = NO;
        CGPoint touchPoint = [touch locationInView:[self mapview]];
        NSString *msg = [NSString stringWithFormat:@"Touch Began at %.0f, %.0f", touchPoint.x, touchPoint.y];
        [[self touchPoseLabel] setText:msg];
    }
}

//Method which reacts to when a touch moves within the view
- (void) touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event {
    UITouch *touch = [touches anyObject];
    CGPoint touchPoint = [touch locationInView:[self mapview]];
    CGFloat boundWidth = touchPoint.x;
    CGFloat boundHeight = touchPoint.y;
    if ([touch view] == [self mapview] ) {
        NSString *msg = [NSString stringWithFormat:@"Touch Moved to %.0f, %.0f", boundWidth, boundHeight];
        [[self touchPoseLabel] setText:msg];
        //used for grabing map touch coordinates
        //[self moveArmBot:[NSNumber numberWithFloat:touchPoint.x] andYval:[NSNumber numberWithFloat:touchPoint.y]];
    }
       else if(boundWidth < 0 || boundWidth > 800 || boundHeight < 0 || boundHeight > 600) {
            NSString *msg = [NSString stringWithFormat:@"Out of Map bounds!"];
            [[self touchPoseLabel] setText:msg];
        }
    }

//Method which reacts to when a touch ends within the view/outside the view
- (void) touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event {
    UITouch *touch = [touches anyObject];
    CGPoint touchPoint = [touch locationInView:[self mapview]];
    CGFloat boundWidth = touchPoint.x;
    CGFloat boundHeight = touchPoint.y;
    if([touch view] == [self mapview]) {
        _insideTheMap = YES;
        NSString *msg = [NSString stringWithFormat:@"Touch Ended at %.0f, %.0f", boundWidth, boundHeight];
        [[self touchPoseLabel] setText:msg];
        //used for grabing map touch coordinates
        //[self moveArmBot:[NSNumber numberWithFloat:touchPoint.x] andYval:[NSNumber numberWithFloat:touchPoint.y]];
        //[self getRobotHighlighted:[NSNumber numberWithFloat:boundWidth] with:[NSNumber numberWithFloat:boundHeight]];
    }
    else if(touchPoint.x <= 800 || touchPoint.y <= 650) {
            NSString *msg = [NSString stringWithFormat:@"Out of Map bounds!"];
            [[self touchPoseLabel] setText:msg];
            _insideTheMap = NO;
        }
}

//Method which reacts to when a touch is cancelled (not used - implemented
//for good programming practise
- (void) touchesCancelled:(NSSet *)touches withEvent:(UIEvent *)event {
    UITouch *touch = [touches anyObject];
    if ([touch view] == [self mapview]) {
        [[self touchPoseLabel] setText:@"Touch Cancelled"];
    }
}

//======================================================================
#pragma mark - Get States/Poses
//======================================================================
//The methods used to get the individial states and poses for the robots

//getStates method takes the string and stores it in an array which-
//separates each state by the character "#".
//Then knowing the order of the states, from the string sent by the-
//server, they are arranged by the corresponding index in the array
- (void)getStates:(NSMutableString *)theStr {
    NSArray * StrArray = [theStr componentsSeparatedByString:@"#"];
    for(int i = 0; i < StrArray.count; i++){
        _mapStatusLabel.text = [StrArray objectAtIndex:5];
        _armStatusLabel.text = [StrArray objectAtIndex:1];
        _baseStatusLabel.text = [StrArray objectAtIndex:2];
        _binStatusLabel.text = [StrArray objectAtIndex:3];
        _flyStatusLabel.text = [StrArray objectAtIndex:4];
        _statusLabel.text = [NSString stringWithFormat: @"Server Status: %@",[StrArray objectAtIndex:6]];
    }
}

//The getPose method is similar to the above getStates method.
//The full string received from the server is stored in an array-
//separated by the character "#".
- (void)getPose:(NSMutableString *)theStr {
    NSArray * StrArray = [theStr componentsSeparatedByString:@"#"];
    for(int i = 0; i < StrArray.count; i++){
        //if the object at index 1 in array is equal to "NONE" then-
        //Print "NONE" value in labels
        /*
        if([[StrArray objectAtIndex:1] isEqualToString:@"NONE"]) {
            _armBotImage.hidden = YES;
            //_armXpos.text = [NSString stringWithFormat:@"X: %@", [StrArray objectAtIndex:1]];
            //_armYpos.text = [NSString stringWithFormat:@"Y: %@", [StrArray objectAtIndex:2]];
            //_armZpos.text = [NSString stringWithFormat:@"Z: %@", [StrArray objectAtIndex:3]];
        }
        //else call convertArmVariables method, and pass the objects in array
        else {
            //[self convertArmVariables:[StrArray objectAtIndex:1] with:[StrArray objectAtIndex:2] andAngle:[StrArray objectAtIndex:3]];
        }
         */
        //if the object at index 4 in array is equal to "NONE" then-
        //Print "NONE" value in labels
        if([[StrArray objectAtIndex:4] isEqualToString:@"NONE"]) {
            _binBotImage.hidden = YES;
            _binXpos.text = [NSString stringWithFormat:@"X: %@", [StrArray objectAtIndex:4]];
            _binYpos.text = [NSString stringWithFormat:@"Y: %@", [StrArray objectAtIndex:5]];
            _binZpos.text = [NSString stringWithFormat:@"Z: %@", [StrArray objectAtIndex:6]];
        }
        //else call convertBinVariables method, and pass the objects in array
        else {
            [self convertBinVariables:[StrArray objectAtIndex:4] with:[StrArray objectAtIndex:5] andAngle:[StrArray objectAtIndex:6]];
            
        }
        //if the object at index 4 in array is equal to "NONE" then-
        //Print "NONE" value in labels
        if([[StrArray objectAtIndex:7] isEqualToString:@"NONE"]) {
            _mapBotImage.hidden = YES;
            _mapXpos.text = [NSString stringWithFormat:@"X: %@", [StrArray objectAtIndex:7]];
            _mapYpos.text = [NSString stringWithFormat:@"Y: %@", [StrArray objectAtIndex:8]];
            _mapZpos.text = [NSString stringWithFormat:@"Z: %@", [StrArray objectAtIndex:9]];
        }
        //else call convertMapVariables method, and pass the objects in array
        else {
            [self convertMapVariables:[StrArray objectAtIndex:7] with:[StrArray objectAtIndex:8] andAngle:[StrArray objectAtIndex:9]];
        }
        //Fly robot not implemented in application, therefore just set values-
        //to the default object in index 0 ("POSE")
        _flyXpos.text = [StrArray objectAtIndex:0];
        _flyYpos.text = [StrArray objectAtIndex:0];
        _flyZpos.text = [StrArray objectAtIndex:0];
    }
}

//======================================================================
#pragma mark - Robot Movement Methods
//======================================================================

- (void)moveMapBot:(UIImageView *)image duration:(NSTimeInterval)duration delay:(NSTimeInterval)delay
              curve:(int)curve rotations:(CGFloat)rotations
{
    _mapBotImage.hidden = YES;
    [UIView animateWithDuration:duration
                          delay:delay
                        options:0
                     animations:^{
                         [UIView setAnimationCurve:curve];
                         _mapBotImage.transform = CGAffineTransformMakeRotation(rotations);
                        [_mapBotImage setFrame:CGRectMake([_mapX floatValue],[_mapY floatValue], _mapBotImage.frame.size.width, _mapBotImage.frame.size.height)];
                         
                     }
                     completion:^(BOOL finished){
                     }];
    [UIView commitAnimations];
    
}

//motion function for arm-bot with pointer to Image View and a set duration, delay -
//and rotation options
- (void)moveArmBot:(UIImageView *)image duration:(NSTimeInterval)duration delay:(NSTimeInterval)delay
             curve:(int)curve rotations:(CGFloat)rotations
{
    //display image on map space
    _armBotImage.hidden = NO;
    //block code for animation
    [UIView animateWithDuration:duration
                          delay:delay
                        options:0 //animation option set to zero, defined when method is called
                     animations:^{
                         [UIView setAnimationCurve:curve];
                         //angular rotation using radians from 0-to-2*Pi
                         _armBotImage.transform = CGAffineTransformMakeRotation(rotations);
                         //create a new frame and redraw image view in armX and armY location
                         //with image view keeping original frame size width/height
                         [_armBotImage setFrame:CGRectMake([_armX floatValue],[_armY floatValue], _armBotImage.frame.size.width, _armBotImage.frame.size.height)];
                         
                     }
                        //with no options for when animation is finished
                     completion:^(BOOL finished){
                     }];
    [UIView commitAnimations];
    
}
 
- (void)moveBinBot:(UIImageView *)image duration:(NSTimeInterval)duration delay:(NSTimeInterval)delay
             curve:(int)curve rotations:(CGFloat)rotations
{
    _binBotImage.hidden = NO;
    [UIView animateWithDuration:duration
                          delay:delay
                        options:0
                     animations:^{
                         [UIView setAnimationCurve:curve];
                         _binBotImage.transform = CGAffineTransformMakeRotation(rotations);
                         [_binBotImage setFrame:CGRectMake([_binX floatValue],[_binY floatValue], _binBotImage.frame.size.width, _binBotImage.frame.size.height)];
                         
                     }
                     completion:^(BOOL finished){
                     }];
    [UIView commitAnimations];
    
}
/*
- (void)moveMapBot {
    _mapBotImage.hidden = NO;
    [UIView animateWithDuration:0.30
                          delay:0.0
                        options: UIViewAnimationOptionCurveEaseOut
                     animations:^{
                         [_mapBotImage setFrame:CGRectMake([_mapX floatValue],[_mapY floatValue], _mapBotImage.frame.size.width, _mapBotImage.frame.size.height)];
                     }
                     completion:^(BOOL finished){
                     }];
}
 code used to capture values on the map view and send to server
 =============================================================================
 - (void)moveArmBot: (NSNumber *)xVal andYval:(NSNumber *)yVal {
 _armBotImage.hidden = NO;
 [UIView animateWithDuration:0.30
 delay:0.0
 options: UIViewAnimationOptionCurveEaseOut
 animations:^{
 [_armBotImage setFrame:CGRectMake([xVal floatValue],[yVal floatValue], _armBotImage.frame.size.width, _armBotImage.frame.size.height)];
 _armZpos.text = @"Z: 230";
 _armXpos.text = [NSString stringWithFormat:@"X: %.f", [xVal floatValue]];
 _armYpos.text = [NSString stringWithFormat:@"Y: %.f", [yVal floatValue]];
 
 }
 completion:^(BOOL finished){
 }];
 }
 =============================================================================
- (void)moveBinBot {
    _binBotImage.hidden = NO;
    [UIView animateWithDuration:0.30
                          delay:0.0
                        options: UIViewAnimationOptionCurveEaseOut
                     animations:^{
                         [_binBotImage setFrame:CGRectMake([_binX floatValue],[_binY floatValue], _binBotImage.frame.size.width, _binBotImage.frame.size.height)];
                     }
                     completion:^(BOOL finished){
                     }];
}

- (void)moveFlyBot {
    _flyBotImage.hidden = NO;
    [UIView animateWithDuration:0.30
                          delay:0.0
                        options: UIViewAnimationOptionCurveEaseOut
                     animations:^{
                         [_flyBotImage setFrame:CGRectMake([_x floatValue],[_y floatValue], _flyBotImage.frame.size.width, _flyBotImage.frame.size.height)];
                     }
                     completion:^(BOOL finished){
                     }];
}

*/
//======================================================================
#pragma mark - Button Methods
//======================================================================
//Handles button presses or button events

//Simple method used to set the button passed to be highlighted
- (void)highlightButton:(UIButton *)button {
    [button setHighlighted:YES];
}

//
- (IBAction)selectRobotButton:(id)sender {
    UIButton *button = (UIButton *)sender;
    NSArray* buttons = [NSArray arrayWithObjects:_mapBotButton, _armBotButton, _binBotButton, _flyBotButton, nil];
    for (button in buttons) {
        if(button == sender){
            [self performSelector:@selector(highlightButton:) withObject:sender afterDelay:0.0];
        }
        else {
            [button setHighlighted:NO];
        }
    }
}

//Button touch-up inside action, calls sendStart method (send start to server)
- (IBAction)setPoseButton:(id)sender {
    [self sendStart];
}

//Button touch-up inside action, calls sendMapDone method (send done to server)
- (IBAction)mapDoneButton:(id)sender {
    [self sendMapDone];
}

//Button touch-up inside action, calls sendReset method (send reset to server)
- (IBAction)resetButton:(id)sender {
    [self sendReset];
}

//======================================================================
#pragma mark - unwindSegue Methods
//======================================================================
//The unwindSegue methods are used in iOS as a type of transition and data-
//passing between view controllers in the application

- (IBAction)unwindSegue:(UIStoryboardSegue *)unwindSegue {
    //create an instance of the first popover menu which will be presented-
    //in the view controller unwindsegue
    //this is to allow values (data) to be passed between the parent view-
    //and the popover menu
    ImageViewController *source= (ImageViewController *)[unwindSegue sourceViewController];
    if(source.pickUp == YES){
        [self sendPickupYes];
    }
    else if(source.pickUp == NO){
        [self sendPickupNo];
    }

    
}
- (IBAction)unwindSegueSecond:(UIStoryboardSegue *)unwindSegue {
    //create an instance of the second popover menu which will be presented-
    //in the view controller unwindsegue
    //this is to allow values (data) to be passed between the parent view-
    //and the popover menu
    secondImageViewController *source2 = (secondImageViewController *) [unwindSegue sourceViewController];
    if(source2.confirm == YES){
        [self sendPickupCheckYes];
    }
    else if(source2.confirm == NO){
        [self sendPickupCheckNo];
    }
    
}

//======================================================================
#pragma mark - loadImage
//======================================================================
- (void)loadImage {
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    NSString *picPath = [documentsDirectory stringByAppendingPathComponent:@"map.png"];
    if ([[NSFileManager defaultManager] fileExistsAtPath:picPath]) {
        UIImage *mapImage = [UIImage imageWithContentsOfFile:picPath];
        //NSLog(@"length of map: %lu", (unsigned long)picPath.length);
        UIGraphicsBeginImageContext(mapImage.size);
        CGContextRef context = UIGraphicsGetCurrentContext();
        CGContextRotateCTM(context, DEGREES_RADIANS(5));
        [mapImage drawAtPoint:CGPointMake(0, 0)];
        UIImage *image = UIGraphicsGetImageFromCurrentImageContext();
        UIGraphicsEndImageContext();
        CGRect rect = CGRectMake(890, 1070, 330, 245);
        CGImageRef imageRef = CGImageCreateWithImageInRect([image CGImage], rect);
        UIImage *img = [UIImage imageWithCGImage:imageRef];
        _mapViewImage.image = img;
        //_mapViewImage.transform = CGAffineTransformMakeRotation(0.0);
    }
}

//======================================================================
#pragma mark - Clear Method
//======================================================================
//Simple method that clears all labels on the application when there is-
//no server connection

-(void)clearAll {
    _mapStatusLabel.text = @"map: ";
    _armStatusLabel.text = @"arm: ";
    _baseStatusLabel.text = @"base: ";
    _binStatusLabel.text = @"bin: ";
    _flyStatusLabel.text = @"fly: ";
    _statusLabel.text = @"Server Status: No connection.";
    _mapXpos.text = @"X:";
    _mapYpos.text = @"Y:";
    _mapZpos.text = @"Z:";
    //_armXpos.text = @"X:";
    //_armYpos.text = @"Y:";
    //_armZpos.text = @"Z:";
    _binXpos.text = @"X:";
    _binYpos.text = @"Y:";
    _binZpos.text = @"Z:";
    _flyXpos.text = @"POSE";
    _flyYpos.text = @"POSE";
    _flyZpos.text = @"POSE";

}
//======================================================================
#pragma mark - Animate Pose Methods
//======================================================================

- (void)convertArmVariables: (NSNumber*)xArm with:(NSNumber*)yArm andAngle:(NSNumber*)zArm {
    if([xArm floatValue] < 6.5) {
        _armX = [NSNumber numberWithFloat: ([xArm floatValue]*100)+50];
        _armXpos.text = [NSString stringWithFormat:@"X: %.f", [_armX floatValue]];
    }
    else if ([xArm floatValue] >= 6.5 && [xArm floatValue] < 8.0){
        _armX = [NSNumber numberWithFloat: ([xArm floatValue]*100)+15];
        _armXpos.text = [NSString stringWithFormat:@"X: %.f", [_armX floatValue]];
    }
    if([yArm floatValue] > 0) {
        _armY = [NSNumber numberWithFloat:([yArm floatValue]*-100)+100];
        _armYpos.text = [NSString stringWithFormat:@"Y: %.f", [_armY floatValue]];
    }
    else if([yArm floatValue] > -4.5 && [yArm floatValue] <= 0) {
        _armY = [NSNumber numberWithFloat:([yArm floatValue]*-100)+100];
        _armYpos.text = [NSString stringWithFormat:@"Y: %.f", [_armY floatValue]];
    }
    else if([yArm floatValue] <= -4.5 && [yArm floatValue] <= 0) {
        _armY = [NSNumber numberWithFloat:([yArm floatValue]*-100)+15];
        _armYpos.text = [NSString stringWithFormat:@"Y: %.f", [_armY floatValue]];
    }
    NSString* zArmFormat = [NSString stringWithFormat:@"%.f", [zArm floatValue]];
    _armZ = [NSNumber numberWithFloat:(([zArmFormat floatValue]* M_PI) / 180)];
    _armZpos.text = [NSString stringWithFormat:@"Z: %@", zArmFormat];
    [self moveArmBot:_armBotImage duration:0.3 delay:0 curve:UIViewAnimationOptionCurveEaseOut rotations:0];
    //[self moveArmBot];
}

- (void)convertBinVariables: (NSNumber*)xBin with:(NSNumber*)yBin andAngle:(NSNumber*)zBin {
    if([xBin floatValue] < 6.5) {
        _binX = [NSNumber numberWithFloat: ([xBin floatValue]*100)+50];
        _binXpos.text = [NSString stringWithFormat:@"X: %.f", [_binX floatValue]];
    }
    else if ([xBin floatValue] >= 6.5 && [xBin floatValue] < 8.0){
        _binX = [NSNumber numberWithFloat: ([xBin floatValue]*100)+15];
        _binXpos.text = [NSString stringWithFormat:@"X: %.f", [_binX floatValue]];
    }
    if([yBin floatValue] > 0) {
        _binY = [NSNumber numberWithFloat:([yBin floatValue]*-100)+100];
        _binYpos.text = [NSString stringWithFormat:@"Y: %.f", [_binY floatValue]];
    }
    else if([yBin floatValue] > -4.5 && [yBin floatValue] <= 0) {
        _binY = [NSNumber numberWithFloat:([yBin floatValue]*-100)+100];
        _binYpos.text = [NSString stringWithFormat:@"Y: %.f", [_binY floatValue]];
    }
    else if([yBin floatValue] <= -4.5 && [yBin floatValue] <= 0) {
        _binY = [NSNumber numberWithFloat:([yBin floatValue]*-100)+15];
        _binYpos.text = [NSString stringWithFormat:@"X: %.f", [_binY floatValue]];
    }
    NSString* zbinFormat = [NSString stringWithFormat:@"%.f", [zBin floatValue]];
    _binZ = [NSNumber numberWithFloat:((([zbinFormat floatValue] - 180)* M_PI) / 180)];
    _binZpos.text = [NSString stringWithFormat:@"Z: %@", zbinFormat];
    [self moveBinBot:_binBotImage duration:0.3 delay:0 curve:UIViewAnimationOptionCurveEaseOut rotations:0];
    //[self moveBinBot];
}

- (void)convertMapVariables: (NSNumber*)xMap with:(NSNumber*)yMap andAngle:(NSNumber*)zMap {
    if([xMap floatValue] < 6.5) {
        _mapX = [NSNumber numberWithFloat: ([xMap floatValue]*100)+50];
        _mapXpos.text = [NSString stringWithFormat:@"X: %.f", [_mapX floatValue]];
    }
    else if ([xMap floatValue] >= 6.5 && [xMap floatValue] < 8.0){
        _mapX = [NSNumber numberWithFloat: ([xMap floatValue]*100)+15];
        _mapXpos.text = [NSString stringWithFormat:@"X: %.f", [_mapX floatValue]];
    }
    if([yMap floatValue] > 0) {
        _mapY = [NSNumber numberWithFloat:([yMap floatValue]*-100)+100];
        _mapYpos.text = [NSString stringWithFormat:@"Y: %.f", [_mapY floatValue]];
    }
    else if([yMap floatValue] > -4.5 && [yMap floatValue] <= 0) {
        _mapY = [NSNumber numberWithFloat:([yMap floatValue]*-100)+100];
        _mapYpos.text = [NSString stringWithFormat:@"Y: %.f", [_mapY floatValue]];
    }
    else if([yMap floatValue] <= -4.5 && [yMap floatValue] <= 0) {
        _mapY = [NSNumber numberWithFloat:([yMap floatValue]*-100)+15];
        _mapYpos.text = [NSString stringWithFormat:@"X: %.f", [_mapY floatValue]];
    }
    NSString* zMapFormat = [NSString stringWithFormat:@"%.f", [zMap floatValue]];
    _mapZ = [NSNumber numberWithFloat:(([zMapFormat floatValue]* M_PI) / 180)];
    _mapZpos.text = [NSString stringWithFormat:@"Z: %@", zMap];
    [self moveMapBot:_mapBotImage duration:0.3 delay:0 curve:UIViewAnimationOptionCurveEaseOut rotations:0];
    //[self moveMapBot];
}

@end
