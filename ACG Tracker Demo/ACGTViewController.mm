/*===========================================================================*\
 *                                                                           *
 *  ACG Tracker from the ECCV'14 Paper                                       *
 *  Scalable 6-DOF Localization on Mobile Devices                            *
 *  Copyright (C) 2014 by Computer Graphics Group, RWTH Aachen               *
 *  Author: Sven Middelberg <middelberg@cs.rwth-aachen.de>                   *
 *  www.rwth.graphics                                                        *
 *                                                                           *
 *---------------------------------------------------------------------------*
 *  This file is part of ACG Tracker                                         *
 *                                                                           *
 *  ACG Tracker is free software: you can redistribute it and/or modify      *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  ACG Tracker is distributed in the hope that it will be useful,           *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with ACG Tracker. If not, see <http://www.gnu.org/licenses/>.      *
 *                                                                           *
\*===========================================================================*/

#import "ACGTViewController.h"
#import "ACGTARView.h"
#import <vector>
extern "C"
{
#import "ShaderUtilities.h"
}

@interface ACGTViewController ()

@end

@implementation ACGTViewController

enum {
    attributeVertexLocation,
    attributeColorLocation
};

enum {
    uniformPointSize
};

- (id)init
{
    self = [super init];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)loadView
{
    CGRect mainScreenFrame = [[UIScreen mainScreen] bounds];
    _arView = [[ACGTARView alloc] initWithFrame:mainScreenFrame];
    _toolbar = [[UIToolbar alloc] initWithFrame:CGRectMake(0, [[UIScreen mainScreen] bounds].size.height-44, mainScreenFrame.size.width, 44)];
    _toolbar.barStyle = UIBarStyleDefault;
    UIBarButtonItem *flexibleSpace =  [[UIBarButtonItem alloc] initWithBarButtonSystemItem:UIBarButtonSystemItemFlexibleSpace target:nil action:nil];
    UIBarButtonItem *cameraButton = [[UIBarButtonItem alloc] initWithBarButtonSystemItem:UIBarButtonSystemItemCamera target:self action:@selector(onCameraButtonPressed:)];
    _toolbar.items = @[flexibleSpace,cameraButton,flexibleSpace];
    [_arView addSubview:_toolbar];
    self.view = _arView;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    // Initialize the class responsible for managing AV capture session and asset writer
    _videoProcessor = [[ACGTVideoProcessor alloc] init];
	_videoProcessor.delegate = self;
    
    // Setup and start the capture session
    [_videoProcessor setupAndStartCaptureSession];
    
    // Setup _camPositionView
    EAGLContext *context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    CGRect screenRect = self.view.frame;
    CGFloat screenWidth = screenRect.size.width;
    CGFloat screenHeight = screenRect.size.height;
    CGRect _camPositionViewFrame = CGRectMake(screenWidth/2.0+10.0, screenHeight-screenWidth/2.0+10.0-_toolbar.frame.size.height, screenWidth/2.0-20.0, screenWidth/2.0-20.0);
    _camPositionView = [[GLKView alloc] initWithFrame:_camPositionViewFrame context:context];
    _camPositionView.delegate = self;
    _camPositionView.layer.cornerRadius = 20;
    _camPositionView.layer.masksToBounds = YES;
    _camPositionView.layer.opaque = NO;
    [self.view addSubview:_camPositionView];
    [self.view bringSubviewToFront:_camPositionView];
    
    _vertexBuffer = 0;
    _camPositionProgram = 0;
    
    // Setup Labels
    _frameRateLabel = [self labelWithText:@"" yPosition: (CGFloat) 30.0];
	[self.view addSubview:_frameRateLabel];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

#pragma mark UI

- (UILabel *)labelWithText:(NSString *)text yPosition:(CGFloat)yPosition
{
	CGFloat labelWidth = 200.0;
	CGFloat labelHeight = 40.0;
	CGFloat xPosition = self.view.bounds.size.width - labelWidth - 10;
	CGRect labelFrame = CGRectMake(xPosition, yPosition, labelWidth, labelHeight);
	UILabel *label = [[UILabel alloc] initWithFrame:labelFrame];
	[label setFont:[UIFont systemFontOfSize:36]];
	[label setLineBreakMode:NSLineBreakByWordWrapping];
	[label setTextAlignment:NSTextAlignmentRight];
	[label setTextColor:[UIColor whiteColor]];
	[label setBackgroundColor:[UIColor colorWithRed:0.0 green:0.0 blue:0.0 alpha:0.25]];
	[[label layer] setCornerRadius: 4];
	[label setText:text];
	
	return label;
}

- (void)onCameraButtonPressed:(id)sender
{
    [_videoProcessor onCameraButtonPressed];
}

#pragma mark ACGTVideoProcessorDelegate methods

- (void)imageDataReadyForDisplay:(ACGTARViewImageData&)imageData
{
    [_arView renderImage:imageData];
}
- (void)imageDataReadyForDisplay:(ACGTARViewImageData&)imageData
                      insetImage:(ACGTARViewImageData&)inset
                         matches:(std::vector<ACGTARViewColoredVertex>&)matches
{
    [_arView renderImage:imageData insetImage:inset matches:matches];
}

- (void) newCamPositionX:(float)x Y:(float)y
{
    if (_camPositions.size() == 0)
    {
        ACGTCamPositionData camPositionData;
        camPositionData.x = x;
        camPositionData.y = y;
        camPositionData.R = 0.0;
        camPositionData.G = 1.0;
        camPositionData.B = 1.0;
        _camPositions.push_back(camPositionData);
    } else if (_camPositions.size() == 1)
    {
        //compute norm of (x,y)^T and compute scale
        float dX = x-_camPositions[0].x;
        float dY = y-_camPositions[0].y;
        _mapScale = 0.4/sqrt(dX*dX+dY*dY);
        ACGTCamPositionData camPositionData;
        camPositionData.x = x*_mapScale;
        camPositionData.y = y*_mapScale;
        camPositionData.R = 0.0;
        camPositionData.G = 1.0;
        camPositionData.B = 1.0;
        _camPositions[0].x *= _mapScale;
        _camPositions[0].y *= _mapScale;
        _camPositions.push_back(camPositionData);
        _camPositionCounter = 2;
    } else if (_camPositions.size() < numCamPositionsToDisplay)
    {
        ACGTCamPositionData camPositionData;
        camPositionData.x = x*_mapScale;
        camPositionData.y = y*_mapScale;
        camPositionData.R = 0.0;
        camPositionData.G = 1.0;
        camPositionData.B = 1.0;
        _camPositions.push_back(camPositionData);
        ++_camPositionCounter;
    } else
    {
        ACGTCamPositionData camPositionData;
        camPositionData.x = x*_mapScale;
        camPositionData.y = y*_mapScale;
        camPositionData.R = 0.0;
        camPositionData.G = 1.0;
        camPositionData.B = 1.0;
        _camPositions[_camPositionCounter%numCamPositionsToDisplay] = camPositionData;
        ++_camPositionCounter;
    }
    
    std::vector<ACGTCamPositionData> centeredCamPositions(_camPositions);
    for (ACGTCamPositionData& centered : centeredCamPositions)
    {
        centered.x -= x*_mapScale;
        centered.y -= y*_mapScale;
    }
  
    [EAGLContext setCurrentContext:_camPositionView.context];
    
    glBindBuffer(GL_ARRAY_BUFFER, _vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, centeredCamPositions.size()*sizeof(ACGTCamPositionData), centeredCamPositions.data(), GL_STATIC_DRAW);
    [_camPositionView display];
}

- (void)updateFrameRate:(float)frameProcessingTime
{
    _lastFrameProcessingTimes.push_back(frameProcessingTime);
    if (_lastFrameProcessingTimes.size() > 30)
        _lastFrameProcessingTimes.pop_front();
    
    // Compute average framerate
    float avgProcessingTime = 0.0;
    for (const float& t : _lastFrameProcessingTimes)
        avgProcessingTime += t;
    avgProcessingTime /= (float)_lastFrameProcessingTimes.size();
    [_frameRateLabel performSelectorOnMainThread:@selector(setText:) withObject:[NSString stringWithFormat:@"%.1f fps",1.0/avgProcessingTime] waitUntilDone:NO];
}

#pragma mark OpenGL related stuff

- (const GLchar *)readFile:(NSString *)name
{
    NSString *path;
    const GLchar *source;
    
    path = [[NSBundle mainBundle] pathForResource:name ofType: nil];
    source = (GLchar *)[[NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil] UTF8String];
    
    return source;
}

- (bool) setupGL
{
    [EAGLContext setCurrentContext:_camPositionView.context];
    
    bool success = true;
    
    glGenBuffers(1, &_vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, _vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, _camPositions.size()*sizeof(ACGTCamPositionData), _camPositions.data(), GL_STATIC_DRAW);
    
    // Load vertex and fragment shaders
    const GLchar *vertSrcPointPrg = [self readFile:@"point.vsh"];
    const GLchar *fragSrcPointPrg = [self readFile:@"point.fsh"];
    
    // attributes
    const GLint attribLocationPointPrg[2] = {
        attributeVertexLocation, attributeColorLocation
    };
    const GLchar *attribNamePointPrg[2] = {
        "position", "color"
    };
    
    // uniforms
    GLint uniformLocationPointPrg[1] = {
        uniformPointSize
    };
    const GLchar *uniformNamePointPrg[1] = {
        "pointsize"
    };
    
    glueCreateProgram(vertSrcPointPrg, fragSrcPointPrg,
                      2, (const GLchar **)&attribNamePointPrg[0], attribLocationPointPrg,
                      1, (const GLchar **)&uniformNamePointPrg[0], uniformLocationPointPrg,
                      &_camPositionProgram);
    
    if (!_camPositionProgram)
        success = false;
    
    return success;
}

- (void) tearDownGL
{
    
    [EAGLContext setCurrentContext:_camPositionView.context];
    
    glDeleteBuffers(1, &_vertexBuffer);
    _vertexBuffer = 0;
}

#pragma mark - GLKViewDelegate methods

- (void) glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    [EAGLContext setCurrentContext:_camPositionView.context];
    
    if (_camPositionProgram == 0) {
		bool success = [self setupGL];
		if ( !success )
			NSLog(@"Problem initializing OpenGL buffers.");
	}

    glClearColor(0.0, 0.0, 0.0, 0.25);
    glClear(GL_COLOR_BUFFER_BIT);
    
    glBindBuffer(GL_ARRAY_BUFFER, _vertexBuffer);
    
    glUseProgram(_camPositionProgram);
    
    glVertexAttribPointer(attributeVertexLocation, 2, GL_FLOAT, 0, sizeof(ACGTCamPositionData), 0);
    glEnableVertexAttribArray(attributeVertexLocation);
    glVertexAttribPointer(attributeColorLocation, 4, GL_FLOAT, 0, sizeof(ACGTCamPositionData), (void*)(2*sizeof(float)));
    glEnableVertexAttribArray(attributeColorLocation);
    
    glUniform1f(glGetUniformLocation(_camPositionProgram, "pointsize"), 5.0);
    
    glDrawArrays(GL_POINTS, 0, _camPositions.size());
}

- (void) dealloc
{
    [self tearDownGL];
}

@end
