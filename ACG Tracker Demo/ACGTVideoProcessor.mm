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

#import "ACGTVideoProcessor.h"
#import <CoreMotion/CoreMotion.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>
#import "ACGTSettings.h"
#import "BGRAImage.h"
#import "GrayscaleImage.h"
#import "Features/BRISK/BRISKDetection.h"
#import "Features/Feature.h"
#import "Features/TwoViewMatching.h"
#import <vector>
#import <iostream>
#import "BRISKDescription.h"
#import "ACGTARView.h"
#import "ACGTracker.h"
#import <algorithm>

@implementation ACGTVideoProcessor
{
    enum ACGTState
    {
        ACGTSDetectFeatures,
        ACGTSMatchToInset,
        ACGTSTrackPose
    };
    
    ACGTState _currentState;
}

- (id) init
{
    if (self = [super init])
    {
        _motionManager = [[CMMotionManager alloc] init];
        [_motionManager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXArbitraryCorrectedZVertical];
        _captureSession = nil;
        _renderInset = false;
        _currentState = ACGTState::ACGTSDetectFeatures;
        _switchState = false;
    }
    return self;
}

#pragma mark Image Processing

- (CVPixelBufferRef)copyCVPixelBuffer:(const CVPixelBufferRef)other
{
    uint8_t* otherData = (uint8_t*)CVPixelBufferGetBaseAddress(other);
    uint8_t* otherEnd = (uint8_t*)CVPixelBufferGetBaseAddress(other)+CVPixelBufferGetDataSize(other);
    uint8_t* copyData = new uint8_t[CVPixelBufferGetDataSize(other)];
    std::copy(otherData, otherEnd, copyData);
    CVPixelBufferRef copy;
    CVPixelBufferCreateWithBytes(nullptr,
                                 CVPixelBufferGetWidth(other),
                                 CVPixelBufferGetHeight(other),
                                 CVPixelBufferGetPixelFormatType(other),
                                 copyData, CVPixelBufferGetBytesPerRow(other),
                                 nullptr,
                                 nullptr,
                                 nullptr,
                                 &copy);
    return copy;
}

- (void)processImage:(ACGTARViewImageData&)image
{
    CVPixelBufferLockBaseAddress(image.pixelBuffer, 0);
    
    // Get camera intrinsics from settings
    static const ACGT::Intrinsics intrinsics([ACGTSettings sharedSettings].camFocalLength,
                                             [ACGTSettings sharedSettings].camPrincipalX,
                                             [ACGTSettings sharedSettings].camPrincipalY);
    
    _renderInset = false;
    
    CMAcceleration gravity = _motionManager.deviceMotion.gravity;
    
    @synchronized(self)
    {
        switch (_currentState)
        {
            case ACGTSDetectFeatures:
            {
                ACGT::BGRAImage bgraImage((ACGT::BGRAPixel*)CVPixelBufferGetBaseAddress(image.pixelBuffer),(int)CVPixelBufferGetWidth(image.pixelBuffer),(int)CVPixelBufferGetHeight(image.pixelBuffer));
                ACGT::Camera camera(bgraImage.grayscaleImage(),intrinsics);
                camera.extractFeatures(ACGT::Settings::MinDetectionThreshold);
                for(int i=0; i<camera.features().size(); ++i)
                {
                    ACGTARViewColoredVertex v;
                    v.x = camera.features()[i].x();
                    v.y = camera.features()[i].y();
                    v.R = 1.0;
                    v.G = 0.0;
                    v.B = 0.5;
                    image.features.push_back(v);
                }
                
                // Check if camera button was pressed
                if (_switchState)
                {
                    // Camera button was pressed, create inset image
                    _insetCamera = std::move(camera);
                    _insetCamera.extractDescriptors();
                    _insetCamera.gravity() = Eigen::Vector3d(gravity.x,gravity.y,gravity.z);
        
                    _inset.pixelBuffer = [self copyCVPixelBuffer:image.pixelBuffer];
                    for(int i=0; i<_insetCamera.features().size(); ++i)
                    {
                        ACGTARViewColoredVertex v;
                        v.x = _insetCamera.features()[i].x();
                        v.y = _insetCamera.features()[i].y();
                        v.R = 0.5;
                        v.G = 0.0;
                        v.B = 1.0;
                        _inset.features.push_back(v);
                    }
                    
                    [_delegate newCamPositionX:0.0f Y:0.0f];
                   
                    _currentState = ACGTSMatchToInset;
                    _switchState = false;
                }
                
                break;
            }
                
            case ACGTSMatchToInset:
            {
                ACGT::BGRAImage bgraImage((ACGT::BGRAPixel*)CVPixelBufferGetBaseAddress(image.pixelBuffer),(int)CVPixelBufferGetWidth(image.pixelBuffer),(int)CVPixelBufferGetHeight(image.pixelBuffer));
                if (!_switchState)
                {
                    ACGT::Camera camera(bgraImage.grayscaleImage(),intrinsics);
                    camera.extractFeatures(ACGT::Settings::MinDetectionThreshold,true);
                    for(int i=0; i<camera.features().size(); ++i)
                    {
                        ACGTARViewColoredVertex v;
                        v.x = camera.features()[i].x();
                        v.y = camera.features()[i].y();
                        v.R = 1.0;
                        v.G = 0.0;
                        v.B = 0.5;
                        image.features.push_back(v);
                    }
                    
                    // Compute matches to inset
                    std::vector<ACGT::Match> matches = ACGT::TwoViewMatching::matchNNMutual<ACGT::BRISKDescriptor<ACGT::Settings::DescriptorSize>>(_insetCamera.descriptors(), camera.descriptors(), ACGT::Settings::MatchingThreshold);
                    _insetMatches.clear();
                    for (int i=0; i<matches.size(); ++i)
                    {
                        ACGTARViewColoredVertex v;
                        v.x = _insetCamera.features()[matches[i].idx1()].x();
                        v.y = _insetCamera.features()[matches[i].idx1()].y();
                        v.R = 0.5;
                        v.G = 0.0;
                        v.B = 1.0;
                        _insetMatches.push_back(v);
                        
                        v.x = camera.features()[matches[i].idx2()].x();
                        v.y = camera.features()[matches[i].idx2()].y();
                        v.R = 1.0;
                        v.G = 0.0;
                        v.B = 0.5;
                        _insetMatches.push_back(v);
                    }
                    
                    _renderInset = true;
                } else
                {
                    // Camera button was pressed, create map
                    ACGT::BGRAImage bgraInset((ACGT::BGRAPixel*)CVPixelBufferGetBaseAddress(_inset.pixelBuffer),(int)CVPixelBufferGetWidth(_inset.pixelBuffer),(int)CVPixelBufferGetHeight(_inset.pixelBuffer));
                    _tracker.createMap(bgraInset, intrinsics, _insetCamera.gravity(), bgraImage, intrinsics,Eigen::Vector3d(gravity.x,gravity.y,gravity.z));
                    
                    CFRelease(_inset.pixelBuffer);
                    _inset.features.clear();
                    _insetCamera = ACGT::Camera();
                    _insetMatches.clear();
                    
                    _currentState = ACGTSTrackPose;
                    _switchState = false;
                }
                
                break;
            }
                
            case ACGTSTrackPose:
            {
                // Track pose
                const ACGT::BGRAImage bgraImage((ACGT::BGRAPixel*)CVPixelBufferGetBaseAddress(image.pixelBuffer),(int)CVPixelBufferGetWidth(image.pixelBuffer),(int)CVPixelBufferGetHeight(image.pixelBuffer));
                ACGT::SE3 pose;
                NSDate *start = [NSDate date];
                bool success = _tracker.localizeImage(bgraImage, intrinsics, pose);
                NSDate *end = [NSDate date];
                double time = [end timeIntervalSinceDate:start];
                [_delegate updateFrameRate:(float)time];
                if (!success)
                    break;
                // Render reprojected points
                const std::vector<ACGT::Point3d>& points = _tracker.map().points();
                for (const ACGT::Point3d& p : points)
                {
                    Eigen::Vector2d reprojection = p.reproject(pose,intrinsics);
                    if (reprojection.x() >= 0.0 &&
                        reprojection.x() < (float)bgraImage.width() &&
                        reprojection.y() >= 0.0 &&
                        reprojection.y() < (float)bgraImage.height())
                    {
                        ACGTARViewColoredVertex v;
                        v.x = reprojection.x();
                        v.y = (float)bgraImage.height()-1.0f-reprojection.y();
                        v.R = p.color().r();
                        v.G = p.color().g();
                        v.B = p.color().b();
                        image.features.push_back(v);
                    }
                }
                
                Eigen::Vector3d camPosition = pose.C();
                [_delegate newCamPositionX:(float)camPosition.x() Y:(float)-camPosition.z()];

                if (_switchState)
                {
                    ////
                    // Reset
                    _tracker.reset();
                    _currentState = ACGTSDetectFeatures;
                    _switchState = false;
                }
                
                break;
            }
                
            default:
                break;
        }
    }
    
    CVPixelBufferUnlockBaseAddress(image.pixelBuffer, 0);
}

#pragma mark Capture

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
	if ( connection == _videoConnection ) {
        
        CMFormatDescriptionRef formatDescription = CMSampleBufferGetFormatDescription(sampleBuffer);
        
		// Get frame dimensions (for onscreen display) (TODO: Might be erronous on resolution change)
		if (self.videoDimensions.width == 0 && self.videoDimensions.height == 0)
			_videoDimensions = CMVideoFormatDescriptionGetDimensions(formatDescription);
        
		CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
        
		// Synchronously process the pixel buffer.
        __block ACGTARViewImageData image(pixelBuffer);
        [self processImage:image];
		
		
		// Enqueue it for preview.  This is a shallow queue, so if image processing is taking too long,
		// we'll drop this frame for preview (this keeps preview latency low).
		OSStatus err = CMBufferQueueEnqueue(_videoBufferQueue, sampleBuffer);
		if ( !err ) {
			dispatch_async(dispatch_get_main_queue(), ^{
				CMSampleBufferRef sbuf = (CMSampleBufferRef)CMBufferQueueDequeueAndRetain(_videoBufferQueue);
				if (sbuf) {
                    if (_renderInset)
                    {
                        // We send a copy of the inset data for rendering since the rendering code converts image coordinates to OpenGL coordinates which would otherwise happen every frame
                        ACGTARViewImageData insetCopy = _inset;
                        [_delegate imageDataReadyForDisplay:image insetImage:insetCopy matches:_insetMatches];
                    }
                    else
                        [_delegate imageDataReadyForDisplay:image];
					CFRelease(sbuf);
				}
			});
		}
	}
}

- (AVCaptureDevice *)videoDeviceWithPosition:(AVCaptureDevicePosition)position
{
    NSArray *devices = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
    for (AVCaptureDevice *device in devices)
    {
        if ([device position] == position)
            return device;
    }
    return nil;
}

- (BOOL) setupCaptureSession
{
    // Create capture session
    _captureSession = [[AVCaptureSession alloc] init];
    
    AVCaptureDevice *device = [self videoDeviceWithPosition:AVCaptureDevicePositionBack];

    // Create video connection
    AVCaptureDeviceInput *videoIn = [[AVCaptureDeviceInput alloc] initWithDevice:device error:nil];
    if ([_captureSession canAddInput:videoIn])
        [_captureSession addInput:videoIn];
    
	AVCaptureVideoDataOutput *videoOut = [[AVCaptureVideoDataOutput alloc] init];
    // Process only the current frame and discard late frames
	[videoOut setAlwaysDiscardsLateVideoFrames:YES];
    //  videoOut.videoSettings = nil;
	[videoOut setVideoSettings:[NSDictionary dictionaryWithObject:[NSNumber numberWithInt:kCVPixelFormatType_32BGRA] forKey:(id)kCVPixelBufferPixelFormatTypeKey]];
    
	dispatch_queue_t videoCaptureQueue = dispatch_queue_create("Video Capture Queue", DISPATCH_QUEUE_SERIAL);
	[videoOut setSampleBufferDelegate:self queue:videoCaptureQueue];

	if ([_captureSession canAddOutput:videoOut])
		[_captureSession addOutput:videoOut];
	_videoConnection = [videoOut connectionWithMediaType:AVMediaTypeVideo];
    _videoConnection.videoOrientation = AVCaptureVideoOrientationPortrait;
    
    switch ([ACGTSettings sharedSettings].videoPreset) {
        case 'l': // Low setting: 480 x 640
            [_captureSession setSessionPreset:AVCaptureSessionPreset640x480];
            break;
            
        case 'm': // Medium setting: 720 x 1280
            [_captureSession setSessionPreset:AVCaptureSessionPreset1280x720];
            break;
            
        case 'h': // High setting: 1080 x 1920
            [_captureSession setSessionPreset:AVCaptureSessionPreset1920x1080];
            break;
            
        default: // Default to 480 x 640
            NSLog(@"Unsupported video preset: %c",[ACGTSettings sharedSettings].videoPreset);
            NSLog(@"Use preset 480x640");
            [_captureSession setSessionPreset:AVCaptureSessionPreset640x480];
            break;
    }
	return YES;
}

- (void) setupAndStartCaptureSession
{
	// Create a shallow queue for buffers going to the display for preview.
	OSStatus err = CMBufferQueueCreate(kCFAllocatorDefault, 1, CMBufferQueueGetCallbacksForUnsortedSampleBuffers(), &_videoBufferQueue);
	if (err)
		[self showError:[NSError errorWithDomain:NSOSStatusErrorDomain code:err userInfo:nil]];
    
    if ( !_captureSession )
		[self setupCaptureSession];
	
	if ( !_captureSession.isRunning )
		[_captureSession startRunning];
}

- (void) stopAndTearDownCaptureSession
{
    [_captureSession stopRunning];
	_captureSession = nil;
	if (_videoBufferQueue) {
		CFRelease(_videoBufferQueue);
		_videoBufferQueue = nil;
	}
}

- (void) pauseCaptureSession
{
	if ( _captureSession.isRunning )
		[_captureSession stopRunning];
}

- (void) resumeCaptureSession
{
	if ( !_captureSession.isRunning )
		[_captureSession startRunning];
}

#pragma mark IO Handling

- (void)onCameraButtonPressed
{
    @synchronized(self)
    {
        _switchState = true;
    }
}

#pragma mark Error Handling

- (void)showError:(NSError *)error
{
    CFRunLoopPerformBlock(CFRunLoopGetMain(), kCFRunLoopCommonModes, ^(void) {
        UIAlertView *alertView = [[UIAlertView alloc] initWithTitle:[error localizedDescription]
                                                            message:[error localizedFailureReason]
                                                           delegate:nil
                                                  cancelButtonTitle:@"OK"
                                                  otherButtonTitles:nil];
        [alertView show];
    });
}

- (void)dealloc
{
    [self stopAndTearDownCaptureSession];
    [_motionManager stopDeviceMotionUpdates];
    CFRelease(_inset.pixelBuffer);
}

@end
