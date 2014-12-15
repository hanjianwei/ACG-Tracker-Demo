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

#import <Foundation/Foundation.h>
#import <CoreMotion/CoreMotion.h>
#import <AVFoundation/AVFoundation.h>
#import "ACGTVideoProcessorDelegate.h"
#import <CoreMedia/CoreMedia.h>
#import <vector>
#import "ACGTARView.h"
#import "ACGTracker.h"

@interface ACGTVideoProcessor : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
{
    // Used to get device gravity
    CMMotionManager *_motionManager;
    
    // Manages camera input
    AVCaptureSession *_captureSession;
	AVCaptureConnection *_videoConnection;
    CMBufferQueueRef _videoBufferQueue;
    
    // Render data for Inset image
    ACGTARViewImageData _inset;
    std::vector<ACGTARViewColoredVertex> _insetMatches;
    bool _renderInset;
    ACGT::Camera _insetCamera;
    
    bool _switchState;
    
    // The tracker
    ACGT::ACGTracker _tracker;
}
@property (strong, nonatomic) id<ACGTVideoProcessorDelegate> delegate;
@property (readonly) CMVideoDimensions videoDimensions;

- (void) onCameraButtonPressed;

- (void) setupAndStartCaptureSession;
- (void) stopAndTearDownCaptureSession;

@end
