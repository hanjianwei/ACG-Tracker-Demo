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

#import "ACGTSettings.h"

@implementation ACGTSettings

#pragma mark Singleton Methods

static ACGTSettings *sharedSettings = nil;

+ (ACGTSettings*)sharedSettings
{
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        sharedSettings = [[self alloc] init];
    });
    return sharedSettings;
}

- (id)init
{
    if (sharedSettings)
    {
        NSLog(@"You should never create your own ACGTSettings Object!");
        NSLog(@"Use [ACGTSettings sharedSettings] instead.");
    }
    if (self = [super init])
        [self loadSettings];
    return self;
}

#pragma mark File IO

- (void)loadSettings
{
    NSString *settingsPath = [[[NSBundle mainBundle] bundlePath] stringByAppendingPathComponent:@"ACGTSettings.plist"];
    NSDictionary *settingsDict = [NSDictionary dictionaryWithContentsOfFile:settingsPath];
    _videoPreset = [settingsDict[@"videoPreset"] UTF8String][0];
    _camFocalLength = [settingsDict[@"camFocalLength"] doubleValue];
    _camPrincipalX = [settingsDict[@"camPrincipalX"] doubleValue];
    _camPrincipalY = [settingsDict[@"camPrincipalY"] doubleValue];
    
}

@end
