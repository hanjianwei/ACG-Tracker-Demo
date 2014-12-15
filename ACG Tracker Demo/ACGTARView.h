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

#import <UIKit/UIKit.h>
#import <OpenGLES/EAGL.h>
#import <CoreVideo/CVOpenGLESTextureCache.h>
#import <vector>

typedef struct ACGTARViewColoredVertex
{
    GLfloat x; // floating point x-coordinate of the point in [0.0,imagewidth)
    GLfloat y; // floating point x-coordinate of the point in [0.0,imageheight)
    GLfloat R; // Red color in [0,1]
    GLfloat G; // Green color in [0,1]
    GLfloat B; // Blue color in [0,1]
} ACGTARViewColoredVertex;

typedef struct ACGTARViewImageData
{
    CVImageBufferRef pixelBuffer;
    std::vector<ACGTARViewColoredVertex> features;
    
    ACGTARViewImageData() {};
    ACGTARViewImageData(CVPixelBufferRef pixelBuffer) : pixelBuffer(pixelBuffer) {};
    ACGTARViewImageData(CVPixelBufferRef pixelBuffer, std::vector<ACGTARViewColoredVertex>& features) : pixelBuffer(pixelBuffer), features(features) {};
} CGTARViewImageData;

@interface ACGTARView : UIView
{
    // OpenGL context
    EAGLContext* _glContext;
    
    // Renderbuffer dimension
    int _renderBufferWidth;
	int _renderBufferHeight;
    
    // OpenGL buffers
    GLuint _frameBufferHandle;
	GLuint _colorBufferHandle;
    GLuint _depthBufferHandle;
    
    // GLSL shaders
    GLuint _textureProgram;
    GLuint _pointProgram;
    GLuint _lineProgram;
    
    // OpenGL ES Texture Cache
    CVOpenGLESTextureCacheRef _videoTextureCache;
}

- (void)renderImage:(ACGTARViewImageData&)image;
- (void)renderImage:(ACGTARViewImageData&)image
         insetImage:(ACGTARViewImageData&)inset
            matches:(std::vector<ACGTARViewColoredVertex>&)matches;

@end
