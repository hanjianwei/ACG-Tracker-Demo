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

#import "ACGTARView.h"
#import <QuartzCore/CAEAGLLayer.h>
#import <OpenGLES/EAGL.h>
extern "C"
{
#import "ShaderUtilities.h"
}

@implementation ACGTARView

enum {
    attributeVertexLocation,
    attributeColorLocation,
    attributeTextureLocation
};

enum {
    uniformPointSize
};

+ (Class)layerClass
{
    return [CAEAGLLayer class];
}

- (id)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    if (self)
    {
        // Use 2x scale factor on Retina displays.
		self.contentScaleFactor = [[UIScreen mainScreen] scale];
        
        // Initialize OpenGL ES 2
        CAEAGLLayer* eaglLayer = (CAEAGLLayer *)self.layer;
        eaglLayer.opaque = YES;
        eaglLayer.drawableProperties = [NSDictionary dictionaryWithObjectsAndKeys:
                                        [NSNumber numberWithBool:NO], kEAGLDrawablePropertyRetainedBacking,
                                        kEAGLColorFormatRGBA8, kEAGLDrawablePropertyColorFormat,
                                        nil];
        _glContext = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
        if (!_glContext || ![EAGLContext setCurrentContext:_glContext])
        {
            NSLog(@"Could not setup OpenGL Context");
            return nil;
        }
    }
    return self;
}

#pragma mark File IO (for Shaders)

- (const GLchar *)readFile:(NSString *)name
{
    NSString *path;
    const GLchar *source;
    
    path = [[NSBundle mainBundle] pathForResource:name ofType: nil];
    source = (GLchar *)[[NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil] UTF8String];
    
    return source;
}

#pragma mark Cropping

- (CGRect)textureSamplingRectForCroppingTextureWithAspectRatio:(CGSize)textureAspectRatio toAspectRatio:(CGSize)croppingAspectRatio
{
	CGRect normalizedSamplingRect = CGRectZero;
	CGSize cropScaleAmount = CGSizeMake(croppingAspectRatio.width / textureAspectRatio.width, croppingAspectRatio.height / textureAspectRatio.height);
	CGFloat maxScale = fmax(cropScaleAmount.width, cropScaleAmount.height);
	CGSize scaledTextureSize = CGSizeMake(textureAspectRatio.width * maxScale, textureAspectRatio.height * maxScale);

	if ( cropScaleAmount.height > cropScaleAmount.width )
    {
		normalizedSamplingRect.size.width = croppingAspectRatio.width / scaledTextureSize.width;
		normalizedSamplingRect.size.height = 1.0;
	} else
    {
		normalizedSamplingRect.size.height = croppingAspectRatio.height / scaledTextureSize.height;
		normalizedSamplingRect.size.width = 1.0;
	}
	// Center crop
	normalizedSamplingRect.origin.x = (1.0 - normalizedSamplingRect.size.width)/2.0;
	normalizedSamplingRect.origin.y = (1.0 - normalizedSamplingRect.size.height)/2.0;
    
	return normalizedSamplingRect;
}

#pragma mark OpenGL

- (BOOL)initializeBuffers
{
	BOOL success = YES;
	
	glDisable(GL_DEPTH_TEST);
    
    glGenFramebuffers(1, &_frameBufferHandle);
    glBindFramebuffer(GL_FRAMEBUFFER, _frameBufferHandle);
    
    glGenRenderbuffers(1, &_colorBufferHandle);
    glBindRenderbuffer(GL_RENDERBUFFER, _colorBufferHandle);
    
    [_glContext renderbufferStorage:GL_RENDERBUFFER fromDrawable:(CAEAGLLayer *)self.layer];
    
	glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &_renderBufferWidth);
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &_renderBufferHeight);
    
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, _colorBufferHandle);
    
    glGenRenderbuffers(1, &_depthBufferHandle);
    glBindRenderbuffer(GL_RENDERBUFFER, _depthBufferHandle);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, _renderBufferWidth, _renderBufferHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, _depthBufferHandle);
    
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        NSLog(@"Could not create framebuffer");
		success = NO;
	}
    
    //  Create a new CVOpenGLESTexture cache
    CVReturn err = CVOpenGLESTextureCacheCreate(kCFAllocatorDefault, NULL, CFBridgingRelease((__bridge void*) _glContext), NULL, &_videoTextureCache);
    if (err) {
        NSLog(@"Error at CVOpenGLESTextureCacheCreate %d", err);
        success = NO;
    }
    
    // Load vertex and fragment shaders
    
    // Texture Program (Renders image as textured square)
    const GLchar *vertSrcTexturePrg = [self readFile:@"texture.vsh"];
    const GLchar *fragSrcTexturePrg = [self readFile:@"texture.fsh"];
    
    // attributes
    const GLint attribLocationTexturePrg[2] = {
        attributeVertexLocation, attributeTextureLocation
    };
    const GLchar *attribNameTexturePrg[2] = {
        "position", "textureCoordinate"
    };
    
    glueCreateProgram(vertSrcTexturePrg, fragSrcTexturePrg,
                      2, (const GLchar **)&attribNameTexturePrg[0], attribLocationTexturePrg,
                      0, 0, 0, // we don't need to get uniform locations
                      &_textureProgram);
 
    if (!_textureProgram)
        success = NO;
    
    // Point program (renders colored points)
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
                      &_pointProgram);
    
    if (!_pointProgram)
        success = NO;
    
    // Line program (renders colored lines)
    const GLchar *vertSrcLinePrg = [self readFile:@"line.vsh"];
    const GLchar *fragSrcLinePrg = [self readFile:@"line.fsh"];
    
    // attributes
    const GLint attribLocationLinePrg[2] = {
        attributeVertexLocation, attributeColorLocation
    };
    const GLchar *attribNameLinePrg[2] = {
        "position", "color"
    };
    
    glueCreateProgram(vertSrcLinePrg, fragSrcLinePrg,
                      2, (const GLchar **)&attribNameLinePrg[0], attribLocationLinePrg,
                      0, 0, 0, // we don't need to get uniform locations
                      &_lineProgram);
    
    if (!_lineProgram)
        success = NO;

    return success;
}


- (void)renderWithSquareVertices:(const GLfloat*)squareVertices textureVertices:(const GLfloat*)textureVertices
{
    // Use shader program.
    glUseProgram(_textureProgram);
    
    // Update attribute values.
	glVertexAttribPointer(attributeVertexLocation, 2, GL_FLOAT, 0, 0, squareVertices);
	glEnableVertexAttribArray(attributeVertexLocation);
	glVertexAttribPointer(attributeTextureLocation, 2, GL_FLOAT, 0, 0, textureVertices);
	glEnableVertexAttribArray(attributeTextureLocation);
    
    // Update uniform values if there are any
    
    // Validate program before drawing. This is a good check, but only really necessary in a debug build.
    // DEBUG macro must be defined in your debug configurations if that's not already the case.
#if defined(DEBUG)
    if (glueValidateProgram(_textureProgram) == 0) {
        NSLog(@"Failed to validate program: %d", _textureProgram);
        return;
    }
#endif
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
}

- (void)renderPoints:(const std::vector<ACGTARViewColoredVertex>&)points withSize:(const float&)pointsize
{
    // Use shader program.
    glUseProgram(_pointProgram);
    
    // Update attribute values.
	glVertexAttribPointer(attributeVertexLocation, 2, GL_FLOAT, 0, sizeof(ACGTARViewColoredVertex), points.data());
	glEnableVertexAttribArray(attributeVertexLocation);
	glVertexAttribPointer(attributeColorLocation, 3, GL_FLOAT, 0, sizeof(ACGTARViewColoredVertex), &points.data()[0].R);
	glEnableVertexAttribArray(attributeColorLocation);
    
    // Update uniform
    glUniform1f(glGetUniformLocation(_pointProgram, "pointsize"), pointsize);
    
    // Validate program before drawing. This is a good check, but only really necessary in a debug build.
    // DEBUG macro must be defined in your debug configurations if that's not already the case.
#if defined(DEBUG)
    if (glueValidateProgram(_pointProgram) == 0) {
        NSLog(@"Failed to validate program: %d", _lineProgram);
        return;
    }
#endif
    
	glDrawArrays(GL_POINTS, 0, points.size());
}

- (void)renderLines:(const std::vector<ACGTARViewColoredVertex>&)lines
{
    // Use shader program.
    glUseProgram(_lineProgram);
    
    // Update attribute values.
	glVertexAttribPointer(attributeVertexLocation, 2, GL_FLOAT, 0, sizeof(ACGTARViewColoredVertex), lines.data());
	glEnableVertexAttribArray(attributeVertexLocation);
	glVertexAttribPointer(attributeColorLocation, 3, GL_FLOAT, 0, sizeof(ACGTARViewColoredVertex),&lines.data()[0].R);
	glEnableVertexAttribArray(attributeColorLocation);
    
    // Update uniform values if there are any
    
    // Validate program before drawing. This is a good check, but only really necessary in a debug build.
    // DEBUG macro must be defined in your debug configurations if that's not already the case.
#if defined(DEBUG)
    if (glueValidateProgram(_lineProgram) == 0) {
        NSLog(@"Failed to validate program: %d", _lineProgram);
        return;
    }
#endif
	glDrawArrays(GL_LINES, 0, 2*(lines.size()/2));
}

- (void)renderImage:(ACGTARViewImageData&)image
{
    [EAGLContext setCurrentContext:_glContext];
	if (_frameBufferHandle == 0)
    {
		BOOL success = [self initializeBuffers];
		if ( !success ) {
			NSLog(@"Could not initialize OpenGL Buffers");
		}
	}
    
    if (_videoTextureCache == NULL)
        return;
	
    // Create a CVOpenGLESTexture from the CVImageBuffer
	size_t frameWidth = CVPixelBufferGetWidth(image.pixelBuffer);
	size_t frameHeight = CVPixelBufferGetHeight(image.pixelBuffer);
    CVOpenGLESTextureRef texture = NULL;
    CVReturn err = CVOpenGLESTextureCacheCreateTextureFromImage(kCFAllocatorDefault,
                                                                _videoTextureCache,
                                                                image.pixelBuffer,
                                                                NULL,
                                                                GL_TEXTURE_2D,
                                                                GL_RGBA,
                                                                frameWidth,
                                                                frameHeight,
                                                                GL_BGRA,
                                                                GL_UNSIGNED_BYTE,
                                                                0,
                                                                &texture);
    
    
    if (!texture || err) {
        NSLog(@"CVOpenGLESTextureCacheCreateTextureFromImage failed (error: %d)", err);
        return;
    }
    
	glBindTexture(CVOpenGLESTextureGetTarget(texture), CVOpenGLESTextureGetName(texture));
    
    // Set texture parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    glBindFramebuffer(GL_FRAMEBUFFER, _frameBufferHandle);
    
    // Set the view port to the entire view
    glViewport(0, 0, _renderBufferWidth, _renderBufferHeight);
    static const GLfloat squareVertices[] = {
        -1.0f, -1.0f,
        1.0f, -1.0f,
        -1.0f,  1.0f,
        1.0f,  1.0f,
    };
    
	// The texture vertices are set up such that we flip the texture vertically.
	// This is so that our top left origin buffers match OpenGL's bottom left texture coordinate system.
	CGRect textureSamplingRect = [self textureSamplingRectForCroppingTextureWithAspectRatio:CGSizeMake(frameWidth, frameHeight) toAspectRatio:self.bounds.size];
	GLfloat textureVertices[] = {
		CGRectGetMinX(textureSamplingRect), CGRectGetMaxY(textureSamplingRect),
		CGRectGetMaxX(textureSamplingRect), CGRectGetMaxY(textureSamplingRect),
		CGRectGetMinX(textureSamplingRect), CGRectGetMinY(textureSamplingRect),
		CGRectGetMaxX(textureSamplingRect), CGRectGetMinY(textureSamplingRect),
	};
    
    // Draw the texture on the screen
    [self renderWithSquareVertices:squareVertices textureVertices:textureVertices];
    
    // Convert image to openGL coordinates
    const double xFactor = 2.0/((CGRectGetMaxX(textureSamplingRect)-CGRectGetMinX(textureSamplingRect))*frameWidth);
    const double yFactor = -2.0/((CGRectGetMaxY(textureSamplingRect)-CGRectGetMinY(textureSamplingRect))*frameHeight); // y needs to be flipped
    
    for (int i=0; i<image.features.size(); ++i)
    {
        image.features[i].x = xFactor*(image.features[i].x-(GLfloat)frameWidth/2.0);
        image.features[i].y = yFactor*(image.features[i].y-(GLfloat)frameHeight/2.0);
    }
    
    // Render points
    [self renderPoints:image.features withSize:5.0];
    
    // Present
    glBindRenderbuffer(GL_RENDERBUFFER, _colorBufferHandle);
    [_glContext presentRenderbuffer:GL_RENDERBUFFER];
    
    // Flush the CVOpenGLESTexture cache and release the texture
    CVOpenGLESTextureCacheFlush(_videoTextureCache, 0);
    CFRelease(texture);
}

- (void)renderImage:(ACGTARViewImageData&)image
         insetImage:(ACGTARViewImageData&)inset
            matches:(std::vector<ACGTARViewColoredVertex>&)matches
{
    [EAGLContext setCurrentContext:_glContext];
	if (_frameBufferHandle == 0)
    {
		BOOL success = [self initializeBuffers];
		if ( !success ) {
			NSLog(@"Could not initialize OpenGL Buffers");
		}
	}
    
    if (_videoTextureCache == NULL)
        return;
	
    // Create a CVOpenGLESTexture from the CVImageBuffer
	size_t frameWidth = CVPixelBufferGetWidth(image.pixelBuffer);
	size_t frameHeight = CVPixelBufferGetHeight(image.pixelBuffer);
    CVOpenGLESTextureRef texture = NULL;
    CVReturn err = CVOpenGLESTextureCacheCreateTextureFromImage(kCFAllocatorDefault,
                                                                _videoTextureCache,
                                                                image.pixelBuffer,
                                                                NULL,
                                                                GL_TEXTURE_2D,
                                                                GL_RGBA,
                                                                frameWidth,
                                                                frameHeight,
                                                                GL_BGRA,
                                                                GL_UNSIGNED_BYTE,
                                                                0,
                                                                &texture);
    
    
    if (!texture || err) {
        NSLog(@"CVOpenGLESTextureCacheCreateTextureFromImage failed (error: %d)", err);
        return;
    }
    
	glBindTexture(CVOpenGLESTextureGetTarget(texture), CVOpenGLESTextureGetName(texture));
    
    // Set texture parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    glBindFramebuffer(GL_FRAMEBUFFER, _frameBufferHandle);
    
    // Set the view port to the entire view
    glViewport(0, 0, _renderBufferWidth, _renderBufferHeight);
    static const GLfloat squareVertices[] = {
        -1.0f, -1.0f,
        1.0f, -1.0f,
        -1.0f,  1.0f,
        1.0f,  1.0f,
    };
    
	// The texture vertices are set up such that we flip the texture vertically.
	// This is so that our top left origin buffers match OpenGL's bottom left texture coordinate system.
	CGRect textureSamplingRect = [self textureSamplingRectForCroppingTextureWithAspectRatio:CGSizeMake(frameWidth, frameHeight) toAspectRatio:self.bounds.size];
	GLfloat textureVertices[] = {
		CGRectGetMinX(textureSamplingRect), CGRectGetMaxY(textureSamplingRect),
		CGRectGetMaxX(textureSamplingRect), CGRectGetMaxY(textureSamplingRect),
		CGRectGetMinX(textureSamplingRect), CGRectGetMinY(textureSamplingRect),
		CGRectGetMaxX(textureSamplingRect), CGRectGetMinY(textureSamplingRect),
	};
    
    // Draw the texture on the screen
    [self renderWithSquareVertices:squareVertices textureVertices:textureVertices];
    
    // Flush the CVOpenGLESTexture cache and release the texture
    CVOpenGLESTextureCacheFlush(_videoTextureCache, 0);
    CFRelease(texture);
    
    // Convert image to openGL coordinates
    const double xFactor = 2.0/((CGRectGetMaxX(textureSamplingRect)-CGRectGetMinX(textureSamplingRect))*frameWidth);
    const double yFactor = -2.0/((CGRectGetMaxY(textureSamplingRect)-CGRectGetMinY(textureSamplingRect))*frameHeight); // y needs to be flipped
    
    for (int i=0; i<image.features.size(); ++i)
    {
        image.features[i].x = xFactor*(image.features[i].x-(GLfloat)frameWidth/2.0);
        image.features[i].y = yFactor*(image.features[i].y-(GLfloat)frameHeight/2.0);
    }
    
    // Render points
    [self renderPoints:image.features withSize:7.5];
    
    // Render inset
    
    // Create a CVOpenGLESTexture from the CVImageBuffer
	size_t insetWidth = CVPixelBufferGetWidth(inset.pixelBuffer);
	size_t insetHeight = CVPixelBufferGetHeight(inset.pixelBuffer);
    texture = NULL;
    err = CVOpenGLESTextureCacheCreateTextureFromImage(kCFAllocatorDefault,
                                                                _videoTextureCache,
                                                                inset.pixelBuffer,
                                                                NULL,
                                                                GL_TEXTURE_2D,
                                                                GL_RGBA,
                                                                insetWidth,
                                                                insetHeight,
                                                                GL_BGRA,
                                                                GL_UNSIGNED_BYTE,
                                                                0,
                                                                &texture);
    
    
    if (!texture || err) {
        NSLog(@"CVOpenGLESTextureCacheCreateTextureFromImage failed (error: %d)", err);
        return;
    }
    
	glBindTexture(CVOpenGLESTextureGetTarget(texture), CVOpenGLESTextureGetName(texture));
    
    // Set texture parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    const float insetScreenHeight = 0.35;
    const float insetScreenWidth = insetScreenHeight*((float)insetWidth/(float)insetHeight)/((float)_renderBufferWidth/(float)_renderBufferHeight);
    const float insetOffset = 0.0;
    const float insetXOrigin = insetOffset;
    const float insetYOrigin = 1.0-insetOffset-insetScreenHeight;
    GLfloat insetSquareVertices[] = {
                    0.0f, 0.0f,
        insetScreenWidth, 0.0f,
                    0.0f, insetScreenHeight,
        insetScreenWidth, insetScreenHeight,
    };
    for (int i=0; i<4; ++i)
    {
        insetSquareVertices[2*i] += insetXOrigin;
        insetSquareVertices[2*i+1] += insetYOrigin;
        
        insetSquareVertices[2*i] *= 2.0;
        insetSquareVertices[2*i+1] *= 2.0;
        
        insetSquareVertices[2*i] -= 1.0;
        insetSquareVertices[2*i+1] -= 1.0;
    }
    
    const float insetSquareMinX = insetSquareVertices[0];
    const float insetSquareMaxX = insetSquareVertices[2];
    const float insetSquareMinY = insetSquareVertices[1];
    const float insetSquareMaxY = insetSquareVertices[5];
    
	// The texture vertices are set up such that we flip the texture vertically.
	// This is so that our top left origin buffers match OpenGL's bottom left texture coordinate system.
	GLfloat insetTextureVertices[] = {
		0.0f, 1.0f,
        1.0f, 1.0f,
        0.0f, 0.0f,
        1.0f, 0.0f,
	};
    
    // Draw the texture on the screen
    [self renderWithSquareVertices:insetSquareVertices textureVertices:insetTextureVertices];
    
    // Flush the CVOpenGLESTexture cache and release the texture
    CVOpenGLESTextureCacheFlush(_videoTextureCache, 0);
    CFRelease(texture);
    
    // Draw inset features
    
    for (int i=0; i<inset.features.size(); ++i)
    {
        inset.features[i].x = inset.features[i].x/(GLfloat)insetWidth*insetScreenWidth+insetXOrigin;
        inset.features[i].y = (1.0-inset.features[i].y/(GLfloat)insetHeight)*insetScreenHeight+insetYOrigin;
        
        inset.features[i].x *= 2.0;
        inset.features[i].y *= 2.0;
        
        inset.features[i].x -= 1.0;
        inset.features[i].y -= 1.0;
    }
    
    // Render points
    [self renderPoints:inset.features withSize:7.5*std::min(insetScreenWidth, insetScreenHeight)];
    
    // Render Matches
    int numValidMatches = 0; // A match is invalid if the corresponding images's coordinate is in the area in which the inset is rendered, so don't render these matches
    for (int i=0; i<matches.size()/2; ++i)
    {
        matches[2*i].x = matches[2*i].x/(GLfloat)insetWidth*insetScreenWidth+insetXOrigin;
        matches[2*i].y = (1.0-matches[2*i].y/(GLfloat)insetHeight)*insetScreenHeight+insetYOrigin;
        
        matches[2*i].x *= 2.0;
        matches[2*i].y *= 2.0;
        
        matches[2*i].x -= 1.0;
        matches[2*i].y -= 1.0;
        
        matches[2*i+1].x = xFactor*(matches[2*i+1].x-(GLfloat)frameWidth/2.0);
        matches[2*i+1].y = yFactor*(matches[2*i+1].y-(GLfloat)frameHeight/2.0);
        
        if (matches[2*i+1].x >= insetSquareMinX && matches[2*i+1].x <= insetSquareMaxX && matches[2*i+1].y >= insetSquareMinY && matches[2*i+1].y <= insetSquareMaxY)
            continue;
        
        matches[2*numValidMatches] = matches[2*i];
        matches[2*numValidMatches+1] = matches[2*i+1];
        numValidMatches++;
    }
    matches.resize(2*numValidMatches);
    [self renderLines:matches];
    
    // Present
    glBindRenderbuffer(GL_RENDERBUFFER, _colorBufferHandle);
    [_glContext presentRenderbuffer:GL_RENDERBUFFER];
}

#pragma mark Deallocation

- (void)dealloc
{
	if (_frameBufferHandle) {
        glDeleteFramebuffers(1, &_frameBufferHandle);
        _frameBufferHandle = 0;
    }
	
    if (_colorBufferHandle) {
        glDeleteRenderbuffers(1, &_colorBufferHandle);
        _colorBufferHandle = 0;
    }
	
    if (_depthBufferHandle) {
        glDeleteRenderbuffers(1, &_depthBufferHandle);
        _depthBufferHandle = 0;
    }
    
    if (_textureProgram) {
        glDeleteProgram(_textureProgram);
        _textureProgram = 0;
    }
	
    if (_videoTextureCache) {
        CFRelease(_videoTextureCache);
        _videoTextureCache = 0;
    }
}

@end
