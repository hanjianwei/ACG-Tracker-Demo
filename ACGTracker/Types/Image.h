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

#ifndef __ACG_Tracker_Demo__Image__
#define __ACG_Tracker_Demo__Image__

#include <vector>
#include <Eigen/Core>
#include <cassert>

namespace ACGT {
    
    class ImageRect
    {
        
    public:
        
        ImageRect(const Eigen::Vector2i& origin, const unsigned int& width, const unsigned int& height) : _origin(origin), _width(width), _height(height) {};
        ImageRect(const int& originX, const int& originY, const unsigned int& width, const unsigned int& height) : _origin(Eigen::Vector2i(originX,originY)), _width(width), _height(height) {};
        
        Eigen::Vector2i& origin()
        {
            return _origin;
        }
        
        unsigned int& width()
        {
            return _width;
        }
        
        unsigned int& height()
        {
            return _height;
        }
        
        const Eigen::Vector2i& origin() const
        {
            return _origin;
        }
        
        const unsigned int& width() const
        {
            return _width;
        }
        
        const unsigned int& height() const
        {
            return _height;
        }
        
    private:
        
        Eigen::Vector2i _origin;
        unsigned int _width;
        unsigned int _height;
    };
    
    template < typename Pixel >
    class Image
    {
        
    public:
        
        // Default constructor, initializes empty image
        Image()
        {
            _data = nullptr;
            _width = 0;
            _height = 0;
            _owning = true;
        }
        
        // Copy constructor that creates a deep copy B of an image A.
        // New memory is allocated for B's _data array and the contents of A._data are copied to B._data.
        // Since B allocates new memory for the pixel data, B will be owning.
        Image(const Image& other)
        {
            _width = other._width;
            _height = other._height;
            _data = new Pixel[other._width*other._height];
            std::copy(other._data, other._data+_width*_height, _data);
            _owning = true;
        }
        
        // Move constructor that creates an image B that takes ownership of an image A.
        // After the call, A's and B's _data arrays point to the same memory.
        // If A is a non-owning image, B will also be non-owning.
        // If A is an owning image, B will take the ownership of A leaving A non-owning and B owning.
        Image(Image&& other)
        {
            _width = other._width;
            _height = other._height;
            _data = other._data;
            _owning = false;
            if (other._owning)
            {
                _owning = true;
                other._owning = false;
            }
        }
        
        // Copy assignment operator that performs a deep copy
        Image& operator=(const Image& other)
        {
            _width = other._width;
            _height = other._height;
            _data = new Pixel[other._width*other._height];
            std::copy(other._data, other._data+_width*_height, _data);
            _owning = true;
            return *this;
        }
        
        // Move assignment operator that swaps the contents of both images (including ownership)
        Image& operator=(Image&& other)
        {
            std::swap(_width, other._width);
            std::swap(_height, other._height);
            std::swap(_data, other._data);
            std::swap(_owning, other._owning);
            return *this;
        }
        
        // Constructor that creates an image B from raw pixel data.
        // After the call, B's _data array and data point to the same memory.
        // B will be a non-owning image, thus, the caller is responsible for the deallocation of data.
        Image(Pixel *data, const int& width, const int& height)
        {
            _width = width;
            _height = height;
            _data = data;
            _owning = false;
        }
        
        // Constructor that creates an image B of size width x height.
        // This constructor allocates space for width*height pixels.
        // B will be an owning image.
        Image(const int& width, const int& height)
        {
            _data = new Pixel[width*height]();
            _width = width;
            _height = height;
            _owning = true;
        }
        
        // Destructor
        ~Image()
        {
            if (_owning)
                delete[] _data;
        }
        
        // Getter for pixel rows
        Pixel* operator [](const int& row)
        {
            assert(row < _height);
            return _data+row*_width;
        }
        
        const Pixel* operator [](const int& row) const
        {
            assert(row < _height);
            return _data+row*_width;
        }
        
        // Getters for image dimensions
        const int& width() const
        {
            return _width;
        }
        
        const int& height() const
        {
            return _height;
        }
        
        /**
         * Subdivides the image in 16 rectangles (4x4 grid).
         * This is used for multithreaded image processing.
         * The indices of these rectangles are as follows:
         *
         *  0  1  2  3
         *  4  5  6  7
         *  8  9 10 11
         * 12 13 14 15
         *
         * If an index outside of the range [1..15] is passed to the function, the rectangle covering the whole image is returned.
         **/
        
        inline ImageRect rectangle(const int& idx)
        {
            switch (idx)
            {
                case 0:
                    return ImageRect(0, 0, _width/4, _height/4);
                    
                case 1:
                    return ImageRect(_width/4, 0, _width/4, _height/4);
                    
                case 2:
                    return ImageRect(2*_width/4, 0, _width/4, _height/4);
                    
                case 3:
                    return ImageRect(3*_width/4, 0, _width/4, _height/4);
                    
                case 4:
                    return ImageRect(0, _height/4, _width/4, _height/4);
                    
                case 5:
                    return ImageRect(1*_width/4, _height/4, _width/4, _height/4);
                    
                case 6:
                    return ImageRect(2*_width/4, _height/4, _width/4, _height/4);
                    
                case 7:
                    return ImageRect(3*_width/4, _height/4, _width/4, _height/4);
                    
                case 8:
                    return ImageRect(0, 2*_height/4, _width/4, _height/4);
                    
                case 9:
                    return ImageRect(_width/4, 2*_height/4, _width/4, _height/4);
                    
                case 10:
                    return ImageRect(2*_width/4, 2*_height/4, _width/4, _height/4);
                    
                case 11:
                    return ImageRect(3*_width/4, 2*_height/4, _width/4, _height/4);
                    
                case 12:
                    return ImageRect(0, 3*_height/4, _width/4, _height/4);
                    
                case 13:
                    return ImageRect(_width/4, 3*_height/4, _width/4, _height/4);
                    
                case 14:
                    return ImageRect(2*_width/4, 3*_height/4, _width/4, _height/4);
                    
                case 15:
                    return ImageRect(3*_width/4, 3*_height/4, _width/4, _height/4);
                    
                default:
                    return ImageRect(0, 0, _width, _height);
            }
        }
        
    protected:
        Pixel *_data;
        int _width;
        int _height;
        bool _owning;
    };
}

#endif /* defined(__ACG_Tracker_Demo__Image__) */
