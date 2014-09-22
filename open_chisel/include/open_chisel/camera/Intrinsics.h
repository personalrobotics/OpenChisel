// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef INTRINSICS_H_
#define INTRINSICS_H_

#include <memory>
#include <open_chisel/geometry/Geometry.h>

namespace chisel
{

    class Intrinsics
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Intrinsics();
            virtual ~Intrinsics();

            inline float GetFx() const { return matrix(0, 0); }
            inline void SetFx(float fx) { matrix(0, 0) = fx; }
            inline float GetFy() const { return matrix(1, 1); }
            inline void SetFy(float fy) { matrix(1, 1) = fy; }
            inline float GetCx() const { return matrix(0, 2); }
            inline void SetCx(float cx) { matrix(0, 2) = cx; }
            inline float GetCy() const { return matrix(1, 2); }
            inline void SetCy(float cy) { matrix(1, 2) = cy; }

            inline const Mat3x3& GetMatrix() const { return matrix; }
            inline Mat3x3& GetMutableMatrix() { return matrix; }
            inline void SetMatrix(const Mat3x3& m) { matrix = m; }

        protected:
            Mat3x3 matrix;
    };
    typedef std::shared_ptr<Intrinsics> IntrinsicsPtr;
    typedef std::shared_ptr<const Intrinsics> IntrinsicsConstPtr;

} // namespace chisel 

#endif // INTRINSICS_H_ 
