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

#include <open_chisel/camera/PinholeCamera.h>

namespace chisel
{

    PinholeCamera::PinholeCamera()
    {


    }

    PinholeCamera::~PinholeCamera()
    {

    }

    Vec3 PinholeCamera::ProjectPoint(const Vec3& point)
    {
        const float& x = point(0);
        const float& y = point(1);
        const float& z = point(2);
        assert(z > 0);
        return Vec3(intrinsics.GetFx() * (x / z) + intrinsics.GetCx(), intrinsics.GetFy() * (y / z) + intrinsics.GetCy(), z);
    }

    Vec3 PinholeCamera::UnprojectPoint(const Vec3& point)
    {
        const float& u = point(0);
        const float& v = point(1);
        const float& z = point(2);
        return Vec3(z * ((u - intrinsics.GetCx()) / intrinsics.GetFx()), z * ((v - intrinsics.GetCy()) / intrinsics.GetFy()), z);
    }

    void PinholeCamera::SetupFrustum(float near, float far, const Transform& view, Frustum* frustum)
    {
        assert(frustum != nullptr);
        frustum->SetFromParams(view, near, far, intrinsics.GetFy(), intrinsics.GetFy(), intrinsics.GetCx(), intrinsics.GetCy(), width, height);
    }

} // namespace chisel 
