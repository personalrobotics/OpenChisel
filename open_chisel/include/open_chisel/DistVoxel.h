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

#ifndef DISTVOXEL_H_
#define DISTVOXEL_H_

#include <limits>
#include <stdint.h>

namespace chisel
{

    class DistVoxel
    {
        public:
            DistVoxel();
            virtual ~DistVoxel();

            inline int16_t GetSDFInt() const { return sdf; }
            inline void SetSDFInt(const int16_t& distance) { sdf = distance; }

            inline float GetSDF() const
            {
                return static_cast<float>(sdf) / static_cast<float>(std::numeric_limits<int16_t>::max());
            }

            inline void SetSDF(const float& distance)
            {
                sdf = static_cast<uint16_t>(distance * std::numeric_limits<int16_t>::max());
            }

            inline uint16_t GetWeight() const { return weight; }
            inline void SetWeight(const uint16_t w) { weight = w; }

            inline void Integrate(const float& distUpdate, const uint16_t weightUpdate)
            {
                float newDist = (weight * sdf + weightUpdate * distUpdate) / (weightUpdate + weight);
                SetSDF(newDist);
                SetWeight(weight + weightUpdate);
            }

        protected:
           int16_t sdf;
           uint16_t weight;
    };

} // namespace chisel 

#endif // DISTVOXEL_H_ 
