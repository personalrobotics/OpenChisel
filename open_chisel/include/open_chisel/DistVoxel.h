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

#include <open_chisel/FixedPointFloat.h>

namespace chisel
{

    class DistVoxel
    {
        public:
            DistVoxel();
            virtual ~DistVoxel();

            inline float GetSDF() const
            {
                return sdf;
            }

            inline void SetSDF(const float& distance)
            {
                sdf = distance;
            }

            inline float GetWeight() const { return weight; }
            inline void SetWeight(const float& w) { weight = w; }

            inline void Integrate(const float& distUpdate, const float& weightUpdate)
            {
                float oldSDF = GetSDF();
                float oldWeight = GetWeight();
                float newDist = (oldWeight * oldSDF + weightUpdate * distUpdate) / (weightUpdate + oldWeight);
                SetSDF(newDist);
                SetWeight(oldWeight + weightUpdate);

            }

            inline void Carve()
            {
                //Reset();
                Integrate(0.0, 1.5);
            }

            inline void Reset()
            {
                sdf = 99999;
                weight = 0;
            }

        protected:
           float sdf;
           float weight;
    };

} // namespace chisel 

#endif // DISTVOXEL_H_ 
