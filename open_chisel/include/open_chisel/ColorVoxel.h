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

#ifndef COLORVOXEL_H_
#define COLORVOXEL_H_

#include <algorithm>
#include <limits>
#include <stdint.h>


namespace chisel
{

    class ColorVoxel
    {
        public:
            ColorVoxel();
            virtual ~ColorVoxel();

            static inline float Saturate(float value)
            {
                return std::min(std::max(value, 0.0f), 255.0f);
            }

            inline uint8_t GetRed() const { return red; }
            inline uint8_t GetGreen() const { return green; }
            inline uint8_t GetBlue()  const { return blue; }
            inline uint8_t GetWeight() const { return weight; }
            inline void SetRed(uint8_t value)
            {
                red = value;
            }
            inline void SetGreen(uint8_t value)
            {
                green = value;
            }
            inline void SetBlue(uint8_t value)
            {
                blue = value;
            }
            inline void SetWeight(uint8_t value)
            {
                weight = value;
            }

            inline void Integrate(const uint8_t& newRed, const uint8_t& newGreen, const uint8_t& newBlue, const uint8_t& weightUpdate)
            {
                if(weight >= std::numeric_limits<uint8_t>::max() - weightUpdate)
                {
                    return;
                }

                float oldRed = static_cast<float>(GetRed());
                float updatedRed = Saturate(static_cast<float>(weight * oldRed + weightUpdate * newRed) / (weightUpdate + weight));
                red = static_cast<uint8_t>(updatedRed);

                float oldGreen = static_cast<float>(GetGreen());
                float updatedGreen = Saturate(static_cast<float>(weight * oldGreen + weightUpdate * newGreen) / (weightUpdate + weight));
                green = static_cast<uint8_t>(updatedGreen);

                float oldBlue = static_cast<float>(GetBlue());
                float updatedBlue = Saturate(static_cast<float>(weight * oldBlue + weightUpdate * newBlue) / (weightUpdate + weight));
                blue = static_cast<uint8_t>(updatedBlue);

                SetWeight(weight + weightUpdate);
            }

            inline void Reset()
            {
                red = 0;
                green = 0;
                blue = 0;
                weight = 0;
            }

        protected:
            uint8_t red;
            uint8_t green;
            uint8_t blue;
            uint8_t weight;
    };

} // namespace chisel 

#endif // COLORVOXEL_H_ 
