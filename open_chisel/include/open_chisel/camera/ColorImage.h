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


#ifndef COLORIMAGE_H_
#define COLORIMAGE_H_


namespace chisel
{

    template <class DataType = uint8_t, size_t numChannels = 3> class ColorImage
    {
        public:
            ColorImage() :
                data(nullptr), width(-1), height(-1)
            {

            }
            ColorImage(int w, int h) :
                data(nullptr), width(w), height(h)
            {
                data = new DataType[width * height * numChannels];
            }

            virtual ~ColorImage()
            {
                if(data != nullptr)
                {
                    delete [] data;
                    data = nullptr;
                }
            }

            inline int Index(int row, int col, int channel) const
            {
                return (col + row * width) * numChannels + channel;
            }

            inline const DataType& At(int row, int col, int channel) const
            {
                return data[Index(row, col, channel)];
            }

            inline DataType& AtMutable(int row, int col, int channel) const
            {
                return data[Index(row, col, channel)];
            }

            inline bool IsInside(int row, int col) const
            {
                return row >= 0 && row < width && col >= 0 && col < height;
            }

            inline const DataType* GetData() const { return data; }
            inline DataType* GetMutableData() { return data; }
            inline void SetData(DataType* d) { data = d; }
            inline int GetWidth() const { return width; }
            inline int GetHeight() const { return height; }
            inline void SetWidth(int w) { width = w; }
            inline void SetHeight(int h) { height = h; }

        protected:
            DataType* data;
            int width;
            int height;
    };
    template <class DataType = uint8_t, size_t numChannels = 3> using ColorImagePtr = std::shared_ptr<ColorImage<DataType, numChannels> >;
    template <class DataType = uint8_t, size_t numChannels = 3> using ColorImageConstPtr = std::shared_ptr<const ColorImage<DataType, numChannels> >;

} // namespace chisel



#endif // COLORIMAGE_H_ 
