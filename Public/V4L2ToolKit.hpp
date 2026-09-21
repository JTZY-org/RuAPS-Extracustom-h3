
#pragma once
#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstring>
#include <sys/mman.h>
#include <errno.h>
#include <queue>
#include <mutex>
#include <memory>
#ifdef DEBUG
#include <iostream>
#endif

namespace V4L2Tools
{
    struct V4l2Info
    {
        int ImgWidth = 640;
        int ImgHeight = 380;
        int FrameRate = 30;
        int FrameBuffer = 4;
        bool Is_AutoSize = false;
        unsigned int PixFormat = V4L2_PIX_FMT_BGR24;
        unsigned int PixFormatOut = V4L2_PIX_FMT_H264;
        v4l2_memory V4L2OUT_TYPE = V4L2_MEMORY_USERPTR;
        bool IsDMABuffSupport = false;
        // H264 camera codec control
        int H264_PSize = 60;
        int H264_Profile = 0;
        int H264_VidQB = 35;
        int H264_Bitrate = 1500000;
        bool H264_EnablePPS = true;
        int ImgWidthOut = 640;
        int ImgHeightOut = 380;
    };

    struct V4l2Subdev
    {
        int padCount;
        std::string device;
    };

    class V4l2Data
    {
    public:
        int id;
        int width;
        int height;
        int maxsize;
        unsigned int size;
        unsigned int pixfmt;
        unsigned int bytesperline;
        uint8_t *data = nullptr;
        int dmabufFD;
        bool ismapping;
        bool isFrameAvaliable;
        bool isBufferModeOnly;
        std::mutex dataLocking;
        //
        V4l2Data() : id(0), width(0), height(0), maxsize(0),
                     size(0), pixfmt(0), data(nullptr),
                     bytesperline(0), dmabufFD(-1), ismapping(true), isFrameAvaliable(false),
                     isBufferModeOnly(false) {};
        V4l2Data(int width,
                 int height,
                 int maxsize,
                 unsigned int size,
                 unsigned int pixfmt,
                 unsigned int bytesperline,
                 bool ismapping = false,
                 int dmabufFD = -1,
                 int id = 0,
                 bool isFrameAvaliable = false,
                 bool isBufferModeOnly = false)
        {
            this->id = id;
            this->width = width;
            this->height = height;
            this->size = size;
            this->maxsize = maxsize;
            this->pixfmt = pixfmt;
            this->bytesperline = bytesperline;
            this->ismapping = ismapping;
            this->dmabufFD = dmabufFD;
            this->isFrameAvaliable = isFrameAvaliable;
            this->isBufferModeOnly = isBufferModeOnly;
            if (!this->ismapping)
            {

                this->data = new uint8_t[this->size];
#ifdef DEBUG
                std::cout << "\033[33m[V4L2Info] V4L2 alloc dataBuffer check:" << std::hex << static_cast<void *>(data) << std::dec << "\n";
#endif
            }
            else
                this->data = nullptr;
        };

        V4l2Data(V4l2Data &&Data) noexcept
        {
            std::lock_guard<std::mutex> lock(dataLocking);
            id = Data.id;
            width = Data.width;
            height = Data.height;
            maxsize = Data.maxsize;
            size = Data.size;
            pixfmt = Data.pixfmt;
            bytesperline = Data.bytesperline;
            data = Data.data;
            dmabufFD = Data.dmabufFD;
            ismapping = Data.ismapping;
            isFrameAvaliable = Data.isFrameAvaliable;
            //
            Data.data = nullptr;
            Data.size = 0;
            Data.ismapping = true;
        }

        V4l2Data(const V4l2Data &DataCpy)
        {
            datacopy(DataCpy);
        };

        V4l2Data &operator=(const V4l2Data &DataCpy)
        {
            datacopy(DataCpy);
            return *this;
        };

        ~V4l2Data()
        {
            if (!ismapping && data != nullptr)
            {
#ifdef DEBUG
                std::cout << "\033[33m[V4L2Info] V4L2 data deleted" << "\n";
#endif
                delete data;
                data = nullptr;
            }
        };

    private:
        void datacopy(const V4l2Data &DataCpy)
        {
            std::lock_guard<std::mutex> lock(dataLocking);
            id = DataCpy.id;
            width = DataCpy.width;
            height = DataCpy.height;
            size = DataCpy.size;
            maxsize = DataCpy.maxsize;
            pixfmt = DataCpy.pixfmt;
            bytesperline = DataCpy.bytesperline;
            dmabufFD = DataCpy.dmabufFD;
            isFrameAvaliable = DataCpy.isFrameAvaliable;
            /*
                   when mapped data copy to selfmap,
                   selfmap copy mapping to self,
                   don't change flag
            */
            if (!DataCpy.ismapping)
            {
                if (size > 0)
                {
#ifdef DEBUG
                    std::cout << "\033[33m[V4L2Info] V4L2 copying "
                              << std::hex << static_cast<void *>(DataCpy.data) << " "
                              << ismapping
                              << " " << static_cast<void *>(data) << std::dec << "\n";
#endif
                    if (data != nullptr)
                    {
                        std::copy(DataCpy.data, DataCpy.data + size, data);
                    }
                    else
                    {
#ifdef DEBUG
                        std::cout << "\033[33m[V4L2Info] V4L2 alloc dataBuffer check" << "\n";
#endif
                        data = new uint8_t[size];
                        std::copy(DataCpy.data, DataCpy.data + size, data);
                    }
                }
            }
            else
            {
                data = DataCpy.data; // Just copy the pointer if the data is mapped (no need for allocation)
            }

            ismapping = DataCpy.ismapping;
        }
    };
}