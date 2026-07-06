// ****************************************************************************
// CUI
//
// The Advanced Framework for Simulation, Integration, and Modeling (AFSIM)
//
// Copyright 2017 Infoscitex, a DCS Company. All rights reserved.
//
// The use, dissemination or disclosure of data in this file is subject to
// limitation or restriction. See accompanying README and LICENSE for details.
// ****************************************************************************

#ifndef FFMPEGVIDEOENCODERTHREAD_HPP
#define FFMPEGVIDEOENCODERTHREAD_HPP
#ifndef WIN32
#include <mutex>
#include <queue>

#include <QFile>
#include <QPixmap>

#include "VideoEncoderThread.hpp"

extern "C"
{
#include <libavcodec/avcodec.h>
}

struct SwsContext;

// this class implements a thread to encode movie files using ffmpeg

namespace VideoCapture
{
class FFMpegEncoderThread : public EncoderThread
{
public:
   FFMpegEncoderThread(const std::string& aOutputDir,
                       const std::string& aVideoTemplate,
                       const int          aBitRate,
                       int                aWidth,
                       int                aHeight);

   ~FFMpegEncoderThread() override = default;

   void Close() override;

   QString GetOutputFileName() const override;

   double GetFrameLength() const override { return 0.0333; }

private:
   void EncodeCapture(const Capture& aCapture) override;
   void EncodeImage(const QImage& aImage);

   AVCodec*        mCodec;
   AVCodecContext* mCodecContext;
   QFile*          mCaptureFile;
   AVFrame*        mFrame;
   AVPacket        mPacket;
   SwsContext*     mSWS_Context;
};
} // namespace VideoCapture

#endif // WIN
#endif
