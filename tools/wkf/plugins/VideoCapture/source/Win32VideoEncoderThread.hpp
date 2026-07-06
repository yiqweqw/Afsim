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

#ifndef WIN32VIDEOENCODERTHREAD_HPP
#define WIN32VIDEOENCODERTHREAD_HPP

#include "VideoEncoderThread.hpp"

#ifdef WIN32

#include <windows.h>
struct IMFSinkWriter;

namespace VideoCapture
{
class Win32EncoderThread : public EncoderThread
{
public:
   Win32EncoderThread(const std::string& aOutputDir,
                      const std::string& aVideoTemplate,
                      const int          aBitRate,
                      int                aWidth,
                      int                aHeight);

   ~Win32EncoderThread() override = default;

   void Close() override;

   QString GetOutputFileName() const override;
   double  GetFrameLength() const override { return 1.0 / mFrameRate; }

private:
   void EncodeCapture(const Capture& aCapture) override;
   void EncodeImage(const QImage& aImage);

   const GUID VIDEO_ENCODING_FORMAT;
   float      mFrameRate;

   DWORD          mStreamIndex;
   IMFSinkWriter* mSinkWriter;
   LONGLONG       mFrameTime;
   int            mWidth;
   int            mHeight;
   QString        mFile;
};
} // namespace VideoCapture

#endif

#endif
