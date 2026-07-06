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

#ifndef VIDEOENCODERTHREAD_HPP
#define VIDEOENCODERTHREAD_HPP

#include <memory>
#include <mutex>
#include <queue>

#include <QPixmap>

#include "UtThread.hpp"

// this class implements a thread to encode movie files using ffmpeg

namespace VideoCapture
{
class EncoderThread : public UtThread
{
public:
   EncoderThread();

   ~EncoderThread() override = default;

   virtual void Close() = 0;
   void         Run() override;
   void         Stop() { mRunning = false; }
   void         PushData(const QPixmap& aPixmap, unsigned int aTimes);

   virtual QString GetOutputFileName() const = 0;

   virtual double GetFrameLength() const = 0;

protected:
   using Capture = std::pair<QPixmap, unsigned int>;
   using Queue   = std::queue<Capture>;

   virtual void EncodeCapture(const Capture& aCapture) = 0;

   std::unique_ptr<Queue> mLocalData;
   std::unique_ptr<Queue> mRemoteData;
   std::mutex             mMutex;

private:
   volatile bool mRunning;
};
} // namespace VideoCapture
#endif
