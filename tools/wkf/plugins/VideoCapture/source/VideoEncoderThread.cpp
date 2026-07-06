// ****************************************************************************
// CUI
//
// The Advanced Framework for Simulation, Integration, and Modeling (AFSIM)
//
// Copyright 2016 Infoscitex, a DCS Company. All rights reserved.
//
// The use, dissemination or disclosure of data in this file is subject to
// limitation or restriction. See accompanying README and LICENSE for details.
// ****************************************************************************

#include "VideoEncoderThread.hpp"

#include "UtMemory.hpp"
#include "UtSleep.hpp"

VideoCapture::EncoderThread::EncoderThread()
   : UtThread()
   , mLocalData(ut::make_unique<Queue>())
   , mRemoteData(ut::make_unique<Queue>())
   , mMutex()
   , mRunning(true)
{
}

void VideoCapture::EncoderThread::Run()
{
   while (mRunning)
   {
      if (mLocalData->empty())
      {
         if (mMutex.try_lock())
         {
            std::swap(mLocalData, mRemoteData);
            mMutex.unlock();
         }
      }
      if (!mLocalData->empty())
      {
         while (!mLocalData->empty())
         {
            EncodeCapture(mLocalData->front());
            mLocalData->pop();
         }
         // encode the local data frames
      }
      else
      {
         UtSleep::Sleep(GetFrameLength()); // lets wait for data
      }
   }
}

void VideoCapture::EncoderThread::PushData(const QPixmap& aPixmap, unsigned int aTimes)
{
   mMutex.lock();
   mRemoteData->emplace(aPixmap, aTimes);
   mMutex.unlock();
}
