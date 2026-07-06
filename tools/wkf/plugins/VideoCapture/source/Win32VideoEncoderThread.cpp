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

#include "Win32VideoEncoderThread.hpp"

#ifdef WIN32

#include <iostream>
#include <new>

#include <windows.h>

#include <Mfidl.h>
#include <QMessageBox>
#include <mfapi.h>
#include <mferror.h>
#include <mfreadwrite.h>

#include "UtPath.hpp"
#include "WkfQueueableMessageObject.hpp"

VideoCapture::Win32EncoderThread::Win32EncoderThread(const std::string& aOutputDir,
                                                     const std::string& aVideoTemplate,
                                                     const int          aBitRate,
                                                     int                aWidth,
                                                     int                aHeight)
   : EncoderThread()
   , VIDEO_ENCODING_FORMAT(MFVideoFormat_H264)
   , mFrameRate(15)
   , mSinkWriter(nullptr)
   , mFrameTime(0)
   , mWidth(aWidth)
   , mHeight(aHeight)
   , mFile("")
{
   int         index   = 0;
   std::string prepath = aOutputDir;
   prepath += '/';
   prepath += aVideoTemplate;
   prepath += '_';
   std::string path = prepath;
   path += QString::number(index).toStdString();
   path += ".mp4";
   UtPath file(path);
   while (file.Exists())
   {
      index++;
      path = prepath;
      path += QString::number(index).toStdString();
      path += ".mp4";
      file = UtPath(path);
   }
   mFile = path.c_str();

   mWidth -= (aWidth % 2);
   mHeight -= (aHeight % 2);
   HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);

   IMFMediaType* pMediaTypeOut = nullptr;
   IMFMediaType* pMediaTypeIn  = nullptr;

   if (SUCCEEDED(hr))
   {
      hr = MFStartup(MF_VERSION);
   }
   if (SUCCEEDED(hr))
   {
      LPCWSTR     str = reinterpret_cast<LPCWSTR>(mFile.utf16());
      std::string foo = mFile.toStdString();
      hr              = MFCreateSinkWriterFromURL(str, NULL, NULL, &mSinkWriter);
   }
   if (FAILED(hr))
   {
      wkf::QueueableMessageObject::DisplayQueuedMessage(
         QMessageBox::Warning,
         "Win32VideoEncoderThread",
         QString("constructor: Something went wrong when creating the sink writer"));
      return;
   }
   if (SUCCEEDED(hr))
   {
      hr = MFCreateMediaType(&pMediaTypeOut);
   }
   if (SUCCEEDED(hr))
   {
      hr = pMediaTypeOut->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
   }
   if (SUCCEEDED(hr))
   {
      hr = pMediaTypeOut->SetGUID(MF_MT_SUBTYPE, VIDEO_ENCODING_FORMAT);
   }
   if (SUCCEEDED(hr))
   {
      hr = pMediaTypeOut->SetUINT32(MF_MT_AVG_BITRATE, aBitRate);
   }
   if (SUCCEEDED(hr))
   {
      hr = pMediaTypeOut->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
   }
   if (SUCCEEDED(hr))
   {
      hr = MFSetAttributeSize(pMediaTypeOut, MF_MT_FRAME_SIZE, mWidth, mHeight);
   }
   if (SUCCEEDED(hr))
   {
      hr = MFSetAttributeRatio(pMediaTypeOut, MF_MT_FRAME_RATE, mFrameRate, 1);
   }
   if (SUCCEEDED(hr))
   {
      hr = MFSetAttributeRatio(pMediaTypeOut, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
   }
   if (SUCCEEDED(hr))
   {
      hr = mSinkWriter->AddStream(pMediaTypeOut, &mStreamIndex);
   }
   if (FAILED(hr))
   {
      wkf::QueueableMessageObject::DisplayQueuedMessage(
         QMessageBox::Warning,
         "Win32VideoEncoderThread",
         QString("constructor: Something went wrong when creating the output media"));
      return;
   }
   if (SUCCEEDED(hr))
   {
      hr = MFCreateMediaType(&pMediaTypeIn);
   }
   if (SUCCEEDED(hr))
   {
      hr = pMediaTypeIn->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
   }
   if (SUCCEEDED(hr))
   {
      hr = pMediaTypeIn->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
   }
   if (SUCCEEDED(hr))
   {
      hr = pMediaTypeIn->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
   }
   if (SUCCEEDED(hr))
   {
      hr = MFSetAttributeSize(pMediaTypeIn, MF_MT_FRAME_SIZE, mWidth, mHeight);
   }
   if (SUCCEEDED(hr))
   {
      hr = MFSetAttributeRatio(pMediaTypeIn, MF_MT_FRAME_RATE, mFrameRate, 1);
   }
   if (SUCCEEDED(hr))
   {
      hr = MFSetAttributeRatio(pMediaTypeIn, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
   }
   if (SUCCEEDED(hr))
   {
      hr = mSinkWriter->SetInputMediaType(mStreamIndex, pMediaTypeIn, NULL);
   }
   if (FAILED(hr))
   {
      wkf::QueueableMessageObject::DisplayQueuedMessage(
         QMessageBox::Warning,
         "Win32VideoEncoderThread",
         QString("constructor: Something went wrong when creating the input media"));
      return;
   }
   if (SUCCEEDED(hr))
   {
      hr = mSinkWriter->BeginWriting();
   }
   if (FAILED(hr))
   {
      wkf::QueueableMessageObject::DisplayQueuedMessage(QMessageBox::Warning,
                                                        "Win32VideoEncoderThread",
                                                        QString(
                                                           "constructor: Something went wrong when beginning writing"));
      return;
   }
}

void VideoCapture::Win32EncoderThread::EncodeCapture(const Capture& aCapture)
{
   if (mSinkWriter)
   {
      QImage img = aCapture.first.toImage().copy(0, 0, mWidth, mHeight);
      for (unsigned int i = 0; i < aCapture.second; ++i)
      {
         EncodeImage(img);
      }
   }
}

void VideoCapture::Win32EncoderThread::EncodeImage(const QImage& aImage)
{
   IMFSample*      pSample  = nullptr;
   IMFMediaBuffer* pBuffer  = nullptr;
   const LONG      cbWidth  = 4 * mWidth;
   const LONG      cbBuffer = cbWidth * mHeight;

   BYTE* pData = nullptr;

   HRESULT hr = MFCreateMemoryBuffer(cbBuffer, &pBuffer);
   if (SUCCEEDED(hr))
   {
      hr = pBuffer->Lock(&pData, NULL, NULL);
   }
   if (SUCCEEDED(hr))
   {
      hr = MFCopyImage(pData, cbWidth, aImage.mirrored(false, true).bits(), cbWidth, cbWidth, mHeight);
   }
   if (FAILED(hr))
   {
      wkf::QueueableMessageObject::DisplayQueuedMessage(QMessageBox::Warning,
                                                        "Win32VideoEncoderThread",
                                                        QString("EncodeImage() Building buffer - failed"));
      Close();
      return;
   }
   if (pBuffer)
   {
      pBuffer->Unlock();
   }
   if (SUCCEEDED(hr))
   {
      hr = pBuffer->SetCurrentLength(cbBuffer);
   }
   if (SUCCEEDED(hr))
   {
      hr = MFCreateSample(&pSample);
   }
   if (SUCCEEDED(hr))
   {
      hr = pSample->AddBuffer(pBuffer);
   }
   if (SUCCEEDED(hr))
   {
      hr = pSample->SetSampleTime(mFrameTime);
   }
   if (SUCCEEDED(hr))
   {
      UINT64 VIDEO_FRAME_DURATION = 10 * 1000 * 1000 / mFrameRate;
      hr                          = pSample->SetSampleDuration(VIDEO_FRAME_DURATION);
      mFrameTime += VIDEO_FRAME_DURATION;
   }
   if (FAILED(hr))
   {
      wkf::QueueableMessageObject::DisplayQueuedMessage(QMessageBox::Warning,
                                                        "Win32VideoEncoderThread",
                                                        QString("EncodeImage() Preparing frame - failed"));
      Close();
      return;
   }
   if (SUCCEEDED(hr))
   {
      hr = mSinkWriter->WriteSample(mStreamIndex, pSample);
   }
   if (FAILED(hr))
   {
      wkf::QueueableMessageObject::DisplayQueuedMessage(QMessageBox::Warning,
                                                        "Win32VideoEncoderThread",
                                                        QString("EncodeImage() WriteSample - failed %1").arg(mFrameTime));
      Close();
      return;
   }
   if (pSample)
   {
      pSample->Release();
      pSample = nullptr;
   }
   if (pBuffer)
   {
      pBuffer->Release();
      pBuffer = nullptr;
   }
}

void VideoCapture::Win32EncoderThread::Close()
{
   if (mSinkWriter)
   {
      HRESULT hr = mSinkWriter->Finalize();
      mSinkWriter->Release();
      mSinkWriter = nullptr;
   }
   MFShutdown();
   CoUninitialize();
}

QString VideoCapture::Win32EncoderThread::GetOutputFileName() const
{
   return mFile;
}

#endif