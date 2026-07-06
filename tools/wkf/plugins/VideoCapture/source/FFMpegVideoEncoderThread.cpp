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

#include "FFMpegVideoEncoderThread.hpp"
#ifndef WIN32
#include "UtSleep.hpp"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <QFile>
#include <QMessageBox>

VideoCapture::FFMpegEncoderThread::FFMpegEncoderThread(const std::string& aOutputDir,
                                                       const std::string& aVideoTemplate,
                                                       const int          aBitRate,
                                                       int                aWidth,
                                                       int                aHeight)
   : EncoderThread()
{
   mCodec = avcodec_find_encoder(AV_CODEC_ID_MPEG1VIDEO);
   if (!mCodec)
   {
      QMessageBox::warning(nullptr, "ERROR", "Could not find video encoding codec");
      return;
   }
   mCodecContext = avcodec_alloc_context3(mCodec);
   if (!mCodecContext)
   {
      QMessageBox::warning(nullptr, "ERROR", "Could not allocate video encoding context");
      return;
   }
   mCodecContext->bit_rate      = aBitRate;
   mCodecContext->width         = aWidth - (aWidth % 2);
   mCodecContext->height        = aHeight - (aHeight % 2);
   mCodecContext->time_base.num = 1;
   mCodecContext->time_base.den = 30;
   mCodecContext->gop_size      = 10;
   mCodecContext->max_b_frames  = 1;
   mCodecContext->pix_fmt       = AV_PIX_FMT_YUV420P;

   if (mCodec->id == AV_CODEC_ID_H264)
   {
      av_opt_set(mCodecContext->priv_data, "preset", "slow", 0);
   }

   if (avcodec_open2(mCodecContext, mCodec, nullptr) < 0)
   {
      QMessageBox::warning(nullptr, "ERROR", "Could not open video encoding codec");
      avcodec_free_context(&mCodecContext);
      return;
   }
   int         index   = 0;
   std::string prepath = aOutputDir;
   prepath += '/';
   prepath += aVideoTemplate;
   prepath += '_';
   std::string path = prepath;
   path += QString::number(index).toStdString();
   path += ".avi";
   mCaptureFile = new QFile();
   mCaptureFile->setFileName(path.c_str());
   while (mCaptureFile->exists())
   {
      index++;
      path = prepath;
      path += QString::number(index).toStdString();
      path += ".avi";
      mCaptureFile->setFileName(path.c_str());
   }

   mCaptureFile->open(QIODevice::WriteOnly);
   if (!mCaptureFile->isOpen())
   {
      QMessageBox::warning(nullptr, "ERROR", "Could not open video output file");
      return;
   }

   mFrame = av_frame_alloc();
   if (!mFrame)
   {
      QMessageBox::warning(nullptr, "ERROR", "Could not allocate video frame");
      return;
   }
   mFrame->format = mCodecContext->pix_fmt;
   mFrame->width  = mCodecContext->width;
   mFrame->height = mCodecContext->height;

   int ret =
      av_image_alloc(mFrame->data, mFrame->linesize, mCodecContext->width, mCodecContext->height, mCodecContext->pix_fmt, 32);
   if (ret < 0)
   {
      QMessageBox::warning(nullptr, "ERROR", "Could not allocate video image data");
   }

   mSWS_Context = sws_getContext(mCodecContext->width,
                                 mCodecContext->height,
                                 AV_PIX_FMT_RGB32,
                                 mCodecContext->width,
                                 mCodecContext->height,
                                 AV_PIX_FMT_YUV420P,
                                 0,
                                 nullptr,
                                 nullptr,
                                 nullptr);
   int* invTable;
   int* table;
   int  srcRange;
   int  dstRange;
   int  brightness;
   int  contrast;
   int  saturation;
   sws_getColorspaceDetails(mSWS_Context, &invTable, &srcRange, &table, &dstRange, &brightness, &contrast, &saturation);
   sws_setColorspaceDetails(mSWS_Context,
                            invTable,
                            1, // srcRange (0 mpeg, 1 jpeg)
                            table,
                            1, // dstRange (0 mpeg, 1 jpeg)
                            brightness,
                            contrast,
                            saturation); // brightness, saturation, contrast correction [-16,16]
}

void VideoCapture::FFMpegEncoderThread::Close()
{
   mMutex.lock();
   while (!mLocalData->empty())
   {
      EncodeCapture(mLocalData->front());
      mLocalData->pop();
   }
   mMutex.unlock();

   /*   uint8_t endcode[] = { 0, 0, 1, 0xb7 };
      mCaptureFile->write((char*)endcode, sizeof(endcode));*/
   mCaptureFile->close();
   delete mCaptureFile;

   avcodec_close(mCodecContext);
   av_free(mCodecContext);
   av_freep(&mFrame->data[0]);
   av_frame_free(&mFrame);
   sws_freeContext(mSWS_Context);
}

QString VideoCapture::FFMpegEncoderThread::GetOutputFileName() const
{
   if (mCaptureFile != nullptr)
   {
      return mCaptureFile->fileName();
   }
   return "";
}

void VideoCapture::FFMpegEncoderThread::EncodeCapture(const Capture& aCapture)
{
   QImage img = aCapture.first.toImage().copy(0, 0, mCodecContext->width, mCodecContext->height);
   for (unsigned int i = 0; i < aCapture.second; ++i)
   {
      EncodeImage(img);
   }
}

void VideoCapture::FFMpegEncoderThread::EncodeImage(const QImage& aImage)
{
   av_init_packet(&mPacket);
   mPacket.data = nullptr;
   mPacket.size = 0;

   const uint8_t* inData[1]     = {aImage.bits()};
   int            inLineSize[1] = {4 * mCodecContext->width};
   sws_scale(mSWS_Context, inData, inLineSize, 0, mCodecContext->height, mFrame->data, mFrame->linesize);

   int retsend = avcodec_send_frame(mCodecContext, mFrame);
   if (retsend < 0)
   {
      QMessageBox::warning(nullptr, "ERROR", "Error encoding video");
      Close();
      return;
   }
   int retrec = avcodec_receive_packet(mCodecContext, &mPacket);
   while (retrec >= 0)
   {
      mCaptureFile->write((char*)mPacket.data, (qint64)mPacket.size);
      av_packet_unref(&mPacket);
      retrec = avcodec_receive_packet(mCodecContext, &mPacket);
   }
}
#endif
