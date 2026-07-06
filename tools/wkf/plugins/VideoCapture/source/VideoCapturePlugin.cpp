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
#include "VideoCapturePlugin.hpp"

#include <QApplication>
#include <QDesktopWidget>
#include <QFile>
#include <QMenu>
#include <QMessageBox>
#include <QPainter>
#include <QScreen>
#include <QStatusBar>
#include <QString>
#include <QWindow>

#include "VideoCapturePrefObject.hpp"
#include "VideoCapturePrefWidget.hpp"
#include "WkfAction.hpp"
#include "WkfEnvironment.hpp"
#include "WkfMainWindow.hpp"
#ifdef WIN32
#include "Win32VideoEncoderThread.hpp"
#else
#include "FFMpegVideoEncoderThread.hpp"
#endif

WKF_PLUGIN_DEFINE_SYMBOLS(VideoCapture::Plugin,
                          "Video Capture",
                          "The Video Capture plugin adds options to the Tools menu to take screen shots and capture "
                          "video from the application.",
                          "all")

VideoCapture::Plugin::Plugin(const QString& aPluginName, const size_t aUniqueId)
   : wkf::Plugin(aPluginName, aUniqueId)
   , mScreenAction(nullptr)
   , mVideoAction(nullptr)
   , mRecording(false)
   , mTimer(nullptr)
   , mFrameDeficit(0.0)
{
   auto   mainWindowPtr = wkfEnv.GetMainWindow();
   QMenu* toolsMenuPtr  = mainWindowPtr->GetMenuByName("Tools");
   if (toolsMenuPtr)
   {
      mVideoAction =
         new wkf::Action(QIcon::fromTheme("video_camera"), "Record Video", mainWindowPtr, QKeySequence("Ctrl+Shift+V"));
      connect(mVideoAction, &QAction::triggered, this, &Plugin::ToggleState);
      toolsMenuPtr->addAction(mVideoAction);
      mScreenAction =
         new wkf::Action(QIcon::fromTheme("camera"), "Screenshot", mainWindowPtr, QKeySequence("Ctrl+Shift+S"));
      connect(mScreenAction, &QAction::triggered, this, &Plugin::ScreenShot);
      toolsMenuPtr->addAction(mScreenAction);
   }

   mStatusLabel = new QLabel(mainWindowPtr->statusBar());
   mStatusLabel->setText("Recording");
   mStatusLabel->setStyleSheet("QLabel { color: rgba(255, 0, 0, 192); }");
   mainWindowPtr->statusBar()->insertPermanentWidget(0, mStatusLabel);
   mStatusLabel->hide();

   mVideoPrefWidgetPtr          = new PrefWidget();
   const PrefObject* prefObject = mVideoPrefWidgetPtr->GetPreferenceObject();

   connect(prefObject, &PrefObject::OutputDirChanged, this, &Plugin::OutputDirChanged);
   connect(prefObject, &PrefObject::ScreenShotTemplateChanged, this, &Plugin::ScreenShotTemplateChanged);
   connect(prefObject, &PrefObject::VideoTemplateChanged, this, &Plugin::VideoTemplateChanged);
   connect(prefObject, &PrefObject::BitRateChanged, this, &Plugin::BitRateChanged);
   connect(prefObject, &PrefObject::CaptureSourceChanged, this, &Plugin::CaptureSourceChanged);
}

VideoCapture::Plugin::~Plugin()
{
   if (nullptr != mTimer)
   {
      mVideoAction = nullptr;
      mTimer       = nullptr; // the main window already took care of these
      Stop();
   }
}

QList<wkf::Action*> VideoCapture::Plugin::GetActions() const
{
   QList<wkf::Action*> list;
   list.push_back(mVideoAction);
   list.push_back(mScreenAction);
   return list;
}

QList<wkf::PrefWidget*> VideoCapture::Plugin::GetPreferencesWidgets() const
{
   QList<wkf::PrefWidget*> prefs;
   prefs << mVideoPrefWidgetPtr;
   return prefs;
}

void VideoCapture::Plugin::ScreenShot()
{
   int index = 0;

   QFile file(mOutputDir + '/' + mScreenShotTemplate + '_' + QString::number(index) + ".png");
   while (file.exists())
   {
      index++;
      file.setFileName(mOutputDir + '/' + mScreenShotTemplate + '_' + QString::number(index) + ".png");
   }
   file.open(QIODevice::WriteOnly);

   QWidget* widget = mUseMainWindow ? wkfEnv.GetMainWindow() : QApplication::focusWidget()->window();

   // it would be preferable to use window->screen()->grabWindow(window->winId())
   // but it is unreliable system to system
   if (widget->grab().save(&file, "PNG"))
   {
      wkfEnv.GetMainWindow()->ShowStatusMessage("Screenshot Saved ( " + file.fileName() + " )", 3000);
   }
   else
   {
      QString errorMsg =
         "Screenshot not saved. Please check the validity of the provided file path of \"" + file.fileName() + "\"";
      QMessageBox::warning(nullptr, "Screenshot Save Error", errorMsg);
   }
   file.close();
}

void VideoCapture::Plugin::ToggleState()
{
   if (nullptr != mVideoAction)
   {
      mRecording = !mRecording;
      if (mRecording)
      {
         mClock.ResetClock();
         Record();
      }
      else
      {
         Stop();
      }
   }
}

void VideoCapture::Plugin::Record()
{
   mStatusLabel->show();
   int pratio = QApplication::desktop()->devicePixelRatio();

   mVideoAction->setText("Stop Recording Video");

   mCaptureWindow = mUseMainWindow ? wkfEnv.GetMainWindow() : QApplication::focusWidget()->window();
   connect(mCaptureWindow, &QWidget::destroyed, this, &Plugin::RecordedWindowVisibilityChanged);
#ifdef WIN32
   mEncoderThread = new Win32EncoderThread(mOutputDir.toStdString(),
                                           mVideoTemplate.toStdString(),
                                           mBitRate,
                                           mCaptureWindow->width() * pratio,
                                           mCaptureWindow->height() * pratio);
#else
   mEncoderThread = new FFMpegEncoderThread(mOutputDir.toStdString(),
                                            mVideoTemplate.toStdString(),
                                            mBitRate,
                                            mCaptureWindow->width() * pratio,
                                            mCaptureWindow->height() * pratio);
#endif
   mEncoderThread->Start();

   mTimer = new QTimer(wkfEnv.GetMainWindow());
   mTimer->setTimerType(Qt::PreciseTimer);
   mTimer->setInterval(1000 * mEncoderThread->GetFrameLength());
   connect(mTimer, &QTimer::timeout, this, &Plugin::CaptureFrame);
   mTimer->start();
}

void VideoCapture::Plugin::Stop()
{
   mStatusLabel->hide();

   QFileInfo filePath(mEncoderThread->GetOutputFileName());
   if (filePath.exists() && filePath.isFile())
   {
      wkfEnv.GetMainWindow()->ShowStatusMessage("Recording Saved ( " + mEncoderThread->GetOutputFileName() + " )", 3000);
   }
   else
   {
      QString errorMsg = "Video not saved. Please check the validity of the provided file path of \"" +
                         mEncoderThread->GetOutputFileName() + "\"";
      QMessageBox::warning(nullptr, "Video Save Error", errorMsg);
   }

   if (nullptr != mTimer)
   {
      delete mTimer;
      mTimer = nullptr;
   }
   if (nullptr != mVideoAction)
   {
      mVideoAction->setText("Record Video");
   }
   disconnect(mCaptureWindow, &QWidget::destroyed, this, &Plugin::RecordedWindowVisibilityChanged);

   mEncoderThread->Stop();
   mEncoderThread->Join();
   mEncoderThread->Close();

   delete mEncoderThread;
}

void VideoCapture::Plugin::RecordedWindowVisibilityChanged(bool aState)
{
   mRecording = false;
   Stop();
}

void VideoCapture::Plugin::CaptureFrame()
{
   if (mRecording)
   {
      double frameTime = mClock.GetClock() + mFrameDeficit;
      mClock.ResetClock();
      int frames = frameTime / mEncoderThread->GetFrameLength();
      if (frames > 0)
      {
         // it would be preferable to use mCaptureWindow->screen()->grabWindow(mCaptureWindow->winId())
         // but it is unreliable system to system
         mEncoderThread->PushData(mCaptureWindow->grab(), frames);
      }
      mFrameDeficit = frameTime - frames * mEncoderThread->GetFrameLength();
   }
}

void VideoCapture::Plugin::OutputDirChanged(const QString& aDir)
{
   mOutputDir = aDir;
}

void VideoCapture::Plugin::ScreenShotTemplateChanged(const QString& aTemplate)
{
   mScreenShotTemplate = aTemplate;
}

void VideoCapture::Plugin::VideoTemplateChanged(const QString& aTemplate)
{
   mVideoTemplate = aTemplate;
}

void VideoCapture::Plugin::BitRateChanged(const QString& aBitRate)
{
   if (aBitRate == "Low")
   {
      mBitRate = 6400000;
   }
   else if (aBitRate == "Medium")
   {
      mBitRate = 12800000;
   }
   else if (aBitRate == "High")
   {
      mBitRate = 25600000;
   }
}

void VideoCapture::Plugin::CaptureSourceChanged(const QString& aSrc)
{
   mUseMainWindow = (aSrc == "Main Window") ? true : false;
}
