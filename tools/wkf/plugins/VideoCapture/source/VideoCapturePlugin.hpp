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
#ifndef VIDEOCAPTUREPLUGIN_HPP
#define VIDEOCAPTUREPLUGIN_HPP

#include <QLabel>
#include <QTimer>

#include "UtWallClock.hpp"
#include "VideoCapturePrefWidget.hpp"
#include "VideoEncoderThread.hpp"
#include "WkfPlugin.hpp"

namespace VideoCapture
{
class Plugin : public wkf::Plugin
{
   Q_OBJECT

public:
   Plugin(const QString& aPluginName, const size_t aUniqueId);
   ~Plugin() override;

   QList<wkf::Action*>     GetActions() const override;
   QList<wkf::PrefWidget*> GetPreferencesWidgets() const override;

private:
   void ToggleState();
   void ScreenShot();
   void OutputDirChanged(const QString& aDir);
   void ScreenShotTemplateChanged(const QString& aTemplate);
   void VideoTemplateChanged(const QString& aTemplate);
   void BitRateChanged(const QString& aTemplate);
   void CaptureSourceChanged(const QString& aSrc);
   void RecordedWindowVisibilityChanged(bool aState);

   void Record();
   void Stop();
   void CaptureFrame();

   wkf::Action* mScreenAction;
   wkf::Action* mVideoAction;
   bool         mRecording;
   QTimer*      mTimer;

   PluginUiPointer<PrefWidget> mVideoPrefWidgetPtr;

   QLabel* mStatusLabel;

   int     mBitRate;
   QString mVideoTemplate;
   QString mScreenShotTemplate;
   QString mOutputDir;

   EncoderThread* mEncoderThread;
   bool           mUseMainWindow;
   QWidget*       mCaptureWindow;
   UtWallClock    mClock;
   double         mFrameDeficit;
};
} // namespace VideoCapture
#endif
