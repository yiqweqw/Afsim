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
#include "VideoCapturePrefObject.hpp"

#include <QSettings>

#include "UtPath.hpp"
#include "UtRunEnvManager.hpp"
#include "WkfEnvironment.hpp"

VideoCapture::PrefObject::PrefObject(QObject* parent)
   : wkf::PrefObjectT<PrefData>(parent, "VideoCapture")
{
}

void VideoCapture::PrefObject::Apply()
{
   emit OutputDirChanged(mCurrentPrefs.mOutputDir);
   emit ScreenShotTemplateChanged(mCurrentPrefs.mScreenshotTemplate);
   emit VideoTemplateChanged(mCurrentPrefs.mVideoTemplate);
   emit BitRateChanged(mCurrentPrefs.mBitRate);
   emit CaptureSourceChanged(mCurrentPrefs.mCaptureSource);
   emit PreferencesChanged(QVariant::fromValue(mCurrentPrefs));
}

VideoCapture::PrefData VideoCapture::PrefObject::ReadSettings(QSettings& aSettings) const
{
   PrefData prefs;
   auto&    defaults = mDefaultPrefs;

   prefs.mOutputDir          = aSettings.value("outputDir", defaults.mOutputDir).toString();
   prefs.mScreenshotTemplate = aSettings.value("screenshotTemplate", defaults.mScreenshotTemplate).toString();
   prefs.mVideoTemplate      = aSettings.value("videoTemplate", defaults.mVideoTemplate).toString();
   prefs.mBitRate            = aSettings.value("bitRate", defaults.mBitRate).toString();
   prefs.mCaptureSource      = aSettings.value("captureSource", defaults.mCaptureSource).toString();

   return prefs;
}

void VideoCapture::PrefObject::SaveSettingsP(QSettings& aSettings) const
{
   aSettings.setValue("outputDir", mCurrentPrefs.mOutputDir);
   aSettings.setValue("screenshotTemplate", mCurrentPrefs.mScreenshotTemplate);
   aSettings.setValue("videoTemplate", mCurrentPrefs.mVideoTemplate);
   aSettings.setValue("bitRate", mCurrentPrefs.mBitRate);
   aSettings.setValue("captureSource", mCurrentPrefs.mCaptureSource);
}
