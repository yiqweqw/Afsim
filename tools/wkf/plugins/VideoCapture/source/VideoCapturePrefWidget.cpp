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
#include "VideoCapturePrefWidget.hpp"

#include "VideoCapturePrefObject.hpp"
#include "WkfEnvironment.hpp"

VideoCapture::PrefWidget::PrefWidget()
   : wkf::PrefWidgetT<PrefObject>()
{
   mUi.setupUi(this);
   connect(mUi.mChooseDir, &QPushButton::clicked, this, &PrefWidget::ChooseDir);

   // Limit characters to alphanumeric, space, underscore and dash, even though special characters are technically valid
   QRegExp     rx("^[\\w\\- ]+$");
   QValidator* validator = new QRegExpValidator(rx, this);
   mUi.mScreenShotTemplate->setValidator(validator);
   mUi.mVideoTemplate->setValidator(validator);
}

void VideoCapture::PrefWidget::WritePreferenceData(PrefData& aPrefData)
{
   aPrefData.mOutputDir          = mUi.mOutputDir->text();
   aPrefData.mScreenshotTemplate = mUi.mScreenShotTemplate->text();
   aPrefData.mVideoTemplate      = mUi.mVideoTemplate->text();
   aPrefData.mBitRate            = mUi.mBitRate->currentText();
   aPrefData.mCaptureSource      = mUi.mCaptureSource->currentText();
}

void VideoCapture::PrefWidget::ReadPreferenceData(const PrefData& aPrefData)
{
   mUi.mOutputDir->setText(aPrefData.mOutputDir);
   mUi.mScreenShotTemplate->setText(aPrefData.mScreenshotTemplate);
   mUi.mVideoTemplate->setText(aPrefData.mVideoTemplate);
   mUi.mBitRate->setCurrentText(aPrefData.mBitRate);
   mUi.mCaptureSource->setCurrentText(aPrefData.mCaptureSource);
}

void VideoCapture::PrefWidget::ChooseDir()
{
   QString dir = wkf::getExistingDirectory(this, "Choose Screenshot/Video Output Directory", mUi.mOutputDir->text());
   if (!dir.isEmpty())
   {
      mUi.mOutputDir->setText(dir);
   }
}
