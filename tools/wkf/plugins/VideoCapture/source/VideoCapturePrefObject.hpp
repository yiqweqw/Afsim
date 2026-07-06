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
#ifndef VIDEOCAPTUREPREFOBJECT_HPP
#define VIDEOCAPTUREPREFOBJECT_HPP

#include <QDir>

#include "WkfPrefObject.hpp"

namespace VideoCapture
{
struct PrefData
{
   QString mOutputDir{QDir::home().absolutePath()}; // find a user directory
   QString mScreenshotTemplate{"screen"};
   QString mVideoTemplate{"video"};
   QString mBitRate{"Medium"};
   QString mCaptureSource{"Main Window"};
};

class PrefObject : public wkf::PrefObjectT<PrefData>
{
   Q_OBJECT

   friend class PrefWidget;

public:
   PrefObject(QObject* parent = nullptr);

signals:
   void OutputDirChanged(const QString& aDir);
   void ScreenShotTemplateChanged(const QString& aTemplate);
   void VideoTemplateChanged(const QString& aTemplate);
   void BitRateChanged(const QString& aBitRate);
   void CaptureSourceChanged(const QString& aSrc);

private:
   void     Apply() override;
   PrefData ReadSettings(QSettings& aSettings) const override;
   void     SaveSettingsP(QSettings& aSettings) const override;
};
} // namespace VideoCapture

Q_DECLARE_METATYPE(VideoCapture::PrefData)
#endif
