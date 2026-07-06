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

#ifndef VIDEOCAPTUREPREFWIDGET_HPP
#define VIDEOCAPTUREPREFWIDGET_HPP

#include "VideoCapturePrefObject.hpp"
#include "WkfPrefWidget.hpp"
#include "ui_VideoCapturePrefWidget.h"

namespace VideoCapture
{
class PrefWidget : public wkf::PrefWidgetT<PrefObject>
{
   Q_OBJECT

public:
   PrefWidget();
   ~PrefWidget() override = default;

private:
   void ChooseDir();
   void ReadPreferenceData(const PrefData& aPrefData) override;
   void WritePreferenceData(PrefData& aPrefData) override;

   Ui::VideoCapturePrefWidget mUi;
};
} // namespace VideoCapture
#endif