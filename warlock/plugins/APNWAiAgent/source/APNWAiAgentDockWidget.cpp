// ****************************************************************************
// CUI
//
// The Advanced Framework for Simulation, Integration, and Modeling (AFSIM)
//
// The use, dissemination or disclosure of data in this file is subject to
// limitation or restriction. See accompanying README and LICENSE for details.
// ****************************************************************************
#include "APNWAiAgentDockWidget.hpp"

APNWAiAgent::DockWidget::DockWidget(QWidget* parent, Qt::WindowFlags flags)
   : QDockWidget(parent, flags)
{
   mUi.setupUi(this);
}
