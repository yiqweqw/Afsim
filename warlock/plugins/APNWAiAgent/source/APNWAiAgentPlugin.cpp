// ****************************************************************************
// CUI
//
// The Advanced Framework for Simulation, Integration, and Modeling (AFSIM)
//
// The use, dissemination or disclosure of data in this file is subject to
// limitation or restriction. See accompanying README and LICENSE for details.
// ****************************************************************************
#include "APNWAiAgentPlugin.hpp"

#include "WkfEnvironment.hpp"
#include "WkfMainWindow.hpp"

WKF_PLUGIN_DEFINE_SYMBOLS(APNWAiAgent::Plugin,
                          "APNWAiAgent",
                          "Empty APNWAiAgent plugin skeleton.",
                          "warlock")

APNWAiAgent::Plugin::Plugin(const QString& aPluginName, size_t aUniqueId)
   : warlock::Plugin(aPluginName, aUniqueId)
   , mDockWidgetPtr(new DockWidget(wkfEnv.GetMainWindow()))
{
   wkf::MainWindow* mainWindowPtr = wkfEnv.GetMainWindow();
   mainWindowPtr->addDockWidget(Qt::RightDockWidgetArea, mDockWidgetPtr);
   //mDockWidgetPtr->hide();
}
