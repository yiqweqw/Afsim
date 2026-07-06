// ****************************************************************************
// CUI
//
// The Advanced Framework for Simulation, Integration, and Modeling (AFSIM)
//
// The use, dissemination or disclosure of data in this file is subject to
// limitation or restriction. See accompanying README and LICENSE for details.
// ****************************************************************************
#ifndef APNWAIAGENTPLUGIN_HPP
#define APNWAIAGENTPLUGIN_HPP

#include "APNWAiAgentDockWidget.hpp"
#include "WkPlugin.hpp"

namespace APNWAiAgent
{
class Plugin : public warlock::Plugin
{
   Q_OBJECT

public:
   Plugin(const QString& aPluginName, size_t aUniqueId);
   ~Plugin() override = default;

private:
   DockWidget* mDockWidgetPtr;
};
} // namespace APNWAiAgent

#endif // APNWAIAGENTPLUGIN_HPP
