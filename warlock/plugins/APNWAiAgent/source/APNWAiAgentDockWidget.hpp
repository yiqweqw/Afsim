// ****************************************************************************
// CUI
//
// The Advanced Framework for Simulation, Integration, and Modeling (AFSIM)
//
// The use, dissemination or disclosure of data in this file is subject to
// limitation or restriction. See accompanying README and LICENSE for details.
// ****************************************************************************
#ifndef APNWAIAGENTDOCKWIDGET_HPP
#define APNWAIAGENTDOCKWIDGET_HPP

#include <QDockWidget>

#include "ui_APNWAiAgentDockWidget.h"

namespace APNWAiAgent
{
class DockWidget : public QDockWidget
{
   Q_OBJECT

public:
   explicit DockWidget(QWidget* parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags());
   ~DockWidget() override = default;

private:
   Ui::APNWAiAgentDockWidget mUi;
};
} // namespace APNWAiAgent

#endif // APNWAIAGENTDOCKWIDGET_HPP
