// RadarDisplayPlugin.cpp
#include "RadarDisplayPlugin.hpp"
#include <QApplication>
#include <QMainWindow>
#include <QDebug>

WKF_PLUGIN_DEFINE_SYMBOLS(RadarDisplay::Plugin, "RadarDisplay",
    "UDP-only plugin with virtual data sender", "warlock")

    namespace RadarDisplay
{
    Plugin::Plugin(const QString& aPluginName, const size_t aUniqueId)
        : warlock::PluginT<RadarDisplayInterface>(aPluginName, aUniqueId)
    {
        QMainWindow* mainWindowPtr = nullptr;
        for (QWidget* widget : qApp->topLevelWidgets()) {
            if (QMainWindow* mainWin = qobject_cast<QMainWindow*>(widget)) {
                mainWindowPtr = mainWin;
                break;
            }
        }

        if (!mainWindowPtr) {
            qDebug() << "RadarDisplay: Failed to get main window";
            return;
        }

        rdWidget = new RadarDisplayWidget(mainWindowPtr);
        rdWidget->setVisible(false); // ÍêÈ«Òþ²Ø£¬ºóÌ¨ÔËÐÐ
        rdWidget->sendDataOnce();

        //connect(rdWidget, &RadarDisplayWidget::sendTriggered,
        //        [this]() { mInterfacePtr->SetSendEnabled(true); });

        qDebug() << "RadarDisplay UDP-only plugin loaded, virtual data sender started (1 Hz)";
    }
}