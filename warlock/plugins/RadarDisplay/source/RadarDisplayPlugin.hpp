// RadarDisplayPlugin.hpp
#ifndef RADARDISPLAYPLUGIN_HPP
#define RADARDISPLAYPLUGIN_HPP

#include "WkPlugin.hpp"
#include "RadarDisplayWidget.hpp"
#include "RadarDisplayInterface.h"
namespace RadarDisplay
{
    class Plugin : public warlock::PluginT<RadarDisplay::RadarDisplayInterface>
    {
    public:
        explicit Plugin(const QString& aPluginName, const size_t aUniqueId);
        ~Plugin() override = default;

    private:
        RadarDisplayWidget* rdWidget = nullptr;
    };
}

#endif // RADARDISPLAYPLUGIN_HPP