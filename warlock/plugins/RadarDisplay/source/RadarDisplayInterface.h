#ifndef RADARDISPLAYINTERFACE_HPP
#define RADARDISPLAYINTERFACE_HPP

#include <atomic>
#include <mutex>

#include <QUdpSocket>
#include <QHostAddress>
#include "WkSimInterface.hpp"
#include <QTimer>

namespace RadarDisplay
{
	class RadarDisplayInterface :public warlock::SimInterfaceBase
	{
	public:
		explicit RadarDisplayInterface(const QString& aPluginName);

		double GetSimTime() const noexcept;
		void SimulationInitializing(const WsfSimulation& aSimulation) override;

		void SetSendEnabled() { mSendEnabled = true; }

	protected:
		void WallClockRead(const WsfSimulation& aSimulation) override;
		void SimulationClockRead(const WsfSimulation& aSimulation) override;
	private:
		

		mutable std::mutex   mSimTimeMutex;
		double               mSimTime     = 0;
		std::atomic<bool>    mSendEnabled { true };

		QUdpSocket*  mUdpSocket   = nullptr;
		QHostAddress mSendAddress {  "127.0.0.1"        }; //          "192.168.3.178" 
		quint16      mSendPort    { 12346 };

		QTimer* m_timer;
	};

}
#endif
