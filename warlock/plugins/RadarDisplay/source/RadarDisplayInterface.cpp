#include "RadarDisplayInterface.h"

#include "UtMemory.hpp"
#include "WsfSimulation.hpp"
#include "WsfPlatformTypes.hpp"
#include "WsfScenario.hpp"
#include <QDebug>
#include "TricorderSensor.hpp" 
#include "WsfPassiveReconSensor.hpp"
#include "RadarModel.cpp"
#include "WsfSensor.hpp"
#include "WsfLocalTrack.hpp"
#include "WsfTrackList.hpp"
#include <cstdint>
#include <cmath>
#include <unordered_map>

namespace
{
std::unordered_map<const WsfPassiveReconSensor*, uint64_t>& ReconLastSentVersionMap()
{
	static std::unordered_map<const WsfPassiveReconSensor*, uint64_t> sReconLastSentVersion;
	return sReconLastSentVersion;
}
}
RadarDisplay::RadarDisplayInterface::RadarDisplayInterface(const QString& aPluginName)
	: warlock::SimInterfaceBase(aPluginName)
{
	mUdpSocket = new QUdpSocket();
	m_timer = new QTimer(this);
	connect(m_timer, &QTimer::timeout, this, &RadarDisplayInterface::SetSendEnabled);
	m_timer->start(1000);
}

double RadarDisplay::RadarDisplayInterface::GetSimTime() const noexcept
{
	std::lock_guard<std::mutex> l{ mSimTimeMutex };
	return mSimTime;
}

void RadarDisplay::RadarDisplayInterface::SimulationInitializing(const WsfSimulation& aSimulation)
{
	ReconLastSentVersionMap().clear();
	std::vector<WsfPlatform*> currentTypes;
	const WsfPlatformTypes& platformTypes = aSimulation.GetScenario().GetPlatformTypes();
	platformTypes.GetCurrentTypes(currentTypes);
	for (const auto* base : currentTypes)
	{
		auto side = QString::fromStdString(base->GetSide());

		for (const auto& category : base->GetCategories().GetCategoryList())
		{
			auto categoryStr = QString::fromStdString(category);
			 
		}
	}

}

void RadarDisplay::RadarDisplayInterface::SimulationClockRead(const WsfSimulation& aSimulation)
{
	std::lock_guard<std::mutex> l{ mSimTimeMutex };
	mSimTime = aSimulation.GetSimTime();
}

void RadarDisplay::RadarDisplayInterface::WallClockRead(const WsfSimulation& aSimulation)
{
	if (!mSendEnabled)
	{		
		return;
	}
		
	mSendEnabled = false;
	for (int platformIdx = 0; platformIdx < aSimulation.GetPlatformCount(); ++platformIdx)
	{
		WsfPlatform* plat = aSimulation.GetPlatformEntry(platformIdx);
		if (plat == nullptr)
		{
			continue;
		}

		const unsigned int sensorCount = plat->GetComponentCount<WsfSensor>();
		for (unsigned int sensorIdx = 0; sensorIdx < sensorCount; ++sensorIdx)
		{
			WsfSensor* sensor = plat->GetComponentEntry<WsfSensor>(sensorIdx);
			if (sensor == nullptr)
			{
				continue;
			}

			TricorderSensor* sen = dynamic_cast<TricorderSensor*>(sensor);
			if (sen != nullptr)
			{
				mUdpSocket->writeDatagram(QByteArray::fromStdString("start Send"), mSendAddress, mSendPort);
				//{"1001","平台名称","雷达名称","经度","纬度","高度"}
				std::ostringstream oss;
				oss << "1001"
					<< "," << plat->GetName()
					<< "," << sen->GetName()
					<< "," << plat->GetLocationLLA().mLon
					<< "," << plat->GetLocationLLA().mLat
					<< "," << plat->GetLocationLLA().mAlt
					<< "," << plat->GetSpeed()
					;
				mUdpSocket->writeDatagram(QByteArray::fromStdString(oss.str()), mSendAddress, mSendPort);
				mUdpSocket->writeDatagram(QByteArray::fromStdString(sen->radar->UdpInterfaceSensorVariables()), mSendAddress, mSendPort);
				mUdpSocket->writeDatagram(QByteArray::fromStdString(sen->radar->UdpInterfaceSensorBeamWidth()), mSendAddress, mSendPort);
				for (int trackIdx = 0; trackIdx < plat->GetMasterTrackList().GetTrackCount(); ++trackIdx)
				{
					WsfLocalTrack *track = plat->GetMasterTrackList().GetTrackEntry(trackIdx);
					std::ostringstream trackOss;
					double lon, lat, alt;
					track->GetLocationLLA(lat, lon, alt);
					trackOss << "1004"
						<< "," << track->GetTargetName()
						<< "," << lon
						<< "," << lat
						<< "," << alt
						<< "," << track->GetSpeed()
						;
					mUdpSocket->writeDatagram(QByteArray::fromStdString(trackOss.str()), mSendAddress, mSendPort);
				}
				
				mUdpSocket->writeDatagram(QByteArray::fromStdString(sen->radar->UdpInterfaceSensorDetectMaxRange()), mSendAddress, mSendPort);
				mUdpSocket->writeDatagram(QByteArray::fromStdString(sen->radar->UdpInterfaceSensorDetectMaxRangePjn()), mSendAddress, mSendPort);
				mUdpSocket->writeDatagram(QByteArray::fromStdString(sen->radar->UdpInterfaceSensorScanRange()), mSendAddress, mSendPort);			
				mUdpSocket->writeDatagram(QByteArray::fromStdString(sen->radar->UdpInterfaceSensorBeamSpeed()), mSendAddress, mSendPort);
				std::ostringstream time;
				time << "1009"
					<< "," << aSimulation.GetSimTime();

				mUdpSocket->writeDatagram(QByteArray::fromStdString(time.str()), mSendAddress, mSendPort);
				mUdpSocket->writeDatagram(QByteArray::fromStdString(sen->radar->UdpInterfaceSensorantennaPatternAgin()), mSendAddress, mSendPort);
				mUdpSocket->writeDatagram(QByteArray::fromStdString("end Send"), mSendAddress, mSendPort);
				continue;
			}

			WsfPassiveReconSensor* reconSensor = dynamic_cast<WsfPassiveReconSensor*>(sensor);
			if (reconSensor == nullptr)
			{
				continue;
			}

			

			mUdpSocket->writeDatagram(QByteArray::fromStdString("start Send"), mSendAddress, mSendPort);
			// {"1030","平台名称","侦察名称"}
			std::ostringstream platformOss;
			platformOss << "1030"
				<< "," << plat->GetName()
				<< "," << reconSensor->GetName();
			mUdpSocket->writeDatagram(QByteArray::fromStdString(platformOss.str()), mSendAddress, mSendPort);

			// {"1031",...侦察参数...}
			mUdpSocket->writeDatagram(QByteArray::fromStdString(reconSensor->UdpInterfaceReconSensorVariables()), mSendAddress, mSendPort);

			

			// {"1035","时间"}
			std::ostringstream simTimeOss;
			simTimeOss << "1035"
				<< "," << aSimulation.GetSimTime();
			mUdpSocket->writeDatagram(QByteArray::fromStdString(simTimeOss.str()), mSendAddress, mSendPort);

			mUdpSocket->writeDatagram(QByteArray::fromStdString("end Send"), mSendAddress, mSendPort);
			

			const uint64_t currentReconVersion = reconSensor->UdpInterfaceReconDataVersion();
			auto& reconLastSentVersion = ReconLastSentVersionMap();
			const auto versionIt = reconLastSentVersion.find(reconSensor);
			const uint64_t lastSentVersion = (versionIt != reconLastSentVersion.end()) ? versionIt->second : 0U;
			if ((currentReconVersion == 0U) || (currentReconVersion == lastSentVersion))
			{
				continue;
			}
			reconLastSentVersion[reconSensor] = currentReconVersion;
			// {"1032","数组"}
			mUdpSocket->writeDatagram(QByteArray::fromStdString(reconSensor->UdpInterfaceReconAntennaPattern()), mSendAddress, mSendPort);


			// {"1033","TOA","PW","RF","PA","DOA","PulseID","SourceID"}
			const auto pdwList = reconSensor->UdpInterfaceReconPdwList();
			for (const auto& pdw : pdwList)
			{
				if (!std::isfinite(pdw.mToa) || !std::isfinite(pdw.mPw) || !std::isfinite(pdw.mRf) ||
					!std::isfinite(pdw.mPa) || !std::isfinite(pdw.mDoa))
				{
					continue;
				}

				std::ostringstream pdwOss;
				pdwOss << "1033"
					<< "," << pdw.mToa
					<< "," << pdw.mPw
					<< "," << pdw.mRf
					<< "," << pdw.mPa
					<< "," << pdw.mDoa
					<< "," << pdw.mPulseId
					<< "," << pdw.mSourceId;
				mUdpSocket->writeDatagram(QByteArray::fromStdString(pdwOss.str()), mSendAddress, mSendPort);
			}

			// {"1034","辐射源","PRI"}
			const auto emitterPriList = reconSensor->UdpInterfaceReconEmitterPriList();
			for (const auto& emitterPri : emitterPriList)
			{
				if (!std::isfinite(emitterPri.mPriSec))
				{
					continue;
				}
				std::ostringstream priOss;
				priOss << "1034"
					<< "," << emitterPri.mEmitterName
					<< "," << emitterPri.mPriSec;
				mUdpSocket->writeDatagram(QByteArray::fromStdString(priOss.str()), mSendAddress, mSendPort);
			}
			// {"1036",频率，频率类别 固定频率，重复间隔pri，脉宽，功率，信噪比，方位，俯仰，信号类型 }

			const auto unidentifiedEmitterList = reconSensor->UdpInterfaceReconUnidentifiedEmitterList();
			for (const auto& unknownEmitter : unidentifiedEmitterList)
			{
				if (!std::isfinite(unknownEmitter.mLevel) || !std::isfinite(unknownEmitter.mAzimuth) ||
					!std::isfinite(unknownEmitter.mElevation) || !std::isfinite(unknownEmitter.mFrequencyHz) ||
					!std::isfinite(unknownEmitter.mPulseWidthSec) || !std::isfinite(unknownEmitter.mPower) ||
					!std::isfinite(unknownEmitter.mSnr) ||
					!std::isfinite(unknownEmitter.mPriSec))
				{
					continue;
				}
				std::ostringstream unknownOss;
				unknownOss << "1036"
					<< "," << unknownEmitter.mName
					<< "," << unknownEmitter.mFrequencyHz
					<< "," << unknownEmitter.mFreType
					<< "," << unknownEmitter.mPriSec
					<< "," << unknownEmitter.mPulseWidthSec
					<< "," << unknownEmitter.mPower
					<< "," << unknownEmitter.mSnr
					<< "," << unknownEmitter.mAzimuth
					<< "," << unknownEmitter.mElevation
					<< "," << unknownEmitter.mSignalType;
				mUdpSocket->writeDatagram(QByteArray::fromStdString(unknownOss.str()), mSendAddress, mSendPort);
			}

		}
	}
}
