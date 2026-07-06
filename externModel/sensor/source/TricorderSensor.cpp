// ****************************************************************************
// CUI
//
// The Advanced Framework for Simulation, Integration, and Modeling (AFSIM)
//
// Copyright 2003-2015 The Boeing Company. All rights reserved.
//
// The use, dissemination or disclosure of data in this file is subject to
// limitation or restriction. See accompanying README and LICENSE for details.
// ****************************************************************************
// ****************************************************************************
// Updated by Infoscitex, a DCS Company
// ****************************************************************************

#include "TricorderSensor.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_map>

// NOTE: We deliberately do NOT include any specific jammer header (e.g.
// PhaserWeapon.hpp). Jammers are discovered polymorphically through the
// receiver's interference-interactor list, which returns WsfEM_Xmtr* — the
// abstract base shared by all jammer types. Mirrors stock WsfRadarSensor.cpp.
#include "UtCast.hpp"
#include "UtEntity.hpp"
#include "UtInput.hpp"
#include "UtInputBlock.hpp"
#include "UtLog.hpp"
#include "UtMath.hpp"
#include "UtMemory.hpp"
#include "UtRandom.hpp"
#include "UtStringUtil.hpp"
#include "UtVec3.hpp"
#include "UtWallClock.hpp"
#include "WsfAuxDataEnabled.hpp"
#include "WsfComponent.hpp"
#include "WsfDefaultSensorScheduler.hpp"
#include "WsfDefaultSensorTracker.hpp"
#include "WsfEM_Clutter.hpp"
#include "WsfEM_ClutterTypes.hpp"
#include "WsfEW_Result.hpp"
#include "WsfEnvironment.hpp"
#include "WsfObject.hpp"
#include "WsfEM_Antenna.hpp"
#include "WsfPlatform.hpp"
#include "WsfRadarSensorErrorModel.hpp"
#include "WsfRadarSignature.hpp"
#include "WsfScenario.hpp"
#include "WsfSensorComponent.hpp"
#include "WsfSensorModeList.hpp"
#include "WsfSensorObserver.hpp"
#include "WsfSimulation.hpp"
#include "WsfSimulationInput.hpp"
#include "WsfSinglePlatformObserver.hpp"
#include "WsfStandardSensorErrorModel.hpp"
#include "WsfStringId.hpp"
#include "WsfEW_EA_EP.hpp"
#include "WsfEW_EA.hpp"

namespace {
std::string sLastImplicitBeamCommand;
}

// =================================================================================================
TricorderSensor::TricorderSensor(WsfScenario &aScenario)
    : WsfSensor(aScenario) 
      ,
      mRadarModeList(), mXmtrList(), mRcvrList(), mAnyModeCanTransmit(true),
      mAnyModeCanReceive(true), mTempGeometryPtr(nullptr) {
  SetClass(cACTIVE | cRADIO); // This is an active RF sensor.
  // Create the mode list with the sensor-specific mode template.
  SetModeList(
      ut::make_unique<WsfSensorModeList>(new RadarMode())); //???????????????

  // Assign the default sensor scheduler and tracker
  SetScheduler(ut::make_unique<WsfDefaultSensorScheduler>());      //????????
  SetTracker(ut::make_unique<WsfDefaultSensorTracker>(aScenario)); //????????
  radar = new RadarModel();
}

// =================================================================================================
TricorderSensor::TricorderSensor(const TricorderSensor &aSrc)
	: WsfSensor(aSrc), mRadarModeList(), mXmtrList(), mRcvrList(),
	mAnyModeCanTransmit(aSrc.mAnyModeCanTransmit),
	mAnyModeCanReceive(aSrc.mAnyModeCanReceive), mTempGeometryPtr(nullptr),
	mRadarT0_(aSrc.mRadarT0_),
	mRadarPulseNum_(aSrc.mRadarPulseNum_),
	mRadarPfa_(aSrc.mRadarPfa_),
	mRadarRangeAccuracy_(aSrc.mRadarRangeAccuracy_),
	mRadarAzimuthAccuracy_(aSrc.mRadarAzimuthAccuracy_)
{
  radar = new RadarModel();
}

TricorderSensor::~TricorderSensor() noexcept {}

// =================================================================================================
// virtual
WsfSensor *TricorderSensor::Clone() const { return new TricorderSensor(*this); }

// =================================================================================================
// virtual
size_t TricorderSensor::GetEM_RcvrCount() const {
  size_t count = 0;
  if (IsTurnedOn()) {
    count = mRcvrList.size();
  } else if (!mRadarModeList.empty()) {
    RadarMode *modePtr = mRadarModeList[mModeListPtr->GetCurrentMode()];
    count = modePtr->mBeamList.size();
  }
  return count;
}

// =================================================================================================
// virtual
WsfEM_Rcvr &TricorderSensor::GetEM_Rcvr(size_t aIndex) const {
  if (IsTurnedOn()) {
    if (aIndex < mRcvrList.size()) {
      return *(mRcvrList[aIndex]);
    }
  } else if (!mRadarModeList.empty()) {
    RadarMode *modePtr = mRadarModeList[mModeListPtr->GetCurrentMode()];
    if (aIndex < modePtr->mBeamList.size()) {
      return *modePtr->mBeamList[aIndex]->mRcvrPtr;
    }
  }
  return WsfSensor::GetEM_Rcvr(aIndex);
}

// =================================================================================================
// virtual
size_t TricorderSensor::GetEM_RcvrCount(size_t aModeIndex) const {
  size_t count = 0;
  if (aModeIndex < mRadarModeList.size()) {
    RadarMode *modePtr = mRadarModeList[aModeIndex];
    count = modePtr->mBeamList.size();
  }
  return count;
}

// =================================================================================================
// virtual
WsfEM_Rcvr &TricorderSensor::GetEM_Rcvr(size_t aModeIndex,
                                        size_t aIndex) const {
  if (aModeIndex < mRadarModeList.size()) {
    RadarMode *modePtr = mRadarModeList[aModeIndex];
    if (aIndex < modePtr->mBeamList.size()) {
      return *modePtr->mBeamList[aIndex]->mRcvrPtr;
    }
  }
  return WsfSensor::GetEM_Rcvr(aModeIndex, aIndex); // return dummy xmtr
}

// =================================================================================================
// virtual
size_t TricorderSensor::GetEM_XmtrCount() const {
  size_t count = 0;
  if (IsTurnedOn()) {
    count = mXmtrList.size();
  } else if (!mRadarModeList.empty()) {
    RadarMode *modePtr = mRadarModeList[mModeListPtr->GetCurrentMode()];
    count = modePtr->mBeamList.size();
  }
  return count;
}

// =================================================================================================
// virtual
WsfEM_Xmtr &TricorderSensor::GetEM_Xmtr(size_t aIndex) const {
  if (IsTurnedOn()) {
    if (aIndex < mXmtrList.size()) {
      return *(mXmtrList[aIndex]);
    }
  } else if (!mRadarModeList.empty()) {
    RadarMode *modePtr = mRadarModeList[mModeListPtr->GetCurrentMode()];
    if (aIndex < modePtr->mBeamList.size()) {
      return *modePtr->mBeamList[aIndex]->mXmtrPtr;
    }
  }
  return WsfSensor::GetEM_Xmtr(aIndex);
}

// =================================================================================================
// virtual
size_t TricorderSensor::GetEM_XmtrCount(size_t aModeIndex) const {
  return GetEM_RcvrCount(aModeIndex);
}

// =================================================================================================
// virtual
WsfEM_Xmtr &TricorderSensor::GetEM_Xmtr(size_t aModeIndex,
                                        size_t aIndex) const {
  if (aModeIndex < mRadarModeList.size()) {
    RadarMode *modePtr = mRadarModeList[aModeIndex];
    if (aIndex < modePtr->mBeamList.size()) {
      return *modePtr->mBeamList[aIndex]->mXmtrPtr;
    }
  }
  return WsfSensor::GetEM_Xmtr(aModeIndex, aIndex); // return dummy xmtr
}

// =================================================================================================
//! An internal method that is called to update the lists for GetEM_Xmtr/Rcvr.
//! This is called whenever modes are selected or deselected. It updates the
//! lists of active transmitters and receivers that are accessed by the
//! GetEM_Rcvr and GetEM_Xmtr methods.
// virtual
void TricorderSensor::UpdateXmtrRcvrLists() {
  mXmtrList.clear();
  mRcvrList.clear();
  for (RadarMode *modePtr : mRadarModeList) {
    if (modePtr->IsSelected()) {
      for (size_t beamIndex = 0; beamIndex < modePtr->mBeamList.size();
           ++beamIndex) {
        RadarBeam &beam = *modePtr->mBeamList[beamIndex];
        if (modePtr->mCanTransmit) {
          mXmtrList.push_back(beam.GetEM_Xmtr());
        }
        if (modePtr->mCanReceive) {
          mRcvrList.push_back(beam.GetEM_Rcvr());
        }
      }
    }
  }
}

void TricorderSensor::RadarBeam::calculateTargetAngle(
    Eigen::Vector3d &position_, Eigen::Vector3d &tgtPos, double &az,
    double &el) {
  static constexpr double PI = 3.14159265358979323846;
  double lat1Rad = position_[1] * PI / 180.0;
  double lon1Rad = position_[0] * PI / 180.0;
  double lat2Rad = tgtPos[1] * PI / 180.0;
  double lon2Rad = tgtPos[0] * PI / 180.0;

  // ????????????????
  double deltaLon = lon2Rad - lon1Rad;
  double y = sin(deltaLon) * cos(lat2Rad);
  double x =
      cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(deltaLon);
  az = atan2(y, x);
  /*static constexpr double TWO_PI = 2.0 * PI;
  az = std::fmod(az, TWO_PI);
  if (az < 0) {
  az += TWO_PI;
  }*/

  // ???????????????????????????????
  static constexpr double EARTH_RADIUS = 6371000.0; // ??????????????

  double centralAngle = acos(sin(lat1Rad) * sin(lat2Rad) +
                             cos(lat1Rad) * cos(lat2Rad) * cos(deltaLon));
  double horizontalDistance = centralAngle * EARTH_RADIUS;

  // ????????????????
  double deltaAlt = tgtPos[2] - position_[2];
  el = atan2(deltaAlt, horizontalDistance);
  /*el = std::fmod(el, TWO_PI);
  if (el < 0) {
  el += TWO_PI;
  }*/
}
double TricorderSensor::RadarBeam::calculationDetectionProbability(
    double aSimTime, WsfPlatform *aTargetPtr, Settings &aSettings,
    WsfEM_Xmtr *aXmtrPtr, WsfSensorResult &aResult) {
	TricorderSensor *ownerSensor = static_cast<TricorderSensor *>(GetSensorMode()->GetSensor());
	WsfEM_Antenna *a = aXmtrPtr->GetAntenna();
	WsfEM_Rcvr* rcv=aXmtrPtr->GetLinkedReceiver();
  std::map<std::string, double> config;
  config["peakPower_"] = aXmtrPtr->GetPower();  
  config["fc"] = aXmtrPtr->GetFrequency();
  config["bandwidth"] = aXmtrPtr->GetBandwidth();  
  config["bandwidthRx_"] = aResult.GetReceiver()->GetBandwidth();
  config["antennaGain"] = UtMath::LinearToDB(aXmtrPtr->GetPeakAntennaGain());
  config["pulseWidth"] = aXmtrPtr->GetPulseWidth();
  config["PRF"] = aXmtrPtr->GetPulseRepetitionFrequency();
  config["Fn"] = UtMath::LinearToDB(rcv->GetInternalLoss());
  config["L_"] = UtMath::LinearToDB(rcv->GetNoiseFigure());
  config["Pd"] = GetSensorMode()->GetRequiredPd();
  config["T0"] = ownerSensor->mRadarT0_;
  config["pulseNum"] = ownerSensor->mRadarPulseNum_;
  config["Pfa"] = ownerSensor->mRadarPfa_;
  config["rangeAccuracy"] = ownerSensor->mRadarRangeAccuracy_;
  config["azimuthAccuracy"] = ownerSensor->mRadarAzimuthAccuracy_;
  
  
  
  
  double az, az1;

  a->GetAzimuthScanLimits(az, az1);
  az = az * UtMath::cDEG_PER_RAD;
  az1 = az1 * UtMath::cDEG_PER_RAD;

  double el, el1;
  a->GetElevationScanLimits(el, el1);
  el = el * UtMath::cDEG_PER_RAD;
  el1 = el1 * UtMath::cDEG_PER_RAD;
  //config["beamwidthAz_"] = az1 - az;
 // config["beamwidthEl_"] = el1 - el; 
 
  //std::cout << "antennam paterrn az beam: " << aXmtrPtr->GetAzimuthBeamwidth() << std::endl;
  config["beamwidthAz_"] = aXmtrPtr->GetAzimuthBeamwidth()*UtMath::cDEG_PER_RAD ;
  config["beamwidthEl_"] = aXmtrPtr->GetElevationBeamwidth() *UtMath::cDEG_PER_RAD;
   
  
  // Apply script-configurable parameters stored on TricorderSensor.
  // These values are runtime-invariant — apply ONCE per beam, not per detection chance.
 
  
  if (!mConfigApplied) {
    ownerSensor->radar->applyConfig(config);
    /*ownerSensor->radar->setAntennaGain(ownerSensor->mRadarAntennaGain_);
    ownerSensor->radar->setPulseWidth(ownerSensor->mRadarPulseWidth_);
    ownerSensor->radar->setFn(ownerSensor->mRadarFn_);
    ownerSensor->radar->setT0(ownerSensor->mRadarT0_);
    ownerSensor->radar->setL(ownerSensor->mRadarL_);
    ownerSensor->radar->setPulseNum(ownerSensor->mRadarPulseNum_);
    ownerSensor->radar->setPfa(ownerSensor->mRadarPfa_);
    ownerSensor->radar->setRangeAccuracy(ownerSensor->mRadarRangeAccuracy_);
    ownerSensor->radar->setAzimuthAccuracy(ownerSensor->mRadarAzimuthAccuracy_);*/
    mConfigApplied = true;
  }

  // Per-frame state: platform kinematics.
  ut::coords::LLA out = GetPlatform()->GetLocationLLA();
  ut::coords::ECS platformECS = GetPlatform()->GetVelocityECS();
  ownerSensor->radar->setPosition(out.mLon, out.mLat, out.mAlt);
  ownerSensor->radar->setVelocity(platformECS[0], platformECS[1], platformECS[2]);
  ut::coords::LLA targetLLA = aTargetPtr->GetLocationLLA();
  ut::coords::ECS targetECS = aTargetPtr->GetVelocityECS();
  std::shared_ptr<Target> target = std::make_shared<Target>(
      Eigen::Vector3d(targetLLA.mLon, targetLLA.mLat, targetLLA.mAlt),
      Eigen::Vector3d(targetECS[0], targetECS[1], targetECS[2]),
      aResult.mRadarSig, aTargetPtr->GetName());
  //std::cout << "detection pridition:" << aTargetPtr->GetName() << std::endl;
  double h1, p1, r1;
  GetPlatform()->GetOrientationNED(h1, p1, r1);
  double yaw = UtMath::NormalizeAngleRad(GetSensorMode()->GetSensor()->GetYaw() + h1, -UtMath::cPI) ; // 3.1415926 * 180;

  //az = std::fmod(az + yaw, 360);
  az1 = std::fmod(az1 + yaw, 360);

  //az = az * 3.1415926 / 180;
  az1 = az1 *UtMath::cRAD_PER_DEG; 

  double center_rad = atan2(sin(az) + sin(az1), cos(az) + cos(az1));
  double center_deg = center_rad * UtMath::cDEG_PER_RAD;
  center_deg = std::fmod(center_deg, 360); // center_deg 
  if (center_deg < 0)
    center_deg += 360;

  center_deg = yaw * UtMath::cDEG_PER_RAD;  //雷达天线波束的角度
  ownerSensor->radar->update(aSimTime, center_deg, 0);

   //std::cout << "angle limit  " << center_deg  << std::endl;
  //  Steering angles for the simplified jammer power model.
  //  `RadarModel::update` updates only azimuth beamDirection_[0], so use
  //  center_deg as steerAz.


  std::map<int, std::shared_ptr<Jammer>> jammers;

  // ===================================================================================
  // Polymorphic jammer discovery.
  //
  // We enumerate jammers through the receiver's interference-interactor list, which is
  // populated by EVERY jammer type (stock WSF_RF_JAMMER, PHASER_WEAPON, and any future
  // jammer plugin) as long as that jammer constructs its transmitter with function
  // WsfEM_Xmtr::cXF_INTERFERER and registers via WsfEM_Rcvr::AddInteractor. The radar
  // therefore has NO compile-time dependency on any concrete jammer class.
  //
  // For runtime population, we only read state available on the abstract WsfEM_Xmtr* /
  // WsfPlatform* / WsfEM_Antenna* interfaces. Any jammer-technique parameters that are
  // *specific* to a particular jammer subclass (e.g. PhaserWeapon's KJC / KJI / fake-
  // target count) are defaulted to neutral values here. If you need to consume jammer-
  // technique data polymorphically, the right pattern is to add virtual accessors on
  // WsfRF_Jammer (or expose them on a small interface implemented by every jammer) and
  // call them via the base pointer — NOT to dynamic_cast to a specific subclass here.
  // ===================================================================================
  WsfEM_Rcvr* rcvrPtr = aResult.GetReceiver();
  const size_t interactorCount = (rcvrPtr != nullptr) ? rcvrPtr->GetInterferenceInteractorCount() : 0;

  for (size_t i = 0; i < interactorCount; ++i)
  {
    WsfEM_Xmtr* jammerXmtr = rcvrPtr->GetInterferenceInteractorEntry(i);
    if (jammerXmtr == nullptr) { continue; }
    WsfPlatform* jammerPlatform = jammerXmtr->GetPlatform();
    if (jammerPlatform == nullptr) { continue; }

    
    auto jammerPtr = std::make_shared<Jammer>();

    // Kinematics / identity --------------------------------------------------------
    const ut::coords::LLA lla = jammerPlatform->GetLocationLLA();
    jammerPtr->position     = Eigen::Vector3d(lla.mLon, lla.mLat, lla.mAlt);
    jammerPtr->platformName = jammerPlatform->GetName();
    double heading = 0.0, pitch = 0.0, roll = 0.0;
    jammerPlatform->GetOrientationNED(heading, pitch, roll);
    jammerPtr->heading          = heading;
    // Presence in the receiver's interference-interactor list already implies
    // the jammer is registered/radiating — WsfRF_Jammer adds/removes the xmtr
    // on TurnOn/TurnOff.
	jammerPtr->isturnOn = jammerPlatform->GetComponentEntry<WsfWeapon>(0)->IsTurnedOn();
	
	
    // Waveform ---------------------------------------------------------------------
    jammerPtr->fj      = jammerXmtr->GetFrequency();
    jammerPtr->Bj      = jammerXmtr->GetBandwidth();
    jammerPtr->Lj      = UtMath::LinearToDB(  jammerXmtr->GetInternalLoss());
    jammerPtr->EqPower = jammerXmtr->GetPeakPower();

    // Antenna ----------------------------------------------------------------------
    jammerPtr->antennaGain   = jammerXmtr->GetPeakAntennaGain();
    jammerPtr->beamDirection = Eigen::Vector2d(0.0, 0.0);
    if (WsfEM_Antenna* ant = jammerXmtr->GetAntenna())
    {
      const double minScanLimit = ant->GetMinimumAzimuthScanLimit();
      const double maxScanLimit = ant->GetMaximumAzimuthScanLimit();
      const double rawBeamWidth = maxScanLimit - minScanLimit;
      const double kTwoPi = 2.0 * UtMath::cPI;
      const double kTol = 1e-9;

      // Preserve 360deg coverage from the original antenna limits.
      if (std::abs(std::abs(rawBeamWidth) - kTwoPi) < kTol) {
        jammerPtr->mBeamWidthAz = kTwoPi;
      } else {
        double normalizedWidth = std::fmod(rawBeamWidth, kTwoPi);
        if (normalizedWidth < 0.0) {
          normalizedWidth += kTwoPi;
        }
        jammerPtr->mBeamWidthAz = normalizedWidth;
      }
		
	  jammerPtr->mBeamMinScanAz = UtMath::NormalizeAngleRad(minScanLimit + jammerPtr->heading, -UtMath::cPI);  
      jammerPtr->mBeamMaxScanAz = UtMath::NormalizeAngleRad(maxScanLimit + jammerPtr->heading,-UtMath::cPI);// 水平俯仰角度平台与波束叠加
	  double rawNorm = fmod(maxScanLimit + jammerPtr->heading, 2 * UtMath::cPI);
	  if (rawNorm < 0) rawNorm += 2 * UtMath::cPI;
	  if (fabs(rawNorm - UtMath::cPI) < 1e-9) {
		  jammerPtr->mBeamMaxScanAz = UtMath::cPI;
	  }
    }
    jammerPtr->xmtr = jammerXmtr;

    // Jammer-technique parameters NOT on the abstract interface — neutral defaults.
    jammerPtr->KJC           = 1.0;
    jammerPtr->KJI           = 1.0;
    jammerPtr->FakeTargetNum = 1;
    	

	WsfEW_EA_EP* eaPtr = WsfEW_EA::GetElectronicAttack(*jammerXmtr);
	const WsfEW_Types::TechniqueIdVec& eaTechniquesToCounter = eaPtr->GetTechniquesInUse();
	for (WsfEW_Types::TechniqueId EA_TechId : eaTechniquesToCounter)
	{
		WsfEW_Technique* techPtr = eaPtr->GetTechnique(EA_TechId);
		jammerPtr->type = techPtr->GetMitigationClassId();
	}
    jammers[static_cast<int>(jammerPlatform->GetIndex())] = jammerPtr;
  }

  
  double  Pjn = ownerSensor->radar->calculatePjn(aSimTime, jammers, aXmtrPtr, aResult);
  double antenna = aXmtrPtr->GetAntennaGain(
      aXmtrPtr->GetPolarization(), aXmtrPtr->GetFrequency(),
      aResult.mXmtrBeam.mAz, aResult.mXmtrBeam.mEl, aResult.mXmtrBeam.mEBS_Az,
      aResult.mXmtrBeam.mEBS_El);
 
  double pd=0;
  bool fakeDetected =
      ownerSensor->radar->detectFakeTarget(aSimTime, jammers, Pjn, aXmtrPtr, aResult,pd,target);
  TricorderSensor *sensorPtr =
	  static_cast<TricorderSensor *>(GetSensorMode()->GetSensor());
  

  if (std::fabs(ownerSensor->radar->getBeamDirectionAz()) < 1e-9) {
	  sensorPtr->deleteFakeTarget(aSimTime);
  }
  if (fakeDetected) {	
    sensorPtr->clearmPendingFakeDetections();
    for (const auto &fd : ownerSensor->radar->getFakeDetections()) {
      sensorPtr->mPendingFakeDetections.push_back(fd);
    }	
	sensorPtr->createFakeTarget(aSimTime);
  }
  else {
	  sensorPtr->clearmPendingFakeDetections();
  }
  
  ownerSensor->radar->plotMaxDetRange(jammers,aXmtrPtr,aResult, aSimTime);

  
  return ownerSensor->radar->detectTarget(target, antenna, Pjn);
}

// =================================================================================================
// virtual
bool TricorderSensor::Initialize(double aSimTime) {
  bool ok = WsfSensor::Initialize(aSimTime);

  // Reduce future dynamic casting by extracting derived class mode pointers.
  mModeListPtr->GetDerivedModeList(mRadarModeList);

  mAnyModeCanTransmit = false;
  mAnyModeCanReceive = false;
  for (RadarMode *modePtr : mRadarModeList) {
    mAnyModeCanTransmit |= modePtr->mCanTransmit;
    mAnyModeCanReceive |= modePtr->mCanReceive;
    // When randomize_radar_frequencies is enabled, notify
    // SensorFrequencyChanged observers of the initial frequencies selected
    // (e.g. via SENSOR_FREQUENCY_CHANGED event) when relevant.
    if (GetSimulation()->RandomizeFrequency() && modePtr->mCanTransmit) {
      WsfObserver::SensorFrequencyChanged(GetSimulation())(aSimTime, this,
                                                           modePtr);
    }
  }
  if (!mAnyModeCanTransmit) {
    SetClass(cSEMI_ACTIVE | cRADIO); // This is a semi-active RF sensor
  }
  return ok;
}

// =================================================================================================
// virtual
void TricorderSensor::PlatformAdded(double aSimTime,
                                    WsfPlatform *aPlatformPtr) {
  // NOTE: A transmit-only device does not perform sensing chances.
  //       All detections are done by the receiver.
  if (mAnyModeCanReceive) {
    WsfSensor::PlatformAdded(aSimTime, aPlatformPtr);
  }
}

// =================================================================================================
// virtual
bool TricorderSensor::ProcessInput(UtInput &aInput) {
	std::string command(aInput.GetCommand());

	bool myCommand = true;

	if (command == "tm" || command == "t0" || command == "Tm" || command == "T0") {
		aInput.ReadValue(mRadarT0_);

	}
	else if (command == "pulseNum") {
		aInput.ReadValue(mRadarPulseNum_);
	}
	else if (command == "Pfa") {
		aInput.ReadValue(mRadarPfa_);
	}
	else if (command == "rangeAccuracy") {
		aInput.ReadValue(mRadarRangeAccuracy_);
	}
	else if (command == "azimuthAccuracy") {
		aInput.ReadValue(mRadarAzimuthAccuracy_);
	}
	else {
		myCommand = WsfSensor::ProcessInput(aInput);
	}

	return myCommand;
}

// =================================================================================================
// virtual
void TricorderSensor::Update(double aSimTime) {
  // Bypass updates if not time for an update.  This avoids unnecessary device
  // updates. (A little slop is allowed to make sure event-driven chances occur
  // as scheduled)
  if (mNextUpdateTime <= (aSimTime + 1.0E-5)) {
    WsfSensor::Update(aSimTime);
	radar->realTime = aSimTime;
    PerformScheduledDetections(
        aSimTime); // Perform any required detection attempts
  }
}

// =================================================================================================
//! Process detections attempts under the control of a scheduler.
//!
//! This is a convenience method to process any required detections as directed
//! by a sensor scheduler.
//!
//! @param aSimTime The current simulation time.
// virtual
void TricorderSensor::PerformScheduledDetections(double aSimTime) {
  // Make sure that a scheduler and tracker are present.
  assert(mModeListPtr != nullptr);
  assert(mSchedulerPtr != nullptr);
  assert(mTrackerPtr != nullptr);
  assert(GetSimulation());

  // Let components do their thing...
  WsfSensorComponent::PrePerformScheduledDetections(*this, aSimTime);

  mPendingFakeDetections.clear();

  WsfTrackId requestId;
  Settings settings;
  WsfSensorResult result;
  WsfSensorTracker::Settings stSettings;
  size_t targetIndex = 0;
  // std::cout << "PerformScheduledDetections " << aSimTime<<"  targetIndex:"<<
  // targetIndex <<std::endl; std::cout << "aSimTime " << aSimTime << std::endl;
  // std::cout << "SelectTarget " << mSchedulerPtr->SelectTarget(aSimTime,
  // mNextUpdateTime, targetIndex, requestId, settings) << std::endl;
  while (mSchedulerPtr->SelectTarget(aSimTime, mNextUpdateTime, targetIndex,
                                     requestId, settings))
  {

    WsfSensorMode *modePtr = mRadarModeList[settings.mModeIndex];
    assert(modePtr != nullptr);

    // std::cout << "SelectTarget " << aSimTime << targetIndex << std::endl;
    //  Perform the sensing chance if the target still exists.
    WsfPlatform *targetPtr = GetSimulation()->GetPlatformByIndex(targetIndex);
	//std::cout << "prev platform :" << targetIndex << ":" << targetPtr->GetLocationLLA().mLat << ":" << targetPtr->GetLocationLLA().mLon << std::endl;
    if (targetPtr != nullptr) {
      if (targetPtr->IsFalseTarget()) {
        continue;
      }

      settings.mRequiredPd = GetRequiredPd(modePtr);
      if (modePtr->WithinDetectionRange(aSimTime, targetPtr)) {

        if (AttemptToDetect(aSimTime, targetPtr, settings, result)) {
          // Apply errors and indicate target is detected
			//std::cout << "self platform :" <<GetPlatform()->GetLocationLLA().mLat<<":"<< GetPlatform()->GetLocationLLA() .mLon<< std::endl;
			//std::cout << "mid platform :" << targetIndex << ":" << targetPtr->GetLocationLLA().mLat << ":" << targetPtr->GetLocationLLA().mLon <<std::endl;
          modePtr->ApplyMeasurementErrors(result);
          mTrackerPtr->TargetDetected(aSimTime, stSettings, requestId,
                                      targetIndex, targetPtr, result);
		  //探测目标信息 {"1004","平台名称","经度","纬度","高度","速度"}
		 // std::cout << "platform :" << targetIndex << ":" << targetPtr->GetLocationLLA().mLat << ":" << targetPtr->GetLocationLLA().mLon << std::endl;
        } else {
          mTrackerPtr->TargetUndetected(aSimTime, stSettings, requestId,
                                        targetIndex, targetPtr, result);
        }
        NotifyTargetUpdated(aSimTime, targetPtr, result);
      } else {
        // Detection chance was culled because it is out of range. We must still
        // report to the tracker because it possible it may be coasting the
        // target.
        result.Reset();
        result.mModeIndex = settings.mModeIndex;
        result.mCheckedStatus = WsfSensorResult::cRCVR_RANGE_LIMITS;
        result.mFailedStatus = WsfSensorResult::cRCVR_RANGE_LIMITS;
        mTrackerPtr->TargetUndetected(aSimTime, stSettings, requestId,
                                      targetIndex, targetPtr, result);
      }
    } 
	else if (TransientCueActive() && (targetIndex == 0)) {
      if (mTempGeometryPtr == nullptr) {
        mTempGeometryPtr = new WsfPlatform(GetScenario());
      }

      double locationWCS[3];
      GetTransientCuedLocationWCS(locationWCS);
      mTempGeometryPtr->SetLocationWCS(locationWCS);

      settings.mRequiredPd = GetRequiredPd(modePtr);
      if (modePtr->WithinDetectionRange(aSimTime, mTempGeometryPtr)) {
        for (unsigned int xmtrIndex = 0; xmtrIndex < GetEM_XmtrCount();
             ++xmtrIndex) {
          if (DebugEnabled()) {
            auto out = ut::log::debug()
                       << "Radar sensor cueing to local track.";
            out.AddNote() << "T = " << aSimTime;
            out.AddNote() << "Platform: " << GetPlatform()->GetName();
            out.AddNote() << "Sensor: " << GetName();
            out.AddNote() << "Track #: " << requestId.GetLocalTrackNumber();
          }
          result.Reset(settings);
          UpdatePosition(aSimTime); // Ensure position is current
          result.BeginGenericInteraction(
              &GetEM_Xmtr(xmtrIndex), mTempGeometryPtr,
              GetEM_Xmtr(xmtrIndex).GetLinkedReceiver());
          result.ComputeUndefinedGeometry();
          result.SetTransmitterBeamPosition();
          if (result.GetTransmitter() != nullptr) {
            result.GetTransmitter()->NotifyListeners(
                aSimTime,
                result); // Notify listeners for possible passive detection
          }
        }
      }
    } else if (mTrackerPtr->TargetDeleted(aSimTime, stSettings, requestId,
                                          targetIndex)) {
      // Target no longer physically exists and is not being tracked.
      mSchedulerPtr->RemoveTarget(aSimTime, targetIndex);
    }
    WsfArticulatedPart::ClearTransientCue(); // Release any transient cue
                                             // created by the scheduler.
  }

  // Build the tracker settings used for all fake-target calls:
  // - cSUPPRESS_TRACKING_CRITERIA  : establish track immediately (bypass M/N).
  // - cSUPPRESS_DETECTION_CHANGE_MESSAGES : don't fire SensorDetectionChanged
  //   with the temp platform's (invalid) simulation index.
 

 

 
  WsfSensorComponent::PostPerformScheduledDetections(*this, aSimTime);
  
  // Set the update interval so the schedule is called at the desired time.
  double updateInterval = std::max(mNextUpdateTime - aSimTime, 1.0E-5);
  SetUpdateInterval(updateInterval);
}

void TricorderSensor::clearmPendingFakeDetections() {
  mPendingFakeDetections.clear();
}

// =================================================================================================
// Nested class TricorderSensor::RadarBeam.
// =================================================================================================

TricorderSensor::RadarBeam::RadarBeam()
    : WsfSensorBeam(), mAntennaPtr(ut::make_unique<WsfEM_Antenna>()),
      mXmtrPtr(ut::make_unique<WsfEM_Xmtr>(WsfEM_Xmtr::cXF_SENSOR,
                                           mAntennaPtr.get())),
      mRcvrPtr(ut::make_unique<WsfEM_Rcvr>(WsfEM_Rcvr::cRF_SENSOR,
                                           mAntennaPtr.get())),
      mClutterPtr(nullptr), mErrorModelAzBeamwidth(-1.0),
      mErrorModelElBeamwidth(-1.0), mErrorModelPulseWidth(-1.0),
      mErrorModelDopplerResolution(-1.0),
      // Start of private data
      mCanTransmit(true), mUseDetector(false), mDopplerResolution(0.0),
      mOneM2DetectRange(0.0), mLookDownFactor(1.0), mPRF_Factor(1.0),
      mPostLockonDetectionThresholdAdjustment(1.0),
      mPostLockonAdjustmentDelayTime(0.0), mLoopGain(0.0),
      mAdjustmentFactor(1.0), mIntegrationGain(1.0),
      mNumberOfPulsesIntegrated(1), mDetector(), mProbabilityTablePtr(nullptr),
      mClutterAttenuationFactor(1.0), mClutterType(), mMutex() {
  // Indicate the receiver is 'linked' with the transmitter.
  mXmtrPtr->SetLinkedReceiver(GetEM_Rcvr());
}

// =================================================================================================
TricorderSensor::RadarBeam::RadarBeam(const RadarBeam &aSrc)
    : WsfSensorBeam(aSrc),
      mAntennaPtr(ut::make_unique<WsfEM_Antenna>(*aSrc.mAntennaPtr)),
      mXmtrPtr(ut::make_unique<WsfEM_Xmtr>(*aSrc.mXmtrPtr, mAntennaPtr.get())),
      mRcvrPtr(ut::make_unique<WsfEM_Rcvr>(*aSrc.mRcvrPtr, mAntennaPtr.get())),
      mClutterPtr(nullptr), mErrorModelAzBeamwidth(aSrc.mErrorModelAzBeamwidth),
      mErrorModelElBeamwidth(aSrc.mErrorModelElBeamwidth),
      mErrorModelPulseWidth(aSrc.mErrorModelPulseWidth),
      mErrorModelDopplerResolution(aSrc.mErrorModelDopplerResolution),
      mCanTransmit(aSrc.mCanTransmit), mUseDetector(aSrc.mUseDetector),
      mDopplerResolution(aSrc.mDopplerResolution),
      mOneM2DetectRange(aSrc.mOneM2DetectRange),
      mLookDownFactor(aSrc.mLookDownFactor), mPRF_Factor(aSrc.mPRF_Factor),
      mPostLockonDetectionThresholdAdjustment(
          aSrc.mPostLockonDetectionThresholdAdjustment),
      mPostLockonAdjustmentDelayTime(aSrc.mPostLockonAdjustmentDelayTime),
      mLoopGain(aSrc.mLoopGain), mAdjustmentFactor(aSrc.mAdjustmentFactor),
      mIntegrationGain(aSrc.mIntegrationGain),
      mNumberOfPulsesIntegrated(aSrc.mNumberOfPulsesIntegrated),
      mDetector(aSrc.mDetector),
      mProbabilityTablePtr(aSrc.mProbabilityTablePtr),
      mClutterAttenuationFactor(aSrc.mClutterAttenuationFactor),
      mClutterType(aSrc.mClutterType), mMutex() {
  if (aSrc.mClutterPtr != nullptr) {
    ut::log::error() << "Unexpected clone of clutter_model.";
  }

  // Indicate the receiver is 'linked' with the transmitter.
  mXmtrPtr->SetLinkedReceiver(GetEM_Rcvr());
}

TricorderSensor::RadarBeam &
TricorderSensor::RadarBeam::operator=(const RadarBeam &aRhs) {
  if (this != &aRhs) {
    WsfSensorBeam::operator=(aRhs);
    mAntennaPtr = ut::make_unique<WsfEM_Antenna>(*aRhs.mAntennaPtr);
    mXmtrPtr = ut::make_unique<WsfEM_Xmtr>(*aRhs.mXmtrPtr, mAntennaPtr.get());
    mRcvrPtr = ut::make_unique<WsfEM_Rcvr>(*aRhs.mRcvrPtr, mAntennaPtr.get());
    mClutterPtr = nullptr;
    mErrorModelAzBeamwidth = aRhs.mErrorModelAzBeamwidth;
    mErrorModelElBeamwidth = aRhs.mErrorModelElBeamwidth;
    mErrorModelPulseWidth = aRhs.mErrorModelPulseWidth;
    mErrorModelDopplerResolution = aRhs.mErrorModelDopplerResolution;
    mCanTransmit = aRhs.mCanTransmit;
    mUseDetector = aRhs.mUseDetector;
    mDopplerResolution = aRhs.mDopplerResolution;
    mOneM2DetectRange = aRhs.mOneM2DetectRange;
    mLookDownFactor = aRhs.mLookDownFactor;
    mPRF_Factor = aRhs.mPRF_Factor;
    mPostLockonDetectionThresholdAdjustment =
        aRhs.mPostLockonDetectionThresholdAdjustment;
    mPostLockonAdjustmentDelayTime = aRhs.mPostLockonAdjustmentDelayTime;
    mLoopGain = aRhs.mLoopGain;
    mAdjustmentFactor = aRhs.mAdjustmentFactor;
    mIntegrationGain = aRhs.mIntegrationGain;
    mNumberOfPulsesIntegrated = aRhs.mNumberOfPulsesIntegrated;
    mDetector = aRhs.mDetector;
    mProbabilityTablePtr = aRhs.mProbabilityTablePtr;
    mClutterAttenuationFactor = aRhs.mClutterAttenuationFactor;
    mClutterType = aRhs.mClutterType;

    if (aRhs.mClutterPtr != nullptr) {
      ut::log::error() << "Unexpected clone of clutter_model.";
    }

    // Indicate the receiver is 'linked' with the transmitter.
    mXmtrPtr->SetLinkedReceiver(GetEM_Rcvr());
  }
  return *this;
}

// =================================================================================================
// virtual
TricorderSensor::RadarBeam::~RadarBeam() { delete mClutterPtr; }

WsfEM_Rcvr *TricorderSensor::RadarBeam::GetEM_Rcvr() { return mRcvrPtr.get(); }

// =================================================================================================
//! Attempt to detect the specified target.
//!
//! @param aSimTime     [input]   The current simulation time.
//! @param aTargetPtr   [input]   The target that is trying to be detected.
//! @param aSettings    [input]   The settings of the sensor which is attempting
//! to detect the aTargetPtr.
//! @param aResult      [updated] On input this contains the current relative
//! geometry information. On output this it will be updated to contain the RCS,
//! signal-to-noise and Pd.
// virtual
void TricorderSensor::RadarBeam::AttemptToDetect(double aSimTime,
                                                 WsfPlatform *aTargetPtr,
                                                 Settings &aSettings,
                                                 WsfSensorResult &aResult) {
  // Must have object pointers so event_output and debug output show locations.
  aResult.BeginGenericInteraction(GetEM_Xmtr(), aTargetPtr, GetEM_Rcvr());

  if (aResult.mFailedStatus == 0) {
    if (mCanTransmit) {
      // Mono-static interaction.
      AttemptToDetect(aSimTime, aTargetPtr, aSettings, GetEM_Xmtr(), aResult);
    } else {
      // Bistatic interaction.  Attempt to receive from any transmitter that is
      // marked as a sensor and take the one that gives the best result.

      bool firstInteraction = true;
      WsfSensorResult tempResult;
      for (unsigned int i = 0; i < mRcvrPtr->GetSensorInteractorCount(); ++i) {
        WsfEM_Xmtr *xmtrPtr = mRcvrPtr->GetSensorInteractorEntry(i);
        // Only attempt the detection if the illuminator platform is not the
        // same as the target platform. (The illuminator does not illuminate
        // itself!)
        if (xmtrPtr->GetPlatform() != aTargetPtr) {
          if (firstInteraction) {
            firstInteraction = false;
            AttemptToDetect(aSimTime, aTargetPtr, aSettings, xmtrPtr, aResult);
          } else {
            tempResult.Reset(aResult);
            tempResult.mBeamIndex = aResult.mBeamIndex;
            AttemptToDetect(aSimTime, aTargetPtr, aSettings, xmtrPtr,
                            tempResult);
            if (tempResult.mSignalToNoise > aResult.mSignalToNoise) {
              aResult = tempResult;
            }
          }
        }
      }
      if (firstInteraction) {
        aResult.mFailedStatus |= WsfEM_Interaction::cSIGNAL_LEVEL;
      }
    }
  }
}

// =================================================================================================
// virtual
void TricorderSensor::RadarBeam::AttemptToDetect(double aSimTime,
                                                 WsfPlatform *aTargetPtr,
                                                 Settings &aSettings,
                                                 WsfEM_Xmtr *aXmtrPtr,
                                                 WsfSensorResult &aResult) {
    UtVec3d ownLocWCS;
    UtVec3d tgtLocWCS;
    UtVec3d range;
    GetPlatform()->GetLocationWCS(ownLocWCS.GetData());
    aTargetPtr->GetLocationWCS(tgtLocWCS.GetData());
    UtVec3d::Subtract(range.GetData(), ownLocWCS.GetData(), tgtLocWCS.GetData());
	double rang1 = UtVec3d::Normalize(range.GetData());
	double viewRange = 4.12 * (std::sqrt(GetPlatform()->GetLocationLLA().mAlt) + std::sqrt(aTargetPtr->GetLocationLLA().mAlt) * 1.0e3);
    if (rang1 >
		viewRange) {
		aResult.mFailedStatus |= WsfSensorResult::cSIGNAL_LEVEL;
      return;
    }

  
    // Set the position of the antenna beam(s).
    aResult.SetTransmitterBeamPosition();
    aResult.SetReceiverBeamPosition();

    // Determine the radar cross section of the target.
    aResult.mRadarSigAz = aResult.mTgtToRcvr.mAz;
    aResult.mRadarSigEl = aResult.mTgtToRcvr.mEl;
    aResult.mRadarSig = WsfRadarSignature::GetValue(
        aTargetPtr, aXmtrPtr, GetEM_Rcvr(), aResult.mTgtToXmtr.mAz,
        aResult.mTgtToXmtr.mEl, aResult.mTgtToRcvr.mAz, aResult.mTgtToRcvr.mEl);

    // Compute the received power FIRST — must run before component hook so
    // mRcvdPower is available to the EW component / jammer math.
    aResult.ComputeRF_TwoWayPower(aResult.mRadarSig);

    // === MANDATORY: invoke sensor components (this is where WsfEW_SensorComponent
    // calls WsfRF_Jammer::ComputeTotalJammerEffects and writes mInterferencePower
    // and mInterferenceFactor). Without this call, NO jammer can affect this sensor.
    WsfSensor* sensorPtr = GetSensorMode()->GetSensor();
    WsfSensorComponent::AttemptToDetect(*sensorPtr, aSimTime, aResult);

    // Optional: run any configured signal processors.
    GetSignalProcessors().Execute(aSimTime, aResult);

    if (aResult.mFailedStatus == 0) {
      // Folded-in interference power goes into the SINR denominator.
      aResult.mSignalToNoise = mRcvrPtr->ComputeSignalToNoise(
          aResult.mRcvdPower, aResult.mClutterPower, aResult.mInterferencePower);

      // Custom Pd model.
      aResult.mPd = calculationDetectionProbability(aSimTime, aTargetPtr,
                                                    aSettings, aXmtrPtr, aResult);

      // MANDATORY: effect-based Pd suppression (drop-track, flooded-blocked, etc.)
      aResult.mPd *= (1.0 - aResult.mInterferenceFactor);

      aResult.mCheckedStatus |= WsfSensorResult::cSIGNAL_LEVEL;
      if (aResult.mPd < aResult.mRequiredPd) {
        aResult.mFailedStatus |= WsfSensorResult::cSIGNAL_LEVEL;
      }
    }
  
}

// =================================================================================================
// private
void TricorderSensor::RadarBeam::Calibrate(bool aPrint) {
  double p_avg = mXmtrPtr->GetAveragePower();
  double dutyCycle =
      mXmtrPtr->GetPulseWidth() * mXmtrPtr->GetPulseRepetitionFrequency();
  double wavelength = UtMath::cLIGHT_SPEED / mXmtrPtr->GetFrequency();
  auto out = ut::log::info();
  if (aPrint) {
    out << "Radar beam calibrated.";

    out.AddNote() << "Peak Power Output: "
                  << UtMath::LinearToDB(mXmtrPtr->GetPeakPower()) << " dbW ("
                  << mXmtrPtr->GetPeakPower() << " W)";
    out.AddNote() << "Pulse Repetition Frequency: "
                  << mXmtrPtr->GetPulseRepetitionFrequency() << " Hz";
    out.AddNote() << "Pulse Width: " << mXmtrPtr->GetPulseWidth() << " sec";
    out.AddNote() << "Duty Cycle (input:PW/PRI): " << mXmtrPtr->GetDutyCycle()
                  << ":" << dutyCycle;
    double pcr = mXmtrPtr->GetPulseCompressionRatio();
    out.AddNote() << "Pulse Compression Ratio: " << UtMath::LinearToDB(pcr)
                  << " dB (" << pcr << ")";
    out.AddNote() << "Average Power Output: " << UtMath::LinearToDB(p_avg)
                  << " dBW (" << p_avg << " W)";

    if (mXmtrPtr->ExplicitFrequencyList() /* && (!mXmtrPtr->UseDefaultFrequency())*/)
                           {
      out.AddNote() << "Frequency (default): " << mXmtrPtr->GetFrequency()
                    << " Hz";
      out.AddNote() << "Wavelength (default): " << wavelength << " m";
      WsfEM_Xmtr::FrequencyList freqList = mXmtrPtr->GetAlternateFrequencies();
      for (unsigned int i = 0; i < (unsigned int)freqList.size(); ++i) {
        out.AddNote() << "Frequency " << i + 1 << ": " << freqList[i] << " Hz";
      }
    } else {
      out.AddNote() << "Frequency: " << mXmtrPtr->GetFrequency() << " Hz";
      out.AddNote() << "Wavelength: " << wavelength << " m";
    }

    double peakGain;
    peakGain = mXmtrPtr->GetPeakAntennaGain();
    out.AddNote() << "Transmitter Antenna Gain: "
                  << UtMath::LinearToDB(peakGain) << " dB (" << peakGain << ")";
    peakGain = mRcvrPtr->GetPeakAntennaGain();
    out.AddNote() << "Receiver Antenna Gain: " << UtMath::LinearToDB(peakGain)
                  << " dB (" << peakGain << ")";

    double internalLoss;
    internalLoss = mXmtrPtr->GetInternalLoss();
    out.AddNote() << "Transmitter Internal Loss: "
                  << UtMath::LinearToDB(internalLoss) << " dB (" << internalLoss
                  << ")";
    internalLoss = mRcvrPtr->GetInternalLoss();
    out.AddNote() << "Receiver Internal Loss: "
                  << UtMath::LinearToDB(internalLoss) << " dB (" << internalLoss
                  << ")";

    double integrationGain = mIntegrationGain;
    if (integrationGain != 1.0) {
      out.AddNote() << "Integration Gain: "
                    << UtMath::LinearToDB(integrationGain) << " dB ("
                    << integrationGain << ")";
    }

    double adjustmentFactor = mAdjustmentFactor;
    if (adjustmentFactor != 1.0) {
      out.AddNote() << "Adjustment Factor: "
                    << UtMath::LinearToDB(adjustmentFactor) << " dB ("
                    << adjustmentFactor << ")";
    }

    if (mOneM2DetectRange > 0.0) {
      out.AddNote() << "1 m^2 Detection Range: " << mOneM2DetectRange << " m";
    } else if (mLoopGain > 0.0) {
      out.AddNote() << "Loop Gain: " << UtMath::LinearToDB(mLoopGain) << " dB";
    } else {
      out.AddNote() << "Receiver Noise Power: "
                    << UtMath::LinearToDB(mRcvrPtr->GetNoisePower()) << " dBW ("
                    << mRcvrPtr->GetNoisePower() << " W)";
    }
  }

  double detectionThreshold = mRcvrPtr->GetDetectionThreshold();
  if (aPrint) {
    double thresholdWatts = detectionThreshold * mRcvrPtr->GetNoisePower();
    out.AddNote() << "Minimum Detectable Signal: "
                  << UtMath::LinearToDB(thresholdWatts) << " dBW ("
                  << thresholdWatts << " W)";
    out.AddNote() << "Minimum Detectable S/N: "
                  << UtMath::LinearToDB(detectionThreshold) << " dB ("
                  << detectionThreshold << ")";
  }

  if (mOneM2DetectRange > 0.0) {
    double r = mOneM2DetectRange;

    double distanceFactor = 1.0 / (4.0 * UtMath::cPI * r * r);

    // Compute the effective radiated power from the antenna.
    double p_radiated = mXmtrPtr->GetPower() * mXmtrPtr->GetPeakAntennaGain() /
                        mXmtrPtr->GetInternalLoss();

    // Compute the power per unit area on at the target
    double p_tgt_area = p_radiated * distanceFactor;

    // Compute the reflected power
    double rcs = 1.0;
    double p_reflected = p_tgt_area * rcs;

    // Compute the incident power per unit area at the receiving antenna.
    double p_rcvr_area = p_reflected * distanceFactor;

    // Compute the effective received power.
    double temp1 = wavelength * wavelength / (4.0 * UtMath::cPI);
    double p_received = p_rcvr_area * temp1 * mRcvrPtr->GetPeakAntennaGain() /
                        mRcvrPtr->GetInternalLoss();

    // Account for the gain due to pulse compression.
    p_received *= mXmtrPtr->GetPulseCompressionRatio();

    // Account for integration gain
    p_received *= mIntegrationGain;

    // Allow for other general post-reception adjustments.
    p_received *= mAdjustmentFactor;

    double rcvrNoise = p_received / detectionThreshold;
    mRcvrPtr->SetNoisePower(rcvrNoise);
    if (aPrint) {
      out.AddNote() << "Receiver Noise: " << UtMath::LinearToDB(rcvrNoise)
                    << " dbW (" << rcvrNoise << " W) (calibrated)";
    }
  } else if (mLoopGain > 0.0) {
    // Solve for the receiver noise
    double temp1 = (wavelength * wavelength) /
                   (UtMath::cFOUR_PI * UtMath::cFOUR_PI * UtMath::cFOUR_PI);
    temp1 *= mXmtrPtr->GetPower() * mXmtrPtr->GetPeakAntennaGain() *
             mRcvrPtr->GetPeakAntennaGain();
    temp1 /= (mXmtrPtr->GetInternalLoss() * mRcvrPtr->GetInternalLoss());
    double rcvrNoise = temp1 / mLoopGain;
    mRcvrPtr->SetNoisePower(rcvrNoise);
    if (aPrint) {
      out.AddNote() << "Receiver Noise: " << UtMath::LinearToDB(rcvrNoise)
                    << " dbW (" << rcvrNoise << " W) (calibrated)";
    }
  }

  {
    double temp1 = (wavelength * wavelength) /
                   (UtMath::cFOUR_PI * UtMath::cFOUR_PI * UtMath::cFOUR_PI);
    temp1 *= mXmtrPtr->GetPower() * mXmtrPtr->GetPeakAntennaGain() *
             mRcvrPtr->GetPeakAntennaGain();
    temp1 /= (mXmtrPtr->GetInternalLoss() * mRcvrPtr->GetInternalLoss());

    // Account for the gain due to pulse compression.
    temp1 *= mXmtrPtr->GetPulseCompressionRatio();

    // Account for integration gain
    temp1 *= mIntegrationGain;

    // Allow for other general post-reception adjustments.
    temp1 *= mAdjustmentFactor;

    double r =
        pow(temp1 / (mRcvrPtr->GetNoisePower() * detectionThreshold), 0.25);
    double loopGain = temp1 / mRcvrPtr->GetNoisePower();
    if (aPrint) {
      out.AddNote() << "1 m^2 Detection Range: " << r
                    << " m (calibrated - free space)";
      out.AddNote() << "Loop Gain: " << UtMath::LinearToDB(loopGain)
                    << " dB (calibrated)";
    }
  }

  if (aPrint && (mClutterPtr != nullptr) && (mClutterAttenuationFactor < 1.0)) {
    out.AddNote() << "Clutter attenuation factor: "
                  << UtMath::LinearToDB(mClutterAttenuationFactor) << " dB";
  }
}

// =================================================================================================
//! Compute the integrated pulse count.
//!
//! This is only used to compute the integrated pulse count for clutter
//! calculations.
//!
//! From SalramMode::Initialize.
// public
double
TricorderSensor::RadarBeam::ComputeIntegratedPulseCount(RadarMode &aMode) {
  double pulseCount = 1.0; // Assuming continuous wave...
  if (mXmtrPtr->GetPulseRepetitionFrequency() != 0.0) {
    // Pulsed radar...

    double timeOnTarget = 0.0;
    if (aMode.GetDwellTime() == 0.0) {
      if (mAntennaPtr->GetScanMode() == WsfEM_Antenna::cSCAN_EL) {
        double minEl, maxEl;
        mAntennaPtr->GetElevationScanLimits(minEl, maxEl);
        timeOnTarget = aMode.GetFrameTime() *
                       mXmtrPtr->GetElevationBeamwidth(0.0, 0.0) /
                       (maxEl - minEl);
      } else {
        double minAz, maxAz;
        mAntennaPtr->GetAzimuthScanLimits(minAz, maxAz);
        timeOnTarget = aMode.GetFrameTime() *
                       mXmtrPtr->GetAzimuthBeamwidth(0.0, 0.0) /
                       (maxAz - minAz);
      }
    } else {
      // Tracker
      timeOnTarget = aMode.GetDwellTime();
    }
    pulseCount = timeOnTarget * mXmtrPtr->GetPulseRepetitionFrequency();
  }
  return pulseCount;
}

// =================================================================================================
bool TricorderSensor::RadarBeam::Initialize(
    double aSimTime, unsigned int aBeamIndex, WsfSensor *aSensorPtr,
    RadarMode *aModePtr, bool aCanTransmit, bool aCanReceive,
    bool aShowCalibrationData, bool aIsMultiBeam) {
  // Propagate the debug flag
  if (aSensorPtr->GetDebugMask() & WsfPlatformPart::cLEVEL_TWO_DEBUG) {
    mRcvrPtr->SetDebugEnabled(aSensorPtr->DebugEnabled());
    mXmtrPtr->SetDebugEnabled(aSensorPtr->DebugEnabled());
  }

  mXmtrPtr->UseDefaultFrequency(
      aSensorPtr->GetScenario().GetSimulationInput().UseDefaultFrequency());

  mXmtrPtr->SetMode(
      aModePtr); // Identify the transmitter with the mode and beam index.
  mXmtrPtr->SetIndex(aBeamIndex);
  mRcvrPtr->SetMode(
      aModePtr); // Identify the transmitter with the mode and beam index.
  mRcvrPtr->SetIndex(aBeamIndex);
  mCanTransmit = aCanTransmit;

  bool ok = mAntennaPtr->Initialize(aSensorPtr);
  if (aCanTransmit) {
    ok &= mXmtrPtr->Initialize(*aSensorPtr->GetSimulation());
  }

  if (aCanReceive) {
    ok &= mRcvrPtr->Initialize(*aSensorPtr->GetSimulation());

    // In case the bandwidth was not set for the noise calculation,
    // set the bandwidth using the pulsewidth of the linked transmitter.
    mRcvrPtr->UpdateNoisePower(mXmtrPtr->GetPulseWidth());

    if (mUseDetector) {
      mDetector.Initialize(0.0, aModePtr, aBeamIndex);
    } else if (mProbabilityTablePtr) {
      mProbabilityTablePtr->Initialize(0.0, aModePtr, aBeamIndex);
    }

    if ((!GetSignalProcessors().Empty()) &&
        ((mOneM2DetectRange > 0.0) || (mLoopGain > 0.0))) {
      ut::log::warning()
          << "'signal_processor' should not be used with "
             "'one_m2_detect_range', 'range_product', or 'loop_gain'";
    }

    // Load and initialize the clutter model object.
    if (!mClutterType.IsNull()) {
      mClutterPtr = WsfEM_ClutterTypes::Get(aSensorPtr->GetScenario())
                        .Clone(mClutterType);
      if (mClutterPtr == nullptr) {
        auto out = ut::log::error() << "Undefined 'clutter_model'.";
        out.AddNote() << "Model: " << mClutterType;
        ok = false;
      } else if (mClutterPtr->IsNullModel()) {
        // The referenced model is a 'null' (no-effect) model - delete it to
        // save processing time.
        delete mClutterPtr;
        mClutterPtr = nullptr;
      } else if (!mClutterPtr->Initialize(GetEM_Rcvr())) {
        ut::log::error() << "Failed to initialize 'clutter_model'.";
        ok = false;
      }
    }
  }

  // Initialize after setting up transmitter, receiver and antenna
  ok &= WsfSensorBeam::Initialize(aSimTime);

  if (ok && aCanTransmit && aCanReceive) {
    if (aShowCalibrationData) {
      auto out = ut::log::info() << "Radar sensor initialized.";
      out.AddNote() << "Platform: " << aSensorPtr->GetPlatform()->GetName();
      out.AddNote() << "Sensor: " << aSensorPtr->GetName();
      out.AddNote() << "Mode: " << aModePtr->GetName();
      if (aIsMultiBeam) {
        out.AddNote() << "Beam: " << aBeamIndex + 1;
      }
    }
    Calibrate(aShowCalibrationData);
  }
  return ok;
}

// =================================================================================================
// private
bool TricorderSensor::RadarBeam::InitializeDetector(RadarMode &aMode) {
  // If a statistical detection is being used, compute the nominal detection
  // threshold and the integration gain.
  if (mProbabilityTablePtr || mUseDetector) {
    double requiredPd =
        aMode.GetRequiredPd(); // Get the required Pd as entered.
    if ((requiredPd <= 0.0) || (requiredPd > 1.0)) {
      requiredPd = 0.5;
    }
    requiredPd = std::min(std::max(requiredPd, 0.002), 0.998);

    // Make two passes through the following loop. The first pass uses the
    // 'number_of_pulses_integrated' as defined by the user. This produces the
    // pre-integration signal-to-noise for the desired Pd. The second pass uses
    // the 'number_of_pulses_integrated' equal to one. This produces the
    // post-integration signal-to-noise for the desired Pd. From these two
    // numbers we compute the effective integration gain.
    //
    // Note that the detector is left with the number_of_pulses_integrated set
    // to one. The computed integration gain is stored in the beam and used to
    // adjust the non-integrated signal-to-noise to explicitly produce an
    // integrated signal-to-noise which is then fed to the detector (which does
    // no integration).
    //
    // The reason for all of this is that the error model should use
    // post-integration signal levels rather than pre-integration signal levels.
    // If this isn't done, the smaller pre-integration signal will be used by
    // the error model and will result in larger errors in the track, and very
    // erratic tracks from the filters.

    double singlePulseThreshold = 0.0;
    double multiPulseThreshold = 0.0;
    int numberOfPulsesIntegrated = mNumberOfPulsesIntegrated;
    mDetector.SetNumberOfPulsesIntegrated(numberOfPulsesIntegrated);
    for (int pass = 1; pass <= 2; ++pass) {
      // Find the signal to noise the generates a Pd equal to the required Pd.
      double loSignalToNoise = 0.0;
      double hiSignalToNoise = 1000.0;
      double detectionThreshold = 0.0;
      while (fabs(hiSignalToNoise - loSignalToNoise) > 0.001) {
        detectionThreshold = 0.5 * (loSignalToNoise + hiSignalToNoise);
        double pd;
        if (mProbabilityTablePtr) {
          pd = mProbabilityTablePtr->ComputeProbabilityOfDetection(
              detectionThreshold);
        } else {
          pd = mDetector.ComputeProbabilityOfDetection(detectionThreshold);
        }
        if (fabs(pd - requiredPd) < 0.001) {
          break;
        } else if (pd < requiredPd) {
          loSignalToNoise = detectionThreshold;
        } else {
          hiSignalToNoise = detectionThreshold;
        }
      }
      // Assume this is the final pass - capture the single pulse threshold.
      singlePulseThreshold = detectionThreshold;
      if (pass == 1) {
        // On first pass capture the multiple-pulse threshold (which is the same
        // as the single-pulse threshold if multiple pulses are not being
        // integrated).
        multiPulseThreshold = detectionThreshold;
        mDetector.SetNumberOfPulsesIntegrated(1);
        if ((numberOfPulsesIntegrated == 1) || mProbabilityTablePtr) {
          break; // Bypass second loop if not integrating multiple pulses
        }
      }
    }
    mRcvrPtr->SetDetectionThreshold(singlePulseThreshold);
    mIntegrationGain = singlePulseThreshold / multiPulseThreshold;
  }
  return true;
}

// =================================================================================================
bool TricorderSensor::RadarBeam::ProcessInput(UtInput &aInput) {
  bool myCommand = true;
  std::string command(aInput.GetCommand());
  if (mAntennaPtr->ProcessInput(aInput) ||
      mXmtrPtr->ProcessInputBlock(aInput) ||
      mRcvrPtr->ProcessInputBlock(aInput)) {
  } else if (WsfEM_ClutterTypes::Get(WsfScenario::FromInput(aInput))
                 .LoadReference(aInput, mClutterType)) {
  } else if (command == "doppler_resolution") {
    aInput.ReadValueOfType(mDopplerResolution, UtInput::cSPEED);
    aInput.ValueGreater(mDopplerResolution, 0.0);
  } else if (command == "integration_gain") {
    aInput.ReadValueOfType(mIntegrationGain, UtInput::cRATIO);
    aInput.ValueGreaterOrEqual(mIntegrationGain, 1.0);
    mUseDetector = false;
  } else if (command == "adjustment_factor") {
    aInput.ReadValueOfType(mAdjustmentFactor, UtInput::cRATIO);
    aInput.ValueGreater(mAdjustmentFactor, 0.0);
  } else if (command == "operating_loss") {
    // Just another name for the reciprocal of the 'adjustment_factor'
    double operatingLoss;
    aInput.ReadValueOfType(operatingLoss, UtInput::cRATIO);
    aInput.ValueGreaterOrEqual(operatingLoss, 1.0);
    mAdjustmentFactor = 1.0 / operatingLoss;
  } else if (command == "detection_threshold") {
    // The receiver also has a detection threshold. It is also put here so the
    // 'integration_gain' and 'detection_threshold' can be grouped together by
    // the user in a logical fashion.
    double detectionThreshold;
    aInput.ReadValueOfType(detectionThreshold, UtInput::cRATIO);
    aInput.ValueGreater(detectionThreshold, 0.0);
    mRcvrPtr->SetDetectionThreshold(detectionThreshold);
    mUseDetector = false;
    mProbabilityTablePtr =
        std::shared_ptr<wsf::DetectionProbabilityTable>(nullptr);
  } else if (command == "post_lockon_detection_threshold_adjustment") {
    aInput.ReadValueOfType(mPostLockonDetectionThresholdAdjustment,
                           UtInput::cRATIO);
    aInput.ValueGreater(mPostLockonDetectionThresholdAdjustment, 0.0);
  } else if (command == "post_lockon_adjustment_delay_time") {
    aInput.ReadValueOfType(mPostLockonAdjustmentDelayTime, UtInput::cTIME);
    aInput.ValueGreaterOrEqual(mPostLockonAdjustmentDelayTime, 0.0);
  } else if (command == "number_of_pulses_integrated") {
    aInput.ReadValue(mNumberOfPulsesIntegrated);
    aInput.ValueGreater(mNumberOfPulsesIntegrated, 0);
  } else if (command == "detection_probability") {
    mProbabilityTablePtr = std::make_shared<wsf::DetectionProbabilityTable>();
    mProbabilityTablePtr->ReadTable(aInput);
    mUseDetector = false;
  } else if (command == "detector_law") {
    std::string word;
    aInput.ReadValue(word);
    if (word == "linear") {
      mDetector.SetDetectorLaw(wsf::MarcumSwerling::cDL_LINEAR);
    } else if (word == "square") {
      mDetector.SetDetectorLaw(wsf::MarcumSwerling::cDL_SQUARE);
    } else if (word == "log") {
      mDetector.SetDetectorLaw(wsf::MarcumSwerling::cDL_LOG);
    } else {
      throw UtInput::BadValue(aInput);
    }
  } else if (command == "probability_of_false_alarm") {
    double probabilityOfFalseAlarm;
    aInput.ReadValue(probabilityOfFalseAlarm);
    aInput.ValueInClosedRange(probabilityOfFalseAlarm, 0.0, 1.0);
    mDetector.SetProbabilityOfFalseAlarm(probabilityOfFalseAlarm);
  } else if (command == "swerling_case") {
    int swerlingCase;
    aInput.ReadValue(swerlingCase);
    aInput.ValueInClosedRange(swerlingCase, 0, 4);
    mDetector.SetCase(swerlingCase);
    mUseDetector = true;
    mProbabilityTablePtr =
        std::shared_ptr<wsf::DetectionProbabilityTable>(nullptr);
  } else if (command == "no_swerling_case") {
    mUseDetector = false;
    mProbabilityTablePtr =
        std::shared_ptr<wsf::DetectionProbabilityTable>(nullptr);
  } else if (command == "error_model_parameters") {
    UtInputBlock block(aInput);
    while (block.ReadCommand(command)) {
      if (command == "azimuth_beamwidth") {
        aInput.ReadValueOfType(mErrorModelAzBeamwidth, UtInput::cANGLE);
        aInput.ValueGreater(mErrorModelAzBeamwidth, 0.0);
      } else if (command == "elevation_beamwidth") {
        aInput.ReadValueOfType(mErrorModelElBeamwidth, UtInput::cANGLE);
        aInput.ValueGreater(mErrorModelElBeamwidth, 0.0);
      } else if (command == "pulse_width") {
        aInput.ReadValueOfType(mErrorModelPulseWidth, UtInput::cTIME);
        aInput.ValueGreater(mErrorModelPulseWidth, 0.0);
      } else if (command == "receiver_bandwidth") {
        // Derive the pulse width assuming a matched filter.
        double bandwidth;
        aInput.ReadValueOfType(bandwidth, UtInput::cFREQUENCY);
        aInput.ValueGreater(bandwidth, 0.0);
        mErrorModelPulseWidth = 1.0 / bandwidth;
      } else if (command == "doppler_resolution") {
        aInput.ReadValueOfType(mErrorModelDopplerResolution, UtInput::cSPEED);
        aInput.ValueGreater(mErrorModelDopplerResolution, 0.0);
      } else {
        throw UtInput::UnknownCommand(aInput);
      }
    }
  } else if (command == "one_m2_detect_range") {
    aInput.ReadValueOfType(mOneM2DetectRange, UtInput::cLENGTH);
    aInput.ValueGreater(mOneM2DetectRange, 0.0);
    mLoopGain = 0.0; // This overrides loop_gain
  } else if (command == "range_product") {
    // range_product = rcs * detect_range^4
    double rangeProduct;
    aInput.ReadValueOfType(rangeProduct, UtInput::cAREA_DB);
    aInput.ValueGreater(rangeProduct, 0.0);
    mOneM2DetectRange = pow(rangeProduct, 0.25);
    mLoopGain = 0.0; // This overrides loop_gain
  } else if (command == "loop_gain") {
    aInput.ReadValueOfType(mLoopGain, UtInput::cRATIO);
    aInput.ValueGreater(mLoopGain, 0.0);
    mOneM2DetectRange = 0.0; // This overrides one_m2_detect_range
  } else if (command == "look_down_factor") {
    aInput.ReadValueOfType(mLookDownFactor, UtInput::cRATIO);
  } else if (command == "prf_factor") {
    aInput.ReadValueOfType(mPRF_Factor, UtInput::cRATIO);
  } else if (command == "clutter_attenuation_factor") {
    aInput.ReadValueOfType(mClutterAttenuationFactor, UtInput::cRATIO);
    aInput.ValueInClosedRange(mClutterAttenuationFactor, 0.0, 1.0);
  } else {
    myCommand = WsfSensorBeam::ProcessInput(aInput);
  }
  return myCommand;
}

WsfPlatform *TricorderSensor::RadarBeam::GetPlatform() {
  return mAntennaPtr->GetPlatform();
}

// =================================================================================================
// Nested class TricorderSensor::RadarMode.

TricorderSensor::RadarMode::RadarMode()
    : WsfSensorMode(), mCanTransmit(true), mCanReceive(true),
      mImplicitBeamUsed(false), mExplicitBeamUsed(false), mBeamList(),
      mAltFreqSelectDelay(0.0), mAltFreqChangeScheduled(false),
      mLastAltFreqSelectTime(0.0), mIsFrequencyAgile(false) {
  SetErrorModel(ut::make_unique<wsf::RadarSensorErrorModel>());
  mBeamList.push_back(new RadarBeam());
  mBeamList[0]->SetBeamOwner(this);
}

// =================================================================================================
TricorderSensor::RadarMode::RadarMode(const RadarMode &aSrc)
    : WsfSensorMode(aSrc), mCanTransmit(aSrc.mCanTransmit),
      mCanReceive(aSrc.mCanReceive), mImplicitBeamUsed(aSrc.mImplicitBeamUsed),
      mExplicitBeamUsed(aSrc.mExplicitBeamUsed), mBeamList(aSrc.mBeamList),
      mAltFreqSelectDelay(aSrc.mAltFreqSelectDelay),
      mAltFreqChangeScheduled(aSrc.mAltFreqChangeScheduled),
      mLastAltFreqSelectTime(aSrc.mLastAltFreqSelectTime),
      mIsFrequencyAgile(aSrc.mIsFrequencyAgile) {
  for (size_t i = 0; i < mBeamList.size(); ++i) {
    mBeamList[i] = new RadarBeam(*aSrc.mBeamList[i]);
  }
}

// =================================================================================================
TricorderSensor::RadarMode &
TricorderSensor::RadarMode::operator=(const RadarMode &aRhs) {
  if (this != &aRhs) {
    WsfSensorMode::operator=(aRhs);
    mCanTransmit = aRhs.mCanTransmit;
    mCanReceive = aRhs.mCanReceive;
    mImplicitBeamUsed = aRhs.mImplicitBeamUsed;
    mExplicitBeamUsed = aRhs.mExplicitBeamUsed;
    mBeamList = aRhs.mBeamList;
    mAltFreqSelectDelay = aRhs.mAltFreqSelectDelay;
    mAltFreqChangeScheduled = aRhs.mAltFreqChangeScheduled;
    mLastAltFreqSelectTime = aRhs.mLastAltFreqSelectTime;
    mIsFrequencyAgile = aRhs.mIsFrequencyAgile;

    for (size_t i = 0; i < mBeamList.size(); ++i) {
      mBeamList[i] = new RadarBeam(*aRhs.mBeamList[i]);
    }
  }
  return *this;
}

// =================================================================================================
TricorderSensor::RadarMode::~RadarMode() {
  for (auto &beamPtr : mBeamList) {
    delete beamPtr;
  }
}

// =================================================================================================
// virtual
WsfMode *TricorderSensor::RadarMode::Clone() const {
  return new RadarMode(*this);
}

// =================================================================================================
// virtual
bool TricorderSensor::RadarMode::Initialize(double aSimTime) {
  // If the mode is a transmit-only mode then force some sort of reporting to be
  // present. Otherwise the base class will indicate an error because the sensor
  // doesn't report anything.
  if (!mCanReceive) {
    mReportingFlags.mReportsOther = true;
    if (mFrameTime == 0.0) {
      mFrameTime = 1000.0;
    }
  }

  bool ok = WsfSensorMode::Initialize(aSimTime);

  // Initialize each of the beams

  mMaximumRange = 0.0;
  mIsFrequencyAgile = false; // reset for each instance
  bool isMultiBeam = (mBeamList.size() > 1);
  for (unsigned int beamIndex = 0; beamIndex < mBeamList.size(); ++beamIndex) {
    RadarBeam &beam = *mBeamList[beamIndex];
    ok &= beam.Initialize(aSimTime, beamIndex, mSensorPtr, this, mCanTransmit,
                          mCanReceive, mSensorPtr->ShowCalibrationData(),
                          isMultiBeam);

    mMaximumRange =
        std::max(mMaximumRange, beam.mAntennaPtr->GetMaximumRange());

    // TODO - may need method to determine how fast of switching determines
    // agility.
    if ((!mIsFrequencyAgile) && mCanTransmit &&
        (beam.mXmtrPtr->GetAlternateFrequencyCount() > 0)) {
      mIsFrequencyAgile = true;
    }
  }

  if (ok) {
    WsfSimulation &sim = *mSensorPtr->GetSimulation();
    if (mCanTransmit && sim.RandomizeFrequency()) {
      int id = mSensorPtr->GetRandom().Uniform(
          0, std::max(0, mBeamList[0]->mXmtrPtr->GetAlternateFrequencyCount() -
                             1));
      SelectAlternateFrequency(aSimTime, id);
    }
  }

  return ok;
}

// =================================================================================================
// virtual
bool TricorderSensor::RadarMode::ProcessInput(UtInput &aInput) {
  bool myCommand = true;
  std::string command(aInput.GetCommand());

  // An explicit 'beam' command can be used only if commands have not been
  // applied to the implicit beam. Commands for the implicit 'beam' cannot be
  // used if an explicit beam has been defined.
  if (command == "beam") {
    if (mImplicitBeamUsed) {
      std::string msg = "'beam' cannot be used in this context; command " +
                        sLastImplicitBeamCommand +
                        " must be moved inside the 'beam' definition.";
      throw UtInput::BadValue(aInput, msg);
    }
    mExplicitBeamUsed = true;
    UtInputBlock inputBlock(aInput);

    // Beam 1 is implicitly created.  Subsequent beams are created by using a
    // beam number that is one higher than the last beam number. The initial
    // values for the beam are copied from the first beam.

    int beamNumber;
    aInput.ReadValue(beamNumber);
    aInput.ValueInClosedRange(beamNumber, 1,
                              static_cast<int>(mBeamList.size() + 2));
    unsigned int beamIndex = static_cast<unsigned int>(beamNumber - 1);
    if (beamIndex == mBeamList.size()) {
      mBeamList.push_back(new RadarBeam(*mBeamList[0]));
      mBeamList.back()->SetBeamIndex(beamIndex);
    }

    while (inputBlock.ReadCommand()) {
      if (!mBeamList[beamIndex]->ProcessInput(aInput)) {
        throw UtInput::UnknownCommand(aInput);
      }
    }
  } else if (mBeamList[0]->ProcessInput(aInput)) {
    if (mExplicitBeamUsed) {
      throw UtInput::BadValue(aInput, "Implicit 'beam' commands cannot be used "
                                      "if an explicit 'beam' has been defined");
    }
    mImplicitBeamUsed = true;
    sLastImplicitBeamCommand = aInput.GetCommand();
  } else if (command == "receive_only") {
    mCanReceive = true;
    mCanTransmit = false;
  } else if (command == "transmit_only") {
    mCanReceive = false;
    mCanTransmit = true;
  } else if (command == "frequency_select_delay") {
    aInput.ReadValueOfType(mAltFreqSelectDelay, UtInput::cTIME);
  } else {
    myCommand = WsfSensorMode::ProcessInput(aInput);
  }
  return myCommand;
}

// =================================================================================================
bool TricorderSensor::RadarMode::AttemptToDetect(double aSimTime,
                                                 WsfPlatform *aTargetPtr,
                                                 Settings &aSettings,
                                                 WsfSensorResult &aResult) {
  bool detected = false;
  aResult.Reset(aSettings);
  aResult.SetCategory(GetSensor()->GetZoneAttenuationModifier());
  GetSensor()->UpdatePosition(aSimTime); // Ensure my position is current
  aTargetPtr->Update(aSimTime); // Ensure the target position is current
  // std::cout << "GetMaxAzSlew0 " << GetSensor()->GetMaxAzSlew() << std::endl;
  // std::cout <<" GetMaxAzSlew1 " << GetSensor()->GetMaxAzCue() << std::endl;
  // std::cout << "GetMaxAzSlew2 " << GetSensor()->GetAzCueRate() << std::endl;
  // std::cout << "GetMaxAzSlew3 " << GetSensor()->GetAzSlewRate() << std::endl;
                
  
  if (GetSensor()->DebugEnabled()) {
    auto out = ut::log::debug() << "Radar sensor attempting to detect target.";
    out.AddNote() << "T = " << aSimTime;
    out.AddNote() << "Platform: " << GetPlatform()->GetName();
    out.AddNote() << "Sensor: " << GetSensor()->GetName();
    out.AddNote() << "Mode: " << GetName();
    out.AddNote() << "Target: " << aTargetPtr->GetName();
  }
  // Determine which beam returns the best results...
  if (!mCanReceive && mCanTransmit) {
    // TRANSMITTER only
  } else if (aResult.mFailedStatus == 0) {
    // Determine if concealed (like in a building).
    aResult.mCheckedStatus |= WsfSensorResult::cCONCEALMENT;
    if (aTargetPtr->GetConcealmentFactor() > 0.99F) {
      // We can't detect if it's in a building (or something like that)
      aResult.mFailedStatus |= WsfSensorResult::cCONCEALMENT;
      // Must have object pointers so event_output and debug output show
      // locations.
      aResult.BeginGenericInteraction(mBeamList[0]->GetEM_Xmtr(), aTargetPtr,
                                      mBeamList[0]->GetEM_Rcvr());
    }

    mBeamList[0]->AttemptToDetect(aSimTime, aTargetPtr, aSettings, aResult);

    // Perform the terrain masking check if the detection was successful and if
    // the masking check was not performed internally as part of the detection
    // processing.
    //
    // Also see NOTE in multi-beam processing.
    int terrainStatusMask = (WsfSensorResult::cXMTR_TERRAIN_MASKING |
                             WsfSensorResult::cRCVR_TERRAIN_MASKING);
    int terrainCheckedStatus = aResult.mCheckedStatus & terrainStatusMask;
    int terrainFailedStatus = aResult.mFailedStatus & terrainStatusMask;
    if ((aResult.mFailedStatus == 0) && (terrainCheckedStatus == 0)) {
      aResult.MaskedByTerrain();
      terrainCheckedStatus = aResult.mCheckedStatus & terrainStatusMask;
      terrainFailedStatus = aResult.mFailedStatus & terrainStatusMask;
    }
    GetSensor()->ScriptAllowDetection(aSimTime, aTargetPtr, aResult);
    GetSensor()->NotifySensorDetectionAttempted(aSimTime, aTargetPtr, aResult);
    if (aResult.GetTransmitter() != nullptr) {
      aResult.GetTransmitter()->NotifyListeners(
          aSimTime, aResult); // Notify listeners for possible passive detection
    }
    if (GetSensor()->DebugEnabled()) {
      auto out = ut::log::debug()
                 << "Radar sensor attempting to detect target.";
      out.AddNote() << "T = " << aSimTime;
      out.AddNote() << "Platform: " << GetPlatform()->GetName();
      out.AddNote() << "Sensor: " << GetSensor()->GetName();
      out.AddNote() << "Mode: " << GetName();
      out.AddNote() << "Target: " << aTargetPtr->GetName();
      if (mBeamList.size() > 1) {
        out.AddNote() << "Beam: 1";
      }
      auto note = out.AddNote() << "Result:";
      aResult.Print(note);
    }

    if (mBeamList.size() > 1) {
      // NOTE: Terrain masking used to be checked AFTER all beams had performed
      // the basic detection processing. Unfortunately this prevented the
      // SensorDetectionAttempt observers from knowing if terrain would mask
      // target. Now terrain masking is now checked on first beam that passes
      // the all other detection criteria and is simply propagated to the
      // subsequent beams.
      //
      // NOTE: This processing cannot be performed for multi-beam bistatic
      // systems because each beam could be receiving from a different
      // transmitter. So the terrain masking will be checked for EACH beam that
      // successfully receives a detectable signal. I'm not sure if we'll ever
      // encounter multi-beam bistatic systems, but just in case...

      WsfSensorResult tempResult;
      for (unsigned int beamIndex = 1; beamIndex < mBeamList.size();
           ++beamIndex) {
        tempResult.Reset(aSettings);
        tempResult.mBeamIndex = beamIndex;
        // Always force a terrain check for multi-beam bistatic (based on first
        // beam)
        if (!aResult.mBistatic) {
          terrainCheckedStatus = 0;
          terrainFailedStatus = 0;
        }
        tempResult.mCheckedStatus =
            ut::safe_cast<unsigned int, int>(terrainCheckedStatus);
        tempResult.mFailedStatus =
            ut::safe_cast<unsigned int, int>(terrainFailedStatus);
        mBeamList[beamIndex]->AttemptToDetect(aSimTime, aTargetPtr, aSettings,
                                              tempResult);

        // Perform terrain masking check (or used the cached result) if the
        // basic detection criteria passed.
        if (tempResult.mFailedStatus == 0) {
          if (terrainCheckedStatus == 0) {
            tempResult.MaskedByTerrain();
            terrainCheckedStatus =
                tempResult.mCheckedStatus & terrainStatusMask;
            terrainFailedStatus = tempResult.mFailedStatus & terrainStatusMask;
          }
          tempResult.mCheckedStatus |= terrainCheckedStatus;
          tempResult.mFailedStatus |= terrainFailedStatus;
        }
        GetSensor()->ScriptAllowDetection(aSimTime, aTargetPtr, tempResult);
        GetSensor()->NotifySensorDetectionAttempted(aSimTime, aTargetPtr,
                                                    tempResult);
        if (tempResult.GetTransmitter() != nullptr) {
          tempResult.GetTransmitter()->NotifyListeners(
              aSimTime, tempResult); // Notify listeners for possible
                                     // passive detection
        }
        if (GetSensor()->DebugEnabled()) {
          auto out = ut::log::debug() << "Beam " << beamIndex + 1 << ":";
          tempResult.Print(out);
        }

        if (tempResult.mSignalToNoise > aResult.mSignalToNoise) {
          aResult = tempResult;
        }
      }
    }

    // Compute component effects.
    /*detected = */ WsfSensorComponent::PostAttemptToDetect(
        *GetSensor(), aSimTime, aTargetPtr, aResult);

    // A successful detection occurs only if the beam with the best
    // signal-to-noise had no failing criteria.
    detected = (aResult.mFailedStatus == 0);
  } else {
    GetSensor()->NotifySensorDetectionAttempted(aSimTime, aTargetPtr, aResult);
    if (GetSensor()->DebugEnabled()) {
      auto out = ut::log::debug() << "Radar Sensor Detection Attempt Result:";
      aResult.Print(out);
    }
  }
  return detected;
}

// =================================================================================================
// virtual
void TricorderSensor::RadarMode::ScheduleAltFreqChange(double aSimTime,
                                                       int aAltFreqId /*=-1*/) {
  double changeTime =
      std::max(aSimTime, mLastAltFreqSelectTime) + mAltFreqSelectDelay;
  GetSimulation()->AddEvent(
      ut::make_unique<AltFrequencyChangeEvent>(changeTime, this, aAltFreqId));
  mAltFreqChangeScheduled = true;
}

// =================================================================================================
//! Changes to the next alternate frequency set for the current mode for all
//! beams.
//! @param aSimTime   The current simulation time.
//! @param aAltFreqId The ID of the frequency set to change to.
// virtual
void TricorderSensor::RadarMode::SelectAlternateFrequency(
    double aSimTime, int aAltFreqId /*=-1*/) {
  for (auto &beam : mBeamList) {
    int id = aAltFreqId;
    if (id < 0) {
      id = beam->mXmtrPtr->GetCurrentAlternateFrequencyId() + 1;
    }
    beam->mXmtrPtr->SelectAlternateFrequency(id);
  }
  mBeamList[0]->mXmtrPtr->NotifyChangeListeners(aSimTime,
                                                GetPlatform()->GetIndex());
  // Limit frequency change monitoring to sim runtime.  When
  // randomize_radar_frequencies is enabled, this method is also called during
  // sim load/init as each mode is being parsed/initialized, but frequency
  // details are not yet available.  Note that checking sim->IsActive() works
  // for mission but not necessarily for all applications, hence the use of
  // aSimTime here.
  if (aSimTime > 0.0) {
    WsfObserver::SensorFrequencyChanged(GetSimulation())(aSimTime, GetSensor(),
                                                         this);
  }
  mLastAltFreqSelectTime = aSimTime;
  mAltFreqChangeScheduled = false;
}

// =================================================================================================
// virtual
void TricorderSensor::RadarMode::Deselect(double /*aSimTime*/) {
  for (auto &beam : mBeamList) {
    if (mCanReceive) {
      beam->mRcvrPtr->Deactivate();
    }
    if (mCanTransmit) {
      beam->mXmtrPtr->Deactivate();
    }
  }
  // Update the lists of active transmitters and receivers for GetEM_Xmtr and
  // GetEM_Rcvr.
  TricorderSensor *sensorPtr = dynamic_cast<TricorderSensor *>(GetSensor());
  if (sensorPtr != nullptr) {
    sensorPtr->UpdateXmtrRcvrLists();
  }
}

// =================================================================================================
// virtual
void TricorderSensor::RadarMode::Select(double aSimTime) {
  for (auto &beam : mBeamList) {
    if (mCanReceive) {
      beam->mRcvrPtr->Activate();
    }
    if (mCanTransmit) {
      beam->mXmtrPtr->Activate();
    }
  }
  // Update the lists of active transmitters and receivers for GetEM_Xmtr and
  // GetEM_Rcvr.
  TricorderSensor *sensorPtr = dynamic_cast<TricorderSensor *>(GetSensor());
  if (sensorPtr != nullptr) {
    sensorPtr->UpdateXmtrRcvrLists();
  }

  if (mCanTransmit) {
    mBeamList[0]->mXmtrPtr->NotifyChangeListeners(aSimTime,
                                                  GetPlatform()->GetIndex());
  }
}

WsfEM_Xmtr *TricorderSensor::RadarBeam::GetEM_Xmtr() { return mXmtrPtr.get(); }

double TricorderSensor::RadarBeam::GetAdjustmentFactor() const {
  return mAdjustmentFactor;
}
double TricorderSensor::RadarBeam::GetIntegrationGain() const {
  return mIntegrationGain;
}
void TricorderSensor::RadarBeam::SetIntegrationGain(double aIntegrationGain) {
  mIntegrationGain = aIntegrationGain;
}
int TricorderSensor::RadarBeam::GetNumberOfPulsesIntegrated() const {
  return mNumberOfPulsesIntegrated;
}
bool TricorderSensor::RadarBeam::UsingDetector() const { return mUseDetector; }
wsf::MarcumSwerling &TricorderSensor::RadarBeam::GetDetector() {
  return mDetector;
}

std::shared_ptr<wsf::DetectionProbabilityTable> &
TricorderSensor::RadarBeam::GetProbabilityTable() {
  return mProbabilityTablePtr;
}

double TricorderSensor::RadarBeam::GetDopplerResolution() const {
  return mDopplerResolution;
}
WsfEM_Clutter *TricorderSensor::RadarBeam::GetClutter() const {
  return mClutterPtr;
}
double TricorderSensor::RadarBeam::GetClutterAttenuationFactor() const {
  return mClutterAttenuationFactor;
}
WsfStringId TricorderSensor::RadarBeam::GetClutterType() const {
  return mClutterType;
}

size_t TricorderSensor::RadarMode::GetBeamCount() const {
  return mBeamList.size();
}
WsfSensorBeam *TricorderSensor::RadarMode::GetBeamEntry(size_t aIndex) {
  return mBeamList[aIndex];
}

double TricorderSensor::RadarMode::GetAltFreqSelectDelay() const {
  return mAltFreqSelectDelay;
}

bool TricorderSensor::RadarMode::IsAltFreqChangeScheduled() const {
  return mAltFreqChangeScheduled;
}
double TricorderSensor::RadarMode::GetLastAltFreqSelectTime() const {
  return mLastAltFreqSelectTime;
}
bool TricorderSensor::RadarMode::IsFrequencyAgile() const {
  return mIsFrequencyAgile;
}

#include "script/WsfScriptContext.hpp"

void TricorderSensor::createFakeTarget(double aSimTime)
{

	WsfSensorTracker::Settings fakeSettings;
	fakeSettings.mOptions =
		WsfSensorTracker::Settings::cSUPPRESS_TRACKING_CRITERIA |
		WsfSensorTracker::Settings::cSUPPRESS_DETECTION_CHANGE_MESSAGES;
	WsfTrackId fakeRequestId;
	fakeRequestId.Null();

	//std::cout << "mPendingFakeDetections:" << mPendingFakeDetections.size() << std::endl;
	if (!mPendingFakeDetections.empty()) {

		WsfSensorResult fakeResult;
		

		for (const auto &fakeDetection : mPendingFakeDetections) {
			WsfPlatform *jammerPlatform = GetSimulation()->GetPlatformByName(
				fakeDetection.jammerName.c_str());
			if (jammerPlatform == nullptr) {
				continue;
			}

			// Stable objectId: unique per fake-target slot per jammer across cycles.
			// Same id in subsequent cycles → tracker Updates the existing WsfTrack.
			const size_t fakeIndexForJammer =
				jammerFakeTargetCounts[jammerPlatform->GetIndex()]++;
			const size_t fakeObjectId =
				(jammerPlatform->GetIndex() << 16) + fakeIndexForJammer + 1;
			currentFakeObjectIds.insert(fakeObjectId);

			targetIds.insert(fakeObjectId);
			// Give each fake target its own temporary platform positioned at the
			// fake location — distinct from jammerPlatform so the tracker creates
			// a separate track identity for every fake detection.
			WsfPlatform fakeTargetPlatform(GetScenario());
			const Eigen::Vector3d &pos = fakeDetection.target->position;
			fakeTargetPlatform.SetLocationLLA(pos.y, pos.x, pos.z);
			fakeTargetPlatform.SetName(fakeDetection.jammerName+"_FakeTarget_"+std::to_string( fakeObjectId));
			
			fakeResult.Reset();
			fakeResult.mModeIndex = 0;
			fakeResult.mPd = 1.0;
			fakeResult.mCheckedStatus = WsfSensorResult::cSIGNAL_LEVEL;
			fakeResult.mFailedStatus = 0;
			// Measurement carries the position; GetLocationWCS() auto-converts
			// LLA → ECEF (UtEllipsoidalEarth) when the tracker reads it.
			fakeResult.mMeasurement.SetLocationLLA(pos.y, pos.x, pos.z);

			
				mTrackerPtr->TargetDetected(aSimTime, fakeSettings, fakeRequestId,
					fakeObjectId, &fakeTargetPlatform,
					fakeResult);
				//std::cout << "TargetDetected:" << fakeObjectId<<std::endl;			

		}
		

	}
	
}

void TricorderSensor::deleteFakeTarget(double aSimTime)
{
	WsfSensorTracker::Settings fakeSettings;
	fakeSettings.mOptions =
		WsfSensorTracker::Settings::cSUPPRESS_TRACKING_CRITERIA |
		WsfSensorTracker::Settings::cSUPPRESS_DETECTION_CHANGE_MESSAGES;
	WsfTrackId fakeRequestId;
	fakeRequestId.Null();
	for (size_t prevId : targetIds) {
		mTrackerPtr->TargetDeleted(aSimTime, fakeSettings, fakeRequestId,prevId);
	}
}

const char* TricorderSensor::GetScriptClassName() const
{
	return "TricorderSensor";
}
ScriptTricorderSensorClass::ScriptTricorderSensorClass(
    const std::string &aClassName, UtScriptTypes *aScriptTypesPtr)
    : WsfScriptSensorClass(aClassName, aScriptTypesPtr) 
{
  SetClassName("TricorderSensor");


  // Radar model parameter accessors
  AddMethod(ut::make_unique<GetAntennaGain_>("GetAntennaGain")); // double GetAntennaGain()
  AddMethod(ut::make_unique<SetAntennaGain_>("SetAntennaGain")); // SetAntennaGain(double)

  AddMethod(ut::make_unique<GetPulseWidth_>("GetPulseWidth")); // double GetPulseWidth()
  AddMethod(ut::make_unique<SetPulseWidth_>("SetPulseWidth")); // SetPulseWidth(double)

  AddMethod(ut::make_unique<GetFn_>("GetFn")); // double GetFn()
  AddMethod(ut::make_unique<SetFn_>("SetFn")); // SetFn(double)

  AddMethod(ut::make_unique<GetT0_>("GetT0")); // double GetT0()
  AddMethod(ut::make_unique<SetT0_>("SetT0")); // SetT0(double)

  AddMethod(ut::make_unique<GetL_>("GetL")); // double GetL()
  AddMethod(ut::make_unique<SetL_>("SetL")); // SetL(double)

  AddMethod(ut::make_unique<GetPulseNum_>("GetPulseNum")); // int GetPulseNum()
  AddMethod(ut::make_unique<SetPulseNum_>("SetPulseNum")); // SetPulseNum(int)

  AddMethod(ut::make_unique<GetPfa_>("GetPfa")); // double GetPfa()
  AddMethod(ut::make_unique<SetPfa_>("SetPfa")); // SetPfa(double)

  AddMethod(ut::make_unique<GetRangeAccuracy_>("GetRangeAccuracy")); // double GetRangeAccuracy()
  AddMethod(ut::make_unique<SetRangeAccuracy_>("SetRangeAccuracy")); // SetRangeAccuracy(double)

  AddMethod(ut::make_unique<GetAzimuthAccuracy_>("GetAzimuthAccuracy")); // double GetAzimuthAccuracy()
  AddMethod(ut::make_unique<SetAzimuthAccuracy_>("SetAzimuthAccuracy")); // SetAzimuthAccuracy(double)

  AddMethod(ut::make_unique<GetAzimuthBeamwidth_>("GetAzimuthBeamwidth"));
  AddMethod(ut::make_unique<GetElevationBeamwidth_>("GetElevationBeamwidth"));

  AddMethod(ut::make_unique<GetMaximumRange_>("GetMaximumRange"));
}



// double antennaGain = <x>.GetAntennaGain();
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        GetAntennaGain_, 0, "double", "") {
  aReturnVal.SetDouble(aObjectPtr->mRadarAntennaGain_);
}

// <x>.SetAntennaGain(double aAntennaGain);
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        SetAntennaGain_, 1, "void", "double") {
  // Argument 1: double aAntennaGain (dB)
  aObjectPtr->mRadarAntennaGain_ = aVarArgs[0].GetDouble();
}

// double pulseWidth = <x>.GetPulseWidth();
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        GetPulseWidth_, 0, "double", "") {
  aReturnVal.SetDouble(aObjectPtr->mRadarPulseWidth_);
}

// <x>.SetPulseWidth(double aPulseWidth);
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        SetPulseWidth_, 1, "void", "double") {
  // Argument 1: double aPulseWidth (s)
  aObjectPtr->mRadarPulseWidth_ = aVarArgs[0].GetDouble();
}

// double fn = <x>.GetFn();
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        GetFn_, 0, "double", "") {
  aReturnVal.SetDouble(aObjectPtr->mRadarFn_);
}

// <x>.SetFn(double aFn);
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        SetFn_, 1, "void", "double") {
  // Argument 1: double aFn (dB, noise figure)
  aObjectPtr->mRadarFn_ = aVarArgs[0].GetDouble();
}

// double t0 = <x>.GetT0();
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        GetT0_, 0, "double", "") {
  aReturnVal.SetDouble(aObjectPtr->mRadarT0_);
}

// <x>.SetT0(double aT0);
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        SetT0_, 1, "void", "double") {
  // Argument 1: double aT0 (K, noise temperature)
  aObjectPtr->mRadarT0_ = aVarArgs[0].GetDouble();
}

// double l = <x>.GetL();
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        GetL_, 0, "double", "") {
  aReturnVal.SetDouble(aObjectPtr->mRadarL_);
}

// <x>.SetL(double aL);
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        SetL_, 1, "void", "double") {
  // Argument 1: double aL (dB, system loss)
  aObjectPtr->mRadarL_ = aVarArgs[0].GetDouble();
}

// int pulseNum = <x>.GetPulseNum();
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        GetPulseNum_, 0, "int", "") {
  aReturnVal.SetInt(aObjectPtr->mRadarPulseNum_);
}

// <x>.SetPulseNum(int aPulseNum);
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        SetPulseNum_, 1, "void", "int") {
  // Argument 1: int aPulseNum (pulse count)
  aObjectPtr->mRadarPulseNum_ = aVarArgs[0].GetInt();
}

// double pfa = <x>.GetPfa();
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        GetPfa_, 0, "double", "") {
  aReturnVal.SetDouble(aObjectPtr->mRadarPfa_);
}

// <x>.SetPfa(double aPfa);
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        SetPfa_, 1, "void", "double") {
  // Argument 1: double aPfa (false alarm probability)
  aObjectPtr->mRadarPfa_ = aVarArgs[0].GetDouble();
}

// double rangeAccuracy = <x>.GetRangeAccuracy();
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        GetRangeAccuracy_, 0, "double", "") {
  aReturnVal.SetDouble(aObjectPtr->mRadarRangeAccuracy_);
}

// <x>.SetRangeAccuracy(double aRangeAccuracy);
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        SetRangeAccuracy_, 1, "void", "double") {
  // Argument 1: double aRangeAccuracy (m)
  aObjectPtr->mRadarRangeAccuracy_ = aVarArgs[0].GetDouble();
}

// double azimuthAccuracy = <x>.GetAzimuthAccuracy();
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        GetAzimuthAccuracy_, 0, "double", "") {
  aReturnVal.SetDouble(aObjectPtr->mRadarAzimuthAccuracy_);
}

// <x>.SetAzimuthAccuracy(double aAzimuthAccuracy);
UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
                        SetAzimuthAccuracy_, 1, "void", "double") {
  // Argument 1: double aAzimuthAccuracy (deg)
  aObjectPtr->mRadarAzimuthAccuracy_ = aVarArgs[0].GetDouble();
}

UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
	GetAzimuthBeamwidth_, 0, "double", "") {	
	aReturnVal.SetDouble(aObjectPtr->GetEM_Xmtr(0).GetAzimuthBeamwidth());
}

UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
	GetElevationBeamwidth_, 0, "double", "") {
	aReturnVal.SetDouble(aObjectPtr->GetEM_Xmtr(0).GetElevationBeamwidth());
}

UT_DEFINE_SCRIPT_METHOD(ScriptTricorderSensorClass, TricorderSensor,
	GetMaximumRange_, 0, "double", "") {
	aReturnVal.SetDouble(aObjectPtr->mRadarMaximumRange);
}