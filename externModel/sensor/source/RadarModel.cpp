// RadarModel.cpp
#include "RadarModel.hpp"
// CRITICAL: do NOT #include a .cpp file and do NOT include any specific
// jammer header. Including PhaserWeapon.cpp here caused its types to be
// defined in BOTH sensor_cssc_01.dll AND jammer_cssc_01.dll; MSVC's
// dynamic_cast then compared typeinfo pointers across DLL boundaries
// and returned nullptr, crashing stock WsfRF_Jammer::UpdateRepeaterResult.
// Use the abstract jammer base via WsfRF_Jammer.hpp from wsf_mil.
#include "WsfRF_Jammer.hpp"
#include "WsfEM_Rcvr.hpp"
#include "WsfEW_Result.hpp"
#include "WsfPlatform.hpp"
#include <algorithm>
#include <cmath>
#include <complex>
#include <fstream>
#include <functional>
#include <iomanip>
#include <math.h>
#include <sstream>

// Target ?? Jammer ?????��?????????????????????

// 这是雷达模型
RadarModel::RadarModel() : uniformDist_(0.0, 1.0), normalDist_(0.0, 1.0) {
  std::random_device rd;
  rng_.seed(rd());
  setDefaultParameters();
  initialize();
}

RadarModel::RadarModel(const std::map<std::string, double> &config)
    : uniformDist_(0.0, 1.0), normalDist_(0.0, 1.0) {
  std::random_device rd;
  rng_.seed(rd());
  setDefaultParameters();
  applyConfig(config);
  initialize();
}

void RadarModel::setDefaultParameters() {
  isOperating_ = true;

  // ???? PRF
  //PRF_ = 1.0 / pulseWidth_;

  // ???????
  pattern_ = generateLineArrayPattern(N_, d_, antennaGain_);

  calculateDerivedParameters();
}

void RadarModel::applyConfig(const std::map<std::string, double> &config) {
  for (const auto &pair : config) {
    
    std::string key = pair.first;
    double value = pair.second;
    if (key == "antennaGain")
      antennaGain_ = value;
    else if (key == "peakPower_") {
      peakPower_ = value;
      averagePower_ = value;
    } else if (key == "fc")
      fc_ = value;
    else if (key == "pulseWidth")
      pulseWidth_ = value;
    else if (key == "PRF")
      PRF_ = value;
    else if (key == "bandwidth")
      bandwidth_ = value;
    else if (key == "Fn")
      Fn_ = value; 
    else if (key == "T0")
      T0_ = value; // 290
    else if (key == "L")
      L_ = value; //  db
    else if (key == "pulseNum")
      pulseNum_ = static_cast<int>(value);		
    else if (key == "Pfa")
      Pfa_ = value;
    else if (key == "Pd")
      Pd_ = value;
    else if (key == "rangeAccuracy")
      rangeAccuracy_ = value;
    else if (key == "azimuthAccuracy")
      azimuthAccuracy_ = value;
    else if (key == "beamwidthAz_")
      beamwidthAz_ = value;
    else if (key == "beamwidthEl_")
      beamwidthEl_ = value;
    else if (key == "bandwidthRx_")
      bandwidthRx_ = value;
    
  }

  calculateDerivedParameters();
}

void RadarModel::calculateDerivedParameters() {
  // ????
  lambda_ = LIGHT_SPEED / fc_;

  // ?????????
  if (PRF_ > 0) {
    unambiguousRange_ = LIGHT_SPEED / (2.0 * PRF_);
  } else {
    unambiguousRange_ = std::numeric_limits<double>::infinity();
  }

  // ????
  if (pulseWidth_ > 0 && PRF_ > 0) {
    dutyCycle_ = pulseWidth_ * PRF_;
  }

  // ???????
  averagePower_ = peakPower_ * dutyCycle_;

  // ???????????????
  if (coherentType_) {
    KI_ = static_cast<double>(pulseNum_);
  } else {
    KI_ = std::sqrt(static_cast<double>(pulseNum_));
  }

  
  if (PCType_) {
    bandwidthRx_ = bandwidth_;
    KC_ = pulseWidth_ * bandwidth_;
  } else {
    bandwidthRx_ = 1.0 / pulseWidth_;
    KC_ = 1.0;
  }

 
  Pn_ = BOLTZMANN * T0_ * db2pow(Fn_) * bandwidthRx_;

  
  rangeResolution_ = LIGHT_SPEED / (2.0 * bandwidthRx_);

 
  pattern_ = generateLineArrayPattern(N_, d_, antennaGain_);
  // beamwidthAz_ = 50.76 / N_;
  // beamwidthEl_ = 50.76 / 1.0;  // ???�T????

  // std::cout << "??????????: " << name_ << std::endl;
}

void RadarModel::initialize() {
  time_ = 0.0;
  lastPlotSecond_ = -1.0;
  scanAngle_ = Eigen::Vector2d(3.0, 0.0);
  detectedTargets_.clear();
  detectedFakeTargets_.clear();
  trackList_.clear();
}

void RadarModel::update(double dt, double az, double el) {
  time_ += dt;

  if (!isOperating_) {
    return;
  }

  beamDirection_[0] = az;
  beamDirection_[1] = el;
  // updateScan(dt);
}

void RadarModel::updateScan(double dt) {
  if (scanType_ == "??????") {
    scanAngle_[0] = std::fmod(scanAngle_[0] + scanRate_ * dt, 360.0);
    // std::cout << "scanAngle_: " << scanAngle_[0] << std::endl;
    // std::cout << "aSimTime SelectTarget" << dt << std::endl;
    beamDirection_ = scanAngle_;
  } else if (scanType_ == "???????") {
    double sectorWidth = 120.0; // 120??????
    double angle =
        sectorWidth / 2.0 * std::sin(2.0 * PI * scanRate_ * dt / 180.0);
    std::cout << "scanAngle_: " << angle << std::endl;
    beamDirection_[0] = angle;
  } else if (scanType_ == "???????") {
    double stepAngle = beamwidthAz_;
    if (std::fmod(time_, 1.0 / scanRate_) < dt) {
      scanAngle_[0] = std::fmod(scanAngle_[0] + stepAngle, 360.0);
      std::cout << "scanAngle_: " << dt << " -" << scanAngle_[0] << std::endl;
      beamDirection_ = scanAngle_;
    }
  } else if (scanType_ == "???????") {
    // ??????????
  } else {
    // std::cerr << "????: ��??????????: " << scanType_ << std::endl;
  }
}
/**
 * @brief 得到2个弧度的中间值

 */
double avg_angle_rad_manual(double a, double b) {
	double diff = fmod(b - a, 2 * UtMath::cPI);
	if (diff < 0) diff += 2 * UtMath::cPI;

	// 取较短弧的中点
	if (diff > UtMath::cPI) {
		// 跨越 0 点，需要特殊处理
		diff = 2 * UtMath::cPI - diff;
		double mid = a - diff / 2.0;
		if (mid < 0) mid += 2 * UtMath::cPI;
		return mid;
	}
	else {
		return a + diff / 2.0;
	}
}

double RadarModel::calculateJammertoRadarAntenna(WsfSensorResult &aResult,
                                                 std::shared_ptr<Jammer> jam,
                                                 double aSimTime) {
	
	std::pair<double, double> angle{ 0, 0 };
	calculateTargetAngle(jam->position, position_, angle.first, angle.second);

	double offaz=checkAngleInSector(angle.first* PI / 180.0,
		(avg_angle_rad_manual(jam->mBeamMinScanAz, jam->mBeamMaxScanAz)),
		jam->mBeamWidthAz,
		0);
  WsfEM_Rcvr *rcvrPtr = aResult.GetReceiver();
  size_t interactorCount(rcvrPtr->GetInterferenceInteractorCount());
  double r2jGain_db = 0;
  if (interactorCount > 0) {
    for (size_t xmtrIndex = 0; xmtrIndex < interactorCount; ++xmtrIndex) {
      WsfEM_Xmtr *jammerXmtrPtr =
          rcvrPtr->GetInterferenceInteractorEntry(xmtrIndex);
      std::string name = jammerXmtrPtr->GetPlatform()->GetName();
      WsfEM_Interaction jammerInteraction;
      WsfEW_Result &ewJmrResult = WsfEW_Result::FindOrCreate(jammerInteraction);
      if (jam->platformName._Equal(name)) {
        // WsfRF_Jammer::ComputeTotalJammerEffects(aSimTime, jammerInteraction,
        // Use the polymorphic public static on the abstract base. WsfRF_Jammer
        // handles ANY jammer subclass (stock WSF_RF_JAMMER, PHASER_WEAPON,
        // future jammers) via the same code path — the radar therefore has
        // no compile-time dependency on any specific jammer class.
        // (ComputeSingleJammerEffect is the public form; ComputeJammerEffect
        // is protected on the base class.)
        WsfRF_Jammer::ComputeSingleJammerEffect(aSimTime, jammerXmtrPtr, aResult,
                                                jammerInteraction, /*aApplyEW_Effects=*/true);
		
        r2jGain_db = jammerXmtrPtr->GetAntennaGain(
            jam->ante.GetPolarization(), jammerXmtrPtr->GetFrequency(),
			offaz, jammerInteraction.mXmtrBeam.mEl,
            aResult.mXmtrBeam.mEBS_Az, aResult.mXmtrBeam.mEBS_El);
        r2jGain_db = pow2db(r2jGain_db);
      }
    }
  }
  return r2jGain_db;
}
double RadarModel::calculatePjn(double aSimTime,
                                std::map<int, std::shared_ptr<Jammer>> &jammers,
                                WsfEM_Xmtr *aXmtrPtr,
                                WsfSensorResult &aResult) {
  double Pjn = 0.0; 

  for (const auto &jamEntry : jammers) {
    const auto &jam = jamEntry.second;
    if (!jam->isturnOn)
      continue;
    if (jam->type != "noise_jamming") {
      continue;
    }

	
    double j2rGain_db = aXmtrPtr->GetAntennaGain(
        aXmtrPtr->GetPolarization(), aXmtrPtr->GetFrequency(),
        aResult.mXmtrBeam.mAz, aResult.mXmtrBeam.mEl, aResult.mXmtrBeam.mEBS_Az,
        aResult.mXmtrBeam.mEBS_El);
    j2rGain_db = pow2db(j2rGain_db);


    double r2jGain_db = calculateJammertoRadarAntenna(aResult, jam, aSimTime);

    Pjn += calculateRecJamPow(jam, j2rGain_db, r2jGain_db);

	//std::cout << "r2jGain_db:" << r2jGain_db << "Pjn " << Pjn << std::endl;
  }

  return Pjn;
}

double RadarModel::detectTarget(const std::shared_ptr<Target> &target,
                                double antenna, double Pjn) {

  std::pair<bool, DetectionInfo> result = {false, {}};

  if (!isOperating_) {
    return 1.0e-9;
  }

  
  std::pair<double, double> angle{0, 0};
  calculateTargetAngle(position_, target->position, angle.first, angle.second);
  //std::cout << "detection pridition:" << target->name << std::endl;
  if (!isTargetInBeam(angle)) {
    return 1.0e-6;
  }
  
  double range = position_.norm(target->position);

  
  antenna = pow2db(antenna);
  double Pr = calculateRecTarPow(antenna, range, target);

  double RecvSNR =
      pow2db(Pr / (Pn_ + Pjn)); // SNR(dB) < 8.69 dB ??????????? Pd < 0.1
                                // SNR(dB) < 8.69 dB  ??? pjn>383.5

  
  double Pd = calculateDetectionProbability(RecvSNR);

 
  if (uniformDist_(rng_) < Pd) {
    result.first = true;

    
    DetectionInfo measurement = generateMeasurement(target, range, RecvSNR);

    result.second = DetectionInfo{time_,
                                  target,
                                  measurement.range,
                                  measurement.azimuth,
                                  measurement.elevation,
                                  measurement.rangeRate,
                                  RecvSNR,
                                  Pd,
                                  false};

    
    addDetection(result.second);
  }

  if (Pd == 1)
    Pd -= 3e-10;

  // std::cout << "detection info:" << target->ID << " pd:" << Pd << std::endl;
  return Pd;
}

bool RadarModel::detectFakeTarget(
    double aSimTime, std::map<int, std::shared_ptr<Jammer>> &jammers,
    double &Pjn, WsfEM_Xmtr *aXmtrPtr, WsfSensorResult &aResult,double pd, const std::shared_ptr<Target> &target) {
  bool detected = false;
  clearFakeDetections();

  
  
  for (const auto &jammerEntry : jammers) {
	 	
    const auto &jammer = jammerEntry.second;
	//std::cout << "detectFakeTarget name:" << jammer->platformName<<" jammer size :"<< jammers.size()<<" update:"<<aSimTime<< std::endl;
    if (!jammer->isturnOn)
      continue;
    if (jammer->type != "false_target_jamming") {
      continue;
    }
	std::pair<double, double> angle{ 0, 0 };
	calculateTargetAngle(position_, jammer->position, angle.first, angle.second);

	
	if (!isTargetInBeam(angle)) {
		continue;
	}
	jammer->KJC = KC_;
	jammer->KJI = pulseNum_;

    double j2rGain_db = aXmtrPtr->GetAntennaGain(
        aXmtrPtr->GetPolarization(), aXmtrPtr->GetFrequency(),
        aResult.mXmtrBeam.mAz, aResult.mXmtrBeam.mEl, aResult.mXmtrBeam.mEBS_Az,
        aResult.mXmtrBeam.mEBS_El);
    j2rGain_db = pow2db(j2rGain_db);

    double r2jGain_db =
        calculateJammertoRadarAntenna(aResult, jammer, aSimTime);
	if (r2jGain_db < 0)continue; // 干扰波束没有照着雷达 假目标干扰失败

    double Pjd = calculateRecJamPow(jammer, j2rGain_db, r2jGain_db);
    double RecvSNR = pow2db(Pjd / (Pn_ + Pjn));
    pd = calculateDetectionProbability(RecvSNR);

    if (uniformDist_(rng_) < pd) {
      detected = true;

      const double jammerRangeKm = jammer->position.norm(position_) / 1000.0;
      for (int k = 0; k < jammer->FakeTargetNum; ++k) {
        auto target = std::make_shared<Target>();
        target->position = randomPositionAlongBearing(position_, jammer->position, 30, 40);
        target->velocity = Eigen::Vector3d::Zero();
        
        double range = position_.norm(target->position);
		
        DetectionInfo detectionInfo;
        detectionInfo.time = aSimTime;
        detectionInfo.target = target;
        detectionInfo.range = range;
        detectionInfo.fakeTarget = true;
        detectionInfo.jammerName = jammer->platformName;

        addFakeDetection(detectionInfo);
      }
    }
  }

  return detected;
}

bool RadarModel::isTargetInBeam(std::pair<double, double> angle) {
  constexpr double TINY_THRESHOLD = 1e-3;
  if (std::fabs(beamwidthAz_ - 360) < TINY_THRESHOLD) {
    beamwidthAz_ = 360.0;
  }
  double startAngle = std::fmod(beamDirection_[0] - beamwidthAz_ / 2.0, 360.0);
  double endAngle = std::fmod(beamDirection_[0] + beamwidthAz_ / 2.0, 360.0);
  if (endAngle < 0)
    endAngle += 360.0;
  if (startAngle < 0)
    startAngle += 360.0;

  if (std::fabs(startAngle) < TINY_THRESHOLD) {
    startAngle = 0.0;
  }
  if (std::fabs(endAngle) < TINY_THRESHOLD) {
    endAngle = 0.0;
  }
  double startAngleEL = beamDirection_[1] - beamwidthEl_ / 2.0;
  double endAngleEL = beamDirection_[1] + beamwidthEl_ / 2.0;

  if (startAngle == endAngle && beamwidthAz_ > 0 &&
      angle.second >= startAngleEL && angle.second <= endAngleEL)
    return true; // 360?
  if (beamwidthAz_ <= 0)
    return false;
  if (startAngle < endAngle) {
    // ??????????? [30??, 150??]
    return angle.first >= startAngle && angle.first <= endAngle &&
           angle.second >= startAngleEL && angle.second <= endAngleEL;
  } else {
    // ??? 0 ??????? [300??, 60??] ????? [300??, 360??) ?? [0??, 60??]
    return angle.first >= startAngle || angle.first <= endAngle &&
                                            angle.second >= startAngleEL &&
                                            angle.second <= endAngleEL;
  }
  return false;
}



void RadarModel::calculateTargetAngle(const Eigen::Vector3d &position_,
                                      const Eigen::Vector3d &tgtPos, double &az,
                                      double &el) const {
  double lat1Rad = position_[1] * PI / 180.0;
  double lon1Rad = position_[0] * PI / 180.0;
  double lat2Rad = tgtPos[1] * PI / 180.0;
  double lon2Rad = tgtPos[0] * PI / 180.0;

  // ???????????��???
  double deltaLon = lon2Rad - lon1Rad;
  double y = sin(deltaLon) * cos(lat2Rad);
  double x =
      cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(deltaLon);
  az = atan2(y, x);
  az = az / 3.1415926 * 180;
  az = fmod(az, 360.0);
  if (az < 0)
    az += 360.0;
  /*static constexpr double TWO_PI = 2.0 * PI;
  az = std::fmod(az, TWO_PI);
  if (az < 0) {
          az += TWO_PI;
  }*/
  // ??????????????????????????��????
  static constexpr double EARTH_RADIUS = 6371000.0; // ??????????????

  double centralAngle = acos(sin(lat1Rad) * sin(lat2Rad) +
                             cos(lat1Rad) * cos(lat2Rad) * cos(deltaLon));
  double horizontalDistance = centralAngle * EARTH_RADIUS;

  // ????????????????
  double deltaAlt = tgtPos[2] - position_[2];
  el = atan2(deltaAlt, horizontalDistance);
  el = el / 3.1415926 * 180;
  el = fmod(el, 360.0);
  if (el < 0)
    el += 360.0;
  /*el = std::fmod(el, TWO_PI);
  if (el < 0) {
          el += TWO_PI;
  }*/
}

double
RadarModel::calculateRecTarPow(double Gr, double range,
                               const std::shared_ptr<Target> &target) const {
  double Gt = Gr; // ???�F?????????????
  Gt = db2pow(Gt);
  Gr = db2pow(Gr);
  double L_dB = db2pow(L_);

  return KI_ * KC_ * averagePower_ * Gt * Gr * lambda_ * lambda_ * target->Rcs /
         (std::pow(4.0 * PI, 3) * std::pow(range, 4) * L_dB);

}

double RadarModel::calculateRecJamPow(const std::shared_ptr<Jammer> &jammer,
                                      double j2rGain, double r2jGain) const {
  j2rGain = db2pow(j2rGain);
  r2jGain = db2pow(r2jGain);
  double Lj = db2pow(jammer->Lj);

  
  Eigen::Vector3d rangeVec = jammer->position - position_;
  double range = jammer->position.norm(position_);

  double lambda_j = LIGHT_SPEED / jammer->fj;

  
  double Fjl = jammer->fj - 0.5 * jammer->Bj;
  double Fju = jammer->fj + 0.5 * jammer->Bj;
  double Frl = fc_ - 0.5 * bandwidthRx_;
  double Fru = fc_ + 0.5 * bandwidthRx_;

  double BrBjCoin = 0.0;
  if (!(Fju <= Frl || Fjl >= Fru)) {
    BrBjCoin = std::min(Fju, Fru) - std::max(Fjl, Frl);
  }
  //std::cout << jammer->KJC << " :" << jammer->KJI << " : " << jammer->EqPower << ":" << j2rGain
	//  << " :" << r2jGain << " : " << lambda_j << " : " << range << " : " << Lj <<" : "<< BrBjCoin << " : " << jammer->Bj << std::endl;
  double number4_4 = jammer->KJC * jammer->KJI * jammer->EqPower * j2rGain *
                     r2jGain * lambda_j * lambda_j /
                     (std::pow(4.0 * PI, 2) * range * range * Lj) *
                     (BrBjCoin / jammer->Bj);
  /*std::ofstream out("RadarModel_number4_4.txt", std::ios::app);
  if (out.is_open()) {
	  out   << number4_4 << '\n';
  }*/
  return number4_4;
}

double RadarModel::calculateDetectionProbability(double SNR) const {
  // ??? SNR ?? dB ??????
  double snr_linear = db2pow(SNR);

  // ???? z ?
  double z = std::sqrt(-std::log(Pfa_)) - std::sqrt(snr_linear + 0.5);

  // ???????????
  double erfc_val = erfc(z);

  return 0.5 * erfc_val;
}

double RadarModel::calculateMinSnr_erfcinv() const {
  double z = erfcinv(Pd_ / 0.5);
  double SNR = std::pow(std::sqrt(-std::log(Pfa_)) - z, 2) - 0.5;

  //std::cout << Pd_ << " :" << Pfa_ << std::endl;
  return pow2db(SNR);
}

RadarModel::DetectionInfo
RadarModel::generateMeasurement(const std::shared_ptr<Target> &target,
                                double trueRange, double snr) {
  // ??????????
  // auto[trueAz, trueEl] = calculateTargetAngle(target);
  std::pair<double, double> angle;
  calculateTargetAngle(position_, target->position, angle.first, angle.second);
  // ??????????
  double rangeError = rangeAccuracy_ * normalDist_(rng_);
  double azError = azimuthAccuracy_ * normalDist_(rng_);
  double elError = azimuthAccuracy_ * normalDist_(rng_);

  // ???????��?????
  double rangeRateError = 1.0 * normalDist_(rng_); // 1m/s

  // ?????????
  Eigen::Vector3d relativeVel = target->velocity - velocity_;
  Eigen::Vector3d rangeVec = target->position - position_;
  Eigen::Vector3d unitVec = rangeVec.normalized();
  double rangeRate = relativeVel.dot(unitVec);

  DetectionInfo measurement;
  measurement.time = time_;
  measurement.target = target;
  measurement.range = trueRange + rangeError;
  measurement.azimuth = angle.first + azError;
  measurement.elevation = angle.second + elError;
  measurement.rangeRate = rangeRate + rangeRateError;
  measurement.snr = snr;
  measurement.probability = 0.0;
  measurement.fakeTarget = false;

  return measurement;
}

void RadarModel::clearDetection() {
  // detectedTargets_.clear();
}
void RadarModel::addDetection(const DetectionInfo &detection) {
  detectedTargets_.push_back(detection);
  updateTracking(detection);
}

void RadarModel::addFakeDetection(const DetectionInfo &detection) {
  detectedFakeTargets_.push_back(detection);
}

void RadarModel::updateTracking(const DetectionInfo &detection) {
  if (detection.fakeTarget) {
    return;
  }

  double associationThreshold = 5000.0; // 5km

  if (trackList_.empty()) {
    // ?????????
    Track track = createTrack(detection);
    trackList_.push_back(track);
  } else {
    // ??????????
    bool associated = false;

    for (auto &track : trackList_) {
      double distance = calculateTrackDistance(track, detection);

      if (distance < associationThreshold) {
        // ????????????????
        updateTrack(track, detection);
        associated = true;
        break;
      }
    }

    if (!associated) {
      // ?????????
      Track track = createTrack(detection);
      trackList_.push_back(track);
    }
  }
}

RadarModel::Track RadarModel::createTrack(const DetectionInfo &detection) {
  Track track;
  track.id = static_cast<int>(trackList_.size()) + 1;
  track.creationTime = time_;
  track.position = detection.target->position;
  track.velocity = detection.target->velocity;
  track.lastUpdateTime = time_;
  track.updateCount = 1;
  track.detections.push_back(detection);
  track.confidence = 0.5;

  return track;
}

double RadarModel::calculateTrackDistance(const Track &track,
                                          const DetectionInfo &detection) {
  return (track.position - detection.target->position).norm();
}

void RadarModel::updateTrack(Track &track, const DetectionInfo &detection) {
  double alpha = 0.3; // ??????

  track.position =
      track.position * (1.0 - alpha) + detection.target->position * alpha;
  track.velocity =
      track.velocity * (1.0 - alpha) + detection.target->velocity * alpha;
  track.lastUpdateTime = time_;
  track.updateCount += 1;
  track.detections.push_back(detection);
  track.confidence = std::min(1.0, track.confidence + 0.1);
}

void RadarModel::displayInfo() const {
  
}
void RadarModel::plotMaxDetRange(std::map<int, std::shared_ptr<Jammer>> &jammers,
	WsfEM_Xmtr *aXmtrPtr, WsfSensorResult &aResult, double aSimTime)
{
	const double plotSecond = std::floor(aSimTime + 1e-9);
	{
		
		if (plotSecond <= lastPlotSecond_) {
			return;
		}
		lastPlotSecond_ = plotSecond;
	}

	std::map<int, double> nextMaxRange;
	std::map<int, double> nextAntennaPatternAgin;
	
	for (int i = 0; i < 360; ++i)
	{
		nextMaxRange[i]=estimateMaxRange(jammers,i,0, aXmtrPtr, aResult, aSimTime);		
		
		double maz = checkAngleInSector(0,
			i * PI / 180.0,
			0,
			0);
		double j2rGain_db = aXmtrPtr->GetAntennaGain(
			aXmtrPtr->GetPolarization(), aXmtrPtr->GetFrequency(),
			maz, 0, aResult.mXmtrBeam.mEBS_Az,
			aResult.mXmtrBeam.mEBS_El);
		j2rGain_db = pow2db(j2rGain_db);
		nextAntennaPatternAgin[i] = j2rGain_db; //发送天线方向图的增益
	}

	
	
	maxRange.swap(nextMaxRange);
	antennaPatternAgin.swap(nextAntennaPatternAgin);
}
double
RadarModel::estimateMaxRange(std::map<int, std::shared_ptr<Jammer>> &jammers,double steeraz, double steerel, 
	WsfEM_Xmtr *aXmtrPtr, WsfSensorResult &aResult, double aSimTime) {
  double minsnr = calculateMinSnr_erfcinv();
  double pjn = calculateMaxpjn(jammers, steeraz, steerel, aXmtrPtr, aResult, aSimTime);
  double prmin = (Pn_ + pjn) * db2pow(minsnr);
  double gt = db2pow(antennaGain_);
  double gr = db2pow(antennaGain_);
  double l_db = db2pow(L_);
  
  //std::cout << steeraz <<" :"<< averagePower_ <<" : "<< gt <<":"<< lambda_ <<" :"<< KI_ <<" : "<<KC_ <<" : "<< prmin <<" : "<< l_db << std::endl;
  
  return std::pow(averagePower_ * gt * gr * lambda_ * lambda_ * 1 * KI_ *
	  KC_ / (std::pow(4.0 * PI, 3) * prmin * l_db), 1.0 / 4.0);
  
}
double RadarModel::calculateMaxpjn(std::map<int, std::shared_ptr<Jammer>> &jammers, double steeraz, double steerel,
	WsfEM_Xmtr *aXmtrPtr, WsfSensorResult &aResult, double aSimTime)
{
	double Pjn = 0.0;

	for (const auto &jamEntry : jammers) {
		const auto &jam = jamEntry.second;
		if (!jam->isturnOn)
			continue;
		if (jam->type != "noise_jamming") {
			continue;
		}
		 
		//	std::cout << "prev:" << Pjn << std::endl;
		std::pair<double, double> angle{ 0, 0 };
		calculateTargetAngle(position_, jam->position, angle.first, angle.second);
		double maz=checkAngleInSector(angle.first* PI / 180.0,
			steeraz * PI / 180.0,
			0,
			0);

		
		double j2rGain_db = aXmtrPtr->GetAntennaGain(
			aXmtrPtr->GetPolarization(), aXmtrPtr->GetFrequency(),
			maz, 0, aResult.mXmtrBeam.mEBS_Az,
			aResult.mXmtrBeam.mEBS_El);
		j2rGain_db = pow2db(j2rGain_db);
		
		//std::cout << steeraz << " az off:" << maz << j2rGain_db << std::endl;
		
		double r2jGain_db = calculateJammertoRadarAntenna(aResult, jam, aSimTime);

 		Pjn += calculateRecJamPow(jam, j2rGain_db, r2jGain_db);
		//std::cout << "angle:" << steeraz << " Pjn: " <<  Pjn << std::endl;
		//std::cout << "adv:" << Pjn << std::endl;
		
	}
	
	return Pjn;
	
}

double RadarModel::erfc(double x) const {
  // ?????????????????
  // ??? Abramowitz and Stegun ???
  double t = 1.0 / (1.0 + 0.3275911 * std::abs(x));
  double poly =
      t * (0.254829592 +
           t * (-0.284496736 +
                t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
  double result = poly * std::exp(-x * x);

  if (x >= 0) {
    return result;
  } else {
    return 2.0 - result;
  }
}

double RadarModel::erfcinv(double y) const {
  // ?��???????????????
  if (y <= 0.0 || y >= 2.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (y > 1.0) {
    return -erfcinv(2.0 - y);
  }

  // ????????
  double t = std::sqrt(-2.0 * std::log(y / 2.0));
  double x =
      -0.70711 *
      ((2.30753 + t * 0.27061) / (1.0 + t * (0.99229 + t * 0.04481)) - t);

  // ???????????
  for (int i = 0; i < 2; ++i) {
    double err = erfc(x) - y;
    x += err / (1.12837916709551257 * std::exp(-x * x));
  }

  return x;
}

std::vector<double> RadarModel::generateLineArrayPattern(int N, double d,
                                                         double gain) const {
  std::vector<double> pattern;
  int numPoints = 181; // -90 ?? 90 ???????????
  pattern.resize(numPoints);

  // ??????????????
  double peakGain = gain;

  for (int i = 0; i < numPoints; ++i) {
    double theta = (i - 90) * DEG_TO_RAD;

    if (std::abs(theta) < 1e-6) {
      pattern[i] = peakGain;
    } else {
      // ????????
      double psi = 2.0 * PI * d / lambda_ * std::sin(theta);
      double arrayFactor = std::sin(N * psi / 2.0) / (N * std::sin(psi / 2.0));

      pattern[i] = peakGain + 20.0 * std::log10(std::abs(arrayFactor));
    }
  }

  return pattern;
}

double RadarModel::normalizeAngle(double angle) {
  angle = std::fmod(angle, 360.0);
  if (angle < 0) {
    angle += 360.0;
  }
  return angle;
}

double RadarModel::wrapTo360(double angle) {
  angle = std::fmod(angle, 360.0);
  if (angle < 0) {
    angle += 360.0;
  }

  return angle;
}

Jammer::Jammer() : antennaGain(10.0), N(1000), d(0.5), pattern_db() {
  calculateDerivedParameters();
  initialize();
}

void Jammer::initialize() { mBeamWidthAz = 0.0; }
inline double pow2db(double p) { return 10.0 * std::log10(p); }
std::vector<double> computeLineArrayPattern(int N, double d, double Gt_db) {
  const double theta_min = -90.0;
  const double theta_max = 90.0;
  const double step = 0.1;
  const int num = static_cast<int>((theta_max - theta_min) / step) + 1;

  std::vector<double> pattern_db(num, 0.0);
  std::vector<double> w(N);
  double PI = 3.14159265358979323846;
  // Hamming window
  for (int j = 0; j < N; ++j) {
    w[j] = 0.54 - 0.46 * std::cos(2.0 * PI * j / (N - 1));
  }

  for (int m = 0; m < num; ++m) {
    const double theta = theta_min + m * step;
    const double phase = 2.0 * PI * d * std::sin(theta * PI / 180.0);
    std::complex<double> af(0.0, 0.0);
    for (int j = 0; j < N; ++j) {
      const double ang = phase * j;
      af += std::complex<double>(std::cos(ang), std::sin(ang)) * w[j];
    }
    const double e = std::abs(af);
    pattern_db[m] = e * e;
  }

  // normalize and dB
  double max_p = 0.0;
  for (double p : pattern_db) {
    if (p > max_p) {
      max_p = p;
    }
  }
  if (max_p <= 0.0) {
    max_p = 1.0;
  }

  for (double &p : pattern_db) {
    p /= max_p;
    if (p <= 1.0e-6) {
      p = 1.0e-6;
    }
    p = pow2db(p) + Gt_db;
  }

  return pattern_db;
}

void Jammer::calculateDerivedParameters() {
  pattern_db = computeLineArrayPattern(N, d, antennaGain);
  // beamwidthAz = 50.76 / static_cast<double>(N);
  // beamwidthEl = 50.76 / 1.0;

  // if (type == "????????") {
  //	KJC = 1.0;
  //	KJI = 1.0;
  //	FakeTargetNum = 0;
  // }
  // else if (type == "???????") {
  //	if (targetRadarInfo.PulseNum <= 0) {
  //		throw std::runtime_error("???????????? targetRadarInfo
  //???????/???");
  //	}
  //	KJC = targetRadarInfo.KC;
  //	KJI = std::sqrt(static_cast<double>(targetRadarInfo.PulseNum));
  //	FakeTargetNum = 10;
  // }

  // if (scanType == "???????") {
  //	// ????????????��?????????????
  //	const auto& p = targetRadarInfo.position;
  //	if (!(p[0] == 0.0 && p[1] == 0.0 && p[2] == 0.0)) {
  //		auto[az, el] = calculateDirectionTo(targetRadarInfo.position);
  //		beamDirection[0] = az;
  //		beamDirection[1] = el;
  //	}
  // }
}

/**
 * ???? [min, max] ??��????????????
 */
double randomDouble(double min, double max) {
  return min + (max - min) * (rand() / (RAND_MAX + 1.0));
}

/**
 * ????��?????????????1~3 km??
 * @param point ????
 * @param minKm ??��???(km)
 * @param maxKm ??????(km)
 * @return ??????
 */
Eigen::Vector3d RadarModel::randomPositionInBeam(double minRangeKm,
                                                 double maxRangeKm) {
  constexpr double kTinyAz = 1e-3;
  const double minKm = std::min(minRangeKm, maxRangeKm);
  const double maxKm = std::max(minRangeKm, maxRangeKm);
  const double rangeM = (minKm + uniformDist_(rng_) * (maxKm - minKm)) * 1000.0;

  double azDeg;
  if (std::fabs(beamwidthAz_ - 360.0) < kTinyAz) {
    azDeg = uniformDist_(rng_) * 360.0;
  } else if (beamwidthAz_ <= 0.0) {
    azDeg = beamDirection_[0];
  } else {
    const double az0 = beamDirection_[0] - 0.5 * beamwidthAz_;
    azDeg = az0 + uniformDist_(rng_) * beamwidthAz_;
    azDeg = std::fmod(azDeg, 360.0);
    if (azDeg < 0.0) {
      azDeg += 360.0;
    }
  }

  const double elLow = beamDirection_[1] - 0.5 * beamwidthEl_;
  const double elHigh = beamDirection_[1] + 0.5 * beamwidthEl_;
  const double elDeg = elLow + uniformDist_(rng_) * (elHigh - elLow);

  const double elRad = elDeg * DEG_TO_RAD;
  const double horizM = rangeM * std::cos(elRad);
  const double deltaAlt = rangeM * std::sin(elRad);

  const double earthR = 6371000.0;
  const double angularDist = horizM / earthR;
  const double latRad = position_[1] * DEG_TO_RAD;
  const double lonRad = position_[0] * DEG_TO_RAD;
  const double azRad = azDeg * DEG_TO_RAD;

  const double newLatRad =
      std::asin(std::sin(latRad) * std::cos(angularDist) +
                std::cos(latRad) * std::sin(angularDist) * std::cos(azRad));
  const double newLonRad =
      lonRad +
      std::atan2(std::sin(azRad) * std::sin(angularDist) * std::cos(latRad),
                 std::cos(angularDist) -
                     std::sin(latRad) * std::sin(newLatRad));

  Eigen::Vector3d out;
  out[0] = newLonRad * RAD_TO_DEG;
  out[1] = newLatRad * RAD_TO_DEG;
  out[0] = std::fmod(out[0] + 540.0, 360.0) - 180.0;
  out[2] = position_[2] + deltaAlt;
  return out;
}

Eigen::Vector3d RadarModel::randomPositionAlongBearing(
    const Eigen::Vector3d &radarPos, const Eigen::Vector3d & /*center*/,
    double minKm, double maxKm) {
  // Fixed direction: beam center azimuth and elevation
  const double azRad = beamDirection_[0] * DEG_TO_RAD;
  const double elRad = beamDirection_[1] * DEG_TO_RAD;

  // Random slant range in [minKm, maxKm] km, projected to horizontal
  const double rangeKm  = minKm + uniformDist_(rng_) * (maxKm - minKm);
  const double horizKm  = rangeKm * std::cos(elRad);
  const double deltaAlt = rangeKm * std::sin(elRad) * 1000.0; // metres

  // Spherical projection from radar position
  const double angularDist = (horizKm * 1000.0) / 6371000.0;
  const double latRad = radarPos[1] * DEG_TO_RAD;
  const double lonRad = radarPos[0] * DEG_TO_RAD;

  const double newLatRad =
      std::asin(std::sin(latRad) * std::cos(angularDist) +
                std::cos(latRad) * std::sin(angularDist) * std::cos(azRad));
  const double newLonRad =
      lonRad +
      std::atan2(std::sin(azRad) * std::sin(angularDist) * std::cos(latRad),
                 std::cos(angularDist) - std::sin(latRad) * std::sin(newLatRad));

  Eigen::Vector3d out;
  out[0] = std::fmod(newLonRad * RAD_TO_DEG + 540.0, 360.0) - 180.0;
  out[1] = newLatRad * RAD_TO_DEG;
  out[2] = radarPos[2] + deltaAlt;
  return out;
}

Eigen::Vector3d RadarModel::addRandomOffset(Eigen::Vector3d point, double minKm,
                                            double maxKm) {
  // ?????????????????????��?
  static bool seeded = false;
  if (!seeded) {
    srand(time(nullptr));
    seeded = true;
  }

  // 1. ??????? 1~3 km
  double distanceKm = randomDouble(minKm, maxKm);

  // 2. ??????? 0~360??
  double bearing = randomDouble(0, 2 * PI);

  // 3. ????????
  double latRad = point.y * PI / 180.0;
  double lonRad = point.x * PI / 180.0;

  // 4. ?????
  const double EARTH_RADIUS = 6371.0;

  // 5. ?????
  double angularDist = distanceKm / EARTH_RADIUS;

  // 6. ??????????
  double newLatRad = asin(sin(latRad) * cos(angularDist) +
                          cos(latRad) * sin(angularDist) * cos(bearing));

  double newLonRad =
      lonRad + atan2(sin(bearing) * sin(angularDist) * cos(latRad),
                     cos(angularDist) - sin(latRad) * sin(newLatRad));

  // 7. ????????
  point.x = newLonRad * 180.0 / PI;
  point.y = newLatRad * 180.0 / PI;

  // 8. ????�Z???? [-180, 180]
  point.x = fmod(point.x + 540.0, 360.0) - 180.0;

  return point;
}


/**
 * @brief 判断目标角度是否在雷达扇形扫描区域内
 * @param targetAngle    目标相对于雷达的角度（弧度，0-2π）
 * @param radarAzimuth   雷达波束当前指向角度（弧度，0-2π）
 * @param beamWidth      波束宽度（弧度，扇形覆盖范围）
 * @param beamAngle      波束张角（弧度，扇形两侧各展开beamAngle/2）
 * @return double        0表示在扫描区内，正值表示逆时针偏离弧度，负值表示顺时针偏离弧度
 */
double RadarModel::checkAngleInSector(double targetAngle,
	double radarAzimuth,
	double beamWidth,
	double beamAngle) {
	// 归一化角度到 [0, 2π)
	auto normalizeAngle = [](double angle) -> double {
		angle = fmod(angle, 2.0 * PI);
		if (angle < 0) angle += 2.0 * PI;
		return angle;
	};

	targetAngle = normalizeAngle(targetAngle);
	radarAzimuth = normalizeAngle(radarAzimuth);
	if (beamWidth >= (2.0 * PI - 1e-10)) {
		return 0.0;
	}
	beamWidth = normalizeAngle(beamWidth);
	// 计算扇形的左右边界（beamWidth是扇形中心覆盖？还是整体宽度？）
	// 这里假设 beamWidth 是波束的整体宽度（波束角覆盖范围）
	// beamAngle 是扇形的锥角（波束在垂直方向的扩展，暂不考虑）
	// 对于平面扫描，主要用beamWidth表示水平覆盖角度

	double halfBeam = beamWidth / 2.0;
	double leftBound = normalizeAngle(radarAzimuth - halfBeam);
	double rightBound = normalizeAngle(radarAzimuth + halfBeam);

	// 计算目标角度与雷达指向的角度差（最短弧）
	double diff = normalizeAngle(targetAngle - radarAzimuth);
	if (diff > PI) diff -= 2.0 * PI;  // 转换到 [-π, π]

	// 检查是否在扫描区内
	if (std::abs(diff) <= halfBeam + 1e-10) {
		return 0.0;  // 在扫描区内
	}

	// 不在扫描区内，计算偏离角度
	// 判断是顺时针偏离还是逆时针偏离
	if (diff > halfBeam) {
		return diff - halfBeam;  // 逆时针偏离，正值
	}
	else {
		return diff + halfBeam;  // 顺时针偏离，负值
	}
}



//{"1002","频率","功率","带宽","脉宽","内损","天线增益","占空比","脉冲数"}
std::string RadarModel::UdpInterfaceSensorVariables() 
{
	std::ostringstream oss;
	oss << "1002"
		<< "," << peakPower_
		<< "," << fc_
		<< "," << pulseWidth_
		<< "," << dutyCycle_
		<< "," << bandwidth_
		<< "," << antennaGain_
		<< ","<< antennaGain_
		<< "," << T0_
		<<","<< Fn_
		<<"," << L_
		<<","<< pulseNum_
		<<","<< Pfa_
		<<","<< Pd_
		<<"," << PCType_
		<<","<< coherentType_
		<<","<< KI_		
		<<","<< PRF_
		;

	return oss.str();
}
//扫描波束宽度{"1003", "波束宽度"}

std::string RadarModel::UdpInterfaceSensorBeamWidth() 
{
	std::ostringstream oss;
	oss << "1003"
		<<"," <<beamDirection_[0]
		<< "," << beamwidthAz_
		;
	//std::cout << "angle limit  " << beamDirection_[0] << std::endl;
	return oss.str();
}
//探测目标信息 {"1004","平台名称","经度","纬度","高度","速度"}
std::string RadarModel::UdpInterfaceSensorDetect() 
{
	return tempUdpSensroDetectInfo;
}
//雷达原始威力范围{ "1005", "数组" }
std::string RadarModel::UdpInterfaceSensorDetectMaxRange() 
{
	std::map<int, double> maxRangeSnapshot;
	{
		
		maxRangeSnapshot = maxRange;
	}

	std::stringstream ss;
	ss << "1005,";
	for (auto it = maxRangeSnapshot.begin(); it != maxRangeSnapshot.end(); ++it) {
		if (it != maxRangeSnapshot.begin()) ss << ", ";
		ss  << it->second;
	}
	
	return ss.str();
}
// 雷达被干扰威力范围{"1006","数组"}
std::string RadarModel::UdpInterfaceSensorDetectMaxRangePjn() 
{
	return std::string();
}
// 扫描范围{"1007","最小值","最大值"}
std::string RadarModel::UdpInterfaceSensorScanRange() 
{
	std::ostringstream oss;
	oss << "1007"
		<< "," << 0
		<<","<<360
		;

	return oss.str();
}

// 波束转动速度{"1008","转速"}
std::string RadarModel::UdpInterfaceSensorBeamSpeed() 
{
	return "1008,1";
}
//仿真时间 "1009","时间"}
std::string RadarModel::UdpInterfaceSensorSimulationTime()
{
	return std::to_string(realTime);
}
//雷达原始威力范围{ "1010", "数组" }
std::string RadarModel::UdpInterfaceSensorantennaPatternAgin()
{
	std::map<int, double> antennaPatternSnapshot;
	{
		std::lock_guard<std::recursive_mutex> lock(mMutex);
		antennaPatternSnapshot = antennaPatternAgin;
	}

	std::stringstream ss;
	ss << "1010,";
	for (auto it = antennaPatternSnapshot.begin(); it != antennaPatternSnapshot.end(); ++it) {
		if (it != antennaPatternSnapshot.begin()) ss << ", ";
		ss << it->second;
	}

	return ss.str();
}