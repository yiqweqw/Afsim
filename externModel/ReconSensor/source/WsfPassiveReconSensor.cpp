#include "WsfPassiveReconSensor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>

#include "UtInput.hpp"
#include "UtMemory.hpp"
#include "WsfEM_Xmtr.hpp"
#include "WsfESM_Sensor.hpp"
#include "WsfPassiveSensor.hpp"
#include "WsfPlatform.hpp"
#include "WsfSensorModeList.hpp"
#include "WsfSensorResult.hpp"
#include "WsfTrack.hpp"

namespace
{
constexpr double cHorizonFactorKmPerSqrtM = 4.12;
constexpr double cMetersPerKm = 1000.0;
constexpr double cLightSpeedMps = 299792458.0;
constexpr double cBoltzmann = 1.380649e-23;
constexpr double cMilliwattToWatt = 1.0e-3;
constexpr size_t cDefaultReconAntennaBins = 360U;
constexpr size_t cFnvOffsetBasis = 1469598103934665603ULL;
constexpr size_t cFnvPrime = 1099511628211ULL;

bool IsEmitterFrequencyInReceiverBand(const WsfEM_Rcvr* aRcvr, double aEmitterFrequencyHz)
{
   if (aRcvr == nullptr)
   {
      return false;
   }

   const auto* passiveRcvr = dynamic_cast<const WsfPassiveSensor::PassiveRcvr*>(aRcvr);
   if (passiveRcvr == nullptr)
   {
      return false;
   }

   for (const auto& band : passiveRcvr->mFrequencyBands)
   {
      if ((aEmitterFrequencyHz >= band.mLowerFrequency) && (aEmitterFrequencyHz <= band.mUpperFrequency))
      {
         return true;
      }
   }
   return false;
}

double ComputeReceiverRadarBandwidthOverlapHz(double aReconFrequencyHz,
                                              double aReconBandwidthHz,
                                              double aEmitterFrequencyHz,
                                              double aEmitterBandwidthHz)
{
   const double bpHz = std::max(aReconBandwidthHz, 1.0);
   const double brHz = std::max(aEmitterBandwidthHz, 1.0);
   const double fpl = aReconFrequencyHz - 0.5 * bpHz;
   const double fpu = aReconFrequencyHz + 0.5 * bpHz;
   const double frl = aEmitterFrequencyHz - 0.5 * brHz;
   const double fru = aEmitterFrequencyHz + 0.5 * brHz;
   if ((fru <= fpl) || (frl >= fpu))
   {
      return 0.0;
   }
   return std::min(fpu, fru) - std::max(fpl, frl);
}

bool IsFiniteNumber(double aValue)
{
   return std::isfinite(aValue);
}

void HashCombineValue(size_t& aHash, size_t aValue)
{
   aHash ^= aValue;
   aHash *= cFnvPrime;
}

size_t QuantizedDouble(double aValue)
{
   if (!IsFiniteNumber(aValue))
   {
      return 0U;
   }
   const double scaled = std::round(aValue * 1.0e6);
   return static_cast<size_t>(std::llround(scaled));
}



double DbToPower(double aDb)
{
   return std::pow(10.0, aDb / 10.0);
}

double PowerToDbm(double aPowerW)
{
   return 10.0 * std::log10(std::max(aPowerW, std::numeric_limits<double>::min()) / cMilliwattToWatt);
}

double MeanValue(const std::vector<double>& aValues)
{
   if (aValues.empty())
   {
      return 0.0;
   }
   const double sum = std::accumulate(aValues.begin(), aValues.end(), 0.0);
   return sum / static_cast<double>(aValues.size());
}

double StdDevValue(const std::vector<double>& aValues, double aMean)
{
   if (aValues.size() <= 1U)
   {
      return 0.0;
   }

   double sqSum = 0.0;
   for (double value : aValues)
   {
      const double delta = value - aMean;
      sqSum += delta * delta;
   }
   return std::sqrt(sqSum / static_cast<double>(aValues.size() - 1U));
}

double MedianValue(std::vector<double> aValues)
{
   if (aValues.empty())
   {
      return 0.0;
   }
   std::sort(aValues.begin(), aValues.end());
   const size_t mid = aValues.size() / 2;
   if ((aValues.size() % 2) == 0U)
   {
      return 0.5 * (aValues[mid - 1] + aValues[mid]);
   }
   return aValues[mid];
}

std::vector<double> EstimateSdifPriCandidates(const std::vector<double>& aSortedToa,
                                              size_t aNeighborWindow,
                                              size_t aMaxHistBins,
                                              size_t aTopK)
{
   std::vector<double> priCandidates;
   if (aSortedToa.size() < 3U)
   {
      return priCandidates;
   }

   std::vector<double> diffs;
   for (size_t i = 0; i + 1U < aSortedToa.size(); ++i)
   {
      const size_t endIdx = std::min(aSortedToa.size(), i + 1U + std::max<size_t>(aNeighborWindow, 1U));
      for (size_t j = i + 1U; j < endIdx; ++j)
      {
         const double diff = aSortedToa[j] - aSortedToa[i];
         if (diff > 0.0)
         {
            diffs.push_back(diff);
         }
      }
   }
   if (diffs.empty())
   {
      return priCandidates;
   }

   const auto minMaxIt = std::minmax_element(diffs.begin(), diffs.end());
   const double minDiff = *(minMaxIt.first);
   const double maxDiff = *(minMaxIt.second);
   if (maxDiff <= minDiff)
   {
      priCandidates.push_back(minDiff);
      return priCandidates;
   }

   const size_t binCount = std::min(std::max<size_t>(1U, aMaxHistBins), diffs.size());
   const double binWidth = (maxDiff - minDiff) / static_cast<double>(binCount);
   std::vector<size_t> counts(binCount, 0U);
   for (double diff : diffs)
   {
      size_t binIdx = static_cast<size_t>((diff - minDiff) / std::max(binWidth, 1.0e-12));
      if (binIdx >= binCount)
      {
         binIdx = binCount - 1U;
      }
      ++counts[binIdx];
   }

   std::vector<size_t> sortedBinIdx(binCount);
   std::iota(sortedBinIdx.begin(), sortedBinIdx.end(), 0U);
   std::sort(sortedBinIdx.begin(),
             sortedBinIdx.end(),
             [&counts](size_t aLhs, size_t aRhs) { return counts[aLhs] > counts[aRhs]; });

   const size_t topK = std::max<size_t>(1U, aTopK);
   for (size_t rank = 0; rank < std::min(topK, sortedBinIdx.size()); ++rank)
   {
      const size_t idx = sortedBinIdx[rank];
      if (counts[idx] == 0U)
      {
         break;
      }
      priCandidates.push_back(minDiff + (static_cast<double>(idx) + 0.5) * binWidth);
   }
   return priCandidates;
}
}

WsfPassiveReconSensor::WsfPassiveReconSensor(WsfScenario& aScenario)
   : WsfESM_Sensor(aScenario)
{
   // 用 ReconMode 替换 ESM 默认 mode
   SetModeList(ut::make_unique<WsfSensorModeList>(new ReconMode()));
}

WsfSensor* WsfPassiveReconSensor::Clone() const
{
   return new WsfPassiveReconSensor(*this);
}

std::string WsfPassiveReconSensor::UdpInterfaceReconSensorVariables() const
{
   const auto* modePtr = dynamic_cast<const ReconMode*>(GetCurrentMode());
   if (modePtr == nullptr)
   {
      return "1031,0,0,0,290,0,0,0,0,0,0";
   }
   return modePtr->BuildUdp1031Message();
}

std::string WsfPassiveReconSensor::UdpInterfaceReconAntennaPattern() const
{
   const auto* modePtr = dynamic_cast<const ReconMode*>(GetCurrentMode());
   if (modePtr == nullptr)
   {
      return "";
   }
   return modePtr->BuildUdp1032Message();
}

std::vector<WsfPassiveReconSensor::UdpReconPdwRecord> WsfPassiveReconSensor::UdpInterfaceReconPdwList() const
{
   const auto* modePtr = dynamic_cast<const ReconMode*>(GetCurrentMode());
   if (modePtr == nullptr)
   {
      return {};
   }
   return modePtr->GetLastFramePdwRecords();
}

std::vector<WsfPassiveReconSensor::UdpReconEmitterPriRecord> WsfPassiveReconSensor::UdpInterfaceReconEmitterPriList() const
{
   const auto* modePtr = dynamic_cast<const ReconMode*>(GetCurrentMode());
   if (modePtr == nullptr)
   {
      return {};
   }
   return modePtr->GetLastFrameEmitterPriRecords();
}

std::vector<WsfPassiveReconSensor::UdpReconUnidentifiedEmitterRecord>
WsfPassiveReconSensor::UdpInterfaceReconUnidentifiedEmitterList() const
{
   const auto* modePtr = dynamic_cast<const ReconMode*>(GetCurrentMode());
   if (modePtr == nullptr)
   {
      return {};
   }
   return modePtr->GetLastFrameUnidentifiedEmitterRecords();
}

uint64_t WsfPassiveReconSensor::UdpInterfaceReconDataVersion() const
{
   const auto* modePtr = dynamic_cast<const ReconMode*>(GetCurrentMode());
   if (modePtr == nullptr)
   {
      return 0U;
   }
   return modePtr->GetFrameDataVersion();
}

// ==================== ReconMode ====================

WsfMode* WsfPassiveReconSensor::ReconMode::Clone() const
{
   return new ReconMode(*this);
}

bool WsfPassiveReconSensor::ReconMode::ProcessInput(UtInput& aInput)
{
   bool myCommand = true;
   std::string command(aInput.GetCommand());

   if (command == "recon_noise_fn_db")
   {
      aInput.ReadValue(mNoiseFnDb);
   }
   else if (command == "recon_noise_t0")
   {
      aInput.ReadValue(mNoiseT0K);
      aInput.ValueGreater(mNoiseT0K, 0.0);
   }
   else if (command == "recon_receiver_bandwidth")
   {
      aInput.ReadValueOfType(mEsmFallbackBandwidthHz, UtInput::cFREQUENCY);
      aInput.ValueGreater(mEsmFallbackBandwidthHz, 0.0);
   }
   else if (command == "recon_snr_scale")
   {
      aInput.ReadValueOfType(mSnrScale, UtInput::cRATIO);
      aInput.ValueGreater(mSnrScale, 0.0);
   }
   else if (command == "recon_pulses_per_capture")
   {
      int value = 0;
      aInput.ReadValue(value);
      aInput.ValueGreater(value, 0);
      mPulsesPerCapture = static_cast<size_t>(value);
   }
   else if (command == "recon_process_time")
   {
      aInput.ReadValueOfType(mProcessTimeSec, UtInput::cTIME);
      aInput.ValueGreaterOrEqual(mProcessTimeSec, 0.0);
   }
   else if (command == "recon_pdw_num_limit")
   {
      int value = 0;
      aInput.ReadValue(value);
      aInput.ValueGreater(value, 0);
      mPdwNumLimit = static_cast<size_t>(value);
   }
   else if (command == "recon_rf_tolerance")
   {
      aInput.ReadValueOfType(mRfToleranceRatio, UtInput::cRATIO);
      aInput.ValueGreater(mRfToleranceRatio, 0.0);
   }
   else if (command == "recon_pw_tolerance")
   {
      aInput.ReadValueOfType(mPwToleranceRatio, UtInput::cRATIO);
      aInput.ValueGreater(mPwToleranceRatio, 0.0);
   }
   else if (command == "recon_doa_tolerance")
   {
      aInput.ReadValueOfType(mDoaToleranceRad, UtInput::cANGLE);
      aInput.ValueGreater(mDoaToleranceRad, 0.0);
   }
   else if (command == "recon_cluster_max_distance")
   {
      aInput.ReadValue(mClusterMaxDistance);
      aInput.ValueGreater(mClusterMaxDistance, 0.0);
   }
   else if (command == "recon_min_cluster_size")
   {
      int value = 0;
      aInput.ReadValue(value);
      aInput.ValueGreater(value, 0);
      mMinClusterSize = static_cast<size_t>(value);
   }
   else if (command == "recon_sdif_neighbor_window")
   {
      int value = 0;
      aInput.ReadValue(value);
      aInput.ValueGreater(value, 0);
      mSdifNeighborWindow = static_cast<size_t>(value);
   }
   else if (command == "recon_sdif_max_hist_bins")
   {
      int value = 0;
      aInput.ReadValue(value);
      aInput.ValueGreater(value, 0);
      mSdifMaxHistogramBins = static_cast<size_t>(value);
   }
   else if (command == "recon_sdif_topk")
   {
      int value = 0;
      aInput.ReadValue(value);
      aInput.ValueGreater(value, 0);
      mSdifTopK = static_cast<size_t>(value);
   }
   else if (command == "recon_identification_threshold")
   {
      aInput.ReadValue(mIdentificationThreshold);
      aInput.ValueInClosedRange(mIdentificationThreshold, 0.0, 1.0);
   }
   else if (command == "recon_use_native_context_builder")
   {
      int value = 1;
      aInput.ReadValue(value);
      aInput.ValueInClosedRange(value, 0, 1);
      mUseNativeContextBuilder = (value != 0);
   }
   else if (command == "minFrequency")
   {
	   aInput.ReadValueOfType(minFrequency, UtInput::cFREQUENCY);
	   aInput.ValueGreater(minFrequency, 0.0);
	  
   }
   else if (command == "maxFrequency")
   {
		aInput.ReadValueOfType(maxFrequency, UtInput::cFREQUENCY);
		aInput.ValueGreater(maxFrequency, 0.0);
   }
   
   else
   {
      myCommand = WsfESM_Sensor::ESM_Mode::ProcessInput(aInput);
   }

   return myCommand;
}

bool WsfPassiveReconSensor::ReconMode::AttemptToDetect(double aSimTime,
                                                       WsfPlatform* aTargetPtr,
                                                       Settings& aSettings,
                                                       WsfSensorResult& aResult)
{
	//std::cout << "adv plat:" << aTargetPtr->GetName() << std::endl;
   if (!PrepareReconContext(aSimTime, aTargetPtr, aSettings, aResult))
   {
      return false;
   }

   // 使用 Recon 专用门限，独立决定是否采样。
   if (IsWithinHorizonRange(aTargetPtr, aResult))
   {
      // 触发帧级分选与识别处理入口。
      CaptureReconSignal(aSimTime, aTargetPtr, aResult);
      FlushFrameProcessing(aSimTime);

      return true;
   }

   if ((aResult.mCheckedStatus & WsfSensorResult::cSIGNAL_LEVEL) == 0U)
   {
      aResult.mCheckedStatus |= WsfSensorResult::cSIGNAL_LEVEL;
      aResult.mFailedStatus |= WsfSensorResult::cSIGNAL_LEVEL;
   }
   return false;
}

bool WsfPassiveReconSensor::ReconMode::WithinDetectionRange(double aSimTime, WsfPlatform* aTargetPtr)
{
	//std::cout << "Pre plat:" << aTargetPtr->GetName()<<std::endl;
	if (aTargetPtr == nullptr || GetPlatform() == nullptr)
	{
		return false;
	}

	double ownLocWCS[3] = { 0.0, 0.0, 0.0 };
	double tgtLocWCS[3] = { 0.0, 0.0, 0.0 };
	GetPlatform()->GetLocationWCS(ownLocWCS);
	aTargetPtr->GetLocationWCS(tgtLocWCS);

	const double dx = tgtLocWCS[0] - ownLocWCS[0];
	const double dy = tgtLocWCS[1] - ownLocWCS[1];
	const double dz = tgtLocWCS[2] - ownLocWCS[2];
	const double slantRangeM = std::sqrt(dx * dx + dy * dy + dz * dz);

	const double ownAltM = std::max(0.0, GetPlatform()->GetLocationLLA().mAlt);
	const double tgtAltM = std::max(0.0, aTargetPtr->GetLocationLLA().mAlt);
	const double horizonRangeM = cHorizonFactorKmPerSqrtM * (std::sqrt(ownAltM) + std::sqrt(tgtAltM)) * cMetersPerKm;

	// 侦察波束模型：圆形距离门限（仅按距离裁剪）。


	GetMaximumRange();
	if (slantRangeM > horizonRangeM)
	{
		return false;
	}
	return true;
}

bool WsfPassiveReconSensor::ReconMode::PrepareReconContext(double aSimTime,
                                                           WsfPlatform* aTargetPtr,
                                                           Settings& aSettings,
                                                           WsfSensorResult& aResult)
{
   if ((aTargetPtr == nullptr) || (GetPlatform() == nullptr))
   {
      aResult.Reset(aSettings);
      aResult.mCheckedStatus |= WsfSensorResult::cSIGNAL_LEVEL;
      aResult.mFailedStatus |= WsfSensorResult::cSIGNAL_LEVEL;
      return false;
   }

   auto* xmtr = aResult.GetTransmitter();
   auto* rcvr = aResult.GetReceiver();
   if (((xmtr == nullptr) || (rcvr == nullptr)) && mUseNativeContextBuilder)
   {
      // 仅在当前 result 没有上下文时，回退到原生链路构建一次。
      WsfSensorResult nativeResult;
      const bool nativeDetected = WsfESM_Sensor::ESM_Mode::AttemptToDetect(aSimTime, aTargetPtr, aSettings, nativeResult);
      (void)nativeDetected;
      aResult = nativeResult;
      xmtr = aResult.GetTransmitter();
      rcvr = aResult.GetReceiver();
   }

   if ((xmtr == nullptr) || (rcvr == nullptr))
   {
      ++mMissingXmtrContextCount;
      aResult.mCheckedStatus |= WsfSensorResult::cSIGNAL_LEVEL;
      aResult.mFailedStatus |= WsfSensorResult::cSIGNAL_LEVEL;
      return false;
   }

   // 关键点：显式在当前 result 上构建一次一跳交互，填充几何/波束与 xmtr/rcvr 关联数据。
   xmtr->UpdatePosition(aSimTime);
   if (aResult.BeginOneWayInteraction(xmtr, rcvr, false, true) == 0U)
   {
      aResult.SetReceiverBeamPosition();
      aResult.SetTransmitterBeamPosition();
   }
   else if (aResult.mXmtrBeam.mGain < 0.0)
   {
      // 几何未完整时至少补齐方位/波束，避免后续公式访问无效字段。
      aResult.ComputeUndefinedGeometry();
      aResult.SetTransmitterBeamPosition();
   }

   return (aResult.GetTransmitter() != nullptr);
}

bool WsfPassiveReconSensor::ReconMode::IsWithinHorizonRange(WsfPlatform* aTargetPtr, const WsfSensorResult& aResult) const
{
   if (aTargetPtr == nullptr || GetPlatform() == nullptr)
   {
      return false;
   }

   double ownLocWCS[3] = {0.0, 0.0, 0.0};
   double tgtLocWCS[3] = {0.0, 0.0, 0.0};
   GetPlatform()->GetLocationWCS(ownLocWCS);
   aTargetPtr->GetLocationWCS(tgtLocWCS);
   
   const double dx = tgtLocWCS[0] - ownLocWCS[0];
   const double dy = tgtLocWCS[1] - ownLocWCS[1];
   const double dz = tgtLocWCS[2] - ownLocWCS[2];
   const double slantRangeM = std::sqrt(dx * dx + dy * dy + dz * dz);

   const double ownAltM = std::max(0.0, GetPlatform()->GetLocationLLA().mAlt);
   const double tgtAltM = std::max(0.0, aTargetPtr->GetLocationLLA().mAlt);
   const double horizonRangeM = cHorizonFactorKmPerSqrtM * (std::sqrt(ownAltM) + std::sqrt(tgtAltM)) * cMetersPerKm;

   // 侦察波束模型：圆形距离门限（仅按距离裁剪）。
   if (slantRangeM > horizonRangeM)
   {
      return false;
   }

   // 使用无源单程接收功率方程，判断截获能量是否过阈值。
   const double esmP = ComputePassiveReceivedPowerWatts(slantRangeM, aResult);
   const double sensitivityW = ComputeReceiverSensitivityWatts(aResult);
   return esmP >= sensitivityW;
}

double WsfPassiveReconSensor::ReconMode::ComputePassiveReceivedPowerWatts(double aRangeM,
                                                                          const WsfSensorResult& aResult) const
{
   const auto* xmtr = aResult.GetTransmitter();
   if ((xmtr == nullptr) || (aRangeM <= 0.0))
   {
      return 0.0;
   }

   const double radarFcHz = xmtr->GetFrequency();
   const double radarBwHz = std::max(xmtr->GetBandwidth(), 1.0);
   auto* rcvr = aResult.GetReceiver();
   if ((rcvr == nullptr) || !IsEmitterFrequencyInReceiverBand(rcvr, radarFcHz))
   {
      return 0.0;
   }

   // 计算时假定 fp=fr（不修改接收机实际工作频率，UDP 仍上报配置值）。
   const double reconFcHz = radarFcHz;
   const double reconBwHz = std::max(rcvr->GetBandwidth(), 1.0);
   const double brBpCoinHz =
      ComputeReceiverRadarBandwidthOverlapHz(reconFcHz, reconBwHz, radarFcHz, radarBwHz);
   if (brBpCoinHz <= 0.0)
   {
      return 0.0;
   }

   const double txPowerW = std::max(xmtr->GetPeakPower() * xmtr->GetPulseWidth()*xmtr->GetPulseRepetitionFrequency(), 1.0e-12); //目标雷达的占空比需要注意 Radar1.PulseWidth*Radar1.PRF
   //std::cout << xmtr->GetPlatform()->GetName() << ":" << xmtr->GetPeakPower() << ":" << xmtr->GetPulseWidth() << ":" << xmtr->GetPulseRepetitionFrequency() << std::endl;
   const double gtLin = std::max(xmtr->GetPeakAntennaGain(), 1.0e-6);
   const double grLin = rcvr->GetPeakAntennaGain(); // 峰值增益
   double lrLin = DbToPower(rcvr->GetInternalLoss());
  
   const double lambdaM = cLightSpeedMps / std::max(radarFcHz, 1.0);

   // 对应 MATLAB EsmRecPow:
   // EsmP = Pt*Gt*Gr*lambda^2/((4*pi)^2*R^2*Lr)*(BrBpCoin/Br)
   const double numerator = txPowerW * gtLin * grLin * lambdaM * lambdaM * (brBpCoinHz / radarBwHz);
   const double denominator = (4.0 * UtMath::cPI) * (4.0 * UtMath::cPI) * aRangeM * aRangeM * lrLin; //matlab是5个0
   return numerator / std::max(denominator, 1.0e-12);
}

double WsfPassiveReconSensor::ReconMode::ComputeReceiverNoisePowerWatts(const WsfSensorResult& aResult) const
{
   const auto* rcvr = aResult.GetReceiver();
   const double bwHz = ((rcvr != nullptr) ? std::max(rcvr->GetBandwidth(), 1.0) : mEsmFallbackBandwidthHz);
   double fnLinear = DbToPower(mNoiseFnDb);
   if ((rcvr != nullptr) && (rcvr->GetNoiseFigure() > 0.0))
   {
      fnLinear = rcvr->GetNoiseFigure();
   }
   else if (rcvr != nullptr)
   {
      fnLinear = DbToPower(rcvr->GetInternalLoss());
   }
   return cBoltzmann * mNoiseT0K * fnLinear * bwHz;
}

double WsfPassiveReconSensor::ReconMode::ComputeReceiverSensitivityWatts(const WsfSensorResult& aResult) const
{
   const auto* rcvr = aResult.GetReceiver();
   const double noiseW = ComputeReceiverNoisePowerWatts(aResult);
   const double snrThreshold = ((rcvr != nullptr) && (rcvr->GetDetectionThreshold() > 0.0))
                                  ? rcvr->GetDetectionThreshold()
                                  : DbToPower(3.0);
   return noiseW * snrThreshold;
}

double WsfPassiveReconSensor::ReconMode::ComputeReceiverSensitivityDbm(const WsfSensorResult& aResult) const
{
   return PowerToDbm(ComputeReceiverSensitivityWatts(aResult));
}

double WsfPassiveReconSensor::ReconMode::ComputeReceiverSensitivityDbm(const WsfEM_Rcvr& aRcvr) const
{
   const double bwHz = std::max(aRcvr.GetBandwidth(), 1.0);
   double fnLinear = DbToPower(mNoiseFnDb);
   if (aRcvr.GetNoiseFigure() > 0.0)
   {
      fnLinear = aRcvr.GetNoiseFigure();
   }
   else
   {
      fnLinear = DbToPower(aRcvr.GetInternalLoss());
   }
   const double noiseW = cBoltzmann * mNoiseT0K * fnLinear * bwHz;
   const double snrThreshold = (aRcvr.GetDetectionThreshold() > 0.0) ? aRcvr.GetDetectionThreshold() : DbToPower(3.0);
   return PowerToDbm(noiseW * snrThreshold);
}

void WsfPassiveReconSensor::ReconMode::UpdateTrack(double aSimTime,
                                                   WsfTrack* aTrackPtr,
                                                   WsfPlatform* aTargetPtr,
                                                   WsfSensorResult& aResult)
{
   // 先沿用 ESM 原有 track 更新（包含频率信号列表、类型上报等）
   WsfESM_Sensor::ESM_Mode::UpdateTrack(aSimTime, aTrackPtr, aTargetPtr, aResult);

   // Phase 1: 预留扩展字段写入（你后续可挂分选ID/识别置信度）
   // TODO: aTrackPtr->SetXXX(...)
}

std::string WsfPassiveReconSensor::ReconMode::BuildUdp1031Message() const
{
  
   
   WsfEM_Rcvr& rcvr=GetSensor()->GetEM_Rcvr(0);
   
   std::ostringstream oss;
   oss << "1031"
	   << "," << rcvr.GetFrequency()   // 工作频率Hz（当前按侦察接收中心频率占位）
	   << "," << rcvr.GetBandwidth()   // 信号带宽Hz
	   << "," << rcvr.GetPeakAntennaGain()          // 接收天线增益 dB（当前未暴露）
	   << "," << mNoiseT0K                 // 噪声热力学温度 K
	   << "," << UtMath::LinearToDB(rcvr.GetNoiseFigure())               // 接收机噪声系数 dB
	   << "," << ComputeReceiverSensitivityDbm(rcvr)                    // 接收机灵敏度 dBm（自动计算）
	   << "," << UtMath::LinearToDB(rcvr.GetInternalLoss())                // 总损失因子 dB（暂用同一噪声损失参数）	   
	   << "," << rcvr.GetAzimuthBeamwidth()              // 波束宽度
	   << "," << 0.0                       // 波束水平角度（无源侦察默认0）
	   << "," << mPdwNumLimit             // 脉冲数量限制
	   << "," << minFrequency
	   << "," << maxFrequency;
   return oss.str();
}
/**
* @brief 判断目标角度是否在雷达扇形扫描区域内
* @param targetAngle    目标相对于雷达的角度（弧度，0 - 2π）
* @param radarAzimuth   雷达波束当前指向角度（弧度，0 - 2π）
* @param beamWidth      波束宽度（弧度，扇形覆盖范围）
* @param beamAngle      波束张角（弧度，扇形两侧各展开beamAngle / 2）
* @return double        0表示在扫描区内，正值表示逆时针偏离弧度，负值表示顺时针偏离弧度
**/
double checkAngleInSector(double targetAngle,
	double radarAzimuth,
	double beamWidth,
	double beamAngle) {
	// 归一化角度到 [0, 2π)
	auto normalizeAngle = [](double angle) -> double {
		angle = fmod(angle, 2.0 * UtMath::cPI);
		if (angle < 0) angle += 2.0 * UtMath::cPI;
		return angle;
	};

	targetAngle = normalizeAngle(targetAngle);
	radarAzimuth = normalizeAngle(radarAzimuth);
	if (beamWidth >= (2.0 * UtMath::cPI - 1e-10)) {
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
	if (diff > UtMath::cPI) diff -= 2.0 * UtMath::cPI;  // 转换到 [-π, π]

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
std::string WsfPassiveReconSensor::ReconMode::BuildUdp1032Message() const
{
	std::ostringstream oss;
	oss << "1032";
	WsfEM_Rcvr& rc=GetSensor()->GetEM_Rcvr(0);
	for (size_t idx = 0; idx < cDefaultReconAntennaBins; ++idx)
	{
		double maz = checkAngleInSector(0,
			idx * UtMath::cPI / 180.0,
			0,
			0);
		double antenna = rc.GetAntennaGain(
			rc.GetPolarization(), rc.GetFrequency(),
			maz, 0, 0,
			0);
		oss << ","<< antenna;
	}
	return oss.str();
  
}

std::vector<WsfPassiveReconSensor::UdpReconPdwRecord> WsfPassiveReconSensor::ReconMode::GetLastFramePdwRecords() const
{
   std::vector<WsfPassiveReconSensor::UdpReconPdwRecord> out;
   out.reserve(mLastFrameSignals.size());
   for (size_t idx = 0; idx < mLastFrameSignals.size(); ++idx)
   {
      const auto& signal = mLastFrameSignals[idx];
      WsfPassiveReconSensor::UdpReconPdwRecord record;
      record.mToa = signal.mTime;
      record.mPw = signal.mPulseWidthSec;
      record.mRf = signal.mFrequencyHz;
      record.mPa = signal.mPa;
      record.mDoa = signal.mAz;
      record.mPulseId = idx + 1U;
      record.mSourceId = (idx < mCurrentFramePulseSourceIds.size()) ? mCurrentFramePulseSourceIds[idx] : 0U;
      out.push_back(record);
   }
   return out;
}

std::vector<WsfPassiveReconSensor::UdpReconEmitterPriRecord>
WsfPassiveReconSensor::ReconMode::GetLastFrameEmitterPriRecords() const
{
   std::vector<WsfPassiveReconSensor::UdpReconEmitterPriRecord> out;
   out.reserve(mCurrentFrameSources.size());
   for (const auto& source : mCurrentFrameSources)
   {
      WsfPassiveReconSensor::UdpReconEmitterPriRecord rec;
      const auto idIt = mKnownEmitters.find(source.mSourceId);
      if ((idIt != mKnownEmitters.end()) && !idIt->second.mName.empty())
      {
         rec.mEmitterName = idIt->second.mName;
		 rec.mPriSec = source.mHasEstimatedPri ? source.mMeanPriSec : 0.0;
		 out.push_back(rec);
      }          
   }
   return out;
}

std::vector<WsfPassiveReconSensor::UdpReconUnidentifiedEmitterRecord>
WsfPassiveReconSensor::ReconMode::GetLastFrameUnidentifiedEmitterRecords() const
{
   return mLastFrameUnidentifiedEmitters;
}

size_t WsfPassiveReconSensor::ReconMode::ComputeFrameDigest(const std::vector<ReconSignalRecord>& aSignals) const
{
   size_t digest = cFnvOffsetBasis;
   HashCombineValue(digest, aSignals.size());
   for (size_t idx = 0; idx < aSignals.size(); ++idx)
   {
      const auto& signal = aSignals[idx];
      HashCombineValue(digest, idx + 1U);
      HashCombineValue(digest, QuantizedDouble(signal.mTime));
      HashCombineValue(digest, QuantizedDouble(signal.mPulseWidthSec));
      HashCombineValue(digest, QuantizedDouble(signal.mFrequencyHz));
      HashCombineValue(digest, QuantizedDouble(signal.mPa));
      HashCombineValue(digest, QuantizedDouble(signal.mAz));
      HashCombineValue(digest, (idx < mCurrentFramePulseSourceIds.size()) ? mCurrentFramePulseSourceIds[idx] : 0U);
   }

   HashCombineValue(digest, mCurrentFrameSources.size());
   for (const auto& source : mCurrentFrameSources)
   {
      HashCombineValue(digest, source.mSourceId);
      HashCombineValue(digest, source.mNumPulses);
      HashCombineValue(digest, source.mHasEstimatedPri ? 1U : 0U);
      HashCombineValue(digest, QuantizedDouble(source.mMeanPriSec));

      const auto idIt = mKnownEmitters.find(source.mSourceId);
      if ((idIt != mKnownEmitters.end()) && !idIt->second.mName.empty())
      {
         HashCombineValue(digest, std::hash<std::string>{}(idIt->second.mName));
      }
      else
      {
         HashCombineValue(digest, std::hash<std::string>{}(std::string("Source_") + std::to_string(source.mSourceId)));
      }
   }
   return digest;
}

void WsfPassiveReconSensor::ReconMode::CaptureReconSignal(double aSimTime,
                                                          WsfPlatform* aTargetPtr,
                                                          const WsfSensorResult& aResult)
{
   if ((aTargetPtr == nullptr) || (GetPlatform() == nullptr) || (aResult.GetTransmitter() == nullptr))
   {
      return;
   }

   // 脉冲参数测量模型（对齐 ESMJammerSim）：
   // 由 SNR 推导测量方差，再注入高斯测量误差。
   auto* xmtr = aResult.GetTransmitter();
   const double rangeM = std::max(aResult.mRcvrToXmtr.mRange, 1.0);
   const double esmP = ComputePassiveReceivedPowerWatts(rangeM, aResult);
   const double esmPn = ComputeReceiverNoisePowerWatts(aResult);
   const double esmSnr = std::max(esmP / std::max(esmPn, 1.0e-30) / mSnrScale, 1.0e-9);

   const double pulseWidthSec = std::max(xmtr->GetPulseWidth(), 1.0e-9);
   const double prfHz = std::max(xmtr->GetPulseRepetitionFrequency(), 1.0);
   const double radarBwHz = std::max(xmtr->GetBandwidth(), 1.0);
   const double radarFcHz = xmtr->GetFrequency();
   const double paBase = std::sqrt(std::max(esmP, 0.0) * 50.0);

   const double sigmaRF = std::sqrt(3.0) / (UtMath::cPI * pulseWidthSec * std::sqrt(esmSnr));
   const double sigmaDOA = std::sqrt(3.0) / (UtMath::cPI * 10.0 * std::sqrt(esmSnr));
   const double sigmaPW = std::sqrt(2.0) / (radarBwHz * std::sqrt(esmSnr));
   const double sigmaPRF = prfHz / (std::sqrt(2.0) * UtMath::cPI * std::sqrt(esmSnr));
   const double sigmaPA = std::sqrt(std::max(esmP, 1.0e-18) * 50.0) / std::sqrt(4.0 * pulseWidthSec * radarBwHz * esmSnr);

   std::normal_distribution<double> rfNoise(0.0, sigmaRF);
   std::normal_distribution<double> doaNoise(0.0, sigmaDOA);
   std::normal_distribution<double> pwNoise(0.0, sigmaPW);
   std::normal_distribution<double> prfNoise(0.0, sigmaPRF);
   std::normal_distribution<double> paNoise(0.0, sigmaPA);

   const size_t targetIndex = aTargetPtr->GetIndex();
   const size_t emitterPartId = (xmtr->GetArticulatedPart() != nullptr) ? xmtr->GetArticulatedPart()->GetUniqueId() : 0;
   auto& bucket = mFrameSignalBuckets[targetIndex];
   bucket.reserve(bucket.size() + mPulsesPerCapture);

   for (size_t pulseIdx = 0; pulseIdx < mPulsesPerCapture; ++pulseIdx)
   {
      const double noisyPrf = std::max(prfHz + prfNoise(mRng), 1.0);
      ReconSignalRecord rec;
      rec.mTime = aSimTime + (rangeM / cLightSpeedMps) + static_cast<double>(pulseIdx) / noisyPrf;
      rec.mTargetIndex = targetIndex;
      rec.mEmitterPartId = emitterPartId;
      rec.mFrequencyHz = std::max(radarFcHz + rfNoise(mRng), 1.0);
      rec.mBandwidthHz = radarBwHz;
      rec.mPulseWidthSec = std::max(pulseWidthSec + pwNoise(mRng), 1.0e-9);
      rec.mPriSec = std::max(1.0 / noisyPrf, 1.0e-9);
      rec.mSnr = 10 * log10(esmP / esmPn); //信噪比
      double pa = std::max(paBase + paNoise(mRng), 0.0);
	  
	  rec.mPower = xmtr->GetPeakPower(); //伏度 电压
	 // std::cout << targetIndex << " :" << rec.mPower << std::endl;
	  rec.mPa = pa*pa / 50;
      double relLocWCS[3] = {0.0, 0.0, 0.0};
      GetPlatform()->GetRelativeLocationWCS(aTargetPtr, relLocWCS);
      GetPlatform()->ComputeAspect(relLocWCS, rec.mAz, rec.mEl); //aResult.mTgtToRcvr.mAz + doaNoise(mRng);
      //rec.mEl = aResult.mTgtToRcvr.mEl;

      rec.mTargetName = aTargetPtr->GetName();
	  double pw=xmtr->GetPeakPower();
      
      rec.mFreType = "\xE5\x9B\xBA\xE5\xAE\x9A\xE9\xA2\x91\xE7\x8E\x87\x20";
    	if (xmtr->GetFunction() == WsfEM_Xmtr::cXF_SENSOR) {
       rec.mSignalType = "\xE7\xBA\xBF\xE6\x80\xA7\xE8\xB0\x83\xE9\xA2\x91";
    	 }
    	else if (xmtr->GetFunction() == WsfEM_Xmtr::cXF_INTERFERER)
    	{
   	   rec.mSignalType = "\xE5\x99\xAA\xE5\xA3\xB0\xE5\xB9\xB2\xE6\x89\xB0";
    	}
		
      bucket.push_back(rec);
   }

   mTotalCapturedSignals += mPulsesPerCapture;
}

void WsfPassiveReconSensor::ReconMode::RunSortingHook(const std::vector<ReconSignalRecord>& aSignals,
                                                       FrameProcessingStats& aStats)
{
   mCurrentFrameSources.clear();
   mCurrentFramePulseSourceIds.clear();
   mCurrentFramePulseEstimatedPriSec.clear();
   if (aSignals.empty())
   {
      return;
   }

   const size_t signalCount = aSignals.size();
   mCurrentFramePulseSourceIds.assign(signalCount, 0U);
   mCurrentFramePulseEstimatedPriSec.assign(signalCount, 0.0);

   std::vector<double> rfFeature(signalCount, 0.0);
   std::vector<double> doaFeature(signalCount, 0.0);
   std::vector<double> pwFeature(signalCount, 0.0);
   for (size_t i = 0; i < signalCount; ++i)
   {
      rfFeature[i] = aSignals[i].mFrequencyHz / 1.0e9;
      doaFeature[i] = aSignals[i].mAz;
      pwFeature[i] = aSignals[i].mPulseWidthSec * 1.0e6;
   }

   const double rfMean = MeanValue(rfFeature);
   const double doaMean = MeanValue(doaFeature);
   const double pwMean = MeanValue(pwFeature);
   const double rfStd = std::max(StdDevValue(rfFeature, rfMean), 1.0e-9);
   const double doaStd = std::max(StdDevValue(doaFeature, doaMean), 1.0e-9);
   const double pwStd = std::max(StdDevValue(pwFeature, pwMean), 1.0e-9);

   std::vector<double> rfNorm(signalCount, 0.0);
   std::vector<double> doaNorm(signalCount, 0.0);
   std::vector<double> pwNorm(signalCount, 0.0);
   for (size_t i = 0; i < signalCount; ++i)
   {
      rfNorm[i] = (rfFeature[i] - rfMean) / rfStd;
      doaNorm[i] = (doaFeature[i] - doaMean) / doaStd;
      pwNorm[i] = (pwFeature[i] - pwMean) / pwStd;
   }

   auto RegionQuery = [&](size_t aAnchorIdx) {
      std::vector<size_t> neighbors;
      neighbors.reserve(signalCount);
      for (size_t j = 0; j < signalCount; ++j)
      {
         const bool rfClose =
            std::abs(aSignals[aAnchorIdx].mFrequencyHz - aSignals[j].mFrequencyHz) <=
            mRfToleranceRatio * std::max(aSignals[aAnchorIdx].mFrequencyHz, 1.0);
         const bool doaClose = std::abs(aSignals[aAnchorIdx].mAz - aSignals[j].mAz) <= mDoaToleranceRad;
         const bool pwClose =
            std::abs(aSignals[aAnchorIdx].mPulseWidthSec - aSignals[j].mPulseWidthSec) <=
            mPwToleranceRatio * std::max(aSignals[aAnchorIdx].mPulseWidthSec, 1.0e-9);
         if (!(rfClose && doaClose && pwClose))
         {
            continue;
         }

         const double dRf = rfNorm[aAnchorIdx] - rfNorm[j];
         const double dDoa = doaNorm[aAnchorIdx] - doaNorm[j];
         const double dPw = pwNorm[aAnchorIdx] - pwNorm[j];
         const double distance = std::sqrt(dRf * dRf + dDoa * dDoa + dPw * dPw);
         if (distance <= mClusterMaxDistance)
         {
            neighbors.push_back(j);
         }
      }
      return neighbors;
   };

   const int cUnclassified = 0;
   std::vector<int> labels(signalCount, cUnclassified);
   std::vector<bool> visited(signalCount, false);
   int nextClusterId = 0;
   
   for (size_t i = 0; i < signalCount; ++i)
   {
      if (visited[i])
      {
         continue;
      }

      visited[i] = true;
      std::vector<size_t> neighbors = RegionQuery(i);
      if (neighbors.size() < mMinClusterSize)
      {
         continue;
      }

      ++nextClusterId;
      labels[i] = nextClusterId;
      std::vector<bool> queued(signalCount, false);
      for (size_t neighborIdx : neighbors)
      {
         queued[neighborIdx] = true;
      }

      size_t expandIdx = 0;
      while (expandIdx < neighbors.size())
      {
         const size_t idx = neighbors[expandIdx++];
         if (!visited[idx])
         {
            visited[idx] = true;
            std::vector<size_t> secondNeighbors = RegionQuery(idx);
            if (secondNeighbors.size() >= mMinClusterSize)
            {
               for (size_t secondNeighborIdx : secondNeighbors)
               {
                  if (!queued[secondNeighborIdx])
                  {
                     queued[secondNeighborIdx] = true;
                     neighbors.push_back(secondNeighborIdx);
                  }
               }
            }
         }
         if (labels[idx] == cUnclassified)
         {
            labels[idx] = nextClusterId;
         }
      }
   }

   std::vector<std::vector<size_t>> clusteredSignalIndices(static_cast<size_t>(nextClusterId));
   for (size_t signalIdx = 0; signalIdx < signalCount; ++signalIdx)
   {
      if (labels[signalIdx] > 0)
      {
         clusteredSignalIndices[static_cast<size_t>(labels[signalIdx] - 1)].push_back(signalIdx);
      }
   }

   size_t sourceId = 1;
   for (size_t clusterOffset = 0; clusterOffset < clusteredSignalIndices.size(); ++clusterOffset)
   {
      const auto& memberIndices = clusteredSignalIndices[clusterOffset];
      if (memberIndices.size() < mMinClusterSize)
      {
         continue;
      }

      SortedSourceInfo src;
      src.mSourceId = sourceId++;
      src.mClusterId = clusterOffset + 1U;
      src.mSignalIndices = memberIndices;
      src.mNumPulses = memberIndices.size();
	  
      std::vector<double> rfVals;
      std::vector<double> doaVals;
      std::vector<double> pwVals;
      std::vector<double> priVals;
      std::vector<double> paVals;
      std::vector<double> snrVals;
      std::vector<double> powerVals;
      std::vector<double> toaVals;
      rfVals.reserve(memberIndices.size());
      doaVals.reserve(memberIndices.size());
      pwVals.reserve(memberIndices.size());
      priVals.reserve(memberIndices.size());
      paVals.reserve(memberIndices.size());
      snrVals.reserve(memberIndices.size());
      powerVals.reserve(memberIndices.size());
      toaVals.reserve(memberIndices.size());

      for (size_t idx : memberIndices)
      {
         const auto& rec = aSignals[idx];
         rfVals.push_back(rec.mFrequencyHz);
         doaVals.push_back(rec.mAz);
         pwVals.push_back(rec.mPulseWidthSec);
         priVals.push_back(rec.mPriSec);
         paVals.push_back(rec.mPa);
         snrVals.push_back(rec.mSnr);
         powerVals.push_back(rec.mPower);
         toaVals.push_back(rec.mTime);
      }

      src.mMeanRfHz = MeanValue(rfVals);
      src.mStdRfHz = StdDevValue(rfVals, src.mMeanRfHz);
      src.mMeanDoaRad = MeanValue(doaVals);
      src.mStdDoaRad = StdDevValue(doaVals, src.mMeanDoaRad);
      src.mMeanPwSec = MeanValue(pwVals);
      src.mStdPwSec = StdDevValue(pwVals, src.mMeanPwSec);
      src.mMeanPa = MeanValue(paVals);
      src.mStdPa = StdDevValue(paVals, src.mMeanPa);
      src.mMeanSnr = MeanValue(snrVals);
      src.mMeanPower = MeanValue(powerVals);
      if (!memberIndices.empty())
      {
         const auto& firstSignal = aSignals[memberIndices.front()];
         src.mTargetName = firstSignal.mTargetName;
         src.mElRad = firstSignal.mEl;
         src.mFreType = firstSignal.mFreType;
         src.mSignalType = firstSignal.mSignalType;
      }

      std::sort(toaVals.begin(), toaVals.end());
      src.mPriCandidates = EstimateSdifPriCandidates(toaVals, mSdifNeighborWindow, mSdifMaxHistogramBins, mSdifTopK);
      std::vector<double> priDiffs;
      for (size_t i = 1; i < toaVals.size(); ++i)
      {
         const double dt = toaVals[i] - toaVals[i - 1];
         if (dt > 0.0)
         {
            priDiffs.push_back(dt);
         }
      }
      if (priDiffs.size() >= 2)
      {
         src.mMeanPriSec = MedianValue(priDiffs);
         src.mStdPriSec = StdDevValue(priDiffs, src.mMeanPriSec);
         src.mHasEstimatedPri = true;
      }
      else if (!src.mPriCandidates.empty())
      {
         src.mMeanPriSec = src.mPriCandidates.front();
         src.mStdPriSec = 0.0;
         src.mHasEstimatedPri = (src.mMeanPriSec > 0.0);
      }
      else
      {
         src.mMeanPriSec = MeanValue(priVals);
         src.mStdPriSec = StdDevValue(priVals, src.mMeanPriSec);
         src.mHasEstimatedPri = (src.mMeanPriSec > 0.0);
      }

      for (size_t idx : memberIndices)
      {
         mCurrentFramePulseSourceIds[idx] = src.mSourceId;
         mCurrentFramePulseEstimatedPriSec[idx] = src.mMeanPriSec;
      }

      mCurrentFrameSources.push_back(std::move(src));
   }

   aStats.mSortedGroupCount = mCurrentFrameSources.size();
}

WsfPassiveReconSensor::ReconMode::RadarIdentity WsfPassiveReconSensor::ReconMode::IdentifyRadar(double aRfHz,
                                                                                                  double aPwSec,
                                                                                                  double aPriSec,
                                                                                                  double& aConfidence) const
{
   // 雷达库匹配器，对应 MATLAB 的 RadarDatabase.identifyRadar。
   struct RadarRef
   {
      const char* mName;
      const char* mType;
      double mRfHz;
      double mPwSec;
      double mPriSec;
      const char* mScanType;
      double mScanRate;
      int mThreat;
   };

   static const RadarRef cRadarLib[] = {
      {"AN/APS-115", "Search", 9.6e9, 20e-6, 1.0e-3, "Circular", 6.0, 3},
      {"9S36", "FireControl", 9.7e9, 8e-6, 0.8e-3, "Conical", 60.0, 9},
      {"AN/TPS-75", "AEW", 3.0e9, 10e-6, 5.0e-3, "Sector", 4.0, 5},
      {"STING", "Tracking", 10.0e9, 0.5e-6, 0.2e-3, "Track", 0.0, 8},
      {"AN/APQ-99", "TerrainFollowing", 16.0e9, 0.1e-6, 0.1e-3, "Sector", 30.0, 4},
   };

   size_t bestIdx = 0;
   aConfidence = 0.0;
   for (size_t i = 0; i < (sizeof(cRadarLib) / sizeof(cRadarLib[0])); ++i)
   {
      const auto& radar = cRadarLib[i];
      const double rfScore = std::max(0.0, 1.0 - std::min(std::abs(aRfHz - radar.mRfHz) / radar.mRfHz, 1.0));
      const double pwScore = std::max(0.0, 1.0 - std::min(std::abs(aPwSec - radar.mPwSec) / radar.mPwSec, 1.0));
      const double priScore = std::max(0.0, 1.0 - std::min(std::abs(aPriSec - radar.mPriSec) / radar.mPriSec, 1.0));
      const double score = 0.4 * rfScore + 0.3 * pwScore + 0.3 * priScore;
      if (score > aConfidence)
      {
         aConfidence = score;
         bestIdx = i;
      }
   }

   RadarIdentity out;
   out.mName = cRadarLib[bestIdx].mName;
   out.mType = cRadarLib[bestIdx].mType;
   out.mRfHz = cRadarLib[bestIdx].mRfHz;
   out.mPwSec = cRadarLib[bestIdx].mPwSec;
   out.mPriSec = cRadarLib[bestIdx].mPriSec;
   out.mPrfHz = 1.0 / cRadarLib[bestIdx].mPriSec;
   out.mScanType = cRadarLib[bestIdx].mScanType;
   out.mScanRate = cRadarLib[bestIdx].mScanRate;
   out.mThreatLevel = cRadarLib[bestIdx].mThreat;
   out.mConfidence = aConfidence;
   if (aConfidence < mIdentificationThreshold)
   {
      out.mName = "Unknown";
      out.mType = "Unknown";
      out.mThreatLevel = 5;
   }
   return out;
}

void WsfPassiveReconSensor::ReconMode::RunIdentificationHook(FrameProcessingStats& aStats)
{
   // 对齐 MATLAB: 对每个分选源识别，构造 EDW，并按 Name 去重。
   mEdwInfoLast = mEdwInfo;
   mKnownEmitters.clear();
   mLastFrameUnidentifiedEmitters.clear();

   size_t identifiedCount = 0;
   for (const auto& source : mCurrentFrameSources)
   {
      if (!source.mHasEstimatedPri)
      {
         // MATLAB sourceInfo 识别输入默认需要有效 EstimatedPRI，无效 PRI 直接跳过。
         continue;
      }
      WsfPassiveReconSensor::UdpReconUnidentifiedEmitterRecord unknownRec;
      unknownRec.mName = source.mTargetName.empty()
         ? (std::string("Source_") + std::to_string(source.mSourceId))
         : source.mTargetName;
      unknownRec.mLevel = source.mMeanPa;
	  
      unknownRec.mAzimuth = source.mMeanDoaRad *UtMath::cDEG_PER_RAD;
      unknownRec.mElevation = source.mElRad*UtMath::cDEG_PER_RAD;
      unknownRec.mFrequencyHz = source.mMeanRfHz;
      unknownRec.mFreType = source.mFreType;
      unknownRec.mPriSec = source.mMeanPriSec;
      unknownRec.mPulseWidthSec = source.mMeanPwSec;
      unknownRec.mPower = source.mMeanPa;
      unknownRec.mSnr = source.mMeanSnr;
      unknownRec.mSignalType = source.mSignalType;
      mLastFrameUnidentifiedEmitters.push_back(std::move(unknownRec));

      double confidence = 0.0;
      RadarIdentity id = IdentifyRadar(source.mMeanRfHz, source.mMeanPwSec, source.mMeanPriSec, confidence);
      id.mConfidence = confidence;
      
         
      

      EdwInfo edw;
      edw.mName = id.mName;
      edw.mType = id.mType;
      edw.mRfHz = id.mRfHz;
      edw.mPwSec = id.mPwSec;
      edw.mPriSec = id.mPriSec;
      edw.mPrfHz = id.mPrfHz;
      edw.mDoaRad = source.mMeanDoaRad;
      edw.mPa = source.mMeanPa;
      edw.mScanType = id.mScanType;
      edw.mScanRate = id.mScanRate;
      edw.mThreatLevel = id.mThreatLevel;
      edw.mConfidence = confidence;

      bool inList = false;
      for (const auto& existingEdw : mEdwInfo)
      {
         if (existingEdw.mName == edw.mName)
         {
            inList = true;
            break;
         }
      }
      if (!inList)
      {
         mEdwInfo.push_back(edw);
         mKnownEmitters[source.mSourceId] = id;
         if (id.mName != "Unknown")
         {
            ++identifiedCount;
         }
      }
   }

   aStats.mIdentifiedGroupCount = identifiedCount;
}

void WsfPassiveReconSensor::ReconMode::FlushFrameProcessing(double aSimTime)
{
   // 帧处理流水线：拉平缓存脉冲 -> 统计 -> 分选 -> 识别 -> 清桶。

   

   size_t bufferedSignalCount = 0;
   for (const auto& kv : mFrameSignalBuckets)
   {
      bufferedSignalCount += kv.second.size();
   }

   if (bufferedSignalCount <= mPdwNumLimit)
   {
	   return;
   }

   FrameProcessingStats stats;
   stats.mTargetCount = mFrameSignalBuckets.size();

   std::vector<ReconSignalRecord> frameSignals;
   for (const auto& kv : mFrameSignalBuckets)
   {
      frameSignals.insert(frameSignals.end(), kv.second.begin(), kv.second.end());
   }

   stats.mSignalCount = frameSignals.size();
   if (!frameSignals.empty())
   {
      double snrSum = 0.0;
      for (const auto& rec : frameSignals)
      {
         snrSum += rec.mSnr;
      }
      stats.mMeanSnr = snrSum / static_cast<double>(frameSignals.size());
   }

   RunSortingHook(frameSignals, stats);
   RunIdentificationHook(stats);

   const size_t currentDigest = ComputeFrameDigest(frameSignals);
   if ((mFrameDataVersion == 0U) || (currentDigest != mLastFrameDigest))
   {
      mLastFrameDigest = currentDigest;
      mLastFrameSignals = frameSignals;
      ++mFrameDataVersion;
   }
   mLastFrameStats = stats;
   mFrameSignalBuckets.clear();
   mLastFlushTime = aSimTime;
   ++mTotalProcessedFrames;
}

// ==================== Script Class ====================

class WsfScriptPassiveReconSensorClass : public WsfScriptPassiveSensorClass
{
public:
   WsfScriptPassiveReconSensorClass(const std::string& aClassName, UtScriptTypes* aScriptTypesPtr)
      : WsfScriptPassiveSensorClass(aClassName, aScriptTypesPtr)
   {
      SetClassName("WsfPassiveReconSensor");
   }
};

std::unique_ptr<UtScriptClass> WsfPassiveReconSensor::CreateScriptClass(const std::string& aClassName,
                                                                         UtScriptTypes* aScriptTypesPtr)
{
   return ut::make_unique<WsfScriptPassiveReconSensorClass>(aClassName, aScriptTypesPtr);
}