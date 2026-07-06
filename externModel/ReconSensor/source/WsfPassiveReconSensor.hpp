#ifndef WSFPASSIVE_RECON_SENSOR_HPP
#define WSFPASSIVE_RECON_SENSOR_HPP

#include "ReconSensor_cssc_01_export.h"

#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "WsfESM_Sensor.hpp"

class UtScriptClass;
class UtScriptTypes;
class UtInput;
class WsfTrack;
class WsfPlatform;
class WsfSensorResult;

// ��Դ��촫���������� WsfESM_Sensor ��չ
class RECONSENSOR_CSSC_01_EXPORT WsfPassiveReconSensor : public WsfESM_Sensor
{
public:
   struct UdpReconPdwRecord
   {
      double mToa = 0.0;
      double mPw = 0.0;
      double mRf = 0.0;
      double mPa = 0.0;
      double mDoa = 0.0;
      size_t mPulseId = 0;
      size_t mSourceId = 0;
   };

   struct UdpReconEmitterPriRecord
   {
      std::string mEmitterName;
      double mPriSec = 0.0;
   };
   
   struct UdpReconUnidentifiedEmitterRecord
   {
      std::string mName;
      double mLevel = 0.0;
      double mAzimuth = 0.0;
      double mElevation = 0.0;
      double mFrequencyHz = 0.0;
      std::string mFreType;
      double mPulseWidthSec = 0.0;
      double mPriSec = 0.0;
      double mPower = 0.0;
      double mSnr = 0.0;
      std::string mSignalType;
   };

   struct ReconSignalRecord
   {
      double mTime = 0.0;
      size_t mTargetIndex = 0;
      size_t mEmitterPartId = 0;
      std::string mTargetName;

      // �����ػ����
      double mFrequencyHz = 0.0;
      double mBandwidthHz = 0.0;
      double mPulseWidthSec = 0.0;
      double mPriSec = 0.0;
      double mSnr = 0.0;
      double mPa = 0.0;
      double mPower = 0.0;
      std::string mFreType;
      std::string mSignalType;

      // ��λ/�������������������䣩
      double mAz = 0.0;
      double mEl = 0.0;
   };

   class RECONSENSOR_CSSC_01_EXPORT ReconMode : public WsfESM_Sensor::ESM_Mode
   {
   public:
      struct FrameProcessingStats
      {
         size_t mSignalCount = 0;
         size_t mTargetCount = 0;
         double mMeanSnr = 0.0;
         size_t mSortedGroupCount = 0;
         size_t mIdentifiedGroupCount = 0;
      };

      struct SortedSourceInfo
      {
         size_t mSourceId = 0;
         size_t mClusterId = 0;
         size_t mNumPulses = 0;
         std::vector<size_t> mSignalIndices;
         std::vector<double> mPriCandidates;
         double mMeanRfHz = 0.0;
         double mStdRfHz = 0.0;
         double mMeanPwSec = 0.0;
         double mStdPwSec = 0.0;
         double mMeanPriSec = 0.0;
         double mStdPriSec = 0.0;
         double mMeanDoaRad = 0.0;
         double mStdDoaRad = 0.0;
         double mElRad = 0.0;
         double mMeanPa = 0.0;
         double mStdPa = 0.0;
         double mMeanSnr = 0.0;
         double mMeanPower = 0.0;
         std::string mFreType;
         std::string mSignalType;
         std::string mTargetName;
         bool mHasEstimatedPri = false;
      };
	  
	  
      struct RadarIdentity
      {
         std::string mName;
         std::string mType;
         double mRfHz = 0.0;
         double mPwSec = 0.0;
         double mPriSec = 0.0;
         double mPrfHz = 0.0;
         std::string mScanType;
         double mScanRate = 0.0;
         int mThreatLevel = 0;
         double mConfidence = 0.0;
      };

      struct EdwInfo
      {
         std::string mName;
         std::string mType;
         double mRfHz = 0.0;
         double mPwSec = 0.0;
         double mPriSec = 0.0;
         double mPrfHz = 0.0;
         double mDoaRad = 0.0;
         double mPa = 0.0;
         std::string mScanType;
         double mScanRate = 0.0;
         int mThreatLevel = 0;
         double mConfidence = 0.0;

      };

      ReconMode() = default;
      ReconMode(const ReconMode&) = default;
      ReconMode& operator=(const ReconMode&) = default;
      ~ReconMode() override = default;

      WsfMode* Clone() const override;

      bool ProcessInput(UtInput& aInput) override;

      bool AttemptToDetect(double aSimTime,
                           WsfPlatform* aTargetPtr,
                           Settings& aSettings,
                           WsfSensorResult& aResult) override;
      bool WithinDetectionRange(double aSimTime, WsfPlatform* aTargetPtr) override;

      void UpdateTrack(double aSimTime,
                       WsfTrack* aTrackPtr,
                       WsfPlatform* aTargetPtr,
                       WsfSensorResult& aResult) override;

      std::string BuildUdp1031Message() const;
      std::string BuildUdp1032Message() const;
      std::vector<UdpReconPdwRecord> GetLastFramePdwRecords() const;
      std::vector<UdpReconEmitterPriRecord> GetLastFrameEmitterPriRecords() const;
      std::vector<UdpReconUnidentifiedEmitterRecord> GetLastFrameUnidentifiedEmitterRecords() const;
      uint64_t GetFrameDataVersion() const { return mFrameDataVersion; }

   private:
      size_t ComputeFrameDigest(const std::vector<ReconSignalRecord>& aSignals) const;
      bool PrepareReconContext(double aSimTime,
                               WsfPlatform* aTargetPtr,
                               Settings& aSettings,
                               WsfSensorResult& aResult);
      bool IsWithinHorizonRange(WsfPlatform* aTargetPtr, const WsfSensorResult& aResult) const;
      double ComputePassiveReceivedPowerWatts(double aRangeM, const WsfSensorResult& aResult) const;
      double ComputeReceiverNoisePowerWatts(const WsfSensorResult& aResult) const;
      double ComputeReceiverSensitivityWatts(const WsfSensorResult& aResult) const;
      double ComputeReceiverSensitivityDbm(const WsfSensorResult& aResult) const;
      double ComputeReceiverSensitivityDbm(const WsfEM_Rcvr& aRcvr) const;
      void CaptureReconSignal(double aSimTime, WsfPlatform* aTargetPtr, const WsfSensorResult& aResult);
      void RunSortingHook(const std::vector<ReconSignalRecord>& aSignals, FrameProcessingStats& aStats);
      void RunIdentificationHook(FrameProcessingStats& aStats);
      RadarIdentity IdentifyRadar(double aRfHz, double aPwSec, double aPriSec, double& aConfidence) const;
      void FlushFrameProcessing(double aSimTime);

      // Phase 1���Ȱ�Ŀ���Ͱ���棬��������滻�ɷ�ѡ�ؽṹ
      std::unordered_map<size_t, std::vector<ReconSignalRecord>> mFrameSignalBuckets;
      double mLastFlushTime = -1.0;
      FrameProcessingStats mLastFrameStats;
      size_t mTotalProcessedFrames = 0;
      size_t mTotalCapturedSignals = 0;
      std::vector<SortedSourceInfo> mCurrentFrameSources;
      std::vector<size_t> mCurrentFramePulseSourceIds;
      std::vector<double> mCurrentFramePulseEstimatedPriSec;
      std::vector<ReconSignalRecord> mLastFrameSignals;
      size_t mLastFrameDigest = 0U;
      uint64_t mFrameDataVersion = 0U;
      std::vector<EdwInfo> mEdwInfo;
      std::vector<EdwInfo> mEdwInfoLast;
      std::vector<UdpReconUnidentifiedEmitterRecord> mLastFrameUnidentifiedEmitters;
      std::unordered_map<size_t, RadarIdentity> mKnownEmitters;
      std::mt19937_64 mRng{20260519ULL};

      // �����ò�����ProcessInput��
      double mNoiseFnDb = 5.0;                // dB
      double mNoiseT0K = 290.0;               // K
      double mEsmFallbackBandwidthHz = 500.0e6;
      double mSnrScale = 5000.0;
      size_t mPulsesPerCapture = 32;   //wizard ��û��������� ��Ĭ��
      double mProcessTimeSec = 2.0;
      size_t mPdwNumLimit = 1000;
      double mRfToleranceRatio = 0.01;
      double mPwToleranceRatio = 0.10;
      double mDoaToleranceRad = 3.14159265358979323846 * 3.0 / 180.0;
      double mClusterMaxDistance = 0.5;
      size_t mMinClusterSize = 20;
      size_t mSdifNeighborWindow = 50;
      size_t mSdifMaxHistogramBins = 1000;
      size_t mSdifTopK = 10;
      double mIdentificationThreshold = 0.6;
      bool mUseNativeContextBuilder = true;
      size_t mMissingXmtrContextCount = 0;

	  double minFrequency;
	  double maxFrequency;
   };

   WsfPassiveReconSensor(WsfScenario& aScenario);
   WsfPassiveReconSensor(const WsfPassiveReconSensor&) = default;
   WsfPassiveReconSensor& operator=(const WsfPassiveReconSensor&) = delete;
   ~WsfPassiveReconSensor() override = default;

   WsfSensor* Clone() const override;

   std::string UdpInterfaceReconSensorVariables() const;
   std::string UdpInterfaceReconAntennaPattern() const;
   std::vector<UdpReconPdwRecord> UdpInterfaceReconPdwList() const;
   std::vector<UdpReconEmitterPriRecord> UdpInterfaceReconEmitterPriList() const;
   std::vector<UdpReconUnidentifiedEmitterRecord> UdpInterfaceReconUnidentifiedEmitterList() const;
   uint64_t UdpInterfaceReconDataVersion() const;

   const char* GetScriptClassName() const override { return "WsfPassiveReconSensor"; }

   static std::unique_ptr<UtScriptClass> CreateScriptClass(const std::string& aClassName,
                                                            UtScriptTypes* aScriptTypesPtr);
};

#endif