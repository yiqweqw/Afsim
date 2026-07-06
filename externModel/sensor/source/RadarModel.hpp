// RadarModel.h
#ifndef RADAR_MODEL_H
#define RADAR_MODEL_H

#include "WsfEM_Xmtr.hpp"
#include "WsfRF_Jammer.hpp"
#include "WsfSensorResult.hpp"
#include "WsfPlatform.hpp"
#include "WsfEM_Xmtr.hpp"
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>


namespace Eigen {
struct Vector2d {
  double x, y;
  Vector2d() : x(0), y(0) {}
  Vector2d(double x_, double y_) : x(x_), y(y_) {}

  double norm() const { return std::sqrt(x * x + y * y); }
  Vector2d operator-(const Vector2d &scalar) const {
    return Vector2d(x - scalar.x, y - scalar.y);
  }
  double &operator[](int index) {
    switch (index) {
    case 0:
      return x;
    case 1:
      return y;
    default:
      break;
    }
  }
  const double operator[](int index) const {
    switch (index) {
    case 0:
      return x;
    case 1:
      return y;
    default:
      break;
    }
  }
};
struct Vector3d {
  double x, y, z;
  double M_PI = 3.14159265358979323846;
  Vector3d() : x(0), y(0), z(0) {}
  Vector3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  static Vector3d Zero() { return Vector3d(0, 0, 0); }
  Vector3d operator*(double scalar) const {
    return Vector3d(x * scalar, y * scalar, z * scalar);
  }
  Vector3d operator-(const Vector3d &scalar) const {
    return Vector3d(x - scalar.x, y - scalar.y, z - scalar.z);
  }
  Vector3d operator+(const Vector3d &scalar) const {
    return Vector3d(x + scalar.x, y + scalar.y, z + scalar.z);
  }
  double &operator[](int index) {
    switch (index) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      break;
    }
  }
  const double &operator[](int index) const {
    switch (index) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      break;
    }
  }
  double norm() const { return std::sqrt(x * x + y * y + z * z); }

  double norm(Vector3d other) const {
    const double R = 6371000.0; 

   
    double lat1_rad = y * M_PI / 180.0;
    double lon1_rad = x * M_PI / 180.0;
    double lat2_rad = other.y * M_PI / 180.0;
    double lon2_rad = other.x * M_PI / 180.0;

    double r1 = R + z;
    double r2 = R + other.z;


    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;

    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2) * sin(dlon / 2);
    double central_angle = 2 * atan2(sqrt(a), sqrt(1 - a));

    return sqrt(r1 * r1 + r2 * r2 - 2 * r1 * r2 * cos(central_angle));
  }
  double dot(const Vector3d &other) const {
    return x * other.x + y * other.y + z * other.z;
  }
  Vector3d normalized() const {
    double n = norm();
    if (n > 0) {
      return Vector3d(x / n, y / n, z / n);
    }
    return *this;
  }
};

struct Vector4d {
  double x, y, z, w;
  Vector4d() : x(0), y(0), z(0), w() {}
  Vector4d(double x_, double y_, double z_, double w_)
      : x(x_), y(y_), z(z_), w(w_) {}
  // double norm()const { return std::sqrt(x*x + y * y + z * z); }
};
} // namespace Eigen

class Target {
public:
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  double Rcs; 
  std::string name;
   
  Target() {}
  Target(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, double rcs,
         const std::string name)
      : position(pos), velocity(vel), Rcs(rcs), name(name) {}
};

class Jammer {
public:
  std::string platformName;
  std::string type;
  Eigen::Vector3d position;
  Eigen::Vector2d beamDirection;
  double fj;      
  double Bj;      
  double Lj;      
  double EqPower; 
  double KJC;
  double KJI;
  int FakeTargetNum;
  double PI = 3.14159265358979323846;
  double antennaGain;
  int N;
  double d;
  std::vector<double> pattern_db;
  double antenna;
  WsfEM_XmtrRcvr ante;
  bool isturnOn;
  double heading;
  WsfEM_Xmtr *xmtr;
  double mBeamMinScanAz;
  double mBeamMaxScanAz;
  double mBeamWidthAz;

  Jammer();

  void initialize();
  void calculateDerivedParameters();
  Eigen::Vector2d calculateDirection(const Eigen::Vector3d &radarPos) const {
   
    const Eigen::Vector3d vec{
        radarPos[0] - position[0],
        radarPos[1] - position[1],
        radarPos[2] - position[2],
    };

    double az = std::atan2(vec[1], vec[0]) * 180.0 / PI;
    if (az < 0.0) {
      az += 360.0;
    }
    const double hr = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
    const double el = std::atan2(vec[2], hr) * 180.0 / PI;
    return {az, el};
  }

  double calculateAntennaGain(double azDiff_deg, double elDiff_deg) const {
    
    const int num = static_cast<int>(pattern_db.size());
    if (num <= 1) {
      return antennaGain;
    }
    const double step = 180.0 / static_cast<double>(num - 1);
    if (azDiff_deg <= 90.0) {
      int idx = static_cast<int>(num / 2 + azDiff_deg / step);
      if (idx < 0) {
        idx = 0;
      }
      if (idx >= num) {
        idx = num - 1;
      }
      return pattern_db[idx];
    }
    return -60.0;
  }
};
class RadarModel {
public:
 
  RadarModel();
  explicit RadarModel(const std::map<std::string, double> &config);
  virtual ~RadarModel() = default;

  RadarModel(const RadarModel &) = delete;
  RadarModel &operator=(const RadarModel &) = delete;

 
  void setDefaultParameters();
  void applyConfig(const std::map<std::string, double> &config);
  void calculateDerivedParameters();
  void initialize();

 
  void update(double dt, double az, double az1);
  void updateScan(double dt);

  
  struct DetectionInfo {
    double time;
    std::shared_ptr<Target> target;
    double range;
    double azimuth;   
    double elevation; 
    double rangeRate;
    double snr; // dB
    double probability;
    bool fakeTarget;
    std::string jammerName; 
  };

  struct Track {
    int id;
    double creationTime;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double lastUpdateTime;
    int updateCount;
    std::vector<DetectionInfo> detections;
    double confidence;
  };

  void plotMaxDetRange(std::map<int, std::shared_ptr<Jammer>> &jammers,
	  WsfEM_Xmtr *aXmtrPtr,WsfSensorResult &aResult,double aSimTime);
  double calculateMaxpjn(std::map<int, std::shared_ptr<Jammer>> &jammers,double steeraz, double steerel,
	  WsfEM_Xmtr *aXmtrPtr, WsfSensorResult &aResult, double aSimTime);
  double calculateJammertoRadarAntenna(WsfSensorResult &aResult,
                                       std::shared_ptr<Jammer> jam,
                                       double aSimTime);
  double detectTarget(const std::shared_ptr<Target> &target, double antenna,
                      double Pjn);
  bool detectFakeTarget(double aSimTime,
                        std::map<int, std::shared_ptr<Jammer>> &jammers,
                        double& Pjn, WsfEM_Xmtr *aXmtrPtr,
                        WsfSensorResult &aResult,double pd, const std::shared_ptr<Target> &target);
  double calculatePjn(double aSimTime,
                      std::map<int, std::shared_ptr<Jammer>> &jammers,
                      WsfEM_Xmtr *aXmtrPtr, WsfSensorResult &aResult);

  
 
  void calculateTargetAngle(const Eigen::Vector3d &position_,
                            const Eigen::Vector3d &position, double &az,
                            double &el) const;
  double calculateRecTarPow(double Gr, double range,
                            const std::shared_ptr<Target> &target) const;
  double calculateRecJamPow(const std::shared_ptr<Jammer> &jammer,
                            double j2rGain, double r2jGain) const;
  double calculateDetectionProbability(double SNR) const;
  double calculateMinSnr_erfcinv() const;

  
  bool isTargetInBeam(std::pair<double, double>);
  DetectionInfo generateMeasurement(const std::shared_ptr<Target> &target,
                                    double trueRange, double snr);
  void addDetection(const DetectionInfo &detection);
  void addFakeDetection(const DetectionInfo &detection);
  Eigen::Vector3d RadarModel::addRandomOffset(Eigen::Vector3d point,
                                              double minKm = 1.0,
                                              double maxKm = 3.0);
  void RadarModel::clearDetection();
  double RadarModel::checkAngleInSector(double targetAngle,
	  double radarAzimuth,
	  double beamWidth,
	  double beamAngle);
 
  void updateTracking(const DetectionInfo &detection);
  Track createTrack(const DetectionInfo &detection);
  double calculateTrackDistance(const Track &track,
                                const DetectionInfo &detection);
  void updateTrack(Track &track, const DetectionInfo &detection);

 
  void displayInfo() const;
  double estimateMaxRange(std::map<int, std::shared_ptr<Jammer>> &jammers, double steeraz, double steerel,
	  WsfEM_Xmtr *aXmtrPtr, WsfSensorResult &aResult, double aSimTime);

  
  inline bool isOperating() const { return isOperating_; }
  inline double getTime() const { return time_; }
  inline const Eigen::Vector3d &getPosition() const { return position_; }
  inline const Eigen::Vector3d &getVelocity() const { return velocity_; }
  inline Eigen::Vector3d &setPosition(double x, double y, double z) {
    position_[0] = x;
    position_[1] = y;
    position_[2] = z;
    return position_;
  }
  inline Eigen::Vector3d &setVelocity(double x, double y, double z) {
    velocity_[0] = x;
    velocity_[1] = y;
    velocity_[2] = z;

    return velocity_;
  }
  inline double getBeamDirectionAz() const { return beamDirection_[0]; }
  inline double getBeamDirectionEl() const { return beamDirection_[1]; }
  inline const std::vector<DetectionInfo> &getDetectedTargets() const {
    return detectedTargets_;
  }
  inline const std::vector<DetectionInfo> &getFakeDetections() const {
    return detectedFakeTargets_;
  }
  inline void clearFakeDetections() { detectedFakeTargets_.clear(); }
  inline const std::vector<Track> &getTrackList() const { return trackList_; }

  // --- 第一类：config 键被注释掉的参数 ---
  inline double getAntennaGain() const { return antennaGain_; }
  inline void setAntennaGain(double v) { antennaGain_ = v; calculateDerivedParameters(); }

  inline double getPulseWidth() const { return pulseWidth_; }
  inline void setPulseWidth(double v) { pulseWidth_ = v; calculateDerivedParameters(); }

  inline double getPRF() const { return PRF_; }
  inline void setPRF(double v) { PRF_ = v; calculateDerivedParameters(); }

  inline double getDutyCycle() const { return dutyCycle_; }

  // --- 第二类：applyConfig 支持但调用时未传入的参数 ---
  inline double getFn() const { return Fn_; }
  inline void setFn(double v) { Fn_ = v; calculateDerivedParameters(); }

  inline double getT0() const { return T0_; }
  inline void setT0(double v) { T0_ = v; calculateDerivedParameters(); }

  inline double getL() const { return L_; }
  inline void setL(double v) { L_ = v; }

  inline int getPulseNum() const { return pulseNum_; }
  inline void setPulseNum(int v) { pulseNum_ = v; calculateDerivedParameters(); }

  inline double getPfa() const { return Pfa_; }
  inline void setPfa(double v) { Pfa_ = v; }

  inline double getRangeAccuracy() const { return rangeAccuracy_; }
  inline void setRangeAccuracy(double v) { rangeAccuracy_ = v; }

  inline double getAzimuthAccuracy() const { return azimuthAccuracy_; }
  inline void setAzimuthAccuracy(double v) { azimuthAccuracy_ = v; }

  
  
  std::string UdpInterfaceSensorVariables(); //雷达参数
  std::string UdpInterfaceSensorBeamWidth(); //扫描波束宽度
  std::string UdpInterfaceSensorDetect(); //探测目标信息
  std::string tempUdpSensroDetectInfo;

  std::string UdpInterfaceSensorDetectMaxRange(); //雷达原始威力范围
  std::string UdpInterfaceSensorDetectMaxRangePjn(); //雷达被干扰威力范围
  std::string UdpInterfaceSensorScanRange(); //扫描范围
  std::string UdpInterfaceSensorBeamSpeed(); //波束转动速度
  std::string UdpInterfaceSensorSimulationTime(); //仿真时间
  std::string UdpInterfaceSensorantennaPatternAgin();//雷达天线方向图

  std::map<int, double> maxRange;
  std::map<int, double> antennaPatternAgin;
  Eigen::Vector4d searchVolume_{ 0.0, 360.0, 0.0,
								30.0 };
  double realTime;

  static constexpr double LIGHT_SPEED = 299792458.0; // m/s
  static constexpr double BOLTZMANN = 1.380649e-23;  // J/K
  static constexpr double PI = 3.14159265358979323846;
  static constexpr double DEG_TO_RAD = PI / 180.0;
  static constexpr double RAD_TO_DEG = 180.0 / PI;

private:
  
  double time_{0.0};
  double lastPlotSecond_{-1.0};
  bool isOperating_{true};

  
  std::string name_{"radar"};
  std::string type_{""};

 
  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity_{0.0, 0.0, 0.0};

  
  double antennaGain_{34.0};    
  int N_{40};                   
  double d_{0.5};               
  std::vector<double> pattern_; 
  double beamwidthAz_{0.0};     
  double beamwidthEl_{0.0};     
  std::string scanType_{""};
  double scanRate_{10.0};               
  Eigen::Vector2d scanAngle_{3.0, 0.0};
  Eigen::Vector2d beamDirection_{0.0, 0.0};
   // [az_min, az_max, el_min, el_max]

 
  double peakPower_{10000};  // W
  double fc_{9.7e9};          // Hz
  double pulseWidth_{8.0e-6}; // s
  double PRF_{125000};           // Hz
  double bandwidth_{5.0e6};   // Hz

 
  double Fn_{3.0};          // dB 接收机噪声系数 dB
  double T0_{290.0};        // K
  double L_{5.0};           // dB 总的损失因子(系统损失+传播损失)dB
  double bandwidthRx_{0.0};  

 
  int pulseNum_{128};		//脉冲积累数
  double Pfa_{1.0e-6};		//虚警概率
  double Pd_{0.9};
  int PCType_{1};		//脉冲压缩类型1表示为脉冲压缩雷达 0表示不是脉冲压缩雷达
  int coherentType_{1}; //脉冲积累类型1表示为相干积累0表示非相干积累

  
  double lambda_{0.0};          
  double unambiguousRange_{0.0};
  double dutyCycle_{0.0};       
  double averagePower_{0.0};    
  double KI_{ 128 };            
  double KC_{0.0};              
  double rangeResolution_{0.0}; 
  double Pn_{0.0};              

  
  double rangeAccuracy_{50.0};  
  double azimuthAccuracy_{1.0}; 

  

  std::vector<DetectionInfo> detectedTargets_;
  std::vector<DetectionInfo> detectedFakeTargets_;
  std::vector<Track> trackList_;


  mutable std::mt19937 rng_;
  std::uniform_real_distribution<double> uniformDist_;
  std::normal_distribution<double> normalDist_;

 
  double db2pow(double db) const { return std::pow(10.0, db / 10.0); }
  double pow2db(double power) const { return 10.0 * std::log10(power); }
  double erfc(double x) const;    
  double erfcinv(double y) const;

  
  std::vector<double> generateLineArrayPattern(int N, double d,
                                               double gain) const;

  /** Random lon/lat/alt inside current radar beam (az/el limits, slant range in
   * [min,max] km). */
  Eigen::Vector3d randomPositionInBeam(double minRangeKm, double maxRangeKm);

  /** Generate a position around `center` in the bearing direction from
   *  `radarPos` to `center`, at a random distance in [minKm, maxKm] km. */
  Eigen::Vector3d randomPositionAlongBearing(const Eigen::Vector3d &radarPos,
                                             const Eigen::Vector3d &center,
                                             double minKm, double maxKm);

 
  static double normalizeAngle(double angle);
  static double wrapTo360(double angle);

  mutable std::recursive_mutex mMutex;
  mutable std::mutex  mutex_;
};

#endif // RADAR_MODEL_H