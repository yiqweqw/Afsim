// ****************************************************************************
// CUI
//
// The Advanced Framework for Simulation, Integration, and Modeling (AFSIM)
//
// Copyright 2003-2013 The Boeing Company. All rights reserved.
//
// The use, dissemination or disclosure of data in this file is subject to
// limitation or restriction. See accompanying README and LICENSE for details.
// ****************************************************************************
// ****************************************************************************
// Updated by Infoscitex, a DCS Company.
// ****************************************************************************

#include "ReconSensor_cssc_01_export.h"
#include "WsfPassiveReconSensor.hpp"
#include "UtMemory.hpp"
#include "UtPlugin.hpp"
#include "UtScriptTypes.hpp"
#include "WsfApplication.hpp"
#include "WsfApplicationExtension.hpp"
#include "WsfPlugin.hpp"
#include "WsfSensorTypes.hpp"

class WsfPassiveReconSensorRegistration : public WsfApplicationExtension
{
   public:
      WsfPassiveReconSensorRegistration() = default;

      ~WsfPassiveReconSensorRegistration() noexcept override = default;

      void AddedToApplication(WsfApplication& aApplication) override
      {
         aApplication.GetScriptTypes()->Register(
            WsfPassiveReconSensor::CreateScriptClass("WsfPassiveReconSensor",
                                                     GetApplication().GetScriptTypes()));
      }
      void ScenarioCreated(WsfScenario& aScenario) override
      {
         WsfSensorTypes::Get(aScenario).Add("WsfPassiveReconSensor",
                                            ut::make_unique<WsfPassiveReconSensor>(aScenario));
      }
};

extern "C"
{
   RECONSENSOR_CSSC_01_EXPORT void WsfPluginVersion(UtPluginVersion& aVersion)
   {
      aVersion = UtPluginVersion(WSF_PLUGIN_API_MAJOR_VERSION,
                                 WSF_PLUGIN_API_MINOR_VERSION,
                                 WSF_PLUGIN_API_COMPILER_STRING);
   }
   RECONSENSOR_CSSC_01_EXPORT void WsfPluginSetup(WsfApplication& aApplicationPtr)
   {
      aApplicationPtr.RegisterExtension("passive_recon_sensor_registration",
                                        ut::make_unique<WsfPassiveReconSensorRegistration>());
   }
}
