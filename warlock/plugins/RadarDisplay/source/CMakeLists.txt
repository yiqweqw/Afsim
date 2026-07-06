# ****************************************************************************
# CUI
#
# The Advanced Framework for Simulation, Integration, and Modeling (AFSIM)
#
# Copyright 2020 Infoscitex, a DCS Company. All rights reserved.
#
# The use, dissemination or disclosure of data in this file is subject to
# limitation or restriction. See accompanying README and LICENSE for details.
# ****************************************************************************
plugin_cmakelists_template(RadarDisplay warlock
						   REQUIRES wsf_mil
                           LIBRARIES warlock_core wkf_common sensor_cssc_01 ReconSensor_cssc_01 )