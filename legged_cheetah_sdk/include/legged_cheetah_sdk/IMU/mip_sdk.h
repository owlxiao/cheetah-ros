/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk.h
//! @author  Nathan Miller
//! @version 1.1
//
//! @description Top-level Include file for MIP SDK
//
// External dependencies:
//
//
//
//!@copyright 2014 Lord Microstrain Sensing Systems.
//
//!@section CHANGES
//!
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, LORD MICROSTRAIN SENSING SYSTEMS
//! SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES
//! WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR
//! THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN
//! CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _MIP_SDK_H
#define _MIP_SDK_H

////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
extern "C" {
#endif

#include "legged_cheetah_sdk/IMU/mip_sdk_interface.h"
#include "legged_cheetah_sdk/IMU/mip_sdk_user_functions.h"
#include "mip.h"


#include "legged_cheetah_sdk/IMU/mip_sdk_3dm.h"
#include "legged_cheetah_sdk/IMU/mip_sdk_ahrs.h"
#include "legged_cheetah_sdk/IMU/mip_sdk_base.h"
#include "legged_cheetah_sdk/IMU/mip_sdk_filter.h"
#include "legged_cheetah_sdk/IMU/mip_sdk_gps.h"
#include "legged_cheetah_sdk/IMU/mip_sdk_system.h"


#ifdef __cplusplus
}
#endif

#endif