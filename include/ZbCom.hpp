/*!
 * This file is part of the ZbCom library which provides basic cross-compatible communication
 * functionality for interfacing a MACS controller. For full license details see the ZbCom
 * library.
 *
 * Copyright: zub machine control AG
 */

#pragma once

/*!
 * The ZbCom library allows TCP communication with the MACS controllers from zub. The
 * communication is based on the SDO and PDO transfer as specified in the 'CANopen' protocol. This
 * library only implements an interface over IP to the controllers. Initializing and handling a list
 * of controllers how it is typically needed in an application with multiple controllers is up to
 * the implementation of the user. For development the 'discovery_service' can be used for quickly
 * creating a controller list. See the provided example for more information how to use it.
 */

// contains the controller class which implements the interface and basic
// communication commands for the MACS controller
#include <zub_controller.hpp>

// provides the discovery service for discovering MACS controller in the local
// network
#include <zub_discovery.hpp>

// contains some basic commands for byte handling
#include <zub_util.hpp>

// contains the global commands to enable/disable the log
#include <zub_log_global.hpp>
