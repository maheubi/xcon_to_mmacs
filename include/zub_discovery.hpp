/*!
 * This file is part of the ZbCom library which provides basic cross-compatible communication
 * functionality for interfacing a MACS controller. For full license details see the ZbCom
 * library.
 *
 * Copyright: zub machine control AG
 */

#pragma once

#include <vector>
#include <zub_controller.hpp>
#include <memory>

/*!
 * The discovery_service broadcasts a discovery UDP package to the local
 * network. All MACS controllers that receive this package will send a
 * response. Since a UDP broadcast cannot travel across a router into a
 * different network, this is only usable for small networks and debugging
 * purposes. In production a static IP address should be used for the
 * MACS controller.
 */
class discovery_service {
public:
    discovery_service();
    ~discovery_service();

    /*!
     * \brief sends a discovery signal over UDP to the local network
     * \return returns a vector of the controllers received
     */
    controller discover(uint32_t serial_number);

private:
    class discovery_service_impl;
    std::unique_ptr<discovery_service_impl> pimpl;
};
