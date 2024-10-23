/*!
 * This file is part of the ZbCom library which provides basic cross-compatible communication
 * functionality for interfacing a MACS controller. For full license details see the ZbCom
 * library.
 *
 * Copyright: zub machine control AG
 */

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

/*!
 * The controller class represents a MACS controller. Every physical MACS controller should have its
 * own instance of this class in the application program. This class is copyable and movable through
 * the use of shared pointers for the low-level interface ressources.
 */
class controller {
  public:
    controller();

    controller(std::string ip, unsigned int port);

    ~controller();

    /*!
     * \brief connects the TCP socket of the computer and the controller
     */
    void connect();

    /*!
     * \brief read a SDO from the connected controller
     * \param index the index of the SDO to read
     * \param subindex the subindex of the SDO to read
     * \param read_data_ptr pointer to the byte-vector in which the answer should be stored
     * \return -1 Operation failed, 0 Operation successful
     */
    int read_sdo(long index, long subindex, std::vector<uint8_t> *read_data_ptr);

    /*!
     * \brief write a SDO from the connected controller
     * \param index the index of the SDO to write
     * \param subindex the subindex of the SDO to write
     * \param write_data data to write in the SDO
     * \return -1 Operation failed, 0 Operation successful
     */
    int write_sdo(long index, long subindex, std::vector<uint8_t> write_data);

    /*!
     * \brief send a PDO to the connected controller
     * \param send_data data to send in the PDO
     */
    void send_pdo(std::vector<uint8_t> send_data);

    /*!
     * \brief poll one PDO from the connected controller. The function
     *              returns at most one PDO, also if multiple PDOs have been
     *              received already. Therefore the function may be called
     *              multiple times until the return value is 0.
     * \param poll_data_ptr data pointer to a byte vector in which the answer is returned
     * \return -1 no PDO was received, n number of PDOs are still available in the buffer ready to
     *         be polled
     */
    int poll_pdo(std::vector<uint8_t> *poll_data_ptr);

    /*!
     * \brief get the IP from the controller as a string
     * \return IP as a string
     */
    std::string get_ip();

  private:
    class controller_impl;
    // be aware that the use of a shared pointer here theoretically allows multiple controller
    // classes to have the same implementation
    std::shared_ptr<controller_impl> pimpl;
};
