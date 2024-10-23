/*!
 * This file is part of the ZbCom library which provides basic cross-compatible communication
 * functionality for interfacing a MACS controller. For full license details see the ZbCom
 * library.
 *
 * Copyright: zub machine control AG
 */

#pragma once

#include <cstdint>
#include <vector>

/*!
 * \brief concatenates a byte vector of four bytes into one 32bit unsigned int
 * \param bytes the byte vector to be concatenated
 * \param index_start the following bytes from the starting index will be concatenated
 * \return the concatenated integer
 */ 
uint32_t concatenate4(std::vector<uint8_t> bytes, int index_start);

/*!
 * \brief splits an 32bit unsigned int into a byte vector
 * \param data the integer to be splitted
 * \return the byte vector with the single bytes
 */ 
std::vector<uint8_t> split4(uint32_t data);
