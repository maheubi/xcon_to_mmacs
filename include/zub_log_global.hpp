/*!
 * This file is part of the ZbCom library which provides basic cross-compatible communication
 * functionality for interfacing a MACS controller. For full license details see the ZbCom
 * library.
 *
 * Copyright: zub machine control AG
 */

#pragma once

#include <boost/filesystem/path.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/trivial.hpp>
#include <string>
#include <vector>



#define LOGFILE "./zubco.log"

#define DEBUG                                                           \
    BOOST_LOG_SEV(global_logger::get(), boost::log::trivial::debug)     \
        << "(" << boost::filesystem::path(__FILE__).filename().string() \
        << ":" << __LINE__ << ", " << __FUNCTION__ << ") "
#define INFO                                                            \
    BOOST_LOG_SEV(global_logger::get(), boost::log::trivial::info)      \
        << "(" << boost::filesystem::path(__FILE__).filename().string() \
        << ":" << __LINE__ << ", " << __FUNCTION__ << ") "
#define WARNING                                                         \
    BOOST_LOG_SEV(global_logger::get(), boost::log::trivial::warning)   \
        << "(" << boost::filesystem::path(__FILE__).filename().string() \
        << ":" << __LINE__ << ", " << __FUNCTION__ << ") "
#define ERROR                                                           \
    BOOST_LOG_SEV(global_logger::get(), boost::log::trivial::error)     \
        << "(" << boost::filesystem::path(__FILE__).filename().string() \
        << ":" << __LINE__ << ", " << __FUNCTION__ << ") "

BOOST_LOG_GLOBAL_LOGGER(global_logger,
    boost::log::sources::
        severity_logger_mt<boost::log::trivial::severity_level>)
        
enum log_level_t {
    log_level_trace,
    log_level_debug,
    log_level_info,
    log_level_warning,
    log_level_error,
    log_level_fatal
};

/*!
 * \brief converts a byte vector to a string in hexadecimal format
 * \param byte_vector the byte vector to be converted
 * \return the string with a whitespace separated hexadecimal numbers from the vector
 */
std::string to_string_hex(std::vector<uint8_t> byte_vector);

/*!
 * \brief enables the log
 * \param log_level for controlling the amount of log that is outputted every log message has a
 *                  level assigned, this parameter controls what level of information is written to
 *                  the log 
 */
void enable_log(log_level_t log_level);

/*!
 * \brief disables the log
 */
void disable_log();

void init_log(bool debug);
