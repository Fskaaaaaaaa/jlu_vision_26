// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.
#pragma once

#include <cstdint>

namespace crc16 {
/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */
uint32_t verifyCRC16CheckSum(const uint8_t *pchMessage, uint32_t dwLength);

/**
 * @brief Append CRC16 value to the end of the buffer
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return none
 */
void appendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);

} // namespace crc16
