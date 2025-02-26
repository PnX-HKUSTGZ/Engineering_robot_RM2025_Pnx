// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__CRC_HPP_
#define RM_SERIAL_DRIVER__CRC_HPP_

#include <stdint.h>

/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */
uint32_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength);

/**
 * @brief Append CRC16 value to the end of the buffer
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return none
 */
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/**
 * @brief CRC16 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
 */
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

/**
 * @brief CRC8 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */
uint32_t Verify_CRC8_Check_Sum(const uint8_t *pchMessage, uint16_t dwLength);

/**
 * @brief Append CRC8 value to the end of the buffer
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return none
 */
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);

/**
 * @brief CRC8 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @param[in] ucCRC8 : CRC8 init value(default : 0xFF)
 * @return : CRC8 checksum
 */
uint8_t Get_CRC8_Check_Sum(const uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8);

#endif // RM_SERIAL_DRIVER__CRC_HPP_
