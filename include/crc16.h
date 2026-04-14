#ifndef CRC16_H
#define CRC16_H
#include <cstdint>

extern const uint16_t CRC16_INIT;
// CRC-16/MCRF4XX
extern const uint16_t W_CRC_TABLE[256];
/**
 * @brief CRC16 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
 * @return : CRC16 checksum
 */
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */
bool Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength);

/**
 * @brief Append CRC16 value to the end of the buffer
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return none
 */
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif