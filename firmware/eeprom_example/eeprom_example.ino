void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

/** Read calibration data from EEPROM, if the CRC of the data is valid, go
 * straight to running state, otherwise go to calibration mode.
 */
void restore_calibration_from_eeprom()
{
    uint16_t stored_crc;
    EEPROM.get(EEPROM_CALIBRATION_ADDRESS, orientation);
    EEPROM.get(EEPROM_CALIBRATION_ADDRESS + sizeof(orientation), stored_crc);
    uint16_t crc = calculate_crc(
        reinterpret_cast<uint8_t*>(&orientation),
        sizeof(orientation));

    if (crc == stored_crc) {
        SET_STATE(STATE_RUNNING);
    } else {
        // reset the Z-Axis so it is acquired again
        vzero(orientation.zaxis);
        SET_STATE(STATE_ACQUIRE_DOWN_DIRECTION);
    }
}


/** Save the calibration values to EEPROM along with a CRC checksum.  Next
 * time the unit boots up it will reuse these values, avoiding the need for a
 * calibration.
 */
void save_calibration_to_eeprom()
{
    uint16_t crc = calculate_crc(
        reinterpret_cast<uint8_t*>(&orientation),
        sizeof(orientation));
    EEPROM.put(EEPROM_CALIBRATION_ADDRESS, orientation);
    EEPROM.put(EEPROM_CALIBRATION_ADDRESS + sizeof(orientation), crc);
}

// ............................................................. eeprom ....

const uint16_t crc_table[16] PROGMEM = {
  0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
  0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
};

uint16_t crc_table_ref(int index)
{
  // NOTE: crc_table + index is different from crc_table[index] since
  // crc_table is in PROGMEM!
  return pgm_read_word_near(crc_table + index);
}

/** Calculate a 16 bit CRC checksum of the 'data' buffer containing 'len'
   bytes.
*/
uint16_t calculate_crc(const uint8_t *data, int len)
{
  uint16_t crc = 0;
  while (len-- > 0) {
    uint8_t byte = *data++;
    uint16_t tmp = crc_table_ref(crc & 0x0F);
    crc = (crc >> 4) & 0x0FFF;
    crc = crc ^ tmp ^ crc_table_ref(byte & 0x0F);
    tmp = crc_table_ref(crc & 0x0F);
    crc = (crc >> 4) & 0x0FFF;
    crc = crc ^ tmp ^ crc_table_ref((byte >> 4) & 0x0F);
  }
  return crc;
}

/** Save the calibration values to EEPROM along with a CRC checksum.  Next
   time the unit boots up it will reuse these values, avoiding the need for a
   calibration.
*/
void save_calibration_to_eeprom()
{
  uint16_t crc = calculate_crc(
                   reinterpret_cast<uint8_t*>(&orientation),
                   sizeof(orientation));
  EEPROM.put(EEPROM_CALIBRATION_ADDRESS, orientation);
  EEPROM.put(EEPROM_CALIBRATION_ADDRESS + sizeof(orientation), crc);
}

/** Read calibration data from EEPROM, if the CRC of the data is valid, go
   straight to running state, otherwise go to calibration mode.
*/
void restore_calibration_from_eeprom()
{
  uint16_t stored_crc;
  EEPROM.get(EEPROM_CALIBRATION_ADDRESS, orientation);
  EEPROM.get(EEPROM_CALIBRATION_ADDRESS + sizeof(orientation), stored_crc);
  uint16_t crc = calculate_crc(
                   reinterpret_cast<uint8_t*>(&orientation),
                   sizeof(orientation));

  if (crc == stored_crc) {
    SET_STATE(STATE_RUNNING);
  } else {
    // reset the Z-Axis so it is acquired again
    vzero(orientation.zaxis);
    SET_STATE(STATE_ACQUIRE_DOWN_DIRECTION);
  }
}
