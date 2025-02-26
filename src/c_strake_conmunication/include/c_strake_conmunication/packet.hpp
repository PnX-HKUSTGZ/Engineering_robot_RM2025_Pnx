// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

#pragma pack(push, 1)
struct target_location{
    uint8_t Header = 0xA5;
    uint8_t reserved : 8;
    float a[16];
    uint16_t crc16 = 0;
};
#pragma pack(pop)

// struct SendPacket
// {
//   uint8_t header = 0xA5;
//   uint8_t tracking : 1;
//   uint8_t iffire : 1;
//   uint8_t id : 3;          // 0-outpost 6-guard 7-base
//   uint8_t reserved : 3;
  
//   float pitch;
//   float yaw;
  
//   uint16_t checksum = 0;
// } __attribute__((packed));

inline target_location fromVector(const std::vector<uint8_t> & data)
{
    target_location packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
    return packet;
}

inline std::vector<uint8_t> toVector(const target_location & data)
{
  std::vector<uint8_t> packet(sizeof(target_location));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(target_location), packet.begin());
  return packet;
}


#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
