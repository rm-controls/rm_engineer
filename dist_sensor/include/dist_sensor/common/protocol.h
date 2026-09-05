//
// Created by ch on 25-6-4.
//

#pragma once
#define __packed __attribute__((packed))

#include <cstdint>
namespace dist_sensor
{
typedef struct
{
  uint8_t dist[2];
  uint8_t amp[2];
  uint8_t temp[2];
} __packed DistanceSensorData;
}  // namespace dist_sensor