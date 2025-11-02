/**
 * @file apps.h
 * @brief Accelerator Pedal Position Sensor interface for plausibility checks and pedal mapping.
 * @author Asher Olgeirson (UCD Formula Student)
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#ifndef APPS_H
#define APPS_H

/* Accelerator Pedal Position Sensor                            \
  See T.11.8 and T.11.9 for more info                                          \
 Requirments:                                                                  \
 - Detect implausibility where there is a 10% difference or more between the   \
 two sensors \
 - Shut down main motors when implausibility exists for 100ms or more          \
 - Fail safe (shut down main motors) when short to ground or supply, open,     \
 mechanicaly impossible, or otherwise broken                                   \
 - Fail safe within 500ms of detecting a failure                               \
                                                                               \
 */

#include "core_pins.h"
#include "usb_serial.h"
#include "wiring.h"
#include <utility>

namespace APPS {

enum class AppsStatus { OK, Implausible, Fault };

struct AppsReading {
  AppsReading(AppsStatus status, double value, double APPS1, double APPS2) {
    this->status = status;
    this->value = value;
    this->APPS1 = APPS1;
    this->APPS2 = APPS2;
  }

  AppsStatus status;
  double value; // This could work as an std::optional<double>, but the risk of
                // bad optional access exceptions outweighs the advantage
  double APPS1;
  double APPS2;
};

//@breif Returns pedal position (%) or -1.0 on implausibility
//@return An AppsReading struct containing validity and pedal position
// Do not trust .value without first checking that .status is OK
AppsReading get_apps_reading();

//@breif T.11.8 compliance. Accepts a pair of APPS1 and APPS2 in order
bool check_plausiblity(std::pair<float, float> percentages);

//@breif T.11.9 compliance. Accepts a pair of APPS1 and APPS2 in order
bool check_integrity(std::pair<float, float> raws);

//@breif reads raw values without adjusting or normalizing
std::pair<float, float> get_raw_values();

//@breif converts raw values to percentages. First position in input pair
//becomes first position in return pair and vice versa
std::pair<float, float> get_percentage_values(std::pair<float, float> raws);

constexpr int DEBUG_MODE = 2; // 0: Off, 1: implausibility only, 2: all

// TODO: Get real measurements
extern const double PEDAL_MIN;
// TODO: Get real measurements
extern const double PEDAL_MAX;
constexpr int APPS_1_PIN = A6;
constexpr int APPS_2_PIN = A7;

// How many MS since an implausibility was detecting
inline int implausibility_start = 0;

// TODO: Get real measurements
// Define pedal calibration constants
// These are RAW ADC values (not voltages), since values DECREASE when
// pressed
constexpr int APPS1_RAW_MIN = 473; // Value when pedal is fully pressed (100%)
constexpr int APPS1_RAW_MAX = 750; // Value when pedal is released (0%)
constexpr int APPS2_RAW_MIN = 468; // Value when pedal is fully pressed (100%)
constexpr int APPS2_RAW_MAX = 750; // Value when pedal is released (0%)

// APPS
constexpr float APPS_PLAUSIBILITY_THRESHOLD = 10.0f; // T11.8.9

const unsigned long PLAUSIBILITY_TIMEOUT_MS =
    100; // Max time for APPS implausibility (Rule EV.5.6.3)
const unsigned long BRAKE_PLAUSIBILITY_TIMEOUT_MS =
    500; // Max time for APPS/Brake implausibility (Rule

} // namespace APPS
#endif // APPS_H
