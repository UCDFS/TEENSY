#include "BrakePedal.h"

// Each channel is judged against its own rest count (BRAKE_FRONT_REST /
// BRAKE_REAR_REST), not a shared absolute count - front and rear rest
// baselines drift independently, so a shared threshold can sit below one
// channel's rest and false-latch permanently. Hysteresis gap between the
// ON/OFF deltas prevents flicker right at the threshold.
bool BrakePedal::read() {
  frontRaw = analogRead(BRAKE_FRONT_PIN);
  rearRaw = analogRead(BRAKE_REAR_PIN);

  int frontRise = frontRaw - BRAKE_FRONT_REST;
  int rearRise = rearRaw - BRAKE_REAR_REST;

  if (!pressed &&
      (frontRise > BRAKE_LIGHT_ON_DELTA || rearRise > BRAKE_LIGHT_ON_DELTA)) {
    pressed = true;
  } else if (pressed &&
             frontRise < BRAKE_LIGHT_OFF_DELTA && rearRise < BRAKE_LIGHT_OFF_DELTA) {
    pressed = false;
  }

  return pressed;
}
