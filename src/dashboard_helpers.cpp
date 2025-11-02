/**
 * @file dashboard_helpers.cpp
 * @brief Provides helper utilities for updating Nextion dashboard widgets.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#include "dashboard_helpers.h"


void setText(NexText &obj, const char *value) {
    obj.setText(value);
}

void setText(NexText &obj, float value, int decimals) {
    char buf[16];
    dtostrf(value, 0, decimals, buf);
    obj.setText(buf);
}

void setBar(NexProgressBar &obj, int value) {
    value = constrain(value, 0, 100);
    obj.setValue(value);
}
