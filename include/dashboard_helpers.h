/**
 * @file dashboard_helpers.h
 * @brief Helper utilities for updating Nextion dashboard widgets.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#ifndef DASHBOARD_HELPERS_H
#define DASHBOARD_HELPERS_H

#include <Nextion.h>

void setText(NexText &obj, const char *value);
void setText(NexText &obj, float value, int decimals = 1);
void setBar(NexProgressBar &obj, int value);

#endif
