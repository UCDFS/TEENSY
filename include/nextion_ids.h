/**
 * @file nextion_ids.h
 * @brief Defines Nextion component identifiers used by the dashboard module.
 * @author Shane Whelan (UCD Formula Student)
 * @date 2025/2026
 */

#ifndef NEXTION_IDS_H
#define NEXTION_IDS_H

// ---------- Page 2 – Driving Info ----------
#define PAGE_TELEMETRY        2
#define KMH_TXT_NAME          "kmh_txt"
#define RPM_TXT_NAME          "rpm_txt"
#define RTDSTATUS_TXT_NAME    "rtdstatus_txt"

// ---------- Page 4 – Sensor Diagnostics ----------
#define PAGE_SENSORS          4
#define APPS1_TXT_NAME        "apps1_txt"
#define APPS2_TXT_NAME        "apps2_txt"
#define BRAKE_FRONT_TXT_NAME  "brakefront_txt"
#define BRAKE_REAR_TXT_NAME   "brakerear_txt"

// ---------- Page 5 – Accelerator ----------
#define PAGE_ACCEL            5
#define ACCEL_BAR_NAME        "accel_bar"
#define BRAKE_LIGHT           "brake_light" // is type bar, but will only be set to 0 or 100

#endif // NEXTION_IDS_H
