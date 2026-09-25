/*
 * The board's own lv_conf.h, with a bigger LVGL heap for the PC.
 *
 * On a 64-bit PC every LVGL object carries 8-byte pointers and takes roughly
 * half as much again as on the 32-bit ESP32, so the board's heap would run out
 * here long before it does on the device. The heap is made large enough never
 * to run out, and heapReport() (lv_budget.cpp) works out what the same objects
 * need on the device instead, from their 32-bit sizes.
 */
#pragma once
#include "../../include/lv_conf.h"
#undef  LV_MEM_SIZE
#define LV_MEM_SIZE (4U * 1024U * 1024U)
