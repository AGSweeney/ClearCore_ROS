/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __CCROS_SERIAL_MENU_H__
#define __CCROS_SERIAL_MENU_H__

#include <stdbool.h>

/*
 * Text setup menu on USB CDC for a person at a terminal. Type "menu" and Enter.
 * Every change goes through the same MotionConfigure / MotionConfigureNetwork
 * path as JSON-RPC, so validation and NVM writes are identical.
 *
 * Returns true when the line was consumed. A line starting with '{' closes the
 * menu and is left for SessionDispatch.
 */
bool SerialMenuHandleLine(const char *line);

#endif
