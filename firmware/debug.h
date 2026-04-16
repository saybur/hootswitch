/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdbool.h>

#define DEBUG_MESSAGE_QUEUE_DEPTH  16
#define DEBUG_MESSAGE_LENGTH_MAX   64

void dbg(const char *format, ...);
void dbg_err(const char *format, ...);
void dbg_trace(const char *format, ...);

void dbg_trace_enable(bool state);
bool dbg_trace_is_enabled(void);

void dbg_init(void);
void dbg_task(__unused void *parameters);

#endif /* __DEBUG_H__ */
