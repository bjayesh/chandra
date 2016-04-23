/*
 * Copyright (C) 2015 Fuji Xerox Co.,Ltd. All rights reserved.
 *
 * Author:
 *   Atsushi Takeshita <atsushi.takeshita@fujixerox.co.jp>, Dec 2015
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */
#ifndef __LM2_PM_LOGGER__	
#define	__LM2_PM_LOGGER__

#define energyTraceKernel(msg) lm2_pm_stamp(__func__, __LINE__, msg)
void lm2_pm_stamp(const char *func, unsigned int line, char *msg);
void lm2_pm_stamp_sysfs_entry(void);

#endif
