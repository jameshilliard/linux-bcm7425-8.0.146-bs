/*
 * Copyright Â© Free Electrons, 2011
 * Copyright Â© International Business Machines Corp., 2006
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Author: David Wagner
 */

#ifndef __UBIBLK_USER_H__
#define __UBIBLK_USER_H__

#include <linux/types.h>

/**
 * ubiblk_ctrl_req - additional ioctl data structure
 * @ubi_num: UBI device number
 * @vol_id: UBI volume identifier
 * @padding: reserved for future, must contain zeroes
 */
struct ubiblk_ctrl_req {
	__s32 ubi_num;
	__s32 vol_id;
	__u8 padding[8];
} __packed;

/* ioctl commands of the UBI control character device */
#define UBIBLK_CTRL_IOC_MAGIC 'O'

/* Create a ubiblk device from a UBI volume */
#define UBIBLK_IOCADD _IOW(UBIBLK_CTRL_IOC_MAGIC, 0x10, struct ubiblk_ctrl_req)
/* Delete a ubiblk device */
#define UBIBLK_IOCDEL _IOW(UBIBLK_CTRL_IOC_MAGIC, 0x11, struct ubiblk_ctrl_req)
/* If you add ioctls here, please note that UBI uses 'O'/0x00-0x06 */

#endif
