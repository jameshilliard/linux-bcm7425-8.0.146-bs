#ifndef __ASM_BRIGHTSIGN_BOARDID_H
#define __ASM_BRIGHTSIGN_BOARDID_H

extern unsigned long brightsign_board_id;
extern unsigned long brightsign_secure_keys_flag;
extern unsigned long brightsign_secure_boot_flag;

/* BrightSign bug #17395 may cause the board ID to be read incorrectly
 * on Cheetah. REVC or REVD boards may appear to be REVA. We should
 * make sure that the firmware copes with that.
 */
#define CHEETAH_BOARDID_REVA	0
/* rev B never built */
#define CHEETAH_BOARDID_REVC	4
#define CHEETAH_BOARDID_REVD	8
/* rev E, F, G all the same as REVD */

#define PANTHER_BOARDID_REVA    0
/* rev B, C the same as REVA */
#define PANTHER_BOARDID_REVD    4
/* rev E the same as REVD */

#define PUMA_WITH_USB_AUDIO_BOARDID_REVA	10
#define PUMA_WITHOUT_USB_AUDIO_BOARDID_REVA	6

#define PUMA_WITH_USB_AUDIO_BOARDID_REVC	11
#define PUMA_WITHOUT_USB_AUDIO_BOARDID_REVC	15

#define PUMA_WITH_USB_AUDIO_BOARDID_REVD	13

/* BrightSign bug #17395 may cause the board ID to be read incorrectly
 * on some HD970 boards. We need to ensure that a board ID of 12 is
 * treated as being equivalent to 13. */
#define PUMA_WITH_USB_AUDIO_BOARDID_REVD_ALTERNATE 12

#endif /* __ASM_BRIGHTSIGN_BOARDID_H */
