#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/brightsign/boardid.h>

unsigned long brightsign_board_id;
unsigned long brightsign_secure_keys_flag;
unsigned long brightsign_secure_boot_flag;
static const char *family;
static const char *bs_cfe_version;
static const char *bs_secure_id;

#ifdef __mips__
static const char *bootloader = "CFE";
#else
static const char *bootloader = "BOLT";
#endif

static int boardid_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Family: %s\n%s-version: %s\nID: %lu\nSecure-keys: %lu\nSecure-boot: %lu\nSecure-ID: %s\n",
		   family, bootloader, bs_cfe_version, brightsign_board_id, brightsign_secure_keys_flag, brightsign_secure_boot_flag,
		   bs_secure_id);
	return 0;
}

static int boardid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, boardid_proc_show, NULL);
}

static const struct file_operations boardid_proc_fops = {
	.owner = THIS_MODULE,
	.open = boardid_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static int __init boardid_init(void)
{
#ifdef __mips__
#if defined(CONFIG_BRIGHTSIGN_PANTHER)
	family = "panther";
#elif defined(CONFIG_BRIGHTSIGN_PUMA)
	family = "puma";
#elif defined(CONFIG_BRIGHTSIGN_CHEETAH)
	family = "cheetah";
#else
#error Unknown family
#endif
#endif

#ifdef CONFIG_OF
	const struct device_node *bolt = of_find_node_by_path ("/bolt");
	if (bolt) {
		int len;
		const void *p;
		family = of_get_property (bolt, "board", NULL);
		bs_cfe_version = of_get_property (bolt, "brightsign-bolt-version", NULL);
		if (!bs_cfe_version)
			bs_cfe_version = "(unknown)";
		p = of_get_property (bolt, "brightsign-board-id", &len);
		if (p)
			brightsign_board_id = be32_to_cpup (p);
		p = of_get_property (bolt, "brightsign-secure-keys", &len);
		if (p)
			brightsign_secure_keys_flag = be32_to_cpup (p);
		p = of_get_property (bolt, "brightsign-secure-boot", &len);
		if (p)
			brightsign_secure_boot_flag = be32_to_cpup (p);
		bs_secure_id = of_get_property (bolt, "brightsign-secure-id", NULL);
	} else {
		printk ("warning, couldn't find BOLT note in dt\n");
	}
#endif

	if (bs_secure_id == NULL)
		bs_secure_id = "";

	if (!proc_create("boardid", 0, NULL, &boardid_proc_fops))
		pr_err("Failed to create /proc/boardid\n");
	return 0;
}

static void __exit boardid_exit(void)
{
	remove_proc_entry("boardid", NULL);
}

module_init(boardid_init);
module_exit(boardid_exit);
