// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#if defined (CONFIG_INITRAMFS_IGNORE_SKIP_FLAG) \
	|| defined(CONFIG_CMDLINE_HWC_IS_SKU) \
	|| defined(CONFIG_CMDLINE_HWC_IS_PRODUCT_SKU)
#define ALTER_CMDLINE
#endif

#ifdef ALTER_CMDLINE
#include <asm/setup.h>

#ifdef CONFIG_INITRAMFS_IGNORE_SKIP_FLAG
#define INITRAMFS_STR_FIND "skip_initramf"
#define INITRAMFS_STR_REPLACE "want_initramf"
#define INITRAMFS_STR_LEN (sizeof(INITRAMFS_STR_FIND) - 1)
#endif

#if defined(CONFIG_CMDLINE_HWC_IS_SKU) || defined(CONFIG_CMDLINE_HWC_IS_PRODUCT_SKU)
#define HWC_STR_FIND "androidboot.hwc="
#define HWC_STR_LEN (sizeof(HWC_STR_FIND) - 1)
#endif

#ifdef CONFIG_CMDLINE_HWC_IS_SKU
#define SKU_STR_INSERT " androidboot.hardware.sku="
#endif

#ifdef CONFIG_CMDLINE_HWC_IS_PRODUCT_SKU
#define PRODUCT_SKU_STR_INSERT " androidboot.product.hardware.sku="
#endif

static char proc_command_line[COMMAND_LINE_SIZE];

static void proc_command_line_init(void) {
	strcpy(proc_command_line, saved_command_line);

#ifdef CONFIG_INITRAMFS_IGNORE_SKIP_FLAG
	char *offset_addr;
	offset_addr = strstr(proc_command_line, INITRAMFS_STR_FIND);
	if (offset_addr)
		memcpy(offset_addr, INITRAMFS_STR_REPLACE, INITRAMFS_STR_LEN);
#endif

#if defined(CONFIG_CMDLINE_HWC_IS_SKU) || defined(CONFIG_CMDLINE_HWC_IS_PRODUCT_SKU)
    char* hwc_offset_addr;
    char hwc_value[8] = "";

    hwc_offset_addr = strstr(proc_command_line, HWC_STR_FIND);
#endif

#ifdef CONFIG_CMDLINE_HWC_IS_SKU
    if (hwc_offset_addr) {
        memcpy(hwc_value, hwc_offset_addr + HWC_STR_LEN,
               strstr(hwc_offset_addr + HWC_STR_LEN, " ") - (hwc_offset_addr + HWC_STR_LEN));
        strcat(proc_command_line, SKU_STR_INSERT);
        strcat(proc_command_line, hwc_value);
    }
#endif

#ifdef CONFIG_CMDLINE_HWC_IS_PRODUCT_SKU
    if (hwc_offset_addr) {
        memcpy(hwc_value, hwc_offset_addr + HWC_STR_LEN,
               strstr(hwc_offset_addr + HWC_STR_LEN, " ") - (hwc_offset_addr + HWC_STR_LEN));
        strcat(proc_command_line, PRODUCT_SKU_STR_INSERT);
        strcat(proc_command_line, hwc_value);
    }
#endif
}
#endif

static int cmdline_proc_show(struct seq_file *m, void *v)
{
#ifdef ALTER_CMDLINE
	seq_printf(m, "%s\n", proc_command_line);
#else
	seq_printf(m, "%s\n", saved_command_line);
#endif
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_cmdline_init(void)
{
#ifdef ALTER_CMDLINE
	proc_command_line_init();
#endif

	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
	return 0;
}
fs_initcall(proc_cmdline_init);
