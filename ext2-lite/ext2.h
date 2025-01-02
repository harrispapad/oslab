/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ext2.h 
 *
 * This file contains the routines related to directory files.
 *
 * Dimitris Siakavaras <jimsiak@cslab.ece.ntua.gr>
 *
 */

#include <linux/fs.h>
#include <linux/blockgroup_lock.h>

/* EXT2 file system version */
#define EXT2FS_DATE    "November 2023"
#define EXT2FS_VERSION "1.0-lite"

/* The EXT2 magic number, taken from include/uapi/linux/magic.h */
#define EXT2_SUPER_MAGIC 0xEF53

/* Maximal count of links to a file */
#define EXT2_LINK_MAX 32000

/* data type for block offset of block group */
typedef int ext2_grpblk_t;

/* data type for filesystem-wide blocks number */
typedef unsigned long ext2_fsblk_t;

/* EXT2 super-block data in memory */
struct ext2_sb_info {
	unsigned long s_inodes_per_block;  /* Number of inodes per block */
	unsigned long s_blocks_per_group;  /* Number of blocks in a group */
	unsigned long s_inodes_per_group;  /* Number of inodes in a group */
	unsigned long s_itb_per_group;     /* Number of inode table blocks per group */
	unsigned long s_gdb_count;         /* Number of group descriptor blocks */
	unsigned long s_desc_per_block;    /* Number of group descriptors per block */
	unsigned long s_groups_count;      /* Number of groups in the fs */
	unsigned long s_overhead_last;     /* Last calculated overhead */
	unsigned long s_blocks_last;       /* Last seen block count */
	struct buffer_head *s_sbh;         /* Buffer containing the super block */
	struct ext2_super_block *s_es;     /* Pointer to the super block in the buffer */
	struct buffer_head **s_group_desc; /* Array of buffers storing group descriptors */
	unsigned long s_mount_opt;
	unsigned long s_sb_block;
	unsigned short s_mount_state;
	unsigned short s_pad;
	int s_addr_per_block_bits;
	int s_desc_per_block_bits;
	int s_inode_size;
	int s_first_ino;
	struct percpu_counter s_freeblocks_counter;
	struct percpu_counter s_freeinodes_counter;
	struct percpu_counter s_dirs_counter;
	struct blockgroup_lock *s_blockgroup_lock;
	/*
	 * s_lock protects against concurrent modifications of s_mount_state,
	 * s_blocks_last, s_overhead_last and the content of superblock's
	 * buffer pointed to by sbi->s_es.
	 *
	 * Note: It is used in ext2_show_options() to provide a consistent view
	 * of the mount options.
	 */
	spinlock_t s_lock;
};

static inline spinlock_t *sb_bgl_lock(struct ext2_sb_info *sbi, unsigned int block_group)
{
	return bgl_lock_ptr(sbi->s_blockgroup_lock, block_group);
}

/*
 * Debug code: define EXT2FS_DEBUG to produce debug messages
 */
#define EXT2FS_DEBUG
#ifdef EXT2FS_DEBUG
#	define ext2_debug(f, a...) printk ("EXT2-fs-lite DEBUG: %s: " f, __func__, ## a)
#else
#	define ext2_debug(f, a...) /**/
#endif

/*
 * XXX More info about the __printf MACRO:
 * https://elixir.bootlin.com/linux/v5.10/source/include/linux/compiler_attributes.h#L151
 */
extern __printf(3, 4)
void ext2_msg(struct super_block *, const char *, const char *, ...);
extern __printf(3, 4)
void ext2_error(struct super_block *, const char *, const char *, ...);

/* Special inode numbers */
#define EXT2_ROOT_INO            2 /* Root inode */
#define EXT2_GOOD_OLD_FIRST_INO 11 /* First non-reserved inode */

static inline struct ext2_sb_info *EXT2_SB(struct super_block *sb)
{
	return sb->s_fs_info;
}

/*
 * Macro-instructions used to manage several block sizes
 */
#define EXT2_MIN_BLOCK_SIZE         1024
#define EXT2_MAX_BLOCK_SIZE         4096
#define EXT2_MIN_BLOCK_LOG_SIZE     10
#define EXT2_BLOCK_SIZE(s)          ((s)->s_blocksize)
#define EXT2_ADDR_PER_BLOCK(s)      (EXT2_BLOCK_SIZE(s) / sizeof (__u32))
#define EXT2_BLOCK_SIZE_BITS(s)     ((s)->s_blocksize_bits)
#define EXT2_ADDR_PER_BLOCK_BITS(s) (EXT2_SB(s)->s_addr_per_block_bits)
#define EXT2_INODE_SIZE(s)          (EXT2_SB(s)->s_inode_size)
#define EXT2_FIRST_INO(s)           (EXT2_SB(s)->s_first_ino)

/* Structure of a blockgroup descriptor */
struct ext2_group_desc {
	__le32 bg_block_bitmap;      /* Blocks bitmap block     */
	__le32 bg_inode_bitmap;      /* Inodes bitmap block     */
	__le32 bg_inode_table;       /* Inodes table block      */
	__le16 bg_free_blocks_count; /* Free blocks count       */
	__le16 bg_free_inodes_count; /* Free inodes count       */
	__le16 bg_used_dirs_count;   /* Directories count       */
	__le16 bg_pad;               /* Padding                 */
	__le32 bg_reserved[3];       /* Reserved for future use */
};

/* Macro-instructions used to manage group descriptors */
#define EXT2_BLOCKS_PER_GROUP(s)    (EXT2_SB(s)->s_blocks_per_group)
#define EXT2_DESC_PER_BLOCK(s)      (EXT2_SB(s)->s_desc_per_block)
#define EXT2_INODES_PER_GROUP(s)    (EXT2_SB(s)->s_inodes_per_group)
#define EXT2_DESC_PER_BLOCK_BITS(s) (EXT2_SB(s)->s_desc_per_block_bits)

/* Constants relative to the data blocks */
#define EXT2_NDIR_BLOCKS 12
#define EXT2_IND_BLOCK   EXT2_NDIR_BLOCKS
#define EXT2_DIND_BLOCK  (EXT2_IND_BLOCK + 1)
#define EXT2_TIND_BLOCK  (EXT2_DIND_BLOCK + 1)
#define EXT2_N_BLOCKS    (EXT2_TIND_BLOCK + 1)

/* EXT2 inode on the disk */
struct ext2_inode {
	__le16 i_mode;                 /* File mode */
	__le16 i_uid;                  /* Low 16 bits of Owner Uid */
	__le32 i_size;                 /* Size in bytes */
	__le32 i_atime;                /* Access time */
	__le32 i_ctime;                /* Creation time */
	__le32 i_mtime;                /* Modification time */
	__le32 i_dtime;                /* Deletion Time */
	__le16 i_gid;                  /* Low 16 bits of Group Id */
	__le16 i_links_count;          /* Links count */
	__le32 i_blocks;               /* Number of 512B blocks */
	__le32 i_flags;                /* File flags */
	__le32 UNUSED_OSD1;            /* XXX: was osd1 */
	__le32 i_block[EXT2_N_BLOCKS]; /* Pointers to blocks */
	__le32 UNUSED_GENERATION;      /* XXX: was i_generation */
	__le32 UNUSED_FADDR;           /* XXX: was i_faddr */
	__u8   UNUSED_OSD2[12];        /* XXX: was osd2 union */
};

/* File system states */
#define EXT2_VALID_FS 0x0001 /* Unmounted cleanly */
#define EXT2_ERROR_FS 0x0002 /* Errors detected */

/* Mount flags */
#define EXT2_MOUNT_DEBUG        0x000008 /* Some debugging messages */
#define EXT2_MOUNT_ERRORS_CONT  0x000010 /* Continue on errors */
#define EXT2_MOUNT_ERRORS_RO    0x000020 /* Remount fs ro on errors */
#define EXT2_MOUNT_ERRORS_PANIC 0x000040 /* Panic on errors */

#define clear_opt(o, opt) o &= ~EXT2_MOUNT_##opt
#define set_opt(o, opt)   o |= EXT2_MOUNT_##opt
#define test_opt(sb, opt) (EXT2_SB(sb)->s_mount_opt & EXT2_MOUNT_##opt)

/* Behaviour when detecting errors */
#define EXT2_ERRORS_CONTINUE   1 /* Continue execution */
#define EXT2_ERRORS_RO         2 /* Remount fs read-only */
#define EXT2_ERRORS_PANIC      3 /* Panic */
#define EXT2_ERRORS_DEFAULT    EXT2_ERRORS_CONTINUE

/* EXT2 super block on disk */
struct ext2_super_block {
	__le32 s_inodes_count;       /* Inodes count */
	__le32 s_blocks_count;       /* Blocks count */
	__le32 s_r_blocks_count;     /* Reserved blocks count */
	__le32 s_free_blocks_count;  /* Free blocks count */
	__le32 s_free_inodes_count;  /* Free inodes count */
	__le32 s_first_data_block;   /* First Data Block */
	__le32 s_log_block_size;     /* Block size */
	__le32 UNUSED_FRAG_SIZE;     /* XXX: was s_log_frag_size */
	__le32 s_blocks_per_group;   /* # Blocks per group */
	__le32 UNUSED_FPG;           /* XXX: was s_frags_per_group */
	__le32 s_inodes_per_group;   /* # Inodes per group */
	__le32 s_mtime;              /* Mount time */
	__le32 s_wtime;              /* Write time */
	__le16 s_mnt_count;          /* Mount count */
	__le16 UNUSED_MMCNT;         /* XXX was s_max_mnt_count */
	__le16 s_magic;              /* Magic signature */
	__le16 s_state;              /* File system state */
	__le16 s_errors;             /* Behaviour when detecting errors */
	__le16 s_minor_rev_level;    /* minor revision level */
	__le32 s_lastcheck;          /* time of last check */
	__le32 s_checkinterval;      /* max. time between checks */
	__le32 s_creator_os;         /* OS where fs was created  */
	__le32 s_rev_level;          /* Revision level */
	__le16 s_def_resuid;         /* Default uid for reserved blocks */
	__le16 s_def_resgid;         /* Default gid for reserved blocks */
	__le32 s_first_ino;          /* First non-reserved inode */
	__le16 s_inode_size;         /* size of inode structure */
	__le16 s_block_group_nr;     /* block group # of this superblock */
	__le32 s_feature_compat;     /* compatible feature set */
	__le32 s_feature_incompat;   /* incompatible feature set */
	__le32 s_feature_ro_compat;  /* readonly-compatible feature set */
	__u8   s_uuid[16];           /* 128-bit uuid for volume */
	char   s_volume_name[16];    /* volume name */
	char   s_last_mounted[64];   /* directory where last mounted */
	__le32 UNUSED_COMPRESSION;   /* XXX: was fields for compression */
	__u16  UNUSED_PREALLOC;      /* XXX: no preallocation*/
	__u16  UNUSED_padding1;      /* XXX: was s_padding1 */
	__u8   UNUSED_JOURNAL[21];   /* XXX: was fields for journal support */
	__le32 s_default_mount_opts; /* default mount options */
	__le32 s_first_meta_bg;      /* First metablock block group */
	__u32  s_reserved[190];      /* Padding to the end of the block */
};

/* Revision levels */
#define EXT2_GOOD_OLD_REV 0 /* The good old (original) format */
#define EXT2_DYNAMIC_REV  1 /* V2 format w/ dynamic inode sizes */

#define EXT2_MAX_SUPP_REV EXT2_DYNAMIC_REV

#define EXT2_GOOD_OLD_INODE_SIZE 128

/* The maximum length (in bytes) of a filename in dir_entry */
#define EXT2_NAME_LEN 255

/*
 * The new version of the directory entry.  Since EXT2 structures are
 * stored in intel byte order, and the name_len field could never be
 * bigger than 255 chars, it's safe to reclaim the extra byte for the
 * file_type field.
 *
 * XXX: In literature the dir_entry typically does not contain the
 * file_type field. But this implementation adds this here.
 */
typedef struct {
	__le32 inode;     /* Inode number */
	__le16 rec_len;   /* Directory entry length */
	__u8   name_len;  /* Name length */
	__u8   file_type;
	char   name[];    /* File name, up to EXT2_NAME_LEN */
} ext2_dirent;

/*
 * EXT2_DIR_PAD defines the directory entries boundaries
 * NOTE: It must be a multiple of 4
 */
#define EXT2_DIR_PAD               4
#define EXT2_DIR_ROUND             (EXT2_DIR_PAD - 1)
#define EXT2_DIR_REC_LEN(name_len) (((name_len) + 8 + EXT2_DIR_ROUND) & ~EXT2_DIR_ROUND)
#define EXT2_MAX_REC_LEN           ((1<<16)-1)

/* EXT2 inode data in memory */
struct ext2_inode_info {
	__le32 i_data[15];
	__u32  i_flags;
	__u16  i_state;
	__u32  i_dtime;

	/*
	 * i_block_group is the number of the block group which contains
	 * this file's inode.  Constant across the lifetime of the inode,
	 * it is used for making block allocation decisions - we try to
	 * place a file's data blocks near its inode block, and new inodes
	 * near to their parent directory's inode.
	 */
	__u32 i_block_group;

	struct inode vfs_inode; //> The VFS inode structure.
};

/* Inode dynamic state flags */
#define EXT2_STATE_NEW 0x00000001 /* inode is newly created */

/*
 * Function prototypes
 */
static inline struct ext2_inode_info *EXT2_I(struct inode *inode)
{
	return container_of(inode, struct ext2_inode_info, vfs_inode);
}

/* balloc.c */
extern struct ext2_group_desc *ext2_get_group_desc(struct super_block * b,
                                                   unsigned int block_group,
                                                   struct buffer_head **bh);
extern void ext2_free_blocks(struct inode *, unsigned long, unsigned long);
extern ext2_fsblk_t ext2_new_blocks(struct inode *, unsigned long *, int *);
extern unsigned long ext2_count_free_blocks(struct super_block *);
extern int ext2_bg_has_super(struct super_block *sb, int group);
extern unsigned long ext2_bg_num_gdb(struct super_block *sb, int group);

/* dir.c */
extern int ext2_add_link(struct dentry *, struct inode *);
extern int ext2_inode_by_name(struct inode *dir, const struct qstr *child, ino_t *ino);
extern int ext2_make_empty(struct inode *, struct inode *);
extern ext2_dirent *ext2_find_entry(struct inode *, const struct qstr *, struct folio **);
extern int ext2_delete_entry(ext2_dirent *, struct folio *);
extern int ext2_empty_dir(struct inode *);
extern ext2_dirent *ext2_dotdot(struct inode *, struct folio **);
extern int ext2_set_link(struct inode *, ext2_dirent *, struct folio *, struct inode *, bool);

/* ialloc.c */
extern struct inode *ext2_new_inode(struct inode *, umode_t);
extern void ext2_free_inode(struct inode *);
extern unsigned long ext2_count_free_inodes(struct super_block *);
extern unsigned long ext2_count_dirs (struct super_block *);

/* inode.c */
extern struct inode *ext2_iget(struct super_block *, unsigned long);
extern int ext2_write_inode(struct inode *, struct writeback_control *);
extern int ext2_get_block(struct inode *, sector_t, struct buffer_head *, int);
extern void ext2_evict_inode(struct inode *);
extern int ext2_setattr(struct mnt_idmap *, struct dentry *, struct iattr *);
extern int ext2_getattr(struct mnt_idmap *, const struct path *, struct kstat *,
                        u32, unsigned int);
extern void ext2_set_inode_flags(struct inode *inode);

/*
 * Inode, file and address_space operations structs.
 */
extern const struct file_operations          ext2_dir_operations;        // dir.c
extern const struct inode_operations         ext2_file_inode_operations; // file.c
extern const struct file_operations          ext2_file_operations;
extern const struct address_space_operations ext2_aops;                  // inode.c
extern const struct inode_operations         ext2_dir_inode_operations;  // namei.c
extern const struct inode_operations         ext2_special_inode_operations;

/* Get the first block number in the given group. */
static inline ext2_fsblk_t ext2_group_first_block_no(struct super_block *sb,
                                                     unsigned long group_no)
{
	return group_no * (ext2_fsblk_t)EXT2_BLOCKS_PER_GROUP(sb) +
	       le32_to_cpu(EXT2_SB(sb)->s_es->s_first_data_block);
}

/* Get the last block number in the given group. */
static inline ext2_fsblk_t ext2_group_last_block_no(struct super_block *sb,
                                                    unsigned long group_no)
{
	struct ext2_sb_info *sbi = EXT2_SB(sb);

	if (group_no == sbi->s_groups_count - 1)
		return le32_to_cpu(sbi->s_es->s_blocks_count) - 1;
	else
		return ext2_group_first_block_no(sb, group_no) +
		       EXT2_BLOCKS_PER_GROUP(sb) - 1;
}
