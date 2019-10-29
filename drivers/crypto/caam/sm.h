
/*
 * CAAM Secure Memory/Keywrap API Definitions
 * Copyright (C) 2008-2015 Freescale Semiconductor, Inc.
 */

#ifndef SM_H
#define SM_H


/* Storage access permissions */
#define SM_PERM_READ 0x01
#define SM_PERM_WRITE 0x02
#define SM_PERM_BLOB 0x03

/* Define treatment of secure memory vs. general memory blobs */
#define SM_SECMEM 0
#define SM_GENMEM 1

/* Define treatment of red/black keys */
#define RED_KEY 0
#define BLACK_KEY 1

/* Define key encryption/covering options */
#define KEY_COVER_ECB 0	/* cover key in AES-ECB */
#define KEY_COVER_CCM 1 /* cover key with AES-CCM */

/*
 * Round a key size up to an AES blocksize boundary so to allow for
 * padding out to a full block
 */
#define AES_BLOCK_PAD(x) ((x % 16) ? ((x >> 4) + 1) << 4 : x)

/* Define space required for BKEK + MAC tag storage in any blob */
#define BLOB_OVERHEAD (32 + 16)

/* Keystore maintenance functions */
u32 sm_detect_keystore_units(struct device *dev);
void caam_sm_shutdown(struct platform_device *pdev);
int caam_sm_example_init(struct platform_device *pdev);

/* Keystore accessor functions */
extern int sm_keystore_slot_alloc(struct device *dev, u32 unit, u32 size,
				  u32 *slot);
extern int sm_keystore_slot_dealloc(struct device *dev, u32 unit, u32 slot);
extern int sm_keystore_slot_load(struct device *dev, u32 unit, u32 slot,
				 const u8 *key_data, u32 key_length);
extern int sm_keystore_slot_read(struct device *dev, u32 unit, u32 slot,
				 u32 key_length, u8 *key_data);
extern int sm_keystore_slot_export(struct device *dev, u32 unit, u32 slot,
				   u8 keycolor, u8 keyauth, u8 *outbuf,
				   u16 keylen, u8 *keymod);
extern int sm_keystore_slot_import(struct device *dev, u32 unit, u32 slot,
				   u8 keycolor, u8 keyauth, u8 *inbuf,
				   u16 keylen, u8 *keymod);

/* Prior functions from legacy API, deprecated */
extern int sm_keystore_slot_encapsulate(struct device *dev, u32 unit,
					u32 inslot, u32 outslot, u16 secretlen,
					u8 *keymod, u16 keymodlen);
extern int sm_keystore_slot_decapsulate(struct device *dev, u32 unit,
					u32 inslot, u32 outslot, u16 secretlen,
					u8 *keymod, u16 keymodlen);

struct sm_page_descriptor;

struct caam_drv_private_sm {
	struct device *parentdev;	/* this ends up as the controller */
	void *smringdev;	/* ring that owns this instance */
	struct platform_device *sm_pdev;  /* Secure Memory platform device */
	spinlock_t kslock ____cacheline_aligned;

	/* Default parameters for geometry */
	u32 page_size;		/* page size */
	u32 slot_size;		/* selected size of each storage block */

	/* Partition/Page Allocation Map */
	u32 localpages;		/* Number of pages we can access */
	struct sm_page_descriptor *pagedesc;	/* Allocated per-page */
};

#endif /* SM_H */
