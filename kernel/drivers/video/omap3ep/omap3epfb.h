/*
 * OMAP3EP FB declarations.
 *
 *      Copyright (C) 2009 Dimitar Dimitrov, Vladimir Krushkin, MM Solutions
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#ifndef OMAP3EPFB_H
#define OMAP3EPFB_H

#include <linux/types.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>


#include "omap3epfb-user.h"

struct fb_info;

/* If defined, the driver will gather various debug statistics. */
#define OMAP3EPFB_GATHER_STATISTICS


#if defined(CONFIG_FB_OMAP3EP_DRIVE_EPAPER_PANEL_GPMC)
#include "gpmc_config.h"
#endif

#include "pmic.h"

#if defined(CONFIG_FB_OMAP3EP_DRIVE_EPAPER_PANEL_DSS)
/* If defined, the driver will gather various debug statistics. */
#define OMAP3EPFB_SUBFI_IDLE		2
#define OMAP3EPFB_SUBFI_RESERVED	1
#endif


#define OMAP3EPFB_REQQ_DEPTH_SHIFT	5u
#define OMAP3EPFB_REQQ_DEPTH		(1u << OMAP3EPFB_REQQ_DEPTH_SHIFT)

#define OMAP3EPFB_BLITQ_DEPTH_SHIFT	4u
#define OMAP3EPFB_BLITQ_DEPTH		(1u << OMAP3EPFB_BLITQ_DEPTH_SHIFT)


typedef enum{
    DSP_NO_VCOM = 0,
    DSP_USE_VCOM
}DSP_VCOM_MODE;

typedef enum {
	DSP_WORK_CODE_NEWSFSEQ	= 1,
	DSP_WORK_CODE_CONTINUE	= 2,
	DSP_WORK_CODE_NOSFSEQ	= 3,
	//
	DSP_WORK_CODE_MASK	= 0xF,
	DSP_WORK_CODE_INCR	= 0x10
} DSP_WORK_CODE;

#if defined(OMAP3EPFB_GATHER_STATISTICS)
/*
 * the enum below denotes the required indexation used when gathering
 * update area video driver timings and latencies -
 * see stamp_mask and update_timings under omap3epfb_par.
 */
enum {
	UPD_APP_REQUEST,
	UPD_PRE_BLITTER,
	UPD_DSS_STARTING,
	UPD_PB_LATENCY,
	UPD_FULL_LATENCY,
	UPD_NUM_ITEMS
};
#endif

#define SUBFRAME_UNKN		0
#define SUBFRAME_FIRST		1
#define SUBFRAME_SECOND		2
#define SUBFRAME_THIRD		3
#define SUBFRAME_FOURTH		4
#define SUBFRAME_MIDDLE		0
#define SUBFRAME_LAST		0xFF

#define ARM_DSP_WORK_CODE_NEWSFSEQ	1
#define ARM_DSP_WORK_CODE_CONTINUE	2
#define ARM_DSP_WORK_CODE_NOSFSEQ	3
#define ARM_DSP_WORK_CODE_MASK		0x0F
#define ARM_DSP_WORK_CODE_INCR		0x10

struct omap3epfb_bfb_queue_element {
	unsigned int x0;        /* area's horizontal position */
	unsigned int y0;        /* area's vertical position */
	unsigned int x1;        /* area last horizontal pixel */
	unsigned int y1;        /* area last vertical pixel */
	// keep allways multiple of 16
	int bq_arm2dsp_req;
	int sleep;
	int dummy2;
	int dummy3;
};

/* Driver-specific parameters. */
struct omap3epfb_par {
	struct fb_info *info;		/* back-pointer */
	struct omap3epfb_buffer	vmem;	/* videomem */
	bool reg_daemon;		/* deamon registretion */
	bool loaded_daemon;		/* daemon fully loaded */
	bool dsp_pm_sleep;              /*DSPSN in is sleep mode*/
	/* For tracking register daemon and deferred io*/
	struct timer_list timer;
	
	/* blitter FB */
	struct {
		struct omap3epfb_buffer	mem;
		struct {
			/* */
			struct omap3epfb_buffer mem;
			/* typecasted pointer to mem.virt*/
			struct omap3epfb_bfb_queue_element *head;
			/* we need synchronization due to purging */
			spinlock_t xi_lock;
			/* sleep/awake synchronization */
			spinlock_t pm_lock;
		}que;
	}bfb;

	struct {
#if defined(CONFIG_FB_OMAP3EP_DRIVE_EPAPER_PANEL_DSS)
//		unsigned int rai;	/* read-acknowledge queue index *///moved to omap3epfb_rd_st
		unsigned int rpi;	/* read-prefetch queue index */
//		unsigned int wi;	/* write queue index *///moved to struct omap3epfb_wr_st
#endif
		struct omap3epfb_sharedbuf shrd;/* Producer (DSP) read from her*/
		struct omap3epfb_sharedbuf shwr;/* Producer (DSP)  write here*/
		struct omap3epfb_sharedbuf shstats;/* DSP writes stats here */
#if defined(CONFIG_FB_OMAP3EP_DRIVE_EPAPER_PANEL_DSS)
		struct omap3epfb_subfbuf bufs[OMAP3EPFB_SUBFQ_DEPTH];
		
		struct omap3epfb_subfbuf idlesubf;
//		bool last_subf_queued;	/* whether the last one is queued */
		bool first_irq_req;	/* helper for IRQ handler */
		bool sequence_finished;	/* helper for IRQ handler */
		
		/*
		 * Wait queue for free buffers.
		 * Completions are not suitable here due to possible race
		 * conditions.
		 */
//		wait_queue_head_t waitfree;

		/* used to signal for completed frame */
		wait_queue_head_t waitframe;
#endif
		/* screen sequence work queue */
		struct workqueue_struct *ss_workq;
		/* screen sequence work */
		struct work_struct ss_work;

		/* check DSP SN state work queue */
		struct workqueue_struct *dspck_workq;
		/* check DSP SN state work */
		struct work_struct dspck_work;
#if defined(CONFIG_FB_OMAP3EP_PGFLIP_DEFER)
		/* work used to emulate deferred I/O with page flip
		 * requests
		 */
		struct delayed_work bursty_work;
		struct workqueue_struct *bursty_workq;

		spinlock_t bursty_keepalive_lock;
		bool bursty_alive;
		bool bursty_keepalive;
#endif
	} subfq;

	struct {
		unsigned int ri;
		unsigned int wi;
		struct omap3epfb_update_area bufs[OMAP3EPFB_REQQ_DEPTH];

		/* we need synchronization due to purging */
		spinlock_t xi_lock;

		/* used to wakeup daemon for processing new request(s) */
		wait_queue_head_t waitreq;
		int cmd_in_queue;
	} reqq;

	struct {
		unsigned int vxres;
		unsigned int vyres;
		unsigned int pxres;
		unsigned int pyres;
		unsigned int bpp;
		unsigned int f_pixclk_khz;
	} mode;

#if defined(OMAP3EPFB_GATHER_STATISTICS)
	/* various statistics presentable to user-space */
	struct omap3epfb_statistics stats;

	/* update area timestamps */
	struct timespec update_timings[UPD_NUM_ITEMS];

	/* stamp_mask is an atomic bit field used for
	 * proper time stamping - bit field assignments
	 * are enumerated with UPD_*
	 */
	unsigned long stamp_mask;

	/* missed frame counter used by the IRQ */
	unsigned int num_missed_subframes;

	/* some of the statistics items consist of several variables so
	 * we need to provide an atomic snapshot of the stats structure.
	 */
	struct mutex stats_mutex;
#endif

	/* various EPD-related parameters */
	struct omap3epfb_epd_fixpar epd_fixpar;
	struct omap3epfb_epd_varpar epd_varpar;
	unsigned int last_yoffset;
    
	struct semaphore screen_update_mutex;

	struct pmic_sess *pwr_sess;
	enum omap3epfb_hwstate hwstate;
	/* Jiffies are not incremented during suspend, so use timeval. */
	struct timeval pmic_next_t_read_tv;

	uint32_t pseudo_palette[64];	/* dummy required by fb framework */

	struct {
		struct fb_ops fbops;
		struct fb_deferred_io defio;
	} fbvars;
	bool pgflip_refresh;
	int user_debug;
	int refresh_percent; /* % == 0, Normal update, % > 0 == GC Update */
	int disable_flags;
	struct omap3epfb_area effect_array[EFFECT_ARRAY_SIZE];
	struct mutex area_mutex;
	int effect_active;
	char effect_active_debug[EFFECT_ARRAY_SIZE+1];
	struct delayed_work clear_work;
	struct delayed_work disable_work;
	int clear_delay;
	u64 last_clear;

#if defined(CONFIG_FB_OMAP3EP_DRIVE_EPAPER_PANEL_GPMC)
	struct gpmc_sess *gpmc;
#endif
};

extern int omap3epfb_reqq_push_back_async(struct fb_info *info,
					const struct omap3epfb_update_area *a);
extern int omap3epfb_reqq_pop_front_sync(struct fb_info *info,
					struct omap3epfb_update_area *a);
extern int omap3epfb_reqq_pop_front_async(struct fb_info *info,
					struct omap3epfb_update_area *a);
extern int omap3epfb_reqq_purge(struct fb_info *info);

extern int omap3epfb_send_dsp_pm(struct fb_info *info,bool sleep,struct omap3epfb_bfb_update_area *area);

extern int omap3epfb_send_recovery_signal(void);

extern int omap3epfb_poll_temperature_safely(struct fb_info *info);

#endif	/* OMAP3EPFB_H */
