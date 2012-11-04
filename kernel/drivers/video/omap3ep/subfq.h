/*
 * OMAP3EP FB: DSS and subframe queue management.
 *
 *      Copyright (C) 2009 Dimitar Dimitrov, Vladimir Krushkin, MM Solutions
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#ifndef SUBFQUEUE_H
#define SUBFQUEUE_H


struct fb_info;
struct omap3epfb_update_area;
struct omap3epfb_epd_par;
struct omap3epfb_statistics;

/* Allocate all buffers needed by the driver. */
extern int omap3epfb_alloc_buffers(struct fb_info *info);

extern void omap3epfb_free_buffers(struct fb_info *info);

extern size_t omap3epfb_videomem_size(struct fb_info *info);
extern size_t omap3epfb_subfmem_size(struct fb_info *info);
extern size_t omap3epfb_bfb_size(struct fb_info *info);
/* Update the whole screen.
 *
 * WARNING: The process takes up to two seconds!
 */
extern int omap3epfb_update_screen(struct fb_info *info, int wvfid,
							bool retry_req);


/* Issue asynchronous update request. The main difference from
 * omap3epfb_update_screen() is that this can be called in a burst and
 * only the last call will initiate a screen update. This is very similar
 * to the deferred I/O but instead of MMU we use page flipping as timout
 * start event.
 */
extern int omap3epfb_update_screen_bursty(struct fb_info *info);

/**
 * Update a partial area of the screen.
 *
 * Return zero if request was successfully queued, or negative on error.
 * Failures might be caused by:
 * 	- another window that is being updated at the moment overlaps with the
 * 	  requested one
 * 	- too many windows have been queued
 */
extern int omap3epfb_update_area(struct fb_info *info,
					struct omap3epfb_update_area *area);

/* Check whether the screen is doing an update or not. */
extern bool omap3epfb_screen_is_flipping(struct fb_info *info);

extern void omap3epfb_get_epd_fixpar(struct fb_info *info,
				struct omap3epfb_epd_fixpar *par);

extern void omap3epfb_get_epd_varpar(struct fb_info *info,
				struct omap3epfb_epd_varpar *par);

extern int omap3epfb_set_epd_varpar(struct fb_info *info,
				const struct omap3epfb_epd_varpar *par);

extern int omap3epfb_get_statistics(struct fb_info *info,
				struct omap3epfb_statistics *stats);

extern int omap3epfb_reset_statistics(struct fb_info *info);

/* Initialize the subframe queue internal state. */
extern void omap3epfb_init_subfq_state(struct fb_info *info);


#if defined(CONFIG_FB_OMAP3EP_DRIVE_EPAPER_PANEL_DSS)
extern int subf_producer_get(struct omap3epfb_par *par);

extern void subf_producer_queue(struct omap3epfb_par *par,int last);

extern int omap3epfb_run_screen_sequence(struct fb_info *info);
#elif defined(CONFIG_FB_OMAP3EP_DRIVE_EPAPER_PANEL_GPMC)
int omap3epfb_pmic_powerup(struct fb_info *info);
#endif
/* Initialize the RT workqueue that is responsible for subframe generation. */
extern int omap3epfb_create_screenupdate_workqueue(struct fb_info *info);

/* Cleanup the screen update workqueue. */
extern void omap3epfb_destroy_screenupdate_workqueue(struct fb_info *info);

/**/
int omap3epfb_bfb_init(struct fb_info *info);
/**/
extern int omap3epfb_bfb_update_area(struct fb_info *info,
					struct omap3epfb_bfb_update_area *area);

/**/
extern int omap3epfb_send_dsp_pm(struct fb_info *info,bool sleep,struct omap3epfb_bfb_update_area *area);

extern int omap3epfb_set_rotate(struct fb_info *info, int rotate);
#endif	/* SUBFQUEUE_H */

