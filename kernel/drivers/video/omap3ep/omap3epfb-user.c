/*
 * OMAP3 Electronic Paper Framebuffer driver.
 *
 *
 *      Copyright (C) 2010 B&N
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>

#include "omap3epfb.h"
#include "subfq.h"
#include "pmic.h"
#if defined(CONFIG_FB_OMAP3EP_DRIVE_EPAPER_PANEL_DSS)
#include "omap3ep-dss.h"
#endif
#if defined(CONFIG_FB_OMAP3EP_DRIVE_EPAPER_PANEL_GPMC)
#include "gpmc_config.h"
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 28)
  #include <asm/gpio.h>
#else
  #include <linux/gpio.h>
#endif

#define TIME_STAMP() ktime_to_us(ktime_get())
#define TIME_DELTA_MS(x) ((u32)(ktime_to_us(ktime_get())-(x)) / 1000)

static inline u32 ktime_to_ms(void)
{
    ktime_t tm = ktime_get();

    return tm.tv.sec * 1000 + tm.tv.nsec/1000000;
}

// Time when EPD disable is set to expire
static u32 g_disable_time = 0;

#define FIRST_USER_REGION 4

#define FB_SYSFS_FLAG_ATTR 1

#define TAG "EPD: "

// Debug level
enum {
	DEBUG_LEVEL0,
	DEBUG_LEVEL1 = 1,
	DEBUG_LEVEL2 = 2,
	DEBUG_LEVEL3 = 4,
	DEBUG_LEVEL4 = 8,
	DEBUG_LEVEL5 = 16
};

// Effect Flags
enum {
	EFFECT_DISABLE = 0,
	EFFECT_ACTIVE  = 1,
	EFFECT_ONESHOT = 2,
	EFFECT_CLEAR   = 4,
	EFFECT_REGION  = 8
};

// Disable Flags
enum {
	EPD_ENABLE  = 0,
	EPD_DISABLE = 1
};

#define DEBUG_REGION(level,r,fmt, args...) \
	do { if (par->user_debug & level) { printk("EPD|%s| ",par->effect_active_debug); printk(fmt, ## args); rect_dump(r); } } while (0)

#define DEBUG_LOG(level,fmt, args...) \
	do { if (par->user_debug & level) { printk("EPD|%s| ",par->effect_active_debug); printk(fmt, ## args); } } while (0)

static void omap3epfb_clear_work(struct work_struct *work)
{
	struct omap3epfb_par *par = container_of(work,
		struct omap3epfb_par,
		clear_work.work);

	DEBUG_LOG(DEBUG_LEVEL2,"delayed FULLSCREEN Update\n");

	omap3epfb_reqq_purge(par->info);
	omap3epfb_update_screen(par->info, OMAP3EPFB_WVFID_GC, false);
}

static void omap3epfb_disable_work(struct work_struct *work)
{
	struct omap3epfb_par *par = container_of(work,
		struct omap3epfb_par,
		disable_work.work);

	g_disable_time = 0;  // Reset expiration time
	par->disable_flags = 0;

	DEBUG_LOG(DEBUG_LEVEL2,"delayed enable EPD Updates\n");

	omap3epfb_reqq_purge(par->info);
	omap3epfb_update_screen(par->info, OMAP3EPFB_WVFID_GC, false);
}


static inline int rect_width(struct omap3epfb_update_area *a)
{
	return a->x1-a->x0;
}

static inline int rect_height(struct omap3epfb_update_area *a)
{
	return a->y1-a->y0;
}


static inline bool rect_isempty(struct omap3epfb_update_area *a)
{
        return (rect_width(a)<=0) || (rect_height(a)<=0);
}

static void rect_dump(struct omap3epfb_update_area *a)
{
	static const char *wvfid_str[OMAP3EPFB_WVFID_NUM] = {
		"GC", "GU", "DU", "A2", "GL", "X1", "X2"
	};
	static const char *wvfm_vl_mode = "VU"; 
	static const char *wvfm_auto_mode = "AT"; 
	static const char *wvfm_undefined = "XX"; 
	char const *txt = wvfm_undefined;

	if ((a->wvfid >= 0) && (a->wvfid < OMAP3EPFB_WVFID_NUM))
		txt =  wvfid_str[a->wvfid];
	if (a->wvfid == OMAP3EPFB_WVFID_AUTO)
		txt = wvfm_auto_mode;
	if (a->wvfid == OMAP3EPFB_WVFID_VU)
		txt = wvfm_vl_mode;

        printk("%d,%d -> %d,%d = %s thres=%d\n", a->x0, a->y0, a->x1, a->y1, txt, a->threshold);
}

static bool rect_intersect(struct omap3epfb_update_area *a, struct omap3epfb_update_area *b, struct omap3epfb_update_area *result)
{
    result->x0 = max(a->x0, b->x0);
    result->y0 = max(a->y0, b->y0);
    result->x1 = min(a->x1, b->x1);
    result->y1 = min(a->y1, b->y1);
    return !(rect_isempty(result));
}

static bool rect_merge(struct omap3epfb_update_area *a, struct omap3epfb_update_area *b, struct omap3epfb_update_area *result)
{
    result->x0 = min(a->x0, b->x0);
    result->y0 = min(a->y0, b->y0);
    result->x1 = max(a->x1, b->x1);
    result->y1 = max(a->y1, b->y1);
    return !(rect_isempty(result));
}

static bool rect_inside(struct omap3epfb_update_area *a, struct omap3epfb_update_area *b)
{
    if (((b->x0 >= a->x0) && (b->x0 <= a->x1)) &&
        ((b->x1 >= a->x0) && (b->x1 <= a->x1)) &&
        ((b->y0 >= a->y0) && (b->y0 <= a->y1)) &&
        ((b->y1 >= a->y0) && (b->y1 <= a->y1)))
        return true;
    return false;
}

static bool rect_equal(struct omap3epfb_update_area *a, struct omap3epfb_update_area *b)
{
    if ((b->x0 == a->x0) && 
        (b->x1 == a->x1) &&
        (b->y0 == a->y0) &&
        (b->y1 == a->y1))
        return true;
    return false;
}

static bool rect_fullscreen(struct fb_info *info, struct omap3epfb_update_area *p)
{
	struct omap3epfb_update_area full = {0};
	full.x0 = 0;
	full.y0 = 0;
	full.x1 = info->var.xres-1;
	full.y1 = info->var.yres-1;
	full.wvfid = OMAP3EPFB_WVFID_GC;
	full.threshold = 0;

	return rect_equal(&full, p);
}

static bool batch_update(struct fb_info *info, struct omap3epfb_update_area *p)
{
	struct omap3epfb_par *par = info->par;

	// If EPD is disabled, do nothing
	if (par->disable_flags > 0)
	{
		DEBUG_REGION(DEBUG_LEVEL3, p,"update DISABLED = ");
		// Tell the caller not to update.
		return false;
	}

	// Check if the delayed full screen updates are enabled.
	if (!par->clear_delay)
	{
		DEBUG_REGION(DEBUG_LEVEL1, p,"   do = ");
		omap3epfb_update_area(info, p);
		return false;
	}

	// If this is not a fullscreen GC update, treat it as a normal update.
	if (!(rect_fullscreen(info, p) && p->wvfid == OMAP3EPFB_WVFID_GC))
	{
		// If we have a fullscreen batched, we do not need to update.
		if (!delayed_work_pending(&par->clear_work))
		{
			DEBUG_REGION(DEBUG_LEVEL1, p,"   do = ");
			omap3epfb_update_area(info, p);
		}
		else
		{
			DEBUG_REGION(DEBUG_LEVEL1, p,"   skip = ");
		}
		return false;
	}

	// We need to do fullscreen batching.
	if (par->user_debug & DEBUG_LEVEL1)
	{
		if (TIME_DELTA_MS(par->last_clear) < 1000)
			DEBUG_REGION(DEBUG_LEVEL1, p,"   req FULLSCREEN PREV %dms ago AUTO = ", TIME_DELTA_MS(par->last_clear));
		else
			DEBUG_REGION(DEBUG_LEVEL1, p,"   req FULLSCREEN = ");
	}
	omap3epfb_reqq_purge(par->info);
	cancel_delayed_work_sync(&par->clear_work);
	schedule_delayed_work(&par->clear_work, msecs_to_jiffies(par->clear_delay));
	par->last_clear = TIME_STAMP();
	return true;
}

static void update_effect(struct omap3epfb_par *par, int index)
{
	static const char wvfid_char[OMAP3EPFB_WVFID_NUM] = "CUDAL12";
	static const char wvfid_char_vu[1] = "V";
	struct omap3epfb_area *area = NULL;

	if ((index < 0) || (index >= EFFECT_ARRAY_SIZE))
		return;

	area = &par->effect_array[index];
	if (area->effect_flags)
	{
		if ((area->effect_area.wvfid >= 0) && (area->effect_area.wvfid < OMAP3EPFB_WVFID_NUM))
			par->effect_active_debug[index] = wvfid_char[area->effect_area.wvfid];
		else if (area->effect_area.wvfid == OMAP3EPFB_WVFID_VU)
			par->effect_active_debug[index] = wvfid_char_vu[0];
		else
			par->effect_active_debug[index] = '*';

		par->effect_active |= (1 << index);
	}
	else
	{
		par->effect_active &= ~(1 << index);
		par->effect_active_debug[index] = '-';
	}
}


// Fast line compare doing 4 bytes at a time
// Faster than the current memcmp() -- 4 times
inline int line_diff(void *p1, void *p2, int length)
{
	u32 *front = (u32*) p1;
	u32 *back =  (u32*) p2;
	int count = length / (sizeof(u32));

	while (count-- > 0)
	{
		if (*front++ != *back++)
			return 1;
	}
	return 0;
}


// Check if the difference between two buffers is greater than or equal to a given
// percent.  Returns TRUE if this condition is met; FALSE otherwise.
// NOTE: this algorithm is optimized for percent values higher than 50 because
//       in EPD displays it does not make sense to flash the screen in auto mode
//       if a small percent has changed.  For those cases, it is better to use the
//       update areas implemented in this driver.

static bool buffer_difference_ge_threshold(struct fb_info *info, int percent_threshold)
{
	struct omap3epfb_par *par = info->par;
	int y_res = par->mode.vyres;
	int line_length = info->fix.line_length;
	int buffer_size = line_length * y_res;
	int top, bottom;
	int line = 0;
	int max_y_equal = (y_res * (100 - percent_threshold)) / 100;
	int y_equal = 0;
	bool result = true;

	u8 *front = info->screen_base;
	u8 *back = info->screen_base + buffer_size;

	// Percent of 0 or negative always returns TRUE and the screen will flash
	// Same for percent greater than 100 since it's invalid
	if ((percent_threshold <= 0) || (percent_threshold > 100))
	{
		return true;
	}

	for (top = 0; top < y_res; top++)
	{
		if (line_diff(&front[line], &back[line], line_length))
		{
			// Found a difference; now go check from the bottom
			break;
		}
		// Check if the number of equal lines passed the threshold; if so,
		// we are done and the result is FALSE
		y_equal++;
		if (y_equal > max_y_equal)
		{
			result = false;
			break;
		}
		line += line_length;
	}
	bottom =  y_res-1;
	// Now start from the bottom, if we need to continue to check
	if (result)
	{
		line = (y_res-1) * line_length;
		for (bottom = y_res-1; bottom > top; bottom--)
		{
			if (line_diff(&front[line], &back[line], line_length))
			{
				// Found a difference, and we never went above the
				// threshold of equal lines.  Stop checking.
				// Result will be TRUE.
				break;
			}
			// Check if the number of equal lines passed the threshold;
			// if so, we are done and the result is FALSE
			y_equal++;
			if (y_equal > max_y_equal)
			{
				result = false;
				break;
			}
			line -= line_length;
		}
	}

	DEBUG_LOG(DEBUG_LEVEL5,"buffer difference: threshold = %d%%, top = %d, bottom = %d, max_y_equal = %d, y_equal = %d, result = %s\n", percent_threshold, top, bottom, max_y_equal, y_equal, result ? "TRUE" : "FALSE");
	return (result);
}


static int process_area(int index, struct fb_info *info, struct omap3epfb_area *area, struct omap3epfb_update_area *p)
{
	struct omap3epfb_par *par = info->par;
	int change = 0;

	if (!(area->effect_flags & (EFFECT_ONESHOT | EFFECT_ACTIVE | EFFECT_CLEAR)))
		return change;

	if (!rect_inside(&area->effect_area, p))
	{
		DEBUG_REGION(DEBUG_LEVEL5, p,"no match 0x%02x region %d = ", area->effect_flags, index);
		return change;
	}

	if (area->effect_flags & EFFECT_ONESHOT)
	{
		p->wvfid = area->effect_area.wvfid;
		p->threshold = area->effect_area.threshold;
		DEBUG_REGION(DEBUG_LEVEL2, p,"process ONESHOT region %d = ", index);
		if (area->effect_flags & EFFECT_REGION)
		{
			p->x0 = area->effect_area.x0;
			p->y0 = area->effect_area.y0;
			p->x1 = area->effect_area.x1;
			p->y1 = area->effect_area.y1;
		}
		par->effect_array[index].effect_flags = 0;
		change = 1;
	}
	else if (area->effect_flags & EFFECT_ACTIVE)
	{
		p->wvfid = area->effect_area.wvfid;
		p->threshold = area->effect_area.threshold;
		DEBUG_REGION(DEBUG_LEVEL2, p,"process ACTIVE region %d = ", index);
		change = 1;
		if (p->wvfid == OMAP3EPFB_WVFID_AUTO)
		{
			// Calculate the percentage of the screen that needs an update.
			int percent = ((rect_width(p) * rect_height(p) * 100)) /
			      ((info->var.xres-1) * (info->var.yres-1));

			// Check if we need to do a GC of the whole screen
			if ((par->refresh_percent > 0) && (percent >= par->refresh_percent))
			{
				DEBUG_REGION(DEBUG_LEVEL1, p,"process ACTIVE %d%% region %d = ", percent, index);
				p->x0 = p->y0 = 0;
				p->x1 = info->var.xres-1;
				p->y1 = info->var.yres-1;
				p->wvfid = OMAP3EPFB_WVFID_GC;
				p->threshold = 0;
			}
		}
		else
		{
			if (area->effect_flags & EFFECT_REGION)
			{
				p->x0 = area->effect_area.x0;
				p->y0 = area->effect_area.y0;
				p->x1 = area->effect_area.x1;
				p->y1 = area->effect_area.y1;
			}
		}
	}
	else if ((area->effect_flags & EFFECT_CLEAR) && rect_equal(&area->effect_area, p))
	{
		// Turn the next update of the effect area into a full page flushing update,
		// then clear the effect area.
		// This is used as a hint that a dialog is closing, then we forcing a full screen GC update.
		p->wvfid = area->effect_area.wvfid;
		p->threshold = area->effect_area.threshold;
		DEBUG_REGION(DEBUG_LEVEL2, p,"process RESET region %d = ", index);
		p->x0 = p->y0 = 0;
		p->x1 = info->var.xres-1;
		p->y1 = info->var.yres-1;
		p->wvfid = OMAP3EPFB_WVFID_GC;
		par->effect_array[index].effect_flags = 0;
		change = 1;
	}

	// Update the fast scan flags.
	update_effect(par, index);

	return change;
}

// Find the area that this update fit into.
// Then modify the update --> apply the waveform and flags from the matching area.  
static int waveform_select(struct fb_info *info, struct omap3epfb_update_area *p)
{
	struct omap3epfb_par *par = info->par;
	int i = 0;
	int ret = 0;

	mutex_lock(&par->area_mutex);
	{
		// Intercept updates to force waveform.
		if (par->effect_active != 0)
		{
			for (i = 0; i < EFFECT_ARRAY_SIZE; i++)
			{
				if ((par->effect_active & (1 << i)) &&
				    process_area(i, info, &par->effect_array[i], p))
				{
					// Override AUTO waveform behaviour
					batch_update(info, p);
					ret = 1;
					break;
				}
			}
		}
	}
	mutex_unlock(&par->area_mutex);

	return ret;
}

// Find the area(s) that fit into the update.
// If any, then draw the update and then draw the areas that fit into it on top.
// This functio is trying to recover from the case where Android merge multiple updates into
// a single update.
static int waveform_decompose(struct fb_info *info, struct omap3epfb_update_area *p)
{
	struct omap3epfb_par *par = info->par;
	int i = 0;
	int ret = 0; // Nothing to do

	DEBUG_LOG(DEBUG_LEVEL5,"DECOMPOSE++\n");
	mutex_lock(&par->area_mutex);
	{
		// Intercept updates to force waveform.
		if (par->effect_active != 0)
		{
			for (i = 0; i < EFFECT_ARRAY_SIZE; i++)
			{
				// Setup a new update
				struct omap3epfb_update_area new_sub_area = par->effect_array[i].effect_area;
				// Setup a new effect
				struct omap3epfb_area new_effect_area = {0};
				new_effect_area.effect_area = *p;
				new_effect_area.effect_area.wvfid = par->effect_array[i].effect_area.wvfid;
				new_effect_area.effect_flags = par->effect_array[i].effect_flags;
 
				if ((par->effect_active & (1 << i)) &&
				    process_area(i, info, &new_effect_area, &new_sub_area))
				{
					if ((ret == 0) && !rect_equal(&new_sub_area, p))
					{
						// Do the update as a normal update first
						batch_update(info, p);
					}
					// Indicate that we have a match.
					ret = 1;
					// Override AUTO waveform behaviour
					if (batch_update(info, &new_sub_area))
					{
						// If is was full screen, do not continue
						break;
					}
				}
			}
		}
	}
	mutex_unlock(&par->area_mutex);
	DEBUG_LOG(DEBUG_LEVEL5,"DECOMPOSE--\n");

	return ret;
}

int user_update(struct fb_info *info, struct omap3epfb_update_area *p)
{
	struct omap3epfb_par *par = info->par;
	int percent = 0;
	u32 start_time = 0;

	// Get time of update
	start_time = ktime_to_ms();
	DEBUG_LOG(DEBUG_LEVEL4, "[%u ms] Start user_update\n", start_time);

	// If EPD is disabled, do nothing
	if (par->pgflip_refresh == 1)
	{
		// Tell the caller not to update.
		return 0;
	}

	// If EPD is disabled, do nothing
	if (par->disable_flags > 0)
	{
		DEBUG_REGION(DEBUG_LEVEL3, p,"update DISABLED = ");
		// Tell the caller not to update.
		return 0;
	}
	DEBUG_REGION(DEBUG_LEVEL5, p,"update REQUEST = ");
	
	if (waveform_select(info, p))
	{
		// Tell the caller not to update.
		return 0;
	}

	if (waveform_decompose(info, p))
	{
		// Tell the caller not to update.
		return 0;
	}

	// Calculate the percentage of the screen that needs an update.
	percent = ((rect_width(p) * rect_height(p) * 100)) /
		   ((info->var.xres-1) * (info->var.yres-1));

	// Check if we need to do a GC of the whole screen
	if ((par->refresh_percent > 0) && (percent >= par->refresh_percent))
	{
		// Check again if this needs to be flushing.
		if (buffer_difference_ge_threshold(info, par->refresh_percent))
		{
			DEBUG_REGION(DEBUG_LEVEL1, p,"process FULLSCREEN %d%% AUTO = ", percent);
			p->x0 = p->y0 = 0;
			p->x1 = info->var.xres-1;
			p->y1 = info->var.yres-1;
			p->wvfid = OMAP3EPFB_WVFID_GC;
			p->threshold = 0;
		}
		else
		{
			DEBUG_REGION(DEBUG_LEVEL1, p,"process false FULLSCREEN %d%% AUTO = ", percent);
		}
	}
	else
	{
		DEBUG_REGION(DEBUG_LEVEL1, p,"process AUTO = ");
	}

	batch_update(info, p);

	// Tell the caller not to update.
	return 0;
}

int omap3epfb_fill_region(struct fb_info *info, struct omap3epfb_area *p)
{
	struct omap3epfb_par *par = info->par;

	if ((p->effect_area.threshold != OMAP3EPFB_THRESHOLD_BLACK) &&
	    (p->effect_area.threshold != OMAP3EPFB_THRESHOLD_WHITE))
		return -EINVAL;

	omap3epfb_reqq_purge(info);
	omap3epfb_update_area(info, &p->effect_area);

	DEBUG_REGION(DEBUG_LEVEL4, &p->effect_area,"  do fill = ");

	return 0;
}

int omap3epfb_set_region(struct fb_info *info, struct omap3epfb_area *p)
{
	struct omap3epfb_par *par = info->par;

	if ((p->index < 0) || (p->index >= EFFECT_ARRAY_SIZE))
		return -EINVAL;

	mutex_lock(&par->area_mutex);
	{
		// Set the correct flag for fast scan.
		if (p->effect_flags)
		{
			memcpy(&par->effect_array[p->index], p, sizeof(struct omap3epfb_area));
		}
		else
		{
			par->effect_array[p->index].effect_flags = 0;
		}
		update_effect(par, p->index);
	}
	mutex_unlock(&par->area_mutex);

	DEBUG_REGION(DEBUG_LEVEL4, &p->effect_area,"set effect=0x%02x region %d = ", p->effect_flags, p->index);

	return 0;
}

int omap3epfb_get_region(struct fb_info *info, struct omap3epfb_area *p)
{
	struct omap3epfb_par *par = info->par;

	if ((p->index < 0) || (p->index >= EFFECT_ARRAY_SIZE))
		return -EINVAL;

	mutex_lock(&par->area_mutex);
	{
		memcpy(p, &par->effect_array[p->index], sizeof(struct omap3epfb_area));
	}
	mutex_unlock(&par->area_mutex);

	DEBUG_REGION(DEBUG_LEVEL4, &p->effect_area,"get effect=0x%02x region %d = ", p->effect_flags, p->index);

	return 0;
}

int omap3epfb_reset_region(struct fb_info *info, int mask)
{
	struct omap3epfb_par *par = info->par;

	mutex_lock(&par->area_mutex);
	{
		int index;
		for (index = 0; index < EFFECT_ARRAY_SIZE; index++)
		{
			if (mask & (1 << index))
			{
				// Clear area
				memset(&par->effect_array[index], 0, sizeof(struct omap3epfb_area));
				update_effect(par, index);
				DEBUG_LOG(DEBUG_LEVEL4,"reset region %d\n", index);
			}
		}
	}
	mutex_unlock(&par->area_mutex);

	return 0;
}

// ATTRIBUTES
/////////////

static ssize_t store_refresh(struct device *device,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	int index = FIRST_USER_REGION;
	struct omap3epfb_area *area = NULL;
	int clear  = simple_strtoul(buf, NULL, 0);

	if (clear > 2 || clear < 0)
		return -EINVAL;

	omap3epfb_reqq_purge(fb);
	switch (clear)
	{
		case 0: omap3epfb_update_screen(fb, OMAP3EPFB_WVFID_AUTO, false);
			break;

		case 1: omap3epfb_update_screen(fb, OMAP3EPFB_WVFID_GC, false);
			break;

		case 2: area = &par->effect_array[index];
			if (!(area->effect_flags & (EFFECT_ONESHOT | EFFECT_ACTIVE | EFFECT_CLEAR)))
				break;
			omap3epfb_update_area(fb, &area->effect_area);
			break;

		default:
			omap3epfb_update_screen(fb, OMAP3EPFB_WVFID_AUTO, false);
	}

	return count;
}

static ssize_t show_refresh(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0\n");
}

static ssize_t store_disable(struct device *device,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	int tmp  = simple_strtoul(buf, NULL, 0);
	u32 t_expires = 0;

	if (tmp < 0)
		return -EINVAL;
		
	par->disable_flags = tmp;

	if (tmp > 0)
	{
		// Schedule work only if no work is currently scheduled, or if its
		// expiration time is later than a currently scheduled work item
		t_expires = ktime_to_ms();
		t_expires += (u32)par->disable_flags;
		if (t_expires > g_disable_time)
		{
			cancel_delayed_work_sync(&par->clear_work);
			cancel_delayed_work_sync(&par->disable_work);
			par->disable_flags = tmp; /* again, in case disable_work just ran */
			schedule_delayed_work(&par->disable_work, msecs_to_jiffies(par->disable_flags));
			g_disable_time = t_expires; // Set new expiration time
			DEBUG_LOG(DEBUG_LEVEL4,"disable EPD Updates for %dms\n", par->disable_flags);
		}
		else
		{
			DEBUG_LOG(DEBUG_LEVEL4,"EPD disable (t = %u ms) request ignored; later disable (t = %u ms) currently scheduled\n", t_expires, g_disable_time);
		}
	}
	else
	{
		cancel_delayed_work_sync(&par->disable_work);
		g_disable_time = 0;  // Reset expiration time
		par->disable_flags = tmp; /* again, in case disable_work just ran */
		DEBUG_LOG(DEBUG_LEVEL4,"enable EPD Updates\n");
	}

	return count;
}

static ssize_t show_disable(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	return snprintf(buf, PAGE_SIZE, "%d\n", par->disable_flags);
}

static ssize_t store_area(struct device *device,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fb_info *fb = dev_get_drvdata(device);

	int index = FIRST_USER_REGION; // Backwards compatibility, map first app area.
	char *last = NULL;
	struct omap3epfb_area p = {0};
	unsigned wvfid;

	p.effect_area.x0 = simple_strtoul(buf, &last, 0);
	last++;
	if (last - buf >= count)
		return -EINVAL;

	p.effect_area.y0 = simple_strtoul(last, &last, 0);
	last++;
	if (last - buf >= count)
		return -EINVAL;

	p.effect_area.x1 = simple_strtoul(last, &last, 0);
	last++;
	if (last - buf >= count)
		return -EINVAL;

	p.effect_area.y1 = simple_strtoul(last, &last, 0);
	last++;
	if (last - buf >= count)
		return -EINVAL;

	// Threshold stored in upper 16 bits.	
	// Waveform id in lower 16 bits.
	wvfid = simple_strtoul(last, &last, 0);
	last++;
	if (last - buf >= count)
		return -EINVAL;

	p.effect_flags = simple_strtoul(last, &last, 0);

	p.effect_area.wvfid = wvfid & 0x0000ffff;
	p.effect_area.threshold = (wvfid & 0xffff0000) >> 16;

	p.index = index;

	omap3epfb_set_region(fb, &p);

	return count;
}


static ssize_t show_area(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;

	int index = FIRST_USER_REGION; // Backwards compatibility, map first app area.

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d\n",
			par->effect_array[index].effect_area.x0, par->effect_array[index].effect_area.y0,
			par->effect_array[index].effect_area.x1, par->effect_array[index].effect_area.y1,
			par->effect_array[index].effect_area.wvfid,
			par->effect_array[index].effect_area.threshold,
			par->effect_array[index].effect_flags);
}


static ssize_t store_pgflip(struct device *device,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	int tmp  = simple_strtoul(buf, NULL, 0);

	if (tmp < 0)
		return -EINVAL;

	par->pgflip_refresh = tmp;

	return count;
}

static ssize_t show_pgflip(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	return snprintf(buf, PAGE_SIZE, "%d\n", par->pgflip_refresh);
}

static ssize_t store_percent(struct device *device,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	int tmp  = simple_strtoul(buf, NULL, 0);

	if ((tmp < 0) || (tmp > 100))
		return -EINVAL;

	par->refresh_percent = tmp;

	return count;
}

static ssize_t show_percent(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	return snprintf(buf, PAGE_SIZE, "%d\n", par->refresh_percent);
}

static ssize_t store_debug(struct device *device,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	int tmp  = simple_strtoul(buf, NULL, 0);

	par->user_debug = tmp;

	return count;
}

static ssize_t show_debug(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	return snprintf(buf, PAGE_SIZE, "%d\n", par->user_debug);
}

static ssize_t store_delay(struct device *device,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	int tmp  = simple_strtoul(buf, NULL, 0);

	par->clear_delay = tmp;

	return count;
}

static ssize_t show_delay(struct device *device,
			    struct device_attribute *attr, char *buf)
{
	struct fb_info *fb = dev_get_drvdata(device);
	struct omap3epfb_par *par = fb->par;
	return snprintf(buf, PAGE_SIZE, "%d\n", par->clear_delay);
}

static ssize_t show_cpld_hw_rev(struct device *device,
				struct device_attribute *attr, char *buf)
{
	// Read CPLD Hardware version.
	int cpld_hw_ver = gpio_get_value(CPLD_V_DET1_GPIO) |
		       (gpio_get_value(CPLD_V_DET2_GPIO) << 1);

	return snprintf(buf, PAGE_SIZE, "%d\n", cpld_hw_ver);
}

static struct device_attribute device_attrs[] = {
	__ATTR(epd_refresh,    S_IRUGO|S_IWUGO, show_refresh, store_refresh),
	__ATTR(epd_disable,    S_IRUGO|S_IWUGO, show_disable, store_disable),
	__ATTR(epd_area,       S_IRUGO|S_IWUGO, show_area, store_area),
	__ATTR(epd_percent,    S_IRUGO|S_IWUGO, show_percent, store_percent),
	__ATTR(epd_delay,      S_IRUGO|S_IWUGO, show_delay, store_delay),
	__ATTR(epd_debug,      S_IRUGO|S_IWUSR, show_debug, store_debug),
	__ATTR(pgflip_refresh, S_IRUGO|S_IWUGO, show_pgflip, store_pgflip),
	__ATTR(cpld_hw_rev,    S_IRUGO|S_IWUSR, show_cpld_hw_rev, NULL),
};


int user_init_device(struct fb_info *fb_info)
{
	int i, error = 0;
	struct omap3epfb_par *par = fb_info->par;

	fb_info->class_flag |= FB_SYSFS_FLAG_ATTR;

	mutex_init(&par->area_mutex);

	for (i = 0; i < ARRAY_SIZE(device_attrs); i++) {
		error = device_create_file(fb_info->dev, &device_attrs[i]);

		if (error)
			break;
	}

	if (error) {
		while (--i >= 0)
			device_remove_file(fb_info->dev, &device_attrs[i]);
		fb_info->class_flag &= ~FB_SYSFS_FLAG_ATTR;
	}
	INIT_DELAYED_WORK_DEFERRABLE(&par->clear_work, omap3epfb_clear_work);
	INIT_DELAYED_WORK_DEFERRABLE(&par->disable_work, omap3epfb_disable_work);

	// Start with page flip mode active.
	par->pgflip_refresh = 1;

	// Disabled as default.
	par->clear_delay = 0;

	par->effect_active = 0;
	strncpy(par->effect_active_debug, "--------", EFFECT_ARRAY_SIZE);
	par->effect_active_debug[EFFECT_ARRAY_SIZE] = 0;

	return 0;
}

void user_cleanup_device(struct fb_info *fb_info)
{
	unsigned int i;
	struct omap3epfb_par *par = fb_info->par;

	if (fb_info->class_flag & FB_SYSFS_FLAG_ATTR) {
		for (i = 0; i < ARRAY_SIZE(device_attrs); i++)
			device_remove_file(fb_info->dev, &device_attrs[i]);

		fb_info->class_flag &= ~FB_SYSFS_FLAG_ATTR;
	}
	cancel_delayed_work_sync(&par->clear_work);
	cancel_delayed_work_sync(&par->disable_work);
	g_disable_time = 0;  // Reset expiration time
}
