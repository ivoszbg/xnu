/*
 * Copyright 2013, winocm. <winocm@icloud.com>
 * Copyright 2018, Bingxing Wang. <i@imbushuo.net>
 * Copyright 2024, Ivaylo Ivanov <ivo.ivanov.ivanov1@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 *   If you are going to use this software in any form that does not involve
 *   releasing the source to this project or improving it, let me know beforehand.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if defined(BOARD_CONFIG_MSM8916_J5LTE)

#include <mach/mach_types.h>

#include <IOKit/IOPlatformExpert.h>

#include <pexpert/pexpert.h>
#include <pexpert/arm/protos.h>
#include <pexpert/arm/boot.h>

#include <machine/machine_routines.h>

#include <vm/pmap.h>
#include <arm/pmap.h>

#include "mdp5.h"
#include "pe_msm8916.h"
#include "pe_j5lte.h"

#define KPRINTF_PREFIX  "PE_MSM8916: "

/* BLSP UART */
vm_offset_t gMsmBlspUartBase;

/* GIC */
vm_offset_t         gMsmQGICCPUBase;
vm_offset_t         gMsmQGICDistributerBase;

/* Timer */
vm_offset_t         gMsmQTimerBase;
static uint64_t     clock_decrementer = 0;
static boolean_t    clock_initialized = FALSE;
static boolean_t    clock_had_irq = FALSE;
static uint64_t     clock_absolute_time = 0;
static uint32_t     ticks_per_sec = 19200000;

/* Functions */
void msm8974_uart_dm_putc(int c);

void udelay(unsigned usecs);
extern void rtc_configure(uint64_t hz);

/*
 * Stub for printing out to framebuffer.
 */
void vcputc(__unused int l, __unused int u, int c);

/* Timer */
static void timer_configure(void)
{
	// Map QTimer
	gMsmQTimerBase = ml_io_map(QTMR_BASE, PAGE_SIZE);

	// qtimer_init @ 19.2MHz
	uint64_t hz = 19200000;
	gPEClockFrequencyInfo.timebase_frequency_hz = hz;

	clock_decrementer = 1000;
	kprintf(KPRINTF_PREFIX "decrementer frequency = %llu\n", clock_decrementer);

	rtc_configure(hz);
	return;
}

void qtimer_enabled(int enable)
{
	uint32_t ctrl;

	assert(gMsmQTimerBase);
	ctrl = readl(gMsmQTimerBase + QTMR_V1_CNTP_CTL);

	if (enable)
	{
		/* Program CTRL Register */
		ctrl |= QTMR_TIMER_CTRL_ENABLE;
		ctrl &= ~QTMR_TIMER_CTRL_INT_MASK;
	}
	else
	{
		/* program cntrl register */
		ctrl &= ~QTMR_TIMER_CTRL_ENABLE;
		ctrl |= QTMR_TIMER_CTRL_INT_MASK;
	}

	writel(ctrl, gMsmQTimerBase + QTMR_V1_CNTP_CTL);
	barrier();

	return;
}

uint32_t qtimer_tick_rate()
{
	return ticks_per_sec;
}

void qtimer_init(void)
{
	uint32_t tick_count;

	// Set RTC Clock, map QTimer.
	timer_configure();

	// Disable QTimer.
	qtimer_enabled(FALSE);

	// Save the timer interval and call back data
	tick_count = clock_decrementer * qtimer_tick_rate() / 1000;
	writel(tick_count, gMsmQTimerBase + QTMR_V1_CNTP_TVAL);
	barrier();

	// Unmask interrupt. (INT_QTMR_FRM_0_PHYSICAL_TIMER_EXP)
	uint32_t reg = GIC_DIST_ENABLE_SET + (INT_QTMR_FRM_0_PHYSICAL_TIMER_EXP / 32) * 4;
	uint32_t bit = 1 << (INT_QTMR_FRM_0_PHYSICAL_TIMER_EXP & 31);
	writel(bit, gMsmQGICDistributerBase + reg);

	// Enable interrupts.
	ml_set_interrupts_enabled(TRUE);

	// Enable QTimer.
	qtimer_enabled(TRUE);

	// Set flag. UART is available now.
	clock_initialized = TRUE;
	
	return;
}

uint64_t timer_value(void)
{
	/* Don't bother. (Why?) */
	uint64_t ret = 0;
	return ret;
}

uint64_t qtimer_get_timebase(void)
{
	uint32_t timestamp;

	if (!clock_initialized) return 0;

	timestamp = timer_value();
	if (timestamp) 
	{
		uint64_t v = clock_absolute_time;
		v += (uint64_t) (((uint64_t) clock_decrementer) - (uint64_t) (timestamp));
		return v;
	} 
	else 
	{
		clock_absolute_time += clock_decrementer;
		return clock_absolute_time;
	}
}

inline __ALWAYS_INLINE uint64_t qtimer_get_phy_timer_cnt()
{
	uint32_t phy_cnt_lo;
	uint32_t phy_cnt_hi_1;
	uint32_t phy_cnt_hi_2;

	do {
		phy_cnt_hi_1 = readl(gMsmQTimerBase + QTMR_V1_CNTPCT_HI);
		phy_cnt_lo = readl(gMsmQTimerBase + QTMR_V1_CNTPCT_LO);
		phy_cnt_hi_2 = readl(gMsmQTimerBase + QTMR_V1_CNTPCT_HI);
	} while (phy_cnt_hi_1 != phy_cnt_hi_2);

	return ((uint64_t) phy_cnt_hi_1 << 32) | phy_cnt_lo;
}

/* Blocking function to wait until the specified ticks of the timer.
 * Note: ticks to wait for cannot be more than 56 bit.
 * Should be sufficient for all practical purposes.
 */
static void delay(uint64_t ticks)
{
	volatile uint64_t cnt;
	uint64_t init_cnt;
	uint64_t timeout = 0;

	cnt = qtimer_get_phy_timer_cnt();
	init_cnt = cnt;

	/* Calculate timeout = cnt + ticks (mod 2^56)
	 * to account for timer counter wrapping
	 */
	timeout = (cnt + ticks) & (uint64_t)(QTMR_PHY_CNT_MAX_VALUE);

	/* Wait out till the counter wrapping occurs
	 * in cases where there is a wrapping.
	 */
	while(timeout < cnt && init_cnt <= cnt)
		/* read global counter */
		cnt = qtimer_get_phy_timer_cnt();

	/* Wait till the number of ticks is reached*/
	while(timeout > cnt)
		/* read global counter */
		cnt = qtimer_get_phy_timer_cnt();

}

void udelay(unsigned usecs)
{
	uint64_t ticks;

	ticks = ((uint64_t) usecs * ticks_per_sec) / 1000000;

	delay(ticks);
}

/* Interrupt Routine */
static uint8_t qgic_get_cpumask()
{
	uint32_t mask=0, i;

	/* Fetch the CPU MASK from the SGI/PPI reg */
	for (i=0; i < 32; i += 4) {
		mask = readl(gMsmQGICDistributerBase + GIC_DIST_TARGET + i);
		mask |= mask >> 16;
		mask |= mask >> 8;
		if (mask)
			break;
	}
	return mask;
}

/* Intialize distributor */
static void qgic_dist_config(uint32_t num_irq)
{
	uint32_t i;

	/* Set each interrupt line to use N-N software model
	 * and edge sensitive, active high
	 */
	for (i = 32; i < num_irq; i += 16)
		writel(0xffffffff, gMsmQGICDistributerBase + (GIC_DIST_CONFIG + i * 4 / 16));

	writel(0xffffffff, gMsmQGICDistributerBase + GIC_DIST_CONFIG + 4);

	/* Set priority of all interrupts */

	/*
	 * In bootloader we dont care about priority so
	 * setting up equal priorities for all
	 */
	for (i = 0; i < num_irq; i += 4)
		writel(0xa0a0a0a0, gMsmQGICDistributerBase + (GIC_DIST_PRI + i * 4 / 4));

	/* Disabling interrupts */
	for (i = 0; i < num_irq; i += 32)
		writel(0xffffffff, gMsmQGICDistributerBase + (GIC_DIST_ENABLE_CLEAR + i * 4 / 32));

	writel(0x0000ffff, gMsmQGICDistributerBase + GIC_DIST_ENABLE_SET);
}

static void qgic_dist_init(void)
{
	uint32_t i;
	uint32_t num_irq = 0;
	uint32_t cpumask = 1;

	cpumask = qgic_get_cpumask();

	cpumask |= cpumask << 8;
	cpumask |= cpumask << 16;

	/* Disabling GIC */
	writel(0, gMsmQGICDistributerBase + GIC_DIST_CTRL);

	/*
	 * Find out how many interrupts are supported.
	 */
	num_irq = readl(gMsmQGICDistributerBase + GIC_DIST_CTR) & 0x1f;
	num_irq = (num_irq + 1) * 32;

	/* Set up interrupts for this CPU */
	for (i = 32; i < num_irq; i += 4)
		writel(cpumask, gMsmQGICDistributerBase + GIC_DIST_TARGET + i * 4 / 4);

	qgic_dist_config(num_irq);

	/* Enabling GIC */
	writel(1, gMsmQGICDistributerBase + GIC_DIST_CTRL);
}

static void qgic_cpu_init(void)
{
	writel(0xf0, gMsmQGICCPUBase + GIC_CPU_PRIMASK);
	writel(1, gMsmQGICCPUBase + GIC_CPU_CTRL);

	return;
}

void qgic_init(void)
{
	assert(gMsmQGICCPUBase && gMsmQGICDistributerBase);

	/* Initialize QGIC. Called from platform specific init code */
	qgic_dist_init();
	qgic_cpu_init();

	return;
}

void handle_qgic(void *context)
{
	uint32_t irq_no = readl(gMsmQGICCPUBase + GIC_CPU_INTACK);
	if(irq_no > NR_IRQS) 
	{
		kprintf(KPRINTF_PREFIX "Got a bogus IRQ?");
		return;
	}

	/* Timer interrupt? */
	if(irq_no == INT_QTMR_FRM_0_PHYSICAL_TIMER_EXP) 
	{
		qtimer_enabled(FALSE);
		clock_absolute_time += (clock_decrementer - (int64_t) timer_value());
		rtclock_intr((arm_saved_state_t *) context);
		qtimer_enabled(TRUE);
		clock_had_irq = TRUE;
	} 
	else 
	{
		irq_iokit_dispatch(irq_no);
	}

	/* EOI */
	writel(irq_no, gMsmQGICCPUBase + GIC_CPU_EOI);

	return;
}

/* Framebuffer */
static void _fb_putc(int c)
{
	if (c == '\n') {
		vcputc(0, 0, '\r');
	}
	vcputc(0, 0, c);
}

/* Initialize a framebuffer */
void panel_init(void)
{
	char tempbuf[16]; 
	uint64_t panel_width = PANEL_WIDTH,
		 panel_height = PANEL_HEIGHT;


	/*
	* The hardware demands a framebuffer, but the framebuffer has to be given
	* in a hardware address.
	*/

	/* void *framebuffer = pmap_steal_memory(1280 * 720 * 4);
	 * void *framebuffer_phys = pmap_extract(kernel_pmap, framebuffer);
	 */

	PE_state.video.v_baseAddr = (unsigned long)0x8e000000;
	PE_state.video.v_rowBytes = panel_width * 4;
	PE_state.video.v_width = panel_width;
	PE_state.video.v_height = panel_height;
	PE_state.video.v_depth = 4 * (8);   /* Always 32bpp */

	kprintf(KPRINTF_PREFIX "Framebuffer initialized\n");

	/*
	 * Enable early framebuffer.
	 */
	if (PE_parse_boot_argn("-early-fb-debug", tempbuf, sizeof(tempbuf)))
		initialize_screen((void *) &PE_state.video, kPEAcquireScreen);
	else if (PE_parse_boot_argn("-graphics-mode", tempbuf, sizeof(tempbuf))) {
		memset(PE_state.video.v_baseAddr, 0xb9, PE_state.video.v_rowBytes * PE_state.video.v_height);

		initialize_screen((void *) &PE_state.video, kPEGraphicsMode);
	} else {
		initialize_screen((void *) &PE_state.video, kPETextMode);
	}
	
	return;
}

/* Map to SPMI restart function */
int spmi_halt_restart(int type)
{
	writel(1, 0x004AB000);
	return 0;
}

void msm8916_mapping_init(void)
{
	/* Map gic base and distr */
	gMsmQGICCPUBase = ml_io_map(MSM_GIC_CPU_BASE, PAGE_SIZE);
	gMsmQGICDistributerBase = ml_io_map(MSM_GIC_DIST_BASE, PAGE_SIZE);

	return;
}

void PE_board_init(void)
{
	msm8916_mapping_init();

	gPESocDispatch.uart_getc = unimplemented;
	gPESocDispatch.uart_putc = unimplemented;
	gPESocDispatch.uart_init = unimplemented;

	gPESocDispatch.interrupt_init = qgic_init;
	gPESocDispatch.handle_interrupt = handle_qgic;

	gPESocDispatch.timebase_init = qtimer_init;
	gPESocDispatch.get_timebase = qtimer_get_timebase;

	gPESocDispatch.timer_value = timer_value;
	gPESocDispatch.timer_enabled = qtimer_enabled;

	gPESocDispatch.framebuffer_init = panel_init;

	panel_init();
	
	PE_halt_restart = spmi_halt_restart;
}

void PE_init_SocSupport_stub(void)
{
	PE_early_puts("PE_init_SocSupport: Initializing for Galaxy J5 2015\n");
	PE_board_init();
}

#endif /* !BOARD_CONFIG_MSM8916_J5LTE */
