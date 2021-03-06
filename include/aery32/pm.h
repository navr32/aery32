/**
 * \file aery32/pm.h
 * \brief Power Manager (PM)
 *
 * \verbatim
 *  _____             ___ ___   |
 * |  _  |___ ___ _ _|_  |_  |  |  Teh framework for 32-bit AVRs
 * |     | -_|  _| | |_  |  _|  |  
 * |__|__|___|_| |_  |___|___|  |  https://github.com/aery32
 *               |___|          |
 * 
 * Copyright (c) 2012, Muiku Oy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *    * Neither the name of Muiku Oy nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * \endverbatim
 */
 
#ifndef __AERY32_PM_H
#define __AERY32_PM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <inttypes.h>
#include <avr32/io.h>

#ifdef AERY_SHORTCUTS
	extern volatile avr32_pm_t *pm;
	extern volatile avr32_pm_pll_t *pll0;
	extern volatile avr32_pm_pll_t *pll1;
#endif

enum Pm_gclk {
	PM_GCLK0,
	PM_GCLK1,
	PM_GCLK2,
	PM_GCLK3,
	PM_GCLK_USBB,
	PM_GCLK_ABDAC
};

enum Pm_gclk_source {
	PM_GCLK_SOURCE_OSC0 = 0b00,
	PM_GCLK_SOURCE_OSC1 = 0b01,
	PM_GCLK_SOURCE_PLL0 = 0b10,
	PM_GCLK_SOURCE_PLL1 = 0b11
};

enum Pm_pll_source {
	PM_PLL_SOURCE_OSC0,
	PM_PLL_SOURCE_OSC1
};

enum Pm_mck_source {
	PM_MCK_SOURCE_RC,
	PM_MCK_SOURCE_OSC0,
	PM_MCK_SOURCE_PLL0
};

enum Pm_osc_mode {
	PM_OSC_MODE_EXTERNAL,
	PM_OSC_MODE_OSC32,
	PM_OSC_MODE_GAIN0 = 4,
	PM_OSC_MODE_GAIN1,
	PM_OSC_MODE_GAIN2,
	PM_OSC_MODE_GAIN3
};

enum Pm_osc_startup {
	PM_OSC_STARTUP_0,
	PM_OSC_STARTUP_560us,
	PM_OSC_STARTUP_1m1s,
	PM_OSC_STARTUP_18ms,
	PM_OSC_STARTUP_36ms,
	PM_OSC_STARTUP_71ms,
	PM_OSC_STARTUP_142ms,
	PM_OSC32_STARTUP_0 = 0,
	PM_OSC32_STARTUP_1m1s,
	PM_OSC32_STARTUP_72m3s,
	PM_OSC32_STARTUP_134ms,
	PM_OSC32_STARTUP_570ms,
	PM_OSC32_STARTUP_1s1,
	PM_OSC32_STARTUP_2s3,
	PM_OSC32_STARTUP_4s6
};

/**
 * Starts the chosen oscillator
 *
 * \note After start you shall wait OSC to stabilize
 *
 * \par Example
 * \code
 * // Start 12MHz crystal connected in XIN0/XOUT0
 * aery_pm_start_osc(0, PM_OSC_MODE_GAIN3, PM_OSC_STARTUP_36ms);
 * aery_pm_wait_osc_to_stabilize(0);
 * \endcode
 *
 * \param oscnum Oscillator number which to start: 0, 1, 32
 * \param mode Oscillator mode, see datasheet p. 74.
 * \param startup Oscillator startup time
 * \return Returns 0 on success and -1 on error
 */
int aery_pm_start_osc(uint8_t, enum Pm_osc_mode, enum Pm_osc_startup);

/**
 * Initializes the frequency of PLL's voltage controlled oscillator (VCO)
 *
 * f_vco = f_src * mul / div, if div > 0
 * f_vco = 2 * f_src * mul, if div == 0
 *
 * \note VCO frequency has to be in range of 80 MHz - 180 MHz or if hifreq mode
 *       is enabled in the range of 160 MHz - 240 MHz. 
 *
 * \param ppll Pointer to pll register
 * \param src Source oscillator
 * \param mul Source frequency is multiplier by this value
 * \param div Source frequency is divided by this value. If zero the 
 *            multiplier is doubled
 * \pram hifreq Enable high frequency mode
 */
int aery_pm_init_pllvco(volatile avr32_pm_pll_t*, enum Pm_pll_source, uint8_t,
                        uint8_t, bool);

/**
 * Enables phase locked loop (PLL)
 *
 * \note Before enabling PLL you have to initialize PLL vco frequency
 *       by calling aery_pm_init_pllvco()
 *
 * \param ppll Poiter to pll register
 * \param divby2 Divide the vco frequency by two
 */
void aery_pm_enable_pll(volatile avr32_pm_pll_t*, bool);

/**
 * Initializes the chosen generic clock
 *
 * \param clknum Generic clock number: 0-5
 * \param clksrc Source of the generic clock
 * \param div Clock source divider, f_gclk = f_src/(2*(div+1))
 * \return Returns 0 on success and -1 on error
 */
int aery_pm_init_gclk(enum Pm_gclk, enum Pm_gclk_source, uint8_t);

/**
 * Waits oscillator to stabilize
 *
 * \param oscnum Oscillator number: 0, 1 or 32
 */
inline void aery_pm_wait_osc_to_stabilize(uint8_t oscnum)
{
	switch (oscnum) {
	case 0:
		while (!(AVR32_PM.isr & AVR32_PM_ISR_OSC0RDY_MASK));
		break;
	case 1:
		while (!(AVR32_PM.isr & AVR32_PM_ISR_OSC1RDY_MASK));
		break;
	case 32:
		while (!(AVR32_PM.isr & AVR32_PM_ISR_OSC32RDY_MASK));
		break;
	}
}

/**
 * Waits pll to lock
 *
 * \param ppll Pointer to PLL register
 */
inline void aery_pm_wait_pll_to_lock(volatile avr32_pm_pll_t *ppll)
{
	if (ppll == &AVR32_PM.PLL[0]) {
		while (!(AVR32_PM.isr & AVR32_PM_ISR_LOCK0_MASK));
	}
	else if (ppll == &AVR32_PM.PLL[1]) {
		while (!(AVR32_PM.isr & AVR32_PM_ISR_LOCK1_MASK));
	}
}

/**
 * Enables the chosen generic clock
 *
 * \param clknum Clock number which to enable
 */
inline void aery_pm_enable_gclk(enum Pm_gclk clknum)
{
	AVR32_PM.GCCTRL[clknum].cen = 1;
}

/**
 * Disables the chosen generic clock
 *
 * \param clknum Clock number which to disable
 */
inline void aery_pm_disable_gclk(enum Pm_gclk clknum)
{
	AVR32_PM.GCCTRL[clknum].cen = 0;

	// We have to wait before cen reads zero.
	while (AVR32_PM.GCCTRL[clknum].cen);
}

/**
 * Selects master clock source
 *
 * \param mcksrc Clock source
 */
inline void aery_pm_select_mck(enum Pm_mck_source mcksrc)
{
	AVR32_PM.MCCTRL.mcsel = mcksrc;
}

#ifdef __cplusplus
}
#endif

#endif
