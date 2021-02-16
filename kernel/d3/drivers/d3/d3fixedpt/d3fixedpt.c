/* d3fixedpt.c - Q42.22 fixed-point in the Linux kernel
 * (actually signed S42.22, so don't use that top bit!)
 *
 * Modified from https://github.com/cxw42/fixedptc-fork , which is
 * based on https://sourceforge.net/projects/fixedptc/
 *
 * Copyright (c) 2020 D3 Engineering, LLC.
 * By Christopher White <cwhite@d3engineering.com>.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * See also acknowlegements at end of file
 */

#define MODULE_NAME "d3fixedpt"

/* Grab EXPORT_SYMBOL */
#include <linux/export.h>

/* Suppress warnings */
#ifndef __ARM_EABI__
#define __ARM_EABI__ 0
#endif

#ifndef __APPLE__
#define __APPLE__ 0
#endif

#ifndef __Fuchsia__
#define __Fuchsia__ 0
#endif

/* Grab the necessary fixed-point helpers */
#include "int_lib.h"
#include "int_util.c"

/* Additional symbols we need */

#if !defined(CHAR_BIT) && defined(__CHAR_BIT__)
#define CHAR_BIT __CHAR_BIT__
#endif

#include "clzsi2.c"
#include "divti3.c"
#include "udivmodti4.c"
#include "udivti3.c"
#include "umodti3.c"

/* Grab the fixed-point functions */
#define FIXEDPTC_IMPLEMENTATION
#include <d3/fixedpt.h>

/* Make sure the upstream format hasn't changed */
#include <media/camera_common.h>
#if FIXED_POINT_SCALING_FACTOR != (1ULL << FIXEDPT_FBITS)
#error "Please update this file --- FIXED_POINT_SCALING_FACTOR has changed"
#endif

#include <linux/module.h>

#ifdef CONFIG_D3_FIXEDPT_STARTUP_TESTS

static void
fixedpt_print(fixedpt A)
{
	char num[64];

	fixedpt_str(A, num, -2);
	pr_info(MODULE_NAME " %s", num);
}


/**
 * d3fixedpt_init() - startup routine - run some tests
 *
 * Return: always 0
 */
static int __init
d3fixedpt_init(void)
{
	fixedpt A, B, C;

	// __LINE__: Poor-man's double-check that we have the right file
	pr_info(MODULE_NAME ": Initializing " __stringify(__LINE__) "\n");

	pr_info(MODULE_NAME " fixedptc library version: %s\n", FIXEDPT_VCSID);
	pr_info(MODULE_NAME " Using %d-bit precision, %d.%d format\n\n", FIXEDPT_BITS,
		FIXEDPT_WBITS, FIXEDPT_FBITS);

	pr_info(MODULE_NAME " The most precise number: ");
	fixedpt_print(1);
	pr_info(MODULE_NAME " The biggest number: ");
	fixedpt_print(U64_MAX);
	pr_info(MODULE_NAME " The biggest number >> 1: ");
	fixedpt_print(U64_MAX >> 1);
	pr_info(MODULE_NAME " Here are some example numbers:\n");

	pr_info(MODULE_NAME " Random number: ");
	fixedpt_print(fixedpt_rconst(143.125));
	pr_info(MODULE_NAME " PI: ");
	fixedpt_print(FIXEDPT_PI);
	pr_info(MODULE_NAME " e: ");
	fixedpt_print(FIXEDPT_E);

	A = fixedpt_rconst(2.5);
	B = fixedpt_fromint(3);

	fixedpt_print(A);
	pr_info(MODULE_NAME " +");
	fixedpt_print(B);
	C = fixedpt_add(A, B);
	pr_info(MODULE_NAME " =");
	fixedpt_print(C);

	fixedpt_print(A);
	pr_info(MODULE_NAME " *");
	fixedpt_print(B);
	pr_info(MODULE_NAME " =");
	C = fixedpt_mul(A, B);
	fixedpt_print(C);

	A = fixedpt_rconst(1);
	B = fixedpt_rconst(4);
	C = fixedpt_div(A, B);

	fixedpt_print(A);
	pr_info(MODULE_NAME " /");
	fixedpt_print(B);
	pr_info(MODULE_NAME " =");
	fixedpt_print(C);

	pr_info(MODULE_NAME " exp(1)=");
	fixedpt_print(fixedpt_exp(FIXEDPT_ONE));

	pr_info(MODULE_NAME " sqrt(pi)=");
	fixedpt_print(fixedpt_sqrt(FIXEDPT_PI));

	pr_info(MODULE_NAME " sqrt(25)=");
	fixedpt_print(fixedpt_sqrt(fixedpt_rconst(25)));

	pr_info(MODULE_NAME " sin(pi/2)=");
	fixedpt_print(fixedpt_sin(FIXEDPT_HALF_PI));

	pr_info(MODULE_NAME " sin(3.5*pi)=");
	fixedpt_print(fixedpt_sin(fixedpt_mul(fixedpt_rconst(3.5), FIXEDPT_PI)));

	pr_info(MODULE_NAME " 4^3.5=");
	fixedpt_print(fixedpt_pow(fixedpt_rconst(4), fixedpt_rconst(3.5)));

	pr_info(MODULE_NAME " 4^0.5=");
	fixedpt_print(fixedpt_pow(fixedpt_rconst(4), fixedpt_rconst(0.5)));

	pr_info(MODULE_NAME " log10(200) (should be ~2.301029996)=");
	fixedpt_print(fixedpt_log(fixedpt_fromint(200), fixedpt_fromint(10)));

	pr_info(MODULE_NAME " trunc 1.5 = %ld",
		(long int)fixedpt_toint_trunc(FIXEDPT_ONE + FIXEDPT_ONE_HALF));

	pr_info(MODULE_NAME " round 1.5 = %ld",
		(long int)fixedpt_toint_round(FIXEDPT_ONE + FIXEDPT_ONE_HALF));

	return 0;
}

module_init(d3fixedpt_init);

#endif /* CONFIG_D3_FIXEDPT_STARTUP_TESTS */

/*
 * Includes code from https://github.com/cxw42/fixedptc-fork/blob/master/test.c
 * That code is licensed as follows:
 *
 * Copyright (c) 2010-2012 Ivan Voras <ivoras@freebsd.org>
 * Copyright (c) 2012 Tim Hartrick <tim@edgecast.com>
 * Copyright (c) 2020 Damian Wrobel <dwrobel@ertelnet.rybnik.pl>
 * Copyright (c) 2020 D3 Engineering, LLC.  Some code by
 *   Christopher White <cwhite@d3engineering.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

MODULE_DESCRIPTION("Driver that provides S42.22 fixed-point math functions.");
MODULE_AUTHOR("Christopher White <cwhite@d3engineering.com>");
MODULE_LICENSE("GPL v2");
