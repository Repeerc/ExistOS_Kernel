

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm.h"
#include "chip.h"
#include "arm_internal.h"

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#  include "pg_macros.h"
#endif


	.file	"stmp3770_sysdat.S"
	
	
	.data
	.align	4
	.globl	g_idle_topstack
	.type	g_idle_topstack, object

g_idle_topstack:

	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_idle_topstack, .-g_idle_topstack
	.end
