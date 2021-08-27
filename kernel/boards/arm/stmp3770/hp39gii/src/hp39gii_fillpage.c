/****************************************************************************
 * boards/arm/stmp3770/hp39gii/src/lpc31_fillpage.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/page.h>

#ifdef CONFIG_PAGING
#ifdef CONFIG_PAGING_BINPATH
#  include <sys/stat.h>
#  include <sys/types.h>
#  include <stdbool.h>
#  include <unistd.h>
#  include <fcntl.h>
#  include <nuttx/fs/fs.h>
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fillpage()
 *
 * Description:
 *  After a page is allocated and mapped by up_allocpage(), the actual
 *  filling of the page with data from the non-volatile, must be performed
 *  by a separate call to the architecture-specific function, up_fillpage().
 *  This function is non-blocking, it will start an asynchronous page fill.
 *  The common paging logic will provide a callback function, pg_callback,
 *  that will be called when the page fill is finished (or an error occurs).
 *  This callback is assumed to occur from an interrupt level when the
 *  device driver completes the fill operation.
 *
 *  NOTE 1: Allocating and filling a page is a two step process.
 *  up_allocpage() allocates the page, and up_fillpage() fills it with data
 *  from some non- volatile storage device.
 *  This distinction is made because up_allocpage()
 *  can probably be implemented in board-independent logic whereas
 *  up_fillpage() probably must be implemented as board-specific logic.
 *
 *  NOTE 2: The initial mapping of vpage will be read-able, write-able,
 *  but non-cacheable.  No special actions will be required of
 *  up_fillpage() in order to write into this allocated page.  If the
 *  virtual address maps to a text region, however, this function should
 *  remap the region so that is is read/execute only.  It should be made
 *  cache-able in any case.

 * Input Parameters:
 *   tcb - A reference to the task control block of the task that needs to
 *         have a page fill.  Architecture-specific logic can retrieve page
 *         fault information from the architecture-specific context
 *         information in this TCB to perform the fill.
 *   pg_callbck - The function to be called when the page fill is complete.
 *
 * Returned Value:
 *   This function will return zero (OK) if the page fill was successfully
 *   started (the result of the page fill is passed to the callback function
 *   as the result argument).  A negated errno value may be returned if an
 *   error occurs.  All errors, however, are fatal.
 *
 *   NOTE: -EBUSY has a special meaning. It is used internally to mean that
 *   the callback function has not executed.  Therefore, -EBUSY should
 *   never be provided in the result argument of pg_callback.
 *
 * Assumptions:
 *   - This function is called from the normal tasking context (but
 *     interrupts disabled).  The implementation must take whatever actions
 *     are necessary to assure that the operation is safe within this
 *     context.
 *   - Upon return, the caller will sleep waiting for the page fill callback
 *     to occur.  The callback function will perform the wakeup.
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING_BLOCKINGFILL

/* Version 1:  Supports blocking fill operations */

int up_fillpage(FAR struct tcb_s *tcb, FAR void *vpage)
{
	
	return OK;
	//return -ENOSYS;
}


#endif



#endif

