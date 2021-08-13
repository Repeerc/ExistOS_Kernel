/****************************************************************************
 * net/local/local_sendpacket.c
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

#include <sys/types.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include "devif/devif.h"

#include "local/local.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOCAL_PREAMBLE_SIZE 8

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_preamble[LOCAL_PREAMBLE_SIZE] =
{
  LOCAL_SYNC_BYTE, LOCAL_SYNC_BYTE, LOCAL_SYNC_BYTE, LOCAL_SYNC_BYTE,
  LOCAL_SYNC_BYTE, LOCAL_SYNC_BYTE, LOCAL_SYNC_BYTE, LOCAL_END_BYTE
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_fifo_write
 *
 * Description:
 *   Write a data on the write-only FIFO.
 *
 * Input Parameters:
 *   filep    File structure of write-only FIFO.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static int local_fifo_write(FAR struct file *filep, FAR const uint8_t *buf,
                            size_t len)
{
  ssize_t nwritten;

  while (len > 0)
    {
      nwritten = file_write(filep, buf, len);
      if (nwritten < 0)
        {
          if (nwritten != -EINTR)
            {
              nerr("ERROR: file_write failed: %zd\n", nwritten);
              return (int)nwritten;
            }

          ninfo("Ignoring signal\n");
        }
      else
        {
          DEBUGASSERT(nwritten > 0 && nwritten <= len);
          len -= nwritten;
          buf += nwritten;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_send_packet
 *
 * Description:
 *   Send a packet on the write-only FIFO.
 *
 * Input Parameters:
 *   filep    File structure of write-only FIFO.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   Packet length is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int local_send_packet(FAR struct file *filep, FAR const struct iovec *buf,
                      size_t len)
{
  FAR const struct iovec *iov;
  FAR const struct iovec *end;
  uint16_t len16;
  int ret;

  /* Send the packet preamble */

  ret = local_fifo_write(filep, g_preamble, LOCAL_PREAMBLE_SIZE);
  if (ret == OK)
    {
      /* Send the packet length */

      end = buf + len;
      for (len16 = 0, iov = buf; iov != end; iov++)
        {
          len16 += iov->iov_len;
        }

      ret = local_fifo_write(filep, (FAR const uint8_t *)&len16,
                             sizeof(uint16_t));
      if (ret == OK)
        {
          /* Send the packet data */

          for (len16 = 0, iov = buf; iov != end; iov++)
            {
              ret = local_fifo_write(filep, iov->iov_base, iov->iov_len);
              if (ret < 0)
                break;
              else
                len16 += iov->iov_len;
            }

          if (ret == OK)
            ret = len16;
        }
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
