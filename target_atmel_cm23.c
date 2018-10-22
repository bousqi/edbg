/*
 * Copyright (c) 2013-2017, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "target.h"
#include "edbg.h"
#include "dap.h"

/*- Definitions -------------------------------------------------------------*/
#define FLASH_ADDR             0
#define FLASH_ROW_SIZE         256
#define FLASH_PAGE_SIZE        64

#define USER_ROW_ADDR          0x00804000
#define USER_ROW_SIZE          256

#define DHCSR                  0xe000edf0
#define DEMCR                  0xe000edfc
#define AIRCR                  0xe000ed0c

#define DSU_CTRL_STATUS        0x41002100
#define DSU_DID                0x41002118

#define NVMCTRL_CTRLA          0x41005000
#define NVMCTRL_CTRLB          0x41005004
#define NVMCTRL_INTFLAG        0x41005014
#define NVMCTRL_STATUS         0x41005018
#define NVMCTRL_ADDR           0x4100501c
#define NVMCTRL_PARAM          0x41005024

#define NVMCTRL_CMD_ER         0xa502
#define NVMCTRL_CMD_WP         0xa504
// #define NVMCTRL_CMD_EAR        0xa505
// #define NVMCTRL_CMD_WAP        0xa506
// #define NVMCTRL_CMD_WL         0xa50f
// #define NVMCTRL_CMD_UR         0xa541
#define NVMCTRL_CMD_PBC        0xa544
// #define NVMCTRL_CMD_SSB        0xa545

#define DEVICE_ID_MASK         0xfffff0ff
#define DEVICE_REV_SHIFT       8
#define DEVICE_REV_MASK        0xf

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  uint32_t  dsu_did;
  char      *name;
  uint32_t  flash_size;
  uint32_t  n_pages;
} device_t;

/*- Variables ---------------------------------------------------------------*/
static device_t devices[] =
{
  // { 0x20830100, "SAM L10x14",  16*1024,  256 },
  // { 0x20830100, "SAM L10x15",  32*1024,  512 },
  // { 0x20830100, "SAM L10x16",  64*1024, 1024 },
  // { 0x20830100, "SAM L11x14",  16*1024,  256 },
  // { 0x20830100, "SAM L11x15",  32*1024,  512 },
  { 0x20830000, "SAM L11x16",  64*1024, 1024 },
  { 0, "", 0, 0 },
};

static device_t target_device;
static target_options_t target_options;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void target_select(target_options_t *options)
{
  uint32_t dsu_did, id, rev;

  // Stop the core
  dap_write_word(DHCSR, 0xa05f0003);
  dap_write_word(DEMCR, 0x00000001);
  dap_write_word(AIRCR, 0x05fa0004);

  dsu_did = dap_read_word(DSU_DID);
  id = dsu_did & DEVICE_ID_MASK;
  rev = (dsu_did >> DEVICE_REV_SHIFT) & DEVICE_REV_MASK;

  for (device_t *device = devices; device->dsu_did > 0; device++)
  {
    // verbose("Checking : %s (Rev %c) - 0x%08x vs 0x%08x\n", device->name, 'A' + rev, device->dsu_did, id);
    if (device->dsu_did == id)
    {
      verbose("Target: %s (Rev %c)\n", device->name, 'A' + rev);

      target_device = *device;
      target_options = *options;

      target_check_options(&target_options, device->flash_size,
          FLASH_ROW_SIZE, USER_ROW_SIZE);

      return;
    }
  }

  error_exit("unknown target device (DSU_DID = 0x%08x)", dsu_did);
}

//-----------------------------------------------------------------------------
static void target_deselect(void)
{
  dap_write_word(DEMCR, 0x00000000);
  dap_write_word(AIRCR, 0x05fa0004);

  target_free_options(&target_options);
}

//-----------------------------------------------------------------------------
static void target_erase(void)
{
  // dap_write_word(DSU_CTRL_STATUS, 0x00001f00); // Clear flags
  // dap_write_word(DSU_CTRL_STATUS, 0x00000010); // Chip erase
  // sleep_ms(100);
  // while (0 == (dap_read_word(DSU_CTRL_STATUS) & 0x00000100));

  uint32_t addr = FLASH_ADDR;
  uint32_t offs = 0;
  uint32_t number_of_rows;
  uint32_t size = target_device.flash_size;

  if (dap_read_word(DSU_CTRL_STATUS) & 0x00010000)
    error_exit("device is locked, perform a chip erase before programming");

  number_of_rows = (size + FLASH_ROW_SIZE - 1) / FLASH_ROW_SIZE;

  dap_write_word(NVMCTRL_CTRLB, 0); // Enable automatic write

  for (uint32_t row = 0; row < number_of_rows; row++)
  {
    dap_write_word(NVMCTRL_ADDR, addr >> 1);

    dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_ER); // Erase Row
    while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));

    addr += FLASH_ROW_SIZE;
    offs += FLASH_ROW_SIZE;

    verbose(".");
  }
}

//-----------------------------------------------------------------------------
static void target_lock(void)
{
  #warning DISABLED : NOT YET PORTED
  // dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_SSB); // Set Security Bit
}

//-----------------------------------------------------------------------------
static void target_program(void)
{
  uint32_t addr = FLASH_ADDR + target_options.offset;
  uint32_t offs = 0;
  uint32_t number_of_rows;
  uint8_t *buf = target_options.file_data;
  uint32_t size = target_options.file_size;

  if (dap_read_word(DSU_CTRL_STATUS) & 0x00010000)
    error_exit("device is locked, perform a chip erase before programming");

  number_of_rows = (size + FLASH_ROW_SIZE - 1) / FLASH_ROW_SIZE;

  dap_write_word(NVMCTRL_CTRLB, 0); // Enable automatic write

  for (uint32_t row = 0; row < number_of_rows; row++)
  {
    dap_write_word(NVMCTRL_ADDR, addr >> 1);

    #warning DISABLED : NOT YET PORTED
    // dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_UR); // Unlock Region
    // while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));

    dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_ER); // Erase Row
    while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));

    dap_write_block(addr, &buf[offs], FLASH_ROW_SIZE);

    addr += FLASH_ROW_SIZE;
    offs += FLASH_ROW_SIZE;

    verbose(".");
  }
}

//-----------------------------------------------------------------------------
static void target_verify(void)
{
  uint32_t addr = FLASH_ADDR + target_options.offset;
  uint32_t block_size;
  uint32_t offs = 0;
  uint8_t *bufb;
  uint8_t *bufa = target_options.file_data;
  uint32_t size = target_options.file_size;

  if (dap_read_word(DSU_CTRL_STATUS) & 0x00010000)
    error_exit("device is locked, unable to verify");

  bufb = buf_alloc(FLASH_ROW_SIZE);

  while (size)
  {
    dap_read_block(addr, bufb, FLASH_ROW_SIZE);

    block_size = (size > FLASH_ROW_SIZE) ? FLASH_ROW_SIZE : size;

    for (int i = 0; i < (int)block_size; i++)
    {
      if (bufa[offs + i] != bufb[i])
      {
        verbose("\nat address 0x%x expected 0x%02x, read 0x%02x\n",
            addr + i, bufa[offs + i], bufb[i]);
        buf_free(bufb);
        error_exit("verification failed");
      }
    }

    addr += FLASH_ROW_SIZE;
    offs += FLASH_ROW_SIZE;
    size -= block_size;

    verbose(".");
  }

  buf_free(bufb);
}

//-----------------------------------------------------------------------------
static void target_read(void)
{
  uint32_t addr = FLASH_ADDR + target_options.offset;
  uint32_t offs = 0;
  uint8_t *buf = target_options.file_data;
  uint32_t size = target_options.size;

  if (dap_read_word(DSU_CTRL_STATUS) & 0x00010000)
    error_exit("device is locked, unable to read");

  while (size)
  {
    dap_read_block(addr, &buf[offs], FLASH_ROW_SIZE);

    addr += FLASH_ROW_SIZE;
    offs += FLASH_ROW_SIZE;
    size -= FLASH_ROW_SIZE;

    verbose(".");
  }

  save_file(target_options.name, buf, target_options.size);
}


//-----------------------------------------------------------------------------
static void target_fuse(void)
{
  uint8_t buf[USER_ROW_SIZE];
  bool read_all = (-1 == target_options.fuse_start);
  int size = (target_options.fuse_size < USER_ROW_SIZE) ?
      target_options.fuse_size : USER_ROW_SIZE;

  dap_read_block(USER_ROW_ADDR, buf, USER_ROW_SIZE);

  if (target_options.fuse_read)
  {
    if (target_options.fuse_name)
    {
      save_file(target_options.fuse_name, buf, USER_ROW_SIZE);
    }
    else if (read_all)
    {
      message("Fuses (user row): ");

      for (int i = 0; i < USER_ROW_SIZE; i++)
        message("%02x ", buf[i]);

      message("\n");
    }
    else
    {
      uint32_t value = extract_value(buf, target_options.fuse_start,
          target_options.fuse_end);

      message("Fuses: 0x%x (%d)\n", value, value);
    }
  }

  if (target_options.fuse_write)
  {
    if (target_options.fuse_name)
    {
      for (int i = 0; i < size; i++)
        buf[i] = target_options.fuse_data[i];
    }
    else
    {
      apply_value(buf, target_options.fuse_value, target_options.fuse_start,
          target_options.fuse_end);
    }

    dap_write_word(NVMCTRL_CTRLB, 0);
    dap_write_word(NVMCTRL_ADDR, USER_ROW_ADDR >> 1);
    #warning DISABLED : NOT YET PORTED
    // dap_write_word(NVMCTRL_CTRLA, NVMCTRL_CMD_EAR);
    // while (0 == (dap_read_word(NVMCTRL_INTFLAG) & 1));

    dap_write_block(USER_ROW_ADDR, buf, USER_ROW_SIZE);
  }

  if (target_options.fuse_verify)
  {
    dap_read_block(USER_ROW_ADDR, buf, USER_ROW_SIZE);

    if (target_options.fuse_name)
    {
      for (int i = 0; i < size; i++)
      {
        if (target_options.fuse_data[i] != buf[i])
        {
          message("fuse byte %d expected 0x%02x, got 0x%02x", i,
              target_options.fuse_data[i], buf[i]);
          error_exit("fuse verification failed");
        }
      }
    }
    else if (read_all)
    {
      error_exit("please specify fuse bit range for verification");
    }
    else
    {
      uint32_t value = extract_value(buf, target_options.fuse_start,
          target_options.fuse_end);

      if (target_options.fuse_value != value)
      {
        error_exit("fuse verification failed: expected 0x%x (%u), got 0x%x (%u)",
            target_options.fuse_value, target_options.fuse_value, value, value);
      }
    }
  }
}

//-----------------------------------------------------------------------------
target_ops_t target_atmel_cm23_ops =
{
  .select   = target_select,
  .deselect = target_deselect,
  .erase    = target_erase,
  .lock     = target_lock,
  .program  = target_program,
  .verify   = target_verify,
  .read     = target_read,
  .fuse     = target_fuse,
};

