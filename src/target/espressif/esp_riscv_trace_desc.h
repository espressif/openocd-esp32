/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   RISC-V trace encoder control for Espressif chips                      *
 *   Copyright (C) 2026 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_RISCV_TRACE_DESC_H
#define OPENOCD_TARGET_ESP_RISCV_TRACE_DESC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <target/target.h>

#define ESP_RISCV_TRACE_MAX_CORES      2

/*
 * Target objects, found through the debug stubs table.
 *   RVOC descriptor -> per-core stream state (banks, seq, ack_seq)
 *                   -> RVTS snapshot -> per-core buffer descriptors
 *
 * RVOC is the RISC-V trace OpenOCD control descriptor. It holds the stub start and stop
 * functions and the per-core capture state.
 * RVTS is the RISC-V trace snapshot. It describes the trace buffers and is shared with coredump.
 * Both names are the ASCII magic values of the objects.
 *
 * The structs below mirror the ESP-IDF ones field by field. They are filled from raw target
 * bytes at the *_OFF_* offsets, so the host layout does not have to match the target.
 */

/* RISC-V trace snapshot "RVTS". Must be in sync with ESP-IDF esp_riscv_trace_snapshot.h */
#define RVTS_MAGIC                     0x53545652
#define RVTS_ABI_MAJOR                 1
#define RVTS_DESC_SIZE                 32
#define RVTS_CORE_DESC_SIZE            36
#define RVTS_READ_RETRIES              4
#define RVTS_SHA256_SIZE               32
#define RVTS_STATE_CAPTURING           2
#define RVTS_STATE_STOPPED             3
#define RVTS_STATE_FROZEN              4
#define RVTS_RESYNC_DISABLED           1
#define RVTS_RESYNC_PACKET             2
#define RVTS_RESYNC_CYCLE              3
#define RVTS_WRITE_SEQ_BUSY            1
#define RVTS_PACKET_FORMAT_PT10        100
#define RVTS_PACKET_FORMAT_ET20        200

/* esp_riscv_trace_snapshot_desc_t offsets */
#define RVTS_OFF_MAGIC                 0
#define RVTS_OFF_WRITE_SEQ             4
#define RVTS_OFF_CORES_ADDR            8
#define RVTS_OFF_APP_ELF_SHA256_ADDR   12
#define RVTS_OFF_ENCODER_PARAMS_ADDR   16
#define RVTS_OFF_TARGET_ID             20
#define RVTS_OFF_CHIP_REVISION         22
#define RVTS_OFF_ABI_MAJOR             24
#define RVTS_OFF_ABI_MINOR             25
#define RVTS_OFF_SNAPSHOT_DESC_SIZE    26
#define RVTS_OFF_CORE_DESC_SIZE        27
#define RVTS_OFF_CORE_COUNT            28
#define RVTS_OFF_CAPTURE_REASON        29
#define RVTS_OFF_APP_ELF_SHA256_SIZE   30
#define RVTS_OFF_ENCODER_PARAMS_SIZE   31

/* esp_riscv_trace_snapshot_core_desc_t offsets */
#define RVTS_CORE_OFF_BUFFER_ADDR      0
#define RVTS_CORE_OFF_CAPACITY         4
#define RVTS_CORE_OFF_HEAD_OFFSET      8
#define RVTS_CORE_OFF_RESYNC_THRESHOLD 12
#define RVTS_CORE_OFF_FIFO_STATUS_RAW  16
#define RVTS_CORE_OFF_INTR_STATUS_RAW  20
#define RVTS_CORE_OFF_CORE_ID          24
#define RVTS_CORE_OFF_STATE            25
#define RVTS_CORE_OFF_MEMORY_MODE      26
#define RVTS_CORE_OFF_PACKET_FORMAT    27
#define RVTS_CORE_OFF_ADDRESS_MODE     28
#define RVTS_CORE_OFF_RESYNC_MODE      29
#define RVTS_CORE_OFF_RESERVED0        30
#define RVTS_CORE_OFF_HEAD_VALID       31
#define RVTS_CORE_OFF_FIFO_EMPTY       32
#define RVTS_CORE_OFF_MEMORY_FULL      33
#define RVTS_CORE_OFF_FIFO_OVERFLOW    34
#define RVTS_CORE_OFF_RESERVED1        35

/* esp_riscv_trace_encoder_params_t, copied raw into the trace file */
#define RVTS_PARAMS_SIZE               28
#define RVTS_PARAMS_VERSION            1
#define RVTS_PARAMS_OFF_VERSION        0
#define RVTS_PARAMS_OFF_SIZE           1

/* RISC-V trace OpenOCD control "RVOC". Must be in sync with ESP-IDF esp_riscv_trace_ocd.h */
#define RVOC_MAGIC                     0x434F5652
#define RVOC_ABI_VERSION               1
#define RVOC_HDR_SIZE                  20
#define RVOC_CORE_SIZE                 40
#define RVOC_BANK_COUNT                2

/* esp_riscv_trace_ocd_hdr_t offsets. cores[] follows the header. */
#define RVOC_HDR_OFF_MAGIC             0
#define RVOC_HDR_OFF_ABI_VERSION       4
#define RVOC_HDR_OFF_CORE_COUNT        5
#define RVOC_HDR_OFF_SNAPSHOT          8
#define RVOC_HDR_OFF_START             12
#define RVOC_HDR_OFF_STOP              16

/* esp_riscv_trace_ocd_core_t offsets */
#define RVOC_CORE_OFF_BANK_ADDR        0
#define RVOC_CORE_OFF_BANK_SIZE        8
#define RVOC_CORE_OFF_BANK_LEN         12
#define RVOC_CORE_OFF_BANK_SEQ         20
#define RVOC_CORE_OFF_SEQ              28
#define RVOC_CORE_OFF_ACK_SEQ          32
#define RVOC_CORE_OFF_MODE             36
#define RVOC_CORE_OFF_STATE            37
#define RVOC_CORE_OFF_ERROR            38
#define RVOC_CORE_OFF_ACTIVE_BANK      39

/* esp_riscv_trace_ocd_mode_t */
#define RVOC_MODE_STREAM               1
#define RVOC_MODE_LOOP                 2

/* esp_err_t values returned by the firmware start/stop functions */
#define RVOC_ERR_OK                    0
#define RVOC_ERR_INVALID_STATE         0x103
#define RVOC_ERR_TIMEOUT               0x107
#define RVOC_ERR_NOT_FINISHED          0x10C

/* Mirror of esp_riscv_trace_snapshot_core_desc_t. Core configuration and capture state. */
struct esp_riscv_trace_snapshot_core_desc {
	uint32_t buffer_addr;      /* Target address of the trace buffer */
	uint32_t capacity;         /* Trace-buffer size in bytes */
	uint32_t head_offset;      /* Next hardware write offset */
	uint32_t resync_threshold; /* Resync interval reported by hardware. Zero when not applicable */
	uint32_t fifo_status_raw;  /* Unmodified hardware status register */
	uint32_t intr_status_raw;  /* Unmodified hardware interrupt status */
	uint8_t core_id;           /* Traced hart ID */
	uint8_t state;             /* UNAVAILABLE / READY / CAPTURING / STOPPED / FROZEN */
	uint8_t memory_mode;       /* UNKNOWN / LINEAR / LOOP */
	uint8_t packet_format;     /* 100 = PT10, 200 = ET20 */
	uint8_t address_mode;      /* UNKNOWN / DELTA / FULL */
	uint8_t resync_mode;       /* UNKNOWN / DISABLED / PACKET / CYCLE */
	uint8_t reserved0;         /* Unused. Stays zero */
	uint8_t head_valid;        /* 1 when head_offset is the hardware write position */
	uint8_t fifo_empty;        /* 1 when the encoder FIFO was empty after stop or freeze */
	uint8_t memory_full;       /* 1 when the memory-full interrupt was set */
	uint8_t fifo_overflow;     /* 1 when the FIFO-overflow interrupt was set */
	uint8_t reserved1;         /* Keeps the descriptor 4-byte aligned */
};

/* Mirror of esp_riscv_trace_snapshot_desc_t. Snapshot root descriptor. */
struct esp_riscv_trace_snapshot_desc {
	uint32_t magic;               /* RVTS_MAGIC */
	uint32_t write_seq;           /* Odd while the snapshot is changing */
	uint32_t cores_addr;          /* Target address of the core array */
	uint32_t app_elf_sha256_addr; /* Target address of the raw ELF SHA-256 bytes */
	uint32_t encoder_params_addr; /* Target address of the shared encoder-parameter block */
	uint16_t target_id;           /* ESP image chip ID */
	uint16_t chip_revision;       /* Major times 100 plus minor */
	uint8_t abi_major;            /* 1 */
	uint8_t abi_minor;            /* 0 */
	uint8_t snapshot_desc_size;   /* 32 */
	uint8_t core_desc_size;       /* 36 */
	uint8_t core_count;           /* Number of contiguous core entries */
	uint8_t capture_reason;       /* UNKNOWN / EXPLICIT_STOP / PANIC */
	uint8_t app_elf_sha256_size;  /* Zero or 32 */
	uint8_t encoder_params_size;  /* Encoder-parameter block size in bytes */
};

/* Snapshot bytes exactly as read from the target, written to the trace file. */
struct esp_riscv_trace_snapshot_raw {
	uint8_t desc[RVTS_DESC_SIZE];
	uint8_t cores[ESP_RISCV_TRACE_MAX_CORES][RVTS_CORE_DESC_SIZE];
};

/* Mirror of esp_riscv_trace_ocd_core_t. Firmware writes every field except ack_seq. */
struct esp_riscv_trace_ocd_core {
	uint32_t bank_addr[RVOC_BANK_COUNT]; /* Target address of each ping-pong bank */
	uint32_t bank_size;                  /* Usable bytes per bank */
	uint32_t bank_len[RVOC_BANK_COUNT];  /* Valid bytes of each bank's last completion */
	uint32_t bank_seq[RVOC_BANK_COUNT];  /* seq when each bank last completed. 0 = never */
	uint32_t seq;                        /* Last completed seq */
	uint32_t ack_seq;                    /* Highest seq consumed by OpenOCD */
	uint8_t mode;                        /* NONE / STREAM / LOOP */
	uint8_t state;                       /* IDLE / STREAMING / LOOPING / STOPPED / OVERRUN / ERROR */
	uint8_t error;                       /* NONE / OVERRUN / FIFO_OVF / HEAD_INVALID / FIFO_TIMEOUT */
	uint8_t active_bank;                 /* Bank the encoder writes now */
};

/* Mirror of esp_riscv_trace_ocd_hdr_t. abi_version is the layout contract. */
struct esp_riscv_trace_ocd_hdr {
	uint32_t magic;      /* RVOC_MAGIC */
	uint8_t abi_version; /* RVOC_ABI_VERSION */
	uint8_t core_count;  /* Number of entries in cores[] */
	uint32_t snapshot;   /* Snapshot root (RVTS) */
	uint32_t start;      /* Called by OpenOCD on a halted core */
	uint32_t stop;       /* Called by OpenOCD on a halted core */
};

/* Mirror of esp_riscv_trace_ocd_t. Root descriptor, registered at ESP_DBG_STUB_ENTRY_RISCV_TRACE. */
struct esp_riscv_trace_ocd {
	struct esp_riscv_trace_ocd_hdr hdr;
	struct esp_riscv_trace_ocd_core cores[ESP_RISCV_TRACE_MAX_CORES];
};

/* Descriptors read from the target for one command. */
struct esp_riscv_trace_target_state {
	uint32_t ocd_addr;                       /* Address of the RVOC descriptor */
	struct esp_riscv_trace_ocd ocd;
	struct esp_riscv_trace_snapshot_desc snap;
	struct esp_riscv_trace_snapshot_core_desc cores[ESP_RISCV_TRACE_MAX_CORES];
	struct esp_riscv_trace_snapshot_raw raw; /* snap and cores as raw target bytes */
};

int esp_riscv_trace_read_target_state(struct target *target, struct esp_riscv_trace_target_state *state);
int esp_riscv_trace_read_app_sha256(struct target *target, const struct esp_riscv_trace_snapshot_desc *snap,
	uint8_t *sha);
bool esp_riscv_trace_core_is_stopped(const struct esp_riscv_trace_snapshot_core_desc *core);

#endif	/* OPENOCD_TARGET_ESP_RISCV_TRACE_DESC_H */
