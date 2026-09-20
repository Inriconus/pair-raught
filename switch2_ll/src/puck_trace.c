/*
 * Breadcrumbs. See puck_trace.h for why this exists.
 */

#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/reboot.h>
#include <string.h>

#include "puck_trace.h"
#include "switch2_store.h"

#define TRACE_MAGIC 0x50554331u	/* "PUC1" */

/*
 * "RF came up" rides in the spare top bits of the persisted stage byte. Stages
 * only go to 12, so bits 6 and 7 are free, and this keeps the two-byte record.
 *
 * Bit 6 marks the record as this format. A record written by firmware from
 * before the flag existed has both bits clear, and there is no way to tell
 * "RF did not come up" from "this build could not say" without it. Reading
 * such a record as rf_up=false makes the very first boot after an update
 * announce a failure that never happened, so an untagged record suppresses
 * the warning instead of guessing.
 *
 * It has to be a flag rather than a comparison against TR_RF_UP. In Pro mode
 * the console starts its USB handshake within a second while the RF handover
 * waits PRO_RF_DELAY_S, so the high-water mark passes TR_PRO_HANDSHAKE before
 * TR_RF_UP is ever offered, and the lower number is then discarded. Asking
 * "stage < TR_RF_UP" of that record answers no on a boot where RF never came
 * up at all, which is precisely the boot worth warning about.
 */
#define TRACE_RF_UP_BIT  0x80u
#define TRACE_FORMAT_BIT 0x40u
#define TRACE_STAGE_MASK 0x3Fu

BUILD_ASSERT(TR_RF_LIVE <= TRACE_STAGE_MASK,
	     "trace stages have grown into the flag bits");

/*
 * Retained across a WARM reboot, because __noinit is not zeroed at startup.
 * This is where a fatal error leaves its note: writing flash from a fault
 * context is asking for trouble (NVS takes a mutex, and the faulting thread may
 * hold anything), so the handler only touches RAM and reboots. The next boot
 * finds it here and persists it properly.
 *
 * A power cut loses this, which is why the stage also goes to NVS as it
 * advances.
 */
struct trace_ram {
	uint32_t magic;
	uint8_t stage;
	uint8_t fault;		/* fatal reason + 1, 0 = none */
	uint8_t rf_up;		/* the radio reached RF at some point */
	uint8_t pad;
};

static __noinit struct trace_ram g_ram;

static uint8_t g_persisted;	/* highest stage written to flash this boot */

static const char *stage_name(uint8_t s)
{
	switch (s) {
	case TR_START:		 return "START (main entered)";
	case TR_STORE:		 return "STORE (NVS mounted)";
	case TR_USB_INIT:	 return "USB_INIT (identity chosen)";
	case TR_PRO_REG:	 return "PRO_REG (interface registered)";
	case TR_USB_UP:		 return "USB_UP (enumerating)";
	case TR_BT_UP:		 return "BT_UP";
	case TR_GATT:		 return "GATT registered";
	case TR_BOOTMODE:	 return "BOOTMODE decided";
	case TR_RF_UP:		 return "RF_UP (radio handed over)";
	case TR_PRO_HANDSHAKE:	 return "PRO_HANDSHAKE (console spoke)";
	case TR_PRO_STREAM:	 return "PRO_STREAM (0x30 selected)";
	case TR_RF_LIVE:	 return "RF_LIVE (controller answered)";
	default:		 return "(none)";
	}
}

void puck_trace_stage(uint8_t stage)
{
	bool new_rf_up;

	if (g_ram.magic != TRACE_MAGIC) {
		/* First run after a power cut: initialise rather than trust
		 * whatever the RAM happened to hold.
		 */
		g_ram.magic = TRACE_MAGIC;
		g_ram.fault = 0;
		g_ram.stage = 0;
		g_ram.rf_up = 0;
	}

	/*
	 * TR_RF_UP is a FLAG, not a rank, and it has to be read BEFORE the
	 * high-water test. In Pro mode it arrives after TR_PRO_HANDSHAKE, so
	 * the test below would drop it as old news and the flag would never be
	 * set in the one case it exists to record.
	 */
	new_rf_up = (stage == TR_RF_UP) && !g_ram.rf_up;

	/* Hot path: TR_RF_LIVE is noted on every RF reply, so make "nothing to
	 * do" a few comparisons and a return.
	 */
	if (!new_rf_up && stage <= g_ram.stage && stage <= g_persisted) {
		return;
	}

	if (new_rf_up) {
		g_ram.rf_up = 1;
	}
	if (stage > g_ram.stage) {
		g_ram.stage = stage;
	}
	if (stage > g_persisted) {
		g_persisted = stage;
	}

	/*
	 * Reached only on a new high-water mark or the first TR_RF_UP, so a
	 * running puck writes nothing. The stage written is the high-water one,
	 * not the stage just offered: TR_RF_UP can be lower than what is
	 * already recorded, and it must not walk the record backwards.
	 */
	switch2_store_save_trace(g_persisted | TRACE_FORMAT_BIT |
				 (g_ram.rf_up ? TRACE_RF_UP_BIT : 0),
				 g_ram.fault);
}

/*
 * The PREVIOUS boot's record, captured before this boot can overwrite it.
 *
 * This has to be a RAM copy taken at startup. The stored record is rewritten as
 * this boot advances through its own stages, so by the time anyone asks, NVS
 * holds THIS boot's progress and the interesting one. The boot that had no
 * console, in the Switch's USB port, is long gone. Losing exactly the record
 * the whole mechanism exists for.
 */
static uint8_t g_prev_stage;
static uint8_t g_prev_fault;
static bool g_prev_valid;
static bool g_prev_rf_up;
static bool g_prev_printed;

void puck_trace_boot_report(void)
{
	uint8_t s = 0, f = 0;

	g_prev_valid = switch2_store_load_trace(&s, &f);
	if (s & TRACE_FORMAT_BIT) {
		g_prev_rf_up = (s & TRACE_RF_UP_BIT) != 0;
		g_prev_stage = s & TRACE_STAGE_MASK;
	} else {
		/* Pre-flag record: the flag is not knowable, so claim RF came up
		 * and keep the warning quiet rather than crying wolf once after
		 * an update. Mask the stage anyway. One build wrote the RF bit
		 * without the format bit, and its record would otherwise name a
		 * stage that does not exist.
		 */
		g_prev_rf_up = true;
		g_prev_stage = s & TRACE_STAGE_MASK;
	}
	g_prev_fault = f;

	/*
	 * A fault noted in retained RAM belongs to the boot that just died, and
	 * is better evidence than the stored stage: persist it, and let it be
	 * what gets reported.
	 */
	if (g_ram.magic == TRACE_MAGIC && g_ram.fault) {
		g_prev_stage = g_ram.stage;
		g_prev_fault = g_ram.fault;
		g_prev_rf_up = g_ram.rf_up;
		g_prev_valid = true;
		switch2_store_save_trace(g_ram.stage | TRACE_FORMAT_BIT |
					 (g_ram.rf_up ? TRACE_RF_UP_BIT : 0),
					 g_ram.fault);
	}

	/*
	 * Everything describing THIS boot starts clean. g_ram survives a warm
	 * reboot, and the fault handler reboots, so leaving stage set carries
	 * the last healthy boot's high-water mark into a fault loop: the next
	 * boot then reports a crash "at RF_LIVE" for one that died at USB_INIT.
	 * The previous boot's value has already been read out above.
	 *
	 * Nothing is printed here: USB has not enumerated yet, so
	 * puck_trace_report() does the printing once the console exists.
	 */
	g_ram.fault = 0;
	g_ram.rf_up = 0;
	g_ram.stage = 0;
	g_persisted = 0;
	puck_trace_stage(TR_START);
}

/*
 * Print the previous boot's outcome, ONCE, from somewhere the console can
 * actually be written to. Safe to call repeatedly.
 */
void puck_trace_report(void)
{
	if (g_prev_printed) {
		return;
	}
	g_prev_printed = true;

	if (!g_prev_valid) {
		printk("trace: no record of a previous boot\n");
		return;
	}
	if (g_prev_fault) {
		printk("\n*** THE PREVIOUS BOOT FAULTED: reason=%u, reached %s ***\n",
		       (unsigned)(g_prev_fault - 1), stage_name(g_prev_stage));
		return;
	}
	printk("trace: previous boot reached %s\n", stage_name(g_prev_stage));
	if (g_prev_stage && !g_prev_rf_up) {
		printk("       (it never got the radio to RF: it hung or lost "
		       "power there)\n");
	}
}

void puck_trace_dump(void)
{
	printk("trace: this boot %s  |  previous boot %s%s\n",
	       stage_name(g_ram.stage),
	       g_prev_valid ? stage_name(g_prev_stage) : "(no record)",
	       g_prev_fault ? " after a FAULT" : "");
}
/*
 * Fatal errors: leave a note in retained RAM and reboot.
 *
 * Rebooting rather than halting matters for the product case. A puck in the
 * console's USB port with no console attached is otherwise just dead, and
 * indistinguishable from working. A reboot at least brings the USB device back,
 * and the note explains itself the next time a PC is attached.
 */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	ARG_UNUSED(esf);

	if (g_ram.magic != TRACE_MAGIC) {
		g_ram.magic = TRACE_MAGIC;
		g_ram.stage = 0;
	}
	g_ram.fault = (uint8_t)(reason + 1);

	printk("\n*** FATAL %u at stage %u: rebooting to leave a trace ***\n",
	       reason, g_ram.stage);

	sys_reboot(SYS_REBOOT_COLD);
	CODE_UNREACHABLE;
}
