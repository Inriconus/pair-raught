// BLE Switch 2 wake proof of concept. REFERENCE ONLY, NOT BUILT, NOT THE CURRENT STACK.
//
// This is where the Switch 2 BLE work started, and it is the code the Zephyr firmware in this repo was
// ported FROM. It is Arduino plus Adafruit Bluefruit, which wraps Nordic's SoftDevice S140. Nothing here
// compiles as part of Pair-Raught and nothing here should be revived.
//
// Two things it records, and both are the reason it is kept:
//
// It began as an attempt to run the idea inside OpenPuck itself, as one more USB personality of that
// firmware. On real hardware that soft-reset within about 1.5 s of Bluefruit.begin() succeeding, wherever
// the call was placed, while a from-scratch sketch containing none of OpenPuck's other modules ran
// stably. None of that code executed in BLE mode, so the cause was structural, from linking those modules
// in at all. It was never identified, and the work continued as its own standalone image.
//
// It is also the stack that could never finish the job. The console opens controller links at a 5 ms
// interval, below what the Bluetooth spec allows, and the S140 is closed source so it silently refuses
// and reports nothing at any layer. That discovery is what moved the project to Zephyr's software
// controller, whose link layer can be patched to accept it. Everything above the link layer in this file
// worked; the link itself could not be held.
//
// So read it the way the project rule says: when doing a step in the Zephyr firmware, read the
// SoftDevice-era finding for that same step here first and carry the REASONS across. Console behavior
// measured here is still true. API codes, handle numbering and stack mechanics are not.
//
// Some comments below describe states later superseded, including a period where the console disconnected
// during service discovery every time, and an earlier model in which the wake was thought to be a Home
// button press inside the input report stream. Bluetooth addresses have been replaced with synthetic
// values.
//
// Build/flash exactly like OpenPuck (same board, same required TinyUSB flags):
//   arduino-cli compile -b adafruit:nrf52:feather52840 \
//     --build-property "build.extra_flags=-DNRF52840_XXAA {build.flags.usb} -DCFG_TUD_HID=6 -DCFG_TUD_TASK_QUEUE_SZ=512 -DCFG_TUD_VENDOR_TX_BUFSIZE=256" \
//     BleSwitch2Poc
// Flash via the bootloader (touch-1200 on an already-running CDC port, or double-tap reset):
//   arduino-cli upload -b adafruit:nrf52:feather52840 -p <bootloader COM port> BleSwitch2Poc
//
// Serial console (115200 baud, CDC always on): 'H' arms a one-shot simulated Home-button press (once
// connected + subscribed), 'L' reprints the persisted handshake-command log, 'E' reprints the persisted
// connect/disconnect/CCCD-subscribe event log, 'S' prints current connection status, 'W' manually fires
// the wake-sequence ATT-client replay against the current connection (NOT automatic on connect -- see
// bleConnectCb()/sendWakeSequence() for why), 'D' discovers the SWITCH's own live GATT table (all primary
// services + characteristics) over the current connection -- ground truth for whatever handles it actually
// has THIS session, instead of trusting the one hardcoded wake capture; 'O' flips the byte order of the
// derived link-layer key (see deriveLinkKey()). All logs persist across a crash/reset (separate flash
// partition from the app image, survives a DFU reflash too) so a silent disconnect or crash mid-handshake
// is still diagnosable on the next boot.
//
// UPDATE 2026-08-06 (capture review pass) -- two corrections that invalidate earlier reasoning here:
//   * The three docs/captures/*_DECRYPTED_*.pcap files were NEVER decrypted. None contains an LL_ENC_REQ
//     (so decryption is impossible for them at any key) and no frame is flagged encrypted; everything ever
//     read out of them was pre-encryption plaintext. joycon2_reconnect_undecrypted.pcap -- the one file
//     nobody looked at -- is the only one with a full LL_ENC_REQ, and it now decrypts cleanly (315/315
//     packets, zero MIC failures). See docs/captures/ble_decrypt.js and joycon2_reconnect_DECRYPTED.txt.
//     That capture also yielded the real link-layer key derivation -- see deriveLinkKey() below.
//   * The "proactive unprompted 8-notification announcement" model is wrong. The real protocol is a strict
//     request->response ping-pong: the Switch writes a command to chrUnk65a7 and the controller answers
//     with exactly one notification on COMMAND_RESPONSE. All 8 notifications are RESPONSES. The two newer
//     captures only looked unprompted because the Switch's writes are NOT DECRYPTED in them.
//     PROVEN 2026-08-09 (audit BUG-6), so treat this as settled and stop re-arguing it. In all three
//     "*_DECRYPTED_*" captures every ATT frame has nordic_ble.direction=False -- only the CONTROLLER's
//     side is readable. NOTE those files are MISNAMED: per ble_decrypt.js they contain no LL_ENC_REQ and
//     were never decrypted at all; every readable byte in them is pre-encryption plaintext. So the
//     one-sidedness is a CAPTURE limitation (the sniffer did not recover the central's payloads), not a
//     decryption one. The console's packets are present but undissectable, interleaving exactly with the
//     responses (frames 3915/3917/3919/3921 alternate with 3916/3918/3920/3922). And the responses are
//     self-proving: Exchange MTU Response (0x03), Read By Type Response (0x09) and Write Response (0x13)
//     all appear with ZERO instances of their mandatory requests (0x02/0x08/0x12). ATT forbids a response
//     without a request, so the requests were on the air and simply were not recorded.
//       tshark -r <cap> -Y btatt -T fields -e frame.number -e nordic_ble.direction -e btatt.opcode
//     The "escalation the Switch only does to us" is the protocol working correctly -- and the ~30ms
//     notification spacing is just the connection interval, not controller-side pacing.
//     sendProactiveAnnouncement() ('P') is therefore still useful as a manual test, but the real fix is to
//     answer each Switch write individually; that restructuring is NOT done yet.
//
// Protocol constants (UUIDs, command framing, the fixed pairing LTKs, memory addresses) are copied
// byte-for-byte from the cached trevlars/switch2-controllers-linux reference (MIT-licensed; that project
// emulates the BLE *central*, a PC connecting to a real controller -- every role here is inverted to the
// *peripheral*, since a Switch connects to us). The advertising payload is NOT inferred from that repo --
// it's ground truth, captured live via nRF Connect's raw-bytes view off an actual Switch 2 Joy-Con sitting
// in sync/pairing mode. See the comment above the manufacturer-data block in bleInitOnce() for the exact
// captured bytes and how our payload maps onto them.
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

Adafruit_USBD_WebUSB usb_web;
static uint8_t g_usbCfgDesc[512];

static void armWatchdog()
{
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |
			  (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = 8UL * 32768UL - 1;
	NRF_WDT->RREN = WDT_RREN_RR0_Msk;
	NRF_WDT->TASKS_START = 1;
}

// ---- stage markers (persisted) -- unambiguous proof of exactly how far a boot attempt got, immune to
// live-CDC-print timing races (a print firing microseconds before a crash/disconnect may never flush).
#define STAGE_ENTERED 1
#define STAGE_SDCHECK_DONE 2
#define STAGE_CALLING_BEGIN 3
#define STAGE_BEGIN_RETURNED 4
#define STAGE_GATT_DONE 5
#define STAGE_ADV_STARTED 6
#define STAGE_INIT_COMPLETE 8
#define STAGE_ALIVE 9
static const char *const STAGE_STR[] = { "NONE",	 "ENTERED",	"SDCHECK_DONE",
					 "CALLING_BEGIN", "BEGIN_RETURNED", "GATT_DONE",
					 "ADV_STARTED",	 "7",		"INIT_COMPLETE",
					 "ALIVE" };

// QUIET MODE (2026-08-14). Silences the high-frequency serial chatter -- the ALIVE heartbeat, the
// per-second report head, and the 2 s mtu/phy line. Adafruit's USB CDC BLOCKS on write when a host holds
// the port open and the FIFO fills, and discards instantly when it does not; the stall detector sees
// 84-85 ms blocks inside loop() with no delay() and no flash write on that path, which is what delayed
// our answer to the console's 0x10/0x01 by 419 ms and got us hung up on. Toggle with 'Q'... no, with '`'.
static bool g_quiet = true;
// Defined further down with the other filesystem helpers; markStage() below is the earliest caller.
static bool writeFileInPlace(const char *path, const uint8_t *data, uint16_t len);

static void markStage(uint8_t stage, uint8_t result = 0)
{
	// QUIET MODE gates the ALIVE heartbeat -- it is by far the highest-frequency print in the firmware.
	// See g_quiet: Serial.printf BLOCKS when a host holds the port open and the CDC FIFO fills, and the
	// stall detector measures 84-85 ms blocks inside loop() with no delay() and no flash write on that
	// path. 419 ms of that landed between the console asking 0x10/0x01 and us reading the question.
	if (!(g_quiet && stage == STAGE_ALIVE))
		Serial.printf("# LEAN stage -> %s (result=%u) t=%lu ms\n",
			      stage < 10 ? STAGE_STR[stage] : "?", result,
			      (unsigned long)millis());
	// DO NOT persist the ALIVE heartbeat. bleHeartbeat() fires it every 100ms for the first 25.5s, and
	// each call was doing a full remove()+open()+write()+close() -- ~255 file create/delete cycles per
	// boot, on a LittleFS partition of a few tens of KB. Across a session's reflashes that is thousands of
	// cycles, and it kills the filesystem: 2026-08-07 it died twice, taking the bond store with it (bond
	// file open() fails -> SEC_INFO_REQUEST answered with NULL keys -> encryption rejected), plus the
	// address-rotation counter. It cost hours before the cause was spotted, because the failure is silent
	// and looks like a protocol problem. The heartbeat's real value is the serial print above; the
	// persisted copy only ever mattered for the boot-attempt stages, which are rare and still written.
	if (stage == STAGE_ALIVE)
		return;
	uint8_t rec[3] = { 0x42, stage, result };
	// In-place, not remove()+create() -- same reasoning as the bond file; see writeFileInPlace().
	writeFileInPlace("/leanstage.bin", rec, sizeof rec);
}

static void printLastStage()
{
	File f(InternalFS);
	uint8_t rec[3];
	if (f.open("/leanstage.bin", FILE_O_READ)) {
		if (f.read(rec, sizeof rec) == sizeof rec && rec[0] == 0x42) {
			Serial.printf(
				"# LEAN stage (from last boot attempt): %s result=%u\n",
				rec[1] < 10 ? STAGE_STR[rec[1]] : "?", rec[2]);
		}
		f.close();
	}
}

// ---- diagnostic: log every handshake command seen, persisted so a crash/disconnect mid-handshake still
// tells us exactly how far the real Switch got. Ring of the last 16 (cmd,subcmd) pairs seen.
#define CMD_LOG_MAX 16
static void logCmd(uint8_t cmd, uint8_t subcmd)
{
	// Gated: the console repeats the whole handshake every ~2 s in the loop, so this fires ~10 times a
	// cycle in the seconds before the arm -- and Serial blocks when a host is draining the CDC.
	if (!g_quiet)
		Serial.printf("# LEAN cmd=0x%02X sub=0x%02X t=%lu ms\n", cmd, subcmd,
			      (unsigned long)millis());
	// NO FLASH WRITE. This used to do remove()+open()+write()+close() for EVERY handshake command --
	// ten-plus per connection, dozens of connections a session, thousands of file create/delete cycles on
	// a LittleFS partition of a few tens of KB. Same disease that markStage()'s ALIVE ticks had: it kills
	// the filesystem, and when the filesystem dies the BOND FILE cannot be written, so the PAIR/LTK2
	// challenge cannot be answered and the Switch drops us. Killed the session twice on 2026-08-07 before
	// the cause was spotted. The serial print above is the diagnostic; the persisted ring never earned its
	// keep, and it cost far more than it gave.
}

// Generic persisted connection-lifecycle event log (connect/disconnect/CCCD subscribe) -- same ring
// pattern as the cmd log. evt: 1=connect 2=disconnect 3=cccd-input 4=cccd-cmdresp. val is event-specific
// (disconnect reason, or the CCCD value written).
#define EVT_LOG_MAX 12
static void logEvent(uint8_t evt, uint16_t val)
{
	// NO FLASH WRITE -- see logCmd() above. Connect/disconnect/CCCD events fire several times per
	// connection and the persisted copy is not worth the wear on the partition the bond key lives on.
	(void)evt;
	(void)val;
}

static void printEvtLog()
{
	// PERSISTENCE IS DISABLED. logEvent() is a no-op (the flash writes were wearing out the partition
	// the bond key lives on), so these files are never written and this always found nothing -- which is
	// indistinguishable from "no events occurred" and has already been read as evidence (audit BUG-2).
	Serial.println("# LEAN event log: PERSISTENCE DISABLED -- use the live serial output instead");
	if (1)
		return;
	static const char *const EVT_STR[] = { "?", "CONNECT", "DISCONNECT", "CCCD_INPUT",
					       "CCCD_CMDRESP", "PHY_REQ_OK", "MTU_REQ_OK" };
	bool any = false;
	for (uint8_t i = 0; i < EVT_LOG_MAX; i++) {
		char path[24];
		snprintf(path, sizeof path, "/evtlog%u.bin", (unsigned)i);
		File f(InternalFS);
		uint8_t rec[4];
		if (f.open(path, FILE_O_READ)) {
			if (f.read(rec, sizeof rec) == sizeof rec && rec[0] == 0x45) {
				if (!any) {
					Serial.println(
						"# LEAN event log (from last boot attempt, order not guaranteed):");
					any = true;
				}
				uint16_t val = (uint16_t)rec[2] | ((uint16_t)rec[3] << 8);
				Serial.printf("#   %s val=0x%04X\n",
					      rec[1] < 7 ? EVT_STR[rec[1]] : "?", val);
			}
			f.close();
		}
	}
}

static void printCmdLog()
{
	// PERSISTENCE IS DISABLED -- see printEvtLog(). An empty log here does NOT mean no commands arrived
	// (audit BUG-2).
	Serial.println("# LEAN cmd log: PERSISTENCE DISABLED -- use the live serial output instead");
	if (1)
		return;
	bool any = false;
	for (uint8_t i = 0; i < CMD_LOG_MAX; i++) {
		char path[24];
		snprintf(path, sizeof path, "/cmdlog%u.bin", (unsigned)i);
		File f(InternalFS);
		uint8_t rec[3];
		if (f.open(path, FILE_O_READ)) {
			if (f.read(rec, sizeof rec) == sizeof rec && rec[0] == 0x44) {
				if (!any) {
					Serial.println(
						"# LEAN cmd log (from last boot attempt, order not guaranteed):");
					any = true;
				}
				Serial.printf("#   cmd=0x%02X sub=0x%02X\n",
					      rec[1], rec[2]);
			}
			f.close();
		}
	}
}

// Overwrite a fixed-size file IN PLACE, without the remove()+create() cycle.
//
// This exists because repeated remove+create is what has killed this board's LittleFS over and over --
// first via markStage()'s heartbeat, then logCmd()/logEvent(), and most recently via the bond file, which
// is rewritten on EVERY connection (the derived link key genuinely changes each time, so the write itself
// cannot be skipped). When the filesystem dies the bond file stops persisting, the PAIR/LTK2 challenge
// cannot be answered, and the console drops us at LTK2 -- a failure that looks like a protocol problem and
// has now silently invalidated three separate conclusions in this project. Opening without remove() reuses
// the existing blocks, which is far gentler; every caller here writes a constant-length record, so an
// in-place overwrite is exact with no stale tail.
//
// Returns false on failure so the caller can self-heal rather than carry on with a dead store.
static bool writeFileInPlace(const char *path, const uint8_t *data, uint16_t len)
{
	// SKIP THE WRITE IF THE BYTES ARE ALREADY THERE. A flash page erase on the nRF52840 takes ~85 ms and
	// BLOCKS, and that is precisely the `*** MAIN LOOP STALLED 84 ms ***` seen twice per connection.
	//
	// Measured 2026-08-14, and it is the disconnect: the console arms the report stream, subscribes, then
	// asks 0x10/0x01 IMMEDIATELY. We were mid-stall, so we did not read its question for 419 ms -- by
	// which time it had given up and sent LL_TERMINATE (reason=0x13), and our reply notified into a dead
	// link (ok=0). In the one connection that HELD, 0x10/0x01 was answered promptly and the console
	// carried on talking.
	//
	// Two writes fire per connection -- savePeerAddr() at connect and markStage() for non-ALIVE stages --
	// and both almost always store bytes IDENTICAL to what is on disk: the same console, the same stage.
	// So the erase bought nothing and cost us the connection. Reading first is cheap (no erase) and
	// changes no behavior; it only removes redundant writes.
	{
		File rf(InternalFS);
		if (rf.open(path, FILE_O_READ)) {
			uint8_t cur[64];
			bool same = false;
			if (len <= sizeof cur) {
				int got = rf.read(cur, len);
				same = (got == (int)len && memcmp(cur, data, len) == 0);
			}
			rf.close();
			if (same)
				return true;
		}
	}
	File f(InternalFS);
	if (!f.open(path, FILE_O_WRITE))
		return false;
	f.seek(0);
	bool ok = (f.write(data, len) == len);
	f.close();
	return ok;
}

// Persist the peer's BLE address the INSTANT a connection happens -- fires at link-layer connect, before
// any GATT work, so it costs almost none of any short surviving window if a crash were still lurking.
static void savePeerAddr(const ble_gap_addr_t &peer)
{
	uint8_t rec[9] = { 0x43,	  peer.addr_type, peer.addr[0], peer.addr[1],
			   peer.addr[2], peer.addr[3],	  peer.addr[4], peer.addr[5],
			   1 };
	writeFileInPlace("/peeraddr.bin", rec, sizeof rec);
}

static void printLastPeerAddr()
{
	File f(InternalFS);
	uint8_t rec[9];
	if (f.open("/peeraddr.bin", FILE_O_READ)) {
		if (f.read(rec, sizeof rec) == sizeof rec && rec[0] == 0x43 &&
		    rec[8] == 1) {
			Serial.printf(
				"# LEAN last peer addr (from a prior connect): type=%u %02X:%02X:%02X:%02X:%02X:%02X\n",
				rec[1], rec[7], rec[6], rec[5], rec[4], rec[3],
				rec[2]);
		}
		f.close();
	}
}

static void checkSoftDeviceImage()
{
	const uint32_t infoAddr = 0x3000;
	volatile const uint8_t *info8 = (volatile const uint8_t *)infoAddr;
	uint8_t structSize = info8[0];
	bool present = (structSize != 0xFF && structSize != 0x00);
	Serial.printf("# LEAN SD image check: structSize=%u -> %s\n",
		      structSize, present ? "PRESENT" : "ABSENT/ERASED");
}

// ============================================================================================
// Switch 2 BLE controller protocol constants -- copied byte-for-byte from the cached reference
// (trevlars/switch2-controllers-linux, MIT).
// ============================================================================================
#define NINTENDO_VENDOR_ID 0x057E
#define NINTENDO_COMPANY_ID 0x0553
// Joy-Con 2 Right's PID. NOTE we no longer advertise this: g_idMode defaults to Pro Controller 2
// (0x2069), which is the identity that actually registers. Kept because the GATT structure below still
// replicates a real Joy-Con 2 Right (the only device we have full ground truth for) and mode 0 uses it.
#define JOYCON2_RIGHT_PID 0x2066

#define CMD_MEMORY 0x02
#define SUB_MEMORY_READ 0x04
#define CMD_LEDS 0x09
#define SUB_LEDS_SET_PLAYER 0x07
#define CMD_VIBRATION 0x0A
#define SUB_VIBRATION_PLAY_PRESET 0x02
#define CMD_FEATURE 0x0C
#define SUB_FEATURE_INIT 0x02
#define SUB_FEATURE_ENABLE 0x04
#define CMD_PAIR 0x15
#define SUB_PAIR_SET_MAC 0x01
#define SUB_PAIR_LTK1 0x04
#define SUB_PAIR_LTK2 0x02
#define SUB_PAIR_FINISH 0x03

#define ADDR_CONTROLLER_INFO 0x00013000UL
#define ADDR_CALIB_JOYSTICK_1 0x000130A8UL // factory, left
#define ADDR_CALIB_JOYSTICK_2 0x000130E8UL // factory, right
#define ADDR_CALIB_USER_JOYSTICK_1 0x001FC042UL // user, left -- return 0xFFFFFF to force factory fallback
#define ADDR_CALIB_USER_JOYSTICK_2 0x001FC062UL // user, right

// Fixed LTK halves the Switch 2 protocol expects during bonding (byte-for-byte from the reference).
static const uint8_t PAIR_LTK1[17] = { 0x00, 0xEA, 0xBD, 0x47, 0x13, 0x89, 0x35, 0x42, 0xC6,
					0x79, 0xEE, 0x07, 0xF2, 0x53, 0x2C, 0x6C, 0x31 };
static const uint8_t PAIR_LTK2[17] = { 0x00, 0x40, 0xB0, 0x8A, 0x5F, 0xCD, 0x1F, 0x9B, 0x41,
					0x12, 0x5C, 0xAC, 0xC6, 0x3F, 0x38, 0xA0, 0x73 };

// The PAIR/LTK1 body WE announce (notification #6 of sendProactiveAnnouncement()). Byte-for-byte the value
// a real Joy-Con 2 sends -- confirmed identical across three captures and two physically different
// controllers, so it is a universal constant, not per-device crypto. Hoisted to file scope 2026-08-06
// because deriveLinkKey() below needs the exact same bytes: this half and the Switch's own PAIR/LTK1 write
// are the two inputs to the real link-layer key. [0] is a 1-byte marker (0x01 from a controller, 0x00 from
// the Switch), NOT part of the key material -- only [1..16] are.
// *** THIS IS THE REAL JOY-CON'S KEY MATERIAL, byte-for-byte. ***
//   ble_decrypt.js  CTRL_LTK1 = 5cf6ee79 2cdf05e1 ba2b6325 c41a5f10
// Since LTK = reverse(switch_LTK1 XOR our_LTK1), announcing the SAME half a registered controller
// announces means that if the console contributes the same switch-side half, we derive THE SAME LINK
// KEY as that Joy-Con. Sharing key-derivation input with a device already in the console's registry
// is the most literal form of the "Siamese twin" problem, and the 2026-08-11 de-clone MISSED IT --
// which is why that run does NOT count as a test of the identity theory.
//
// The value is ARBITRARY IN PRINCIPLE: any 16 bytes work provided we announce what we actually use.
// But it is live key material, so it is isolated behind '6' rather than bundled with the safe swaps.
// If pairing itself breaks after enabling '6', turn it off FIRST before suspecting anything else.
static const uint8_t ANNOUNCED_LTK1_OWN[17] = { 0x01, 0x3e, 0xb7, 0x41, 0xd8, 0x6a, 0x02, 0x9c, 0x55,
						0xf1, 0x27, 0x8d, 0x4b, 0xe0, 0x93, 0x36, 0xca };
static bool g_ownKeyMaterial = false; // '6' -- default OFF, this is live key material
static const uint8_t ANNOUNCED_LTK1[17] = { 0x01, 0x5c, 0xf6, 0xee, 0x79, 0x2c, 0xdf, 0x05, 0xe1,
					    0xba, 0x2b, 0x63, 0x25, 0xc4, 0x1a, 0x5f, 0x10 };

// The other three PAIR bodies we answer with, hoisted to file scope alongside ANNOUNCED_LTK1 for the same
// reason: the decrypted reconnect capture shows every one of these is a RESPONSE to a matching write from
// the Switch (0x15/0x01, 0x15/0x02, 0x15/0x03), not something a controller volunteers -- so both
// sendProactiveAnnouncement() and the reactive path in chrHandshakeCb() need the identical bytes.
//
// Our BLE address, least-significant octet first (the order ble_gap_addr_t::addr uses, and also the order
// SET_MAC announces it in -- the real Joy-Con's SET_MAC payload tail was cc bb aa ce c6 38 for address
// the real unit's). The address is FIXED, not rotated -- see initBleAddr().
//
// Why rotation: the Switch applies an adaptive backoff to an identity that repeatedly fails to pair. After
// ~28 failed attempts on the earlier address in the 2026-08-06 session it stopped sending CONNECT_IND to us
// altogether -- confirmed with the sniffer, which showed our advertisements still going out normally while
// the Switch simply never responded. A reset now gets a fresh identity instead of needing a reflash.
static uint8_t g_bleAddr[6] = { 0x99, 0x88, 0x77, 0x7E, 0x05, 0x00 };
// Built from g_bleAddr in initBleAddr() so the advertised address and the address we announce over GATT
// physically cannot drift apart -- they previously had to be kept in sync by hand across two files' worth
// of comments. Prefix (01 04 01) copied verbatim from the real capture, purpose unknown.
static uint8_t g_announcedSetMac[9] = { 0x01, 0x04, 0x01, 0x99, 0x88, 0x77, 0x7E, 0x05, 0x00 };
#define ANNOUNCED_SET_MAC g_announcedSetMac

// Sets our FIXED BLE address and rebuilds the SET_MAC payload to match. Bumps a boot counter only.
// ROTATION DISABLED 2026-08-07. This was added earlier the same day to dodge the Switch's pairing backoff,
// on the theory that a fresh address would look like a new device. It does -- and that is the problem: a
// real controller has ONE fixed address for its lifetime, whereas we minted a new identity on every reboot.
// Across a dozen reflashes in one session the console accumulates a dozen phantom controllers, and its
// registry is finite. That fits the symptom exactly: backoff arriving after ~12 attempts this morning but
// after only 1-3 tonight, and a console restart giving temporary relief. Pinning the address is also just
// more faithful -- so the counter is still bumped and logged (it is a useful boot counter) but no longer
// feeds the address.
static void initBleAddr() // was rotateBleAddr(): it PINS the address, it does not rotate (audit STALE-3)
{
	uint8_t ctr = 0;
	File f(InternalFS);
	if (f.open("/addrctr.bin", FILE_O_READ)) {
		f.read(&ctr, 1);
		f.close();
	}
	ctr++;
	// writeFileInPlace(), NOT remove()+create() -- this runs on EVERY boot, and this board is reflashed
	// dozens of times a session. Same reasoning as saveBondedHost(); missed in the same sweep.
	writeFileInPlace("/addrctr.bin", &ctr, 1);
	Serial.printf("# LEAN boot #%u (address is FIXED, not rotated)\n", ctr);
	// Bumped 0x01 -> 0x02 (2026-08-07). Still PINNED, not rotating -- this is a one-off change of identity.
	// After the console has bonded with us, it skips the PAIR ceremony and does an ENCRYPTED RECONNECT with
	// the key it stored. If our own copy is gone -- e.g. because the filesystem was formatted, which we do
	// whenever it fills up -- we cannot match it and the link dies with reason 0x3D (MIC failure), forever,
	// because it never re-pairs. Presenting a fresh address makes it treat us as a new device and pair from
	// scratch. LESSON: do NOT format the filesystem after a successful pairing without also bumping this.
	// EXPERIMENT 2026-08-08: use a NINTENDO OUI. Stored little-endian.
	//
	// Our address had used a 00:05:7E prefix -- 00:05:7E is not a Nintendo OUI at all; it looks like it
	// was chosen to echo the 0x057E vendor id in the advertisement, which is a different field entirely.
	// A real Joy-Con 2 Right uses the Nintendo OUI. We spoof Nintendo's VID/PID in the advert but have
	// never matched the BT address, and the address is how the console identifies and stores a registered
	// controller. By this point everything else is eliminated: the console sends us byte-identical
	// requests, we send byte-identical replies, encryption succeeds, and it neither sends the 0x0085
	// stream-start to our discovered handle nor to the hardcoded one. Whatever it judges is not in the
	// conversation, and the address is the last identity field we had never varied.
	//
	// Tail kept distinct from the real Joy-Con's (AA:BB:CC) so we cannot collide with a registered device.
	// OUI CHANGED 2026-08-09: 38:C6:CE is the REAL Joy-Con's OUI. We had
	// adopted it deliberately to look like Nintendo hardware, but it means we share the top half of our
	// identity with a controller registered on this same console -- and this console is DOCUMENTED to
	// track per-device identity and terminate on a collision (see the bd283 finding: serving the real
	// Joy-Con's bd283 while that Joy-Con was registered got us dropped at LTK2 every cycle).
	//
	// Symptom that prompted this: our puck and the real Joy-Con behave like conjoined devices -- both hold
	// a connection while both are registered, and when the Joy-Cons are removed entirely our puck cannot
	// pair at all. That is what an identity the console cannot cleanly separate would look like.
	//
	// 0x02 in the top octet = locally administered, so this collides with no vendor's OUI by construction.
	// Tail bumped to 04 as well, which additionally forces GATT rediscovery after today's table change
	// (the false wall was removed, shifting every handle down by one, and with Service Changed disabled a
	// bonded client would otherwise keep using its cached copy).
	//
	// ---- OUI RESTORED TO NINTENDO 2026-08-13 (user's observation + hypothesis) --------------------
	// Wireshark renders us as `MS-NLB-PhysServer-26_7f:77:88:04` and a real controller as
	// `Nintendo_aa:bb:cc`. The LABEL is cosmetic (Wireshark's local OUI table; 02:1a:* is Microsoft
	// NLB). The FACT under it is not: 0x02 sets the locally-administered bit -- "this address was
	// invented" -- while bleInitOnce() declares addr_type = BLE_GAP_ADDR_TYPE_PUBLIC, and a PUBLIC
	// address is by definition IEEE-assigned. We were announcing a public address and presenting a
	// made-up one, on a device whose advertisement claims byte-for-byte to be Nintendo hardware. A
	// real Pro Controller 2 carries a Nintendo OUI and means it.
	//
	// User's hypothesis: the console may simply not allocate an input stream to a device that is not
	// Nintendo-branded at the link layer. That fits today's reframe -- the console completes our
	// registration cleanly, never retries anything, and THEN declines to assign a stream, i.e. the
	// decision is late and identity-shaped.
	//
	// Why this is not just re-treading 2026-08-09: that move away from 38:C6:CE was made and judged on
	// firmware that could not complete registration at all. It has never been evaluated on a build that
	// gets this far. It is an UNTESTED variable that has been wearing an eliminated variable's clothes.
	// The Siamese-twin concern is respected by keeping the tail far from the real Joy-Con's AA:BB:CC.
	g_bleAddr[5] = 0x38;
	g_bleAddr[4] = 0xC6;
	g_bleAddr[3] = 0xCE;
	// Tail bumped 77:88:02 -> 77:88:03 on 2026-08-08 when the attribute table was re-laid-out for handle
	// alignment. A BLE client caches a bonded device's GATT table and only re-reads it when told the table
	// changed -- via Service Changed, which we just disabled to reclaim handles. So the console had no way
	// to learn our layout moved, and could have kept using a stale cached copy indefinitely. Presenting a
	// new address is the only clean way to force rediscovery now. Bump this again after any future change
	// to the table's shape.
	// Tail bumped 04 -> 05 on 2026-08-13 with the OUI change: a new address is a new device to the
	// console, which discards the stale registration and forces a clean re-pair and rediscovery -- both
	// wanted after today's attribute-table edits.
	g_bleAddr[2] = 0x77;
	g_bleAddr[1] = 0x88;
	g_bleAddr[0] = 0x07; // bumped 06 -> 07 2026-08-14: d5a9 and fd2 SWAPPED places, so the attribute
			     // table CHANGED SHAPE. A bonded client caches the table and, with Service Changed
			     // disabled, has no way to learn it moved -- it kept writing the stream trigger to
			     // d5a9 old descriptor handle and nothing armed. A new address forces rediscovery.
			     // BUMP THIS AFTER EVERY CHANGE TO THE TABLE SHAPE. Previously bumped 05 -> 06 2026-08-13: reason=0x3D MIC-failure loop -- the console held a
			     // link key for ...05 that our bond store no longer matched. A new address is
			     // the only clean escape; the console pairs us from scratch.
	memcpy(g_announcedSetMac + 3, g_bleAddr, 6);
	Serial.printf("# LEAN BLE address this boot: %02X:%02X:%02X:%02X:%02X:%02X\n", g_bleAddr[5],
		      g_bleAddr[4], g_bleAddr[3], g_bleAddr[2], g_bleAddr[1], g_bleAddr[0]);
}
// LTK2: unlike LTK1 this differs between the two real controllers captured, so it is a per-device
// value. It is NOT the link-layer key (that comes from the LTK1 pair -- see deriveLinkKey()), and
// skipping it entirely was already tested via 'K' with no behavioral change, so it appears inert.
// REDACTED. ANNOUNCED_LTK2 held the captured unit's own value and was replaced with a placeholder
// before publishing, the same treatment as SERIAL_CAPTURED and BD283_CAPTURED. The _OWN variant is
// unchanged, and uses the same toggle as ANNOUNCED_LTK1. Recapture from your own hardware if needed.
static const uint8_t ANNOUNCED_LTK2_OWN[17] = { 0x01, 0x7d, 0x05, 0xbe, 0x62, 0x19, 0xa4, 0xc3, 0x38,
						0x8f, 0x46, 0xd1, 0x2a, 0x97, 0x5c, 0xe4, 0x0b };
static const uint8_t ANNOUNCED_LTK2[17] = { 0x01, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00, 0x11,
					    0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99 };
static const uint8_t ANNOUNCED_PAIR_FINISH[1] = { 0x01 };

// The three non-PAIR command bodies, same story: each is the real controller's RESPONSE to a matching
// Switch write, and each has real content that the old reactive path replaced with an empty ack.
// Byte-verified from the pre-encryption plaintext of both clean fresh-pairing captures AND the decrypted
// reconnect capture (all three agree): 0x07/0x01 -> 9 bytes total, 0x10/0x01 -> 20, 0x16/0x01 -> 32.
static const uint8_t ANNOUNCED_CMD07[1] = { 0x00 };
// [3] is a side/model discriminator: 0x01 on the right Joy-Con, 0x00 on the left.
// UNRESOLVED (audit BUG-3): we advertise a PRO CONTROLLER (0x2069), not a Joy-Con, so 0x01 is very
// likely wrong -- the comment justifying it cited PID 0x2066, which has not been our PID since the
// identity switch. No Pro Controller capture exists to copy the right value from. The Joy-Con oracle
// (docs/captures/jc_rumble.py) can at least confirm the field IS a discriminator by comparing the
// left and right units.
// [3] is a side/model discriminator: 0x00 = left Joy-Con, 0x01 = right Joy-Con (both measured).
// A Pro Controller is NEITHER SIDE, so 0x01 was a direct self-contradiction -- we advertise
// 0x2069 while announcing "I am the right Joy-Con" (audit BUG-3).
// No Pro Controller capture exists, so the correct value is UNKNOWN. 0x02 is the natural next
// enum value after L=0x00/R=0x01, but that is INFERENCE, NOT EVIDENCE. Hence it is mutable and
// swept with the '3' console command rather than baked in -- four candidates, zero reflashes.
//
// *** FIRST CHECK IF THINGS FAIL. *** If bring-up regresses or the stream trigger still never
// arrives, sweep this before touching anything else -- it is the cheapest variable in the firmware
// and the only one where we are knowingly guessing.
//
// A REAL FULL CONTROLLER MAY NOT NEED THIS FIELD AT ALL. The field exists to say WHICH SIDE a
// half-controller is; a Pro Controller has no side to report, so the honest value may be 0x00
// meaning "not applicable" rather than a new enum member. That makes 0x00 a strong candidate for a
// reason unrelated to it being the left Joy-Con's value -- same byte, different meaning. It is also
// possible the console ignores this field entirely for a two-stick device, in which case none of
// the four candidates will change anything, and that null result is itself informative.
// ============================================================================================
// WORKING CONFIGURATION -- 2026-08-13. The day the console finally consumed our report stream.
//
// Achieved, measured, not inferred: the console arms the stream, subscribes our report CCCD,
// streams for 62+ s at ~30/s with ZERO hvx failures, assigns a player LED (0x09/0x07), sends a
// rumble preset (0x0A/0x02), and RENDERS OUR STICK DEFLECTION ON SCREEN.
//
// What had to be true, in the order it was found:
//   1. 679d descriptor must be 2 bytes AND variable length. Bluefruit's addDescriptor() hardcodes
//      vlen=0, so the console's 2-byte 0x0085 trigger was rejected with ATT 0x0D and raised NO
//      application event. See addVarLenDescriptor().
//   2. The trigger goes to **d5a9's** 679d descriptor, not fd2's -- and d5a9's copy had
//      addDescriptor()'s default write_perm of SECMODE_NO_ACCESS, so it was bounced with ATT 0x03,
//      invisibly, for the entire life of this project. Found only by opening EVERY attribute
//      (see armProbe()).
//   3. d5a9 is the report channel. The console finds it BY UUID at 0x002C-0x002E; handle alignment
//      is irrelevant and the old 0x000F/0x0010 assertion was deleted because it would now FAIL on
//      a working build.
//   4. 0x01/0x0C must be ANSWERED (the console asks for it after arming the stream).
//   5. report[8] must be written AFTER the motion memcpy -- MOTION_OFF=4/MOTION_LEN=52 covers
//      [4:56] and was clobbering the battery/mode byte every frame.
//   6. Replies must be the CAPTURED values: RSP_01_0C = 61 12 50 10 (g_ownMisc=false) and
//      ANNOUNCED_CMD10[3] = 0x01 (below). We were sending invented bytes in both.
//   7. Joy-Con 2 Right identity (g_idMode = 0), and a registration created FRESH under it.
//
// METHOD WARNING THAT COST WEEKS: everything "eliminated" before 2026-08-13 was eliminated in a
// conversation that ended before the console read anything. CMD10[3] was swept and declared
// "genuinely unread"; RSP_01_0C content was "irrelevant"; report CONTENT was "eliminated". None of
// those tests could have detected an effect. RE-TEST anything crossed off before that date.
//
// STILL OPEN: the console reports "low battery" (was "out of battery") and eventually stops
// talking -> supervision timeout. The battery field is somewhere in the replayed body [4:56] and
// is NOT report[8] (0x88 tried) and NOT [31:35] as millivolts (4000 tried).
//
// *** DO NOT BISECT THE LIVE REPORT BODY BY SPLICING BYTE SPANS. *** Freezing part of the body and
// replaying the rest produces reports no real controller would ever emit -- an IMU sample on
// another frame's timestamp -- and it CRASHED THE CONSOLE on 2026-08-13. Harmless for months while
// the stream was discarded; not harmless now that it is parsed. Vary ONE semantic field at a time,
// at a plausible value, keeping every frame internally coherent.
// ============================================================================================
static uint8_t ANNOUNCED_CMD10[12] = { 0x02, 0x01, 0x04, 0x01, 0x0c, 0x00,
				      0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t g_cmd10Disc = 0x01; // mirrors ANNOUNCED_CMD10[3]; '3' cycles 0x02->0x03->0x00->0x01
static const uint8_t ANNOUNCED_CMD16[24] = { 0 };

// ============================================================================================
// PHASE 6 -- post-encryption init. Everything below is transcribed from the decrypted reconnect
// capture (docs/captures/joycon2_reconnect_DECRYPTED.txt) rather than guessed: once the link is
// encrypted the Switch runs a normal controller bring-up, and every one of these is its request
// answered by the real controller's reply. Same channels as before (Switch writes chrUnk65a7,
// controller notifies COMMAND_RESPONSE) -- encryption changes nothing about the framing.
// NOT yet exercised against a real console.
// ============================================================================================

// Calibration/config blobs the Switch reads by address. 0x001FC040 reading all-0xFF is what makes a
// real controller fall back to factory calibration, so it is correct (not missing data).
//
// REDACTED. The per-unit factory calibration of the captured Joy-Con 2 Right (gyro zero-rate bias,
// measured gravity, stick centre and travel) was replaced with nominal values before publishing, the
// same values switch2_memory.c serves. Layouts and structural bytes are unchanged.
static const uint8_t MEM_00013040[16] = { 0xec, 0xbb, 0xdb, 0x41, 0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// ECHO PROBE. The console READS this page and writes its first 8 bytes straight back to us inside
// its 0x0A/0x08 command:
//   console -> 01 | 69 09 00 00 FF FF FF FF | 35 00 | 46 00 | 00 00 ...
//   our page ->      69 09 00 00 ff ff ff ff ff ff ...
// So `69 09 00 00` (LE 0x00000969 = 2409) is not something the console knows independently -- it is
// OUR OWN CLONED DATA being reflected. Meaning still undecoded: the rest of the page is erased 0xFF,
// leaving four meaningful bytes with no context to decode them from.
//
// Because it is echoed, this is SELF-VERIFYING and worth more as a probe than as a guess: swap in a
// distinct value with '5' and watch the 0x0A/0x08 ALIGN log. If [26]/[27] come back as our new bytes,
// that is the FIRST direct proof the console ingests and acts on data it reads from our memory pages
// -- something never yet confirmed. If the echo still reads 69 09, the console is replaying a cached
// or hardcoded value and our page is not what it sounds like.
//
// Default OFF: this is an experiment, not part of the de-clone baseline.
static bool g_ownPage13060 = false;
static const uint8_t MEM_00013060_OWN[32] = { 0xa7, 0x3c, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
					      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
					      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static const uint8_t MEM_00013060[32] = { 0x69, 0x09, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
					  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
					  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static const uint8_t MEM_00013080[64] = {
	0x04, 0xac, 0xca, 0xaa, 0x53, 0x35, 0x55, 0x93, 0x30, 0x09, 0x93, 0x30, 0x09, 0xd2, 0x20, 0x0d,
	0xd2, 0x20, 0x0d, 0xcc, 0xac, 0xd9, 0xcc, 0xac, 0xd9, 0xd4, 0x42, 0x2d, 0xd4, 0x42, 0x2d, 0x8f,
	0xf4, 0x48, 0x8f, 0xf4, 0x48, 0x0f, 0xff, 0xff, 0x00, 0x08, 0x80, 0x40, 0x06, 0x64, 0x40, 0x06,
	0x64, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
// 0x000130C0 is the ONE address the console reads from us that we had no blob for, so it fell through to
// the unknown-address default and we answered 64 bytes of 0xFF -- erased, unprogrammed flash.
//
// That is the RIGHT STICK's factory calibration chunk. The console reads calibration in 64-byte pages:
// 0x13080 covers ADDR_CALIB_JOYSTICK_1 (0x130A8, left) and 0x130C0 covers ADDR_CALIB_JOYSTICK_2
// (0x130E8, right). Both sit at offset 0x28 = 40 within their page -- confirmed by MEM_00013080, whose
// bytes [40..48] are exactly three packed 12-bit XY triples (center, max, min), the shape
// packStickXY() produces.
//
// So we were telling the console that a PRO CONTROLLER -- a two-stick device -- has an uncalibrated right
// stick. The existing ADDR_CALIB_JOYSTICK_2 special case never fired, because the console never asks for
// that address directly; it asks for the page containing it.
//
// Calibration is not identity, so reusing the captured LEFT stick's real values for the right is safe
// and more plausible than synthesising neutral ones -- both sticks on a real Pro Controller are the
// same part with near-identical factory ranges.
//
// RESIDUAL HAZARD (was audit RISK-2): no capture of a real 0x000130C0 page exists, so this remains the
// only page we answer with content not taken from that address on real hardware. If the console ever
// validates something specific to the right stick, this is still where it would fail.
// RIGHT STICK CALIBRATION. Rebuilt 2026-08-12 -- the previous contents were 0xFF for bytes [0:40],
// i.e. ERASED FLASH, with only the 9-byte tail copied across from the left stick. The comment above
// claimed it reused "the LEFT stick's real, known-valid triples"; it did not. Intent and
// implementation had diverged, and the page passed every structural check we devised because
// 0xFF == 0xFF satisfies the duplicate-pair test trivially.
//
// Why that mattered: we advertise a PRO CONTROLLER, so the console reads this page (a read it never
// performs on a one-stick Joy-Con) and we answered "that stick has no calibration data" -- a device
// claiming an input axis it cannot describe. Not a wrong value; a MISSING one, on hardware we claim
// to have.
//
// Now a byte-for-byte copy of the LEFT stick page (0x13080), which is real captured data and
// therefore structurally complete: 7-byte header, five 3-byte records each stored TWICE (the
// flash-integrity duplication), the 0x0FFFFF terminator, the 9-byte tail, then erased padding.
// Reusing the left stick's numbers is safe -- calibration is not identity, and both sticks on a real
// Pro Controller are the same part with near-identical factory ranges.
// REVERTED 2026-08-12 to the long-standing contents. On 2026-08-12 I filled bytes [0:40] with a copy
// of 0x13080's [0:40] on the belief that the right stick "had never been calibrated". THAT WAS WRONG:
// ADDR_CALIB_JOYSTICK_2 is 0x130E8, i.e. OFFSET 40 in this page, and those 9 bytes were already the
// left stick's real, known-valid triples -- exactly as the original comment claimed. The stick
// calibration was correct all along; I had mis-decoded which part of the page held it.
//
// Bytes [0:40] cover addresses 0x130C0-0x130E7, whose meaning is UNKNOWN. Copying the left page's
// equivalent region was a guess that could just as easily be garbage, so this returns to 0xFF --
// the erased-flash convention, and the state every earlier session ran with.
// RIGHT stick calibration, OURS. Round-3 review lead 5: `g_ownCalib` swapped the LEFT page's factory
// triples while this page was behind no toggle, so by default we claimed one controller's two sticks
// were calibrated on different devices -- and the right stick stayed a byte-for-byte clone of the
// registered Joy-Con no matter what '4' was set to. Same shifted triples as MEM_00013080_OWN, at the
// same offset 40 (= ADDR_CALIB_JOYSTICK_2, 0x130E8). The 0xFF regions are untouched.
static const uint8_t MEM_000130C0_OWN[64] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x4e, 0x96, 0x7b, 0xa8, 0x02, 0x48, 0xd5, 0xf2,
	0x47, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
static const uint8_t MEM_000130C0[64] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x08, 0x80, 0x40, 0x06, 0x64, 0x40, 0x06,
	0x64, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
// ---- OWN-IDENTITY VARIANTS (audit/de-clone 2026-08-09) --------------------------------------
// Every table we serve was a BYTE-FOR-BYTE copy of one specific physical Joy-Con -- verified 0
// bytes differing on MEM_00013080/40/60 and RSP_11_03/01_0C against the decrypted capture. That
// unit is registered on the same console we are trying to join, so we were not merely "shaped
// like" a Joy-Con, we were impersonating a particular one.
//
// These variants keep the exact STRUCTURE (length, field layout, the zero/0xFF regions) and change
// only the per-unit random-looking bytes -- the same treatment BD283_UNIQUE already had.
// Selected by g_ownFingerprint (default ON). '2' toggles back to the clone values for A/B.
//
// *** DO NOT USE THESE. Decoded 2026-08-12: the bytes they perturb are NOT fingerprints, they are
// PHYSICS. ***
//   MEM_00013040 = gyro calibration -- float32 reference temperature (27.4668 degC) + 3-axis
//                  zero-rate bias in rad/s
//   MEM_00013100 = accelerometer calibration -- the unit's measured gravity plus offsets
//   RSP_11_03    = IMU scale factors and full-scale ranges (8g/2g, 2000/500 dps), exact to 7 s.f.
// Perturbing them makes us claim an impossible gravity constant, absurd gyro bias and nonsensical
// sensor scaling. These values are not identifying -- every controller with the same IMU reports the
// same constants -- so there is nothing to de-clone here. KEEP '2' SET TO CLONE. A reflash resets it
// to OURS (garbage); press '2' after every flash. Better: excise them from the de-clone in code.
static const uint8_t MEM_00013040_OWN[16] = {
	0x00, 0x00, 0xd1, 0x41, 0x38, 0xba, 0x0c, 0xbb,
	0x8f, 0xcd, 0xa7, 0x3b, 0x3c, 0xde, 0x43, 0xbb
};
static const uint8_t MEM_00013100[24] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x41,
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// Accelerometer calibration, OURS. Category 2 (per-unit measurement): the real unit reported its own
// measured gravity; we report 9.81042, equally plausible for a real part (standard g is 9.80665) and
// unmistakably a different device. Layout: three 0.0 floats, then measured gravity and two offsets.
static const uint8_t MEM_00013100_OWN[24] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x7b, 0xf7, 0x1c, 0x41,
	0xa9, 0x09, 0x89, 0x3b, 0x5c, 0xb6, 0xef, 0xbb
};
// IMU scale factors and full-scale ranges. CATEGORY 1 -- PURE ARITHMETIC, NOT IDENTIFYING.
// These are 8g/32767, 2000dps/32767, and the ranges themselves. EVERY controller with +-8g/+-2000dps
// computes exactly these numbers, so matching the real device is not cloning -- it is being correct.
// Deliberately identical to the captured values. Do not "de-clone" these; it only makes us wrong.
static const uint8_t RSP_11_03_OWN[29] = {
	0x01, 0x20, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c,
	0x3b, 0x79, 0x7d, 0x8b, 0x3a, 0x0a, 0xe8, 0x9c,
	0x42, 0x58, 0xa0, 0x0b, 0x42, 0x0a, 0xe8, 0x9c,
	0x41, 0x58, 0xa0, 0x0b, 0x41
};
// Stick calibration, OURS. Category 2. Structure preserved exactly (7-byte header, five 3-byte
// records each stored TWICE for flash integrity, 0x0FFFFF terminator, then the 9-byte factory
// calibration at offset 40 = ADDR_CALIB_JOYSTICK_1). Only the packed 12-bit triples at offset 40
// are shifted a few counts from the real unit -- still a plausible factory range, still coherent.
static const uint8_t MEM_00013080_OWN[64] = {
	0x04, 0xac, 0xca, 0xaa, 0x53, 0x35, 0x55, 0x93, 0x30, 0x09, 0x93, 0x30, 0x09, 0xd2, 0x20, 0x0d,
	0xd2, 0x20, 0x0d, 0xcc, 0xac, 0xd9, 0xcc, 0xac, 0xd9, 0xd4, 0x42, 0x2d, 0xd4, 0x42, 0x2d, 0x8f,
	0xf4, 0x48, 0x8f, 0xf4, 0x48, 0x0f, 0xff, 0xff, 0x4e, 0x96, 0x7b, 0xa8, 0x02, 0x48, 0xd5, 0xf2,
	0x47, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
// DEFAULT FALSE. These _OWN variants perturb gyro bias, measured gravity and the IMU scale
// factors -- PHYSICS, not fingerprints (decoded 2026-08-12). A reflash silently re-enabling them has
// already invalidated three separate test rounds. Turn on deliberately, never by default.
static bool g_ownFingerprint = false; // '2' -- per-unit pages ours vs the captured unit's
// DEFAULT FALSE, same reasoning -- stick calibration is measured data, and perturbing it breaks the
// duplicate-pair flash-integrity invariant. Now covers BOTH sticks (left 0x13080 + right 0x130C0).
static bool g_ownCalib = false;       // '4' -- stick calibration ours vs the captured unit's
static const uint8_t MEM_001FC040[64] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

// Command replies. Note CMD_FEATURE (0x0c) INIT and ENABLE both answer with 4 payload bytes, not the
// empty ack -- an empty one here would stall bring-up the same way it stalled the pairing ceremony.
static const uint8_t RSP_FEATURE[4] = { 0x00, 0x00, 0x00, 0x00 }; // 0x0c/0x02 and 0x0c/0x04
static const uint8_t RSP_11_01[4] = { 0x01, 0x00, 0x00, 0x00 };
static const uint8_t RSP_13_01[4] = { 0x01, 0x00, 0x00, 0x00 };
static const uint8_t RSP_11_03[29] = { 0x01, 0x20, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x3b, 0x79,
				       0x7d, 0x8b, 0x3a, 0x0a, 0xe8, 0x9c, 0x42, 0x58, 0xa0, 0x0b,
				       0x42, 0x0a, 0xe8, 0x9c, 0x41, 0x58, 0xa0, 0x0b, 0x41 };
// Unsolicited notification the real controller fires once, immediately after the report stream is
// enabled and just before the first input report. Meaning unknown.
// Verified byte-identical to the real unit (decrypt frame 1940). Swapped by '7'.
static const uint8_t RSP_01_0C_OWN[4] = { 0x2d, 0x8a, 0x47, 0xf3 };
static bool g_ownMisc = false; // WORKING DEFAULT 2026-08-13 -- see the WORKING CONFIGURATION block
static const uint8_t RSP_01_0C[4] = { 0x61, 0x12, 0x50, 0x10 };

// Real 63-byte input report, captured idle (frame 1942). Used as a template rather than a zero-filled
// buffer for the same reason as REAL_CONTROLLER_INFO_TEMPLATE: several fields have non-obvious resting
// values, and an all-zero report is not what a real device ever sends.
// Layout, measured on 678 consecutive real reports rather than assumed:
//   [0:2]   16-bit report sequence counter, +1 per report -- NOT a millisecond timestamp
//   [2:4]   button field (see the BUTTON MAP block below)
//   [4]     constant 0x07 in every real frame, minimal and full alike. Meaning unknown.
//   [5:8]   STICK, 12-bit packed: x = b0 | (b1 & 0x0F) << 8, y = b1 >> 4 | b2 << 4.
//           NOT [4:8] as previously recorded -- that framing decodes a resting stick to
//           (2823, 3460), i.e. jammed into a corner. [5:8] gives (2123, 2013) against a
//           2048 center. Confirmed by scanning all 60 possible triples across 658 resting
//           frames: [5:8] is the ONLY one that is both near-center AND still (spread 5/2);
//           every other near-center triple sits in the IMU block with ~4095 spread, i.e.
//           it is noise that happens to average out near the middle.
//   [8]     status byte: (battery << 4) | mode flags, the same shape Switch 1 used.
//           Low nibble went 0x0 -> 0x8 the instant CMD_FEATURE enabled full reporting,
//           so that half is report mode, not charge. High nibble held 3 for a whole
//           capture while ...fd2 reported 3389mV -- a low pack -- so the high nibble is
//           the battery level the console displays. THIS is where the console gets
//           battery: not from a millivolt field (there is none in this report) and not
//           from ...fd2, which no capture ever shows the console reading or subscribing.
//           Our replayed 0x38 is why a battery shows up for us at all -- it is frozen at
//           whatever the captured controller had, and never moves.
//   [9:16]  unclassified -- varies, but not yet attributed to any input.
//   [16:56] IMU, replayed from MOTION_REPLAY -- never static on a real device
//   [56:63] constant zero in every real frame
#include "motion_replay.h"
// ---- CAPTURED-TEMPLATE VARIANTS (de-clone round 3) ------------------------------------------
// Both templates below are byte-captures of a REAL unit. The clone checker
// (docs/captures/clonecheck.js) flags INPUT_REPORT_TEMPLATE outright; CONTROLLER_INFO evades it
// only because 14 of its 64 bytes (the serial) were already changed -- ~50 bytes are still real.
//
// These variants are GENERATED from the originals, preserving structure and changing only
// per-unit bytes:
//   INPUT_REPORT     -- [0:8] kept (counter/buttons/constant 0x07/packed stick; overwritten per
//                       report anyway); the IMU/motion body is perturbed.
//   CONTROLLER_INFO  -- vendor/product [18:22] KEPT (our claimed identity must stay coherent),
//                       serial [2:16] already ours, 0xFF tail from [37] kept (erased-flash
//                       convention); everything else perturbed.
// Selected by g_ownTemplates, default ON. '8' toggles back to the captured originals for A/B.
static bool g_ownTemplates = false; // WORKING DEFAULT 2026-08-13
static const uint8_t INPUT_REPORT_TEMPLATE_OWN[63] = {
	0x06, 0x18, 0x00, 0x00, 0x07, 0x68, 0xb8, 0x7c, 0xcb, 0x99, 0xa0, 0x00, 0x00, 0x15, 0x00, 0xec,
	0x34, 0x82, 0x00, 0xee, 0xea, 0x1e, 0xf8, 0x92, 0xf2, 0xc9, 0x97, 0x0e, 0xf6, 0x5e, 0xf9, 0x61,
	0xe0, 0x8b, 0x59, 0xb0, 0x50, 0x49, 0x44, 0xaa, 0xb3, 0x91, 0xe5, 0x4a, 0x8e, 0x89, 0x97, 0x4f,
	0x6a, 0xc2, 0x3a, 0x7b, 0x4b, 0xf4, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const uint8_t REAL_CONTROLLER_INFO_TEMPLATE_OWN[64] = {
	0x5c, 0x00, 0x48, 0x43, 0x50, 0x37, 0x31, 0x30, 0x39, 0x39, 0x38, 0x38, 0x37, 0x37, 0x30, 0x31,
	0x00, 0x00, 0x7e, 0x05, 0x66, 0x20, 0xf6, 0x04, 0x05, 0x3c, 0x43, 0x4a, 0xc9, 0xd0, 0xd7, 0x66,
	0x6d, 0x74, 0x7b, 0x82, 0x89, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
static const uint8_t INPUT_REPORT_TEMPLATE[63] = {
	0x06, 0x18, 0x00, 0x00, 0x07, 0x68, 0xb8, 0x7c, 0x38, 0xff, 0xff, 0x00, 0x00, 0x5f, 0x00, 0x28,
	0x69, 0xb0, 0x00, 0x0e, 0x03, 0x30, 0x03, 0x96, 0xef, 0xbf, 0x86, 0xf6, 0xd7, 0x38, 0xcc, 0x2d,
	0xa5, 0x49, 0x10, 0x60, 0xf9, 0xeb, 0xdf, 0x3e, 0x40, 0x17, 0x64, 0xc2, 0xff, 0xf3, 0xfa, 0xab,
	0xbf, 0x10, 0x81, 0xbb, 0x84, 0x26, 0x85, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Returns the blob the Switch expects for a MEMORY read at `addr`, or false if we have no capture of it.
static bool lookupMemory(uint32_t addr, const uint8_t **out, uint8_t *outLen)
{
	switch (addr) {
	// Own-identity selection: see the OWN-IDENTITY VARIANTS block for why these are not the
	// captured Joy-Con's values by default.
	case 0x00013040UL: *out = g_ownFingerprint ? MEM_00013040_OWN : MEM_00013040;
			   *outLen = sizeof MEM_00013040; return true;
	case 0x00013060UL: *out = g_ownPage13060 ? MEM_00013060_OWN : MEM_00013060;
			   *outLen = sizeof MEM_00013060; return true;
	case 0x00013080UL: *out = g_ownCalib ? MEM_00013080_OWN : MEM_00013080;
			   *outLen = sizeof MEM_00013080; return true;
	case 0x000130C0UL: *out = g_ownCalib ? MEM_000130C0_OWN : MEM_000130C0;
			   *outLen = sizeof MEM_000130C0; return true;
	case 0x00013100UL: *out = g_ownFingerprint ? MEM_00013100_OWN : MEM_00013100;
			   *outLen = sizeof MEM_00013100; return true;
	case 0x001FC040UL: *out = MEM_001FC040; *outLen = sizeof MEM_001FC040; return true;
	default: return false;
	}
}

// ---- Device identity. Until 2026-08-06 we shipped a chimera: bd283 was a REAL right Joy-Con's per-device
// identity value (copied from a capture of the very controller already paired to this console), the serial
// was the obviously-synthetic "OPENPUCKBLE001", and SET_MAC announced a third, unrelated address. A real
// controller is self-consistent. Live testing got the Switch all the way through the PAIR ceremony
// (SET_MAC -> LTK1 -> LTK2) and then a deliberate LL_TERMINATE_IND with nothing ever shown on screen,
// which looks much more like a final identity/consistency check failing than a protocol error.
//
// Real serials are 14 chars: a 3-char model code then 11 digits -- "HCW00000000000" (right Joy-Con) and
// "HBW10000000000" (left), decoded from the captures. "OPENPUCKBLE001" matches neither the charset nor the
// shape, so it fails even a trivial format check.
//
// 'I' flips both back to the captured values at runtime so this can be bisected without a reflash.
// Advert byte, payload[16]/[17]. Two forms exist on real hardware:
//   REAL    0x0F at [16] -- what genuine hardware broadcasts; puts the console on its RECOGNIZED
//                           path, which is how it spots a controller it already knows from any screen.
//   GENERIC 0xF0 at [17] -- asks the console to DISCOVER us. Needed to get registered in the first place.
//
// buildAdvertising() selects on BOND STATE (unbonded -> generic, bonded -> real); this flag forces the
// real form while unbonded, for testing. Toggled by 'Z'.
//
// The recognized path used to die on its first write (to 0x0005) -- that was OUR bug, not the console's:
// bd281 was handing it a real device's handles. Fixed 2026-08-09. Superseded history in the memory file.
static bool g_advRealByte = false;
// bd283 identity, toggled by 'Q' independently of the serial.
// DEFAULT false (our own synthetic unique value) -- TESTED 2026-08-08 and the captured value is actively
// WORSE: serving the real Joy-Con's bd283 while that Joy-Con is registered on the same console gets us
// terminated at PAIR/LTK2, 43ms after the challenge, every cycle. That is the same "43ms at LTK2"
// signature seen in earlier sessions, which were using the captured identity. The console evidently
// tracks bd283 as a per-device identity and rejects a collision with an already-registered controller.
// So bd283 is NOT the blocker, and a unique value is required rather than merely acceptable.
static bool g_bd283Captured = false;
static bool g_useCapturedIdentity = false;
// Real shape (3-char model code + 11 digits), not a real device's digits. The model code was "HCW",
// which is a JOY-CON's -- both captured Joy-Cons use it -- so we were declaring a Joy-Con model number
// inside the info blob while advertising a Pro Controller 2. No Pro Controller 2 serial has ever been
// captured publicly, so "HCP" is a guess that is merely SELF-CONSISTENT rather than correct. If the
// console turns out to validate the model code, this is the first thing to revert.
static const char SERIAL_SYNTHETIC[15] = "HCP71099887701";
// REDACTED. This held the real serial of a captured Joy-Con 2 Right, and BD283_CAPTURED held that
// controller's real identity bytes. Both were replaced with synthetic values before publishing, so the
// clone path below no longer clones anything. Recapture from your own hardware if you need it.
//
// Why: the console lets us through the entire ceremony and then decides against us at the one step that
// is a judgement rather than a question. We claim to BE a Joy-Con 2 Right (vendor 0x057E, product
// 0x2066), so the console applies Joy-Con validation to us -- and our serial is invented. Both real
// serials we have continue "102..." after the prefix where ours jumps to "109...", so a format or check
// rule is plausible. This is the clean test: if a byte-perfect identity still gets rejected, identity is
// eliminated for good; if it is accepted, serial validation is the gate.
//
// CAVEAT: this duplicates the identity of a controller that may be registered on the same console, so a
// rejection could be a duplicate-identity refusal rather than a validation pass/fail. Unregister the real
// Joy-Con first if the result looks ambiguous.
static const char SERIAL_CAPTURED[15] = "HCW00000000000";
// bd283 (attribute 0x0006, read by the Switch before any CCCD activity). Per-device on real hardware --
// per-device on both captured units, with no discernible structure, so
// a distinct value
// should be as valid as any. The point is to stop claiming an identity this console already knows.
static const uint8_t BD283_UNIQUE[8] = { 0x5a, 0x0e, 0x71, 0xc3, 0x88, 0x24, 0xb6, 0x1f };
static const uint8_t BD283_CAPTURED[8] = { 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00, 0x11 };

// GATT characteristic UUIDs (byte-identical across independently-written reference repos).
const uint8_t UUID_INPUT_REPORT[16] = { 0xd2, 0x7f, 0xdf, 0x09, 0x8f, 0x11, 0x8f, 0x82,
					0xad, 0x49, 0xfe, 0x89, 0xbe, 0xe9, 0x7d, 0xab };
const uint8_t UUID_COMMAND_WRITE[16] = { 0x05, 0xf0, 0xe5, 0x4f, 0xa5, 0x1e, 0x44, 0xaf,
					 0x6c, 0x4e, 0xb7, 0x8e, 0xc9, 0x4a, 0x9d, 0x64 };
const uint8_t UUID_COMMAND_RESPONSE[16] = { 0x6a, 0x83, 0x11, 0xb1, 0x15, 0x53, 0x0a, 0xa2,
					    0x36, 0x4d, 0xd8, 0xd9, 0x61, 0xa9, 0x65, 0xc7 };
// Vibration. The UUID DIFFERS PER CONTROLLER MODEL -- values from trevlars/switch2-controllers-linux:
//   Pro Controller 2  cc483f51-9258-427d-a939-630c31f72b05
//   Joy-Con 2 Right   fa19b0fb-cd1f-46a7-84a1-bbb09e00c149
//   Joy-Con 2 Left    289326cb-a471-485d-a8f4-240c14f18241
//
// RESTORED to the Pro Controller value 2026-08-08. It had been switched to the Joy-Con Right UUID on
// 2026-08-04 to match a Joy-Con identity -- correct then, wrong now: we register as a Pro Controller 2
// (PID 0x2069), which is what finally made registration work, so exposing a Joy-Con's vibration
// characteristic is exactly the "we are inconsistent about what we ARE" problem. That matters because a
// control test showed a registered-but-SILENT puck still knocks the console's controller subsystem over,
// taking a real Joy-Con down with it -- so the destabilisation comes from our structure and self-
// description, not from anything we stream. This is structural: read during discovery, before any input.
// Bytes are little-endian (reverse of the printed UUID), as everywhere else in this file.
const uint8_t UUID_VIBRATION_PRO[16] = { 0x05, 0x2b, 0xf7, 0x31, 0x0c, 0x63, 0x39, 0xa9,
					 0x7d, 0x42, 0x58, 0x92, 0x51, 0x3f, 0x48, 0xcc };
// Real primary service UUID, ab7de9be-89fe-49ad-828f-118f09df7fd0 -- ground truth from connecting directly
// to a genuine Joy-Con 2 with nRF Connect Mobile's Services browser (2026-08-04). Note this differs from
// UUID_INPUT_REPORT only in the last hex digit (fd0 vs fd2) -- the real controller groups INPUT_REPORT,
// COMMAND_WRITE, COMMAND_RESPONSE, and 7 other characteristics all under this ONE service.
const uint8_t UUID_SWITCH2_SERVICE[16] = { 0xd0, 0x7f, 0xdf, 0x09, 0x8f, 0x11, 0x8f, 0x82,
					   0xad, 0x49, 0xfe, 0x89, 0xbe, 0xe9, 0x7d, 0xab };

// ---- Full real GATT structure below, all ground truth from the same nRF Connect Mobile session
// (2026-08-04), purpose of each "unknown" characteristic/descriptor not understood -- these are
// implemented as inert placeholders purely to occupy the same ATT handle numbers a real Joy-Con 2 Right
// does, on the theory the Switch expects specific fixed handles and won't proceed without them present.

// Second, entirely separate primary service: 00c5af5d-1964-4e30-8f51-1956f96bd280, 3 characteristics.
const uint8_t UUID_SERVICE1[16] = { 0x80, 0xd2, 0x6b, 0xf9, 0x56, 0x19, 0x51, 0x8f,
				    0x30, 0x4e, 0x64, 0x19, 0x5d, 0xaf, 0xc5, 0x00 };
const uint8_t UUID_S1_CHAR_R1[16] = { 0x81, 0xd2, 0x6b, 0xf9, 0x56, 0x19, 0x51, 0x8f,
				      0x30, 0x4e, 0x64, 0x19, 0x5d, 0xaf, 0xc5, 0x00 }; // READ
const uint8_t UUID_S1_CHAR_W[16] = { 0x82, 0xd2, 0x6b, 0xf9, 0x56, 0x19, 0x51, 0x8f,
				     0x30, 0x4e, 0x64, 0x19, 0x5d, 0xaf, 0xc5, 0x00 }; // WRITE
const uint8_t UUID_S1_CHAR_R2[16] = { 0x83, 0xd2, 0x6b, 0xf9, 0x56, 0x19, 0x51, 0x8f,
				      0x30, 0x4e, 0x64, 0x19, 0x5d, 0xaf, 0xc5, 0x00 }; // READ

// Service 2's 7 undocumented characteristics, in real declaration order interspersed among the 3 known
// ones (INPUT_REPORT, COMMAND_WRITE, COMMAND_RESPONSE) -- see setupGatt() for the full real order.
const uint8_t UUID_UNK_D5A9[16] = { 0x42, 0xf4, 0x2b, 0x14, 0x67, 0x8b, 0x0c, 0xb2,
				    0xca, 0x4c, 0xfc, 0x2f, 0x1e, 0xe0, 0xa9, 0xd5 }; // NOTIFY,READ
const uint8_t UUID_UNK_65A7[16] = { 0xff, 0x27, 0x6b, 0x37, 0x42, 0xa3, 0x78, 0x80,
				    0x61, 0x4a, 0xe7, 0xf1, 0xb3, 0x24, 0xa7, 0x65 }; // WRITE NO RESPONSE
const uint8_t UUID_UNK_4147[16] = { 0x8d, 0x9f, 0xf5, 0x5d, 0x3e, 0xd2, 0xf7, 0xa4,
				    0xf7, 0x4d, 0xae, 0xfd, 0x3d, 0x42, 0x47, 0x41 }; // WRITE NO RESPONSE
const uint8_t UUID_UNK_640C[16] = { 0x0b, 0x69, 0x2b, 0xaf, 0x6f, 0x42, 0xf3, 0xa7,
				    0x0c, 0x41, 0x88, 0x0e, 0x8e, 0xa5, 0x0c, 0x64 }; // NOTIFY
const uint8_t UUID_UNK_D3BD[16] = { 0x80, 0x2a, 0x6d, 0x40, 0x6f, 0xf8, 0x15, 0xab,
				    0x41, 0x42, 0x1c, 0x84, 0xd2, 0x69, 0xbd, 0xd3 }; // NOTIFY
const uint8_t UUID_UNK_FDE[16] = { 0xde, 0x7f, 0xdf, 0x09, 0x8f, 0x11, 0x8f, 0x82,
				   0xad, 0x49, 0xfe, 0x89, 0xbe, 0xe9, 0x7d, 0xab }; // NOTIFY,READ
const uint8_t UUID_UNK_FDF[16] = { 0xdf, 0x7f, 0xdf, 0x09, 0x8f, 0x11, 0x8f, 0x82,
				   0xad, 0x49, 0xfe, 0x89, 0xbe, 0xe9, 0x7d, 0xab }; // WRITE NO RESPONSE

// Extra non-standard descriptor UUIDs seen alongside the CCCD (0x2902) on several notify characteristics.
// Purpose unknown; added purely to match the real device's handle count.
const uint8_t UUID_DESC_679D[16] = { 0xcb, 0x6e, 0x48, 0x80, 0xdf, 0x95, 0x57, 0x95,
				     0xee, 0x4d, 0x24, 0x5a, 0x10, 0x55, 0x9d, 0x67 };
const uint8_t UUID_DESC_B746[16] = { 0x79, 0xf9, 0xa4, 0xed, 0xbb, 0xe3, 0xd2, 0x9c,
				     0x5b, 0x49, 0x58, 0xf3, 0x8c, 0xdf, 0x46, 0xb7 };

BLEService bleSvc(UUID_SWITCH2_SERVICE);
BLECharacteristic chrInput(UUID_INPUT_REPORT);
BLECharacteristic chrUnkD5a9(UUID_UNK_D5A9);
BLECharacteristic chrCmdWrite(UUID_COMMAND_WRITE);
BLECharacteristic chrUnk65a7(UUID_UNK_65A7);
BLECharacteristic chrUnk4147(UUID_UNK_4147);
BLECharacteristic chrCmdResp(UUID_COMMAND_RESPONSE);
BLECharacteristic chrUnk640c(UUID_UNK_640C);
BLECharacteristic chrUnkD3bd(UUID_UNK_D3BD);
BLECharacteristic chrUnkFde(UUID_UNK_FDE);
BLECharacteristic chrUnkFdf(UUID_UNK_FDF);
BLECharacteristic chrVibration(UUID_VIBRATION_PRO);

BLEService bleSvc1(UUID_SERVICE1);
// Empty spacer service. Exists solely to consume ONE attribute handle (a primary service declaration is
// exactly one attribute) so the main service lands at 0x000C and its first characteristic's CCCD and
// 679d descriptor land on 0x000F and 0x0010 -- the handles the console hardcodes. Never discovered for
// content, never read, never written; its UUID just has to be unique.
const uint8_t UUID_SERVICE_SPACER[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1 };
BLEService bleSvcSpacer(UUID_SERVICE_SPACER);
BLECharacteristic chrS1R1(UUID_S1_CHAR_R1);
BLECharacteristic chrS1W(UUID_S1_CHAR_W);
BLECharacteristic chrS1R2(UUID_S1_CHAR_R2);

static uint16_t g_connHdl = BLE_CONN_HANDLE_INVALID;
static uint16_t g_hInputVal, g_hInputCccd, g_hCmdWriteVal, g_hCmdRespVal, g_hCmdRespCccd,
	g_hVibrationVal;
// ---- ADVERTISING-INTERVAL SWEEP (2026-08-13) ---------------------------------------------------
// Measured this session: the console offers us a CONNECT_IND with connInterval = 4 units = 5.0 ms in
// 142 of 144 attempts. The BLE minimum is 7.5 ms (6 units), so the SoftDevice correctly IGNORES those
// -- which is the whole explanation for "144 connection attempts, 4 connections". The two attempts
// that carried the legal 12 units (15 ms) both connected, and the real Joy-Con's single captured
// CONNECT_IND was also 12.
//
// Open question this sweep exists to answer: does the initiator's chosen connInterval track OUR
// advertising interval? If it does, we can steer the console into offering something acceptable.
// Cycled at runtime with '[' so a whole sweep costs zero reflashes. Units are 0.625 ms.
static const uint16_t ADV_INTERVAL_UNITS[4] = { 32, 48, 80, 160 }; // 20, 30, 50, 100 ms
static uint8_t g_advIntervalIdx = 0;

static uint8_t g_failThisConn = 0;   // hvx failures logged this connection (reset in bleConnectCb)
static uint32_t g_armedMs = 0;       // millis() when the console armed the stream, for failure timing
static uint8_t g_firstReports = 0;   // how many post-arm reports have been dumped in full this conn
static uint16_t g_autoTick = 0;      // button auto-press phase; RESET when the console arms the stream
static bool g_reportSentThisSlot = false; // did this 30 ms slot actually put a report on air?
static uint16_t g_imuTs12 = 0;       // synthesised 12-bit IMU sample clock; never runs backwards
static uint16_t g_hD5a9Cccd, g_hD5a9Desc; // report channel's CCCD + the 679d descriptor that starts it
// bd281's value handle. (It used to land on 0x0010 and was watched in rawBleEventCb() to catch a
// hypothetical hardcoded 0x0085 write; bd281 has since moved to ~0x0031 and no such write exists, so
// that watch is dead -- audit STALE-5.)
static uint16_t g_hS1R1Val;
static bool g_streaming = false; // set once the Switch writes 0x0085 to g_hD5a9Desc
static bool g_askedForStream = false; // one-shot per connection: gates the post-0x0C/0x04 block
// Whether to actually SEND the unprompted 0x01/0x0C. OFF by default -- it is the prime suspect for the
// supervision-timeout drop; see the send site. 'U' toggles it for A/B testing.
static bool g_askUnprompted = false;

// Advertised (and reported) vendor/product identity, cycled by 'G'.
//
// Every identity experiment in this project has kept the Joy-Con ids pinned, which forces the console
// down its FIRST-PARTY path -- where it presumably has expectations about genuine Nintendo hardware that
// we can never fully satisfy. But third-party controllers DO work on a Switch 2, and they are not
// Joy-Cons and do not pretend to be: they would declare their own ids and be handled as licensed
// accessories, under rules designed to be satisfiable by someone who is not Nintendo. Claiming to BE a
// Joy-Con may be the mistake. Same protocol, same aligned GATT layout -- only the identity changes.
//   0 = Joy-Con 2 Right (what we have always sent)
//   1 = Pro Controller 2 -- still Nintendo, different model
//   2 = HORI, a real licensed third-party accessory maker -- the actual test of the idea
// DEFAULT 1 = Pro Controller 2. This is the identity that actually registers: with it the console
// completed registration, put us in player slot 1 with a battery readout, and stopped hanging up.
// Claiming to be a Joy-Con (mode 0) puts us on the half-of-a-pair registration flow and gets us dropped
// with 0x13 about 59ms after 0x0C/0x04 -- the wall this project sat behind for months.
// It MUST be the compile-time default, not just a runtime toggle: a reflash resets it, and that silently
// undid the fix once already.
// SET TO 2 (HORI, third-party) 2026-08-09. Reason, and it is the first evidence-backed reason this option
// has ever had: the user confirms an **8BitDo controller survives alone on this console**. So the console
// has no problem holding a single controller -- "we die when we are the only link" is NOT universal
// console behavior, and the conclusion that nothing on our side can change it was WRONG.
//
// What differs is that a licensed third-party accessory declares its OWN vendor id and is handled under
// accessory rules, while modes 0 and 1 both claim to BE Nintendo hardware (0x057E) and are judged on the
// first-party path -- against expectations set by hardware Nintendo actually builds. Mode 0 pair-loops.
// Mode 1 registers and streams perfectly but will not persist as the sole link. Mode 2 has NEVER been
// tried, in the entire history of this project.
//
// If this survives alone, the whole "impostor of first-party hardware" framing was the mistake, and the
// months spent perfecting a Joy-Con impersonation were spent on the harder of two available paths.
// REVERTED to 1 the same day: mode 2 DOES NOT PAIR AT ALL. Not conclusive though -- in mode 2 we are a
// chimera. The advert and the info blob say HORI (0x0F0D/0x00C1), while the vibration UUID is still the
// Pro Controller's, and bd281, bd283, the serial and all five memory/calibration pages remain Nintendo
// Joy-Con data. A licensed accessory may also register over a path we have never implemented.
//
// So "third-party identity fails" is NOT established -- only "this half-converted version fails". Making
// it self-consistent is real work, not a constant change, and the 8BitDo evidence (it survives alone)
// still makes it the best-motivated direction we have.
static uint8_t g_idMode = 0; // WORKING DEFAULT 2026-08-13: Joy-Con 2 Right
static bool g_phy2M = true; // 'P' toggles; see requestPHY -- 2M matches real hardware but halves range
// Connection start, for the duration line printed on disconnect.
static unsigned long g_connStartMs = 0;
// (The automatic wake-mode fallback that used to live here is GONE, along with its g_cmdSeenThisConn /
// g_lastCmdOkMs bookkeeping, which had become write-only -- audit. There is no correct value for "how
// long before we forget our console"; 'C' forgets it on demand instead.)
static const uint16_t ID_VID[3] = { 0x057E, 0x057E, 0x0F0D };
static const uint16_t ID_PID[3] = { 0x2066, 0x2069, 0x00C1 };
static const char *ID_NAME[3] = { "Joy-Con 2 Right (0x057E/0x2066)", "Pro Controller 2 (0x057E/0x2069)",
				  "HORI third-party (0x0F0D/0x00C1)" };
static uint32_t g_reportsSent = 0, g_reportsFailed = 0; // per connection, reset on disconnect
static uint8_t g_hvnOutstanding = 0; // notifications queued in the SoftDevice but not yet on air
// Last time the console ACKNOWLEDGED one of our notifications, and last time it sent us anything at all.
// Together these say which side fell silent first -- the question reason 0x08 cannot answer on its own,
// and which the sniffer keeps failing to capture because it drops these long connections.
static unsigned long g_lastTxCompleteMs = 0;
static uint32_t g_txCompleteCount = 0;
static uint16_t g_connIntervalUnits = 0; // negotiated conn interval, units of 1.25ms
static unsigned long g_ackGapMaxMs = 0;  // longest gap between console acknowledgments this connection // notifications the peer actually ACKED this connection
static unsigned long g_lastRxFromConsoleMs = 0;
static unsigned long g_lastCmdReplyMs = 0; // input reports back off briefly after a command reply
static uint16_t g_reportSeq = 0; // report[0:2]: increments by exactly 1 per report actually sent
static uint16_t g_motionFrame = 0; // cursor into MOTION_REPLAY, advanced with g_reportSeq
// Input report [8] = (battery << 4) | report-mode. Sweepable with ']' -- see sendInputReport().
// 0x88 = full charge + full report mode. 0x38 is the captured real device's exact value (charge 3),
// kept in the sweep as the known-good ground-truth control.
// BATTERY, FULLY DECODED 2026-08-14 by watching the console's own icon change:
//   report[8] = (level << 1) | charging, in the HIGH nibble; low nibble = report mode (8 = full).
//     0x38 -> nibble 0b0011 -> level 1, charging SET   -> orange, nearly empty, lightning bolt
//     0x88 -> nibble 0b1000 -> level 4, charging CLEAR -> white, FULL, no bolt (the icon ANIMATES up,
//                                                        so a photo mid-fill shows half -- wait for it)
//   Level 4 IS full, so the scale is 0-4. 0xE8/0xC8/0xA8 are in the sweep only as spare probes;
//   0x88 is the value to ship.
// This only became readable once reports were notified from 0x000E -- while they came from 0x002C the
// console rendered "out of battery" no matter what this byte held.
// NOTE the captured real device sent 0x38: that unit was genuinely near-flat and on charge when it was
// captured. Copying ground truth was WRONG here -- this is a state field, not an identity field.
static const uint8_t BATTERY_BYTES[8] = { 0xE8, 0xF8, 0xC8, 0xA8, 0x88, 0x58, 0x38, 0x28 };
static uint8_t g_batteryIdx = 0;
static uint8_t g_batteryByte = 0x88; // OBSERVED FULL on the console icon -- see BATTERY_BYTES
// Millivolt-shaped battery field at report[31:33], current at [33:35]. '-' toggles, '=' cycles the
// voltage. Default ON at 4000 mV -- a healthy Li-ion cell, well clear of any low-battery threshold.
static bool g_batteryMvOn = false; // OFF, and it must stay off. [18:20] is a PACKED IMU TIMESTAMP, not a
				   // voltage -- see the long note at the write site in sendInputReport().
				   // There is NO cell-voltage field in this report: [8]'s high nibble is the whole
				   // battery signal. Both offsets tried here ([31:35] from the reference repos'
				   // fd2 layout, then [18:20]) corrupted real fields and neither was a voltage.
// Candidate u16 slots for the battery field, inside the replayed body [4:56]. [4] is a constant 0x07
// and [5:8] is the packed stick, so the hunt starts at 9. 'B' steps through them; 0 = probe off.
static const uint8_t BATT_PROBE_OFF[] = { 9,  11, 13, 15, 17, 19, 21, 23, 25, 27,
					  29, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53 };
#define BATT_PROBE_N ((uint8_t)(sizeof BATT_PROBE_OFF / sizeof BATT_PROBE_OFF[0]))
static uint8_t g_battProbe = 0;
// Motion replay on/off ('/'). Off = send the captured template body verbatim every frame.
// See the BISECT STEP 1 note in sendInputReport().
static bool g_motionOn = true;
// Which slice of the report the motion replay is allowed to write. Everything outside it keeps the
// captured template's bytes. '/' cycles the slices; see the BISECT note in sendInputReport().
// ONLY COHERENT STATES ARE OFFERED HERE, AND THAT IS DELIBERATE.
// The partial ranges that used to live in this table (halves, quarters) made the firmware emit frames
// with part of one captured sample spliced onto another's timestamp -- frames no real controller ever
// sends. **They CRASHED THE CONSOLE TWICE on 2026-08-13**, the second time merely because stepping this
// cycle forward PASSES THROUGH the spliced entries on the way to the frozen one, leaving the puck
// emitting garbage for seconds at a time. A forward-only cycle through unsafe states is a trap; the
// unsafe states are gone rather than merely discouraged.
// Both remaining entries emit real, self-consistent captured frames.
static const uint8_t REPLAY_RANGES[][2] = {
	{ 4, 56 }, // 0: full replay -- real frames, changing (normal operation)
	{ 0, 0 },  // 1: frozen -- one real frame, repeated. Reads "low battery".
};
static uint8_t g_replayIdx = 0;
static uint8_t g_replayLo = 4, g_replayHi = 56;
static const uint16_t BATTERY_MV[4] = { 4200, 4000, 3843, 3584 }; // 4200 = full cell; 3584 = real device
static uint8_t g_batteryMvIdx = 0;
static uint16_t g_batteryMv = 4200;
// 'N' toggles. DEFAULT TRUE -- force the report stream on rather than waiting for the console.
//
// Must be armed at connect: the SoftDevice latches notification state at connection setup, so enabling
// mid-connection returns INVALID_STATE and silently does nothing.
//
// Why forced at all: the console never sends a stream-start trigger (measured 2026-08-09 -- it sends
// none to a real controller either; a real Joy-Con simply begins streaming ~810ms after the handshake).
// The 'N' control test answered the obvious objection: registered but completely SILENT, the console
// still dropped both us and a real Joy-Con. So our report content is exonerated -- but note the console
// has never been shown to CONSUME our stream either (the stick test moved nothing on screen).
static bool g_forceStream = true;
// 'Y' cycles the 0x0A/0x08 reply: 0 ack, 1 silent, 2 01000000, 3 00000000, 4 echo.
// DEFAULT IS 0 (bare 8-byte ack) -- CONFIRMED against a real Joy-Con being registered from scratch
// (2026-08-08 capture, frame 2425): the real controller answers `0a 01 01 08 10 78 00 00`, byte-identical
// to what mode 0 produces. Earlier today I made mode 1 the default after observing that replying died at
// ~6.2s while silence held the link for 136s -- that read the causality backwards. Silence does not fix
// anything; it PARKS the console before `0x11/0x01`, so it never reaches `0x0C/0x04` and never opens the
// report stream. The long "stable" connections were the console stalled, not healthy.
static uint8_t g_a08Mode = 0;

// The console we last bonded with, wire order, persisted across reboots. Drives whether we advertise in
// pairing mode or wake mode -- see buildAdvertising().
static uint8_t g_bondedHost[6];
static bool g_haveBondedHost = false;

static void saveBondedHost(const uint8_t *addr6)
{
	memcpy(g_bondedHost, addr6, 6);
	g_haveBondedHost = true;
	uint8_t rec[7] = { 0x46, addr6[0], addr6[1], addr6[2], addr6[3], addr6[4], addr6[5] };
	// writeFileInPlace(), NOT remove()+create(). This runs on every successful registration, on the same
	// partition the bond key lives on, and remove+create is the pattern documented above as having killed
	// this filesystem repeatedly -- which then takes the bond store with it and looks like a protocol fault.
	// Missed when writeFileInPlace() was introduced.
	if (!writeFileInPlace("/bondedhost.bin", rec, sizeof rec))
		Serial.println("# LEAN bonded-host write FAILED -- wake mode will not survive a reboot");
	Serial.printf("# LEAN bonded host saved: %02X:%02X:%02X:%02X:%02X:%02X\n", addr6[5], addr6[4],
		      addr6[3], addr6[2], addr6[1], addr6[0]);
}

static void loadBondedHost()
{
	File f(InternalFS);
	uint8_t rec[7];
	if (f.open("/bondedhost.bin", FILE_O_READ)) {
		if (f.read(rec, sizeof rec) == sizeof rec && rec[0] == 0x46) {
			memcpy(g_bondedHost, rec + 1, 6);
			g_haveBondedHost = true;
		}
		f.close();
	}
}
static bool g_wakeArmed = false; // set by the 'H' console command
// Hold the reconnect report stream until phase 6 finishes (0x0C/0x04) instead of opening it at connect.
// Streaming through the handshake is what made every first command reply fail and cost 10s per retry.
// Still a guaranteed start, just a later one -- so a reconnected controller can never sit silent, which
// is the failure the immediate start was originally protecting against.
static bool g_deferStreamToPhase6 = true;

// BUTTON MAP -- MEASURED, not inferred (2026-08-07). Connected to a real Joy-Con 2 Right as BLE central
// with bleak (scratchpad/jc_buttons.py), sent CMD_FEATURE INIT+ENABLE to turn on full reporting, then
// logged reports while each button was held ~2.5s. Bit positions read straight off the transitions:
//
//   byte[3] 0x80 = SL     byte[3] 0x40 = SR     byte[2] 0x01 = B     byte[3] 0x01 = HOME
//
// This does NOT match either PC reference repo, which put the field at [4:8]
// with SL=0x20/SR=0x10. Switch 2 uses different positions -- which is why every guess failed. Note byte[3]
// had been assumed part of a 4-byte timestamp; the timestamp is really [0:2].
//
// Also required: a Joy-Con only streams buttons AFTER CMD_FEATURE INIT(0x02) then ENABLE(0x04) with flags
// 0x37. Before that it sends a minimal report with everything past byte 8 zeroed -- which is why three
// earlier logging attempts showed no button activity at all.
#define BTN_OFFSET 2 // buttons are a little-endian field starting at report byte 2
#define BTN_B 0x00000001UL // byte[2] bit0
#define BTN_HOME 0x00000100UL // byte[3] bit0
#define BTN_SR 0x00004000UL // byte[3] bit6
#define BTN_SL 0x00008000UL // byte[3] bit7
static uint8_t g_buttonOffset = BTN_OFFSET;
// Registration ("press L and R") wants the rail buttons on a single Joy-Con. Deliberately NOT B: B is
// cancel on the Change Grip/Order screen, so including it asks to register and to back out at once.
// DEFAULT: press NOTHING. The map below was measured from a Joy-Con 2 Right, but we now present as a Pro
// Controller 2 (which is what finally got us registered), and a Pro Controller has a completely different
// button layout -- so these bits are unknown buttons on the device we are claiming to be. Holding two
// unknown buttons down ~2.5 times a second forever is not a neutral baseline; if either is HOME or a
// system button we are spamming the console continuously. Be a quiet, well-behaved controller until the
// Pro Controller map is measured the same way the Joy-Con's was (docs/captures/jc_buttons.py). 'M' cycles.
// Default changed to SL+SR on 2026-08-13. Be careful what you conclude from that -- the causal claim
// I first wrote here was WRONG and was falsified within two minutes.
//
// What was actually measured, and nothing more: with the mask at 0 the console ran an "out of battery"
// loop (~1.6 s per connection, reason=0x13, a deliberate hangup). Enabling SL+SR was followed by a held
// connection -- 782 reports over 25 s at ~31/s, zero reconnects, zero hvx failures, battery rendered as
// CHARGING. Then, with NOTHING CHANGED, it went back to the ~1.5 s cycle.
//
// So the link alternates between holding and cycling under an unchanged configuration, and the mask is
// NOT a reliable fix. It is defaulted on because a stream where a button occasionally moves is closer to
// a real controller than a wholly inert one, not because it is known to keep the link up.
//
// SIX battery/stability hypotheses were chased and falsified on 2026-08-13 (report[8] nibble values,
// millivolts at [31:35], a raised field at [9:11), the BLE-link-count matrix, the Nintendo OUI, and this
// mask). Whatever gates link lifetime has not been found. Do not add a seventh guess to this list
// without a way to tell a real effect from the alternation.
static uint32_t g_buttonMask = BTN_SL | BTN_SR;
static bool g_homeHeld = false;

// ---- response builder: [cmd_id][0x01][0x01][subcmd_id][typeByte][0x78][0x00][0x00][payload...]
// CORRECTED 2026-08-06 against real ground truth: byte-decoded all 5 notifications a genuine Joy-Con 2
// sends during fresh pairing (docs/captures/joycon2_pairing_DECRYPTED_ltk2.pcap, frames 513-524). The old
// framing here ([cmd][0x01][subcmd][0x00][len]...) was wrong on bytes 2-3 (swapped) and invented bytes 4-7
// entirely -- the real shape is [cmd][0x01][0x01][subcmd][typeByte][0x78][0x00][0x00][payload...], where
// typeByte is 0x10 for every real sample except the MEMORY-read response, which uses 0x20.
static void sendResponseOn(BLECharacteristic &chr, uint8_t cmd, uint8_t subcmd, const uint8_t *payload,
			    uint8_t len)
{
	uint8_t buf[8 + 80];
	if (len > 80)
		len = 80;
	// CORRECTED 2026-08-06: typeByte is 0x10 for EVERY notification including the MEMORY read.
	// The old "0x20 for MEMORY" special case came from exactly one packet -- frame 515 of
	// joycon2_pairing_DECRYPTED_ltk2.pcap -- which has a BAD CRC (`nordic_ble.crcok == 0`, tshark
	// reports "CRC is bad"). Every one of that frame's 7 byte differences from the two good
	// captures is a single-bit flip, 0x10 -> 0x20 among them. The clean left-Joy-Con capture, the
	// second right-Joy-Con capture, and the fully-decrypted reconnect capture all send 0x10 here.
	const uint8_t typeByte = 0x10;
	buf[0] = cmd;
	buf[1] = 0x01;
	buf[2] = 0x01;
	buf[3] = subcmd;
	buf[4] = typeByte;
	buf[5] = 0x78;
	buf[6] = 0x00;
	buf[7] = 0x00;
	if (len)
		memcpy(buf + 8, payload, len);
	// notify()'s return value was previously discarded entirely -- meaning every prior test round has been
	// ASSUMING all 8 announcement notifications actually went out, never confirmed. notify() returns false
	// if e.g. the peer hasn't subscribed that CCCD yet, or the HVN TX queue is full -- either would silently
	// drop a notification with zero visibility, indistinguishable from "sent successfully but ignored."
	bool ok = chr.notify(buf, 8 + len);
	// Count it. HVN_TX_COMPLETE decrements g_hvnOutstanding for every completed notification --
	// command replies included -- so not incrementing here made the counter drift downward on each
	// handshake and let the report-path gate through more than it should.
	if (ok && g_hvnOutstanding < 255)
		g_hvnOutstanding++;
	g_lastCmdReplyMs = millis();
	// A FAILED notify always prints -- that is the interesting case. Successful ones are gated: the
	// console repeats the whole handshake every ~2 s in the loop and each reply logged a line.
	if (!g_quiet || !ok)
		Serial.printf("# LEAN notify: cmd=0x%02X sub=0x%02X len=%u ok=%d t=%lu ms\n", cmd, subcmd,
			      8 + len, ok, (unsigned long)millis());
}

static void sendResponse(uint8_t cmd, uint8_t subcmd, const uint8_t *payload, uint8_t len)
{
	sendResponseOn(chrCmdResp, cmd, subcmd, payload, len);
}

// Pack two 12-bit stick values into the 3-byte format get_stick_xy() expects: value = (y<<12)|x, LE.
static void packStickXY(uint8_t *out3, uint16_t x12, uint16_t y12)
{
	uint32_t v = ((uint32_t)(y12 & 0xFFF) << 12) | (x12 & 0xFFF);
	out3[0] = (uint8_t)(v & 0xFF);
	out3[1] = (uint8_t)((v >> 8) & 0xFF);
	out3[2] = (uint8_t)((v >> 16) & 0xFF);
}

// Byte-exact template of a real Joy-Con 2 Right's CONTROLLER_INFO content. Only bytes [2:16] (serial) are
// customized below; everything else -- including several bytes whose meaning is still unknown (content[0],
// content[22:25]) and a 27-byte 0xFF tail from content[37] on (matches unwritten/erased flash convention)
// -- is copied byte-for-byte from a real device rather than guessed at. vendor/product (content[18:22])
// happen to already match NINTENDO_VENDOR_ID/JOYCON2_RIGHT_PID exactly since the source device IS that
// model.
//
// CORRECTED 2026-08-06 (second pass): this was originally transcribed from frame 515 of
// docs/captures/joycon2_pairing_DECRYPTED_ltk2.pcap -- a frame with a BAD CRC. It is the only bad-CRC ATT
// frame in that whole capture, and it was the sole source for this array. Re-derived against the two clean
// captures (joycon2_pairing2_DECRYPTED_8notif.pcap frame 3932 and the decrypted reconnect capture, which
// agree byte-for-byte with each other), the corrupted bytes were: content[4]/[5] 0x55,0x3b -> 0x57,0x37
// (inside the serial, overwritten below anyway), content[17] 0x01 -> 0x00, content[31] 0xfb -> 0xff, and
// content[38] 0xdf -> 0xff. All are single-bit flips, as corruption looks. Note content[17] and content[38]
// had been specifically written up as "meaningful unknown bytes we must replicate exactly" -- they were
// bit errors.
static const uint8_t REAL_CONTROLLER_INFO_TEMPLATE[64] = {
	0x01, 0x00, 0x48, 0x43, 0x50, 0x37, 0x31, 0x30, 0x39, 0x39, 0x38, 0x38, 0x37, 0x37, 0x30,
	0x31, 0x00, 0x00, 0x7e, 0x05, 0x66, 0x20, 0x01, 0x08, 0x02, 0x32, 0x32, 0x32, 0xaa, 0xaa,
	// Third color was 0xff,0x8c,0x5f -- the Joy-Con 2 Right's ORANGE, and the most conspicuously
	// wrong thing left in a blob we serve while claiming to be a (black) Pro Controller 2. The four
	// color slots are confirmed as data[25:28], [28:31], [31:34], [34:37] by the reference's own
	// ControllerInfo.from_bytes(), so this is the accent color, not a coincidence. Neutralised to the
	// same dark grey as the other slots. Revert these three bytes first if identity is ever suspected.
	0xaa, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff
};
// ---- CONTROLLER_INFO REGION BISECT ----------------------------------------------------------
// Established 2026-08-11: perturbing 16 bytes of this blob makes the console complete the PAIR
// ceremony, encrypt, then DELIBERATELY hang up (reason=0x13) ~13ms later -- and retry in a loop.
// Serving the captured original instead restores normal bring-up. So the console VALIDATES this
// blob. It is the only lever found so far that provably changes the console's behavior.
//
// The failing variant perturbed everything EXCEPT [2:16] (serial), [18:22] (vendor/product) and
// [37:64] (0xFF tail). So at least one byte in [0:2], [16:18], [22:25] or [25:37] is load-bearing.
// This builds the served blob at runtime, perturbing exactly ONE region so a single connection
// isolates it. Cycle with '9'.
//
// Region 0 = untouched original (the known-good baseline -- always re-verify it between tests).
// Region 5 = every candidate region at once (the known-BAD case; reproduces the rejection).
static uint8_t g_infoRegion = 0;
static const char *const INFO_REGION_NAME[6] = {
	"0 none (original, known GOOD)", "1 [0:2]", "2 [16:18]", "3 [22:25]", "4 [25:37]",
	"5 all candidates (known BAD)"
};
// CONTROLLER COLORS. `[25:37]` decoded 2026-08-11 as FOUR RGB TRIPLES, confirmed on screen:
//   [25:28] body   [28:31] buttons   [31:34] left grip   [34:37] right grip
// The captured original is body/grips 0x323232 (near-black) with 0xAAAAAA buttons. These are OURS --
// distinct per slot so each one is individually identifiable on screen at a glance.
// NOTE the deliberate avoidance of 0x00 and 0xFF. A first attempt used pure FF0000/00FF00/0000FF/
// FFFF00 and the icon stayed BLACK -- while region 4's perturbation (0x3C434A etc.) had visibly
// changed it through this same code path. The difference is the sentinels: 0xFF is this protocol's
// erased-flash "no data" convention throughout (see MEM_001FC040 and the 0xFF tails), so a channel
// of 0xFF or 0x00 is likely read as UNSET and the console falls back to its default black.
// These stay vivid and unmistakable while keeping every channel a real value.
// Slot mapping, confirmed on screen 2026-08-11 with a red/green/blue/yellow test pattern:
//   [25:28] body    [28:31] buttons    [31:34] top accent bar    [34:37] BOTH grips
// (the third slot is an accent strip, not a left grip, and the fourth colors both grips together.)
//
// GAMECUBE INDIGO. The iconic combination is the indigo body, the green A button and the yellow
// C-stick, so those get the three visible slots. No channel is 0x00 or 0xFF -- that is this
// protocol's erased-flash "no data" convention elsewhere, so it is avoided on principle.
static const uint8_t COLORS_RGBY[12] = {
	0x5c, 0x4b, 0x99, // body      INDIGO   -- the GameCube purple
	0x43, 0xb0, 0x2a, // buttons   GREEN    -- the A button
	0xe8, 0xc0, 0x22, // accent    YELLOW   -- the C-stick
	0x4a, 0x3d, 0x80  // grips     INDIGO   -- slightly darker than the body
};
static bool g_ownColors = true; // '0' toggles back to the captured original colors
static void stampColors(uint8_t *buf)
{
	if (!g_ownColors) {
		Serial.println("# LEAN colors: serving the CAPTURED ORIGINAL (g_ownColors=0)");
		return;
	}
	memcpy(buf + 25, COLORS_RGBY, sizeof COLORS_RGBY);
	// Print what actually goes on the wire. Guessing at this cost several wrong conclusions:
	// the toggle's own message was stale after a color change, so only a dump of the real bytes
	// is trustworthy.
	Serial.printf("# LEAN colors served: body %02X%02X%02X btn %02X%02X%02X "
		      "accent %02X%02X%02X grips %02X%02X%02X\n",
		      buf[25], buf[26], buf[27], buf[28], buf[29], buf[30],
		      buf[31], buf[32], buf[33], buf[34], buf[35], buf[36]);
}
// Same perturbation the failing variant used, so a positive result reproduces it exactly.
static uint8_t infoPerturb(uint8_t b, uint8_t i)
{
	uint8_t v = (uint8_t)(b + 0x5b + i * 7);
	return (v == 0x00 || v == 0xff) ? 0x5a : v;
}
// FORCE RE-REGISTRATION ("refresh"). The console CACHES this blob when it registers a controller and
// will not re-render from later reads -- which is why color changes appeared to do nothing until a
// registration was forced. Poisoning the validated [0:2] header makes the console reject us
// (reason=0x13, ~13ms after encryption) and DROP its cached entry; when the poison lifts, the next
// connection is a genuine fresh registration that re-reads everything.
//
// User's idea, 2026-08-11, after noticing that break-then-restore brought the new colors in.
// Non-blocking: '.' arms it for a few seconds and loop() lets it expire on its own.
static unsigned long g_poisonUntilMs = 0;
// BYTE-0 SWEEP. 0xFFFF = inactive (serve the original 0x01). Any other value is written straight
// into CONTROLLER_INFO[0]. ';' steps through the candidates.
// Outcomes and what they mean:
//   only 0x01 accepted            -> magic number or version, NOT a boolean
//   0x00 and 0x01 both accepted   -> boolean
//   0x02/0x03 also accepted       -> enum or count (and the value may select something)
static uint16_t g_byte0Override = 0xFFFF;
static uint16_t g_byte1Override = 0xFFFF; // byte 1 has NEVER been tested: it is 0x00 and the
					  // perturbation skipped every 0x00 byte.
static const uint8_t SWEEP_CAND[6] = { 0x00, 0x02, 0x03, 0x04, 0x10, 0x01 }; // ends back on 0x01
static uint8_t g_sweepIdx = 0;
static const uint8_t *buildInfoBlob()
{
	static uint8_t buf[sizeof REAL_CONTROLLER_INFO_TEMPLATE];
	memcpy(buf, REAL_CONTROLLER_INFO_TEMPLATE, sizeof buf);
	if (g_poisonUntilMs && millis() < g_poisonUntilMs) {
		buf[0] = infoPerturb(buf[0], 0); // [1] is 0x00 and would be skipped anyway
		stampColors(buf);
		return buf;
	}
	if (g_byte0Override != 0xFFFF)
		buf[0] = (uint8_t)g_byte0Override;
	if (g_byte1Override != 0xFFFF)
		buf[1] = (uint8_t)g_byte1Override;
	if (g_infoRegion == 0) {
		stampColors(buf);
		return buf;
	}
	for (uint8_t i = 0; i < sizeof buf; i++) {
		bool sel = false;
		switch (g_infoRegion) {
		case 1: sel = (i < 2); break;
		case 2: sel = (i >= 16 && i < 18); break;
		case 3: sel = (i >= 22 && i < 25); break;
		case 4: sel = (i >= 25 && i < 37); break;
		case 5: sel = (i < 2) || (i >= 16 && i < 18) || (i >= 22 && i < 25) || (i >= 25 && i < 37);
			break;
		}
		if (sel && buf[i] != 0x00)
			buf[i] = infoPerturb(buf[i], i);
	}
	stampColors(buf);
	return buf;
}

// The array is [64] but only had 63 initialisers, so byte 63 was C-zero-filled to 0x00 while a real
// device sends 0xFF -- caught 2026-08-08 by diffing against the good-CRC first-time-registration
// capture. The whole tail is meant to be 0xFF (erased-flash convention); one byte was simply missed.

// ControllerInfo.from_bytes layout (relative to content start): serial=data[2:16], vendor=data[18:20],
// product=data[20:22], colors=data[25:28],[28:31],[31:34],[34:37]. Shared by handleMemoryRead() (replying
// to an explicit Switch request) and the proactive fresh-pairing announcement (sendProactiveAnnouncement())
// -- both need the same content shape, just wrapped in different outer framing.
static void buildControllerInfoContent(uint8_t *content, uint8_t contentLen)
{
	uint8_t copyLen = contentLen > sizeof(REAL_CONTROLLER_INFO_TEMPLATE) ?
				   (uint8_t)sizeof(REAL_CONTROLLER_INFO_TEMPLATE) :
				   contentLen;
	// g_infoRegion selects which region (if any) is perturbed -- see buildInfoBlob().
	memcpy(content, buildInfoBlob(), copyLen);
	if (contentLen > copyLen)
		memset(content + copyLen, 0xFF, contentLen - copyLen);
	const char *serial = g_useCapturedIdentity ? SERIAL_CAPTURED : SERIAL_SYNTHETIC;
	if (contentLen >= 16)
		memcpy(content + 2, serial, 14);
	// Keep the REPORTED vendor/product in step with the ADVERTISED ones ('G'). content[18:20] is the
	// vendor id and content[20:22] the product id, little-endian, in the 0x00013000 info blob. Letting
	// these disagree with the advertisement would be a confound of its own making.
	if (contentLen >= 22) {
		content[18] = (uint8_t)(ID_VID[g_idMode] & 0xFF);
		content[19] = (uint8_t)(ID_VID[g_idMode] >> 8);
		content[20] = (uint8_t)(ID_PID[g_idMode] & 0xFF);
		content[21] = (uint8_t)(ID_PID[g_idMode] >> 8);
	}
}

// Fills `content` for a MEMORY read at `addr`. Shared by every path that answers one: the plain
// COMMAND_WRITE handler, the chrUnk65a7 handshake handler, and the proactive announcement -- they only
// differ in framing, never in content, and they used to each carry their own partial copy of this logic.
static void buildMemoryContent(uint32_t addr, uint8_t *content, uint8_t contentLen)
{
	memset(content, 0, contentLen);

	const uint8_t *blob;
	uint8_t blobLen;
	if (addr == ADDR_CONTROLLER_INFO) {
		buildControllerInfoContent(content, contentLen);
	} else if (lookupMemory(addr, &blob, &blobLen)) {
		// Captured blob for one of the addresses the Switch reads during post-encryption bring-up.
		memcpy(content, blob, contentLen < blobLen ? contentLen : blobLen);
		if (contentLen > blobLen) {
			// Silent-hole guard. lookupMemory() MATCHED, so the UNKNOWN-addr warning cannot fire and
			// a short blob would pad with 0xFF unnoticed. Verified against session logs: the console
			// asks for exactly the blob length every time (0x13040 len=16, 0x13060 len=32,
			// 0x13100 len=24), so this should never print. If it does, that blob needs page-sizing.
			Serial.printf("# LEAN *** SHORT BLOB addr=0x%08lX: asked %u, have %u, padding %u ***\n",
				      (unsigned long)addr, contentLen, blobLen,
				      (unsigned)(contentLen - blobLen));
			memset(content + blobLen, 0xFF, contentLen - blobLen);
		}
		// NOTE (audit DEAD-1): the two branches below are UNREACHABLE for anything the console
		// actually asks for -- it reads 64-byte PAGES (0x13080, 0x130C0, 0x1FC040) which lookupMemory()
		// catches first, never the item addresses these test for. Kept only for a direct item-address
		// read, which has never been observed.
	} else if (addr == ADDR_CALIB_USER_JOYSTICK_1 || addr == ADDR_CALIB_USER_JOYSTICK_2) {
		// First 3 bytes = 0xFFFFFF forces a fallback to factory calibration.
		content[0] = 0xFF;
		content[1] = 0xFF;
		content[2] = 0xFF;
	} else if (addr == ADDR_CALIB_JOYSTICK_1 || addr == ADDR_CALIB_JOYSTICK_2) {
		// Neutral, symmetric calibration: center=2048, max/min deflection=2048 (full 0..4095, centered).
		packStickXY(content + 0, 2048, 2048);
		packStickXY(content + 3, 2048, 2048);
		packStickXY(content + 6, 2048, 2048);
	} else {
		// Unknown address. 0xFF (erased-flash convention) is a safer default than zeros -- the
		// all-zero controller-info tail was previously confirmed to look anomalous to the Switch.
		memset(content, 0xFF, contentLen);
		Serial.printf("# LEAN memory read: UNKNOWN addr=0x%08lX -- answering 0xFF fill\n",
			      (unsigned long)addr);
	}
}

static void handleMemoryRead(const uint8_t *data, uint8_t len)
{
	// Request payload (from read_memory()): [length][0x7e][0x00][0x00][addr LE 4 bytes]
	if (len < 8)
		return;
	uint8_t reqLen = data[0];
	uint32_t addr = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
			((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
	Serial.printf("# LEAN memory read: addr=0x%08lX len=%u\n",
		      (unsigned long)addr, reqLen);

	uint8_t inner[8 + 64];
	inner[0] = reqLen;
	// Bytes 1-3 are all zero. (History: originally 0x7e,0x00,0x00 by mirroring the REQUEST framing, then
	// "corrected" to 0x00,0x01,0x00 from the bad-CRC frame 515 -- see REAL_CONTROLLER_INFO_TEMPLATE above.
	// The two clean captures and the decrypted reconnect capture all show 0x00,0x00,0x00; the 0x01 was the
	// same single-bit corruption.)
	inner[1] = 0x00;
	inner[2] = 0x00;
	inner[3] = 0x00;
	inner[4] = (uint8_t)(addr & 0xFF);
	inner[5] = (uint8_t)((addr >> 8) & 0xFF);
	inner[6] = (uint8_t)((addr >> 16) & 0xFF);
	inner[7] = (uint8_t)((addr >> 24) & 0xFF);
	uint8_t *content = inner + 8;
	uint8_t contentLen = reqLen > 64 ? 64 : reqLen;
	memset(content, 0, contentLen);

	buildMemoryContent(addr, content, contentLen);

	sendResponse(CMD_MEMORY, SUB_MEMORY_READ, inner, (uint8_t)(8 + contentLen));
}

static void chrCmdWriteCb(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data,
			  uint16_t len)
{
	(void)conn_hdl;
	(void)chr;
	// Request framing (build_command): [cmd_id][0x91][0x01][subcmd_id][0x00][len][0x00][0x00][data...]
	if (len < 8)
		return;
	uint8_t cmd = data[0];
	uint8_t subcmd = data[3];
	uint8_t dlen = data[5];
	const uint8_t *payload = data + 8;
	uint16_t avail = (len > 8) ? (len - 8) : 0;
	if (dlen > avail)
		dlen = (uint8_t)avail;

	logCmd(cmd, subcmd);

	switch (cmd) {
	case CMD_MEMORY:
		if (subcmd == SUB_MEMORY_READ)
			handleMemoryRead(payload, dlen);
		break;
	case CMD_LEDS:
		Serial.printf("# LEAN LEDs set (sub=0x%02X)\n", subcmd);
		sendResponse(cmd, subcmd, nullptr, 0);
		break;
	case CMD_FEATURE:
		Serial.printf("# LEAN feature (sub=0x%02X flags=0x%02X)\n", subcmd,
			      dlen ? payload[0] : 0);
		// FOUR ZERO BYTES, not an empty ack -- measured 2026-08-09 against a real Joy-Con that
		// actually complied (LED lit, motor blipped). This handler is dormant (the console writes
		// 65a7, never COMMAND_WRITE) but it was a second, divergent copy of the protocol that would
		// have answered wrongly the moment anything used it (audit BUG-4).
		sendResponse(cmd, subcmd, RSP_FEATURE, sizeof RSP_FEATURE);
		break;
	case CMD_VIBRATION:
		sendResponse(cmd, subcmd, nullptr, 0);
		break;
	case CMD_PAIR:
		if (subcmd == SUB_PAIR_SET_MAC && dlen >= 8) {
			Serial.printf(
				"# LEAN PAIR SET_MAC: host=%02X:%02X:%02X:%02X:%02X:%02X\n",
				payload[7], payload[6], payload[5], payload[4],
				payload[3], payload[2]);
		} else {
			Serial.printf("# LEAN PAIR sub=0x%02X\n", subcmd);
		}
		sendResponse(cmd, subcmd, nullptr, 0);
		break;
	default:
		Serial.printf("# LEAN unhandled cmd=0x%02X sub=0x%02X len=%u\n", cmd,
			      subcmd, dlen);
		sendResponse(cmd, subcmd, nullptr, 0);
		break;
	}
}

static void chrVibrationCb(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data,
			   uint16_t len)
{
	(void)conn_hdl;
	(void)chr;
	// LOG IT. This was a silent no-op, and the raw GATTS logger in rawBleEventCb() explicitly EXCLUDES
	// the vibration handle -- so a rumble write from the console left no trace anywhere. On 2026-08-09
	// that blind spot produced a stated conclusion ("the console has never written our vibration
	// characteristic") derived from a handle census that could not have shown it. We have no motor, but
	// we must at least know when we are being told to use one.
	Serial.printf("# LEAN VIBRATION write: len=%u data=", len);
	for (uint16_t i = 0; i < len && i < 24; i++)
		Serial.printf("%02X", data[i]);
	Serial.println();
}

static void cccdCb(uint16_t conn_hdl, BLECharacteristic *chr, uint16_t cccd_value);
static void readAuthCb(uint16_t conn_hdl, BLECharacteristic *chr, ble_gatts_evt_read_t *request);

// Generic no-op write callback for the "unknown purpose" characteristics -- just logs so a future capture
// can tell us if the Switch ever actually writes to one of these once the rest of the handshake works.
static void unkWriteCb(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
	(void)conn_hdl;
	Serial.printf("# LEAN unknown-char write: handle=0x%04X len=%u data=", chr->handles().value_handle,
		      len);
	for (uint16_t i = 0; i < len; i++)
		Serial.printf("%02X", data[i]);
	Serial.println();
}

// Which characteristic a manually-triggered reply goes out on -- selectable at runtime via the 'R' console
// command, so each candidate can be tried without a reflash cycle.
static uint8_t g_handshakeReplyChannel = 0; // 0=COMMAND_RESPONSE, 1=chrUnk640c, 2=chrUnkD3bd
static bool g_skipLtk2 = false; // toggled via 'K' -- test whether LTK2's presence/content matters at all
static const char *const HANDSHAKE_CHANNEL_NAME[] = { "COMMAND_RESPONSE", "chrUnk640c",
							"chrUnkD3bd" };

// Pending-reply state for commands we do NOT auto-answer. Known-safe commands (0x07/0x01, 0x02/0x04,
// 0x10/0x01, 0x16/0x01) ARE auto-answered -- see the knownSafe list in chrHandshakeCb.
// History: replies were manual-only for a while, after auto-replying with wrong-shaped content twice
// correlated with the console itself crashing. That turned out to be the CONTENT, not the channel --
// well-formed captured frames are safe to send. Anything unrecognized still waits for 'A'.
static bool g_handshakePendingReply = false;
static uint8_t g_pendingCmd = 0, g_pendingSubcmd = 0;
static uint8_t g_pendingRaw[64]; // 64 to match chrUnk65a7's raised setMaxLen -- was 33, which silently
				  // truncated the 42-byte PAIR/LTK1 and PAIR/LTK2 writes
static uint16_t g_pendingRawLen = 0;

static void sendHandshakeReply(uint8_t cmd, uint8_t subcmd, const uint8_t *raw, uint16_t rawLen);

// ============================================================================================
// Link-layer key derivation -- SOLVED 2026-08-06 by decrypting docs/captures/
// joycon2_reconnect_undecrypted.pcap (315/315 packets, zero MIC failures; see
// docs/captures/ble_decrypt.js for the tool and the full derivation).
//
//   LTK = reverse_bytes( <Switch's PAIR/LTK1 body> XOR <our PAIR/LTK1 body> )
//
// where each "body" is the 16 bytes AFTER the 1-byte marker of a CMD=0x15 / sub=0x04 message.
// The reversal produces the MSO-first form AES wants; the SoftDevice's ble_gap_enc_info_t::ltk
// is documented in SMP wire order (least-significant octet first), so what we hand it is the
// XOR result WITHOUT the reversal -- the two byte-order flips cancel. That reasoning is sound
// but unverified against real silicon, so 'O' flips it at runtime rather than needing a
// reflash to test the other order.
//
// The real LL_ENC_REQ carries EDIV=0 and Rand=0 -- an out-of-band key, no SMP, no key lookup.
// So the GATT PAIR ceremony IS the key agreement. The long-standing belief that these bytes
// were "decorative" (and that PAIR_LTK2 above was a universal link key) was wrong: the three
// "*_DECRYPTED_*" captures never contained an LL_ENC_REQ and therefore were never decrypted at
// all -- every readable byte in them is pre-encryption plaintext.
//
// Delivery problem this has to work around: Bluefruit answers BLE_GAP_EVT_SEC_INFO_REQUEST
// itself, from BLESecurity::_eventHandler(), which bluefruit.cpp dispatches BEFORE the user
// event callback rawBleEventCb() -- with sd_ble_gap_sec_info_reply(conn, NULL, NULL, NULL)
// when no bond is stored, which rejects encryption. Replying from rawBleEventCb() would always
// lose that race. So instead we pre-load the key into Bluefruit's own bond store the moment we
// can derive it (on the Switch's PAIR/LTK1 write, ~2 command exchanges / ~60-90ms before
// LL_ENC_REQ), and let Bluefruit's existing loadBondKey() path serve it.
// ============================================================================================
static bool g_haveLinkKey = false;
static bool g_ltkReverse = false; // toggled by 'O' -- try the opposite byte order in the bond store
// The link key in AES/most-significant-octet-first order. Needed separately from the bond-store copy
// because the PAIR/LTK2 challenge-response below feeds it straight to the AES engine.
static uint8_t g_linkKeyMsb[16];
static uint8_t g_linkKeyWire[16]; // same key in GATT wire order, for the bond-store verify below

static void deriveLinkKey(const uint8_t *switchLtk1Body)
{
	// MUST be static, not a stack local. Bluefruit's bond_save_keys() defers the actual flash write via
	// ada_callback(NULL, 0, bond_save_keys_dfr, role, conn_hdl, bkeys) -- passing NULL/0 for the data means
	// it does NOT copy the struct, it just keeps the pointer and dereferences it later, from another task.
	// With a stack local this frame is long gone by then, so garbage got written, loadBondKey() failed, and
	// Bluefruit answered SEC_INFO_REQUEST with NULL keys -- which the sniffer showed as our peripheral
	// sending LL_REJECT_EXT_IND right after LL_ENC_RSP. (saveBondKey() returns true unconditionally, so its
	// return value never revealed any of this.)
	static bond_keys_t bk;
	memset(&bk, 0, sizeof bk);

	// XOR in wire order (least-significant octet first, as both halves arrive over GATT).
	uint8_t wire[16];
	for (uint8_t i = 0; i < 16; i++)
		// MUST match the array announced below -- derivation and announcement cannot diverge.
		wire[i] = switchLtk1Body[i] ^ (g_ownKeyMaterial ? ANNOUNCED_LTK1_OWN : ANNOUNCED_LTK1)[1 + i];
	for (uint8_t i = 0; i < 16; i++)
		g_linkKeyMsb[i] = wire[15 - i];
	memcpy(g_linkKeyWire, wire, 16);

	uint8_t key[16];
	memcpy(key, wire, 16);
	if (g_ltkReverse)
		memcpy(key, g_linkKeyMsb, 16);

	memcpy(bk.own_enc.enc_info.ltk, key, 16);
	bk.own_enc.enc_info.ltk_len = 16;
	bk.own_enc.enc_info.auth = 0; // unauthenticated -- no SMP ever happens on this link
	bk.own_enc.enc_info.lesc = 0;
	bk.own_enc.master_id.ediv = 0; // matches the real LL_ENC_REQ (EDIV=0, Rand=0)
	memset(bk.own_enc.master_id.rand, 0, sizeof bk.own_enc.master_id.rand);

	Serial.printf("# LEAN derived link key (%s order): ", g_ltkReverse ? "reversed" : "as-XORed");
	for (uint8_t i = 0; i < 16; i++)
		Serial.printf("%02x", key[i]);
	Serial.println();

	BLEConnection *conn = Bluefruit.Connection(g_connHdl);
	if (!conn) {
		Serial.println("# LEAN link key NOT installed -- no connection object");
		return;
	}
	bk.peer_id.id_addr_info = conn->getPeerAddr();

	// Write Bluefruit's bond file OURSELVES, synchronously. conn->saveBondKey() routes through
	// bond_save_keys() -> ada_callback(), i.e. a deferred write on another task, and live testing showed it
	// never lands inside the ~150ms between PAIR/LTK1 and the Switch's LL_ENC_REQ -- the read-back check
	// below failed on every single cycle. Bluefruit's SEC_INFO_REQUEST handler then answers with NULL keys
	// and the SoftDevice sends LL_REJECT_EXT_IND (seen on the sniffer right after LL_ENC_RSP).
	//
	// The on-disk format is trivial: each field is a 1-byte length followed by that many bytes, and
	// bond_load_keys() only reads the first field, so [len][bond_keys_t] plus an empty name is enough.
	// Path/format from utility/bonding.h: BOND_FNAME_PRPH "/adafruit/bond_prph/%02X..." over addr[0..5].
	ble_gap_addr_t peer = conn->getPeerAddr();
	char fname[48];
	snprintf(fname, sizeof fname, "/adafruit/bond_prph/%02X%02X%02X%02X%02X%02X", peer.addr[0],
		 peer.addr[1], peer.addr[2], peer.addr[3], peer.addr[4], peer.addr[5]);
	// Build the whole record first, then write it in ONE in-place pass -- see writeFileInPlace() for why
	// the old remove()+create()-per-connection pattern had to go. Fixed length, so no stale tail.
	uint8_t rec[3 + sizeof(bond_keys_t) + 1];
	uint16_t reclen = 0;
	rec[reclen++] = (uint8_t)sizeof(bond_keys_t);
	memcpy(rec + reclen, &bk, sizeof(bond_keys_t));
	reclen += sizeof(bond_keys_t);
	rec[reclen++] = 1; // empty device name field: length 1 ...
	rec[reclen++] = 0; // ... containing just a NUL
	// THIRD FIELD, length zero -- and it is not cosmetic. writeFileInPlace() cannot truncate
	// (Adafruit_LittleFS_File.cpp:56 maps FILE_O_WRITE to LFS_O_RDWR|LFS_O_CREAT, no LFS_O_TRUNC), and
	// Bluefruit APPENDS a third field to this same file: BLEGatt::_eventHandler() calls conn->saveCccd()
	// on every CCCD write while secured, and bond_save_cccd_dfr() writes [len][sys_attr] after the name.
	// So without this byte, connection N leaves a CCCD blob behind, connection N+1 rewrites only the two
	// fields ahead of it, and bond_load_cccd() then restores a PREVIOUS connection's CCCD state mid-link
	// (or gets NRF_ERROR_INVALID_DATA if the attribute table has shifted) -- silently, either way.
	// A zero length makes bond_load_cccd()'s `if (len > 0)` fail deterministically instead: no stale
	// restore, and the NULL sys_attr reset it falls back to is already handled at CONN_SEC_UPDATE.
	rec[reclen++] = 0;
	bool wrote = writeFileInPlace(fname, rec, reclen);
	// SELF-HEAL. A failed bond write is not survivable: without the key we cannot answer the PAIR/LTK2
	// challenge, the console terminates at LTK2, and the run looks like a protocol failure. That exact
	// confusion has cost this project three wrong conclusions, so recover automatically and say so
	// loudly rather than limping on. Formatting also clears /bondedhost.bin, which only means we fall
	// back to pairing-mode advertising -- correct behavior for a device that has just lost its bonds.
	// RETRY IN PLACE BEFORE FORMATTING (2026-08-09 audit, RISK-3). Formatting destroys the bond store,
	// /bondedhost.bin and the boot counter, leaving us unbonded -- a state indistinguishable from "the
	// console forgot us", which has already caused misdiagnosis. One transient failure should not cost all
	// of that, so try again first and only format if the store is genuinely unusable.
	if (!wrote) {
		Serial.println("# LEAN bond write failed -- retrying in place before considering a format");
		wrote = writeFileInPlace(fname, rec, reclen);
	}
	if (!wrote) {
		Serial.println("# LEAN bond write FAILED TWICE -- filesystem is full/corrupt, formatting and retrying");
		InternalFS.format();
		InternalFS.mkdir("/adafruit/bond_prph");
		g_haveBondedHost = false;
		wrote = writeFileInPlace(fname, rec, reclen);
		Serial.printf("# LEAN self-heal %s\n", wrote ? "OK -- bond store recovered" :
							       "FAILED -- bond store is unusable");
	}
	Serial.printf("# LEAN bond file %s: %s\n", fname, wrote ? "written synchronously" : "WRITE FAILED");
	// Deliberately NOT calling conn->saveBondKey() -- see the synchronous write above for why. (It also
	// returns true unconditionally, so it could never have reported the failure it was hiding.)
	// Bluefruit's own loadBondKey() sets its _bonded flag when it reads the file we just wrote.
	g_haveLinkKey = wrote;
	Serial.printf("# LEAN link key ready=%d t=%lu ms\n", wrote, (unsigned long)millis());
}

// Dedicated handler for chrUnk65a7 (65a724b3-..., "unknown purpose" in the real Joy-Con 2's structure).
// A live sniffer capture (2026-08-05) caught a real Switch writing here as the FIRST thing it does after
// finishing GATT/CCCD discovery during a genuine fresh-pairing attempt against this firmware -- previously
// invisible because this characteristic used the generic unkWriteCb() above, which only prints live and
// isn't persisted, so it was never seen in printCmdLog(). The payload was 17 zero bytes followed by
// 07 91 01 01 00 00 00 00 -- byte-for-byte the same CMD=0x07/subcmd=0x01 "handshake ping" a real Joy-Con 2
// sends to the SWITCH's own GATT server during a Home-button wake (see sendWakeSequence()'s cmd07[]).
// Evidently this is a bidirectional ping using the SAME command framing as COMMAND_WRITE
// ([cmd][0x91][0x01][subcmd][0x00][len][0x00][0x00][payload...]) but offset 17 bytes into this
// characteristic's payload instead of starting at byte 0. The Switch retried once ~10s after getting no
// reply on the very first (pre-fix) test, then the connection cycled shortly after.
static void chrHandshakeCb(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
	(void)conn_hdl;
	(void)chr;
	// Verbose hex dumps are gated: the console re-runs the whole handshake every ~2 s in the loop, so
	// these fire ~50 lines per cycle in the seconds right before the arm -- exactly when the CDC needs
	// to be empty. Serial blocks when a host is draining it. Keep the EVENT lines, drop the dumps.
	if (!g_quiet) {
		Serial.printf("# LEAN handshake-char write: len=%u data=", len);
		for (uint16_t i = 0; i < len; i++)
			Serial.printf("%02X", data[i]);
		Serial.println();
	}
	if (len < 25) {
		logCmd(0xFF, 0xFF); // sentinel: recognized channel, but shorter than the one known payload shape
		return;
	}
	uint8_t cmd = data[17];
	uint8_t subcmd = data[20];
	// PAYLOAD ALIGNMENT, printed rather than counted. I derived the parameter offset twice by hand and
	// got byte 25 once and byte 26 the other time, which is exactly why the "comply" test came back
	// reading 0x00 for flags the console had set to 0x27. Print the candidate bytes with their indices
	// so the real offset is read off a live frame instead of inferred from a hex string.
	if (!g_quiet && len >= 29)
		Serial.printf("# LEAN ALIGN cmd=0x%02X sub=0x%02X len=%u | [21]=%02X [22]=%02X [23]=%02X "
			      "[24]=%02X [25]=%02X [26]=%02X [27]=%02X\n",
			      cmd, subcmd, len, data[21], data[22], data[23], data[24], data[25], data[26],
			      data[27]);
	logCmd(cmd, subcmd);

	// PAIR/LTK1 from the Switch is the other half of the real link-layer key -- derive and install it
	// immediately (see deriveLinkKey()). Frame shape: 17-byte zero prefix, then
	// [cmd][0x91][0x01][subcmd][0x00][len=0x11][0x00][0x00][marker 0x00][16 key bytes] = 42 bytes.
	// This fires routinely now -- `derived link key` appears on normal pairings. (The old note here said
	// it had never fired, and described the 0x07/0x02/0x10/0x16 sequence as an "escalation"; that framing
	// is retracted -- the console sends the same sequence to real controllers on reconnect.)
	// 0x07/0x01, 0x10/0x01 and 0x16/0x01 all carry real payloads in a genuine controller's reply. Sending
	// the 8-byte empty ack sendHandshakeReply() produces was measurably wrong: in the 2026-08-06 live test
	// the Switch worked through 0x07 -> 0x02(MEMORY, answered with real content) -> 0x10, then stopped dead
	// and never sent 0x16 -- i.e. it stalled immediately after the first reply where we owed content and
	// sent none. The MEMORY read was already answered properly; these three were not.
	if (cmd == 0x07 && subcmd == 0x01) {
		sendResponseOn(chrCmdResp, 0x07, 0x01, ANNOUNCED_CMD07, sizeof ANNOUNCED_CMD07);
		return;
	}
	if (cmd == 0x10 && subcmd == 0x01) {
		sendResponseOn(chrCmdResp, 0x10, 0x01, ANNOUNCED_CMD10, sizeof ANNOUNCED_CMD10);
		return;
	}
	if (cmd == 0x16 && subcmd == 0x01) {
		sendResponseOn(chrCmdResp, 0x16, 0x01, ANNOUNCED_CMD16, sizeof ANNOUNCED_CMD16);
		return;
	}

	// ---- Phase 6: post-encryption bring-up. Same channel and framing as everything above; the Switch just
	// starts issuing these once the link is encrypted. Contents from the decrypted reconnect capture.
	{
		const uint8_t *body = nullptr;
		uint8_t bodyLen = 0;
		bool handled = true;
		if (cmd == CMD_VIBRATION) {
			// Historically this force-started the report stream here, on the theory that the console
			// would never send the 0x0085 descriptor write and we had to stream anyway for it to see
			// the registration press. The 2026-08-08 first-time-registration capture shows that is
			// backwards: a real Joy-Con sends NOTHING until the console's 0x0085 write, which the
			// console issues ~600ms after this point as a normal part of init. Streaming early is a
			// deviation no real device makes. Now gated behind 'N' (default off) so the console gets
			// the chance to open the stream itself.
			// Honour the phase-6 deferral here too. This fires on the VIBRATION command, i.e.
			// MID-HANDSHAKE -- and on a FRESH pairing `g_haveBondedHost` is still false at the
			// 0x0C/0x04 deferral site (saveBondedHost runs ~20 lines later), so that check could
			// never fire and the stream started here regardless. Net effect: reconnects deferred
			// correctly, fresh registrations did not -- which is exactly the queue starvation the
			// deferral existed to prevent (audit BUG-5).
			if (g_forceStream && !g_streaming && !g_deferStreamToPhase6) {
				forceEnableReportCccd();
				g_streaming = true;
				Serial.println("# LEAN *** report stream FORCED on (vibration cmd seen) ***");
			}
			// COMPLY (see the LEDS branch): echo the requested preset back instead of a bare ack.
			// 0x0A/0x02's payload names a vibration preset; acknowledging without repeating it is the
			// same "say ok, do nothing" shape that leaves the console with no evidence we acted.
			// (0x0A/0x08 keeps its own 'Y'-selectable behavior below, untouched.)
			// ANY vibration subcommand gets the bare 8-byte ack. 0x0A/0x02 is the one in the capture,
			// but a live console also sends 0x0A/0x08 (2026-08-07) once it has our calibration -- its
			// payload starts 69 09 00 00, the first bytes of what we served from 0x00013060, so it is
			// echoing our own data back. Blocking on it stalled the session with the controller
			// already visible on screen.
			// MEASURED: a real Joy-Con answers VIBRATION with an EMPTY body -- verified on hardware
			// that actually buzzed, so this is the reply of a controller that complied.
			body = nullptr; bodyLen = 0;
			// 0x0A/0x08 is the ONE divergence left between our post-encryption sequence and the real
			// reconnect capture: the console sends it only to us, never to a genuine controller, so
			// there is no ground truth for the right answer and the bare ack here is a guess. Since
			// the console's next step in the capture is the 0x0085 write that starts the report
			// stream -- the thing we never get -- this reply is the best remaining suspect. 'Y'
			// cycles the candidates against the (now ~7s) reconnect loop instead of reflashing.
			if (subcmd == 0x08) {
				static const uint8_t A08_ONE[4] = { 0x01, 0x00, 0x00, 0x00 };
				static const uint8_t A08_ZERO[4] = { 0x00, 0x00, 0x00, 0x00 };
				static uint8_t echoBuf[64];
				switch (g_a08Mode) {
				case 1: // say nothing at all -- does the console retry, or move on?
					Serial.println("# LEAN a08: NOT replying (mode 1)");
					return;
				case 2: body = A08_ONE; bodyLen = sizeof A08_ONE; break;
				case 3: body = A08_ZERO; bodyLen = sizeof A08_ZERO; break;
				case 4: { // echo its own payload back
					uint8_t n = (len > 25) ? (uint8_t)(len - 25) : 0;
					if (n > sizeof echoBuf)
						n = sizeof echoBuf;
					memcpy(echoBuf, data + 25, n);
					body = echoBuf; bodyLen = n;
					break;
				}
				default: break; // 0 = bare 8-byte ack, the long-standing behavior
				}
				Serial.printf("# LEAN a08: reply mode %u, %u payload bytes\n", g_a08Mode, bodyLen);
			}
		} else if (cmd == CMD_LEDS && subcmd == SUB_LEDS_SET_PLAYER) {
			// COMPLY, 2026-08-09. These are not queries -- they are INSTRUCTIONS WITH PARAMETERS, and
			// we had been answering "ok" while doing nothing. Decoded from the console's own writes:
			//   0x09/0x07 payload 00 01 00 00 00  = light player LED pattern 0x01
			//   0x0A/0x02 payload 00 03 00 00 00  = play vibration preset 3
			//   0x0C/0xx  payload 00 27 00 00 00  = enable features, flag set 0x27
			// We cannot physically rumble, so the REPLY is the only place compliance can show. Echo the
			// accepted parameters back rather than an empty ack -- this file already records that empty
			// acks stall bring-up ("an empty one here would stall the same way it stalled the pairing
			// ceremony"), which was fixed for 0x0C and left in place here.
			// MEASURED 2026-08-09 by asking a real Joy-Con directly over bleak
			// (docs/captures/jc_rumble.py): it answers LEDS/SET_PLAYER with an EMPTY body. The
			// original bare ack was CORRECT; echoing the parameters was my invention and made us
			// diverge from real hardware.
			body = nullptr; bodyLen = 0;
		} else if (cmd == CMD_FEATURE && (subcmd == SUB_FEATURE_INIT || subcmd == SUB_FEATURE_ENABLE)) {
			// Echo the requested flag set instead of RSP_FEATURE's four invented zero bytes. The
			// console asks for 0x27 here; 0x0C/0x04 is the command that gates full reporting, so
			// answering it with zeros may be us declining the very stream we want.
			// MEASURED against a real Joy-Con that ACTUALLY COMPLIED (LED lit, motor blipped):
			// it answers FEATURE with exactly four zero bytes. RSP_FEATURE was correct all along.
			body = RSP_FEATURE; bodyLen = sizeof RSP_FEATURE;
		} else if (cmd == 0x11 && subcmd == 0x01) {
			body = RSP_11_01; bodyLen = sizeof RSP_11_01;
		} else if (cmd == 0x11 && subcmd == 0x03) {
			body = g_ownFingerprint ? RSP_11_03_OWN : RSP_11_03; bodyLen = sizeof RSP_11_03;
		} else if (cmd == 0x13 && subcmd == 0x01) {
			body = RSP_13_01; bodyLen = sizeof RSP_13_01;
		} else if (cmd == 0x01 && subcmd == 0x0C) {
			// NEW TERRITORY, 2026-08-13. Once the console actually arms the stream (0x0085 to d5a9's
			// 679d descriptor) it goes on to ASK us for 0x01/0x0C -- twice, ~10 s apart. The firmware
			// used to fall through to "unrecognized -- NOT replying automatically", the console got
			// nothing both times, and the link died of supervision timeout. That silence was the end
			// of every successful bring-up we had never seen the far side of.
			//
			// We have always had the right bytes: RSP_01_0C is the exact payload a real Joy-Con
			// NOTIFIES on this cmd/sub right before its reports start (`0101010c 1078 0000 61125010`
			// in the decrypted reconnect). They were only ever used for an unprompted announcement,
			// which is why this looked like a command we had no answer for.
			body = g_ownMisc ? RSP_01_0C_OWN : RSP_01_0C; bodyLen = sizeof RSP_01_0C;
		} else {
			handled = false;
		}
		if (handled) {
			if (!g_quiet)
				Serial.printf("# LEAN phase6 cmd=0x%02X sub=0x%02X -- answering (%u payload "
					      "bytes)\n", cmd, subcmd, bodyLen);
			sendResponseOn(chrCmdResp, cmd, subcmd, body, bodyLen);
			// ASK, rather than only ever answering.
			//
			// 0x0C/0x04 is the last thing the console says before it either sends the 0x0085
			// stream-start (real hardware) or hangs up (us). Everything we have done for months has
			// been reactive -- we have never once spoken first. In the real capture the controller
			// sends 0x01/0x0C right around here, and our firmware already has its exact reply bytes,
			// but only fires them in response to the 0x0085 write we never receive. So it has never
			// gone out at all. Send it unprompted as a readiness signal and see whether the console
			// responds with the descriptor write.
			//
			// Content is a real captured frame, not a guess, which matters: the crashes this project
			// saw years-ago-in-session-time were from malformed/empty notifications on this channel,
			// not from using it uninvited. One-shot per connection so we cannot spam it.
			if (cmd == CMD_FEATURE && subcmd == SUB_FEATURE_ENABLE && !g_askedForStream) {
				g_askedForStream = true;
				// Phase 6 is done, so the handshake no longer needs the queue to itself: this is the
				// safe moment to open the reconnect stream that bleConnectCb() deliberately withheld.
				if (g_forceStream && g_deferStreamToPhase6 && !g_streaming) { // no g_haveBondedHost: false here on a FRESH pair (audit BUG-5)
					// RE-ARM, do not assume bleConnectCb's arming survived. Encryption happens
					// between the two, and Bluefruit resets every CCCD on the connection at
					// CONN_SEC_UPDATE (see the BLE_GAP_EVT_CONN_SEC_UPDATE case in rawBleEventCb).
					// Without this the "stream is now open" line below was followed by hvx
					// returning NRF_ERROR_INVALID_STATE (0x08) on every single report.
					forceEnableReportCccd();
					g_streaming = true;
					Serial.println("# LEAN phase 6 complete on a reconnect -- opening the report "
						       "stream now");
				}
				// Registration is complete by this point (0x0C/0x04 is the last phase-6 step), so
				// REMEMBER this console and start advertising in wake mode -- "I belong to you" --
				// instead of continuing to broadcast an all-zero reconnect MAC meaning "I am
				// unpaired, anyone may connect".
				//
				// We had been re-presenting as a brand-new controller forever, which is why the
				// console re-pairs us every time the Change Grip/Order screen is opened. It also
				// makes the actual goal impossible: a sleeping Switch scans for ITS OWN address in
				// that field, so a pairing-mode advertiser is invisible to it by design.
				//
				// Auto-saving was disabled in an earlier session because flipping to wake mode
				// mid-registration hid the puck from the Change Grip/Order screen. That reasoning
				// is obsolete now that registration actually completes -- and doing it here, after
				// the final phase-6 command rather than merely on encryption, is what makes it safe.
				if (g_connHdl != BLE_CONN_HANDLE_INVALID) {
					BLEConnection *c2 = Bluefruit.Connection(g_connHdl);
					if (c2 && !g_haveBondedHost) {
						ble_gap_addr_t pa = c2->getPeerAddr();
						saveBondedHost(pa.addr);
						// REBUILD the advertisement payload, or this does nothing observable.
						// saveBondedHost() only sets the flag and the file; the bytes actually
						// broadcast are whatever buildAdvertising() last produced. Without this
						// the status line claims "WAKE inviting" while the radio is still
						// sending a zeroed reconnect MAC -- so the console keeps treating us as
						// an unpaired controller and re-pairs us every time, which is exactly
						// the symptom the user reported. Takes effect when advertising restarts
						// on disconnect (restartOnDisconnect is enabled).
						buildAdvertising();
						Serial.println("# LEAN registration complete -- remembering this "
							       "console, advert rebuilt for WAKE mode");
					}
				}
				// PRIME SUSPECT for the supervision-timeout drop -- default OFF, 'U' toggles.
				// Measured 2026-08-09, two consecutive connections: this notify is the LAST ATT
				// event of the connection. The console never writes again, and the link dies of
				// supervision timeout 13.3s / 21.7s later (arithmetic matches the post-mortem's
				// "last write FROM console" exactly). It does not hang up -- it stops responding.
				//
				// The two premises this send was built on are both now DISPROVEN: there is no
				// 0x0085 stream-start to elicit (STALE-14), and the protocol is strictly reactive
				// with no unprompted controller traffic in any capture (BUG-6, proven). So we are
				// the only party ever speaking out of turn, at exactly the moment things die.
				if (g_askUnprompted) {
					Serial.println("# LEAN >>> ASKING: unprompted 0x01/0x0C after 0x0C/0x04 <<<");
					sendResponseOn(chrCmdResp, 0x01, 0x0c, g_ownMisc ? RSP_01_0C_OWN : RSP_01_0C,
						       sizeof RSP_01_0C);
				} else {
					Serial.println("# LEAN phase 6 complete -- staying SILENT (unprompted "
						       "0x01/0x0C suppressed; 'U' to re-enable)");
				}
			}
			return;
		}
	}

	// The full PAIR ceremony, answered reactively. Ground truth is the decrypted reconnect capture, where
	// each of these is a write from the Switch answered by exactly one notification from the controller:
	//   0x15/0x01 SET_MAC -> controller's own address     0x15/0x04 LTK1   -> the universal constant
	//   0x15/0x02 LTK2    -> controller's own LTK2        0x15/0x03 FINISH -> single byte 0x01
	// Without these the generic path would send an empty ack, which is NOT what a real controller replies
	// -- and since SET_MAC is the very next thing the Switch writes after the 0x07/0x02/0x10/0x16 block we
	// already answer, a wrong reply there would stall us at exactly the point we are trying to get past.
	if (cmd == CMD_PAIR) {
		const uint8_t *body = nullptr;
		uint8_t bodyLen = 0;
		switch (subcmd) {
		case SUB_PAIR_SET_MAC:
			body = ANNOUNCED_SET_MAC;
			bodyLen = sizeof ANNOUNCED_SET_MAC;
			break;
		case SUB_PAIR_LTK1:
			// This write carries the Switch's half of the real link-layer key.
			if (len >= 42)
				deriveLinkKey(data + 26);
			else
				Serial.printf("# LEAN PAIR/LTK1 too short (len=%u, need 42) -- no key\n", len);
			body = g_ownKeyMaterial ? ANNOUNCED_LTK1_OWN : ANNOUNCED_LTK1;
			bodyLen = sizeof ANNOUNCED_LTK1;
			break;
		case SUB_PAIR_LTK2:
			// CHALLENGE-RESPONSE, found 2026-08-06. The Switch's LTK2 write is a random 16-byte
			// challenge; the controller must answer with it encrypted under the link key derived
			// from the LTK1 pair:
			//     response = AES-128-ECB-Encrypt(key = LTK(MSO-first), reverse(challenge))
			// with the ciphertext sent as-is (no reversal). Verified as an exact 16-byte match
			// against the one captured pair in the decrypted reconnect capture, which is also an
			// independent confirmation that deriveLinkKey() is correct -- the same key that decrypts
			// the link also produces the right answer here.
			//
			// This is the real gate. Answering with a fixed constant (what we did until now, and
			// what the reference PC repos hardcode) fails the proof-of-knowledge, and the Switch
			// terminates the connection ~43ms later with LL_TERMINATE_IND and shows nothing on
			// screen. It also explains why third-party pads need Switch-2-specific firmware rather
			// than a generic HID profile: this step can't be faked without implementing it.
			if (g_haveLinkKey && len >= 42) {
				nrf_ecb_hal_data_t ecb;
				memcpy(ecb.key, g_linkKeyMsb, 16);
				for (uint8_t i = 0; i < 16; i++)
					ecb.cleartext[i] = data[26 + 15 - i]; // challenge, reversed
				uint32_t err = sd_ecb_block_encrypt(&ecb);
				if (err == NRF_SUCCESS) {
					static uint8_t resp[17];
					resp[0] = 0x01; // same marker byte a real controller uses
					memcpy(resp + 1, ecb.ciphertext, 16);
					Serial.printf("# LEAN PAIR/LTK2 challenge answered: ");
					for (uint8_t i = 0; i < 16; i++)
						Serial.printf("%02x", ecb.ciphertext[i]);
					Serial.println();
					body = resp;
					bodyLen = sizeof resp;
					break;
				}
				Serial.printf("# LEAN PAIR/LTK2 AES failed err=0x%08lX\n",
					      (unsigned long)err);
			} else {
				Serial.printf(
					"# LEAN PAIR/LTK2 cannot answer challenge (haveKey=%d len=%u) -- falling back to the fixed constant, which the Switch will reject\n",
					g_haveLinkKey, len);
			}
			body = g_ownKeyMaterial ? ANNOUNCED_LTK2_OWN : ANNOUNCED_LTK2;
			bodyLen = sizeof ANNOUNCED_LTK2;
			break;
		case SUB_PAIR_FINISH: {
			// Last chance to check the bond store before the Switch's LL_ENC_REQ lands (~30ms after
			// our reply to this). The write is deferred, so this read is what actually proves the key
			// is retrievable -- saveBondKey() always returns true and tells us nothing.
			BLEConnection *c = Bluefruit.Connection(g_connHdl);
			static bond_keys_t probe;
			memset(&probe, 0, sizeof probe);
			if (c && c->loadBondKey(&probe)) {
				bool match = (memcmp(probe.own_enc.enc_info.ltk,
						     g_ltkReverse ? g_linkKeyMsb : g_linkKeyWire, 16) == 0);
				Serial.printf("# LEAN bond store verify: key readable, matches=%d ltk=", match);
				for (uint8_t i = 0; i < 16; i++)
					Serial.printf("%02x", probe.own_enc.enc_info.ltk[i]);
				Serial.println();
			} else {
				Serial.println(
					"# LEAN bond store verify: FAILED to read back -- Bluefruit will answer SEC_INFO_REQUEST with NULL and encryption will be rejected");
			}
			body = ANNOUNCED_PAIR_FINISH;
			bodyLen = sizeof ANNOUNCED_PAIR_FINISH;
			break;
		}
		default:
			break;
		}
		if (body) {
			Serial.printf("# LEAN PAIR sub=0x%02X -- answering with real content (%u bytes)\n",
				      subcmd, bodyLen);
			sendResponseOn(chrCmdResp, CMD_PAIR, subcmd, body, bodyLen);
			return;
		}
	}

	// Auto-reply immediately for the specific (cmd,subcmd) pairs already confirmed safe live on real
	// hardware (2026-08-06): 0x07/0x01 (the initial ping), 0x02/0x04 (MEMORY READ, answered with real
	// content), 0x10/0x01, 0x16/0x01 -- all observed with NO crash across multiple live tests. These arrive
	// in a fast burst (as little as ~70ms apart) when the Switch escalates right after the proactive
	// announcement, too fast for a human-driven manual 'A' to catch each one before the next overwrites the
	// pending state -- auto-replying removes that race. Anything NOT in this known-safe list still defers
	// to manual 'A'/'X' given the earlier crash history on this exact channel with unrecognized content.
	// Only CMD_MEMORY can actually reach here: 0x07/0x01, 0x10/0x01 and 0x16/0x01 are handled by dedicated
	// branches above that return early (audit DEAD-3). Kept listed for intent, but they are unreachable.
	bool knownSafe = (cmd == CMD_MEMORY && subcmd == SUB_MEMORY_READ);
	if (knownSafe) {
		if (!g_quiet)
			Serial.printf("# LEAN handshake cmd=0x%02X sub=0x%02X -- known-safe, auto-replying\n",
				      cmd, subcmd);
		sendHandshakeReply(cmd, subcmd, data, len);
		return;
	}

	g_pendingCmd = cmd;
	g_pendingSubcmd = subcmd;
	g_pendingRawLen = (len > sizeof g_pendingRaw) ? sizeof g_pendingRaw : len;
	memcpy(g_pendingRaw, data, g_pendingRawLen);
	g_handshakePendingReply = true;
	Serial.printf(
		"# LEAN handshake cmd=0x%02X sub=0x%02X (unrecognized) -- NOT replying automatically. Send 'A' to ack on %s, 'X' to echo the raw frame back on chrUnk640c (safe channel only).\n",
		cmd, subcmd, HANDSHAKE_CHANNEL_NAME[g_handshakeReplyChannel]);
}

// Echoes the exact raw request frame back verbatim (17-byte prefix included) instead of the plain 8-byte
// sendResponse() ack -- content-variation experiment once channel alone was ruled out as sufficient.
// HARDCODED to chrUnk640c regardless of g_handshakeReplyChannel: that's the one channel confirmed safe
// (2026-08-06 testing) against COMMAND_RESPONSE and chrUnkD3bd both crashing the Switch on reply -- keep
// this experiment on the known-safe channel rather than compounding two untested variables at once.
static void echoPendingHandshake()
{
	if (!g_handshakePendingReply) {
		Serial.println("# LEAN: 'X' ignored, no pending handshake reply");
		return;
	}
	g_handshakePendingReply = false;
	Serial.printf("# LEAN: echoing raw %u-byte frame back on chrUnk640c\n", g_pendingRawLen);
	// The one notify path that bypasses sendResponseOn(), so it must count itself -- otherwise
	// HVN_TX_COMPLETE decrements for a notification nothing incremented.
	if (chrUnk640c.notify(g_pendingRaw, g_pendingRawLen) && g_hvnOutstanding < 255)
		g_hvnOutstanding++;
}

// Proactive fresh-pairing announcement -- byte-decoded from TWO REAL Joy-Con 2 pairing sessions
// (docs/captures/joycon2_pairing_DECRYPTED_ltk2.pcap frames 513-524, 2026-08-06 first session; a second
// live capture the same night with a different pairing cycle of the same physical device). The real
// controller does NOT wait to be asked: immediately after its 3 CCCDs get subscribed, it fires 8
// notifications on COMMAND_RESPONSE completely unprompted -- the first capture only showed 5 (a sniffer
// capture gap, not a real absence; confirmed by the second capture showing all 8, including CMD=0x16/0x01
// and the controller's OWN PAIR/LTK2 + PAIR/FINISH notifications, which the first capture missed entirely).
// This reframes the whole CMD=0x07/sub=0x01 "ping" we'd been trying to answer -- it's not a question
// expecting a targeted reply, it's the first of these 8 self-announcements, and the Switch writing it TO us
// is most likely just its nudge/retry for an otherwise-silent controller. Manually triggered via 'P' (not
// automatic) given the crash history on this exact channel with wrong-shaped content.
static void sendProactiveAnnouncement()
{
	if (g_connHdl == BLE_CONN_HANDLE_INVALID) {
		Serial.println("# LEAN: 'P' ignored, not connected");
		return;
	}
	Serial.println(
		"# LEAN: sending proactive fresh-pairing announcement (8 notifications on COMMAND_RESPONSE, 30ms spaced)");

	// Real inter-notification timing measured 2026-08-06 from a clean, complete capture (left Joy-Con,
	// zero packet loss): a consistent ~30ms between every one of the 8 notifications. Our previous
	// back-to-back-as-fast-as-possible delivery was the one remaining untested difference from real
	// controller behavior -- added here as the next experiment now that content is confirmed byte-exact
	// against two independent real devices.
	#define ANNOUNCE_GAP_MS 30

	// 1) CMD=0x07/sub=0x01 -- real payload was a single zero byte.
	sendResponseOn(chrCmdResp, 0x07, 0x01, ANNOUNCED_CMD07, sizeof ANNOUNCED_CMD07);
	delay(ANNOUNCE_GAP_MS);

	// 2) CMD=0x02(MEMORY)/sub=0x04(READ) -- our own controller info, same content shape handleMemoryRead()
	// builds for an explicit request, just self-announced instead of asked for.
	{
		uint8_t inner[8 + 64];
		inner[0] = 0x40; // reqLen -- matches the real capture's self-announced length exactly
		inner[1] = 0x00; // 1-3 all zero -- see handleMemoryRead(), the old 0x01 came from a bad-CRC frame
		inner[2] = 0x00;
		inner[3] = 0x00;
		inner[4] = (uint8_t)(ADDR_CONTROLLER_INFO & 0xFF);
		inner[5] = (uint8_t)((ADDR_CONTROLLER_INFO >> 8) & 0xFF);
		inner[6] = (uint8_t)((ADDR_CONTROLLER_INFO >> 16) & 0xFF);
		inner[7] = (uint8_t)((ADDR_CONTROLLER_INFO >> 24) & 0xFF);
		buildControllerInfoContent(inner + 8, 64);
		sendResponseOn(chrCmdResp, CMD_MEMORY, SUB_MEMORY_READ, inner, sizeof inner);
	}
	delay(ANNOUNCE_GAP_MS);

	// 3) CMD=0x10/sub=0x01 -- undocumented, replicated byte-for-byte from the real capture.
	sendResponseOn(chrCmdResp, 0x10, 0x01, ANNOUNCED_CMD10, sizeof ANNOUNCED_CMD10);
	delay(ANNOUNCE_GAP_MS);

	// 4) CMD=0x16/sub=0x01 -- undocumented, discovered in the SECOND real capture. Real payload: 24 zero
	// bytes. ORDER FIXED 2026-08-06: this was previously placed AFTER SET_MAC/LTK1 (items 6 here), which
	// does not match the real capture's actual sequence -- real order is ping, MEMORY, 0x10, 0x16, SET_MAC,
	// LTK1, LTK2, FINISH. Re-verified directly against the left Joy-Con capture's frame order before fixing.
	sendResponseOn(chrCmdResp, 0x16, 0x01, ANNOUNCED_CMD16, sizeof ANNOUNCED_CMD16);
	delay(ANNOUNCE_GAP_MS);

	// 5) CMD=0x15(PAIR)/sub=0x01(SET_MAC) -- decoded as the CONTROLLER announcing its OWN address: the
	// real payload's tail matched the real Joy-Con's actual BT address byte-for-byte.
	// Prefix bytes (01 04 01) copied verbatim, purpose unknown; our own advertised address substituted in
	// the same trailing position (matches the addr[] bytes set in bleInitOnce()'s Bluefruit.setAddr() call).
	sendResponseOn(chrCmdResp, CMD_PAIR, SUB_PAIR_SET_MAC, ANNOUNCED_SET_MAC, sizeof ANNOUNCED_SET_MAC);
	delay(ANNOUNCE_GAP_MS);

	// 6) CMD=0x15(PAIR)/sub=0x04(LTK1) -- confirmed 2026-08-06 via THREE real captures (two separate
	// pairing sessions of the right Joy-Con, one of the physically different left Joy-Con): this exact
	// 17-byte value is BYTE-FOR-BYTE IDENTICAL across all three -- a universal fixed value, not per-device
	// or per-session crypto as first worried. Using the real bytes directly.
	sendResponseOn(chrCmdResp, CMD_PAIR, SUB_PAIR_LTK1, ANNOUNCED_LTK1, sizeof ANNOUNCED_LTK1);
	delay(ANNOUNCE_GAP_MS);

	// 7) CMD=0x15(PAIR)/sub=0x02(LTK2) -- the CONTROLLER's own LTK2 notification. Unlike LTK1, this value
	// DIFFERS between the left and right Joy-Con -- ambiguous evidence: could mean the Switch checks it
	// (foreign device's real value + our fake identity = internally inconsistent, silently rejected with no
	// visible error, which is indistinguishable from every other stall we've seen), or could just mean the
	// controller computes something here for its own bookkeeping that the Switch never inspects, same as
	// LTK1. Runtime-toggleable via 'K' console command so this can be tested without a reflash cycle.
	if (!g_skipLtk2) {
		sendResponseOn(chrCmdResp, CMD_PAIR, SUB_PAIR_LTK2, ANNOUNCED_LTK2, sizeof ANNOUNCED_LTK2);
		delay(ANNOUNCE_GAP_MS);
	} else {
		Serial.println("# LEAN: skipping LTK2 notification ('K' toggle active)");
	}

	// 8) CMD=0x15(PAIR)/sub=0x03(FINISH) -- the CONTROLLER's own pairing-finish notification. Real payload:
	// single byte 0x01.
	sendResponseOn(chrCmdResp, CMD_PAIR, SUB_PAIR_FINISH, ANNOUNCED_PAIR_FINISH,
			sizeof ANNOUNCED_PAIR_FINISH);

	#undef ANNOUNCE_GAP_MS
}

// Builds and sends the appropriate reply for a received handshake-channel command -- content-aware for
// CMD_MEMORY/SUB_MEMORY_READ (real controller-info content, confirmed live 2026-08-06), generic empty ack
// otherwise. Shared by the manual 'A' command and the auto-reply path in chrHandshakeCb() below. Takes an
// explicit snapshot (cmd/subcmd/raw/rawLen) rather than reading the g_pending* globals directly so the
// auto-reply path can call this before those globals get overwritten by a fast-arriving next command.
static void sendHandshakeReply(uint8_t cmd, uint8_t subcmd, const uint8_t *raw, uint16_t rawLen)
{
	BLECharacteristic &target = (g_handshakeReplyChannel == 1) ? chrUnk640c :
				     (g_handshakeReplyChannel == 2) ? chrUnkD3bd :
								       chrCmdResp;

	// CMD_MEMORY/SUB_MEMORY_READ needs real content, not an empty ack -- confirmed live 2026-08-06: the
	// Switch escalated to sending this over the handshake channel after the proactive announcement, request
	// payload (byte-identical shape to handleMemoryRead()'s) starts at raw[25]. Build the same content
	// handleMemoryRead() would for an explicit request.
	if (cmd == CMD_MEMORY && subcmd == SUB_MEMORY_READ && rawLen >= 33) {
		const uint8_t *req = raw + 25; // [reqLen][0x7e][0x00][0x00][addr LE 4 bytes]
		uint8_t reqLen = req[0];
		uint32_t addr = (uint32_t)req[4] | ((uint32_t)req[5] << 8) | ((uint32_t)req[6] << 16) |
				((uint32_t)req[7] << 24);
		if (!g_quiet)
			Serial.printf(
			"# LEAN: answering cmd=0x%02X sub=0x%02X (MEMORY READ addr=0x%08lX len=%u) on %s\n",
			cmd, subcmd, (unsigned long)addr, reqLen, HANDSHAKE_CHANNEL_NAME[g_handshakeReplyChannel]);
		uint8_t inner[8 + 64];
		inner[0] = reqLen;
		inner[1] = 0x00; // 1-3 all zero -- see handleMemoryRead(), the old 0x01 came from a bad-CRC frame
		inner[2] = 0x00;
		inner[3] = 0x00;
		inner[4] = req[4];
		inner[5] = req[5];
		inner[6] = req[6];
		inner[7] = req[7];
		uint8_t contentLen = reqLen > 64 ? 64 : reqLen;
		buildMemoryContent(addr, inner + 8, contentLen);
		sendResponseOn(target, CMD_MEMORY, SUB_MEMORY_READ, inner, (uint8_t)(8 + contentLen));
		return;
	}

	Serial.printf("# LEAN: answering cmd=0x%02X sub=0x%02X on %s\n", cmd, subcmd,
		      HANDSHAKE_CHANNEL_NAME[g_handshakeReplyChannel]);
	sendResponseOn(target, cmd, subcmd, nullptr, 0);
}

static void answerPendingHandshake()
{
	if (!g_handshakePendingReply) {
		Serial.println("# LEAN: 'A' ignored, no pending handshake reply");
		return;
	}
	g_handshakePendingReply = false;
	sendHandshakeReply(g_pendingCmd, g_pendingSubcmd, g_pendingRaw, g_pendingRawLen);
}

// Bluefruit's addDescriptor() cannot build the report-stream trigger. It hardcodes meta.vlen = 0 and
// init_len = max_len = len (BLECharacteristic.cpp:379-406), so EVERY descriptor it creates is FIXED
// length -- and the console starts the report stream by writing TWO bytes (0x0085) to the 679d descriptor
// at handle 0x0010 (capture: `05 0004 00 12 1000 8500`, an ATT Write REQUEST). A 2-byte write into a
// fixed 1-byte attribute is rejected by the SoftDevice with ATT 0x0D INVALID_ATT_VAL_LENGTH; the stack
// sends the error response itself and raises NO BLE_GATTS_EVT_WRITE. The rejection is therefore invisible
// to this firmware -- which is why "the console never sends 0x0085" read like a measurement for months
// when the serial log looks identical whether it sends the write or not. It is also the console's ONLY
// Write Request in the whole reconnect (everything else is 0x52 Write Command, no response), so it is the
// one message where an error can travel back, and it is followed by silence and supervision timeout.
//
// vlen = 1 accepts any length up to max_len, so both a 1- and a 2-byte write land. Returns the REAL
// handle rather than leaving the caller to infer it as cccd+1.
static uint16_t addVarLenDescriptor(const uint8_t uuid128[16], uint16_t maxLen,
				    SecureMode_t read_perm, SecureMode_t write_perm)
{
	ble_gatts_attr_md_t meta;
	memset(&meta, 0, sizeof meta);
	memcpy(&meta.read_perm, &read_perm, 1);
	memcpy(&meta.write_perm, &write_perm, 1);
	meta.vlen = 1;
	meta.vloc = BLE_GATTS_VLOC_STACK;

	BLEUuid bleuuid(uuid128);
	bleuuid.begin();

	static uint8_t descInit[2] = { 0, 0 };
	ble_gatts_attr_t desc = {
		.p_uuid    = &bleuuid._uuid,
		.p_attr_md = &meta,
		.init_len  = 1,
		.init_offs = 0,
		.max_len   = maxLen,
		.p_value   = descInit,
	};

	uint16_t hdl = 0;
	uint32_t err = sd_ble_gatts_descriptor_add(BLE_GATT_HANDLE_INVALID, &desc, &hdl);
	if (err != NRF_SUCCESS) {
		Serial.printf("# LEAN *** descriptor_add FAILED err=0x%08lX -- the stream trigger has no "
			      "attribute to land on ***\n", (unsigned long)err);
		return 0;
	}
	return hdl;
}

// ---- WRITE-PROBE INSTRUMENTATION (2026-08-13) -------------------------------------------------
// The SoftDevice answers a rejected write ITSELF and raises no BLE_GATTS_EVT_WRITE, so this firmware
// cannot see ANY write the console makes to an attribute that will not accept it -- wrong length, or
// write permission NO_ACCESS. That blindness already cost this project weeks: the 679d descriptor at
// 0x0010 was a fixed 1-byte attribute, so the console's 2-byte 0x0085 stream trigger would have been
// bounced with ATT 0x0D and recorded as "the console never sends it". It was not observable either way.
//
// So every attribute the console could conceivably write is now OPEN and VARIABLE LENGTH, purely as
// instrumentation. Declared PROPERTIES are deliberately left untouched: permissions are enforced by the
// SoftDevice but never exposed to the peer, so discovery looks byte-identical to before and the
// console's behavior cannot change as a side effect of the probe itself.
//
// CAVEAT, so a negative is read correctly: a client that respects the properties field will not write
// to a NOTIFY-only characteristic in the first place. This catches only a console writing to a cached
// or hardcoded handle regardless of properties -- which is exactly the hypothesis worth testing, since
// it demonstrably writes 0x0010 on a real device without ever discovering descriptors.
static struct {
	uint16_t handle;
	const char *name;
} g_probes[20];
static uint8_t g_nProbes = 0;
static void armProbe(uint16_t handle, const char *name)
{
	if (handle && g_nProbes < (uint8_t)(sizeof g_probes / sizeof g_probes[0]))
		g_probes[g_nProbes++] = { handle, name };
}
static const char *probeName(uint16_t handle)
{
	for (uint8_t i = 0; i < g_nProbes; i++)
		if (g_probes[i].handle == handle)
			return g_probes[i].name;
	return nullptr;
}

static void printGattCheck();
static void setupGatt()
{
	uint8_t zero1 = 0;

	// ---- HANDLE ALIGNMENT ("false wall") ----------------------------------------------------------
	// An empty primary service costs exactly ONE attribute handle, which makes it a perfect spacer. With
	// Service Changed disabled the SoftDevice's reserved block ends at 0x000A, so this wall puts our main
	// service at 0x000C and lands the report characteristic on the real device's numbers:
	//
	//   0x000B  spacer service (this)
	//   0x000C  main service decl
	//   0x000D  report char decl, 0x000E value, 0x000F CCCD, 0x0010 679d descriptor
	//   0x0011  vibration decl, 0x0012 value ... and the rest follows, since our spacing already mirrors
	//   the real device.
	//
	// WHY IT IS STILL HERE, given the console demonstrably DISCOVERS handles for its writes (proven by
	// removing it: the console followed us to 0x0015/0x001A/0x001E/0x0022 without complaint):
	// a NOTIFICATION is the other direction -- we choose the handle it arrives on. The wall places
	// d5a9's VALUE on 0x000E, the real device's report handle. Whether the console filters our reports by
	// handle is UNRESOLVED; it has never been observed consuming our stream in any layout.
	//
	// COST: the spacer is a fake service with an invented UUID that no genuine controller exposes, and the
	// console enumerates all services on the generic path. Remove this one line to drop it.
	// Consequences of the layout, both deliberate: d5a9e01e is declared FIRST in the main service, and
	// ...fd2 plus the whole bd28x service move to the END.
	bleSvcSpacer.begin();


	// Main custom service (ab7de9be-...-fd0), all 10 characteristics in the same order the real Joy-Con 2
	// Right declares them.
	bleSvc.begin();


	// d5a9e01e DECLARED FIRST, so its value lands on 0x000E -- the handle a real Joy-Con notifies its
	// reports from. RESTORED 2026-08-14 (user's idea).
	//
	// The console picks its WRITE targets by UUID: it found d5a9's 679d descriptor and CCCD at
	// 0x002E/0x002D and used them correctly with d5a9 at the end of the service. But a NOTIFICATION goes
	// the other way -- WE choose the handle our reports arrive on, and the console gets no say. It has
	// been receiving our reports from 0x002C while a real device sends them from 0x000E, on every
	// connection since the (wrong) fd2 experiment moved d5a9 to the end.
	//
	// Symptom that prompted this: the console arms the stream, takes 2-3 reports, and drops the link
	// ~166 ms later. A host that accepts the subscription but filters incoming reports by the handle it
	// expects would look exactly like that.
	chrUnkD5a9.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
	chrUnkD5a9.setPermission(SECMODE_OPEN, SECMODE_OPEN); // WRITE PROBE
	chrUnkD5a9.setMaxLen(sizeof INPUT_REPORT_TEMPLATE);
	chrUnkD5a9.setCccdWriteCallback(cccdCb);
	chrUnkD5a9.setReadAuthorizeCallback(readAuthCb, false); // log ATT reads; false = BLE ctx
	chrUnkD5a9.begin();
	{
		// A READ must still return 63 bytes, as it did under setFixedLen().
		static const uint8_t d5a9Init[sizeof INPUT_REPORT_TEMPLATE] = { 0 };
		chrUnkD5a9.write(d5a9Init, sizeof d5a9Init);
	}
	armProbe(chrUnkD5a9.handles().value_handle, "d5a9 value");
	// THE STREAM TRIGGER lands on this descriptor -- measured 2026-08-13. Two bytes and variable length
	// via addVarLenDescriptor(): Bluefruit's addDescriptor() builds FIXED-length attributes and would
	// bounce the console's 2-byte 0x0085 write with ATT 0x0D, invisibly.
	g_hD5a9Desc = addVarLenDescriptor(UUID_DESC_679D, 2, SECMODE_OPEN, SECMODE_OPEN);
	g_hD5a9Cccd = chrUnkD5a9.handles().cccd_handle;
	armProbe(g_hD5a9Desc, "d5a9 679d desc  <-- THE STREAM TRIGGER");
	Serial.printf("# LEAN REPORT CHANNEL d5a9: value=0x%04X cccd=0x%04X 679d-desc=0x%04X"
		      "  (real device: 0x000E/0x000F/0x0010)\n",
		      chrUnkD5a9.handles().value_handle, g_hD5a9Cccd, g_hD5a9Desc);

	// Vibration's real position confirmed via bleak GATT dump against the real Joy-Con 2 (2026-08-05):
	// 3rd characteristic in this service, right here between d5a9e01e and COMMAND_WRITE -- not at the end
	// as originally guessed.
	chrVibration.setProperties(CHR_PROPS_WRITE_WO_RESP);
	chrVibration.setPermission(SECMODE_OPEN, SECMODE_OPEN);
	chrVibration.setMaxLen(64); // was 16 -- a longer HD-rumble frame would be rejected invisibly (audit RISK-1)
	chrVibration.setWriteCallback(chrVibrationCb);
	chrVibration.begin();

	chrCmdWrite.setProperties(CHR_PROPS_WRITE_WO_RESP);
	chrCmdWrite.setPermission(SECMODE_OPEN, SECMODE_OPEN);
	chrCmdWrite.setMaxLen(128);
	chrCmdWrite.setWriteCallback(chrCmdWriteCb);
	chrCmdWrite.begin();

	chrUnk65a7.setProperties(CHR_PROPS_WRITE_WO_RESP);
	chrUnk65a7.setPermission(SECMODE_OPEN, SECMODE_OPEN);
	// 64, not 33. RAISED 2026-08-06 -- 33 was a real latent blocker at the LAST step of the handshake:
	// the Switch's PAIR/LTK1 and PAIR/LTK2 writes are 42 bytes each (17-byte zero prefix + 8-byte command
	// header + 17-byte body), confirmed in both joycon2_pairing_DECRYPTED_ltk2.pcap (frame 525) and the
	// decrypted reconnect capture (frames 1836/1840). A 42-byte write to a 33-byte characteristic is
	// rejected by the SoftDevice with ATT error 0x0D (Invalid Attribute Value Length) -- so even with
	// everything else correct, the key exchange could never have completed. 64 leaves headroom for any
	// longer command shape not yet seen.
	chrUnk65a7.setMaxLen(64);
	chrUnk65a7.setWriteCallback(chrHandshakeCb);
	chrUnk65a7.begin();

	chrUnk4147.setProperties(CHR_PROPS_WRITE_WO_RESP);
	chrUnk4147.setPermission(SECMODE_OPEN, SECMODE_OPEN);
	chrUnk4147.setMaxLen(20);
	chrUnk4147.setWriteCallback(unkWriteCb);
	chrUnk4147.begin();

	chrCmdResp.setProperties(CHR_PROPS_NOTIFY);
	chrCmdResp.setPermission(SECMODE_OPEN, SECMODE_OPEN); // WRITE PROBE
	chrCmdResp.setMaxLen(128);
	chrCmdResp.setCccdWriteCallback(cccdCb);
	chrCmdResp.begin();
	armProbe(chrCmdResp.handles().value_handle, "COMMAND_RESPONSE value");
	armProbe(addVarLenDescriptor(UUID_DESC_B746, 2, SECMODE_OPEN, SECMODE_OPEN),
		 "COMMAND_RESPONSE b746 desc");

	chrUnk640c.setProperties(CHR_PROPS_NOTIFY);
	chrUnk640c.setCccdWriteCallback(cccdCb); // no callback was ever registered here before -- meant we had
						  // zero visibility into whether this CCCD actually gets
						  // subscribed on our connection at all, only assumed it did.
	chrUnk640c.setPermission(SECMODE_OPEN, SECMODE_OPEN); // WRITE PROBE
	// 33, not the real device's presumed 1 -- raised 2026-08-06 so notify() (used by both the 'A' ack and
	// 'X' echo handshake-reply experiments) can't be silently truncated. The real device's actual value
	// length here is unknown/irrelevant for our purposes since we're using this purely as an outbound
	// diagnostic channel, not replicating real content.
	chrUnk640c.setMaxLen(33);
	chrUnk640c.begin();
	armProbe(chrUnk640c.handles().value_handle, "640c value");
	armProbe(addVarLenDescriptor(UUID_DESC_B746, 2, SECMODE_OPEN, SECMODE_OPEN), "640c b746 desc");

	chrUnkD3bd.setProperties(CHR_PROPS_NOTIFY);
	chrUnkD3bd.setCccdWriteCallback(cccdCb); // same gap as chrUnk640c above -- never had visibility before
	chrUnkD3bd.setPermission(SECMODE_OPEN, SECMODE_OPEN); // WRITE PROBE
	chrUnkD3bd.setMaxLen(20); // was 1 -- any write would be rejected by the SoftDevice INVISIBLY (audit RISK-1)
	chrUnkD3bd.begin();
	armProbe(chrUnkD3bd.handles().value_handle, "D3bd value");
	armProbe(addVarLenDescriptor(UUID_DESC_B746, 2, SECMODE_OPEN, SECMODE_OPEN), "D3bd b746 desc");

	chrUnkFde.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
	chrUnkFde.setPermission(SECMODE_OPEN, SECMODE_OPEN); // WRITE PROBE
	chrUnkFde.setMaxLen(20); // was setFixedLen(1); the write below keeps a READ returning the same 1 byte
	chrUnkFde.setReadAuthorizeCallback(readAuthCb, false); // log ATT reads; false = BLE ctx
	chrUnkFde.begin();
	chrUnkFde.write(&zero1, 1);
	armProbe(chrUnkFde.handles().value_handle, "fde value");
	armProbe(addVarLenDescriptor(UUID_DESC_679D, 2, SECMODE_OPEN, SECMODE_OPEN), "fde 679d desc");

	chrUnkFdf.setProperties(CHR_PROPS_WRITE_WO_RESP);
	chrUnkFdf.setPermission(SECMODE_OPEN, SECMODE_OPEN);
	chrUnkFdf.setMaxLen(20);
	chrUnkFdf.setWriteCallback(unkWriteCb);
	chrUnkFdf.begin();

	// ...fd2 moved to the END of the service, swapping places with d5a9e01e (2026-08-14). The console
	// never subscribes fd2 and never writes it in any capture, so its position is free -- and giving the
	// front position back to d5a9 is what puts our report notifications on 0x000E, where a real device
	// sends them. fd2's descriptor stays writable and probe-armed so a write here would still be VISIBLE.
	chrInput.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
	chrInput.setPermission(SECMODE_OPEN, SECMODE_OPEN); // WRITE PROBE -- see armProbe()
	chrInput.setMaxLen(63); // variable: a short write must LAND, not be bounced
	chrInput.setReadAuthorizeCallback(readAuthCb, false); // false = BLE ctx, not deferred
	chrInput.setCccdWriteCallback(cccdCb);
	chrInput.begin();
	chrInput.write(g_ownTemplates ? INPUT_REPORT_TEMPLATE_OWN : INPUT_REPORT_TEMPLATE,
			 sizeof INPUT_REPORT_TEMPLATE);
	{
		uint16_t hFd2Desc = addVarLenDescriptor(UUID_DESC_679D, 2, SECMODE_OPEN, SECMODE_OPEN);
		armProbe(chrInput.handles().value_handle, "fd2 value");
		armProbe(hFd2Desc, "fd2 679d desc (NOT the trigger -- armed for visibility)");
		Serial.printf("# LEAN fd2 (not the report chan): value=0x%04X cccd=0x%04X 679d-desc=0x%04X\n",
			      chrInput.handles().value_handle, chrInput.handles().cccd_handle, hFd2Desc);
	}

	// The bd28x service moved to the END too. The console reads bd281 and bd283 BY UUID (Read By Type
	// over 0x0001-0xFFFF), which finds them at any handle, so only their ORDER relative to the main
	// service mattered -- and the main service needed the low handles.
	bleSvc1.begin();
	// bd281's real value CONFIRMED FIXED/UNIVERSAL 2026-08-06: byte-identical across the right AND left
	// Joy-Con (both real captures' first pre-notification Read By Type Response, handle 0x0002). Previously
	// sent as a 1-byte all-zero stub -- wrong on both length (real is 7 bytes) and content. This is read by
	// the Switch very early, before the CCCD/notification exchange even starts.
	// bd281's value is a POINTER TO bd282, not an opaque constant -- found 2026-08-08 by asking why the
	// console writes to 0x0005 when it happily found bd281 by UUID at our own handle 0x0031.
	//
	//   real bd281 value:  04 00 05 00 01 01 00
	//                      ^^^^^ ^^^^^
	//                      0x0004 0x0005  = bd282's DECLARATION and VALUE handles on a real device
	//
	// The console reads bd281 wherever it is, parses those two little-endian handles out of it, and
	// writes to the second one. It is not hardcoding 0x0005 -- WE were sending it there, because this
	// value was copied byte-for-byte from a real controller. The "CONFIRMED FIXED/UNIVERSAL, byte-
	// identical across the right AND left Joy-Con" note was accurate and misleading: identical across two
	// devices that both keep bd282 at 0x0005.
	//
	// So it is built from our OWN handles below, after chrS1W exists to report them. This also retires
	// the conclusion that the SoftDevice's immovable GAP service (which owns 0x0001-0x0005 and can never
	// be given away) is a hard ceiling -- nothing needs to live at 0x0005 if we stop claiming it does.
	chrS1R1.setProperties(CHR_PROPS_READ);
	// Write PERMISSION is open even though the declared PROPERTIES stay read-only. Deliberate, and it is
	// a probe, not a feature.
	//
	// bd281's VALUE lands at handle 0x0010 on our device -- and 0x0010 is exactly where a real Joy-Con
	// keeps the 679d descriptor that the console writes 0x0085 to in order to start the report stream.
	// The console is proven to use hardcoded handles on its recognized path; if it also hardcodes THIS
	// write on the generic path, it has been writing 0x0085 into our read-only bd281 all along, getting
	// ATT 0x03 Write Not Permitted, and giving up ~59ms later -- which is exactly the failure we see.
	// We could never observe it because the SoftDevice rejects a write to a read-only attribute without
	// raising any application event.
	//
	// Permissions are what the SoftDevice enforces; the properties field is only advisory to the client.
	// So opening write permission while leaving the declaration read-only makes the write SUCCEED and
	// surface as BLE_GATTS_EVT_WRITE, without altering what the console sees during discovery -- which
	// matters, because changing the declaration could itself change the console's behavior and confound
	// the result. Watch for a write of "8500" to handle 0x0010 in rawBleEventCb's GATTS write log.
	chrS1R1.setPermission(SECMODE_OPEN, SECMODE_OPEN); // WRITE PROBE, re-armed 2026-08-13 (see armProbe())
	chrS1R1.setMaxLen(8); // variable; the 7-byte pointer value is written below, so a READ is unchanged
	chrS1R1.setReadAuthorizeCallback(readAuthCb, false); // log ATT reads; false = BLE ctx
	chrS1R1.begin();
	g_hS1R1Val = chrS1R1.handles().value_handle;

	chrS1W.setProperties(CHR_PROPS_WRITE);
	chrS1W.setPermission(SECMODE_OPEN, SECMODE_OPEN);
	chrS1W.setMaxLen(20);
	chrS1W.setWriteCallback(unkWriteCb);
	chrS1W.begin();

	// Now bd282 has handles, so bd281 can honestly advertise where it lives. Trailing 01 01 00 is copied
	// from the real value; its meaning is unknown and it does not look handle-shaped, so it is left alone.
	{
		uint16_t bd282Val = chrS1W.handles().value_handle;
		uint16_t bd282Decl = (uint16_t)(bd282Val - 1); // a characteristic's declaration precedes its value
		uint8_t bd281Val[7] = { (uint8_t)(bd282Decl & 0xFF), (uint8_t)(bd282Decl >> 8),
					(uint8_t)(bd282Val & 0xFF),  (uint8_t)(bd282Val >> 8),
					0x01, 0x01, 0x00 };
		chrS1R1.write(bd281Val, sizeof bd281Val);
		Serial.printf("# LEAN bd281 -> bd282 decl=0x%04X val=0x%04X (real device says 0x0004/0x0005)\n",
			      bd282Decl, bd282Val);
	}

	// bd283's real value DIFFERS between the right and left Joy-Con (unlike bd281) -- likely some kind of
	// per-device identity/factory value. We don't have a legitimate value of our own, so using the real
	// right-Joy-Con's captured bytes as the closest available stand-in -- previously an all-zero stub was
	// used here too (also wrong on length: real is 8 bytes, not 1).
	chrS1R2.setProperties(CHR_PROPS_READ);
	chrS1R2.setPermission(SECMODE_OPEN, SECMODE_OPEN); // WRITE PROBE
	chrS1R2.setMaxLen(sizeof BD283_UNIQUE); // variable; written below, so a READ is unchanged
	chrS1R2.setReadAuthorizeCallback(readAuthCb, false); // log ATT reads; false = BLE ctx
	chrS1R2.begin();
	// 'Q' toggles bd283 INDEPENDENTLY of the serial. The 'I' toggle bundles bd283 with the malformed
	// "OPENPUCKBLE001" serial, so it can never test this one value cleanly. bd283 matters because the
	// 2026-08-08 registration capture shows the console reads exactly two things by UUID before it does
	// anything else -- bd281 (ours matches the real value byte-for-byte) and bd283, which is the ONLY
	// value in the whole exchange that we invent. Identity was "ruled out" in an earlier session, but
	// that test ran when the flow still died at PAIR/LTK2, far short of where it now reaches.
	chrS1R2.write(g_bd283Captured ? BD283_CAPTURED : BD283_UNIQUE, sizeof BD283_UNIQUE);
	armProbe(chrS1R2.handles().value_handle, "bd283 value");
	armProbe(chrS1R1.handles().value_handle, "bd281 value");

	// Stash assigned ATT handles for on-demand query via 'S'. Real handles (confirmed exact via a bleak
	// GATT dump against the actual device, 2026-08-05): COMMAND_WRITE=0x0013, COMMAND_RESPONSE=0x0019.
	// Ours will always start a few handles later since Generic Access/Attribute are mandatorily
	// auto-inserted by the SoftDevice at the lowest handles and can't be removed or relocated -- the real
	// device's custom service starts at handle 0x0001 with no Generic Access before it at all, which is
	// not achievable on this stack. Exact absolute match is therefore not expected; this is about getting
	// the *relative* spacing between our own characteristics to mirror the real device's exactly.
	// Verify EVERY characteristic actually registered. Bluefruit's begin() fails SILENTLY when the
	// attribute table or the vendor-UUID slots run out -- no error, no return value checked, the handle
	// just stays 0x0000 and the service is quietly truncated. A central then bails partway through
	// discovery, which looks exactly like a protocol problem and sends you hunting in the wrong place.
	// That is what phase 6 did on 2026-08-07, and it was invisible because the status dump only listed
	// five of the twelve characteristics. Check all of them, loudly.
	printGattCheck();

	// Every attribute that can now ACCEPT a write it would previously have bounced invisibly. A hit on
	// any of these is a project-changing event, so print the map once at boot to make an unexpected
	// handle in the write log immediately attributable.
	Serial.printf("# LEAN write probes armed on %u attributes:\n", g_nProbes);
	for (uint8_t i = 0; i < g_nProbes; i++)
		Serial.printf("# LEAN   0x%04X  %s\n", g_probes[i].handle, g_probes[i].name);

	g_hInputVal = chrInput.handles().value_handle;
	g_hInputCccd = chrInput.handles().cccd_handle;
	g_hCmdWriteVal = chrCmdWrite.handles().value_handle;
	g_hCmdRespVal = chrCmdResp.handles().value_handle;
	g_hCmdRespCccd = chrCmdResp.handles().cccd_handle;
	g_hVibrationVal = chrVibration.handles().value_handle;
}

// Callable from setupGatt() AND the 'S' console command -- the boot-time print happens before any serial
// listener can attach, which made the one piece of information we needed unreadable.
static void printGattCheck()
{
	{
		struct { const char *name; BLECharacteristic *c; } all[] = {
			{ "S1R1(bd281)", &chrS1R1 },   { "S1W(bd282)", &chrS1W },
			{ "S1R2(bd283)", &chrS1R2 },   { "INPUT_REPORT", &chrInput },
			{ "d5a9(reports)", &chrUnkD5a9 }, { "vibration", &chrVibration },
			{ "COMMAND_WRITE", &chrCmdWrite }, { "65a7(cmd-in)", &chrUnk65a7 },
			{ "4147", &chrUnk4147 },       { "COMMAND_RESPONSE", &chrCmdResp },
			{ "640c", &chrUnk640c },       { "d3bd", &chrUnkD3bd },
			{ "fde", &chrUnkFde },         { "fdf", &chrUnkFdf },
		};
		uint8_t bad = 0;
		for (auto &e : all) {
			uint16_t h = e.c->handles().value_handle;
			if (h == 0) {
				Serial.printf("# LEAN *** GATT FAILURE: %s handle=0x0000 (begin() failed)\n",
					      e.name);
				bad++;
			}
		}
		Serial.printf("# LEAN GATT check: %u/%u characteristics registered%s\n",
			      (unsigned)(sizeof all / sizeof all[0]) - bad,
			      (unsigned)(sizeof all / sizeof all[0]),
			      bad ? "  <-- BROKEN, discovery will fail" : " -- all OK");
	}
}

// ============================================================================================
// GATTC (client-role) support -- discover the Switch's OWN GATT table live, and send the wake-sequence
// writes to it properly serialized. Added 2026-08-05 after a live test showed sendWakeSequence()'s old
// fire-all-8-writes-synchronously approach silently dropped 6 of 8 writes: the SoftDevice only allows one
// outstanding confirmed write (WRITE_REQ) per connection at a time, and WRITE_CMD has a small queue depth
// -- everything after the first either got NRF_ERROR_BUSY (0x11) or NRF_ERROR_RESOURCES (0x13) and was
// never sent. The one write that DID go out got a real ATT-level response from the Switch itself
// (status=0x0106 = attribute-error base 0x0100 + 0x06 Request Not Supported) -- proof two-way ATT traffic
// with the Switch's own GATT server works over this link; the hardcoded handle just didn't mean anything
// in a fresh-pairing context (it came from a completely different already-bonded wake-from-sleep capture).
// ============================================================================================

// The real controller streams from d5a9e01e even though the Switch never subscribes that CCCD -- Nintendo's
// stack notifies regardless. The SoftDevice will NOT: sd_ble_gatts_hvx() returns NRF_ERROR_INVALID_STATE
// unless the CCCD says notifications are on. A CCCD is just an attribute though, so we can set our own copy
// for this connection; the peer never sees it and never needs to have asked.
static void forceEnableReportCccd()
{
	if (g_connHdl == BLE_CONN_HANDLE_INVALID)
		return;
	// 'N' clears this to run the control test: does the console behave differently if we DON'T stream?
	// We have always forced the stream on, so "unsolicited notifications on a channel the console never
	// subscribed" has never been ruled out as the reason it hangs up -- a well-behaved host may treat that
	// as a protocol violation. Testing the inverse of a long-held assumption is cheap; assuming is not.
	if (!g_forceStream)
		return;
	// sd_ble_gatts_value_set() on a CCCD is FORBIDDEN (0x0F), and sd_ble_gatts_hvx() then fails with
	// INVALID_STATE (0x08) -- confirmed live, so the gate is the SoftDevice's, not Bluefruit's. The only
	// remaining route is the system-attribute blob, which is where the SoftDevice actually keeps CCCD
	// state. Read the current blob (it holds the three CCCDs the Switch legitimately subscribed), append
	// an enabled entry for our report channel, and write it back. Entry format is
	// {uint16 handle, uint16 len, uint8 value[len]}; appending must not disturb the existing entries or
	// COMMAND_RESPONSE notifications stop working and the whole handshake dies.
	uint8_t blob[256];
	uint16_t blobLen = sizeof blob;
	uint32_t err = sd_ble_gatts_sys_attr_get(g_connHdl, blob, &blobLen,
						 BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
	if (err != NRF_SUCCESS) {
		Serial.printf("# LEAN sys_attr_get failed err=0x%08lX\n", (unsigned long)err);
		return;
	}
	// Dump the raw blob. The layout is opaque in Nordic's docs and I was guessing at it; sys_attr_set
	// now returns SUCCESS but hvx still says INVALID_STATE, which means the bytes I patched are not the
	// ones that hold the enable bit. Print it so the structure can be decoded from real data rather than
	// assumed -- the same mistake that cost us the whole controller-info template earlier.
	if (!g_quiet) {
		Serial.printf("# LEAN sys_attr blob (%u bytes, our CCCD=0x%04X): ", blobLen, g_hD5a9Cccd);
		for (uint16_t i = 0; i < blobLen && i < 64; i++)
			Serial.printf("%02X", blob[i]);
		Serial.println();
	}

	// Already present? Then just make sure it says "notifications on".
	bool patched = false;
	for (uint16_t i = 0; i + 4 <= blobLen;) {
		uint16_t h = (uint16_t)(blob[i] | (blob[i + 1] << 8));
		uint16_t l = (uint16_t)(blob[i + 2] | (blob[i + 3] << 8));
		if (h == g_hD5a9Cccd && l >= 1) {
			blob[i + 4] = 0x01;
			patched = true;
			break;
		}
		if (l == 0 || i + 4 + l > blobLen)
			break;
		i += 4 + l;
	}
	// Insert BEFORE the 2-byte CRC trailer, not after it.
	//
	// blobLen < 2 means the SoftDevice holds NO user sys-attrs yet -- which is the normal state at
	// connect time, i.e. exactly when bleConnectCb calls this. The old guard was `blobLen >= 2`, so on
	// that path the append was skipped, the CRC block below was skipped, and sys_attr_set was called
	// with length 0: it returns NRF_SUCCESS and the line at the end printed "(appended, 0 bytes) -- OK"
	// having enabled nothing whatsoever. Build the blob from scratch in that case instead: one entry
	// plus a fresh CRC trailer. `at` is where the old CRC trailer starts (or 0 when there is none) --
	// the trailer is recomputed below either way, so overwriting it is intended.
	bool appended = false;
	if (!patched) {
		uint16_t at = (blobLen >= 2) ? (uint16_t)(blobLen - 2) : 0;
		if ((uint32_t)at + 8 <= sizeof blob) {
			blob[at + 0] = (uint8_t)(g_hD5a9Cccd & 0xFF);
			blob[at + 1] = (uint8_t)(g_hD5a9Cccd >> 8);
			blob[at + 2] = 0x02;
			blob[at + 3] = 0x00;
			blob[at + 4] = 0x01; // notifications enabled
			blob[at + 5] = 0x00;
			blobLen = (uint16_t)(at + 8); // entry + 2-byte CRC trailer written below
			appended = true;
		} else {
			Serial.printf("# LEAN *** force-enable report CCCD (0x%04X) DID NOTHING: blob is %u "
				      "bytes, no room to append ***\n", g_hD5a9Cccd, blobLen);
			return;
		}
	}
	// The blob ends with a CRC16 over everything before it. Patching an entry without recomputing it gets
	// NRF_ERROR_INVALID_DATA (0x0B) -- which is exactly what happened first time. Nordic's crc16_compute,
	// CRC16-CCITT seeded 0xFFFF.
	if (blobLen >= 2) {
		uint16_t crc = 0xFFFF;
		for (uint16_t i = 0; i < (uint16_t)(blobLen - 2); i++) {
			crc = (uint16_t)((uint8_t)(crc >> 8) | (crc << 8));
			crc ^= blob[i];
			crc ^= (uint16_t)((uint8_t)(crc & 0xFF) >> 4);
			crc ^= (uint16_t)((crc << 8) << 4);
			crc ^= (uint16_t)(((crc & 0xFF) << 4) << 1);
		}
		blob[blobLen - 2] = (uint8_t)(crc & 0xFF);
		blob[blobLen - 1] = (uint8_t)(crc >> 8);
	}
	err = sd_ble_gatts_sys_attr_set(g_connHdl, blob, blobLen, BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
	// Say WHICH path ran. "appended" used to be printed whenever the entry was not found, including the
	// case where nothing was appended either, so the log could not distinguish success from a no-op.
	if (!g_quiet || err)
		Serial.printf("# LEAN force-enable report CCCD (0x%04X) via sys_attr (%s, %u bytes): err=0x%08lX%s\n",
		      g_hD5a9Cccd, patched ? "patched existing entry" :
				   appended ? "appended new entry" : "NO CHANGE MADE",
		      blobLen, (unsigned long)err, err ? "  <-- FAILED" : " -- OK");
}

static void printUuid(const ble_uuid_t &u)
{
	if (u.type == BLE_UUID_TYPE_BLE) {
		Serial.printf("0x%04X", u.uuid);
	} else {
		uint8_t buf[16];
		uint8_t len;
		if (sd_ble_uuid_encode(&u, &len, buf) == NRF_SUCCESS && len == 16) {
			for (int i = 15; i >= 0; i--)
				Serial.printf("%02x", buf[i]);
		} else {
			Serial.printf("type=%u short=0x%04X(undecoded)", u.type, u.uuid);
		}
	}
}

// ---- Discovery: Discover All Primary Services, then all characteristics within each, on the SWITCH's
// side. Paginates both stages (a single response is bounded by ATT MTU and won't necessarily list
// everything). Triggered on demand via the 'D' console command.
#define MAX_DISC_SERVICES 12
struct DiscSvc {
	uint16_t start, end;
	ble_uuid_t uuid;
};
static DiscSvc g_discSvcs[MAX_DISC_SERVICES];
static uint8_t g_discSvcCount = 0;
static int8_t g_discCharSvcIdx = -1; // -1 = not currently discovering characteristics

static void startCharDiscoveryForService(uint8_t idx)
{
	g_discCharSvcIdx = (int8_t)idx;
	ble_gattc_handle_range_t range;
	range.start_handle = g_discSvcs[idx].start;
	range.end_handle = g_discSvcs[idx].end;
	Serial.printf("# LEAN discovering chars for SVC[%u] (0x%04X-0x%04X)\n", idx,
		      range.start_handle, range.end_handle);
	sd_ble_gattc_characteristics_discover(g_connHdl, &range);
}

static void startServiceDiscovery()
{
	g_discSvcCount = 0;
	g_discCharSvcIdx = -1;
	Serial.println("# LEAN starting Switch GATT discovery (Discover All Primary Services)...");
	sd_ble_gattc_primary_services_discover(g_connHdl, 1, NULL);
}

// ---- Wake-sequence write queue: exact replay of what a real Joy-Con 2 sends when its Home button wakes a
// sleeping Switch (captured via sniffer 2026-08-05) -- now properly
// serialized (one outstanding op at a time) instead of fired all at once. Handles are still hardcoded from
// that one capture; 'D' discovery above exists to find out whether they're even valid in a given session.
struct GattcWriteOp {
	uint16_t handle;
	uint8_t op;
	uint8_t len;
	uint8_t data[33];
};
#define WAKE_QUEUE_MAX 8
static GattcWriteOp g_wakeQueue[WAKE_QUEUE_MAX];
static uint8_t g_wakeQueueCount = 0;
static uint8_t g_wakeQueueIdx = 0;
static bool g_wakeQueueActive = false;
static unsigned long g_wakeQueueLastTryMs = 0;

static void wakeQueuePush(uint16_t handle, uint8_t op, const uint8_t *data, uint8_t len)
{
	if (g_wakeQueueCount >= WAKE_QUEUE_MAX || len > sizeof(GattcWriteOp::data))
		return;
	GattcWriteOp &o = g_wakeQueue[g_wakeQueueCount++];
	o.handle = handle;
	o.op = op;
	o.len = len;
	memcpy(o.data, data, len);
}

// Attempts to send the current queue index; on NRF_ERROR_BUSY/RESOURCES it just returns and leaves the
// index unchanged so bleHeartbeat()'s periodic retry (or the next WRITE_RSP-triggered call) tries again.
// WRITE_REQ (confirmed) ops only advance on a matching BLE_GATTC_EVT_WRITE_RSP; WRITE_CMD ops advance
// immediately once the SoftDevice accepts them (no per-write response exists for that op type).
static void wakeQueueTrySend()
{
	if (!g_wakeQueueActive)
		return;
	if (g_wakeQueueIdx >= g_wakeQueueCount) {
		g_wakeQueueActive = false;
		Serial.println("# LEAN wake queue: all ops sent");
		return;
	}
	g_wakeQueueLastTryMs = millis();
	GattcWriteOp &o = g_wakeQueue[g_wakeQueueIdx];
	ble_gattc_write_params_t params = {};
	params.write_op = o.op;
	params.handle = o.handle;
	params.len = o.len;
	params.p_value = o.data;
	uint32_t err = sd_ble_gattc_write(g_connHdl, &params);
	Serial.printf("# LEAN GATTC write[%u/%u]: handle=0x%04X op=%u len=%u err=0x%08lX t=%lu ms\n",
		      g_wakeQueueIdx + 1, g_wakeQueueCount, o.handle, o.op, o.len,
		      (unsigned long)err, (unsigned long)millis());
	if (err == NRF_SUCCESS && o.op == BLE_GATT_OP_WRITE_CMD) {
		g_wakeQueueIdx++;
		wakeQueueTrySend();
	}
	// WRITE_REQ success: wait for BLE_GATTC_EVT_WRITE_RSP (see rawBleEventCb) to advance.
	// Any error: leave index as-is, retried by the next heartbeat tick or WRITE_RSP.
}

static void sendWakeSequence(uint16_t conn_hdl)
{
	(void)conn_hdl; // queue always targets g_connHdl, set by bleConnectCb before this can run
	g_wakeQueueCount = 0;
	g_wakeQueueIdx = 0;

	static const uint8_t enableNotify[2] = { 0x01, 0x00 };
	wakeQueuePush(0x0005, BLE_GATT_OP_WRITE_REQ, enableNotify, 2);
	wakeQueuePush(0x001b, BLE_GATT_OP_WRITE_REQ, enableNotify, 2);
	wakeQueuePush(0x001f, BLE_GATT_OP_WRITE_REQ, enableNotify, 2);
	wakeQueuePush(0x0023, BLE_GATT_OP_WRITE_REQ, enableNotify, 2);

	static const uint8_t cmd07[25] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
					   0x91, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 };
	static const uint8_t cmdMemRead[33] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x02, 0x91, 0x01, 0x04, 0x00, 0x08, 0x00,
						0x00, 0x40, 0x7E, 0x00, 0x00, 0x00, 0x30, 0x01,
						0x00 };
	static const uint8_t cmd10[25] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
					   0x91, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 };
	static const uint8_t cmd16[25] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
					   0x91, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 };
	wakeQueuePush(0x0016, BLE_GATT_OP_WRITE_CMD, cmd07, sizeof cmd07);
	wakeQueuePush(0x0016, BLE_GATT_OP_WRITE_CMD, cmdMemRead, sizeof cmdMemRead);
	wakeQueuePush(0x0016, BLE_GATT_OP_WRITE_CMD, cmd10, sizeof cmd10);
	wakeQueuePush(0x0016, BLE_GATT_OP_WRITE_CMD, cmd16, sizeof cmd16);

	g_wakeQueueActive = true;
	wakeQueueTrySend();
}

// Raw softdevice event logging for GATTC (client-role) events -- Bluefruit's high-level API doesn't
// expose these since it's normally used peripheral-only.
static void rawBleEventCb(ble_evt_t *evt)
{
	switch (evt->header.evt_id) {
	case BLE_GATTC_EVT_WRITE_RSP:
		Serial.printf("# LEAN GATTC write rsp: handle=0x%04X status=0x%04X t=%lu ms\n",
			      evt->evt.gattc_evt.params.write_rsp.handle,
			      evt->evt.gattc_evt.gatt_status, (unsigned long)millis());
		if (g_wakeQueueActive && g_wakeQueueIdx < g_wakeQueueCount &&
		    g_wakeQueue[g_wakeQueueIdx].op == BLE_GATT_OP_WRITE_REQ) {
			g_wakeQueueIdx++;
			wakeQueueTrySend();
		}
		break;
	case BLE_GATTC_EVT_READ_RSP:
		Serial.printf("# LEAN GATTC read rsp: handle=0x%04X status=0x%04X len=%u t=%lu ms\n",
			      evt->evt.gattc_evt.params.read_rsp.handle,
			      evt->evt.gattc_evt.gatt_status, evt->evt.gattc_evt.params.read_rsp.len,
			      (unsigned long)millis());
		break;
	// Every GATT-server write, including writes to descriptors -- which Bluefruit's high-level API has no
	// callback for. This is how the report-stream trigger is detected (see below), and it doubles as
	// visibility into any write to an attribute we haven't wired a handler to.
	case BLE_GATTS_EVT_HVN_TX_COMPLETE: {
		uint8_t n = evt->evt.gatts_evt.params.hvn_tx_complete.count;
		g_hvnOutstanding = (g_hvnOutstanding > n) ? (uint8_t)(g_hvnOutstanding - n) : 0;
		// WHO GOES QUIET FIRST. This event only fires once the PEER has acknowledged a notification, so
		// it is a receive-side heartbeat we get for free -- and unlike the sniffer, it cannot lose the
		// connection. If the console stops acknowledging, these stop, our flow-control gate latches shut
		// and we simply stop queueing: 994 sent / 0 failed looks perfectly healthy right up to the
		// supervision timeout. Reported at disconnect (see bleDisconnectCb) against the 2000ms timeout.
		{
			// SERVICE-RATE measurement. The negotiated interval is 10ms in BOTH the dying and the
			// surviving configuration, so raw frequency is not the difference -- but packets drained
			// per connection event still can be. The gap between acknowledgments is how much airtime
			// the console ACTUALLY gives us. A large max gap means we were starved; identical stats in
			// both configurations would mean radio time is irrelevant and the link-count effect is
			// console-side policy, which no firmware change of ours could reach.
			unsigned long nowMs = millis();
			if (g_lastTxCompleteMs) {
				unsigned long gap = nowMs - g_lastTxCompleteMs;
				if (gap > g_ackGapMaxMs)
					g_ackGapMaxMs = gap;
			}
		}
		g_lastTxCompleteMs = millis();
		// Count them. g_reportsSent counts hvx() returning NRF_SUCCESS, which means QUEUED, not sent --
		// and a sniffer capture on 2026-08-09 showed our peripheral emitting 33 EMPTY PDUs/sec and ZERO
		// data packets through an entire streaming window in which the firmware claimed "451 sent, 0
		// failed". Either the sniffer is missing our data packets or forceEnableReportCccd() makes hvx
		// accept notifications the SoftDevice then never transmits. This counter decides it: TX_COMPLETE
		// fires only on PEER ACKNOWLEDGMENT, so if acks ~= reports queued we really are streaming and the
		// sniffer is at fault; if acks stay near zero while reports climb, we have been silent all along --
		// which would invalidate every report content/rate/silence test ever run.
		g_txCompleteCount += n;
		break;
	}
	case BLE_GATTS_EVT_WRITE: {
		const ble_gatts_evt_write_t &w = evt->evt.gatts_evt.params.write;
		g_lastRxFromConsoleMs = millis(); // console-is-still-talking heartbeat, see g_lastTxCompleteMs
		// *** THIS BRANCH IS LIVE AND CRITICAL -- do not mark it dead again. ***
		// STALE-14 (written earlier on 2026-08-09) claimed the 0x0085 stream-start "DOES NOT EXIST" and
		// that the console sends no stream trigger at all. BOTH FALSE, retracted the same day against a
		// fully decrypted both-directions capture of a real Joy-Con RECONNECT:
		//
		//   node ble_decrypt.js joycon2_reconnect_undecrypted.pcap
		//   1935 C->P  05 0004 00 12 1000 8500   ATT Write REQUEST, handle 0x0010, value 0x0085
		//   1938 P->C  01 0004 00 13             Joy-Con: Write Response
		//   1940 P->C  0f 0004 00 1b 1a00 ...    notify 0x01/0x0C on handle 0x001a
		//   1942 P->C  42 0004 00 1b 0e00 ...    INPUT REPORTS BEGIN on handle 0x000e
		//
		// So the trigger is REAL and it is the whole mechanism: 0x0085 -> ack -> 0x01/0x0C -> stream.
		// It is also the ONLY ATT Write Request in the entire reconnect (everything else is 0x52 Write
		// Command to the 0x0016 cmd channel), and the console does NO discovery on a reconnect -- 12x
		// 0x52 and 1x 0x12, zero MTU/Read-By-Type. It runs on CACHED handles, so our placement of the
		// report value at 0x000E / CCCD 0x000F / this descriptor at 0x0010 is correct and matters.
		//
		// CORRECTED 2026-08-13: "we have never received this write, so the console never issues it"
		// was NEVER a measurement. Until today the 679d descriptor was a FIXED 1-byte attribute
		// (Bluefruit's addDescriptor() hardcodes vlen = 0), so a 2-byte write was rejected by the
		// SoftDevice with ATT 0x0D and raised NO BLE_GATTS_EVT_WRITE -- this branch was unreachable by
		// construction and the serial log looked identical whether the console sent the trigger or not.
		// The descriptor is now 2 bytes and variable-length. Whether it sends the trigger is OPEN AGAIN.
		if (w.handle == g_hS1R1Val) {
			uint16_t v = (w.len >= 2) ? (uint16_t)(w.data[0] | (w.data[1] << 8)) : w.data[0];
			Serial.printf("# LEAN *** 0x%04X written to bd281 (0x%04X) -- UNEXPECTED, nothing should "
				      "write here ***\n", v, w.handle);
			if (v == 0x0085) {
				forceEnableReportCccd();
				sendResponseOn(chrCmdResp, 0x01, 0x0c, g_ownMisc ? RSP_01_0C_OWN : RSP_01_0C,
						       sizeof RSP_01_0C);
				g_streaming = true;
				Serial.println("# LEAN *** INPUT REPORT STREAM ENABLED (via hardcoded handle) ***");
			}
			break;
		}
		if (w.handle == g_hD5a9Desc) {
			uint16_t v = (w.len >= 2) ? (uint16_t)(w.data[0] | (w.data[1] << 8)) : w.data[0];
			Serial.printf("# LEAN report-stream trigger: 0x%04X written to 679d desc (0x%04X)\n", v,
				      w.handle);
			// The real controller streams from d5a9e01e even though the Switch never subscribes its
			// CCCD -- Nintendo's stack notifies regardless. The SoftDevice will NOT: sd_ble_gatts_hvx()
			// fails with NRF_ERROR_INVALID_STATE unless the CCCD says notifications are on. So set our
			// own CCCD for this connection. It is a normal attribute; writing it locally is legitimate
			// and invisible to the peer.
			forceEnableReportCccd();
			if (v & 0x0001) {
				// One unsolicited notification precedes the first report on a real device.
				sendResponseOn(chrCmdResp, 0x01, 0x0c, g_ownMisc ? RSP_01_0C_OWN : RSP_01_0C,
						       sizeof RSP_01_0C);
				g_streaming = true;
				g_armedMs = millis(); // stamp the arm for hvx-failure timing
				g_autoTick = 0;       // phase-lock the button press to the arm
				Serial.println("# LEAN *** INPUT REPORT STREAM ENABLED ***");
			}
		} else if (w.handle != g_hCmdWriteVal && w.handle != g_hVibrationVal) {
			// A probe firing means the console wrote somewhere this firmware was previously deaf
			// to -- the exact failure mode that hid the 679d descriptor for weeks. Shout about it.
			const char *pn = probeName(w.handle);
			// A probe hit ALWAYS prints -- it is the one thing worth blocking for. The routine
			// per-command dump is gated: the console repeats the whole handshake every ~2 s in the
			// loop, so it floods the CDC right before the arm.
			if (pn) {
				Serial.printf("# LEAN *** WRITE PROBE HIT: %s (0x%04X) len=%u data=", pn,
					      w.handle, w.len);
				for (uint16_t i = 0; i < w.len && i < 16; i++)
					Serial.printf("%02X", w.data[i]);
				Serial.println(" ***");
			} else if (!g_quiet) {
				Serial.printf("# LEAN GATTS write: handle=0x%04X len=%u data=", w.handle, w.len);
				for (uint16_t i = 0; i < w.len && i < 16; i++)
					Serial.printf("%02X", w.data[i]);
				Serial.println();
			}
		}
		break;
	}
	case BLE_GATTC_EVT_TIMEOUT:
		Serial.printf("# LEAN GATTC timeout t=%lu ms\n", (unsigned long)millis());
		break;
	case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP: {
		const ble_gattc_evt_prim_srvc_disc_rsp_t &r =
			evt->evt.gattc_evt.params.prim_srvc_disc_rsp;
		uint16_t status = evt->evt.gattc_evt.gatt_status;
		uint16_t lastEnd = 0;
		if (status == BLE_GATT_STATUS_SUCCESS) {
			for (uint16_t i = 0; i < r.count; i++) {
				if (g_discSvcCount < MAX_DISC_SERVICES) {
					DiscSvc &s = g_discSvcs[g_discSvcCount];
					s.start = r.services[i].handle_range.start_handle;
					s.end = r.services[i].handle_range.end_handle;
					s.uuid = r.services[i].uuid;
					Serial.printf("# LEAN SVC[%u]: 0x%04X-0x%04X uuid=", g_discSvcCount,
						      s.start, s.end);
					printUuid(s.uuid);
					Serial.println();
					g_discSvcCount++;
				}
				lastEnd = r.services[i].handle_range.end_handle;
			}
		}
		if (status == BLE_GATT_STATUS_SUCCESS && lastEnd > 0 && lastEnd < 0xFFFF) {
			sd_ble_gattc_primary_services_discover(g_connHdl, lastEnd + 1, NULL);
		} else {
			Serial.printf(
				"# LEAN service discovery complete (status=0x%04X), %u service(s) found.\n",
				status, g_discSvcCount);
			if (g_discSvcCount > 0)
				startCharDiscoveryForService(0);
		}
		break;
	}
	case BLE_GATTC_EVT_CHAR_DISC_RSP: {
		const ble_gattc_evt_char_disc_rsp_t &r = evt->evt.gattc_evt.params.char_disc_rsp;
		uint16_t status = evt->evt.gattc_evt.gatt_status;
		uint16_t lastValHandle = 0;
		if (status == BLE_GATT_STATUS_SUCCESS) {
			for (uint16_t i = 0; i < r.count; i++) {
				// char_props is a 1-byte bitfield struct (ble_gatt_char_props_t), not a plain
				// integer -- memcpy to print it as a hex flags byte.
				uint8_t props;
				memcpy(&props, &r.chars[i].char_props, 1);
				Serial.printf(
					"# LEAN   CHAR decl=0x%04X val=0x%04X props=0x%02X uuid=",
					r.chars[i].handle_decl, r.chars[i].handle_value, props);
				printUuid(r.chars[i].uuid);
				Serial.println();
				lastValHandle = r.chars[i].handle_value;
			}
		}
		uint16_t svcEnd = (g_discCharSvcIdx >= 0) ? g_discSvcs[g_discCharSvcIdx].end : 0;
		if (status == BLE_GATT_STATUS_SUCCESS && lastValHandle > 0 && lastValHandle < svcEnd) {
			ble_gattc_handle_range_t range;
			range.start_handle = lastValHandle + 1;
			range.end_handle = svcEnd;
			sd_ble_gattc_characteristics_discover(g_connHdl, &range);
		} else {
			uint8_t next = (uint8_t)(g_discCharSvcIdx + 1);
			if (next < g_discSvcCount) {
				startCharDiscoveryForService(next);
			} else {
				Serial.println("# LEAN characteristic discovery complete for all services.");
				g_discCharSvcIdx = -1;
			}
		}
		break;
	}
	// GAP-level security/encryption events -- added 2026-08-06 for direct, sniffer-independent proof of
	// whether the link ever actually gets encrypted after the proactive announcement. Bluefruit's high-level
	// API doesn't surface these to app code by default; the raw softdevice event callback sees everything.
	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		Serial.printf("# LEAN GAP: SEC_PARAMS_REQUEST t=%lu ms\n", (unsigned long)millis());
		break;
	// SEC_INFO_REQUEST is the event that proves the Switch sent LL_ENC_REQ. It is the ONLY one of
	// these that distinguishes "the Switch never asked to encrypt" from "it asked and we had no key" --
	// CONN_SEC_UPDATE and AUTH_STATUS below only fire on SUCCESS, so their absence has never been
	// evidence that LL_ENC_REQ didn't happen, contrary to how it was previously written up.
	// By the time this fires, Bluefruit's BLESecurity::_eventHandler() has ALREADY replied (it runs
	// before this user callback) -- with our bond key if deriveLinkKey() installed one, else with NULL,
	// which rejects encryption. So this is diagnostic only; the actual fix is upstream in deriveLinkKey().
	case BLE_GAP_EVT_SEC_INFO_REQUEST:
		Serial.printf(
			"# LEAN GAP: SEC_INFO_REQUEST t=%lu ms  <-- SWITCH SENT LL_ENC_REQ. link key %s\n",
			(unsigned long)millis(),
			g_haveLinkKey ? "was installed -- Bluefruit should have replied with it" :
					 "NOT available -- Bluefruit replied NULL, encryption will fail");
		break;
	case BLE_GAP_EVT_CONN_SEC_UPDATE: {
		const ble_gap_conn_sec_t &sec = evt->evt.gap_evt.params.conn_sec_update.conn_sec;
		Serial.printf(
			"# LEAN GAP: CONN_SEC_UPDATE sm=%u lv=%u encr_key_size=%u t=%lu ms  <-- ENCRYPTION STATE CHANGED\n",
			sec.sec_mode.sm, sec.sec_mode.lv, sec.encr_key_size, (unsigned long)millis());
		// Encryption succeeded, so this console has accepted us -- remember it, and from now on advertise
		// inviting it specifically rather than in open pairing mode. That is the difference between an
		// advert a sleeping Switch ignores and one it connects to.
		// Deliberately NOT saving the bonded host here. Encryption succeeding is too early -- doing it
		// here flipped us into WAKE advertising mid-registration and hid the puck from Change Grip/Order.
		// It is saved at 0x0C/0x04 instead (the last phase-6 step, i.e. registration genuinely complete)
		// in chrHandshakeCb. 'C' forgets it manually. (The BOND KEY is separate, written in
		// deriveLinkKey(); this only governs which advertisement we broadcast.)
		//
		// RE-ARM THE FORCED REPORT CCCD HERE. Bluefruit has ALREADY wiped it by the time this prints:
		// BLEConnection::_eventHandler() (BLEConnection.cpp:311-323) runs on this same event from
		// bluefruit.cpp:789, long before the user callback at :934, and does
		//     if ( !loadCccd() ) sd_ble_gatts_sys_attr_set(_conn_hdl, NULL, 0, 0);
		// loadCccd() skips two bond-file fields and reads a third length byte that deriveLinkKey() never
		// writes, so on a fresh registration it fails and the NULL set runs -- with flags = 0, which
		// restricts nothing and therefore resets EVERY CCCD on the connection, ours included. After that
		// sd_ble_gatts_hvx() on chrInput returns NRF_ERROR_INVALID_STATE (0x08) for the rest of the link,
		// so the stream phase 6 announces as open emits nothing at all.
		// g_connHdl is normally set in bleConnectCb, which Bluefruit dispatches DEFERRED on the Ada
		// callback task (bluefruit.cpp:829) while this runs in BLE event context -- so adopt the event's
		// handle rather than silently no-op'ing in forceEnableReportCccd() if the order ever inverts.
		if (g_connHdl == BLE_CONN_HANDLE_INVALID)
			g_connHdl = evt->evt.gap_evt.conn_handle;
		if (sec.sec_mode.lv > 1)
			forceEnableReportCccd();
		break;
	}
	case BLE_GAP_EVT_AUTH_STATUS:
		Serial.printf("# LEAN GAP: AUTH_STATUS auth_status=0x%02X t=%lu ms\n",
			      evt->evt.gap_evt.params.auth_status.auth_status, (unsigned long)millis());
		break;
	default: {
		// Catch-all. Everything unhandled used to be dropped silently, which means any event the
		// console provoked that we had not thought to name looked exactly like nothing happening --
		// the same blind spot that has cost this project several sessions. Log each event id once per
		// boot so a new one announces itself without flooding the line.
		// NOTE this still cannot see ATT READs: the SoftDevice answers those from the attribute table
		// with no application event at all unless read authorization is enabled.
		static uint16_t seen[24];
		static uint8_t nSeen = 0;
		uint16_t id = evt->header.evt_id;
		for (uint8_t i = 0; i < nSeen; i++)
			if (seen[i] == id)
				return;
		if (nSeen < 24)
			seen[nSeen++] = id;
		Serial.printf("# LEAN raw BLE event 0x%02X (unhandled, first time this boot) t=%lu ms\n", id,
			      (unsigned long)millis());
		break;
	}
	}
}

static void bleConnectCb(uint16_t conn_hdl)
{
	g_connHdl = conn_hdl;
	g_failThisConn = 0; // fresh failure budget per connection -- see the hvx failure logging
	// Start every connection at the same replay phase (review 2026-08-14 #3). These used to carry over,
	// so each connection began at an arbitrary point in the 256-frame capture and the console saw a
	// different opening IMU sequence every time -- one more uncontrolled variable behind the alternation.
	g_reportSeq = 0;
	g_motionFrame = 0;
	g_imuTs12 = 0; // the IMU clock belongs to this reset too (review 2026-08-14b #5); it wraps anyway,
		       // but leaving it out contradicted the stated intent of the block it sits in.
	g_firstReports = 0;
	g_armedMs = 0;
	BLEConnection *conn = Bluefruit.Connection(conn_hdl);
	ble_gap_addr_t peer = conn->getPeerAddr();
	savePeerAddr(peer);
	logEvent(1, 0);
	Serial.printf(
		"# LEAN CONNECT! peer=%02X:%02X:%02X:%02X:%02X:%02X type=%u t=%lu ms\n",
		peer.addr[5], peer.addr[4], peer.addr[3], peer.addr[2],
		peer.addr[1], peer.addr[0], peer.addr_type,
		(unsigned long)millis());
	// Enable our report channel's CCCD HERE, at connect, before any GATT activity. The blob dump proved
	// the patch location was right and sys_attr_set returns SUCCESS mid-connection -- but hvx still says
	// INVALID_STATE, so the SoftDevice evidently latches CCCD state at connection setup and a later set
	// is accepted-then-ignored. This is also the documented time to restore system attributes.
	//
	// DATA LENGTH EXTENSION. Found 2026-08-09 by finally getting a usable sniffer capture of our OWN link
	// and diffing packet SIZES against a real Joy-Con's:
	//
	// RETRACTED 2026-08-09: the packet-size figures once quoted here mixed BOTH DIRECTIONS -- the
	// small packets were the CONSOLE talking to us, not our reports fragmenting. Firmware counters
	// later proved 1836 notifications were acknowledged, so the sniffer simply was not recording our
	// data packets at all. Do not cite packet sizes from that capture.
	//
	// RETRACTED 2026-08-12: the old text here read "ESTABLISHED: we never send LL_LENGTH_REQ (0
	// occurrences)". That is FALSE. tshark on our own fresh-pairing capture shows LL_LENGTH_REQ
	// (control opcode 0x14) and LL_LENGTH_RSP (0x15) both present, and the firmware logs
	// "data-length update requested (251): ok=1" on every connection. DLE IS negotiated.
	//   tshark -r <cap>.pcap -Y "btle.control_opcode" -T fields -e btle.control_opcode
	// Measured max link-layer payload: ours 237 bytes vs a real Joy-Con's 138 -- we carry LARGER
	// packets than the real device. DLE is not a deficiency of ours and is not worth re-investigating.
	//
	// Why it may matter far beyond efficiency: three packets per report is roughly TRIPLE the air time,
	// and radio time is the one resource we share with the console's OTHER links. That is a mechanism for
	// the oldest unexplained clue in this project -- a registered puck dragging a real Joy-Con down with
	// it -- which no change to our payload could ever have addressed.
	//
	// NULL requests the maximum the SoftDevice allows, matching the real device's 251-octet negotiation.
	{
		// EXPLICIT params, not NULL. NULL asks for the SoftDevice's PREFERRED values, which are still the
		// 27-octet default -- so it returned ok=1 and put nothing on air (verified: LL_LENGTH_REQ still
		// zero occurrences after the first attempt). Naming 251 gives it something to actually negotiate,
		// matching the real controller's own LL_LENGTH_REQ.
		// Times left 0 = let the SoftDevice derive them from the octet counts and the active PHY.
		// 138, NOT 251 -- match the real device's negotiated length (2026-08-14, user's matrix
		// observation). The note above measured "ours 237 bytes vs a real Joy-Con's 138" and concluded
		// DLE was fine because we were BIGGER. That is backwards. max_tx_octets sizes the AIRTIME SLOT
		// the console must reserve for our connection event, and we never use it: our largest
		// transmission is a 63-byte report (80 for a command reply).
		//
		// It is the one mechanism that fits the configuration matrix, which no change to our payload
		// could ever explain:
		//     puck alone            -> drop
		//     puck + 1 Joy-Con      -> BOTH drop      <- a REAL Joy-Con dies too
		//     puck + 2 Joy-Cons     -> survive
		// A real controller being dropped alongside us cannot be about what our reports contain; it can
		// be about us taking radio time that link needed. The comment above even names this as "the
		// oldest unexplained clue in this project" before dismissing it.
		ble_gap_data_length_params_t dlp = { 0 };
		dlp.max_tx_octets = 138;
		dlp.max_rx_octets = 138;
		dlp.max_tx_time_us = 0;
		dlp.max_rx_time_us = 0;
		ble_gap_data_length_limitation_t dll = { 0 };
		bool dlOk = conn && conn->requestDataLengthUpdate(&dlp, &dll);
		Serial.printf("# LEAN data-length update requested (251): ok=%d limits tx=%u rx=%u payload=%u\n",
			      dlOk, dll.tx_payload_limited_octets, dll.rx_payload_limited_octets,
			      dll.tx_rx_time_limited_us);
	}
	forceEnableReportCccd();
	// Start streaming immediately on a RECONNECT, rather than waiting for the vibration command.
	//
	// The stream trigger used to be "first CMD_VIBRATION seen", which is part of the full phase-6 init
	// the console runs when it first registers a controller. On a reconnect it skips most of that, so the
	// trigger never arrives and we sit connected and completely SILENT -- confirmed live: `streaming=0,
	// 0 sent` on a reconnected, registered controller. A registered controller that reports nothing is
	// indistinguishable from a dead one, and the console drops it, which is exactly the 0x08 supervision
	// timeouts we keep seeing after registration succeeds.
	//
	// Only when we already know this console (a genuine reconnect). On a first-time registration the
	// normal phase-6 path still starts the stream at the right moment.
	//
	// DEFERRED 2026-08-08 -- see the g_deferStreamToPhase6 branch below. The premise above ("on a
	// reconnect the console skips most of phase 6") was true only while the recognized path was dying on
	// the bd282 pointer. Now that it completes, the console runs the FULL handshake on a reconnect, and
	// streaming through it starves the command replies: every first reply failed (`ok=0`), the console
	// retried each one exactly 10s later, and the handshake took so long it hit the 2s supervision
	// timeout budget's patience and dropped at 0x08. A real controller is silent until told to stream.
	if (g_forceStream && g_haveBondedHost && !g_deferStreamToPhase6) {
		g_streaming = true;
		Serial.println("# LEAN reconnect to a known console -- streaming immediately");
	}
	// Arm the wake-mode escape hatch. Wake advertising says "I belong to console X", which is right while
	// that console still has us registered -- but if it ever FORGETS us (it drops us, and the controller
	// slot goes away), we are left inviting a console that is no longer listening for us AND we are no
	// longer pairable, so the Change Grip/Order screen cannot see us either. That is a dead end needing a
	// manual 'C', and this session hit it twice. If a reconnect produces no command traffic at all within
	// a few seconds, the console does not consider us registered -- so drop back to pairing mode and let
	// it re-register us. Cleared as soon as any command arrives (see chrHandshakeCb).
	g_connStartMs = millis();

	// 2M, NOT 1M -- corrected 2026-08-08 by comparing the LINK LAYER of the real registration capture
	// against ours, a dimension never examined before (we had compared ATT/GATT to exhaustion).
	//
	//   real Joy-Con:  PHY_UPDATE_IND -> 2M, and it stays there for the whole session
	//   ours:          PHY_UPDATE_IND -> 2M ... then PHY_UPDATE_IND -> 1M, because of this call
	//
	// The console negotiates a 2M link with us exactly as it does with real hardware, and we were
	// immediately dragging it back down. The 1M request dates from 2026-08-04, when connections were
	// dying during PHY negotiation in a firmware that has since been almost entirely rewritten; it is a
	// workaround that outlived its problem and was never revisited. Match the real device instead.
	// 'P' toggles 2M vs 1M at runtime. 2M matches the real device, but 2M also has roughly HALF the range
	// of 1M -- and a real Joy-Con sits inches from the console while our puck is tethered to a PC across
	// the room. Now that registration succeeds, connections live long enough for link quality to matter,
	// and we are seeing supervision timeouts (0x08) rather than rejections. Being able to A/B this without
	// a reflash matters because each reflash also resets runtime state (that already cost us once).
	bool phyOk = conn->requestPHY(g_phy2M ? BLE_GAP_PHY_2MBPS : BLE_GAP_PHY_1MBPS);
	logEvent(5, phyOk);
	Serial.printf("# LEAN requestPHY(%s) ok=%d t=%lu ms\n", g_phy2M ? "2M" : "1M", phyOk,
		      (unsigned long)millis());
	// NOTE: sendWakeSequence() is deliberately NOT called here anymore. It replays what a real, ALREADY-
	// BONDED Joy-Con 2 does as ATT CLIENT toward the Switch's own GATT server during a Home-button wake --
	// completely wrong for a fresh/unbonded connection, where the Switch expects US to sit passively as a
	// GATT SERVER and wait to be read. Auto-firing it on every connect (as this did previously) meant every
	// single connection attempt -- fresh pairing included -- immediately sent unsolicited client-side writes
	// to hardcoded handles on the Switch's side, which is almost certainly why bonding never progressed:
	// no real unbonded controller behaves this way, and it likely trips the Switch's security state machine
	// into bailing right around the same ~0.6-1.5s window where disconnects were observed. Trigger it
	// manually instead via the 'W' console command once a connection is confirmed up, so fresh-pairing
	// (passive GATT server) behavior can be tested in isolation first.
}

static void bleDisconnectCb(uint16_t conn_hdl, uint8_t reason)
{
	logEvent(2, reason);
	Serial.printf("# LEAN DISCONNECT reason=0x%02X t=%lu ms\n", reason,
		      (unsigned long)millis());
	if (conn_hdl == g_connHdl)
		g_connHdl = BLE_CONN_HANDLE_INVALID;
	g_handshakePendingReply = false; // don't let 'A' answer a stale request from a dead connection
	// The stream is per-connection: the Switch re-writes the 679d descriptor after every reconnect, and
	// the forced CCCD value doesn't survive either. Leaving this set would notify into a dead link.
	// Report the stream's actual outcome for THIS connection. Previously the "flowing"/"auto-pressing"
	// messages were one-shot statics that fired once per BOOT -- so after the first connection they never
	// appeared again, and their absence looked exactly like "the stream never ran". That cost several
	// runs of chasing a bug that was not there.
	// QUEUED vs ACKED-on-air. "sent" was always a lie: it counts hvx() returning NRF_SUCCESS, which only
	// means the SoftDevice accepted the notification. HVN_TX_COMPLETE fires on PEER ACKNOWLEDGMENT, so
	// the third number is what actually reached the console.
	Serial.printf("# LEAN reports this connection: %lu QUEUED, %lu failed, %lu ACKED-on-air",
		      (unsigned long)g_reportsSent, (unsigned long)g_reportsFailed,
		      (unsigned long)g_txCompleteCount);
	Serial.println();
	// CONNECTION INTERVAL at death. The user's configuration matrix (2026-08-09) shows we die at 1-2 BLE
	// links and survive at 3 -- and a console with fewer links services each one far more aggressively.
	// The console gives 15ms on a fresh pairing but 5ms on reconnect, so the interval we actually get may
	// depend on how busy its radio is. Units are 1.25ms.
	Serial.printf("# LEAN conn interval at disconnect: %u (x1.25ms = %u.%02u ms)\n", g_connIntervalUnits,
		      (unsigned)(g_connIntervalUnits * 125 / 100), (unsigned)((g_connIntervalUnits * 125) % 100));
	// The 0x08 post-mortem. Supervision timeout is 2000ms (confirmed in every CONNECT_IND, ours and a real
	// Joy-Con's alike), so if the last acknowledgment lands ~2s before the disconnect the console stopped
	// listening first and our link layer merely noticed. If acknowledgments continue right up to the end,
	// the silence was ours.
	{
		unsigned long now = millis();
		Serial.printf("# LEAN 0x08 post-mortem: last ACK of our notify %lu ms ago, last write FROM "
			      "console %lu ms ago (supervision timeout is 2000 ms)\n",
			      g_lastTxCompleteMs ? (unsigned long)(now - g_lastTxCompleteMs) : 0UL,
			      g_lastRxFromConsoleMs ? (unsigned long)(now - g_lastRxFromConsoleMs) : 0UL);
	}
	g_lastTxCompleteMs = 0;
	g_lastRxFromConsoleMs = 0;
	g_askedForStream = false; // re-arm the unprompted 0x01/0x0C for the next connection
	// Restart the wake-mode fallback clock. It measures time spent advertising with nobody answering, so
	// it has to begin when the link ends -- otherwise a long, healthy, command-free connection counts
	// toward "this console has forgotten us" and we delete a registration that plainly works.
	g_reportsSent = 0;
	g_reportsFailed = 0;
	// Report service quality, then reset. g_connIntervalUnits must be cleared too, or the next
	// connection's interval is only inferable from a line NOT printing -- which is how the last
	// measurement had to be read, and inference is what keeps going wrong here.
	{
		unsigned long dur = millis() - g_connStartMs;
		Serial.printf("# LEAN service: %lu acks in %lu ms (%lu/s), longest ack gap %lu ms\n",
			      (unsigned long)g_txCompleteCount, dur,
			      dur ? (unsigned long)(g_txCompleteCount * 1000UL / dur) : 0UL, g_ackGapMaxMs);
	}
	g_ackGapMaxMs = 0;
	g_connIntervalUnits = 0;
	g_txCompleteCount = 0;
	g_streaming = false;
	g_homeHeld = false;
	g_hvnOutstanding = 0;
	// Both of these used to survive a disconnect and produce confident but WRONG logs on the next
	// connection: the wake queue retried forever against an invalid handle (20x/sec for the rest of
	// the boot), and a link reaching PAIR/LTK2 without a preceding LTK1 would answer the challenge
	// with the PREVIOUS connection's key and report "challenge answered" as if it were valid.
	g_wakeQueueActive = false;
	g_haveLinkKey = false;
	memset(g_linkKeyMsb, 0, sizeof g_linkKeyMsb);
	// FINISH verifies against whichever byte order g_ltkReverse selects, so clearing only one of
	// these still let a stale key report matches=1 on a connection that never ran LTK1.
	memset(g_linkKeyWire, 0, sizeof g_linkKeyWire);
}

// ATT READ LOGGER. Two hard constraints, either of which breaks the link if got wrong:
//   1. Bluefruit invokes this and DOES NOT REPLY (its sd_ble_gatts_rw_authorize_reply calls are in
//      the WRITE path only -- verified in BLECharacteristic.cpp:418-440). If we do not reply here,
//      the read never completes and the connection dies on the ATT timeout.
//   2. Registered with useAdaCallback = false so this runs in BLE context. The deferred path replies
//      from another task, which is exactly the latency an ATT transaction cannot absorb.
// update = 0 means "serve the stored attribute value" -- we are observing, not substituting.
static void readAuthCb(uint16_t conn_hdl, BLECharacteristic *chr, ble_gatts_evt_read_t *request)
{
	const char *name = (chr == &chrInput)	 ? "INPUT_REPORT(fd2)" :
			    (chr == &chrUnkD5a9) ? "*** REPORT CHANNEL d5a9 ***" :
			    (chr == &chrUnkFde)  ? "chrUnkFde" :
			    (chr == &chrS1R1)	 ? "bd281" :
			    (chr == &chrS1R2)	 ? "bd283" :
						   "?";
	Serial.printf("# LEAN ATT READ: %s handle=0x%04X offset=%u t=%lu ms\n", name, request->handle,
		      request->offset, (unsigned long)millis());

	ble_gatts_rw_authorize_reply_params_t reply;
	memset(&reply, 0, sizeof reply);
	reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
	reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
	reply.params.read.update = 0; // serve the stored value unchanged
	uint32_t err = sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);
	if (err != NRF_SUCCESS)
		Serial.printf("# LEAN ATT READ reply FAILED err=0x%08lX -- link will drop\n",
			      (unsigned long)err);
}
static void cccdCb(uint16_t conn_hdl, BLECharacteristic *chr, uint16_t cccd_value)
{
	(void)conn_hdl;
	uint8_t which = (chr == &chrInput) ? 3 : (chr == &chrCmdResp) ? 4 : 0;
	logEvent(which, cccd_value);
	// chrUnkD5a9 is the REPORT CHANNEL -- the one that matters. It had no callback registered at all
	// until 2026-08-12, so a subscription would have gone unlogged; without this arm it would log "?".
	const char *name = (chr == &chrInput)	      ? "INPUT_REPORT" :
			    (chr == &chrCmdResp)      ? "COMMAND_RESPONSE" :
			    (chr == &chrUnk640c)      ? "chrUnk640c" :
			    (chr == &chrUnkD3bd)      ? "chrUnkD3bd" :
			    (chr == &chrUnkD5a9)      ? "*** REPORT CHANNEL d5a9 ***" :
			    (chr == &chrUnkFde)	      ? "chrUnkFde" :
			    (chr == &chrInput)	      ? "INPUT_REPORT(fd2)" :
							"?";
	Serial.printf("# LEAN CCCD write: %s value=0x%04X t=%lu ms\n", name, cccd_value,
		      (unsigned long)millis());
}

// 63-byte input report on d5a9e01e -- the real report channel (see setupGatt). Built from a real captured
// idle report rather than zeros, with only the timestamp refreshed and the button field patched, so every
// field whose resting value we don't understand keeps the value a real controller actually sends.
static void sendInputReport(bool homePressed)
{
	if (g_connHdl == BLE_CONN_HANDLE_INVALID)
		return;
	uint8_t report[63];
	memcpy(report, g_ownTemplates ? INPUT_REPORT_TEMPLATE_OWN : INPUT_REPORT_TEMPLATE, sizeof report);
	// [0:2] is a REPORT SEQUENCE COUNTER, not a millisecond timestamp. Measured on 678 consecutive real
	// reports (docs/captures/jc_buttons.py log): 674 of the deltas are exactly 1, the rest are dropped
	// packets. We were sending millis(), which leaps 15-40 per report -- to anything tracking sequence
	// continuity that reads as massive packet loss.
	//
	// It also used to be written as 32 bits over [0:4], which clobbered the BUTTON bytes at [2:4]:
	// byte[2] took (millis>>16), and byte[2] bit0 is B. So from 65.5s of uptime on, every report claimed
	// B was held down -- and B is CANCEL on the Change Grip/Order screen. We were asking to register and
	// backing out in the same packet. The template rests [2:4] at 0x00/0x00 because nothing is pressed.
	// [0] is an 8-BIT counter. [1] is NOT its high byte -- it is a constant, and the template's 0x18 is
	// now left alone (review 2026-08-14 #2).
	//
	// The tree held two measurements that contradict each other: "[0:2] is a 16-bit sequence counter,
	// 674 of 678 deltas exactly 1" and find_fields.js's "[1] CONSTANT 0x18". A 16-bit LE counter cannot
	// keep its high byte constant across 658 increments, so the second wins -- and both observations are
	// satisfied by an 8-bit counter. VERIFIED against the decrypted capture: report[1] = 0x18 in ALL 283
	// real reports. INPUT_REPORT_TEMPLATE[1] is 0x18 independently.
	//
	// The old write put a rolling byte there, wrong on ~255 of every 256 reports.
	report[0] = (uint8_t)(g_reportSeq & 0xFF);
	// Replay real motion/stick data over [4:56]. A frozen sample means a controller whose IMU is
	// perfectly dead, which is what we streamed before -- see motion_replay.h for the measurement.
	// BISECT STEP 1 ('/'). The console reports "out of battery" and the icon FLAPS -- appears and
	// disappears in a loop. A flap means the field it reads CHANGES every frame, and the only thing that
	// changes every frame is this replay, which covers [4:56]. Freezing it sends the captured template
	// body verbatim on every report (only the sequence counter moves). If the flap stops, the offending
	// field is inside [4:56] and we subdivide from here. If it does not, the console is not reading the
	// replayed body at all and the battery figure comes from somewhere else entirely.
	// BISECT ('/' cycles). Replay is applied ONLY to [g_replayLo, g_replayHi) of the report; everything
	// else keeps the captured template's bytes. Frozen body -> "low battery"; full replay -> "out of
	// battery", so the field the console reads is inside the replayed span and the replay drives it to
	// empty. Narrow the span until the message flips, and that isolates the bytes.
	// REPLAY ONLY THE IMU, [16:56]. MOTION_OFF=4/MOTION_LEN=52 covered [4:56] -- twelve bytes more than
	// the IMU block this replay is documented to be ("[16:56] IMU, replayed from MOTION_REPLAY"). It was
	// overwriting the constant 0x07 at [4], the packed stick at [5:8], the battery/mode byte at [8] (which
	// is why that had to be re-written afterwards), and [9:16].
	//
	// Measured 2026-08-14 by dumping our first post-arm reports and diffing them against the real
	// device's first post-trigger report (joycon2_reconnect_DECRYPTED.txt frame 1942):
	//     real  ... 38 ff ff 00 00 5f 00 28 ...
	//     ours  ... 88 00 00 00 00 FF 00 28 ...
	//                  ^^^^^       ^^
	// [9][10] and [13] were wrong in EVERY report and constant, so not IMU noise -- the template holds the
	// right values and the replay was overwriting them. The console takes ~2 reports and hangs up
	// deliberately (reason=0x13) within ~35-78 ms of subscribing, i.e. it is rejecting report CONTENT.
	// REVERTED to the full [4:56] replay. I narrowed this to [16:56] after diffing our first post-arm
	// report against the real device's FIRST post-trigger report (frame 1942) and finding [9][10] and [13]
	// different -- concluding the replay was corrupting documented non-IMU bytes.
	//
	// That conclusion was WRONG. The real device's LATER frames (2506-2514) carry
	//     ... 38 00 00 00 00 ff 00 28 ...
	// exactly the values the replay was producing, while frame 1942 carries ff ff / 5f. **Those fields
	// are DYNAMIC on real hardware.** Freezing them to the template's first-frame values made us less
	// faithful, not more. The replay is real captured data ([4:56] of 256 consecutive frames off a
	// genuine Joy-Con 2 Right) and replaying all of it is the closest we get to a device that works.
	// [8] is re-written afterwards because that one field is state we must control.
	if (g_replayHi > g_replayLo) {
		uint8_t lo = g_replayLo < MOTION_OFF ? MOTION_OFF : g_replayLo;
		uint8_t hi = g_replayHi > MOTION_OFF + MOTION_LEN ? MOTION_OFF + MOTION_LEN : g_replayHi;
		if (hi > lo)
			memcpy(report + lo, MOTION_REPLAY[g_motionFrame] + (lo - MOTION_OFF), hi - lo);
	}
	// BATTERY / REPORT-MODE BYTE. [8] = (battery << 4) | mode, measured 2026-08-08.
	//
	// This MUST be written after the motion memcpy: MOTION_OFF=4 / MOTION_LEN=52 covers [4:56], so the
	// replay was silently overwriting [8] on every single frame with whatever that byte happened to be
	// in the captured motion data. On 2026-08-13, the FIRST time the console ever consumed our stream
	// (it armed d5a9, subscribed the CCCD, and read our reports) it immediately reported the controller
	// as **out of battery** and hung up with reason=0x13 after ~2 s. That is the console acting on this
	// byte -- and it is the first time in this project that it has acted on our report content at all.
	//
	// Low nibble 8 = full report mode (measured: it flipped 0x0 -> 0x8 exactly when CMD_FEATURE enabled
	// full reporting). High nibble is the charge level; the captured real device sent 3 the whole time.
	// Cycled at runtime with ']' so the encoding can be swept without a reflash.
	report[8] = g_batteryByte;
	// BATTERY COHERENCE (user's catch, 2026-08-14). [8] is a COARSE STATE -- (level << 1) | charging --
	// but the report also carries an actual cell VOLTAGE at [18:20], u16 little-endian millivolts. On the
	// real device the two agree: level 1 + charging alongside a constant 3584 mV, i.e. a half-flat pack on
	// charge. We were overriding [8] to level 4 (FULL) while [18:20] still carried the REPLAY's 3843 mV --
	// a mid-charge voltage. Claiming "full" next to 3.84 V is a contradiction no real pack produces, and a
	// console that cross-checks them would reject the pair and end up with no reading at all, which is
	// exactly what an empty/animating battery icon looks like.
	//
	// [18:20] sits inside the replayed span, so like [8] it must be written AFTER the memcpy.
	// 4200 mV is a full Li-ion cell, coherent with level 4.
	// *** [18:20] IS NOT A VOLTAGE. IT IS A PACKED IMU SAMPLE TIMESTAMP. *** (review 2026-08-14 #1)
	//
	//   ts12    = ((report[17] & 0x0F) << 8) |  report[16]        sample timestamp, wraps at 4096
	//   delta12 = ((report[18] & 0x0F) << 4) | (report[17] >> 4)  ticks since the previous sample
	//
	// VERIFIED over all 256 frames of motion_replay.h: ts12[i] - ts12[i-1] == delta12[i] (mod 4096) on
	// 255/255 consecutive pairs, zero mismatches, deltas clustered 46-49 with exactly two at 96 -- the
	// two dropped packets in that capture, at double the interval. A decoded structure, not a fit.
	//
	// I added a "battery voltage" write here an hour before this review and shipped it ON by default. It
	// forced delta12 to 128/143 alternating while ts12 kept advancing 48, so EVERY report asserted a
	// self-contradictory IMU clock -- and that is the "field that changes every frame" flap signature.
	// It is also the one field we corrupt that the console can validate with NO knowledge of our
	// hardware, purely by checking the report against itself.
	//
	// Why 3843 mV "looked like ground truth": 3843 = 0x0F03 -> report[18]=0x03, report[19]=0x0F, which
	// are bytes real hardware genuinely emits. Scanning for "a plausible cell voltage" found a timestamp
	// low nibble sitting next to a constant. The scan was the mistake, not the data.
	//
	// Left in place, OFF, purely so the mistake is documented at the site. Do not re-enable.
	if (g_batteryMvOn) {
		report[18] = (uint8_t)(g_batteryMv & 0xFF);
		report[19] = (uint8_t)(g_batteryMv >> 8);
	}
	// SYNTHESISE THE IMU CLOCK instead of replaying it, so it never goes backwards.
	//
	// MOTION_REPLAY is 256 frames on a loop. Replaying the timestamp verbatim means that at the 255->0
	// seam ts12 steps FORWARD BY 32 while delta12 claims 48 -- a 16-tick shortfall, not a backwards
	// jump as an earlier version of this comment said (ts[255]=0x00f -> ts[0]=0x02f). One self-contradictory report
	// every 256 frames. At ~33 reports/s that is one bad report every ~7.7 s, and "drops after a few
	// seconds once off the Change Grip/Order screen" is that interval. The pairing screen does not
	// validate the stream; a live controller's does.
	//
	// So keep the replay's motion DATA and generate the clock ourselves: ts12 advances by a fixed 48
	// ticks per report and delta12 says 48, which satisfies the invariant on every report including the
	// seam. 48 is the modal step in the real capture (173 of 255 pairs; the rest are 46/47/49 jitter and
	// two 96s where packets dropped).
	//
	//   ts12    = ((report[17] & 0x0F) << 8) |  report[16]
	//   delta12 = ((report[18] & 0x0F) << 4) | (report[17] >> 4)
	//
	// report[18]'s high nibble is 0 on real hardware and its low nibble is delta>>4 -- the three values
	// the capture ever shows, 0x02/0x03/0x06, are exactly 47>>4, 48>>4 and 96>>4.
	// COMPUTED here, COMMITTED only on a successful hvx -- see the NRF_SUCCESS block below.
	//
	// It first advanced g_imuTs12 right here, which was a REGRESSION (review 2026-08-14b #1): this runs
	// BEFORE the 8 ms command-holdoff and the flow-control early-out, while g_reportSeq/g_motionFrame
	// advance only on transmission. Worse, the slot-retry fix in the same commit left lastReportMs
	// unstamped on a skip, so the heartbeat re-entered at loop() rate for the whole holdoff -- each pass
	// bumping the clock by 48 without sending. The report that finally went out carried a timestamp
	// jumped by 48 x (spin count) while delta12 still claimed 48: the exact self-contradiction finding #1
	// was about, now triggered by EVERY console command instead of once per 7.7 s replay seam. ts12 is
	// 12 bits, so ~86 spin passes randomise it outright.
	const uint16_t tsStep = 48;
	const uint16_t tsNext = (uint16_t)((g_imuTs12 + tsStep) & 0x0FFF);
	report[16] = (uint8_t)(tsNext & 0xFF);
	report[17] = (uint8_t)(((tsNext >> 8) & 0x0F) | ((tsStep & 0x0F) << 4));
	report[18] = (uint8_t)((tsStep >> 4) & 0x0F);
	// BATTERY VOLTAGE / CURRENT, millivolt-shaped. 2026-08-13: with [8] set to 0x88 (full) the console
	// STILL reported "out of battery", and the icon FLAPPED -- appearing and disappearing in a loop.
	// A static wrong value gives a steady wrong reading; a flapping one means the field it reads is
	// CHANGING every frame. The motion replay covers [4:56], so anything battery-shaped in that span
	// gets fresh IMU noise 30x a second. That is the signature of the console reading a numeric battery
	// field out of the replayed body.
	//
	// trevlars/switch2-controllers-linux and CareyScott/switch2controllerpc both decode
	// battery_voltage = u16le([31:33]) mV and battery_current = u16le([33:35]) on the INPUT_REPORT
	// channel. Our own earlier measurement found fd2[0x1F:0x21] = 3389 mV, a sane cell voltage at
	// exactly that offset. Those offsets were dismissed for d5a9 because "the layouts are different" --
	// but that judgement was made when the console had never read a single report from us, so it was
	// never actually tested. Hold them steady at a healthy value and see.
	// WARNING (review 2026-08-14b #5): BATT_PROBE_OFF includes 15, 17 and 19, so a probe step writes over
	// report[16]/[17]/[18] -- the packed IMU sample clock. This tool is now strictly DESTRUCTIVE, not
	// "safe by construction" as the note below claims. Default-off, so latent. Drop those three offsets
	// before using it again -- and note there is no voltage field to hunt for anyway: [8]'s high nibble
	// is the entire battery signal.
	//
	// BATTERY FIELD HUNT ('B' steps the candidate). SAFE by construction: the body is FROZEN to a real
	// captured frame (coherent, self-consistent, static) and exactly ONE u16 slot is raised. Nothing
	// changes frame to frame, which is what crashed the console when spans were spliced.
	//
	// Raised to FF 0F: byte[o] = 0xFF reads high for any byte-scaled field, and u16le = 0x0FFF = 4095
	// reads high for a 12-bit or millivolt-scaled one, so each step probes both encodings at once.
	// Known-eliminated and therefore absent from the list: [8] (nibble, 0x88 and 0x38 both tried) and
	// [31:33]/[33:35] (millivolts, 4000 tried).
	if (g_battProbe > 0 && g_battProbe <= BATT_PROBE_N) {
		uint8_t o = BATT_PROBE_OFF[g_battProbe - 1];
		report[o] = 0xFF;
		report[o + 1] = 0x0F;
	}
	// (The old override here wrote the voltage at [31:35], taken from the reference repos' fd2 layout.
	// That is the WRONG OFFSET for d5a9 -- it corrupted real IMU bytes and never moved the reading. The
	// voltage is at [18:20], written above with the battery level so the two stay coherent. Scanning the
	// real device's reports for a plausible cell voltage finds [18:20] = 3584 mV, constant across frames,
	// while every other in-range hit varies frame to frame and is IMU noise.)
	// PERIODIC REAL STICK INPUT (user's suggestion, 2026-08-09). Everything we stream is a replay in which
	// the stick never leaves center and no button is ever pressed -- so from the console's side we are a
	// controller nobody has touched since it was switched on. This deflects the stick fully DOWN for 400ms
	// every 3s so our reports carry an actual, deliberate input.
	//
	// [5:8] is the stick, 12-bit packed (measured: it is the ONLY triple in the report that rests
	// near-center AND still). Deflecting it is the smallest change that makes us look used rather than
	// inert, and it is deliberately periodic so it can be correlated against the drop timing.
	// FOLLOWS g_buttonMask, so "NONE -- press nothing (quiet controller)" now means exactly that.
	// It did not before: the mask silenced the buttons while this kept slamming the stick fully down for
	// 400 ms every 3 s, so the "quiet controller" A/B was testing a controller that still sent deliberate
	// input. The user spotted it -- "it is still sending the down command on the stick".
	//
	// This matters for the acceptance test as they framed it: once paired, a controller should reconnect
	// on its own on ANY screen without pressing anything. A puck that keeps injecting stick input is not
	// that controller, and cannot demonstrate it.
	if (g_buttonMask) {
		unsigned long ph = millis() % 3000;
		if (ph < 400) {
			// INSIDE our own declared calibration (review 2026-08-14b #3). MEM_00013080[40..48] is
			// the factory stick calibration we hand the console at registration -- three packed
			// 12-bit XY triples (center, max delta, min delta):
			//     center 2048,2048 | max 1600,1600 | min 1600,1600
			// declaring an operating range of 448..3648 on both axes.
			//
			// This used to send y=200: BELOW the floor we told the console this stick can
			// physically reach. On real hardware that reading is impossible -- it is outside the
			// calibrated travel of the part. Only a console CONSUMING the stick has reason to
			// range-check it, which is why the pairing screen tolerates it and a live controller
			// driving the Home menu might not. 900 is a genuine full-down deflection with margin.
			//
			// x sits at the declared center, 2048. The replay's own resting value is (2123, 2013),
			// well inside the declared travel either way.
			packStickXY(report + 5, 2048, 900); // x at the declared center, y full down in range
			static unsigned long lastLog = 0;
			if (millis() - lastLog > 2500) {
				lastLog = millis();
				Serial.println("# LEAN stick: sending DOWN");
			}
		}
	}
	// BUTTON FIELD -- nothing patched here, deliberately.
	// RETRACTED (audit BUG-11): an earlier version of this comment argued the console reads buttons at
	// [4:8] and concluded we had been "mashing a random handful of buttons ~60 times a second". That was
	// wrong and is not evidence for anything. What follows is the measurement that replaced it.
	// I briefly zeroed [4:8] as a "Pro Controller button field" and wrote a
	// battery value at [0x1F:0x21], taking both offsets from trevlars/switch2-controllers-linux. BOTH
	// WERE WRONG, and measuring our own captured Joy-Con d5a9 frames proved it (scratchpad/
	// find_fields.js, 658 real frames):
	//
	//   [0]  256 distinct 0-255  -> sequence counter low byte
	//   [1]  CONSTANT 0x18       -> counter high byte
	//   [2]  2 distinct (0,1)    -> BUTTONS (B)            <- matches our direct measurement
	//   [3]  4 distinct (0,128)  -> BUTTONS (SL/SR/HOME)   <- matches our direct measurement
	//   [4]  CONSTANT 0x07 (meaning unknown), [5:8] STICK (12-bit packed) -- NOT buttons
	//   [16..] high churn        -> IMU
	//
	// and NO 16-bit field anywhere in the report sits in the 3400-4200mV range, so battery is not in it
	// either. That repo parses the ...fd2 characteristic; the console streams from d5a9e01e, and the two
	// layouts are simply different. Importing fd2 offsets here meant zeroing stick data and overwriting
	// IMU samples with a battery reading. Leave the replayed payload alone -- it is real d5a9 data from a
	// real controller, which is the best-founded thing in this whole report.
	if (homePressed && g_buttonOffset + 4 <= sizeof report) {
		// OR the bit in rather than overwriting the field -- the template's resting value there is
		// non-zero and its meaning is unknown, so clobbering it would change more than the button.
		uint32_t v = (uint32_t)report[g_buttonOffset] |
			     ((uint32_t)report[g_buttonOffset + 1] << 8) |
			     ((uint32_t)report[g_buttonOffset + 2] << 16) |
			     ((uint32_t)report[g_buttonOffset + 3] << 24);
		// HAZARD (audit RISK-4): this is a 32-bit OR at [2], so it spans [2:6] -- but only [2:4] is the
		// button field in the layout we measured. [4] is the constant 0x07 and [5] is the top of the
		// packed stick. Any mask bit above 0x0000FFFF therefore corrupts stick data. BTN_HOME (0x00000100)
		// is safely inside [2:4]; nothing currently sets a high bit, so this is latent, not active.
		v |= g_buttonMask;
		report[g_buttonOffset + 0] = (uint8_t)(v & 0xFF);
		report[g_buttonOffset + 1] = (uint8_t)((v >> 8) & 0xFF);
		report[g_buttonOffset + 2] = (uint8_t)((v >> 16) & 0xFF);
		report[g_buttonOffset + 3] = (uint8_t)((v >> 24) & 0xFF);
	}
	// Bypass Bluefruit's notify() wrapper and drive the SoftDevice directly, so we see the REAL error.
	// The console never subscribes this characteristic's CCCD -- the real controller streams anyway
	// because Nintendo's stack does not gate on it, and sd_ble_gatts_value_set() refuses to fake the
	// CCCD (NRF_ERROR_FORBIDDEN, 0x0F). This tells us whether the remaining block is Bluefruit's own
	// notifyEnabled() check or the SoftDevice's.
	uint16_t len = sizeof report;
	ble_gatts_hvx_params_t hvx = {};
	// ...d5a9e01e, NOT ...fd2. MEASURED 2026-08-13: the console writes the 0x0085 stream trigger to
	// d5a9's 679d descriptor and then SUBSCRIBES d5a9's CCCD -- at 0x002E/0x002D, nowhere near the real
	// device's 0x000E, so it finds the characteristic BY UUID and the handle it sits on is irrelevant.
	// We had been streaming into fd2, which the console never subscribes and never listens to: 486
	// reports queued, 0 failed, 502 acked on air, all wasted. The reason this took so long to see is
	// that d5a9's 679d descriptor was created with addDescriptor()'s DEFAULT write_perm of
	// SECMODE_NO_ACCESS, so every trigger write was rejected by the SoftDevice with ATT 0x03 and raised
	// no application event -- the trigger has been arriving, and being silently bounced, all along.
	hvx.handle = chrUnkD5a9.handles().value_handle;
	hvx.type = BLE_GATT_HVX_NOTIFICATION;
	hvx.offset = 0;
	hvx.p_len = &len;
	hvx.p_data = report;
	// Yield the queue to command traffic. A command reply that fails is fatal to the handshake -- the
	// Switch just retries and stalls -- whereas a dropped input report is invisible at 66 reports/sec. So
	// skip a report if a reply went out very recently rather than competing with it.
	// Was a 40ms hold-off after every command reply, to stop reports starving the command channel. That
	// predates the g_hvnOutstanding flow control below, which solves the same problem properly -- and it
	// backfired badly: the console sends commands ~43ms apart in bursts, so reports were skipped almost
	// continuously and the stream never actually produced a packet. 8ms is enough to yield a slot.
	if (millis() - g_lastCmdReplyMs < 8)
		return;
	// Rate matters now that we are a REGISTERED controller. SUPERSEDED 2026-08-09: this block used to
	// argue for ~66/sec and treat our observed ~36/sec as a defect to chase. The real device does 33/sec
	// (30ms, measured directly -- see the timer above), so ~36/sec was already right and the "half rate
	// looks like a failing device" reasoning was chasing a target that never existed.
	// Flow control. We were pushing a report every 15ms regardless of whether the radio had sent the
	// previous one; the link carries roughly one packet per connection interval, so the queue filled
	// permanently (NRF_ERROR_RESOURCES) and command replies -- which actually matter -- were dropped.
	// Only queue a report when the SoftDevice has drained what we already gave it.
	// BACK TO 2. Raising this to 6 to chase the real device's ~66/sec was a mistake: measured live, it
	// produced "3115 sent, 5111 FAILED" -- more rejected hvx calls than successful ones, because actual
	// throughput is capped by the connection interval, not by how many we are willing to queue. All the
	// extra attempts did was hammer a full queue.
	// The "at 2 the same firmware reports 0 failed consistently" this comment used to claim was measured
	// on connections that carried no command traffic while streaming. With commands in flight the counter
	// drifts to 0 and this gate stops gating at all -- see the NRF_ERROR_RESOURCES clamp below, which is
	// what actually holds the line. If the rate genuinely needs to be higher, the lever is the connection
	// interval, not this gate.
	if (g_hvnOutstanding >= 2)
		return;
	uint32_t err = sd_ble_gatts_hvx(g_connHdl, &hvx);
	if (err == NRF_SUCCESS) {
		g_reportSentThisSlot = true; // the caller spends the 30 ms slot only on a real transmission
		g_imuTs12 = tsNext;          // commit the IMU clock ONLY here, in step with the counters below
		g_hvnOutstanding++;
		g_reportsSent++;
		// Advance ONLY on a report that actually went out. Bumping these on a report the flow control
		// skipped would put gaps in the sequence -- the exact defect this counter is here to avoid.
		g_reportSeq++;
		g_motionFrame = (uint16_t)((g_motionFrame + 1) % MOTION_FRAMES);
	} else {
		// NRF_ERROR_RESOURCES means the queue really is full, and it is the ONLY trustworthy signal we
		// have -- g_hvnOutstanding provably drifts. BLE_GATTS_EVT_HVN_TX_COMPLETE decrements it for
		// every notification the SoftDevice completes, but only input reports ever increment it, so
		// each command reply knocks it down for free. Under command traffic it reaches 0 while reports
		// are still queued, the gate above opens, and we hammer a full queue: measured live at 1961 and
		// then 7601+ failures on connections that carried commands, while command-free connections
		// reported "0 failed" and made the gate look sound.
		//
		// Clamping to the gate value stops the hammering until a real TX_COMPLETE arrives, which
		// self-corrects the drift no matter what caused it. That matters beyond wasted calls: a full
		// queue is what made our REPLIES fail (`ok=0`), and an unanswered command leaves the console
		// retrying ~10s later.
		if (err == NRF_ERROR_RESOURCES)
			g_hvnOutstanding = 2;
		g_reportsFailed++;
	}
	// Show the head of what actually went out, ~1/sec. "The counter says N sent" does not tell us whether
	// the button bytes hold what we think -- and a wrong byte[2]/byte[3] is exactly the bug that hid here
	// for a week. Bytes 0-1 timestamp, 2-3 buttons.
	// FIRST REPORTS AFTER THE ARM, IN FULL. The console arms the stream, subscribes, takes ~2 reports and
	// hangs up deliberately (reason=0x13) within ~35-78 ms -- it is rejecting the report CONTENT, and the
	// stall is a consequence that arrives after the link is already dead. We have the real device's first
	// post-trigger reports in joycon2_reconnect_DECRYPTED.txt (frame 1942 onward), so dumping ours whole
	// makes it a byte-for-byte comparison instead of another guess.
	if (!g_quiet && err == NRF_SUCCESS && g_armedMs && g_firstReports < 3) {
		g_firstReports++;
		Serial.printf("# LEAN RPT#%u (%lu ms after arm): ", g_firstReports,
			      (unsigned long)(millis() - g_armedMs));
		for (uint16_t i = 0; i < sizeof report; i++)
			Serial.printf("%02X", report[i]);
		Serial.println();
	}
	static uint32_t lastDumpMs = 0;
	if (!g_quiet && err == NRF_SUCCESS && millis() - lastDumpMs >= 1000) {
		lastDumpMs = millis();
		Serial.printf("# LEAN report head: seq=%u | btn %02X %02X | imu %02X %02X %02X %02X (frame %u)\n",
			      (unsigned)g_reportSeq, report[2], report[3], report[20], report[21], report[22],
			      report[23], (unsigned)g_motionFrame);
	}
	// FAILURE LOGGING, PER CONNECTION (2026-08-13). The old rate limit was "every 100th failure since
	// boot", so by the time anything printed it was failure #5301 and told us nothing about the one that
	// mattered -- the FIRST failure after the console arms the stream, which is when it decides whether
	// to keep us. Measured discriminator between a held and a cycling connection: the held one had
	// 769 reports / 0 failures, the cycling ones 1 report / 10-14 failures. So the first few failures of
	// each connection are the whole question. Log the first 6 of every connection, with the error code
	// and how long after the arm it happened; g_failThisConn resets in bleConnectCb.
	//
	// Codes seen so far: 0x3002 BLE_ERROR_INVALID_CONN_HANDLE (link already gone),
	//                    0x3401 BLE_ERROR_GATTS_SYS_ATTR_MISSING (CCCD state not established --
	//                           suspicious, since forceEnableReportCccd() writes the sys-attr blob and
	//                           the console's own CCCD write can make Bluefruit reload it underneath us).
	static uint32_t failCount = 0;
	if (err != NRF_SUCCESS) {
		failCount++;
		if (!g_quiet && g_failThisConn < 6) {
			g_failThisConn++;
			Serial.printf("# LEAN *** hvx FAIL #%u this conn: err=0x%08lX (%s) %lu ms after arm, "
				      "streaming=%d ***\n",
				      g_failThisConn, (unsigned long)err,
				      err == 0x3401 ? "SYS_ATTR_MISSING" :
				      err == 0x3002 ? "INVALID_CONN_HANDLE" :
				      err == NRF_ERROR_RESOURCES ? "RESOURCES" : "?",
				      (unsigned long)(g_armedMs ? millis() - g_armedMs : 0), (int)g_streaming);
		}
	}
	else if (err == NRF_SUCCESS && failCount == 0) {
		static bool once = false;
		if (!once) { once = true; Serial.println("# LEAN *** INPUT REPORTS FLOWING ***"); }
	}
}

static bool s_bleInitDone = false;

static void bleInitOnce()
{
	markStage(STAGE_ENTERED);
	checkSoftDeviceImage();
	markStage(STAGE_SDCHECK_DONE);
	// Must precede Bluefruit.begin() -- without this, the SoftDevice is configured with the default
	// mtu_max (BLE_GATT_ATT_MTU_DEFAULT = 23), which silently clamps every MTU exchange to 23 regardless
	// of what's requested (peripheral connection config caps it via minof(requested, configured_max) in
	// BLEConnection's MTU exchange handler). Sniffer capture showed our peripheral replying to the
	// Switch's Exchange MTU Request with "Server Rx MTU: 23" -- the real Joy-Con 2 replies with 512.
	// Default GATT attribute table (BLE_GATTS_ATTR_TAB_SIZE_DEFAULT = 1408 bytes) isn't enough for the
	// full real GATT structure below (2 services, 13 characteristics, several with extra 128-bit-UUID
	// descriptors) -- chrCmdResp/chrVibration silently got handle 0 (begin() failed) before this was added.
	// RAISED to 8192 2026-08-07. Phase 6 grew the table (chrUnkD5a9's value 1 -> 63 bytes, chrUnk65a7's
	// max 33 -> 64) and the Switch immediately stopped getting past MTU exchange -- it would connect,
	// negotiate, then never discover or subscribe anything. Reverting to the pre-phase-6 build on the same
	// console minutes later restored CCCD subscribes, so the regression was ours, and an overflowing
	// attribute table is the known failure mode here: begin() fails SILENTLY on whichever characteristics
	// no longer fit, leaving handles at 0x0000 and a truncated service that a central bails on mid-
	// discovery. This bit the project once already (see the configAttrTableSize/configUuid128Count history
	// in the memory notes). 8192 is well clear; the nRF52840 has RAM to spare (18% used).
	Bluefruit.configAttrTableSize(8192);
	// Default vendor-UUID slot count (BLE_UUID_VS_COUNT_DEFAULT) is only 10 -- we now register ~17
	// distinct 128-bit UUIDs (service + 13 characteristics + 2 descriptor types), still not enough on its
	// own even with the bigger attr table above; this is the actual second resource limit being hit.
	Bluefruit.configUuid128Count(24);
	// Drop the Service Changed characteristic, and with it the whole Generic Attribute service, to free
	// the four handles it occupies (0x000A-0x000D) at the bottom of the attribute table.
	//
	// Frees four handles (0x000A-0x000D) at the bottom of the table, which the handle-alignment layout
	// in setupGatt() depends on.
	// CAVEAT (audit BUG-12): this was originally justified by "the console hardcodes handles", which is
	// DISPROVEN -- see setupGatt(). It also has a real cost: with Service Changed gone, a bonded console
	// cannot learn our attribute table moved and keeps a stale cached copy, which is why the BLE address
	// must be bumped after any layout change. Re-decide deliberately if handle placement stops mattering.
	// ENABLED 2026-08-12 to test a specific, measured divergence. The console issues ZERO ATT Read
	// Requests to a real Joy-Con on reconnect (its whole conversation is 12x Write Command + 1x Write
	// Request), but reads bd281 AND bd283 from US on every single reconnect. That is verification work
	// it does not need to do for a device it trusts.
	//
	// Service Changed is exactly the mechanism by which a server tells a bonded client "my attribute
	// table is stable, keep your cache". Without it a client may be unable to trust a cached table and
	// re-read to verify -- which would explain the re-reads, and ties in the stale-cache problem that
	// forced BLE address bumps after every layout change.
	//
	// PREDICTION: with this on, the bd281/bd283 reads should stop. If they do, the cache theory holds.
	// COST: it reclaims handles 0x000A-0x000D, shifting our layout up ~4 -- check printGattCheck() and
	// the report value / 679d descriptor placement afterwards. Set back to false if the layout matters
	// more than the reads.
	// TESTED 2026-08-12 and REVERTED: enabling it did NOT stop the bd281/bd283 re-reads (they still
	// fired, at the shifted handles 0x0034/0x0038). The GATT-cache theory is DISPROVEN. Reverted because
	// it cost the handle alignment -- report value 0x000E->0x0011, 679d descriptor 0x0010->0x0013 -- for
	// no measured benefit.
	Bluefruit.configServiceChanged(false);
	// HVN TX queue raised from the default (1) to 10, and the event length from 3 to 6 (x1.25ms). Once the
	// input-report stream came up (63 bytes every 15ms) it saturated the single-entry notification queue
	// and starved COMMAND_RESPONSE: every reply came back ok=0 and the Switch sat retrying the same
	// MEMORY read every 10s. Reports and command replies share this queue, so it has to hold more than one
	// packet, and the connection event needs room to actually drain it.
	Bluefruit.configPrphConn(512, 6, 10, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
	markStage(STAGE_CALLING_BEGIN);
	bool ok = Bluefruit.begin(1, 0);
	markStage(STAGE_BEGIN_RETURNED, (uint8_t)(ok ? 1 : 0));
	if (!ok)
		return;
	Bluefruit.setEventCallback(rawBleEventCb);

	// Make sure Bluefruit's bond directory exists. bond_init() (bluefruit.cpp) is supposed to create it
	// during begin(), but a live check showed InternalFS.exists("/adafruit/bond_prph") == 0 and every
	// attempt to create a bond file failed to open -- even while disconnected, so this was never the
	// flash-during-radio-activity problem it first looked like. deriveLinkKey() writes that file directly
	// (see there for why it can't use conn->saveBondKey()), so the directory has to be present. Doing it
	// here, at init, keeps it off the time-critical path during the handshake.
	InternalFS.mkdir("/adafruit");
	InternalFS.mkdir("/adafruit/bond_prph");
	Serial.printf("# LEAN bond dir present=%d\n", InternalFS.exists("/adafruit/bond_prph"));

	loadBondedHost(); // decides pairing vs wake advertising below

	// PUBLIC-type address instead of Bluefruit's default random-static one. nRF Connect showed a
	// default-address capture had the top two bits set (random-static pattern); real controller chips
	// are normally provisioned with genuine public addresses. Not a real Nintendo-assigned OUI -- just
	// the address TYPE bit, which is what we could actually test. Didn't change the disconnect pattern
	// on its own, but left in since it's a closer match to a real controller either way.
	// Rotated 2026-08-06: one value had been used for the entire session -- after dozens of connect/disconnect
	// cycles with that same identity never completing pairing, connection attempts were visibly taking much
	// longer to even start than they did early in the session. Testing whether that's an adaptive backoff
	// the Switch applies to a specific device identity that's repeatedly failed to pair, by trying a fresh
	// one. (SET_MAC's payload below must stay in sync with this -- it announces this same address.)
	initBleAddr();
	ble_gap_addr_t addr;
	addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
	memcpy(addr.addr, g_bleAddr, 6);
	Bluefruit.setAddr(&addr);

	// GATT Device Name. Not in the advert (there is no room), but it IS readable by anyone who connects,
	// so it must agree with the advertised PID. A real Joy-Con reads back "Joy-Con 2 (R)"; no Pro
	// Controller 2 has ever been dumped publicly, so this follows Nintendo's observed naming shape.
	// Keep in step with g_idMode.
	// DERIVED from g_idMode instead of hardcoded (review 2026-08-14 #4). It said "Pro Controller 2"
	// while g_idMode defaulted to 0 = Joy-Con 2 Right, so everything read over GATT contradicted
	// everything advertised -- with a comment two lines up promising they were kept in step.
	// A real Joy-Con reads back "Joy-Con 2 (R)"; no Pro Controller 2 has been dumped publicly, so that
	// entry follows Nintendo's observed naming shape.
	static const char *const GAP_NAME[3] = { "Joy-Con 2 (R)", "Pro Controller 2", "HORI Pad" };
	Bluefruit.setName(GAP_NAME[g_idMode]);
	// GAP Appearance = Gamepad (0x03C4, Bluetooth SIG assigned number under the HID category). Added
	// after the first few connections all disconnected without ever touching a characteristic; didn't
	// change the pattern either, but a real controller almost certainly sets this so it stays.
	Bluefruit.setAppearance(0x03C4);
	Bluefruit.Periph.setConnectCallback(bleConnectCb);
	Bluefruit.Periph.setDisconnectCallback(bleDisconnectCb);
	// Deliberately no security/pairing request -- real Switch 2 controllers
	// accept an unauthenticated link and apparently drop it if a central attempts standard SMP pairing.

	setupGatt();
	markStage(STAGE_GATT_DONE);

	// Manufacturer-specific-data AD field -- GROUND TRUTH, captured live via nRF Connect's raw-bytes view
	// off an actual Switch 2 Joy-Con in sync/pairing mode (NOT inferred from the PC-side reference repos,
	// which never needed to construct this themselves -- they only ever receive it). Full raw AD packet
	// was exactly 31 bytes (the legacy advertising limit, no slack) with NO name field at all:
	//   02 01 06                                                            (Flags = 0x06)
	//   1B FF 53 05 | 01 00 03 7E 05 66 20 00 01 00 00 00 00 00 00 00 00 F0 00 00 00 00 00 00
	//   (Mfg Data AD: company ID 0x0553, then this 24-byte payload)
	// An earlier attempt had the offsets wrong (only 1 filler byte before VID instead of 3), which
	// shifted VID/PID out of place and got total silence from the Switch's pairing screen -- this is the
	// byte-for-byte-matched version, substituting Pro Controller 2's PID for the captured Joy-Con2-Right
	// one. addManufacturerData() does NOT prepend the company ID itself -- the buffer passed in must
	// start with it (2 bytes LE) per the BLE AD spec.
	buildAdvertising();
	Bluefruit.Advertising.restartOnDisconnect(true);
	// Both intervals are the FAST value on purpose. Measured off the captures (scratchpad/adv_interval.js):
	// a real Joy-Con 2 advertises every ~20ms and never backs off -- p90 inter-event gap is 19.4ms in the
	// wake capture and 20.3ms during fresh pairing. Bluefruit's default here dropped us to the slow interval
	// (244 * 0.625 = 152ms) after 30s, i.e. 7.6x fewer chances for the console to hear us. Because
	// restartOnDisconnect() resets the fast phase, this stayed invisible while the console was actively
	// cycling and only bit during a quiet spell -- which then sustained itself, since a 152ms advertiser is
	// much easier for a scan to miss. That produced exactly the bursty connect pattern in the logs: several
	// connections seconds apart, then minutes of nothing.
	// Advertising interval is now a RUNTIME SWEEP ('[' cycles it) -- see g_advIntervalIdx. Default index
	// 0 is 32 units = 20 ms, the measured real-device value and the long-standing behavior.
	Bluefruit.Advertising.setInterval(ADV_INTERVAL_UNITS[g_advIntervalIdx],
					  ADV_INTERVAL_UNITS[g_advIntervalIdx]);
	Bluefruit.Advertising.setFastTimeout(0);
	Bluefruit.Advertising.start(0);
	markStage(STAGE_ADV_STARTED);
	markStage(STAGE_INIT_COMPLETE);
}

// ---- WAKE. Confirmed from docs/captures/joycon2_reconnect_undecrypted.pcap, which IS a Home-button
// wake-from-sleep capture: the real Joy-Con broadcasts TWO different manufacturer payloads.
//
//   pairing mode  01 00 03 7e 05 66 20 00 01 00 | 00 00 00 00 00 00 | 0f 00 ...
//   wake mode     01 00 03 7e 05 66 20 00 01 00 | ff ee dd cc bb aa | 0f 00 ...
//                                                 ^ bonded host's MAC, wire order
//
// Bytes [10:16] are the "reconnect MAC" -- zero means "in pairing mode, anyone may connect", non-zero
// invites one specific host. A sleeping console scans for its own address there. Advertising zeros, as this
// firmware always has, means a sleeping Switch will never respond no matter how correct everything else is;
// it only ever gets picked up by the Change Grip/Order screen.
//
// Also note the 283 input reports in that capture contain NO button-press signature -- no byte behaves like
// pressed-then-released. So the console appears to wake from a registered controller connecting and
// completing the handshake, not from a HOME report. The Home button's job is to wake the CONTROLLER's radio
// so it starts advertising. That makes wake mostly a matter of advertising the right thing.
static void buildAdvertising()
{
	uint8_t mfgData[26];
	memset(mfgData, 0, sizeof mfgData);
	mfgData[0] = (uint8_t)(NINTENDO_COMPANY_ID & 0xFF);
	mfgData[1] = (uint8_t)(NINTENDO_COMPANY_ID >> 8);
	mfgData[2] = 0x01;
	mfgData[3] = 0x00;
	mfgData[4] = 0x03;
	// Vendor/product come from g_idMode ('G'), not the fixed Joy-Con constants -- see g_idMode.
	mfgData[5] = (uint8_t)(ID_VID[g_idMode] & 0xFF);
	mfgData[6] = (uint8_t)(ID_VID[g_idMode] >> 8);
	mfgData[7] = (uint8_t)(ID_PID[g_idMode] & 0xFF);
	mfgData[8] = (uint8_t)(ID_PID[g_idMode] >> 8);
	mfgData[9] = 0x00;
	mfgData[10] = 0x01;
	// mfgData[i] is payload[i-2] (the first two bytes are the company ID). Reconnect MAC = payload[10:16]
	// = mfgData[12..17]: zero for pairing mode, the bonded host's address in wire order for wake.
	if (g_haveBondedHost) {
		memcpy(mfgData + 12, g_bondedHost, 6);
		Serial.printf("# LEAN advertising WAKE mode, inviting %02X:%02X:%02X:%02X:%02X:%02X\n",
			      g_bondedHost[5], g_bondedHost[4], g_bondedHost[3], g_bondedHost[2],
			      g_bondedHost[1], g_bondedHost[0]);
	} else {
		Serial.println("# LEAN advertising PAIRING mode (reconnect MAC zeroed)");
	}
	// ADVERT BYTE -- follows bond state. NOTE the index convention: mfgData[i] is payload[i-2], because
	// the first two mfgData bytes are the company ID. So payload[16]/[17] == mfgData[18]/[19].
	//
	//   unbonded -> GENERIC (0xF0 at mfgData[19]): the console DISCOVERS us. Required to register.
	//   bonded   -> REAL    (0x0F at mfgData[18]): what genuine hardware broadcasts, and how the console
	//                                              recognizes a controller it already knows, from any screen.
	//
	// Verified over the air (bleak scan) that the byte actually changes -- an earlier A/B compared a value
	// that was never broadcast. Autonomous return REQUIRES the real form: broadcasting generic forever is
	// why the puck was only ever picked up from Change Grip/Order.
	// 'Z' forces the real form while unbonded, for testing.
	if (g_advRealByte || g_haveBondedHost) {
		mfgData[18] = 0x0F;
		mfgData[19] = 0x00;
	} else {
		mfgData[18] = 0x00;
		mfgData[19] = 0xF0;
	}

	// No addName() at all -- the real captured advertisement has no Complete/Shortened Local Name AD
	// structure whatsoever, and the packet is already exactly at the 31-byte legacy limit with none to
	// spare. The Bluefruit.setName() value stays a CDC/USB-side label only, never broadcast.
	Bluefruit.Advertising.clearData();
	bool okFlags = Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
	bool okMfg = Bluefruit.Advertising.addManufacturerData(mfgData, sizeof mfgData);
	if (!okFlags || !okMfg)
		Serial.printf("# LEAN ADVERTISING BUILD FAILED (flags=%d mfg=%d) -- payload is STALE\n",
			      okFlags, okMfg);
}

// Re-advertise in the other mode without a reboot. Verified 2026-08-07 that simply calling this while
// advertising was live did NOT take effect: the old payload kept going out over the air until an unrelated
// reset rebuilt it. clearData() will not apply to a running advertiser, so addManufacturerData() silently
// left the previous data in place. stop() must actually complete first, and the result has to be checked --
// the failure is invisible otherwise, and "toggled the mode but it kept advertising the old one" is exactly
// the kind of thing that eats an evening.
static void restartAdvertising()
{
	Bluefruit.Advertising.stop();
	delay(20); // let the advertiser actually stop before touching its data
	buildAdvertising();
	bool ok = Bluefruit.Advertising.start(0);
	Serial.printf("# LEAN advertising restarted: start=%d mode=%s\n", ok,
		      g_haveBondedHost ? "WAKE" : "PAIRING");
}

static void bleHeartbeat()
{
	static unsigned long startMs = 0;
	static bool init = false;
	unsigned long now = millis();
	if (!init) {
		init = true;
		startMs = now;
		return;
	}
	uint32_t elapsedMs = now - startMs;
	static uint8_t lastTick = 0xFF;
	uint8_t tick = (uint8_t)((elapsedMs / 100 > 255) ? 255 : (elapsedMs / 100));
	// Once per SECOND, not every 100ms. The flash write was removed long ago; the print remained and
	// buried every meaningful event under 255 lines per boot (audit BUG-14).
	if (tick != lastTick && (tick % 10) == 0) {
		lastTick = tick;
		markStage(STAGE_ALIVE, tick);
	}

	// Periodic MTU/PHY snapshot while connected. (The requestMtuExchange(247) call this once watched was
	// REMOVED -- a peripheral must not initiate MTU exchange; only the console does. Removing it was
	// recorded at the time as the single biggest unlock of that session.)
	static unsigned long lastMtuLogMs = 0;
	if (!g_quiet && g_connHdl != BLE_CONN_HANDLE_INVALID && now - lastMtuLogMs >= 2000) {
		lastMtuLogMs = now;
		BLEConnection *conn = Bluefruit.Connection(g_connHdl);
		if (conn) {
			Serial.printf("# LEAN mtu=%u phy=%u t=%lu ms\n", conn->getMtu(),
				      conn->getPHY(), (unsigned long)millis());
		}
	}

	// Retry a stuck wake-queue entry (the last sd_ble_gattc_write() call got NRF_ERROR_BUSY/RESOURCES
	// because the SoftDevice's single outstanding-WRITE_REQ slot or the WRITE_CMD queue was still full) --
	// covers the case where no WRITE_RSP event will ever arrive to trigger the retry itself.
	if (g_wakeQueueActive && now - g_wakeQueueLastTryMs >= 50)
		wakeQueueTrySend();

	// Stream input reports once per 15ms (the real device's rate), but ONLY after the Switch has started
	// the stream by writing the 679d descriptor -- a real controller sends nothing before that, and
	// notifying early would just be unsolicited traffic the Switch never asked for. Also handles the
	// one-shot armed Home press from 'H'.
	// Wake-mode escape hatch (see bleConnectCb). A reconnect to a console that still has us registered
	// starts issuing commands within a few hundred ms. Total silence for 6s means it has forgotten us,
	// and staying in wake mode would leave us permanently invisible: unrecognized by that console, and
	// unpairable on the Change Grip/Order screen. Fall back so it can register us again.
	// Fires on TIME, not only while connected. The first version only ran inside a live connection, so if
	// the console stopped reconnecting altogether we sat in wake mode advertising at something that was no
	// longer listening -- no connection, so the check never ran, and we stayed invisible to both paths
	// forever. That is the exact dead end this is meant to prevent.
	// MUST be disconnected, and the timer measures time spent ADVERTISING UNANSWERED -- not time since
	// the last command. The original form (any 20s without a command, connected or not) fired ~20s after
	// every SUCCESSFUL registration and deleted the record it had just written: 5cdbd17's own serial log
	// shows `bonded host saved` -> `registration complete` -> this message, while the link was up and
	// streaming 1093 reports with 0 failures. Post-registration silence is what steady state LOOKS like;
	// once the stream runs the console has nothing to say, so treating quiet as amnesia threw away our
	// registration every time it succeeded. That is why the console's Find screen shows a "?" (it reaches
	// for a bonded controller while we advertise as unpaired) and why we had to be re-paired from Change
	// Grip/Order on every use -- the durability the acceptance test is actually about.
	// REMOVED ENTIRELY 2026-08-08. There is no timeout at which "the console has not connected" justifies
	// forgetting it. A console can be asleep, off, or simply not looking for hours, and a real controller
	// keeps its pairing across all of that -- that is what being paired MEANS. Every version of this check
	// was wrong in the same direction, just at a different threshold:
	//
	//   20s, any silence      deleted the record ~20s after every successful registration, while the link
	//                         was up and streaming (see 5cdbd17's log).
	//   180s, disconnected    deleted it 180s after a REBOOT, because g_lastCmdOkMs is seeded at boot. So
	//                         kicking the puck -- the acceptance test itself -- armed a timer that erased
	//                         the bond before the console ever came back, and from then on only Change
	//                         Grip/Order could recover it.
	//
	// The dead end this was meant to escape (bonded to a console that has genuinely dropped us, so we are
	// invisible to wake AND unpairable) is real, but it already has a manual escape: the 'C' console
	// command forgets the host on demand. A deliberate key press cannot fire by accident 3 minutes after
	// every reboot.

	// Sample the live connection interval. Cheap, and it catches a mid-connection parameter update
	// without needing a dedicated event handler.
	static unsigned long lastIvlMs = 0;
	if (g_connHdl != BLE_CONN_HANDLE_INVALID && now - lastIvlMs >= 1000) {
		lastIvlMs = now;
		BLEConnection *ic = Bluefruit.Connection(g_connHdl);
		if (ic) {
			uint16_t iv = ic->getConnectionInterval();
			// Print on the FIRST sample of a connection as well as on any change. The previous
			// version only printed on change, and since the initial value matches the reset value
			// it printed nothing at all -- an instrumentation bug, not a measurement.
			static bool firstThisConn = true;
			if (g_connIntervalUnits == 0)
				firstThisConn = true;
			if (iv != g_connIntervalUnits || firstThisConn) {
				firstThisConn = false;
				Serial.printf("# LEAN conn interval now %u (x1.25ms = %u.%02u ms)\n", iv,
					      (unsigned)(iv * 125 / 100), (unsigned)((iv * 125) % 100));
				g_connIntervalUnits = iv;
			}
		}
	}
	static unsigned long lastReportMs = 0;
	// 30ms, NOT 15ms. MEASURED 2026-08-09 off a real Joy-Con 2 Right streaming to the console with our
	// puck unplugged (scratchpad/handoff_real_joycon.pcap, t=24-31): 202 report packets, and 200 of the
	// 201 gaps are EXACTLY 30ms. We had been sending at double the real rate.
	//
	// The old "~66 reports/sec (75 reports in 1140ms)" figure is 15.2ms -- exactly half of 30ms, which is
	// what you measure by counting BOTH directions as if every packet on the link were a report. Same
	// error shape as the rest of this file's history: a real measurement, read one level too coarsely.
	if (g_streaming && g_connHdl != BLE_CONN_HANDLE_INVALID && now - lastReportMs >= 30) {
		// DO NOT consume the slot here. sendInputReport() returns without sending in two cases (the
		// 8 ms command-reply holdoff and the g_hvnOutstanding flow-control gate), and g_motionFrame /
		// g_reportSeq only advance on a SUCCESSFUL hvx. Stamping the timer up front meant every
		// skipped slot advanced wall-clock by 30 ms and our embedded IMU timestamp by 0 -- so the
		// clock we transmit fell permanently behind the link's real cadence, by an amount that
		// depended on how much command traffic collided with the stream, i.e. differently on every
		// connection. Retry the slot instead; the timer is stamped below only if a report went out.
		// (review 2026-08-14 #3 -- the first candidate that explains the VARIABILITY, not just the
		// failure.)
		g_reportSentThisSlot = false;
		static uint8_t homeHoldTicks = 0;
		if (g_wakeArmed && homeHoldTicks == 0) {
			homeHoldTicks = 8; // hold ~120ms to look like a real press-release
			g_wakeArmed = false;
			Serial.println("# LEAN: sending HOME press");
		}
		// The decrement moved BELOW the send and is gated on g_reportSentThisSlot (review 2026-08-14b
		// #2). While a slot is being skipped the heartbeat re-enters at loop() rate, so decrementing
		// here burned the "hold ~120 ms" window in eight loop passes -- under a millisecond, and no
		// press edge the console could ever see.
		bool homeNow = (homeHoldTicks > 0) || g_homeHeld;

		// AUTO-PRESS once the stream starts. Two separate things need this and both must work with
		// nobody at a console: the Change Grip/Order registration step asks for an L+R press, and the
		// post-wake lock screen needs a face button 3-4 times or it goes back to sleep. Since we do not
		// know the bit positions yet, g_buttonMask presses every bit in the field -- the console only
		// has to see the ones it is waiting for. ~200ms down, ~200ms up, ten cycles.
		// Press CONTINUOUSLY while streaming, ~200ms down / ~200ms up, for as long as the connection
		// lasts -- not a one-shot burst. The burst version fired for ~4s right after the stream started
		// and never completed registration; each connection only survives ~10s, so a fixed window can
		// easily miss whenever the console actually wants the press. Repeated press/release also gives a
		// real edge rather than a level, which button-prompt UIs generally want.
		// ONLY when a mask is actually set. This ran unconditionally and printed
		// "auto-pressing mask=0x00000000 byte=2 (continuous)" on every connection, which reads as
		// "we are pressing buttons" while the OR writes nothing (audit BUG-13).
		// PHASE-LOCKED TO THE ARM (user's idea, 2026-08-14). This counter used to be a free-running
		// static that never reset, so the press/release cycle -- 13 reports down, 13 up, roughly
		// 430 ms each at ~30/s -- drifted against the console's connection cadence. The stream only
		// opens ~1.5-2 s into a connection and we get a handful of reports before the drop, so whether
		// SL+SR happened to be DOWN or UP during that window was down to where the free-running counter
		// had wandered. Two asynchronous cycles beating against each other produce exactly the observed
		// behavior: rare alignment, minutes of nothing, then a sudden good run.
		//
		// g_autoTick is now reset when the console arms the stream, so every connection presents the
		// same button phase from the same instant and the outcome stops depending on luck.
		// g_autoTick likewise advances only on a transmitted report (below), not per call -- otherwise
		// the 13-down/13-up cycle runs at loop() rate during a skip and the press flickers instead of
		// holding, which defeats the phase-lock this counter exists for.
		if (g_buttonMask) {
			if (((g_autoTick / 13) % 2) == 0)
				homeNow = true;
			// GATED. This test runs on every CALL while g_autoTick advances only on a transmitted
			// report, and the heartbeat re-enters at loop() rate whenever a slot is skipped -- so
			// during a skip it fired every pass. Worse, it is guaranteed to happen at the arm: the
			// 0x01/0x0C reply there stamps g_lastCmdReplyMs, which arms the 8 ms holdoff, so the
			// first slot after every arm is inside it. Serial.printf BLOCKS ~85 ms when a host holds
			// the CDC open, and this put that block in a tight loop at the exact moment the console
			// starts watching for reports. (review 2026-08-14c #2 -- my regression from the b#2 fix,
			// which moved the counter and left the log gate testing it.)
			if (!g_quiet && g_autoTick == 0) {
				Serial.printf("# LEAN *** auto-pressing mask=0x%08lX byte=%u (continuous)",
					      (unsigned long)g_buttonMask, g_buttonOffset);
				Serial.println(" ***");
			}
		}
		sendInputReport(homeNow);
		// Only now is the slot spent. A skipped report leaves lastReportMs alone so the next loop
		// pass retries immediately, keeping the transmitted IMU clock in step with real time.
		//
		// EVERY per-report counter advances here and nowhere else, so all of them stay in step with the
		// reports actually on air: the IMU clock (committed in sendInputReport's success path),
		// g_reportSeq, g_motionFrame, and these two press timers.
		if (g_reportSentThisSlot) {
			lastReportMs = now;
			if (homeHoldTicks > 0)
				homeHoldTicks--;
			if (g_buttonMask)
				g_autoTick++;
		}
	}
}

static void serialConsolePoll()
{
	while (Serial.available()) {
		int c = Serial.read();
		if (c == 'H' || c == 'h') {
			g_wakeArmed = true;
			Serial.printf("# LEAN: HOME press armed (offset=%u mask=0x%08lX)%s\n", g_buttonOffset,
				      (unsigned long)g_buttonMask,
				      g_streaming ? "" : " -- WARNING: stream not enabled yet, nothing will be sent");
		} else if (c == 'C' || c == 'c') {
			// Toggle pairing-mode vs wake-mode advertising. Wake mode needs a remembered console, saved
			// automatically at 0x0C/0x04 (registration complete). This is the ONLY escape from the dead
			// end where the console has forgotten us but we are still inviting it -- the automatic
			// fallback that used to do it was removed (no timeout is correct; see loop()).
			if (g_haveBondedHost) {
				g_haveBondedHost = false;
				InternalFS.remove("/bondedhost.bin");
				Serial.println("# LEAN: forgot bonded host -> PAIRING mode");
			} else {
				loadBondedHost();
				if (!g_haveBondedHost) {
					// Nothing stored yet: fall back to the console seen in this session's
					// logs so wake can be tested before a successful bond.
					static const uint8_t knownSwitch[6] = { 0x77, 0x4C, 0xF7,
										0xF7, 0x44, 0x40 };
					saveBondedHost(knownSwitch);
				}
				Serial.println("# LEAN: -> WAKE mode");
			}
			restartAdvertising();
		} else if (c == 'U' || c == 'u') {
			g_phy2M = !g_phy2M;
			Serial.printf("# LEAN: PHY -> %s (takes effect on the next connection)\n",
				      g_phy2M ? "2M (matches real hardware)" : "1M (longer range)");
		} else if (c == 'G' || c == 'g') {
			g_idMode = (uint8_t)((g_idMode + 1) % 3);
			Serial.printf("# LEAN: identity -> %s\n", ID_NAME[g_idMode]);
			buildAdvertising();
			restartAdvertising();
		} else if (c == 'Q' || c == 'q') {
			g_bd283Captured = !g_bd283Captured;
			chrS1R2.write(g_bd283Captured ? BD283_CAPTURED : BD283_UNIQUE, sizeof BD283_UNIQUE);
			Serial.printf("# LEAN: bd283 -> %s\n",
				      g_bd283Captured ? "CAPTURED real per-device value" : "synthetic unique value");
		} else if (c == 'Z' || c == 'z') {
			g_advRealByte = !g_advRealByte;
			Serial.printf("# LEAN: advert byte -> %s\n",
				      g_advRealByte ? "0x0F at payload[16] (real Joy-Con form)" :
						      "0xF0 at payload[17] (old form)");
			buildAdvertising();
			restartAdvertising();
		} else if (c == 'M' || c == 'm') {
			// Cycle WHICH buttons the auto-press holds. The point of the B option is a decisive test
			// of whether the console reads our report stream at all: B is cancel on the Change
			// Grip/Order screen, so if reports are being consumed, holding B backs the screen out
			// visibly. If nothing happens for any mask, the console is discarding the stream -- which
			// we have never actually been able to rule out, since the SoftDevice accepting a
			// notification for transmission says nothing about the peer's host layer keeping it.
			static const uint32_t MASKS[] = { 0, BTN_SL | BTN_SR, BTN_B, BTN_HOME, 0x0000FFFFUL };
			static const char *MNAMES[] = { "NONE -- press nothing (quiet controller)",
							"SL+SR (Joy-Con registration gesture)",
							"B only (Joy-Con map)", "HOME only (Joy-Con map)",
							"every bit in the field" };
			uint8_t i = 0;
			while (i < 5 && MASKS[i] != g_buttonMask)
				i++;
			i = (uint8_t)((i + 1) % 5);
			g_buttonMask = MASKS[i];
			Serial.printf("# LEAN: button mask -> 0x%08lX  %s\n", (unsigned long)g_buttonMask, MNAMES[i]);
		} else if (c == 'Y' || c == 'y') {
			static const char *names[5] = { "bare 8-byte ack (default)", "no reply at all",
							"payload 01 00 00 00", "payload 00 00 00 00",
							"echo its own payload back" };
			g_a08Mode = (uint8_t)((g_a08Mode + 1) % 5);
			Serial.printf("# LEAN: 0x0A/0x08 reply mode %u -- %s\n", g_a08Mode, names[g_a08Mode]);
		} else if (c == '2') {
			// De-clone toggle. Default ON = serve OUR per-unit data, not the captured Joy-Con's.
			g_ownFingerprint = !g_ownFingerprint;
			Serial.printf("# LEAN per-unit identity: %s\n", g_ownFingerprint
				      ? "OURS (13040/13100/11_03 unique)"
				      : "CLONED Joy-Con (byte-identical to a real unit)");
		} else if (c == '6') {
			// KEY MATERIAL. ANNOUNCED_LTK1 is the real unit's, byte-for-byte, and feeds link-key
			// derivation -- see its declaration. Isolated from the safe swaps because if pairing
			// itself breaks, this is the first thing to turn back off.
			g_ownKeyMaterial = !g_ownKeyMaterial;
			Serial.printf("# LEAN key material (LTK1/LTK2): %s\n", g_ownKeyMaterial
				      ? "OURS -- no longer sharing key-derivation input with a real Joy-Con"
				      : "CLONED from the captured Joy-Con");
		} else if (c == ';') {
			// Step byte 0 through the candidates. See g_byte0Override for how to read the result.
			g_byte0Override = SWEEP_CAND[g_sweepIdx];
			g_sweepIdx = (uint8_t)((g_sweepIdx + 1) % 6);
			Serial.printf("# LEAN CONTROLLER_INFO[0] = 0x%02X  (original is 0x01). Reject within "
				      "~13ms of encryption => this value is INVALID\n", g_byte0Override);
		} else if (c == ',') {
			// Byte 1 -- never tested, always skipped as a zero. 0x00 is its original value.
			g_byte1Override = (g_byte1Override == 0xFFFF) ? 0x77 : 0xFFFF;
			Serial.printf("# LEAN CONTROLLER_INFO[1] = %s\n", g_byte1Override == 0xFFFF
				      ? "original 0x00" : "0x77 (first ever test of this byte)");
		} else if (c == '+') {
			// Step the battery-field probe ('+'). Forces the body FROZEN so every frame stays a real,
			// coherent, static captured frame -- see the crash warning in the WORKING CONFIGURATION
			// block. Frozen alone reads "low battery"; if raising a slot makes it read FULL, that
			// slot is the field.
			g_battProbe = (uint8_t)((g_battProbe + 1) % (BATT_PROBE_N + 1));
			g_replayIdx = 1; // {0,0} -- replay writes nothing
			g_replayLo = 0;
			g_replayHi = 0;
			if (g_battProbe)
				Serial.printf("# LEAN BATT PROBE %u/%u: body FROZEN, raising [%u:%u) to FF 0F\n",
					      g_battProbe, BATT_PROBE_N, BATT_PROBE_OFF[g_battProbe - 1],
					      BATT_PROBE_OFF[g_battProbe - 1] + 2);
			else
				Serial.println("# LEAN BATT PROBE off -- body still FROZEN, nothing raised");
		} else if (c == '/') {
			g_replayIdx = (uint8_t)((g_replayIdx + 1) % 2);
			g_replayLo = REPLAY_RANGES[g_replayIdx][0];
			g_replayHi = REPLAY_RANGES[g_replayIdx][1];
			Serial.printf("# LEAN BISECT %u: replay writes [%u:%u), rest FROZEN to template\n",
				      g_replayIdx, g_replayLo, g_replayHi);
		} else if (c == '-') {
			g_batteryMvOn = !g_batteryMvOn;
			Serial.printf("# LEAN report[31:35] battery mV field: %s\n",
				      g_batteryMvOn ? "HELD STEADY (see g_batteryMv)" :
						      "off -- motion replay writes those bytes");
		} else if (c == '=') {
			g_batteryMvIdx = (uint8_t)((g_batteryMvIdx + 1) % 4);
			g_batteryMv = BATTERY_MV[g_batteryMvIdx];
			Serial.printf("# LEAN battery voltage -> %u mV\n", g_batteryMv);
		} else if (c == '`') {
			g_quiet = !g_quiet;
			Serial.printf("# LEAN quiet mode: %s\n", g_quiet ?
				      "ON -- heartbeat/report-head/mtu prints suppressed" :
				      "OFF -- full chatter (WARNING: blocks the loop ~85 ms when a host reads COM3)");
		} else if (c == ']') {
			g_batteryIdx = (uint8_t)((g_batteryIdx + 1) % 8);
			g_batteryByte = BATTERY_BYTES[g_batteryIdx];
			Serial.printf("# LEAN report[8] battery/mode -> 0x%02X (charge nibble %u, mode %u)%s\n",
				      g_batteryByte, g_batteryByte >> 4, g_batteryByte & 0x0F,
				      g_batteryByte == 0x38 ? "  <-- captured real-device value" : "");
		} else if (c == '[') {
			// Sweep the advertising interval and re-advertise, to test whether the console's
			// offered connInterval tracks it. Measure the result in the sniffer, not here:
			// the offered value lives in the CONNECT_IND, which we never see when we ignore it.
			g_advIntervalIdx = (uint8_t)((g_advIntervalIdx + 1) % 4);
			uint16_t u = ADV_INTERVAL_UNITS[g_advIntervalIdx];
			Bluefruit.Advertising.stop();
			Bluefruit.Advertising.setInterval(u, u);
			restartAdvertising();
			Serial.printf("# LEAN advertising interval -> %u units = %u.%02u ms\n", u,
				      (unsigned)(u * 625UL / 1000), (unsigned)((u * 625UL % 1000) / 10));
		} else if (c == '.') {
			// Force the console to re-register us -- see buildInfoBlob(). Use this after ANY
			// change to data the console only reads at registration time (colors are the
			// proven case; other cached fields likely behave the same).
			// 45s, not 8s. The 8s version produced only ~7 rejections and the console kept
			// retrying rather than giving up -- the manual break/restore that DID refresh the
			// colors had the bad header live for well over a minute. If the console has to
			// fully evict the registration rather than merely fail a few connections, the
			// window has to outlast its retry budget.
			g_poisonUntilMs = millis() + 45000;
			Serial.println("# LEAN *** REFRESH: poisoning CONTROLLER_INFO[0] for 45s. Expect a "
				       "long run of reason=0x13 while the console retries and gives up. "
				       "Re-pair from Change Grip/Order once it clears. ***");
		} else if (c == '0') {
			g_ownColors = !g_ownColors;
			Serial.printf("# LEAN controller colors: %s\n", g_ownColors
				      ? "OURS (see the colors-served line on the next info read)"
				      : "captured original (near-black body, light grey buttons)");
		} else if (c == '9') {
			g_infoRegion = (uint8_t)((g_infoRegion + 1) % 6);
			Serial.printf("# LEAN CONTROLLER_INFO perturbed region: %s\n",
				      INFO_REGION_NAME[g_infoRegion]);
			Serial.println("# LEAN   reason=0x13 right after encryption => this region IS validated");
		} else if (c == '8') {
			// The two captured TEMPLATES. INPUT_REPORT is flagged outright by clonecheck.js;
			// CONTROLLER_INFO evaded it only because its serial bytes were already changed.
			g_ownTemplates = !g_ownTemplates;
			Serial.printf("# LEAN captured templates (report + controller info): %s\n",
				      g_ownTemplates ? "OURS" : "CLONED from the real unit");
		} else if (c == '7') {
			g_ownMisc = !g_ownMisc;
			Serial.printf("# LEAN RSP_01_0C: %s\n", g_ownMisc ? "OURS" : "cloned");
		} else if (c == '5') {
			// Echo probe -- see MEM_00013060's declaration. Watch the next 0x0A/0x08 ALIGN line:
			// [26]/[27] should come back as our bytes if the console really reads our pages.
			g_ownPage13060 = !g_ownPage13060;
			Serial.printf("# LEAN 0x13060 page: %s -- watch 0x0A/0x08 ALIGN [26]/[27], "
				      "expect %02X %02X\n",
				      g_ownPage13060 ? "OURS" : "cloned Joy-Con",
				      g_ownPage13060 ? 0x3c : 0x69, g_ownPage13060 ? 0x00 : 0x09);
		} else if (c == '4') {
			// Calibration is staged separately: 0x13080's tail may be a checksum (see declaration).
			g_ownCalib = !g_ownCalib;
			Serial.printf("# LEAN calibration page 0x13080: %s\n", g_ownCalib
				      ? "OURS (checksum risk -- if bring-up regresses, turn this back off first)"
				      : "cloned (known-good)");
		} else if (c == '3') {
			// Sweep the side/model discriminator. L=0x00 and R=0x01 are measured; the Pro Controller
			// value is UNKNOWN, so cycle candidates rather than reflashing for each guess.
			// 0xFF and 0x7F are deliberately OUTLANDISH. 0x00-0x03 all behaved identically,
			// which says the console ignores this field -- but "ignores" and "accepts anything
			// in a small range" look the same from four adjacent values. An out-of-range value
			// distinguishes them: if 0xFF also sails through, the field is genuinely unread.
			static const uint8_t cand[6] = { 0x02, 0x03, 0x00, 0x01, 0xff, 0x7f };
			uint8_t i = 0;
			while (i < 6 && cand[i] != g_cmd10Disc) i++;
			g_cmd10Disc = cand[(i + 1) % 6];
			ANNOUNCED_CMD10[3] = g_cmd10Disc;
			Serial.printf("# LEAN CMD10[3] discriminator = 0x%02X%s\n", g_cmd10Disc,
				      g_cmd10Disc == 0x00 ? " (left Joy-Con's value -- but for US it likely reads"
							    " 'NO SIDE APPLICABLE', which is what a full controller"
							    " actually is. Strong candidate.)"
				      : g_cmd10Disc == 0x01 ? " (RIGHT Joy-Con -- measured, contradicts our PID)"
							    : " (Pro Controller candidate -- unverified guess)");
		} else if (c == '1') {
			// A/B the unprompted 0x01/0x0C (see the send site). Default off; this turns it back on
			// so the failing behavior can be reproduced on demand rather than from memory.
			// A DIGIT, not a letter: every letter A-Z is already bound, and 'U' in particular is the
			// PHY toggle -- binding this to 'U' produced a silently unreachable duplicate branch.
			g_askUnprompted = !g_askUnprompted;
			Serial.printf("# LEAN unprompted 0x01/0x0C after 0x0C/0x04: %s\n",
				      g_askUnprompted ? "ON (prime suspect -- expect a ~15s supervision timeout)"
						      : "OFF (we never speak first)");
		} else if (c == 'N' || c == 'n') {
			// Control test: stop forcing the report stream on. We have forced it on every run since
			// the stream was first made to work, so "the console hangs up BECAUSE we notify a channel
			// it never subscribed" has never actually been ruled out. Persists across connections so
			// a whole cycle runs clean, unlike 'V' which forceEnableReportCccd() overrides at connect.
			g_forceStream = !g_forceStream;
			if (!g_forceStream)
				g_streaming = false;
			Serial.printf("# LEAN: report stream forcing %s\n", g_forceStream ? "ON" : "OFF (control test)");
		} else if (c == 'V' || c == 'v') {
			// Force the report stream on without waiting for the Switch's descriptor write. Insurance
			// for tonight: if the trigger isn't detected (or sd_ble_gatts_value_set can't set a CCCD
			// on this SoftDevice), this still exercises the report path so the two failures can be
			// told apart.
			g_streaming = !g_streaming;
			if (g_streaming)
				forceEnableReportCccd();
			Serial.printf("# LEAN: report stream forced %s\n", g_streaming ? "ON" : "OFF");
		} else if (c == 'J' || c == 'j') {
			// Hold HOME continuously rather than a ~120ms pulse. For wake testing it is much easier
			// to tell a held button apart from a missed one-shot.
			g_homeHeld = !g_homeHeld;
			Serial.printf("# LEAN: HOME %s\n", g_homeHeld ? "HELD DOWN" : "released");
		} else if (c == 'B' || c == 'b') {
			// Sweep the button-field offset. The reference repos say [4:8], but the captured idle
			// reports have varying bytes there, so that is unconfirmed for this device -- these are
			// the offsets whose bytes look constant-ish across captured idle reports.
			static const uint8_t CAND[] = { 2, 4, 9, 13, 16, 20, 55 }; // 2 = BTN_OFFSET, the MEASURED value (audit BUG-16)
			uint8_t i = 0;
			while (i < sizeof CAND && CAND[i] != g_buttonOffset)
				i++;
			g_buttonOffset = CAND[(i + 1) % sizeof CAND];
			Serial.printf("# LEAN: button offset -> %u (mask 0x%08lX)\n", g_buttonOffset,
				      (unsigned long)g_buttonMask);
		} else if (c == 'L' || c == 'l') {
			printCmdLog();
		} else if (c == 'E' || c == 'e') {
			printEvtLog();
		} else if (c == 'W' || c == 'w') {
			if (g_connHdl == BLE_CONN_HANDLE_INVALID) {
				Serial.println("# LEAN: 'W' ignored, not connected");
			} else {
				Serial.println("# LEAN: firing wake sequence (ATT client writes to Switch's GATT table)");
				sendWakeSequence(g_connHdl);
			}
		} else if (c == 'D' || c == 'd') {
			if (g_connHdl == BLE_CONN_HANDLE_INVALID) {
				Serial.println("# LEAN: 'D' ignored, not connected");
			} else {
				startServiceDiscovery();
			}
		} else if (c == 'R' || c == 'r') {
			g_handshakeReplyChannel = (uint8_t)((g_handshakeReplyChannel + 1) % 3);
			Serial.printf("# LEAN: handshake reply channel -> %s\n",
				      HANDSHAKE_CHANNEL_NAME[g_handshakeReplyChannel]);
		} else if (c == 'A' || c == 'a') {
			answerPendingHandshake();
		} else if (c == 'X' || c == 'x') {
			echoPendingHandshake();
		} else if (c == 'P' || c == 'p') {
			sendProactiveAnnouncement();
		} else if (c == 'K' || c == 'k') {
			g_skipLtk2 = !g_skipLtk2;
			Serial.printf("# LEAN: LTK2 notification %s\n",
				      g_skipLtk2 ? "SKIPPED on next 'P'" : "included on next 'P'");
		} else if (c == '~') {
			// DUMP THE STORED LINK KEY, so a sniffer capture can be decrypted OFFLINE.
			//
			// This exists because holding COM3 open during a connection CAUSES the failure we are
			// trying to observe (Serial blocks ~85 ms when a host drains the CDC; the console asks
			// 0x10/0x01 right after arming the stream and we answer ~419 ms late, so it hangs up).
			// The sniffer on COM10 touches nothing, but our link is encrypted -- so: sniff
			// unobserved, then read the key back HERE while DISCONNECTED, and decrypt after the
			// fact with docs/captures/ble_decrypt.js. Full visibility, zero perturbation.
			//
			// The bond file is [len][bond_keys_t][name...]; bond_keys_t starts with ble_gap_enc_key_t
			// whose first 16 bytes are the LTK. Print both byte orders: ble_decrypt.js wants the
			// AES/MSO-first (reversed) form, which is what matched the real capture 315/315.
			{
				// g_bondedHost is wire order, same as the peer address deriveLinkKey() uses to
				// build the filename. 'C' clears it, so dump BEFORE switching to pairing mode.
				loadBondedHost();
				if (!g_haveBondedHost) {
					Serial.println("# LEAN no bonded host stored -- pair first, then dump");
				} else {
					const uint8_t *peer6 = g_bondedHost;
					char fn[48];
					snprintf(fn, sizeof fn, "/adafruit/bond_prph/%02X%02X%02X%02X%02X%02X",
						 peer6[0], peer6[1], peer6[2], peer6[3], peer6[4], peer6[5]);
					File f(InternalFS);
					if (!f.open(fn, FILE_O_READ)) {
						Serial.printf("# LEAN bond file %s NOT FOUND\n", fn);
					} else {
						uint8_t len = (uint8_t)f.read();
						uint8_t key[16];
						int got = f.read(key, sizeof key);
						f.close();
						Serial.printf("# LEAN bond %s (field len %u, read %d)\n", fn, len, got);
						Serial.print("# LEAN LTK as-stored : ");
						for (int i = 0; i < 16; i++) Serial.printf("%02x", key[i]);
						Serial.println();
						Serial.print("# LEAN LTK reversed  : ");
						for (int i = 15; i >= 0; i--) Serial.printf("%02x", key[i]);
						Serial.println("   <- feed this to ble_decrypt.js");
					}
				}
			}
		} else if (c == 'T' || c == 't') {
			// Self-test for deriveLinkKey(), against a PAIR/LTK1 body captured from a console.
			//
			// The captured body and its expected results were removed before publishing, so the
			// placeholder below proves nothing on its own. To use it, drop in the LTK1 body from
			// your own capture and compare the printed key against the one that decrypts it. Since
			// ANNOUNCED_LTK1 is the universal constant a real controller answers with, the
			// reversed form is what AES wants, most significant octet first.
			//
			// Bond-store install reports failure when run while disconnected. That is expected
			// here; this test is about the key bytes, not the install.
			static const uint8_t testSwitchLtk1[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55,
								    0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb,
								    0xcc, 0xdd, 0xee, 0xff };
			// Bond-file write test, run on demand while DISCONNECTED so it isolates "flash writes
			// fail during an active BLE connection" (SoftDevice must schedule sd_flash_write between
			// radio events, and can refuse while a connection is up) from "this write is just
			// broken". Uses the real path shape and the real peer MAC.
			{
				// DUMMY MAC, not the console's. This used to be the REAL
				// console -- and the test remove()s that path twice, so running 'T' DELETED the
				// stored link key the reconnect path depends on. `puck.sh preflight` runs 'T',
				// so preflighting before a reconnect test destroyed the very state under test
				// (hit live 2026-08-09). The test only needs a well-formed path, not a real peer.
				static const uint8_t testMac[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
				char tf[48];
				snprintf(tf, sizeof tf, "/adafruit/bond_prph/%02X%02X%02X%02X%02X%02X",
					 testMac[0], testMac[1], testMac[2], testMac[3], testMac[4],
					 testMac[5]);
				Serial.printf("# LEAN: bond-write test, dir exists=%d, path %s\n",
					      InternalFS.exists("/adafruit/bond_prph"), tf);
				InternalFS.remove(tf);
				File tfh(InternalFS);
				if (tfh.open(tf, FILE_O_WRITE)) {
					uint8_t klen = (uint8_t)sizeof(bond_keys_t), nlen = 1, nul = 0;
					static bond_keys_t tbk;
					memset(&tbk, 0xA5, sizeof tbk);
					tfh.write(&klen, 1);
					tfh.write((uint8_t const *)&tbk, sizeof(bond_keys_t));
					tfh.write(&nlen, 1);
					tfh.write(&nul, 1);
					tfh.close();
					File rd(InternalFS);
					bool back = rd.open(tf, FILE_O_READ);
					Serial.printf("# LEAN: bond-write test OK, read-back=%d size=%u\n",
						      back, back ? (unsigned)rd.size() : 0u);
					if (back)
						rd.close();
					InternalFS.remove(tf);
				} else {
					Serial.println("# LEAN: bond-write test FAILED to open (while disconnected)");
				}
			}
			Serial.println("# LEAN: link-key self-test (placeholder input vs ANNOUNCED_LTK1)");
			Serial.println("# LEAN:   compare against your own capture; see the note above");
			deriveLinkKey(testSwitchLtk1);
			// End-to-end known-answer test for the PAIR/LTK2 challenge-response, using the same
			// capture's challenge (frame 1840) -- if the derived key AND the on-device AES byte
			// order are both right, this reproduces the real controller's reply from frame 1843
			// exactly. Verifies sd_ecb_block_encrypt()'s operand order without needing a Switch.
			{
				static const uint8_t testChallenge[16] = { 0x1d, 0x1e, 0xd7, 0xdd, 0x33, 0x26,
									   0x11, 0x0f, 0xfb, 0x19, 0x13, 0xf1,
									   0x6a, 0xda, 0x2a, 0x5c };
				nrf_ecb_hal_data_t ecb;
				memcpy(ecb.key, g_linkKeyMsb, 16);
				for (uint8_t i = 0; i < 16; i++)
					ecb.cleartext[i] = testChallenge[15 - i];
				Serial.println(
					"# LEAN: LTK2 challenge self-test, compare against your own capture");
				if (sd_ecb_block_encrypt(&ecb) == NRF_SUCCESS) {
					Serial.printf("# LEAN: LTK2 challenge response       ");
					for (uint8_t i = 0; i < 16; i++)
						Serial.printf("%02x", ecb.ciphertext[i]);
					Serial.println();
				} else {
					Serial.println("# LEAN: LTK2 challenge self-test AES FAILED");
				}
			}
		} else if (c == 'F' || c == 'f') {
			// Nuke and re-init the internal filesystem. Diagnosis 2026-08-06: nothing persists on
			// this board -- rotateBleAddr()'s counter never advanced past 1 across many reflashes,
			// InternalFS.exists("/adafruit/bond_prph") reported 0 even right after mkdir, and every
			// bond-file open() failed EVEN WHILE DISCONNECTED (so not radio/flash contention). Most
			// (The remove+create-per-ALIVE-tick that caused this has since been removed, as have the
			// cmd/event log writes. Kept because a full or corrupt FS is still recoverable only this way.)
			Serial.println("# LEAN: formatting InternalFS...");
			InternalFS.format();
			InternalFS.begin();
			InternalFS.mkdir("/adafruit");
			InternalFS.mkdir("/adafruit/bond_prph");
			Serial.printf("# LEAN: format done, bond dir present=%d\n",
				      InternalFS.exists("/adafruit/bond_prph"));
		} else if (c == 'I' || c == 'i') {
			// Flip the whole device identity (serial + bd283) between the synthetic-but-well-formed
			// pair and the previously-shipped captured-Joy-Con pair, so the two can be A/B'd against
			// a live Switch without a reflash. Takes effect on the next connection.
			g_useCapturedIdentity = !g_useCapturedIdentity;
			chrS1R2.write(g_useCapturedIdentity ? BD283_CAPTURED : BD283_UNIQUE,
				      sizeof BD283_UNIQUE);
			Serial.printf("# LEAN: identity = %s (serial \"%s\")\n",
				      g_useCapturedIdentity ? "CAPTURED Joy-Con (old behavior)" :
							       "synthetic, real-format (default)",
				      g_useCapturedIdentity ? SERIAL_CAPTURED : SERIAL_SYNTHETIC);
		} else if (c == 'O' || c == 'o') {
			// Byte order of the derived link key handed to the SoftDevice. Reasoning says
			// "as-XORed" is right (SMP wire order), but that isn't verified on real silicon --
			// if encryption still fails with a key installed, flip this and retry.
			g_ltkReverse = !g_ltkReverse;
			Serial.printf("# LEAN: link key byte order = %s\n",
				      g_ltkReverse ? "REVERSED" : "as-XORed (default)");
		} else if (c == 'S' || c == 's') {
			// NON-DESTRUCTIVE toggle readout. Added because there was no way to ASK the state:
			// '9' CYCLES the region, so checking it changed it, which silently walked the puck
			// through broken/perturbing regions mid-session. Reconstructing state from command
			// history also failed once, after a reflash quietly reset 4/5/6 to their defaults.
			Serial.printf("# LEAN toggles: 0 colors=%s | 1 unprompted=%s | 2 pages=%s | "
				      "3 CMD10[3]=0x%02X | 4 calib=%s | 5 0x13060=%s | 6 keymat=%s | "
				      "7 RSP_01_0C=%s | 8 templates=%s | 9 region=%u%s\n",
				      g_ownColors ? "OURS" : "orig", g_askUnprompted ? "ON" : "off",
				      g_ownFingerprint ? "OURS" : "clone", g_cmd10Disc,
				      g_ownCalib ? "OURS" : "clone", g_ownPage13060 ? "OURS" : "clone",
				      g_ownKeyMaterial ? "OURS" : "clone", g_ownMisc ? "OURS" : "clone",
				      g_ownTemplates ? "OURS" : "clone", g_infoRegion,
				      (g_poisonUntilMs && millis() < g_poisonUntilMs)
					      ? "  *** REFRESH POISON ACTIVE ***" : "");
			Serial.printf(
				"# LEAN status: connHdl=%u (%s) t=%lu ms\n",
				g_connHdl,
				g_connHdl == BLE_CONN_HANDLE_INVALID ? "disconnected" :
									"CONNECTED",
				(unsigned long)millis());
			// Build stamp: several sessions were spent unsure whether the board was running the
			// source in the tree. This makes that a one-keystroke question.
			Serial.printf("# LEAN build: " __DATE__ " " __TIME__ "\n");
			// Live counters, not just the disconnect summary -- reading them mid-connection does not
			// depend on catching the disconnect line.
			// QUEUED vs ACKED, same as the disconnect line -- "sent" only ever meant hvx() accepted
			// it, which says nothing about the peer receiving it (audit STALE-25).
			Serial.printf("# LEAN reports this connection: %lu QUEUED, %lu failed, %lu ACKED-on-air\n",
				      (unsigned long)g_reportsSent, (unsigned long)g_reportsFailed,
				      (unsigned long)g_txCompleteCount);
			printGattCheck();
			Serial.printf("# LEAN report chan: d5a9 cccd=0x%04X 679d-desc=0x%04X streaming=%d\n",
				      g_hD5a9Cccd, g_hD5a9Desc, g_streaming);
			// Advertising mode belongs in status: it decides whether the Change Grip/Order screen
			// can see us (PAIRING) or a sleeping console can (WAKE), and inferring it from a sniffer
			// capture is both slow and, as of today, was actively misleading.
			Serial.printf("# LEAN advertising: %s%s%s | addr %02X:%02X:%02X:%02X:%02X:%02X"
				      " | interval %u units (%u ms) ['[' to sweep]\n",
				      g_haveBondedHost ? "WAKE inviting " : "PAIRING (open)", "",
				      g_haveBondedHost ? "" : "", g_bleAddr[5], g_bleAddr[4], g_bleAddr[3],
				      g_bleAddr[2], g_bleAddr[1], g_bleAddr[0],
				      ADV_INTERVAL_UNITS[g_advIntervalIdx],
				      (unsigned)(ADV_INTERVAL_UNITS[g_advIntervalIdx] * 625UL / 1000));
			if (g_haveBondedHost)
				Serial.printf("# LEAN   bonded host: %02X:%02X:%02X:%02X:%02X:%02X\n",
					      g_bondedHost[5], g_bondedHost[4], g_bondedHost[3],
					      g_bondedHost[2], g_bondedHost[1], g_bondedHost[0]);
			Serial.printf(
				"# LEAN handles: input val=0x%04X cccd=0x%04X | cmdWrite val=0x%04X | cmdResp val=0x%04X cccd=0x%04X | vibration val=0x%04X\n",
				g_hInputVal, g_hInputCccd, g_hCmdWriteVal, g_hCmdRespVal,
				g_hCmdRespCccd, g_hVibrationVal);
		}
	}
}

void setup()
{
	armWatchdog();

	// Mirror OpenPuck's MODE_BLE_POC static-mount USB rebuild: detach, custom config buffer (NOT
	// clearConfiguration -- keeps CDC), WebUSB registered, attach.
	USBDevice.detach();
	delay(30);
	USBDevice.setConfigurationBuffer(g_usbCfgDesc, sizeof g_usbCfgDesc);
	usb_web.begin();
	USBDevice.setConfigurationAttribute(0x80 | 0x20);
	USBDevice.attach();

	Serial.begin(115200);
	for (int i = 0; i < 300 && !USBDevice.mounted(); i++)
		delay(10);

	InternalFS.begin();
	printLastStage(); // report the previous boot attempt's result, if any
	printLastPeerAddr(); // report a captured peer address from a prior connect, if any
	printCmdLog(); // report which handshake commands (if any) were seen last boot attempt
	printEvtLog(); // report connect/disconnect/CCCD-subscribe history from last boot attempt

	Serial.println(
		"# LEAN: setup() complete. Console commands:\n"
		"#   STATE:    S status | C forget bonded host (ONLY escape from the wake-mode dead end)\n"
		"#   IDENTITY: G cycle VID/PID | Z advert byte | Q bd283 | I serial+bd283\n"
		"#   LINK:     U PHY 2M/1M | O link-key byte order\n"
		"#   STREAM:   N stream forcing | V force stream now | M button mask | B button offset\n"
		"#             1 unprompted 0x01/0x0C after 0x0C/0x04 (default OFF -- drop suspect)\n"
		"#   DE-CLONE: 2 per-unit identity ours/cloned | 3 sweep CMD10[3] | 4 calibration page\n"
		"#             5 0x13060 echo probe | 6 KEY MATERIAL LTK1/LTK2 | 7 RSP_01_0C\n"
		"#             8 captured templates | 9 CONTROLLER_INFO region bisect (0=orig 5=all)\n"
		"#   INPUT:    H one-shot HOME | J hold HOME\n"
		"#   PROTO:    P announcement | A answer pending | X echo frame | R reply channel\n"
		"#             Y 0x0A-0x08 mode | K skip LTK2 | D discover Switch GATT\n"
		"#   DIAG:     T self-test | F format InternalFS\n"
		"#   LEGACY:   W wake sequence (wrong model) | L/E logs (persistence disabled).");
}

void loop()
{
	NRF_WDT->RR[0] = WDT_RR_RR_Reload;
	// STALL DETECTOR. The connections now die of supervision timeout (0x08), which only proves OUR link
	// layer stopped hearing the console -- it cannot distinguish "the console went quiet" from "our radio
	// missed its slots". The sniffer cannot settle it either; it keeps losing these long connections
	// before the drop. But we can test our own half directly: on this chip a flash write HALTS code
	// execution and can block the interrupts the radio depends on, and this firmware writes flash from
	// inside BLE callbacks (bond file, peer address, stage record). If that is starving the radio, the
	// main loop will stall for the same stretch -- so measure it. A stall of even a few hundred ms is
	// enough to miss the connection events that cause a supervision timeout.
	{
		static unsigned long lastLoopMs = 0;
		unsigned long nowMs = millis();
		if (lastLoopMs && (nowMs - lastLoopMs) > 50)
			Serial.printf("# LEAN *** MAIN LOOP STALLED %lu ms *** t=%lu\n",
				      (unsigned long)(nowMs - lastLoopMs), (unsigned long)nowMs);
		lastLoopMs = nowMs;
	}
	// WHICH CALL IS STALLING? The 84-85 ms blocks survived every explanation tried: they are not flash
	// (writeFileInPlace() now skips redundant writes and they continued), not serial volume (quiet mode
	// suppressed the heartbeat/report-head/mtu prints and they continued), and there is no delay() on
	// this path. Time the two candidates separately rather than guess a fourth time.
	unsigned long tA = millis();
	serialConsolePoll();
	unsigned long tB = millis();
	if (!s_bleInitDone) {
		s_bleInitDone = true;
		bleInitOnce();
	} else {
		bleHeartbeat();
	}
	unsigned long tC = millis();
	if (!g_quiet && ((tB - tA) > 40 || (tC - tB) > 40))
		Serial.printf("# LEAN *** STALL BREAKDOWN: serialConsolePoll=%lu ms bleHeartbeat=%lu ms ***\n",
			      (unsigned long)(tB - tA), (unsigned long)(tC - tB));
}
