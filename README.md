<p align="center">
  <img src="assets/pair-raught-banner.png" alt="Pair-Raught" width="520">
</p>

# Pair-Raught

Firmware for an nRF52840 that takes input from a Valve Steam Controller over Valve's own RF protocol and
presents it to a Nintendo Switch 2 as a Pro Controller over USB. It also wakes the console from sleep over
BLE.

*Raught* is the past tense of *reach*. It reaches a console that is asleep, and it reaches across two
systems that were never meant to touch.

Pair-Raught is an independent project. It is not affiliated with, endorsed by or sponsored by Nintendo or
Valve. Nintendo Switch, Joy-Con, Valve, Steam and Steam Controller are trademarks of their respective owners.

## What it does

**Steam Controller in, Pro Controller out.** The controller pairs to the puck the way it pairs to Valve's
own dongle, over RF. The console sees a Pro Controller on USB, with buttons, both sticks, motion, and
rumble.

**Wakes a sleeping console.** Hold the QAM button (Quick Access Menu), the one with three dots between
the trackpads. The puck hands its radio to BLE, wakes the console, and hands it straight back to RF.
Waking is the point of the BLE support; it is not a general BLE controller mode.

**Chimes on the controller.** A short melody plays before a wake, and another before the puck reboots into
a different USB identity, so a button held for seconds is not answered with silence. They play through the
controller's own haptics: every note's pitch, length and haptic, and the volume of all of them, are set on
the config page, and each chime can be turned off.

**Two USB identities.** Pro Controller by default, so plugging into a console works with nothing to
configure. Back 4 + L1 + R1 switches to the Steam dongle identity, which is how a controller gets paired to
the puck through Steam in the first place.

**Configures itself.** In dongle mode the puck appears as a small read-only USB drive holding a config
page. Open it in Chrome or Edge and it talks back to the puck over WebSerial. Nothing to install, and no
copy of the page that can disagree with the firmware behind it.

**Motion, rumble, and a trackpad mouse.** The IMU reaches the console and its calibration survives a power
cycle. Rumble arrives from the console over USB and is relayed to the controller over RF. The right
trackpad drives a USB mouse, though no context has yet been found where the console acts on it.

## What you need

**An nRF52840 Pro Micro board.** This is the one used to build and test everything here:
[nRF52840 Pro Micro](https://www.amazon.com/dp/B0CYLNZ6V4). Anything that presents a nice!nano compatible
UF2 bootloader should work, but nothing else has been tried. Check the SoftDevice before you flash, see
the next section.

**A USB cable that carries data.** Plenty of cables sold with small electronics are charge only. If the
board never appears as a drive or a serial port on any port you try, suspect the cable before the board.

**A Steam Controller (2026)**, and **Steam on a PC** to pair it to the puck. Pairing happens over USB with both
plugged into the same machine; there is no over-the-air pairing.

**Chrome or Edge** for the config page. It uses WebSerial, which Firefox and Safari do not implement
natively. There are plugins for Firefox that can allow that browser to be used.

**A Switch 2**, for the half of this that talks to a console.

## Before you flash: check the SoftDevice

**Do this first.** It is the difference between a board that works and one that looks dead, and there is
no error message anywhere to tell you which you have.

Plug the board in. A new one has no firmware, so it comes up in its bootloader and appears as a drive
called `NICENANO`. Open `INFO_UF2.TXT` on it and read the SoftDevice line:

    SoftDevice: S140 version 6.1.1      good, carry on
    SoftDevice: not found               stop, see "Restoring the SoftDevice" below

The firmware does not use the SoftDevice. It runs Zephyr's own link layer. What matters is that the
bootloader decides where the application starts based on whether a SoftDevice is sitting at `0x1000`.
With one present, applications start at `0x26000`, where this firmware is built to run. With none,
they start at `0x1000`, and an image built for `0x26000` lands a quarter of a megabyte away from where it
expects to be, faults on its first call, and reboots forever.

## Flashing

1. Plug the board in. A board with nothing installed comes up in its bootloader and appears as a drive
   called `NICENANO`. If no drive appears, see "Getting into the bootloader" below.
2. Copy `switch2_ll.uf2` onto that drive.
3. The drive disappears on its own within a couple of seconds and the board reboots as the puck. A drive
   called `PAIR-RAUGHT` takes its place.

That is the whole thing. No tools to install.

<details>
<summary><b>What Windows says about the copy</b></summary>

Go by the drive, not the dialog.

**This error means it worked:**

    Error 0x800701B1: A device which does not exist was specified.

The board takes the last block, flashes it, and reboots immediately, so the drive is gone before Windows
can finish the copy it thinks is still running. Nothing is wrong. Click Cancel.

**This error means you copied onto the wrong drive:**

    There is not enough space on the disk.

A puck already running this firmware presents its own drive, `PAIR-RAUGHT`, which holds the config page
and is far too small for a firmware image. It can take the same drive letter the bootloader had, so check
the volume name before copying: `NICENANO` is the bootloader and takes firmware,
`PAIR-RAUGHT` is the running puck and does not.

**No error, and the drive still sitting there after a few seconds, means it did not work.** Windows can
report a copy to a removable drive as finished while the tail of the file is still in its write cache,
and the bootloader waits for every block before it does anything. Eject the drive to force the cache out,
or copy the file again.

Either way, do not unplug the board to find out. A partly written application runs and faults, which
looks exactly like a dead board.

</details>

<details>
<summary><b>Getting into the bootloader</b></summary>

A board only lands in the bootloader by itself when it has no application to run. If yours arrived with
firmware on it, or you have flashed it before, it boots that instead and no drive appears.

**Reset it twice in quick succession**, roughly as fast as a mouse double-click. The drive appears and
stays put, waiting.

Most of these boards have no reset button. They have two exposed contacts marked **RST** and **GND**, and
you reset the board by briefly bridging them, with tweezers, a paperclip, or a scrap of wire. So the
sequence is: touch the two contacts, break contact, touch them again, all inside about half a second.

Too slow and the board just restarts into whatever it was running, so try again. Nothing is harmed by
getting it wrong.

If your board does have a button, press it twice the same way.

Check the LED to see which state you are in. A **slow pulse** means the bootloader is waiting for a file.
A **fast pulse** means an application is running and faulting, which is covered further down.

</details>

<details>
<summary><b>Reflashing</b></summary>

The same three steps. Reset the board twice as above, the bootloader drive comes back, and you copy the
new `switch2_ll.uf2` onto it.

</details>

<details>
<summary><b>Reading the LED</b></summary>

    slow pulse    bootloader, waiting for a UF2
    fast pulse    the application is faulting and resetting

A board that flashes successfully and then fast-pulses forever is almost always the SoftDevice problem
above. Enough rapid resets will trip the bootloader's double-reset detection and drop it into the
bootloader on its own, which makes it look like it went there deliberately.

</details>

## Setting up

Do these in order. The puck arrives from a fresh flash in Steam dongle mode, which is where pairing a
controller has to happen.

### 1. Pair the Steam Controller

Pairing is done by Steam over USB. There is no over-the-air pairing, so both the puck and the controller
have to be plugged into the same PC.

**If the controller is already paired to an official Valve puck, use its second slot.** The Steam
Controller holds two RF pairings. A controller that has been paired to a Valve puck is using the first
one, and pairing again without moving to the second will take that pairing away.

Slots are chosen at power-on, not with the controller running:

1. Turn the controller off. Steam + Y does it, held for a couple of seconds if Steam is not running.
   Holding the Steam button on its own also gets there eventually.
2. Hold **L1 + A + Steam** to turn it back on. That selects the second slot. **R1 + A + Steam** selects
   the first.
3. The start-up chime is different, which is how you know it took.

Then pair:

1. Plug the puck into the PC. On a fresh flash it is already in dongle mode.
2. Plug the Steam Controller into the same PC.
3. In Steam, open the controller settings and start the pairing flow for a new controller.
4. Follow it through. Steam writes the bond to the puck and matches the controller to it by serial
   number.

**A "Pairing Failed (16)" message does not mean pairing failed.** Steam writes the bond first and then
waits for the controller to answer over the radio. The error is that wait timing out, and the bond is
already on the puck by then. Unplug the controller, power it on away from the PC, and see whether the
puck picks it up before pairing again.

### 2. Switch to Pro Controller mode

The puck has two USB identities. Dongle mode is for pairing through Steam. A console understands Pro
Controller mode.

Hold the **four back paddles + L1 + R1** on the Steam Controller. The puck reboots into Pro Controller mode
and stays there on every power-up from then on.

### 3. Connect to the Switch 2

Plug the puck into a USB port on the console **that supplies power**. The dock's ports do. The port on
the console itself may not, depending on how it is being used.

The console should show a Pro Controller. Buttons, both sticks, motion and rumble all work from here.

### 4. Register with the console, for waking

Only needed if you want the puck to wake the console from sleep. Skip it if you just want a wired pad.

Registration happens over Bluetooth, and the puck has one radio, so it borrows it from RF for as long as
this takes and hands it back afterwards. The controller goes quiet meanwhile.

**At the console**, with the puck plugged in and the controller in your hands:

1. On the console, open **Change Grip/Order**.
2. Hold the **four back paddles + L3 + R3** (both stick clicks) on the Steam Controller.
3. Wait for the puck to appear and be registered. It returns to RF on its own, and gives up after two
   minutes if no console registers it.

**From a PC** instead, if you would rather: press **Pair with a console** on the config page, which
reboots the puck into pairing mode, then open Change Grip/Order. The console has to be in Bluetooth range
of the PC for this to work, which is why the chord exists.

Once registered, holding QAM wakes the console from sleep.

<details>
<summary><b>Configuring</b></summary>

Open `RAUGHT.HTM` from the puck's own drive in Chrome or Edge. It talks back over WebSerial, so nothing
needs installing, and the page can never be out of step with the firmware serving it.

The drive is only present in dongle mode. In Pro Controller mode the puck does not offer one, because a
console has no use for a drive and adding interfaces to a working Pro Controller is a risk with nothing
to gain.

</details>

<details>
<summary><b>Starting over</b></summary>

The config page has a **Factory reset** button. It erases everything the puck remembers: the paired
controller, the registered console, the settings and the motion calibration. Use it for a bad pairing, or
before passing the puck to someone else.

Reflashing does **not** do this. The application and the stored data live in different parts of flash, and
the bootloader will not write to the part holding the data, so a puck reflashed a dozen times still knows
everything it ever knew.

</details>

<details>
<summary><b>Restoring the SoftDevice</b></summary>

Needed only if `INFO_UF2.TXT` says `SoftDevice: not found`. No debug probe required.

Serial DFU cannot do this. `adafruit-nrfutil` checks bootloader versions, and its SoftDevice packages
target Adafruit's 0.9.1 while a nice!nano runs 0.6.0, so it refuses. UF2 can, because with no SoftDevice
installed the bootloader's writable window starts at `0x1000`, which is exactly where the SoftDevice
goes.

You need an S140 6.1.1 image as a UF2 targeting `0x1000`. If you have the Adafruit nRF52 Arduino core
installed, one can be carved out of any of its combined bootloader hexes: the SoftDevice occupies
`0x1000` up to about `0x25E00`, with the MBR below it and the bootloader far above at `0xF4000`.

Copy that UF2 onto the bootloader drive, then check `INFO_UF2.TXT` again. It should now report the
SoftDevice, and flashing works normally from there.

Only `0x1000..0x26000` is written. The bootloader and the MBR are untouched, so a failed attempt leaves
the board no worse off than it already is.

</details>

<details>
<summary><b>Building</b></summary>

    ./build.sh

Needs:

- the nRF Connect SDK v3.4.0, and `nrfutil` with its `toolchain-manager` command
- bash, which on Windows means Git Bash
- `xxd`, which ships with Git for Windows and with vim, and on some Linux systems is a separate package
- optionally `adafruit-nrfutil`, which lets the build also produce a serial-DFU `.zip`

`build.sh` looks for the SDK at `C:/ncs/v3.4.0` on Windows and `~/ncs/v3.4.0` on Linux and macOS. If
yours is somewhere else, point `NCS_WORKSPACE` at it. The build works in a `pair-raught` folder next to
the SDK, which it clears on every run.

    NCS_WORKSPACE=/path/to/ncs/v3.4.0 ./build.sh

The output is `switch2_ll/switch2_ll.uf2`, which is the file you copy onto the board.

Before the first build, apply the patch to Zephyr's software link layer in the SDK. The build does not
do this for you, and it only needs doing once per SDK install. Without it the console never completes a
connection. "The BLE protocol" below explains why.

    cd <nRF Connect SDK>/zephyr
    git apply <path to this repo>/patches/ull_peripheral-accept-5ms.patch

</details>

<details>
<summary><b>The BLE protocol</b></summary>

The console opens controller links at a 5 ms interval, below what the Bluetooth spec allows, so a
closed-source link layer refuses silently and reports nothing. That is why this is built on Zephyr's
software controller rather than a vendor stack, and it is most of the reason the work took as long as it
did.

</details>

<details>
<summary><b>reference/</b></summary>

`BleSwitch2Poc.ino` is where the BLE work started, on Arduino and Nordic's SoftDevice. It is not built and
should not be revived, because that stack cannot do the 5 ms connection. It is kept because the firmware
here was ported from it, and its comments record what was measured and when. When doing a step here, read
the SoftDevice-era note for the same step first and carry the reasons across.

Comments here and in both copies of `motion_replay.h` cite capture files and scripts under
`docs/captures/`. Those were working material and are not part of this repository.

</details>

<details>
<summary><b>Relationship to OpenPuck</b></summary>

This is not a fork of [OpenPuck](https://github.com/safijari/openpuck), and it is not a drop-in
replacement for it. It is separate firmware, written for Zephyr, and it would not exist without that
project.

OpenPuck is where the Steam Controller side of this comes from. Its Arduino firmware worked out Valve's
RF protocol, the dongle's HID interfaces, the haptics framing and the Pro Controller mapping, and parts
of this firmware were ported from it rather than worked out again. Files say what they were ported from
where it matters, for example `rf_radio.h`, `rf_link.h`, `puck_input.h` and `puck_hid.c`.

What is new here is the Switch 2 side: the BLE work, the wake, the config surface, and the move to a link
layer whose source can be patched.

If you want a Steam Controller dongle, use OpenPuck. This is for driving a Switch 2 with one.

</details>

<details>
<summary><b>The chimes, and SteamHapticsSinger</b></summary>

The controller plays tones because of
[SteamHapticsSinger](https://github.com/CrazyCritic89/SteamHapticsSinger), which plays MIDI files on a
Steam Controller's haptics and supports the 2026 controller through a puck. Reading it is where the
command came from: output report 0x83, carrying a haptic, a gain and a frequency in hertz. Without it
the search would have ended at Steam's vibration test, which only picks a preset.

No code was taken from it. It is BSD-3-clause; the credit is owed rather than required.

</details>

<details>
<summary><b>How this was written</b></summary>

Claude, Anthropic's model, assisted with this project, including this firmware.

Every behavior claimed here was tested on real hardware. The puck was flashed and run against a real
Switch 2 and a real Steam Controller, and the protocol findings came from radio captures of both. Where
something is believed but not demonstrated, it is written down as unverified rather than claimed.

</details>

<details>
<summary><b>License</b></summary>

AGPL-3.0-only. See `LICENSE`.

Parts of this firmware were ported from OpenPuck, which is AGPL-3.0, so this carries the same license.
That means you are free to use, change and redistribute it, and that anything you distribute built on it
has to be open under the same terms.

</details>
