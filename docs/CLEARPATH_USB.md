# ClearPath MC/SD USB link

What we know about the ClearPath MC/SD USB port, the port ClearPath-MSP uses. `host/clearpath_usb.py` reads a motor over this link.

## Sources

Everything here comes from one of two places:

- **Public SDK.** Teknic's sFoundation SDK, downloadable from teknic.com, defines the packet framing, the command numbers, and the parameter option bit.
- **USB captures.** We recorded MSP 2.0.39 talking to a CPM-SDSK-2310S on our bench (2026-10-30) with USBPcap and Wireshark. MSP was driven through its normal dialogs. Each parameter below lists the capture evidence that ties it to what MSP displayed or wrote.

Nothing here comes from Teknic's closed binaries.

## Transport

- **Device.** The motor enumerates as USB `2890:0201`. Windows binds Teknic's port driver, and the motor appears as a COM port.
- **Endpoints.** Bulk OUT `0x01` and bulk IN `0x81` carry the node protocol bytes unchanged.
- **Baud rate.** No CDC line-coding requests appear in any capture. Our own client talks at 9600 baud.

## Framing

From the public SDK (`pubMnNetDef.h`, `SerialEx.cpp`). Across all captures (more than 100,000 packets) no packet failed its checksum.

| Byte | Contents |
|------|----------|
| 0 | `0x80` start of packet, packet type in bits 4–6, node address in bits 0–3 |
| 1 | payload length in link characters (bits 0–4), mode (bit 5), source (bit 6) |
| 2.. | payload, packed 7 bits per character, least significant group first |
| last | checksum: the 7-bit sum of all bytes is 0 |

Replies to commands have packet type 1 (`0x90` for node 0). A write is acknowledged with the empty reply `90 00 70`.

Link behaviour we found by testing:

- **Serial break.** A break on the link latches alert bit 28 (EStopped) on our MC/SD. The SDK uses a break to reset the link rate. MSP does not send one, and neither should a monitor tool. To clear the bit, send cmd 4 with data `18`.
- **Stuck parser.** A malformed packet can leave the node ignoring further packets until it receives MSP's link-reset pair: type 3 and type 7 packets, each with payload `00`. MSP sends this pair first on every connect.
- **Bad writes.** A monitor write with one byte too many was refused with a 1-character reply instead of the empty acknowledgement, and the setting was not changed.

## Commands seen

| Command | Meaning | Evidence |
|---------|---------|----------|
| type 0, cmd 0 / 2 / 11 / 13 | get parameter, banks 0–3 | Public SDK names. Thousands of reads in every capture. |
| type 0, cmd 3 | set parameter, bank 1 | Captured when MSP's Tracking Error Limit dialog was OK'd (see below). |
| parameter index bit `0x80` | non-volatile copy | Public SDK `cpmNonVol()`. MSP writes each setting twice, live then non-volatile. |
| type 5, no payload (`d0 00 30`) | set address | Sent at connect. The reply's address field is 1, so one node is on the link. |
| type 0, cmd 5 | keepalive | Public SDK name `MN_CMD_NET_ACCESS`. MSP sends it about 3 times a second; it always returns 72. |
| type 3, payload `03 nn` | heartbeat | `nn` counts up by one about 5 times a second. |
| type 3 / type 7, payload `00` | link reset | First packets after the port opens; the node echoes them. |
| type 7, payload `09 nn` | change link rate to `nn` × 9600 baud | Public SDK `infcChangeLinkSpeed`. MSP asks for `0C` (115200), then `18` (230400). The node echoes at the old rate, then switches. `host/clearpath_usb.py` uses `18` to stream and `01` to return to 9600. |
| type 0, cmd 4, data `18` | clear the node's EStopped alert | Public SDK `netNodeStop` with `STOP_TYPE_CLR_ESTOP`. It cleared alert bit 28 on our motor. |
| type 0, cmd 0x16 / 0x1E | monitor port 0 / 1: scope channel variable, gain, and filter | Public SDK names and layout. See "Choosing the variable". |
| type 0, cmd 0x19 | scope start (`03`) and stop (`00`) | See "Scope stream". |
| type 3, node-originated, 7-byte payload | scope sample | See "Scope stream". |

## Scope stream

MSP's scope makes the motor stream samples without being asked. These findings come from one capture. Channel 1 was set to "Velocity, Commanded", channel 2 was off, and M0 ran known constant-speed moves at −240 and +480 RPM, as shown in MSP.

- **Start and stop.** Type 0, cmd `0x19`. Data `03` starts the stream with channel 1 only, `04` starts it with both channels, and `00` stops it. MSP sends `00` twice when the scope closes. Command `0x19` is not one of the SDK's common commands.
- **Samples.** While the scope is open, the node sends type 3 packets with the source bit set (node-originated). Each has 8 link characters, which is 7 bytes. We saw about 845 per second, which is one every 4 servo samples. Every one had a valid checksum.
- **One value per sample.** Each sample carries one monitor value in bytes 2–3. With only channel 1 on, it is channel 1's variable. With both channels on, it was channel 2's variable. We have not found where channel 1 travels in two-channel mode.

### Choosing the variable

MSP selects each channel's variable with the monitor port commands: `0x16` for channel 1 and `0x1E` for channel 2. The SDK names these `MN_CMD_GET_SET_MON0` and `MN_CMD_GET_SET_MON1`.

- **Read.** The command byte alone reads the channel.
- **Write.** The command byte plus 9 bytes of the SDK's `iscMonNodeState` (`pubIscRegs.h`): a 16-bit variable, a 32-bit gain, a 16-bit filter, and the low byte of tune-sync. The SDK's `cpmSetMonitor` uses a packet length of 10, which cuts tune-sync to one byte. The node acknowledges with an empty reply.
- **Without the save flag.** `host/clearpath_usb.py scope` writes without bit `0x4000`, so only the working setting changes. It puts the original back on exit.
- **Units.** The SDK's `convertMonGain` gives full scale for each variable from the gain. Value = raw / 65,536 × full scale. This reproduced 240 RPM exactly for commanded velocity.
- **Gain.** The gain follows MSP's vertical scale for the channel. Halving the scale (12.5 to 25 %/div) halved the gain.
- **Saved to the motor.** MSP sets bit `0x4000` on the variable, which the SDK defines as `MON_SAVE_NV_MASK` ("save in Non-Volatile"). So changing a scope channel in MSP rewrites the motor's stored monitor setting.

On our motor at connect, before we changed anything, channel 1 was variable 3 with gain `0x00206C3C` and channel 2 was variable 35 with gain `0x00033F36`, both with filter 0. We restored both exactly and confirmed the restore in the capture.

Every variable MSP offers maps to the SDK's `monTestPoints` enum (`pubMonPort.h`). We captured all 13 being written, and moved the motor while each was selected:

| MSP name | ID | SDK name | Bytes 2–3 during a 20 mm move |
|----------|----|----------|-------------------------------|
| Position Error | 27 | `MON_POSN_DIR_TRK` | −929 to +1216 |
| Velocity, Commanded | 3 | `MON_VEL_CMD` | −1953 to +3937 (8.136 per RPM at MSP's 500 RPM/div gain) |
| Torque, Measured | 35 | `MON_TRQ_MEAS_PEAK` | −801 to +960 |
| Torque Utilization | 39 | `MON_TRQ_CMD_UTIL` | 0 to 416 |
| Velocity Error (Gated) | 94 | `MON_VEL_ERR_GATED` | −32 to +31 |
| Bus Voltage | 36 | `MON_BUS_VOLTS` | flat at about 12,197 |
| Max Phase Voltage | 23 | `MON_VOLTS_MAX` | 0 to 1728 |
| Torque, Commanded | 8 | `MON_TRQ_CMD` | −353 to +448 |
| Position, Measured | 18 | `MON_POSN_MEAS` | ramps with the move |
| Acceleration, Commanded | 21 | `MON_ACC_CMD` | about ±32,000 at the start and stop of each move |
| Jerk, Commanded | 20 | `MON_JRK_CMD` | −7232 to +6207 |
| Torque Error (peak) | 34 | `MON_TRQ_TRK_PEAK` | −1025 to +832 |
| Velocity, Measured | 2 | `MON_VEL_MEAS` | −1953 to +3937 |

The values depend on each channel's gain. Only velocity has been scaled to physical units.

| Byte | Contents | Evidence |
|------|----------|----------|
| 0 | always `01` | Constant across every state. |
| 1, bits 0–1 | sample counter, counting 0 to 3 and wrapping | Consecutive samples count 0, 1, 2, 3, 0. The upper bits change with motion and are not decoded. |
| 2–3 | monitor value, signed 16-bit. See "Choosing the variable". | With commanded velocity selected, medians of −1953 at −240 RPM and +3905 at +480 RPM, 0 at rest. That is counts per servo sample × 256, which puts the sample period at about 298 µs. |
| 4 | state flags | `00` enabled at rest, `c1` at −240 RPM, `c3` at +480 RPM, `e0` disabled. Not decoded. |
| 5 | state flags | Equal to byte 4 with one channel on. Constant `e0` with both channels on. Not decoded. |
| 6 | digital lanes; both nibbles are always equal | Bit 3 is high when enabled at rest, low when disabled, and toggles every sample while moving, like the HLFB output (MSP's "out" lane). Bit 0 is 0 only when disabled, like the enable input. Other bits are not assigned. |

## Parameters

A parameter number is `bank << 8 | index`. Values are little-endian.

### Confirmed

| Param | Meaning | Evidence |
|-------|---------|----------|
| 0x053 | measured position, counts (signed, 3 bytes) | MSP showed +2,945 while 0x053 read 2945. A 10 mm move (2 revolutions) changed it by −12,801 on both repeats; Reverse Direction is checked in MSP. |
| 0x054 | commanded position, counts | Equal to 0x053 at rest, read at connect. |
| 0x110 | tracking error limit, counts (4 bytes) | MSP wrote 0x110 = 9600, then 6400, when its Tracking Error Limit dialog was OK'd with those values. Each write was followed by the same value on the non-volatile copy. |
| 0x123 | drive temperature, °C | Polled only while Power and Temperature was open. Read 44–45 while MSP showed 45. |
| 0x26D | maximum drive temperature, °C | Read when Power and Temperature opened. Value 83; MSP showed 83. |
| 0x232 | logic reset voltage, V × 32 | Read when Power and Temperature opened. Value 560 = 17.5 × 32; MSP's text says 17.5 V. |
| 0x05B | in-range position window, counts | Read when the HLFB dialog opened. Value 10; MSP showed 10. |
| 0x140 | input resolution, steps per revolution | The only parameter read at connect with value 800; MSP showed 800. |
| 0x055, 0x056 | velocity (signed, 4 bytes), about 16,555 per RPM | During constant-speed moves MSP showed −240 RPM while both read about −3,974,000, and +480 RPM while both read about +7,946,000. That is a 2:1 ratio with matching signs. These are the two velocity registers, but this data does not say which is measured and which is commanded. |
| 0x05C | move-done verify time, servo samples | Read when the HLFB dialog opened. Value 34 while MSP showed 10.1 ms, which gives 297 µs per sample. |

The two velocity scales agree on the servo sample period. 16,555 per RPM equals counts per sample × 2¹⁹ at a period of about 296 µs, and 34 samples for 10.1 ms gives 297 µs.

### Candidates

| Param | Likely meaning | Why not confirmed |
|-------|----------------|-------------------|
| 0x116, 0x118 | bus voltage | Both read about 20,333 while MSP showed 74.5 V. That fits 120 V full scale, but this is one data point and there are two candidates. |
| 0x052, 0x062 | RMS or torque level | Polled continuously. Both rise with speed: 0x052 had medians of 187 at rest, 321 at 240 RPM, and 560 at 480 RPM; 0x062 had 53, 57, and 65. MSP showed 1–2% RMS, which is too coarse to fit a scale. |
| 0x155 | in-range velocity window, % | Value 3 matches MSP, but small values recur across many parameters. |
| 0x009, 0x00D, 0x010, 0x019 | alert, status (accumulating), status (real-time), warning | Polled continuously, with register sizes 12, 6, 6, and 12 bytes. Bit meanings for MC/SD are not established by capture. |

## Not decoded yet

- Where channel 1's value travels when both scope channels are on.
- How to convert each monitor gain to physical units. The SDK's `convertMonGain()` is the starting point. Only velocity is scaled so far.
- Which status and alert bits mean what on MC/SD. In the real-time status register (0x010), bits 2, 13, 22, and 30 are set while disabled and clear on enable. This held for every enable and disable we captured (disabled `0x405ca2280c`, enabled at rest `0x401ce20808`). Other bits change during motion. Naming them needs more known events, such as a limit or a deliberate fault.
- The alert register (0x009) stayed 0 through every capture, including enables, disables, watchdog stops, and estops from the ClearCore. No alert bit has been observed set yet.
- Settings MSP reads only at connect. More stimulus writes resolve these one at a time: change the value in MSP, capture, restore.
