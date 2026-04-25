# UDS Client for VW MQB Gateway (J533 / ECU 19)

A C++17 SocketCAN UDS client designed for Raspberry Pi + MCP2515 hardware, with
ISO 15765-2 (ISO-TP) multi-frame handling, Security Access, Read/Write Data By
Identifier, and two canned MQB adaptations:

1. **Disable Start-Stop** (Gateway long coding at DID `0x0600`)
2. **Fix "Emergency Call Function Impaired"** — remove Module `0x75`
   (Telematics / OCU) from the Installation List at DID `0x0608`

> **New to all of this? Read `GUIDE.md` first.**
> It explains every acronym (ISO-TP, UDS, DID, NRC, SID, CAN frame…) in
> plain English with JS/Python analogies, shows the exact bytes that go on
> the wire for each command, and walks through what each bitmask is doing.
> Once you've skimmed it, the hex constants in the source files will make
> sense.

## ⚠️ READ THIS FIRST

Writing the wrong bytes to a Gateway **can brick your car**. The DID numbers,
byte offsets, and Security Access algorithm used by this code are generic MQB
defaults. Variants of the J533 Gateway (hardware revisions, SW versions,
regional builds) use slightly different layouts.

**Mandatory workflow for any first-time use on a new vehicle:**

1. Battery maintainer connected. Engine off. No other diag tool talking to the
   car (unplug the dealer tool, close VCDS/OBDeleven/Carista, etc.).
2. `uds_main ident` — confirm VIN, part number (`F187`) and SW version
   (`F189`). Compare against VCDS label files / ODIS / your notes for **your
   exact Gateway part number**.
3. **Take a backup** that captures identity + coding + installation list in a
   single human-readable file:
   ```
   ./build/uds_main backup pre-change-$(date +%F).txt
   ```
4. Run the target command in **dry-run mode first** (`uds_main fix-ecall`
   without `--apply`). Verify the before/after diff printed is exactly what you
   intend, and nothing else has changed.
5. Only then re-run with `--apply --backup pre-change-<date>.txt`. The tool
   will:
   - refuse to write if the backup file is missing or mismatches the live ECU,
   - perform the write,
   - **immediately read the DID back** and compare byte-for-byte,
   - fail loudly if the readback doesn't match what was written.
6. If anything ever looks wrong after a write, restore from the backup:
   ```
   ./build/uds_main --apply restore pre-change-<date>.txt
   ```

See the `GUIDE.md` section **"Will this brick my car?"** for the full
explanation of the safety guarantees, the bricking failure modes, and the
recovery procedure.

If Security Access fails (`NRC 0x35 invalidKey` or `0x33 securityAccessDenied`)
it means your Gateway does **not** accept the static legacy login and you need
a real seed→key algorithm. See `uds/vw_mqb.h` for options and plug a proper
function into `UdsClient::securityAccess`.

## Build

On the Raspberry Pi:

```bash
make uds
build/uds_main --help
```

On macOS (for syntax checks only — the resulting binary cannot open a CAN
socket):

```bash
g++ -std=c++17 -Iuds -Itools/linux_stubs \
    uds/iso_tp.cpp uds/uds_client.cpp uds/vw_mqb.cpp uds/main.cpp \
    -o /tmp/uds_main_check
```

For a full real-Linux build + automated ISO-TP / UDS loopback test on any
host with Docker (including macOS with Docker Desktop):

```bash
docker build -f Dockerfile.build -t elm-uds-build .
```

The Dockerfile builds both binaries with `-Wall -Wextra -Wpedantic`, then runs
`tests/isotp_loopback` which spins up a fake ECU thread over an `AF_UNIX`
`SOCK_SEQPACKET` socketpair and exercises the full stack:

- Single Frame request + Single Frame response
- Single Frame request + First Frame + Consecutive Frames response (VIN)
- Single Frame request + First Frame + many CF response (128-byte list)
- Security Access all-zero-seed "already unlocked" fast path
- First Frame + CF request + Single Frame response (100-byte WDBI payload)

Expected tail of the build log: `OVERALL: PASS (0 failures)`.

## CAN bus prep (Raspberry Pi)

```bash
sudo ip link set can0 up type can bitrate 500000
sudo ip link set can0 txqueuelen 1000
ip -details link show can0
```

## Typical session

```bash
# 1. Identify the Gateway
build/uds_main ident

# 2. Back up the coding + installation list (DRY-RUN-safe, read-only)
build/uds_main dump-coding | tee coding.before.txt
build/uds_main dump-list   | tee list.before.txt

# 3. Preview the ecall fix without writing
build/uds_main fix-ecall

# 4. Apply it (requires working Security Access)
build/uds_main --apply fix-ecall

# 5. Ditto for start-stop -- verify the byte index + mask first!
build/uds_main disable-ss
build/uds_main --apply disable-ss
```

## Project layout

```
uds/
  hex_utils.h      -- hex helpers
  iso_tp.{h,cpp}   -- ISO 15765-2 transport (SF/FF/CF/FC, padded, 29-bit)
  uds_client.{h,cpp} -- Services 0x10, 0x22, 0x27, 0x2E, 0x3E; NRC decoding
  vw_mqb.{h,cpp}   -- VW/MQB DIDs, seed-key stubs, task helpers
  main.cpp         -- CLI (safe by default; needs --apply to write)
tools/linux_stubs/ -- Linux header stubs for host-only syntax checks on macOS
```

The existing OBD-II reader (`advanced_smart_reader.cpp` + `obd2_parser.*`)
is untouched; `make obd2` still builds it as before.

## Integrating with NestJS later

`UdsClient` has no global state and holds a plain `int` SocketCAN fd, so it is
trivial to wrap behind any IPC layer (DBus, a local Unix socket, or a small C
shim called from Node.js via `node-addon-api`). A realistic layout is:

- **C++ daemon** owns `can0` and runs `UdsClient`, exposes a JSON-over-WS or
  local socket interface.
- **NestJS gateway** connects to that local socket and relays sanitized,
  rate-limited commands from its own WebSocket clients. Never let the
  outside world directly issue `WriteDataByIdentifier` — gate every write
  behind an allow-list of `(DID, expected-shape)` on the NestJS side.

## Known limitations / honest disclaimers

- The seed→key function for VW Gateways is proprietary and varies by SW. The
  bundled `keyFromStaticLogin` covers only the very old legacy case that some
  pre-MQB and a few early MQB Gateways still accept. Modern Gateways will
  reject it.
- DID `0x0608` and `0x0600` are the standardised VW identifiers for Coding
  and Installation List, but individual Gateways may expose them through
  different routines. If `read-did 0608` returns `NRC 0x31 requestOutOfRange`
  on your car, consult your Gateway's VCDS label file for the right DID.
- The Start-Stop helper touches the *Gateway* coding. Many guides recommend
  disabling Start-Stop at the *Engine ECU (address 01)* adaptation instead,
  which is often more robust across firmware updates. Use whichever you have
  documented evidence for.
