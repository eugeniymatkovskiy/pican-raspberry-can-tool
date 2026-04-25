# A Gentle Guide to CAN / ISO-TP / UDS for JS & Python Devs

This document explains what the code in `uds/` is actually doing, in language
aimed at a JS or Python developer who hasn't spent years elbow-deep in
automotive protocols. If you've written a `fetch()` call or a Flask endpoint,
you have enough background — the rest is just *different vocabulary for very
similar ideas*.

---

## 1. The big picture in one diagram

When you open a socket to the car and ask "what's the VIN?", five different
protocol layers are active at once. From the lowest (wires) up to the highest
(your code):

```
  Your C++ code       -->  "give me DID 0xF190 (the VIN)"
  ---------------------------------------------------------
  UDS (ISO 14229)     -->  service 0x22  +  identifier 0xF190
  ---------------------------------------------------------
  ISO-TP (ISO 15765-2)-->  chops the UDS message into 8-byte chunks
  ---------------------------------------------------------
  SocketCAN (Linux)   -->  one syscall = one 8-byte CAN frame
  ---------------------------------------------------------
  CAN bus (hardware)  -->  two twisted-pair wires (OBD-II pins 6 & 14)
```

Compare this to a web request, which also has layers:

```
  Your JS code        -->  fetch('/api/users/123')
  ---------------------------------------------------------
  HTTP                -->  GET /api/users/123 + headers
  ---------------------------------------------------------
  TCP                 -->  reliable stream, retransmits, acks
  ---------------------------------------------------------
  IP                  -->  routes packets host-to-host
  ---------------------------------------------------------
  Ethernet / Wi-Fi    -->  actual wires / radio
```

The mapping is almost 1:1:

| Web layer  | Automotive layer | Role                                          |
|------------|------------------|-----------------------------------------------|
| Ethernet   | CAN bus          | Physical wires                                |
| IP         | CAN ID           | Addressing — who should receive this frame    |
| TCP        | ISO-TP           | Cut big things into small things and reassemble |
| HTTP       | UDS              | Request/response semantics, error codes       |
| REST paths | DIDs             | 16-bit numbers that identify a resource       |

---

## 2. Bytes, hex and bitmasks, 60-second refresher

A **byte** is 8 bits, so it holds a number from 0 to 255. In every automotive
doc (and this code) bytes are written in **hexadecimal** because each hex
digit maps to exactly 4 bits:

```
  binary  0000  0001  0010  0011  0100  0101  0110  0111
  hex     0     1     2     3     4     5     6     7

  binary  1000  1001  1010  1011  1100  1101  1110  1111
  hex     8     9     A     B     C     D     E     F
```

So one byte = two hex digits. `0x7F` is just `127` in decimal, and `0x10`
is `16`.

**A "nibble" is 4 bits — the two halves of a byte.** ISO-TP uses this a lot:
the *high* nibble of the first byte tells you what kind of frame it is, and
the *low* nibble often carries a length or a sequence counter.

Bit operations look *identical* in C++, JS, and Python:

```cpp
uint8_t b = 0x10;
b = b | 0x03;              // set low bits ->  0x13   (C++)
bool is_set = (b & 0x04);  // test a bit  ->  true
```

```javascript
let b = 0x10;
b = b | 0x03;                 // same thing in JS
let isSet = Boolean(b & 0x04);
```

```python
b = 0x10
b = b | 0x03                  # same thing in Python
is_set = bool(b & 0x04)
```

If you're ever confused, remember `x & 0x0F` means "keep only the low nibble"
and `x & 0xF0` means "keep only the high nibble".

---

## 3. CAN frames: the UDP packets of cars

A CAN frame is a fixed, tiny thing:

```
  +------------+-----------+---------+---------+
  |  CAN ID    |  DLC      | data[0..DLC-1]    |
  |  11 or 29b |  0..8     | up to 8 bytes     |
  +------------+-----------+---------+---------+
```

- **CAN ID**: 11 bits (standard) or 29 bits (extended). Think of it as a
  topic in MQTT / a label. It is *not* an address; every node on the bus
  sees every frame and picks the ones it cares about based on the ID.
- **DLC** ("Data Length Code"): how many of the data bytes are valid (0-8).
- **Data**: **at most 8 bytes**. This is the hard limit we have to work
  around with ISO-TP.

For diagnostics on modern VWs, we use **29-bit** IDs. We talk to the Gateway
using this pair:

- `0x18DA10F1` — our tester talking to **the Gateway (`0x10`)**, from itself
  (`0xF1`).
- `0x18DAF110` — Gateway talking back to **us (`0xF1`)**, from itself
  (`0x10`).

So the last two bytes of those IDs are `destination.source`. If you had a
message with ID `0x18DA01F1` you'd be talking to the Engine ECU (address
`0x01`). This is the **29-bit normal-fixed-addressing** convention from
ISO 15765-2.

**The Python analogy**: imagine UDP packets where the port number is
`0x18DA{dst}{src}` and the maximum payload is 8 bytes. That's SocketCAN.

---

## 4. Why we need ISO-TP: the "8 bytes isn't enough" problem

Suppose we want to read the Installation List — it's 128 bytes. But a CAN
frame holds 8 bytes max, and **two** of those are taken up by the UDS
headers (service + sub-function echo). That leaves *at most 6 bytes per
frame*.

**ISO-TP** (`iso_tp.cpp` in this repo) is exactly the same idea as HTTP
`Transfer-Encoding: chunked` — it splits one logical message into a
sequence of CAN frames and reassembles them on the other side.

There are only four kinds of ISO-TP frames, and you tell them apart by the
**high nibble** of the first data byte (which ISO calls the "PCI", Protocol
Control Information):

| High nibble | Kind               | Meaning                               |
|-------------|--------------------|---------------------------------------|
| `0x0`       | **Single Frame**   | Whole message fits in one frame       |
| `0x1`       | **First Frame**    | "I'm about to send a big one"         |
| `0x2`       | **Consecutive Fr** | Middle/end of a big one               |
| `0x3`       | **Flow Control**   | "Go ahead / wait / I can't fit this"  |

### 4.1 Single Frame (SF) — most requests fit here

```
byte:    0     1     2     3     4     5     6     7
     +-----+-----+-----+-----+-----+-----+-----+-----+
     | 0L  | D1  | D2  | ... | DL  | AA  | AA  | AA  |
     +-----+-----+-----+-----+-----+-----+-----+-----+
         |       \-----------v-----------/      |
         |                  L bytes of payload  |
         |                                      |
         high nibble = 0 (Single Frame)         unused bytes are padding
         low  nibble = payload length (0..7)    (0xAA on VW)
```

Example: "Enter the Extended Diagnostic Session" is exactly 2 bytes of UDS:
`10 03`. ISO-TP wraps it like this:

```
  02 10 03 AA AA AA AA AA
  ^^ ^^ ^^
  |  |  \_ sub-function 0x03 = Extended
  |  \____ UDS service 0x10 = DiagnosticSessionControl
  \_______ SF of length 2
```

This is literally what `[ISO-TP TX] ID=18DA10F1 [02 10 03 AA AA AA AA AA]`
prints in the logs.

### 4.2 First Frame + Consecutive Frames + Flow Control

When the payload is 8+ bytes we get a conversation:

```
   Tester                                  ECU
     |---------- FF (first 6 data bytes) --------->|
     |                                             |
     |<--------- FC: "Continue To Send" -----------|
     |         (also: block size, stmin)           |
     |                                             |
     |---------- CF #1 (next 7 data bytes) ------->|
     |---------- CF #2 (next 7 data bytes) ------->|
     |---------- CF #N (final bytes + padding) --->|
```

**First Frame (FF)** header is 2 bytes:

```
  byte[0] = 0x1L   <- high nibble 1 means "First Frame"
                      low nibble L = top 4 bits of the total length
  byte[1] = LL     <- bottom 8 bits of the total length

  Total length = (L << 8) | LL       # 12 bits => up to 4095 bytes
```

**Consecutive Frame (CF)** header is just 1 byte:

```
  byte[0] = 0x2S   <- high nibble 2 means "Consecutive Frame"
                      low nibble S = sequence counter (1..F, wraps to 0..F)
```

The sequence counter is the single most error-prone thing in ISO-TP. It
starts at **1** (the first CF, not the FF), counts up by 1, and *wraps from
`0xF` back to `0x0`* — not to `0x1`. Our loopback test specifically hits
the wrap because 19 CFs are enough to cross the boundary (`...2E 2F 20 21`).

**Flow Control (FC)** tells the sender to continue, wait, or abort:

```
  byte[0] = 0x3F   <- high nibble 3 means "Flow Control"
                      low nibble F = 0:Continue, 1:Wait, 2:Overflow
  byte[1] = BS     <- Block Size: how many CFs before I want another FC
                      (0 = "send everything, no more FCs needed")
  byte[2] = STmin  <- min time between CFs
                      0x00..0x7F = 0..127 ms
                      0xF1..0xF9 = 100..900 microseconds
```

---

## 5. UDS: the "REST API" of the car

UDS (ISO 14229-1) is what runs *inside* those reassembled ISO-TP messages.
The first byte of any UDS request is the **service ID (SID)** — the verb of
the request. The first byte of a positive response is **`SID + 0x40`**.

| SID  | Name                          | REST-ish analogy                   |
|------|-------------------------------|------------------------------------|
| 0x10 | DiagnosticSessionControl      | `POST /sessions  { type: ... }`    |
| 0x27 | SecurityAccess                | login / CSRF challenge-response    |
| 0x22 | ReadDataByIdentifier          | `GET /did/{hex}`                   |
| 0x2E | WriteDataByIdentifier         | `PUT /did/{hex}` + body            |
| 0x3E | TesterPresent                 | `POST /heartbeat`  (keep-alive)    |
| 0x31 | RoutineControl                | `POST /rpc/{routine}`              |
| 0x11 | ECUReset                      | `POST /reboot`                     |

A **DID (Data Identifier)** is a 16-bit number that identifies a resource,
very much like a REST path:

| DID      | What it holds                           |
|----------|-----------------------------------------|
| `0xF190` | VIN (17 ASCII characters)               |
| `0xF187` | VW spare-part number (the Gateway's PN) |
| `0xF189` | VW application software version         |
| `0x0600` | Long coding (the "config file")         |
| `0x0608` | Installation List (what modules exist)  |

### 5.1 Reading a DID, annotated

```
  Request  (tester -> ECU):  22 F1 90
    22      SID  = ReadDataByIdentifier
    F1 90   DID  = 0xF190 (VIN)

  Response (ECU -> tester):  62 F1 90 57 56 47 42 56 37 41 58 37 4C 4B 30 30 30 30 30 31
    62      SID echo = 0x22 + 0x40   <-- "positive response" bit
    F1 90   DID echo = 0xF190        <-- confirms which DID
    57 ..   17 bytes ASCII = "WVGBV7AX7LK000001"
```

The ASCII conversion works like this in Python:

```python
response = bytes.fromhex("62 F1 90 57 56 47 42 56 37 41 58 37 4C 4B 30 30 30 30 30 31".replace(" ", ""))
vin = response[3:].decode("ascii")   # "WVGBV7AX7LK000001"
```

### 5.2 Negative responses (errors)

If something goes wrong the ECU replies `7F <requested SID> <NRC>`. NRC is
the automotive equivalent of an HTTP status code:

| NRC     | Meaning                                     | Feels like |
|---------|---------------------------------------------|------------|
| `0x11`  | serviceNotSupported                         | 404        |
| `0x13`  | incorrectMessageLengthOrInvalidFormat       | 400        |
| `0x22`  | conditionsNotCorrect (engine running, etc.) | 409        |
| `0x31`  | requestOutOfRange (bad DID)                 | 404        |
| `0x33`  | securityAccessDenied                        | 401        |
| `0x35`  | invalidKey                                  | 403        |
| `0x36`  | exceededNumberOfAttempts (rate limited)     | 429        |
| `0x78`  | requestCorrectlyReceived-ResponsePending    | 102 (like "still processing") |
| `0x7F`  | serviceNotSupportedInActiveSession          | "you're in the wrong session" |

**`0x78` is special**: it's not really an error. It means "I heard you, hold
on, I'll have a real answer for you in a moment." `UdsClient::request` loops
silently while NRCs of `0x78` come in — same idea as waiting on a WebSocket
until the server finishes crunching.

### 5.3 The extended session, in REST terms

The ECU's default "session" only allows read-only stuff. To do anything
interesting (adaptations, writes, security) you first POST to the session
endpoint:

```
  22 F1 90     --> works in default session      (read VIN)
  2E 06 08 .. --> NRC 0x7F in default session     (write needs extended)
```

So we always do this first:

```
  10 03   --> POST /sessions { "type": "extended" }
            -> ECU responds "OK, we're in extended now"
```

And because the extended session times out after ~5 seconds of silence, you
send `3E 00` (TesterPresent) periodically — a heartbeat. That's what the
`testerPresent()` helper does.

---

## 6. Security Access, the login you can't skip

Writing with `0x2E` almost always requires **Security Access (0x27)** first.
The flow is a challenge-response:

```
  1.  Tester:  27 03              (give me a seed, level 3)
  2.  ECU:     67 03 AA BB CC DD  (here's a random 4-byte seed)
  3.  Tester:  27 04 11 22 33 44  (here's the key, computed from the seed)
  4.  ECU:     67 04              (ok, you're unlocked)
```

The `key = f(seed)` function is proprietary and varies by ECU software
version. In `vw_mqb.h` there's a `KeyFn` typedef:

```cpp
using KeyFn = std::function<std::vector<uint8_t>(const std::vector<uint8_t>& seed)>;
```

...which is literally the same shape as:

```python
def key_from_seed(seed: bytes) -> bytes: ...
```

You plug your algorithm in, and the client does the rest.

A "seed of all zeros" is the ECU's way of saying "you are already unlocked,
no need for a key". Our code detects that and skips the key step.

---

## 7. Walking through "Fix Emergency Call" byte by byte

This is exactly what `build/uds_main --apply fix-ecall` does. Suppose the
current Installation List (DID `0x0608`) has `0x01` at byte `0x75`:

```
  1) Enter extended session
     TX: 02 10 03 AA AA AA AA AA
     RX: 06 50 03 00 32 01 F4 AA   <- OK, P2 timing = 50ms, P2* = 5s

  2) SecurityAccess (here assuming it's already unlocked)
     TX: 02 27 03 AA AA AA AA AA
     RX: 06 67 03 00 00 00 00 AA   <- seed all zero => unlocked

  3) Read the Installation List (multi-frame!)
     TX: 03 22 06 08 AA AA AA AA                      # SF of 3 bytes
     RX: 10 83 62 06 08 00 01 00   <- FF, total 0x083 = 131 bytes
     TX: 30 00 00 AA AA AA AA AA                      # FC: Continue, BS=0, ST=0
     RX: 21 00 00 00 ...                              # CF #1
         22 ...                                       # CF #2
         ...
         2E AB AB ... AB                              # last CF

     After reassembly, the UDS payload is 131 bytes:
       62 06 08  <128 bytes of list>
          ^^^^^ DID echo
     list[0x75] == 0x01  (Telematics module = INSTALLED = SOS warning on)

  4) Flip one bit (byte 0x75, bit 0): 0x01 -> 0x00
     All other bytes are copied verbatim. Critical.

  5) Write back (multi-frame request this time!)
     TX: 10 83 2E 06 08 00 01 00   <- FF, total 0x083 = 131 bytes
     RX: 30 00 00 AA AA AA AA AA   <- FC from ECU
     TX: 21 00 00 00 ...           <- CF #1
         22 ...
         ...
         2E 00 00 00 AB            <- last CF (note: byte 0x75 is 0x00 here)
     RX: 03 6E 06 08 AA AA AA AA   <- SF positive response (0x6E = 0x2E + 0x40)

  6) Return to default session
     TX: 02 10 01 AA AA AA AA AA
     RX: 06 50 01 00 32 01 F4 AA
```

That's it. One bit flipped, turning the dashboard warning off forever.

---

## 8. Why Start-Stop is subtler than ecall

The Installation List bit for module `0x75` is unambiguous: it's either 0 or 1
and module 75 is always "telematics". The car doesn't care *why* the bit is
cleared, only that the Gateway stops expecting that module to exist.

**Start-Stop is coding**, not installation. The Gateway stores a block of
maybe 20-60 configuration bytes at DID `0x0600`, and each byte has many
*different* bits assigned to unrelated features. Something like:

```
  coding[14] = 0b 0 0 1 0 0 1 1 0
                  | | | | | | | |
                  | | | | | | | +-- (reserved)
                  | | | | | | +---- (reserved)
                  | | | | | +------ some unrelated feature
                  | | | | +-------- some other feature
                  | | | +---------- maybe start-stop enable?
                  | | +------------ (reserved)
                  | +-------------- another feature
                  +---------------- (reserved)
```

The bit index that controls Start-Stop is **vehicle-specific**. Different
Gateway part numbers have different coding layouts. That's why
`vw::disableStartStop(...)` takes `coding_byte_index` and `bit_mask` as
*parameters* rather than hard-coding them — we refuse to guess which bit
is yours without your coding dump to compare against.

In the code:

```cpp
uint8_t after = clear_bit_to_disable
              ? (before & ~bit_mask)     // clear  (JS: before & ~mask)
              : (before |  bit_mask);    // set    (JS: before |  mask)
```

`~bit_mask` is "invert every bit of the mask". `before & ~bit_mask` leaves
every bit untouched *except* the ones in the mask, which become 0.

---

## 9. Glossary

| Term              | 30-second definition |
|-------------------|----------------------|
| **bit**           | 0 or 1 |
| **byte**          | 8 bits, values 0-255, written as `0x00..0xFF` in hex |
| **nibble**        | 4 bits, half of a byte |
| **CAN frame**     | One message on the bus: ID + up to 8 data bytes |
| **CAN ID**        | Like an MQTT topic; tells listeners "this one is for you" |
| **11-bit / 29-bit ID** | Short / extended addressing. VW diagnostics = 29-bit |
| **SocketCAN**     | Linux API that makes a CAN interface look like a socket |
| **ISO-TP**        | Chunking protocol; splits >8-byte messages into CAN frames |
| **PCI**           | "Protocol Control Information" = the ISO-TP header byte |
| **SF / FF / CF / FC** | ISO-TP frame types: Single, First, Consecutive, Flow Ctrl |
| **UDS**           | Request/response diagnostic protocol layered on ISO-TP |
| **SID**           | UDS service byte (`0x10`, `0x22`, `0x27`, `0x2E`...) |
| **DID**           | UDS data identifier (16-bit, like a resource path) |
| **NRC**           | UDS negative-response code (`0x33`, `0x35`, `0x78`...) |
| **OBD-II PID**    | Older, simpler standard. Service `0x01` + 1-byte PID for live data |
| **Extended session** | UDS session state that unlocks adaptations/writes |
| **Security Access**  | Challenge-response login required before writes |
| **Adaptation**    | "Change a setting" (usually via WriteDataByIdentifier) |
| **Coding**        | The Gateway's config blob at DID `0x0600` |
| **Installation List** | Bitmap at DID `0x0608`: which modules are present |
| **ECU**           | Any controller on the bus. Engine = 01, Gateway = 19, etc. |
| **Gateway (J533)**| The central router ECU; we target it via `0x18DA10F1` |
| **VCDS / ODIS / OBDeleven** | Commercial diagnostic tools that do all this with a GUI |

---

## 10. Where to start reading the code

If you've read this far, the suggested reading order is:

1. **`uds/iso_tp.cpp`** — see §4 of this guide next to the code; the comments
   now walk through every bit manipulation.
2. **`uds/uds_client.cpp`** — §5. NRC decoding, the `0x78` loop, and how a
   positive vs negative response is recognised.
3. **`uds/vw_mqb.cpp`** — §7 and §8. The actual bit flip for module `0x75`
   and the coding byte manipulation for Start-Stop.
4. **`uds/main.cpp`** — just glue: parses CLI args, opens the socket,
   dispatches to one of the commands above.

The test harness at **`tests/isotp_loopback.cpp`** is itself a worked
example: it builds a fake ECU on a socketpair and runs every frame shape
through the stack. If you want to try tweaking things without a car, that
test is where to experiment.

---

## 11. "Will this brick my car?" — the honest answer

You asked two closely related questions:

1. **How much test coverage is there?**
2. **How can I be sure my car won't end up a brick?**

They're related but distinct. Short version up front:

- Current automated coverage is **99.5% of lines** across `uds/*.cpp`
  (`backup.cpp` 100.0%, `uds_client.cpp` 100.0%, `iso_tp.cpp` 99.1%,
  `vw_mqb.cpp` 98.7%). Honest number, measured by `gcov` in Docker.
  See `Dockerfile.coverage`. The 3 remaining "uncovered" lines are
  `gcov` attribution artifacts (a bare closing `}` and middle lines of
  chained `std::cerr << ... << ... << "\n"` statements whose surrounding
  lines are all hit) — not real missing paths.
- **Coverage does NOT protect you from bricking.** Coverage tells you
  "the code paths ran" — it does **not** tell you "they produced the
  correct bytes for your particular car". A procedure protects you.
  Below is the procedure.

### 11.1 What "brick" actually means here

Writing through UDS on a Gateway (ECU 19) almost never *destroys* the
hardware. A real brick — "the car won't crank again" — usually requires
one of:

- Power loss during a flash update (we don't do flashing; only coding).
- Writing a corrupt ECU firmware binary (we only write coding DIDs).
- Writing to Immobilizer / Kessy / steering-lock modules (we don't).

The realistic failure modes for *this* tool are:

| Failure mode | How bad |
|---|---|
| Wrong bit flipped in long coding | Warning light, feature stops working; re-read + re-flip fixes it |
| Installation list byte cleared by mistake | "Module X not installed" CAN errors until restored |
| Coding length mismatch | ECU returns NRC `0x13` and does *not* write (safe) |
| SecurityAccess fails | Nothing is written; session drops |
| CAN wire unplugged mid-write | Write partially lands; **this is where readback + restore saves you** |

All of these are **recoverable** if you have a backup.

### 11.2 The rules this tool enforces in code

These are not advice — they are in the source. You cannot skip them
without `--no-backup` (which screams at you):

1. **Dry-run by default.** Every write command refuses to transmit `0x2E`
   unless you pass `--apply`. Until then you only see diffs.
2. **`--apply` requires `--backup <file>`.** The tool loads the file,
   reads the live Gateway's VIN and part number, and compares. If they
   don't match → abort, no write sent. `uds/main.cpp::enforceBackupPolicy`.
3. **Readback after every write.** Every helper that calls `0x2E` (Write
   Data By Identifier) immediately follows up with `0x22` (Read Data By
   Identifier) and byte-compares. A mismatch is a hard failure.
   See `writeInstallationList()` and `disableStartStop()` in
   `uds/vw_mqb.cpp`.
4. **Length check before write.** Each helper reads the buffer first and
   writes back the same length, with a single byte mutated. Writing a
   shorter buffer is refused by the ECU anyway (NRC `0x13`), but we also
   refuse to generate it.
5. **Identity mismatch = no write.** Restoring a backup from a different
   car (VIN mismatch) or different Gateway part number is an error.

### 11.3 The procedure you should follow on the actual car

Literally do it in this order. If you skip a step and something goes
wrong, you're on your own:

1. **Battery on a charger.** Low battery during coding = aborted writes.
   Aim for > 12.6 V resting.
2. **Engine off. Ignition on.** Don't crank while writing.
3. **Nothing else on the OBD bus.** No VCDS open, no OBDeleven app
   connected. Two testers fighting each other is a great way to get
   undefined behaviour.
4. **Take a backup.** This is the single most important step.
   ```
   ./build/uds_main backup pre-change-$(date +%F).txt
   ```
   Open the file. Confirm the VIN is yours. Keep it.
5. **Dry-run the change.**
   ```
   ./build/uds_main fix-ecall
   ./build/uds_main disable-ss
   ```
   The output shows exactly what bytes *would* be written. Eyeball the
   `AFTER (planned)` block. Only one byte should differ for `fix-ecall`;
   only one byte for `disable-ss`.
6. **Apply, with the backup you just took:**
   ```
   ./build/uds_main --apply --backup pre-change-$(date +%F).txt fix-ecall
   ```
   Watch for the `readback matches write` line. If you see a mismatch,
   the tool has already aborted — but run `restore` anyway to be safe.
7. **Verify in the car.** Turn the ignition off. Wait ~30 seconds.
   Turn it back on. Drive. Confirm the warning/feature behaved as you
   wanted. If not → restore (below).

### 11.4 If something goes wrong: how to recover

You have the backup from step 4. The recovery is one command:

```
./build/uds_main --apply --backup pre-change-$(date +%F).txt restore
```

This:

- Verifies the live ECU is still the same car (VIN + part number).
- Writes both the saved coding (`DID 0x0600`) and the saved installation
  list (`DID 0x0608`) back.
- Re-reads both and confirms the restore landed byte-for-byte.

If the restore itself fails (e.g. SecurityAccess rejected because the
ECU locked itself out), the fallback is a dealer VCDS/ODIS session —
they can always force-flash a Gateway coding from the factory defaults.
The backup file you made in step 4 is plain text, human-readable, and
can be handed to the dealer as "this is what my car looked like before
I touched it".

### 11.5 What test coverage does and does not prove

What the **99.5%** line coverage gives you:

- The transport layer (`iso_tp.cpp`, 99.1%) correctly segments and
  reassembles ISO-TP messages, honours flow control (CTS / Wait / OVFL),
  respects the ECU's STmin (including the reserved-value 127 ms
  fallback), wraps sequence counters at 0xF → 0x0, times out when the
  peer goes silent, and surfaces `SocketError` / `UnexpectedPci` when
  the bus is physically broken or feeds malformed frames.
- The UDS layer (`uds_client.cpp`, 100%) parses positive responses,
  echoes and rejects mismatched SIDs, handles the `0x78`
  response-pending storm including exhaustion, decodes the full NRC
  table in `nrcToString()`, suppresses positive responses on request,
  and negotiates SecurityAccess with real seeds, all-zero seeds
  ("already unlocked"), empty responses, and invalid even sub-functions.
- The VW helpers (`vw_mqb.cpp`, 98.7%) read gateway identity
  (full and partial), read/write installation list, apply the
  `fixEmergencyCall` bitmask (dry-run, no-op, apply, readback-mismatch,
  refusal on short list), and apply `disableStartStop` across every
  branch (bit already in state, byte index out of range, RDBI error,
  coding response too short, WDBI NRC, readback RDBI failure, readback
  byte mismatch).
- Backup round-trip (`backup.cpp`, 100%) is bit-exact: save → load
  produces identical bytes; identity mismatches, VIN/part-number
  changes, SW-version warnings, missing files, empty files, and
  malformed hex are all surfaced to the operator.
- Readback verify **actually catches** the case where the ECU accepts a
  write but stores something different — the loopback test exercises
  this path with a mutable fake ECU that is deliberately corrupted
  after the write.

What coverage does **not** tell you:

- Whether the VW Gateway part number in *your* car uses the byte/bit
  offsets assumed by `disable-ss`. Those come from a VCDS label file
  specific to your Gateway SW version, and you must confirm them.
- Whether your Gateway needs a real seed/key algorithm rather than the
  legacy 20103 static login. If it does, SecurityAccess fails with NRC
  `0x35` and no write is sent — safe, but the tool can't proceed until
  you plug in a proper `KeyFn` (see `uds/uds_client.h`).

### 11.6 Tl;dr

You are safe if you do three things:

1. Take a backup **before** every `--apply`.
2. Read the `BEFORE` / `AFTER (planned)` dry-run output before passing
   `--apply`.
3. If anything ever looks wrong after a write, run `restore` with the
   backup from step 1.

The tool refuses to write without a matching backup on disk. That is
the single safeguard that makes this workflow significantly safer than
manual VCDS clicking.

