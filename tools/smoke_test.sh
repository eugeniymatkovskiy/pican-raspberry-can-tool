#!/usr/bin/env bash
# Smoke test the UDS binary against a virtual CAN interface.
#
# - Brings up vcan0
# - Starts candump in the background to capture what our binary transmits
# - Runs `uds_main self-test` (expected to time out: no ECU is responding)
# - Asserts that the expected Session Control request hit the bus
set -uo pipefail

LOG=/tmp/candump.log

echo "=== Loading vcan module & bringing up vcan0 ==="
modprobe vcan || { echo "SKIP: vcan kernel module unavailable in this VM"; exit 77; }
ip link add dev vcan0 type vcan 2>/dev/null || true
ip link set up vcan0
ip -details link show vcan0

echo
echo "=== Starting candump on vcan0 ==="
rm -f "$LOG"
candump -t d -x vcan0 > "$LOG" 2>&1 &
DUMP_PID=$!
sleep 0.2

echo
echo "=== Running uds_main self-test (expected: TimeoutRx) ==="
./build/uds_main --iface vcan0 --tx 18DA10F1 --rx 18DAF110 self-test || true

sleep 0.3
kill "$DUMP_PID" 2>/dev/null || true
wait "$DUMP_PID" 2>/dev/null || true

echo
echo "=== candump capture ==="
cat "$LOG"

echo
echo "=== Assertions ==="
rc=0

# We expect our binary to have transmitted a 29-bit frame on ID 18DA10F1
# carrying an ISO-TP Single Frame [02 10 03 AA AA AA AA AA]
#    02 = SF, length 2
#    10 = Diagnostic Session Control
#    03 = Extended Session sub-function
#    AA = ISO-TP pad bytes
if grep -qE '18DA10F1.* 02 10 03 AA AA AA AA AA' "$LOG"; then
    echo "PASS: ExtendedSession request frame observed on vcan0"
else
    echo "FAIL: Did not observe SF [02 10 03 ...] on 18DA10F1"
    rc=1
fi

# After the session attempt times out, we still try DefaultSession cleanup
# [02 10 01 AA AA AA AA AA].
if grep -qE '18DA10F1.* 02 10 01 AA AA AA AA AA' "$LOG"; then
    echo "PASS: DefaultSession cleanup frame observed on vcan0"
else
    echo "FAIL: Did not observe SF [02 10 01 ...] on 18DA10F1"
    rc=1
fi

exit $rc
