import serial
import json
import sys
import time

# ---- Configuration ----------------------------------------------------------
SERIAL_PORT  = 'COM3'
BAUD_RATE    = 921600
OUTPUT_FILE  = 'live_capture.csi.jsonl'
TIMEOUT_S    = 2       # silence warning after this many seconds with no data
# -----------------------------------------------------------------------------

print("=== RuView Live Streamer ===")
print(f"Port: {SERIAL_PORT}  Baud: {BAUD_RATE}  Output: {OUTPUT_FILE}")
print("Ctrl+C to stop.\n")
sys.stdout.flush()

try:
    ser = serial.Serial(
        port        = SERIAL_PORT,
        baudrate    = BAUD_RATE,
        timeout     = 1,            # readline() returns after 1 s if no newline
        write_timeout = 2,
    )
    # Discard any stale bytes that arrived during USB enumeration
    ser.reset_input_buffer()
    print(f"Serial port {SERIAL_PORT} opened OK.")
    print("Serial connected, waiting for first packet...")
    sys.stdout.flush()

except serial.SerialException as e:
    print(f"ERROR: Could not open {SERIAL_PORT}: {e}")
    sys.exit(1)

captured   = 0
total_lines = 0
last_packet_time = time.time()

try:
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        # ── 7-second countdown ────────────────────────────────────────────────
        # The file is now open (and empty), so the visualizer can be started
        # right away and will tail for packets.  No data is written until the
        # countdown reaches zero, guaranteeing all captured packets come from
        # an empty room.
        print("[Wi-Sense]  Leaving the room...  7", flush=True)
        for _secs in range(6, 0, -1):
            time.sleep(1)
            print(f"  {_secs}", flush=True)
        time.sleep(1)
        print("[Wi-Sense]  Room clear -- starting live stream and capture.",
              flush=True)
        print()
        # ─────────────────────────────────────────────────────────────────────

        while True:
            # ------------------------------------------------------------------
            # Read one line. With timeout=1 this returns b'' after 1 s of
            # silence — it never hangs indefinitely.
            # ------------------------------------------------------------------
            try:
                raw = ser.readline()
            except serial.SerialException as e:
                print(f"[ERROR] Serial read failed: {e}")
                sys.stdout.flush()
                break

            # Timeout — no newline arrived within 1 s
            if not raw:
                elapsed = time.time() - last_packet_time
                if elapsed > TIMEOUT_S:
                    print(f"[WAIT] No CSI packet for {elapsed:.0f}s "
                          f"(total captured: {captured}) — "
                          f"check that both boards are powered and on CH1.")
                    sys.stdout.flush()
                    last_packet_time = time.time()  # reset so we don't spam
                continue

            # Decode — ignore stray non-UTF8 bytes
            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            total_lines += 1

            # ------------------------------------------------------------------
            # Show everything the board sends so we can see boot messages,
            # errors, or unexpected output.
            # ------------------------------------------------------------------
            if not line.startswith('{'):
                print(f"[BOARD] {line}")
                sys.stdout.flush()
                continue

            # ------------------------------------------------------------------
            # Must be a JSON line — try to parse it
            # ------------------------------------------------------------------
            try:
                data = json.loads(line)
            except json.JSONDecodeError as e:
                print(f"[BAD JSON] ({e}) raw={line[:120]}")
                sys.stdout.flush()
                continue

            # ------------------------------------------------------------------
            # Our compact Wi-Sense schema: {"s":N,"r":R,"n":N,"m":"mac","l":L,"d":"hex"}
            # ------------------------------------------------------------------
            if 's' not in data or 'd' not in data:
                print(f"[UNKNOWN JSON] keys={list(data.keys())}  line={line[:80]}")
                sys.stdout.flush()
                continue

            # Write to JSONL and flush so RuView sees it immediately
            f.write(line + '\n')
            f.flush()

            captured += 1
            last_packet_time = time.time()

            print(
                f"#{captured:>6}  seq={data['s']}  "
                f"RSSI={data.get('r')} dBm  "
                f"noise={data.get('n')} dBm  "
                f"MAC={data.get('m')}  "
                f"CSI={data.get('l')}B"
            )
            sys.stdout.flush()

except KeyboardInterrupt:
    print(f"\nStopped by user. Captured {captured} packets → {OUTPUT_FILE}")
except Exception as e:
    print(f"\n[FATAL] {type(e).__name__}: {e}")
    sys.stdout.flush()
finally:
    ser.close()
    print("Serial port closed.")
