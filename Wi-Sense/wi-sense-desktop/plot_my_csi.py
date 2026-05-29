#!/usr/bin/env python3
"""
Wi-Sense CSI Visualization Dashboard  v4  —  Two-Stage Adaptive State Machine
===============================================================================
Zero-blind-time motion detection with instant-on anomaly triggering during the
5-second calibration window and maximum-sensitivity tracking afterwards.

STATE MACHINE
─────────────
STAGE 1 · CALIBRATING  (first 500 packets / 5 s @ 100 Hz)
  ┌─ Baseline built with a fast EMA (α=0.02, τ≈50 packets).
  ├─ Threshold uses Welford online statistics (valid from packet 1).
  │    n < 20  → absolute bootstrap guard  (energy > BOOTSTRAP_THRESH)
  │    n ≥ 20  → Welford mean + CALIB_ZSCORE·σ   (k = 4.5)
  └─ Any packet exceeding that limit → "⚡ MOTION (HIGH ANOMALY!)" in orange.
     No silent window — the detector is live from the very first frame.

STAGE 2 · TRACKING  (frames 500+ / after 5 s)
  ┌─ Baseline transitions to a slow EMA (α=0.003, τ≈333 packets).
  │    Update is motion-gated: the slow EMA freezes during detected motion
  │    so a stationary person does not become the new "empty room".
  ├─ Threshold drawn from a 400-packet rolling energy window:
  │    threshold = μ_400 + TRACK_ZSCORE·σ_400   (k = 2.2)
  └─ Any deviation above threshold → "MOTION DETECTED" in red.

VISUAL LANGUAGE
───────────────
  ● Yellow border + yellow text   CALIBRATING (idle)
  ● Orange text                   CALIBRATING (high anomaly!)
  ● Green  text                   TRACKING / IDLE
  ● Red    border + red text      TRACKING / MOTION DETECTED

PANELS
──────
  A  Amplitude-deviation heatmap (63 subcarriers × time)
     Uses the active baseline so sensitivity rises from Stage 1 → Stage 2.
  B  Motion-energy trace with Savitzky-Golay smoothing and adaptive threshold.
  C  Per-subcarrier variance bars — guides feature selection for the DL model.

Supported input formats
───────────────────────
  Wi-Sense compact  {"s":N,"r":-15,"n":-96,"m":"mac","l":128,"d":"<hex>"}
  Legacy RuView     {"timestamp":T,"iq_hex":"<hex>","subcarriers":64}

Usage
─────
  # Live mode (tails live_capture.csi.jsonl while stream_to_ruview.py runs)
  python plot_my_csi.py

  # Custom file
  python plot_my_csi.py -f path/to/capture.csi.jsonl

  # Offline static plot
  python plot_my_csi.py --offline

  # Tune window and calibration frame count
  python plot_my_csi.py -w 800 --calib 300

  # Hide guard-band subcarriers in the heatmap
  python plot_my_csi.py --hide-guards

Dependencies: numpy  matplotlib  scipy
  pip install numpy matplotlib scipy
"""

import argparse
import collections
import json
import sys
from enum import Enum
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.signal import savgol_filter


# ════════════════════════════════════════════════════════════════════════════
# Constants
# ════════════════════════════════════════════════════════════════════════════

N_SC              = 63      # subcarriers after dropping DC
DC_IDX            = 32      # DC subcarrier position in the raw 64-pair buffer
GUARD_THR         = 0.5     # EMA baseline below this → guard/null subcarrier

# Stage 1 — Calibration
CALIB_FRAMES      = 500     # packets to collect before entering tracking mode
EMA_ALPHA_FAST    = 0.02    # fast baseline decay (τ ≈ 50 packets ≈ 0.5 s)
CALIB_ZSCORE      = 4.5     # high-anomaly Z-score multiplier
BOOTSTRAP_THRESH  = 5.0     # absolute fallback threshold for n < 20 packets

# Stage 2 — Tracking
EMA_ALPHA_SLOW    = 0.003   # slow baseline decay (τ ≈ 333 packets ≈ 3.3 s)
TRACK_ZSCORE      = 2.5     # tracking Z-score (lowered for sensitivity; debounce prevents noise)
ENERGY_STAT_WIN   = 400     # rolling window for Stage-2 threshold statistics
DEBOUNCE_FRAMES   = 5       # consecutive frames required to flip is_motion (≈ 50 ms @ 100 Hz)

# Stage 2 — Per-subcarrier anomaly criterion (second OR leg)
SC_ANOM_DB        = 2.5     # |ΔdB| from per-SC EMA baseline to flag one subcarrier
SC_ANOM_COUNT     = 8       # number of anomalous SCs required to trigger sc criterion

# Stage 2 — Micro-motion criterion (third OR leg)
# Models the 40-packet rolling window via two nested EMAs:
#   micro_mean  (slow, τ≈40 frames): tracks local mean of per-SC amplitude_dB
#   micro_var   (fast, τ≈10 frames): smooths squared deviation from that mean
# sqrt(micro_var) ≈ rolling RMS — sustained oscillation keeps it elevated;
# a single noise spike decays inside the fast τ before debounce fires.
MICRO_SLOW_ALPHA  = 0.025   # τ ≈ 40 frames = 400 ms  (rolling-window equivalent)
MICRO_FAST_ALPHA  = 0.10    # τ ≈ 10 frames = 100 ms  (power-estimate smoothing)
MICRO_THR_DB      = 0.6     # per-SC smoothed-std threshold in dB
MICRO_SC_COUNT    = 3       # concurrent flagged SCs needed to assert crit_micro

# Stage 2 — Presence-timeout adaptive reset
# If is_motion has been True for >PRESENCE_TIMEOUT consecutive frames AND the
# energy trace over the last PRESENCE_VAR_WIN frames has variance below
# PRESENCE_VAR_THR, the occupant has become static in a new position.
# Force-adopt the current amplitudes as the new room baseline to break the lock.
PRESENCE_TIMEOUT  = 300    # consecutive motion frames before reset eligibility (~3 s)
PRESENCE_VAR_WIN  = 100    # energy frames used for the flatness test (~1 s)
PRESENCE_VAR_THR  = 0.05   # energy variance below this => signal flat => force reset

# Savitzky-Golay smoothing of the energy trace
SG_WIN            = 21
SG_POLY           = 3


# ════════════════════════════════════════════════════════════════════════════
# Online statistics (Welford's algorithm)
# ════════════════════════════════════════════════════════════════════════════

class WelfordStats:
    """
    Numerically-stable online mean and variance.
    Accepts individual scalar updates; valid from the very first sample.
    """

    __slots__ = ('n', '_mean', '_M2')

    def __init__(self) -> None:
        self.n     = 0
        self._mean = 0.0
        self._M2   = 0.0

    def update(self, x: float) -> None:
        self.n += 1
        d1 = x - self._mean
        self._mean += d1 / self.n
        d2 = x - self._mean
        self._M2   += d1 * d2

    @property
    def mean(self) -> float:
        return self._mean

    @property
    def variance(self) -> float:
        return self._M2 / max(self.n - 1, 1)

    @property
    def std(self) -> float:
        return self.variance ** 0.5


# ════════════════════════════════════════════════════════════════════════════
# Detector state
# ════════════════════════════════════════════════════════════════════════════

class Stage(Enum):
    CALIBRATING = 'CALIBRATING'
    TRACKING    = 'TRACKING'


# ════════════════════════════════════════════════════════════════════════════
# Parsing
# ════════════════════════════════════════════════════════════════════════════

def _parse_hex(hex_str: str) -> 'np.ndarray | None':
    """
    Decode an ESP-IDF CSI hex string → amplitude array shape (63,).

    ESP-IDF buffer layout (confirmed by esp32_jsonl_to_rvcsi.py):
        buf[2k]   = imaginary part of subcarrier k  (signed int8)
        buf[2k+1] = real      part of subcarrier k  (signed int8)
    numpy dtype=int8 handles the two's-complement sign extension automatically.
    """
    try:
        raw = np.frombuffer(bytes.fromhex(hex_str), dtype=np.int8).astype(np.float32)
    except (ValueError, TypeError):
        return None
    n_pairs = len(raw) // 2
    if n_pairs < 4:
        return None
    imag     = raw[0 : 2 * n_pairs : 2]
    real     = raw[1 : 2 * n_pairs : 2]
    amp_full = np.hypot(imag, real)          # sqrt(I²+Q²), numerically stable
    a64      = np.zeros(64, dtype=np.float32)
    a64[:min(n_pairs, 64)] = amp_full[:64]
    return np.delete(a64, DC_IDX)            # drop DC → 63 values


def parse_line(line: str) -> 'tuple | None':
    """
    Returns (amp63, rssi, seq) or None.
    Accepts both Wi-Sense ('d') and legacy RuView ('iq_hex') formats.
    """
    line = line.strip()
    if not line or line[0] != '{':
        return None
    try:
        data = json.loads(line)
    except json.JSONDecodeError:
        return None
    hex_str = data.get('d') or data.get('iq_hex', '')
    if not hex_str:
        return None
    amp63 = _parse_hex(hex_str)
    if amp63 is None:
        return None
    return (
        amp63,
        float(data.get('r', data.get('rssi', 0))),
        int(  data.get('s', data.get('seq',  0))),
    )


# ════════════════════════════════════════════════════════════════════════════
# Ring buffer with two-stage adaptive state machine
# ════════════════════════════════════════════════════════════════════════════

class CsiBuffer:
    """
    Rolling buffer of CSI amplitude vectors.

    All signal processing and state-machine logic lives here; the Dashboard
    only reads public properties — clean separation of concerns.
    """

    def __init__(self, capacity: int, calib_frames: int = CALIB_FRAMES) -> None:
        self._cap          = capacity
        self._calib_frames = calib_frames

        # Raw sample storage
        self._amps   = collections.deque(maxlen=capacity)
        self._energy = collections.deque(maxlen=capacity)
        self._motion = collections.deque(maxlen=capacity)  # bool per frame
        self._rssi   = collections.deque(maxlen=capacity)

        # Stage 1: fast EMA baseline
        self._bl_fast: 'np.ndarray | None' = None

        # Stage 2: slow EMA baseline (initialised from _bl_fast at transition)
        self._bl_slow: 'np.ndarray | None' = None

        # Online statistics over ALL energy samples (Welford — valid from n=1)
        self._welford = WelfordStats()

        # Rolling window used by Stage-2 threshold
        self._energy_win = collections.deque(maxlen=ENERGY_STAT_WIN)

        # ── Public detector state (read by Dashboard) ──────────────────────
        self.frame_count:      int   = 0
        self.stage:            Stage = Stage.CALIBRATING
        self.is_motion:        bool  = False
        self.current_thresh:   float = BOOTSTRAP_THRESH
        self.calib_frac:       float = 0.0   # 0.0 → 1.0 during calibration

        # Label / colour strings consumed directly by Dashboard.draw()
        self.label:  str = 'CALIBRATING (0%)'
        self.colour: str = '#ffcc00'          # yellow while calibrating

        # Per-criterion debounce streak counters (Stage 2 only).
        # Each criterion must fire for DEBOUNCE_FRAMES consecutive frames
        # INDEPENDENTLY before it can contribute to a motion trigger.
        # This prevents cross-criterion accumulation: MACD RF noise (crit_micro)
        # cannot combine with a single energy spike (crit_energy) to reach the
        # 5-frame threshold — each leg is evaluated in its own isolated counter.
        self._energy_streak: int = 0  # consecutive frames crit_energy is True
        self._sc_streak:     int = 0  # consecutive frames crit_sc is True
        self._micro_streak:  int = 0  # consecutive frames crit_micro is True
        self._below_streak:  int = 0  # consecutive frames ALL criteria are False

        # Transition tracking for console log / stats overlay
        self.last_energy:   float = 0.0
        self.last_sc_anom:  int   = 0   # SC count that tripped crit_sc last frame
        self.last_micro_sc: int   = 0   # SC count that tripped crit_micro last frame

        # Micro-motion MACD state (per-subcarrier, shape 63, init on first Stage-2 frame)
        # MACD = |fast_ema - slow_ema|: responds to oscillation, not static offset.
        # Static presence → both converge → MACD → 0 (no false positive after person stops).
        # Oscillation → fast leads slow → MACD stays elevated for duration of motion.
        self._micro_slow: 'np.ndarray | None' = None  # slow EMA of amp_dB (α=MICRO_SLOW_ALPHA)
        self._micro_fast: 'np.ndarray | None' = None  # fast EMA of amp_dB (α=MICRO_FAST_ALPHA)

        self._prev_motion: bool  = False
        self._prev_stage:  Stage = Stage.CALIBRATING

        # Consecutive Stage-2 frames where is_motion is True.
        # Used by the presence-timeout adaptive reset to detect static lock.
        self._motion_streak: int = 0

    # ── ingestion ────────────────────────────────────────────────────────────

    def push(self, amp63: np.ndarray, rssi: float) -> None:
        self.frame_count += 1

        # ---- 1. Update fast EMA baseline (always) ---------------------------
        if self._bl_fast is None:
            self._bl_fast = amp63.copy()
        else:
            self._bl_fast += EMA_ALPHA_FAST * (amp63 - self._bl_fast)

        # ---- 2. Choose which baseline drives this frame's energy ------------
        if self.frame_count < self._calib_frames:
            baseline = self._bl_fast
        else:
            # Initialise slow EMA from fast EMA exactly at the transition
            if self._bl_slow is None:
                self._bl_slow = self._bl_fast.copy()
            baseline = self._bl_slow

        # ---- 3. Compute motion energy for this frame ------------------------
        active = baseline > GUARD_THR
        if active.any():
            energy = float(np.abs(amp63 - baseline)[active].mean())
        else:
            energy = 0.0
        self.last_energy = energy

        self._amps.append(amp63)
        self._rssi.append(rssi)

        # ---- 4. State machine -----------------------------------------------
        # IMPORTANT: thresholds are computed from the window BEFORE appending
        # the current energy, so a spike cannot self-cancel its own detection.
        n = self.frame_count

        if n < self._calib_frames:
            # ── STAGE 1: CALIBRATING ─────────────────────────────────────────
            self.stage      = Stage.CALIBRATING
            self.calib_frac = n / self._calib_frames

            if n < 20:
                # Not enough data for reliable statistics — absolute guard
                thresh = BOOTSTRAP_THRESH
            else:
                thresh = self._welford.mean + CALIB_ZSCORE * max(self._welford.std, 1e-6)

            self.current_thresh = thresh
            self.is_motion      = energy > thresh

            # Append AFTER threshold so this frame cannot self-cancel its spike
            self._welford.update(energy)
            self._energy_win.append(energy)
            self._energy.append(energy)

            pct = int(self.calib_frac * 100)
            bar = '#' * (pct // 5) + '.' * (20 - pct // 5)
            if self.is_motion:
                self.label  = '!! MOTION (HIGH ANOMALY!)'
                self.colour = '#ff9900'                         # orange
            else:
                self.label  = f'CALIBRATING  [{bar}]  {pct}%'
                self.colour = '#ffcc00'                         # yellow

        else:
            # ── STAGE 2: TRACKING ────────────────────────────────────────────
            self.stage      = Stage.TRACKING
            self.calib_frac = 1.0

            # Compute threshold from the rolling energy window
            win_arr = np.asarray(self._energy_win, dtype=np.float32)
            mu      = float(win_arr.mean())
            sigma   = float(win_arr.std())
            thresh  = mu + TRACK_ZSCORE * max(sigma, 1e-6)
            self.current_thresh = thresh

            # ── Criterion 1: mean energy exceeds adaptive threshold ──────────
            crit_energy = energy > thresh

            # ── Criterion 2: localised per-subcarrier dB anomaly count ───────
            # Counts active SCs whose amplitude deviates > SC_ANOM_DB dB from
            # the slow EMA baseline.  Catches localised shifts missed by the
            # mean-energy metric (e.g. one limb crossing a single beam path).
            bl_db      = 20.0 * np.log10(np.maximum(baseline, 1e-3))
            amp_db_sc  = 20.0 * np.log10(np.maximum(amp63,    1e-3))
            dev_db     = np.abs(amp_db_sc - bl_db)             # |ΔdB| per SC
            sc_anom    = int((dev_db[active] > SC_ANOM_DB).sum())
            self.last_sc_anom = sc_anom
            crit_sc    = sc_anom > SC_ANOM_COUNT

            # ── Criterion 3: micro-motion via per-SC MACD ────────────────────
            # MACD = |fast_ema − slow_ema| per subcarrier (both in dB).
            # Why MACD instead of rolling power:
            #   • Static presence → both EMAs converge → MACD → 0 in ≈ fast_τ frames.
            #     No 40-frame false tail after the person stops moving.
            #   • Sustained oscillation (hand wave) → fast leads slow →
            #     MACD stays elevated for the full duration of the motion.
            #   • Random RF noise → both EMAs track the same mean → MACD ≈ 0.
            # amp_db_sc is already computed above — zero extra log10 calls.
            if self._micro_slow is None:
                # First Stage-2 frame: seed both at the current amplitude so
                # MACD = 0 at t=0; no false spike at the Stage-1 → Stage-2 boundary.
                self._micro_slow = amp_db_sc.copy()
                self._micro_fast = amp_db_sc.copy()
            elif not (crit_energy or crit_sc):
                # Only update during quiet frames (neither large-motion criterion
                # is active). This prevents walking/SC events from diverging the
                # EMAs — without gating, MACD stays elevated for 40-120 frames
                # after the person stops, creating a false micro-motion tail.
                # With gating, EMAs are frozen at the idle level during large
                # motion, so MACD = 0 instantly when large motion ends.
                self._micro_slow += MICRO_SLOW_ALPHA * (amp_db_sc - self._micro_slow)
                self._micro_fast += MICRO_FAST_ALPHA * (amp_db_sc - self._micro_fast)
            macd      = np.abs(self._micro_fast - self._micro_slow)
            micro_sc  = int((macd[active] > MICRO_THR_DB).sum())
            self.last_micro_sc = micro_sc
            crit_micro = micro_sc >= MICRO_SC_COUNT

            # ── Per-criterion debounce — trigger / AND-clear ─────────────────
            # Trigger : ANY single criterion fires for DEBOUNCE_FRAMES consecutive
            #   frames on its own.  Criteria cannot pool frames across each other,
            #   so an isolated RF spike (1 frame crit_energy) cannot be combined
            #   with pre-existing MACD noise (4 frames crit_micro) to reach 5.
            # Clear   : ALL criteria silent for DEBOUNCE_FRAMES consecutive frames.
            self._energy_streak = self._energy_streak + 1 if crit_energy else 0
            self._sc_streak     = self._sc_streak     + 1 if crit_sc     else 0
            self._micro_streak  = self._micro_streak  + 1 if crit_micro  else 0

            any_above = crit_energy or crit_sc or crit_micro
            self._below_streak  = self._below_streak  + 1 if not any_above else 0

            if not self.is_motion and (
                    self._energy_streak >= DEBOUNCE_FRAMES or
                    self._sc_streak     >= DEBOUNCE_FRAMES or
                    self._micro_streak  >= DEBOUNCE_FRAMES):
                self.is_motion = True
            elif self.is_motion and self._below_streak >= DEBOUNCE_FRAMES:
                self.is_motion = False

            # ── Presence-timeout adaptive reset ──────────────────────────────
            # Root cause of permanent lock: a person standing still keeps
            # crit_sc True (>SC_ANOM_COUNT subcarriers remain shifted by
            # >SC_ANOM_DB dB from the frozen empty-room baseline), and _bl_slow
            # is gated on is_motion so it never updates — a closed feedback loop.
            #
            # Fix: count consecutive frames where is_motion is True.  Once the
            # streak exceeds PRESENCE_TIMEOUT AND the energy trace has been flat
            # for PRESENCE_VAR_WIN frames (occupant genuinely static, not moving),
            # force-adopt the current amplitudes as the new room baseline.
            # All three criteria then drop to zero and the dashboard returns IDLE.
            if self.is_motion:
                self._motion_streak += 1
            else:
                self._motion_streak = 0

            if (self._motion_streak > PRESENCE_TIMEOUT
                    and len(self._energy) >= PRESENCE_VAR_WIN):
                recent_e   = np.array(
                    list(self._energy)[-PRESENCE_VAR_WIN:], dtype=np.float32)
                recent_var = float(recent_e.var())
                if recent_var < PRESENCE_VAR_THR:
                    # Instant baseline adoption: copy current amplitudes into
                    # both EMA arrays so that energy ≈ 0 on the very next frame.
                    self._bl_slow[:] = amp63
                    self._bl_fast[:] = amp63          # keep fast EMA consistent
                    # Reset MACD EMAs so micro-motion starts from zero divergence.
                    if self._micro_slow is not None:
                        self._micro_slow[:] = amp_db_sc
                        self._micro_fast[:] = amp_db_sc
                    # Unlock is_motion and clear all streak counters.
                    self.is_motion      = False
                    self._energy_streak = 0
                    self._sc_streak     = 0
                    self._micro_streak  = 0
                    self._below_streak  = 0
                    self._motion_streak = 0
                    print(f"[Wi-Sense {self.frame_count:>6}]  ** Presence-timeout RESET"
                          f"  streak>{PRESENCE_TIMEOUT}"
                          f"  var={recent_var:.5f}  energy={energy:.3f}")

            # Append AFTER threshold so this frame cannot self-cancel its spike
            self._welford.update(energy)
            self._energy_win.append(energy)
            self._energy.append(energy)

            # Motion-gated slow baseline update:
            # freeze the background model while motion is present so a person
            # standing still does not drift into the "empty room" baseline.
            if not self.is_motion:
                self._bl_slow += EMA_ALPHA_SLOW * (amp63 - self._bl_slow)

            if self.is_motion:
                self.label  = 'MOTION DETECTED'
                self.colour = '#ff4444'                         # red
            else:
                self.label  = 'IDLE / STATIC'
                self.colour = '#44ff88'                         # green

        # ---- 5. Record motion flag (co-indexed with _energy) ----------------
        self._motion.append(self.is_motion)

        # ---- 6. Console log for state transitions ---------------------------
        if self.stage is not self._prev_stage:
            if self.stage is Stage.TRACKING:
                print(f"[Wi-Sense {self.frame_count:>6}]  -- Stage 2 TRACKING started --")
            self._prev_stage = self.stage

        if self.is_motion != self._prev_motion:
            if self.is_motion:
                tag = 'HIGH ANOMALY' if self.stage is Stage.CALIBRATING else 'MOTION'
                print(f"[Wi-Sense {self.frame_count:>6}]  !! {tag} DETECTED"
                      f"   energy={self.last_energy:.3f}  thresh={self.current_thresh:.3f}")
            else:
                print(f"[Wi-Sense {self.frame_count:>6}]  -- Motion CLEARED"
                      f"   energy={self.last_energy:.3f}")
            self._prev_motion = self.is_motion

    # ── read-only views ──────────────────────────────────────────────────────

    @property
    def n(self) -> int:
        return len(self._amps)

    @property
    def active_baseline(self) -> 'np.ndarray | None':
        """Whichever baseline is currently driving detection."""
        return self._bl_slow if self._bl_slow is not None else self._bl_fast

    @property
    def active_mask(self) -> np.ndarray:
        bl = self.active_baseline
        if bl is None:
            return np.ones(N_SC, dtype=bool)
        return bl > GUARD_THR

    @property
    def amp_matrix(self) -> np.ndarray:
        """Shape (63, N) — subcarriers × packets."""
        if not self._amps:
            return np.zeros((N_SC, 1), dtype=np.float32)
        return np.column_stack(list(self._amps)).astype(np.float32)

    @property
    def dev_matrix_db(self) -> np.ndarray:
        """
        Per-subcarrier zero-mean amplitude deviation in dB.
        Uses the active baseline so Stage 2 is noticeably more sensitive.
        """
        bl   = self.active_baseline
        m    = self.amp_matrix + 1e-3
        db   = 20.0 * np.log10(m)
        if bl is not None:
            # Subtract per-row EMA baseline (in dB) for cleaner static removal
            bl_db  = 20.0 * np.log10(np.maximum(bl, 1e-3))
            db    -= bl_db[:, np.newaxis]
        else:
            db -= db.mean(axis=1, keepdims=True)
        return db

    @property
    def energy_arr(self) -> np.ndarray:
        return np.array(self._energy, dtype=np.float32)

    @property
    def motion_flags(self) -> np.ndarray:
        """Bool array co-indexed with energy_arr: True where motion was detected."""
        return np.array(self._motion, dtype=bool)

    @property
    def subcarrier_var(self) -> np.ndarray:
        return self.amp_matrix.var(axis=1)


# ════════════════════════════════════════════════════════════════════════════
# Dashboard
# ════════════════════════════════════════════════════════════════════════════

class Dashboard:

    # Colour palette
    BG          = '#0d0d1a'
    AXES_BG     = '#14142b'
    TICK        = '#b0b0cc'
    SPINE_IDLE  = '#2a2a4a'
    SPINE_CALIB = '#ccaa00'    # amber during calibration
    SPINE_MOT   = '#cc2222'    # red during motion

    def __init__(self, buf: CsiBuffer, args: argparse.Namespace) -> None:
        self.buf  = buf
        self.args = args
        self._prev_stage: 'Stage | None' = None

        matplotlib.rcParams.update({'font.size': 9, 'axes.titlesize': 10})

        self.fig = plt.figure(figsize=(15, 9), constrained_layout=True)
        self.fig.patch.set_facecolor(self.BG)

        gs = self.fig.add_gridspec(3, 1, height_ratios=[2.8, 1.8, 1.2])
        self.ax_heat   = self.fig.add_subplot(gs[0])
        self.ax_energy = self.fig.add_subplot(gs[1])
        self.ax_var    = self.fig.add_subplot(gs[2])

        self._style_axes()
        self._build_heatmap()
        self._build_energy()
        self._build_var()

        self.fig.suptitle(
            'Wi-Sense  —  Adaptive CSI Motion-Detection Dashboard',
            fontsize=14, color='#d0d0ff', fontweight='bold',
        )

    # ── styling ──────────────────────────────────────────────────────────────

    def _style_axes(self) -> None:
        for ax in (self.ax_heat, self.ax_energy, self.ax_var):
            ax.set_facecolor(self.AXES_BG)
            ax.tick_params(colors=self.TICK)
            ax.title.set_color(self.TICK)
            ax.xaxis.label.set_color(self.TICK)
            ax.yaxis.label.set_color(self.TICK)
            for sp in ax.spines.values():
                sp.set_edgecolor(self.SPINE_IDLE)

    def _set_spine_colour(self, ax, colour: str) -> None:
        for sp in ax.spines.values():
            sp.set_edgecolor(colour)

    # ── panel builders ───────────────────────────────────────────────────────

    def _build_heatmap(self) -> None:
        W = self.args.window
        self.im = self.ax_heat.imshow(
            np.zeros((N_SC, W), dtype=np.float32),
            aspect='auto', cmap='RdBu_r', origin='lower',
            vmin=-6, vmax=6, interpolation='nearest',
        )
        cb = self.fig.colorbar(self.im, ax=self.ax_heat,
                               fraction=0.018, pad=0.008)
        cb.set_label('Deviation from adaptive baseline (dB)', color=self.TICK)
        cb.ax.yaxis.set_tick_params(color=self.TICK)
        plt.setp(cb.ax.yaxis.get_ticklabels(), color=self.TICK)
        self.ax_heat.set_xlabel('Packet index  →  newest on right')
        self.ax_heat.set_ylabel('Subcarrier index  (0–62, DC removed)')

    def _build_energy(self) -> None:
        ax = self.ax_energy
        self.ln_raw,    = ax.plot([], [], color='#4488cc', lw=0.8,
                                  alpha=0.40, label='Raw motion energy')
        self.ln_smooth, = ax.plot([], [], color='#ee4444', lw=1.8,
                                  label=f'Savitzky-Golay  (win={SG_WIN})')
        self.ln_thresh, = ax.plot([], [], lw=1.4, ls='--',
                                  label='Adaptive threshold', color='#ffcc00')
        self.fill_above   = None
        self._motion_fill = None

        ax.set_xlim(0, self.args.window)
        ax.set_ylim(0, 1)
        ax.set_xlabel('Packet index')
        ax.set_ylabel('Energy (linear)')
        ax.legend(loc='upper left', facecolor=self.AXES_BG,
                  labelcolor=self.TICK, framealpha=0.85, fontsize=8)

        # Main detector readout — top-right
        self.txt_status = ax.text(
            0.993, 0.95, '…', transform=ax.transAxes,
            ha='right', va='top', fontsize=15, fontweight='bold',
            color='#ffcc00',
        )
        # Secondary stats — below status
        self.txt_stats = ax.text(
            0.993, 0.72, '', transform=ax.transAxes,
            ha='right', va='top', fontsize=8, color=self.TICK,
        )
        # Stage badge — top-left of energy panel
        self.txt_stage = ax.text(
            0.007, 0.95, '', transform=ax.transAxes,
            ha='left', va='top', fontsize=9, color='#ffcc00',
            fontfamily='monospace',
        )

    def _build_var(self) -> None:
        ax = self.ax_var
        self.bars = ax.bar(range(N_SC), np.zeros(N_SC),
                           color='#2ecc71', alpha=0.85, width=1.0)
        ax.axvspan(-0.5,  3.5, alpha=0.10, color='white')
        ax.axvspan(58.5, 62.5, alpha=0.10, color='white')
        ax.set_xlim(-1, N_SC)
        ax.set_xlabel('Subcarrier index')
        ax.set_ylabel('Variance')

    # ── render ───────────────────────────────────────────────────────────────

    def draw(self) -> None:
        buf = self.buf
        if buf.n < 2:
            return

        W   = self.args.window
        dev = buf.dev_matrix_db
        eng = buf.energy_arr
        var = buf.subcarrier_var
        N   = dev.shape[1]

        stage_changed = (buf.stage != self._prev_stage)
        self._prev_stage = buf.stage

        # ── Spine colours signal the active state ──────────────────────────
        if buf.stage is Stage.CALIBRATING:
            spine_col = self.SPINE_CALIB
        elif buf.is_motion:
            spine_col = self.SPINE_MOT
        else:
            spine_col = self.SPINE_IDLE

        self._set_spine_colour(self.ax_heat,   spine_col)
        self._set_spine_colour(self.ax_energy, spine_col)

        # ── Heatmap ---------------------------------------------------------
        if N < W:
            pad  = np.zeros((N_SC, W - N), dtype=np.float32)
            disp = np.concatenate([pad, dev], axis=1)
        else:
            disp = dev[:, -W:]

        if self.args.hide_guards:
            bl = buf.active_baseline
            if bl is not None:
                disp[bl <= GUARD_THR, :] = 0.0

        self.im.set_data(disp)
        nonzero = disp[disp != 0]
        vrange  = max(float(np.nanpercentile(np.abs(nonzero), 97)), 1.0) \
                  if len(nonzero) > 0 else 3.0
        self.im.set_clim(-vrange, vrange)

        stage_tag = '[ ] CALIBRATING' if buf.stage is Stage.CALIBRATING else '[*] TRACKING'
        self.ax_heat.set_title(
            f'CSI Amplitude-Deviation Heatmap    {stage_tag}  —  '
            f'baseline: {"FAST EMA (α=0.02)" if buf.stage is Stage.CALIBRATING else "SLOW EMA (α=0.003, motion-gated)"}'
        )

        # ── Energy trace ----------------------------------------------------
        if len(eng) >= SG_WIN:
            wl = SG_WIN
            wl = min(wl, len(eng) if len(eng) % 2 == 1 else len(eng) - 1)
            wl = max(wl, SG_POLY + 2)
            if wl % 2 == 0:
                wl += 1
            smooth = savgol_filter(eng, window_length=wl, polyorder=SG_POLY)
        else:
            smooth = eng.copy()

        thresh = buf.current_thresh
        xs     = np.arange(len(eng), dtype=np.float32)

        self.ln_raw.set_data(xs, eng)
        self.ln_smooth.set_data(xs, smooth)
        self.ln_thresh.set_data(xs, np.full_like(xs, thresh))

        # Threshold line colour follows stage
        thresh_col = self.SPINE_CALIB if buf.stage is Stage.CALIBRATING else '#ff9900'
        self.ln_thresh.set_color(thresh_col)

        # Fill above threshold
        if self.fill_above is not None:
            try:
                self.fill_above.remove()
            except ValueError:
                pass
            self.fill_above = None
        above = smooth > thresh
        if above.any():
            fill_col = '#ff9900' if buf.stage is Stage.CALIBRATING else '#ee4444'
            self.fill_above = self.ax_energy.fill_between(
                xs, thresh, smooth, where=above,
                alpha=0.30, color=fill_col, interpolate=True,
            )

        # Motion spans — single fill_between using xaxis transform so the pink
        # shading always covers the full plot height regardless of y-axis scale.
        if self._motion_fill is not None:
            try:
                self._motion_fill.remove()
            except ValueError:
                pass
            self._motion_fill = None
        flags = buf.motion_flags
        if flags.any():
            self._motion_fill = self.ax_energy.fill_between(
                np.arange(len(flags), dtype=np.float32),
                0, 1,
                where=flags,
                transform=self.ax_energy.get_xaxis_transform(),
                color='deeppink', alpha=0.80, zorder=1, linewidth=0,
            )

        emax = max(float(eng.max()) * 1.25, thresh * 1.6, 0.01)
        self.ax_energy.set_xlim(0, max(W, len(eng)))
        self.ax_energy.set_ylim(0, emax)

        cur_e = float(smooth[-1]) if len(smooth) else 0.0

        # Stage 2 stats for display
        if buf.stage is Stage.TRACKING and len(buf._energy_win) > 1:
            w   = np.asarray(buf._energy_win, dtype=np.float32)
            mu  = float(w.mean())
            sig = float(w.std())
            stats_str = (
                f'frames={buf.frame_count:,}  energy={cur_e:.3f}  '
                f'thresh={thresh:.3f}  '
                f'sc={buf.last_sc_anom}  uM={buf.last_micro_sc}/{int(buf.active_mask.sum())}  '
                f'μ={mu:.3f}  σ={sig:.3f}'
            )
        else:
            stats_str = (
                f'frames={buf.frame_count:,}  energy={cur_e:.3f}  '
                f'thresh={thresh:.3f}  '
                f'Welford μ={buf._welford.mean:.3f}  σ={buf._welford.std:.3f}'
            )

        # Status text and stage badge
        self.txt_status.set_text(buf.label)
        self.txt_status.set_color(buf.colour)
        self.txt_stats.set_text(stats_str)

        if buf.stage is Stage.CALIBRATING:
            pct        = int(buf.calib_frac * 100)
            badge_col  = '#ffcc00'
            badge_text = f'[ ] CALIBRATING  {pct}%'
            etitle     = (f'Motion Energy  —  Stage 1: HIGH-ANOMALY TRIGGER  '
                          f'(Z ≥ {CALIB_ZSCORE})    {pct}% complete')
        else:
            badge_col  = '#44ff88' if not buf.is_motion else '#ff4444'
            badge_text = '[*] TRACKING'
            etitle     = (f'Motion Energy  —  Stage 2: MAX SENSITIVITY  '
                          f'(Z ≥ {TRACK_ZSCORE})    '
                          f'active SC: {int(buf.active_mask.sum())} / {N_SC}')

        self.txt_stage.set_text(badge_text)
        self.txt_stage.set_color(badge_col)
        self.ax_energy.set_title(etitle)

        # ── Variance bars ---------------------------------------------------
        vmax = max(float(var.max()) * 1.25, 0.01)
        for bar, h, active in zip(self.bars, var, buf.active_mask):
            bar.set_height(float(h))
            bar.set_alpha(0.88 if active else 0.20)
            bar.set_color('#2ecc71' if active else '#555577')
        self.ax_var.set_ylim(0, vmax)
        self.ax_var.set_title(
            f'Per-Subcarrier Amplitude Variance  '
            f'(motion-sensitive: {int(buf.active_mask.sum())} active  '
            f'|  {buf.n:,} frames buffered)'
        )


# ════════════════════════════════════════════════════════════════════════════
# File reader — supports live tail
# ════════════════════════════════════════════════════════════════════════════

class JsonlReader:
    def __init__(self, path: Path) -> None:
        self._path = path
        self._fh   = None

    def open(self) -> 'JsonlReader':
        self._fh = open(self._path, 'r', encoding='utf-8', errors='replace')
        return self

    def close(self) -> None:
        if self._fh:
            self._fh.close()
            self._fh = None

    def drain(self):
        """Yield all currently available lines without blocking."""
        if not self._fh:
            return
        while True:
            line = self._fh.readline()
            if not line:
                break
            yield line


# ════════════════════════════════════════════════════════════════════════════
# Main
# ════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Wi-Sense CSI adaptive motion-detection dashboard',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('-f', '--file', default='live_capture.csi.jsonl',
                        help='JSONL input (default: live_capture.csi.jsonl)')
    parser.add_argument('-w', '--window', type=int, default=500,
                        help='Scrolling display window in packets (default: 500)')
    parser.add_argument('--calib', type=int, default=CALIB_FRAMES,
                        help=f'Calibration packet count (default: {CALIB_FRAMES} = 5 s @ 100 Hz)')
    parser.add_argument('--offline', action='store_true',
                        help='Read entire file at once; show static figure')
    parser.add_argument('--hide-guards', action='store_true',
                        help='Blank guard-band subcarriers in the heatmap')
    parser.add_argument('--interval', type=int, default=250,
                        help='Live-refresh interval ms (default: 250)')
    args = parser.parse_args()

    fpath = Path(args.file)
    if not fpath.exists():
        fpath = Path(__file__).parent / args.file
    if not fpath.exists():
        print(f"ERROR: file not found: {args.file!r}")
        print("  Run stream_to_ruview.py first, or pass -f <path>")
        sys.exit(1)

    print("Wi-Sense CSI Dashboard  v4  —  Two-Stage Adaptive State Machine")
    print(f"  File       : {fpath}")
    print(f"  Window     : {args.window} packets ({args.window/100:.1f} s @ 100 Hz)")
    print(f"  Stage 1    : {args.calib} frames, Z ≥ {CALIB_ZSCORE} (high-anomaly trigger)")
    print(f"  Stage 2    : Z ≥ {TRACK_ZSCORE} (sensitive tracking, motion-gated baseline)")
    print(f"  Mode       : {'OFFLINE (static)' if args.offline else 'LIVE (auto-refresh)'}")
    print()

    # Buffer: 3× display window keeps variance stats stable; at least calib+win
    cap = max(args.window * 3, args.calib + args.window)
    buf = CsiBuffer(capacity=cap, calib_frames=args.calib)

    if args.offline:
        n = 0
        with open(fpath, 'r', encoding='utf-8', errors='replace') as fh:
            for line in fh:
                r = parse_line(line)
                if r:
                    buf.push(r[0], r[1])
                    n += 1
        print(f"Loaded {n:,} packets  ->  stage={buf.stage.value}  "
              f"active_SC={int(buf.active_mask.sum())}")
        if n == 0:
            print("No valid CSI packets found — check file format.")
            sys.exit(1)
        dash = Dashboard(buf, args)
        dash.draw()
        plt.show()

    else:
        # The countdown now lives in stream_to_ruview.py — it waits 7 seconds
        # before writing the first byte to the JSONL file, so all captured
        # packets are already from an empty room by the time we read them.
        # The visualizer can therefore open the file and start tailing instantly.
        reader = JsonlReader(fpath).open()

        n = 0
        for line in reader.drain():
            r = parse_line(line)
            if r:
                buf.push(r[0], r[1])
                n += 1
        print(f"Pre-loaded {n:,} existing packets.  Tailing {fpath.name}...")

        dash = Dashboard(buf, args)

        def _animate(_):
            new = 0
            for line in reader.drain():
                r = parse_line(line)
                if r:
                    buf.push(r[0], r[1])
                    new += 1
            if new > 0 or buf.n > 0:
                dash.draw()

        ani = animation.FuncAnimation(      # noqa: F841  (kept alive by ref)
            dash.fig, _animate,
            interval=args.interval,
            cache_frame_data=False,
        )
        try:
            plt.show()
        finally:
            reader.close()


if __name__ == '__main__':
    main()
