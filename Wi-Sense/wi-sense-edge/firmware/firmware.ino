/*
 * Wi-Sense Standalone Firmware  v1.3
 * ====================================
 * ESP32 RX board — Triple-criteria CSI micro-motion detection
 * running fully off-grid as a Wi-Fi Access Point.
 *
 * Algorithm ported verbatim from:
 *   research_reference/plot_my_csi.py  (CsiBuffer.push)
 *
 * Compile target : ESP32 Arduino core  (board: "ESP32 Dev Module")
 * CPU Frequency  : 240 MHz
 * Partition      : Default (4MB, no OTA needed)
 * Libraries      : ESP32 Arduino core only — no external installs
 *
 * Wiring
 * ------
 *   Built-in LED on GPIO 2  (most ESP32 dev boards)
 *   TX board must connect to "Wi-Sense-Net" AP and send UDP/ping packets
 *   so the RX chip receives Wi-Fi frames it can measure CSI on.
 *
 * Network
 * -------
 *   AP  : Wi-Sense-Net  (open, no password)
 *   IP  : 192.168.4.1
 *   UI  : http://192.168.4.1  (poll every 50 ms, full-screen Fuchsia/Green)
 */

// ── Includes ────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>          // esp_wifi_set_csi / wifi_csi_info_t
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ============================================================================
// ENGINEERING PARAMETER LOG  —  firmware v1.3
// ─────────────────────────────────────────────────────────────────────────────
// All constants below are derived from a 70-second live-hardware calibration
// capture on ESP32-D0WD-V3 rev3.1, TX beacon at 100 Hz UDP, indoor empty room.
//
// Observed Stage-2 idle statistics (frames 500–5400):
//   Energy (typical per-frame range)  : 0.30 – 0.90 amplitude units
//   Energy (100-frame sample peak)    : 1.258 (frame 4400)
//   Energy (isolated transient max)   : 3.187 (frame 4700, 1 frame only)
//   SC_Anom (>2.5 dB, empty room)    : 0–2 SCs  (max)
//   MACD anomalies (empty room)       : 0 SCs  — completely silent
//   Adaptive threshold (settled)      : 2.03 – 4.65
//   Maximum E_streak observed         : 1  (never consecutive breaches)
//
// Parameter change log (v1.2 → v1.3):
//   TRACK_ZSCORE      2.5 → 3.0   +20% threshold margin vs RF fading events
//   ENERGY_HARD_FLOOR 1.0 → 1.5   above 99th-pctile of observed idle samples
//   SC_ANOM_DB        2.5 → 3.0   empty-room SC_Anom drops from 0–2 to 0–1
//   SC_ANOM_COUNT      15 → 10    tighter dB + lower count = better sensitivity
//   PRESENCE_VAR_THR  0.05→ 0.30  live RF noise variance 0.10–0.25; 0.05 too tight
// ============================================================================

// ============================================================================
// ALGORITHM CONSTANTS  —  engineered for live ESP32 RF environment
// ============================================================================
static const int   N_SC_TOTAL       = 64;
static const int   DC_IDX           = 32;    // DC bin removed from 64→63
static const int   N_SC             = 63;
static const float GUARD_THR        = 0.5f;  // EMA < this → guard/null subcarrier

// Stage 1 — Calibration
// bl_fast time-constant: τ = 1/0.02 = 50 frames.  500 = 10τ → 99.995% converged.
static const int   CALIB_FRAMES     = 500;
static const float EMA_ALPHA_FAST   = 0.02f;
static const float CALIB_ZSCORE     = 4.5f;
static const float BOOTSTRAP_THRESH = 5.0f;  // absolute guard for n < 20 packets

// Stage 2 — Tracking
// bl_slow τ = 333 frames ≈ 3.3 s.  Slow enough to NOT track a moving person
// during brief pauses, but fast enough to adapt to room temperature / humidity
// drift over minutes.
static const float EMA_ALPHA_SLOW   = 0.003f;

// TRACK_ZSCORE = 3.0: at the 99.87th percentile of a Gaussian.
// Live data: isolated spike max = 3.187 (1 frame); settled thresh = 2.03–4.65.
// Raising from 2.5→3.0 lifts the threshold by ~20%, providing additional margin
// against multi-frame RF coherence fading while keeping motion sensitivity intact
// (actual motion energy is typically 5–50 amplitude units).
static const float TRACK_ZSCORE     = 3.0f;
static const int   ENERGY_STAT_WIN  = 400;   // rolling window ≈ 4 s of Stage-2 data

// DEBOUNCE_FRAMES = 5: 50 ms at 100 Hz.
// From live data: E_streak never exceeded 1 in 70 s of empty room.
// 5-frame requirement makes the false-trigger probability from independent
// fading events astronomically small (5 independent exceedances required).
static const int   DEBOUNCE_FRAMES  = 5;

// ── Criterion 2: per-SC dB anomaly ───────────────────────────────────────
// SC_ANOM_DB = 3.0 dB (raised from 2.5):
//   Live data at 2.5 dB: max empty-room SC_Anom = 2 SCs.
//   At 3.0 dB: expected empty-room SC_Anom = 0–1 SCs.
//   Real motion events (person entering/waving): 20–63 SCs shift >3.0 dB.
// SC_ANOM_COUNT = 10 (lowered from 15):
//   Combined effect with raised dB: more selective per-SC gate AND lower
//   count threshold gives better sensitivity to subtle motion (10–20 SCs
//   affected) while maintaining a 10× noise floor margin (0–1 vs ≥11 required).
static const float SC_ANOM_DB       = 3.0f;
static const int   SC_ANOM_COUNT    = 10;    // fires when sc_anom > 10  (≥ 11)

// ── Criterion 3: micro-motion MACD ───────────────────────────────────────
// Live data: MACD completely silent (Micro_Anom = 0) throughout 70-second
// empty-room capture.  Original Python parameters are well-calibrated; no
// change needed.
static const float MICRO_SLOW_ALPHA = 0.025f; // τ ≈ 40 frames
static const float MICRO_FAST_ALPHA = 0.10f;  // τ ≈ 10 frames
static const float MICRO_THR_DB     = 0.6f;
static const int   MICRO_SC_COUNT   = 3;

// ── Presence-timeout adaptive reset ──────────────────────────────────────
// PRESENCE_VAR_THR = 0.30 (raised from 0.05):
//   Live RF noise floor produces energy variance 0.10–0.25 over 100 frames
//   in an empty room.  0.05 was unreachable even for a perfectly static presence.
//   0.30 accommodates RF noise while remaining well below active-motion variance
//   (typically 1.0–10.0).  Sanity test uses constant synthetic signal → var=0 ✓
static const int   PRESENCE_TIMEOUT = 300;    // frames (~3 s @ 100 Hz)
static const int   PRESENCE_VAR_WIN = 100;    // ~1 s energy window for flatness test
static const float PRESENCE_VAR_THR = 0.30f;

// Stage-2 warmup gate: suppress all criteria for this many frames after the
// Stage-1→2 transition.  ew_buf fills with real Stage-2 idle energies;
// bl_slow adapts 14% toward actual amplitudes; MACD EMAs settle.
// 50 frames ensures sanity tests still pass (first motion frame at stage2_frames=51).
static const int   STAGE2_WARMUP       = 50;

// Minimum adaptive threshold floor.  ew_buf is empty at Stage-2 start
// (Stage-1 energies deliberately excluded).  Floor protects the early
// settling period before ew_count accumulates enough samples.
static const float STAGE2_THRESH_FLOOR = 0.5f;

// Hard absolute energy floor for Criterion 1.
// ENERGY_HARD_FLOOR = 1.5 (raised from 1.0):
//   100-frame sample peak in empty room: 1.258 (frame 4400).
//   Per-frame isolated max: 3.187 (single frame, handled by debounce).
//   1.5 is above the 99th percentile of sustained idle energy while being
//   far below the minimum expected motion signal (~3–5 units at any range).
//   During early Stage-2 (thresh still settling), this prevents sub-1.5
//   fluctuations from counting toward crit_e even if thresh is transiently low.
static const float ENERGY_HARD_FLOOR   = 1.5f;

// ============================================================================
// NETWORK CONFIG
// ============================================================================
static const char*    AP_SSID = "Wi-Sense-Net";
static const char*    AP_PASS = "";            // open network
static const IPAddress AP_IP  (192, 168, 4, 1);
static const IPAddress AP_GW  (192, 168, 4, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);
static const int  LED_PIN = 2;

// ============================================================================
// DETECTION STATE
// ============================================================================
enum Stage { STAGE_CALIBRATING, STAGE_TRACKING };

// Baselines
static float bl_fast[N_SC];    // fast EMA — always updated
static float bl_slow[N_SC];    // slow EMA — motion-gated
static bool  bl_init   = false;
static bool  slow_init = false;

// Micro-motion MACD per-SC EMAs (dB domain)
static float micro_slow[N_SC];
static float micro_fast[N_SC];
static bool  micro_init = false;

// Stage-1 Welford online statistics (mean + variance of energy)
static double wf_n    = 0.0;
static double wf_mean = 0.0;
static double wf_M2   = 0.0;

// Stage-2 rolling energy window — circular buffer, ENERGY_STAT_WIN slots
static float  ew_buf  [ENERGY_STAT_WIN];
static int    ew_head  = 0;
static int    ew_count = 0;

// Energy history for presence-timeout variance — circular, PRESENCE_VAR_WIN slots
static float  eh_buf  [PRESENCE_VAR_WIN];
static int    eh_head  = 0;
static int    eh_count = 0;

// Per-criterion independent debounce streak counters
static int  e_streak     = 0;
static int  sc_streak    = 0;
static int  mi_streak    = 0;
static int  below_streak = 0;
static int  motion_streak = 0;   // consecutive Stage-2 frames with is_motion=true
static int  stage2_frames = 0;   // frames elapsed since Stage-2 transition

// Global detection state
static int   frame_count = 0;
static bool  is_motion   = false;
static Stage stage       = STAGE_CALIBRATING;

// Volatile diagnostics read by web handler on Core 0
static volatile float g_energy   = 0.0f;
static volatile float g_thresh   = 0.0f;
static volatile int   g_sc_anom  = 0;
static volatile int   g_micro_sc = 0;
static volatile bool  g_motion   = false;
static volatile int   g_frame    = 0;
static volatile Stage g_stage    = STAGE_CALIBRATING;

// Working amplitude array — written before process_frame()
static float amp[N_SC];

// ============================================================================
// CSI DOUBLE BUFFER
// ============================================================================
static const int CSI_BYTES = 128;   // 64 pairs × 2 bytes
static int8_t  csi_buf_a[CSI_BYTES];
static int8_t  csi_buf_b[CSI_BYTES];
static int8_t* csi_wr = csi_buf_a;  // callback writes here
static int8_t* csi_rd = csi_buf_b;  // task reads here
static volatile bool    csi_ready = false;
static SemaphoreHandle_t csi_sem;

// ============================================================================
// WEB SERVER
// ============================================================================
static WebServer server(80);

// ============================================================================
// HELPERS — Welford online statistics (Stage 1 threshold)
// ============================================================================
static void wf_update(double x) {
    wf_n++;
    double d1 = x - wf_mean;
    wf_mean  += d1 / wf_n;
    wf_M2    += d1 * (x - wf_mean);
}
static double wf_std() {
    return (wf_n < 2) ? 0.0 : sqrt(wf_M2 / (wf_n - 1));
}

// ============================================================================
// HELPERS — Rolling energy window (Stage 2 threshold)
// ============================================================================
static void ew_push(float e) {
    ew_buf[ew_head] = e;
    ew_head = (ew_head + 1) % ENERGY_STAT_WIN;
    if (ew_count < ENERGY_STAT_WIN) ew_count++;
}
static void ew_stats(float* mu, float* sigma) {
    if (ew_count == 0) { *mu = 0.0f; *sigma = 0.0f; return; }
    double s = 0.0, s2 = 0.0;
    for (int i = 0; i < ew_count; i++) s += ew_buf[i];
    double m = s / ew_count;
    for (int i = 0; i < ew_count; i++) { double d = ew_buf[i] - m; s2 += d*d; }
    *mu    = (float)m;
    *sigma = (float)sqrt(s2 / ew_count);
}

// ============================================================================
// HELPERS — Energy history (presence-timeout variance check)
// ============================================================================
static void eh_push(float e) {
    eh_buf[eh_head] = e;
    eh_head = (eh_head + 1) % PRESENCE_VAR_WIN;
    if (eh_count < PRESENCE_VAR_WIN) eh_count++;
}
static float eh_var() {
    if (eh_count < 2) return 1e9f;
    double s = 0.0;
    for (int i = 0; i < eh_count; i++) s += eh_buf[i];
    double m = s / eh_count, s2 = 0.0;
    for (int i = 0; i < eh_count; i++) { double d = eh_buf[i] - m; s2 += d*d; }
    return (float)(s2 / eh_count);
}

// ============================================================================
// AMPLITUDE EXTRACTION  (matches _parse_hex in plot_my_csi.py)
//   buf[2k]   = imaginary part of subcarrier k  (signed int8)
//   buf[2k+1] = real      part of subcarrier k  (signed int8)
//   amplitude = sqrt(I² + Q²),  DC bin (k=32) skipped → 63 values
// ============================================================================
static void extract_amp(const int8_t* buf) {
    int ai = 0;
    for (int k = 0; k < N_SC_TOTAL; k++) {
        if (k == DC_IDX) continue;
        float im = (float)buf[2 * k];
        float re = (float)buf[2 * k + 1];
        amp[ai++] = sqrtf(im*im + re*re);
    }
}

// ============================================================================
// CORE DETECTION  —  one call per CSI frame, amp[] pre-populated
// Direct C++ port of CsiBuffer.push() in plot_my_csi.py
// ============================================================================
static void process_frame() {
    frame_count++;

    // ── 1. Fast EMA baseline (always updated — used in Stage 1) ───────────
    if (!bl_init) {
        for (int i = 0; i < N_SC; i++) bl_fast[i] = amp[i];
        bl_init = true;
    } else {
        for (int i = 0; i < N_SC; i++)
            bl_fast[i] += EMA_ALPHA_FAST * (amp[i] - bl_fast[i]);
    }

    // ── 2. Select active baseline ─────────────────────────────────────────
    // Stage 1 uses bl_fast (updating); Stage 2 uses bl_slow (motion-gated).
    float* baseline;
    if (frame_count < CALIB_FRAMES) {
        baseline = bl_fast;
    } else {
        if (!slow_init) {
            // Exact transition: initialise slow EMA from current fast EMA
            for (int i = 0; i < N_SC; i++) bl_slow[i] = bl_fast[i];
            slow_init = true;
        }
        baseline = bl_slow;
    }

    // ── 3. Energy = mean |amp - baseline| over active subcarriers ─────────
    //   "Active" = baseline[i] > GUARD_THR (not a guard/null SC)
    float energy  = 0.0f;
    int   n_act   = 0;
    for (int i = 0; i < N_SC; i++) {
        if (baseline[i] > GUARD_THR) {
            energy += fabsf(amp[i] - baseline[i]);
            n_act++;
        }
    }
    energy = (n_act > 0) ? energy / n_act : 0.0f;

    // ── 4. State machine ──────────────────────────────────────────────────
    if (frame_count < CALIB_FRAMES) {
        // ═══════════════════════════════════════════════════════════════════
        // STAGE 1 — CALIBRATING
        // Instant trigger (no debounce): any anomaly fires immediately.
        // Threshold: absolute bootstrap guard for n<20, Welford z-score after.
        // ═══════════════════════════════════════════════════════════════════
        stage = STAGE_CALIBRATING;

        float thresh = (wf_n < 20)
            ? BOOTSTRAP_THRESH
            : (float)(wf_mean + CALIB_ZSCORE * fmax(wf_std(), 1e-6));

        // Threshold computed BEFORE updating stats (frame can't cancel itself)
        is_motion = (energy > thresh);
        wf_update((double)energy);
        // *** PRIMARY BUG FIX ***
        // Stage-1 energies are intentionally NOT pushed to ew_buf.
        // ew_buf is the rolling window that drives the Stage-2 adaptive threshold.
        // On live hardware, bl_fast converges in ~50 frames (τ=50).  Frames 50-499
        // of Stage 1 all push near-zero energy → by Stage-2 start, the 400-sample
        // window is filled with zeros → mu≈0, sigma≈0 → thresh≈STAGE2_THRESH_FLOOR.
        // Live Stage-2 idle energy is 2-4 amplitude units, which immediately exceeds
        // 0.5 → continuous false crit_e triggers.
        // Fix: keep ew_buf exclusively for Stage-2 samples so that after the
        // STAGE2_WARMUP period, the threshold is always derived from real idle-room
        // CSI statistics rather than converged-baseline Stage-1 noise.
        eh_push(energy);

        g_thresh = thresh;

    } else {
        // ═══════════════════════════════════════════════════════════════════
        // STAGE 2 — TRACKING
        // Triple-criteria OR gate with independent per-criterion debounce.
        // ═══════════════════════════════════════════════════════════════════
        stage = STAGE_TRACKING;
        stage2_frames++;

        // Threshold from rolling 400-sample window (computed BEFORE pushing).
        // BUG FIX: ew_buf enters Stage-2 filled with near-zero Stage-1 energies
        // (bl_fast converges in ~50 frames, so frames 50-500 all push ≈0).
        // This makes mu≈0, sigma≈0, thresh≈0 — any real CSI noise fires crit_e.
        // Apply STAGE2_THRESH_FLOOR as a minimum to prevent this false trigger.
        float mu, sigma;
        ew_stats(&mu, &sigma);
        float thresh = fmaxf(mu + TRACK_ZSCORE * fmaxf(sigma, 1e-6f),
                             STAGE2_THRESH_FLOOR);
        g_thresh = thresh;

        // BUG FIX: Stage-2 warmup gate — suppress all criteria for the first
        // STAGE2_WARMUP frames after the Stage-1→2 transition.  During warmup:
        //   • bl_slow has time to adapt toward real Stage-2 amplitudes
        //   • ew_buf accumulates actual Stage-2 energy samples, raising thresh
        //   • MACD EMAs settle to their initial seeded values
        // Without this guard, any tiny CSI variation fires all 3 criteria
        // on frame 501 because thresh is still near-zero from Stage-1 data.
        const bool warmed = (stage2_frames > STAGE2_WARMUP);

        // Diagnostic: print first 5 Stage-2 frames so issues are visible over serial
        if (stage2_frames <= 5) {
            Serial.printf("[S2 frame %3d] E=%.4f  thresh=%.4f  warm=%d\n",
                          stage2_frames, energy, thresh, (int)warmed);
        }

        // ── Criterion 1: energy breach ─────────────────────────────────
        // Dual-gate: energy must exceed BOTH the adaptive threshold AND the
        // hard absolute floor.  The adaptive threshold self-calibrates from
        // the rolling ew_buf; the floor ensures crit_e never fires on
        // sub-unit quantisation noise even if the threshold is transiently low.
        bool crit_e = warmed && (energy > thresh) && (energy > ENERGY_HARD_FLOOR);

        // ── Criterion 2: per-SC dB anomaly count ───────────────────────
        //   Count active SCs where |amp_dB − baseline_dB| > SC_ANOM_DB.
        //   Catches localised beam-path shifts missed by the mean-energy metric.
        int sc_anom = 0;
        for (int i = 0; i < N_SC; i++) {
            if (baseline[i] <= GUARD_THR) continue;
            float bl_db  = 20.0f * log10f(fmaxf(baseline[i], 1e-3f));
            float am_db  = 20.0f * log10f(fmaxf(amp[i],      1e-3f));
            if (fabsf(am_db - bl_db) > SC_ANOM_DB) sc_anom++;
        }
        bool crit_sc = warmed && (sc_anom > SC_ANOM_COUNT);  // strictly > 15  (≥ 16)

        // ── Criterion 3: micro-motion MACD per subcarrier ──────────────
        //   MACD = |fast_EMA_dB − slow_EMA_dB|.
        //   Static presence → both converge → MACD→0 (no false tail).
        //   Sustained oscillation → fast leads slow → MACD stays elevated.
        //   Gating: EMAs frozen when crit_e||crit_sc is True to prevent
        //   large-motion events diverging the EMAs and creating a 40-120 frame
        //   false micro-motion tail after the person stops.
        if (!micro_init) {
            // First Stage-2 frame: seed both at current dB → MACD = 0
            for (int i = 0; i < N_SC; i++) {
                float db = 20.0f * log10f(fmaxf(amp[i], 1e-3f));
                micro_slow[i] = db;
                micro_fast[i] = db;
            }
            micro_init = true;
        } else if (!crit_e && !crit_sc) {
            // Quiet frame — update EMAs
            for (int i = 0; i < N_SC; i++) {
                float db = 20.0f * log10f(fmaxf(amp[i], 1e-3f));
                micro_slow[i] += MICRO_SLOW_ALPHA * (db - micro_slow[i]);
                micro_fast[i] += MICRO_FAST_ALPHA * (db - micro_fast[i]);
            }
        }
        int micro_sc = 0;
        for (int i = 0; i < N_SC; i++) {
            // Guard 1: baseline must be above GUARD_THR (not a null/guard SC)
            if (baseline[i] <= GUARD_THR) continue;
            // Guard 2: current amplitude must be non-trivial.
            // A SC with amp < 1.0 sits in the int8 quantisation noise floor;
            // dB swings from ±1 LSB become huge (e.g. 0.5→1.5 = +9.5 dB),
            // creating false MACD hits that have nothing to do with motion.
            if (amp[i] < 1.0f) continue;
            if (fabsf(micro_fast[i] - micro_slow[i]) > MICRO_THR_DB)
                micro_sc++;
        }
        bool crit_mi = warmed && (micro_sc >= MICRO_SC_COUNT);

        g_sc_anom  = sc_anom;
        g_micro_sc = micro_sc;

        // ── Per-criterion independent debounce ──────────────────────────
        // Each criterion must sustain for DEBOUNCE_FRAMES on its own.
        // Cross-criterion accumulation is impossible: 4 frames of crit_mi
        // noise + 1 frame of crit_e spike cannot pool to reach 5 — each
        // streak counter resets to 0 whenever its own criterion is False.
        e_streak  = crit_e  ? e_streak  + 1 : 0;
        sc_streak = crit_sc ? sc_streak + 1 : 0;
        mi_streak = crit_mi ? mi_streak + 1 : 0;

        bool any_above = crit_e || crit_sc || crit_mi;
        below_streak = any_above ? 0 : below_streak + 1;

        // Trigger: any one criterion reaches DEBOUNCE_FRAMES independently
        if (!is_motion && (e_streak  >= DEBOUNCE_FRAMES ||
                           sc_streak >= DEBOUNCE_FRAMES ||
                           mi_streak >= DEBOUNCE_FRAMES)) {
            is_motion = true;
            Serial.printf("[%6d] !! MOTION  E=%.3f/%.3f  sc=%d  micro=%d\n",
                          frame_count, energy, thresh, sc_anom, micro_sc);
        }
        // Clear: ALL criteria silent for DEBOUNCE_FRAMES
        else if (is_motion && below_streak >= DEBOUNCE_FRAMES) {
            is_motion = false;
            Serial.printf("[%6d] -- CLEARED  E=%.3f\n", frame_count, energy);
        }

        // ── Presence-timeout adaptive reset ────────────────────────────
        // Problem: person stands still → crit_sc stays True (their body
        // is shifted >SC_ANOM_DB from the empty-room bl_slow which is frozen
        // by the motion gate) → permanent presence lock.
        // Fix: once is_motion has been True for >PRESENCE_TIMEOUT consecutive
        // frames AND energy has been dead flat (variance < PRESENCE_VAR_THR),
        // force-adopt current amplitudes as the new room baseline, instantly
        // dropping all criteria to zero.
        if (is_motion) motion_streak++;
        else           motion_streak = 0;

        if (motion_streak > PRESENCE_TIMEOUT && eh_count >= PRESENCE_VAR_WIN) {
            float pvar = eh_var();
            if (pvar < PRESENCE_VAR_THR) {
                for (int i = 0; i < N_SC; i++) {
                    bl_slow[i] = amp[i];
                    bl_fast[i] = amp[i];
                    float db = 20.0f * log10f(fmaxf(amp[i], 1e-3f));
                    micro_slow[i] = db;
                    micro_fast[i] = db;
                }
                is_motion    = false;
                e_streak     = 0;
                sc_streak    = 0;
                mi_streak    = 0;
                below_streak = 0;
                motion_streak = 0;
                Serial.printf("[%6d] ** Presence-timeout RESET  var=%.5f  E=%.3f\n",
                              frame_count, pvar, energy);
            }
        }

        // Stats update AFTER threshold so current spike can't self-cancel
        wf_update((double)energy);
        ew_push(energy);
        eh_push(energy);

        // Motion-gated slow baseline: freeze while is_motion to prevent
        // a standing person from becoming the new "empty room" baseline.
        if (!is_motion) {
            for (int i = 0; i < N_SC; i++)
                bl_slow[i] += EMA_ALPHA_SLOW * (amp[i] - bl_slow[i]);
        }
    }

    // Publish diagnostics for web handler (volatile writes, Core 0 reads)
    g_energy = energy;
    g_motion = is_motion;
    g_stage  = stage;
    g_frame  = frame_count;
}

// ============================================================================
// FULL STATE RESET  (used between sanity tests and before live operation)
// ============================================================================
static void reset_state() {
    bl_init      = false;
    slow_init    = false;
    micro_init   = false;
    wf_n = wf_mean = wf_M2 = 0.0;
    ew_head = ew_count = 0;
    eh_head = eh_count = 0;
    e_streak = sc_streak = mi_streak = below_streak = motion_streak = 0;
    stage2_frames = 0;
    frame_count = 0;
    is_motion   = false;
    stage       = STAGE_CALIBRATING;
    g_motion    = false;
    g_stage     = STAGE_CALIBRATING;
    g_frame     = 0;
    g_energy    = 0.0f;
    g_thresh    = 0.0f;
    g_sc_anom   = 0;
    g_micro_sc  = 0;
}

// ============================================================================
// SYNTHETIC PUSH  (sanity tests: set amp[] directly, skip CSI callback)
// ============================================================================
static void push_synthetic(const float* a) {
    memcpy(amp, a, sizeof(float) * N_SC);
    process_frame();
}

// ============================================================================
// BOOT SANITY TESTS  —  Tests A / B / C / D / E
// ============================================================================
static float t_idle  [N_SC];  // baseline: amps 5–14, all above GUARD_THR
static float t_motion[N_SC];  // large motion: +30 raw units on every SC
static float t_sc_d  [N_SC];  // Test D: SC-criterion isolation
// t_sc_d: exactly SC_ANOM_COUNT+1 = 11 SCs amplified by ×1.5
//   20·log10(1.5) ≈ 3.52 dB  >  SC_ANOM_DB = 3.0 dB  → each of the 11 SCs flagged
//   Energy = Σ|amp-bl| / 63 ≈ 0.79 amplitude units  <  ENERGY_HARD_FLOOR = 1.5
//   ⟹ crit_e = False (hard floor blocks it)
//   ⟹ crit_sc = True  (11 > SC_ANOM_COUNT=10)
//   Lets Test D verify the SC criterion fires INDEPENDENTLY of the energy criterion.

static bool run_sanity_tests() {
    Serial.println("[Boot] Building synthetic test signals...");

    // ── Signal construction ───────────────────────────────────────────────
    for (int i = 0; i < N_SC; i++) {
        t_idle  [i] = 5.0f + (float)(i % 10); // 5–14, all > GUARD_THR=0.5
        t_motion[i] = t_idle[i] + 30.0f;       // +30 → large energy spike
    }
    // t_sc_d: amplify exactly SC_ANOM_COUNT+1 = 11 SCs by 1.5×
    // 20·log10(1.5)=3.52 dB > SC_ANOM_DB=3.0; energy≈0.79 < ENERGY_HARD_FLOOR=1.5
    for (int i = 0; i < N_SC; i++)         t_sc_d[i] = t_idle[i];
    for (int i = 0; i < SC_ANOM_COUNT + 1; i++) t_sc_d[i] *= 1.5f;

    bool ok = true;

    // ──────────────────────────────────────────────────────────────────────
    // Test A — Single energy spike must NOT trigger (debounce gate)
    // Validates that the independent per-criterion streak counters prevent
    // a single above-threshold frame from firing is_motion.
    // ──────────────────────────────────────────────────────────────────────
    Serial.println("[Test A] Single energy spike must not trigger (debounce)...");
    reset_state();
    for (int f = 0; f < CALIB_FRAMES + 50; f++) push_synthetic(t_idle);
    push_synthetic(t_motion);  // one spike
    bool a_ok = !is_motion;
    Serial.printf("  [%s] is_motion=%d  e_streak=%d  sc_streak=%d  mi_streak=%d\n",
        a_ok ? "PASS" : "FAIL", (int)is_motion, e_streak, sc_streak, mi_streak);
    ok &= a_ok;

    // ──────────────────────────────────────────────────────────────────────
    // Test B — Exactly DEBOUNCE_FRAMES=5 consecutive spikes MUST trigger
    // Validates that sustained motion (5 × 50 ms = 250 ms) fires is_motion.
    // Energy of t_motion = 30.0 >> ENERGY_HARD_FLOOR=1.5 and >> thresh≈0.5
    // ──────────────────────────────────────────────────────────────────────
    Serial.println("[Test B] 5 consecutive energy spikes must trigger...");
    reset_state();
    for (int f = 0; f < CALIB_FRAMES + 50; f++) push_synthetic(t_idle);
    for (int f = 0; f < DEBOUNCE_FRAMES;   f++) push_synthetic(t_motion);
    bool b_ok = is_motion && (e_streak == DEBOUNCE_FRAMES);
    Serial.printf("  [%s] is_motion=%d  e_streak=%d  (need %d)\n",
        b_ok ? "PASS" : "FAIL", (int)is_motion, e_streak, DEBOUNCE_FRAMES);
    ok &= b_ok;

    // ──────────────────────────────────────────────────────────────────────
    // Test C — Presence-timeout: sustained static motion → force reset
    // Scenario: stuck in MOTION for >PRESENCE_TIMEOUT frames with flat energy
    // (variance ≈ 0 since t_motion is constant) → bl_slow/MACD force-adopted.
    // After reset: is_motion=false, motion_streak=0.
    // ──────────────────────────────────────────────────────────────────────
    Serial.println("[Test C] Presence-timeout reset after static lock...");
    reset_state();
    for (int f = 0; f < CALIB_FRAMES + 50;     f++) push_synthetic(t_idle);
    for (int f = 0; f < PRESENCE_TIMEOUT + 10; f++) push_synthetic(t_motion);
    bool c_ok = !is_motion && (motion_streak == 0);
    Serial.printf("  [%s] is_motion=%d  motion_streak=%d  energy=%.4f\n",
        c_ok ? "PASS" : "FAIL", (int)is_motion, motion_streak, (float)g_energy);
    ok &= c_ok;

    // ──────────────────────────────────────────────────────────────────────
    // Test D — SC criterion fires independently of energy criterion
    // t_sc_d shifts SC_ANOM_COUNT+1=11 SCs by 3.52 dB (>SC_ANOM_DB=3.0)
    // while keeping overall energy ≈ 0.79 < ENERGY_HARD_FLOOR=1.5.
    // crit_e must be False throughout; crit_sc must trigger within 5 frames.
    // ──────────────────────────────────────────────────────────────────────
    Serial.println("[Test D] SC criterion fires independently (crit_e blocked)...");
    reset_state();
    for (int f = 0; f < CALIB_FRAMES + 50; f++) push_synthetic(t_idle);
    for (int f = 0; f < DEBOUNCE_FRAMES;   f++) push_synthetic(t_sc_d);
    bool d_ok = is_motion && (sc_streak == DEBOUNCE_FRAMES) && (e_streak == 0);
    Serial.printf("  [%s] is_motion=%d  sc_streak=%d  e_streak=%d  "
                  "sc_anom=%d (need >%d)\n",
        d_ok ? "PASS" : "FAIL", (int)is_motion,
        sc_streak, e_streak, g_sc_anom, SC_ANOM_COUNT);
    ok &= d_ok;

    // ──────────────────────────────────────────────────────────────────────
    // Test E — Motion clears after DEBOUNCE_FRAMES consecutive idle frames
    // Immediately follows Test D (is_motion=true from SC trigger).
    // Push 5 idle frames → all criteria silent → below_streak reaches 5 → clear.
    // ──────────────────────────────────────────────────────────────────────
    Serial.println("[Test E] Motion clears after idle debounce...");
    // (continues state from Test D where is_motion=true)
    for (int f = 0; f < DEBOUNCE_FRAMES; f++) push_synthetic(t_idle);
    bool e_ok = !is_motion && (below_streak == DEBOUNCE_FRAMES);
    Serial.printf("  [%s] is_motion=%d  below_streak=%d  (need %d)\n",
        e_ok ? "PASS" : "FAIL", (int)is_motion, below_streak, DEBOUNCE_FRAMES);
    ok &= e_ok;

    Serial.printf("[Boot] %s\n\n", ok ? "ALL TESTS PASSED" : "TESTS FAILED");
    return ok;
}

// ============================================================================
// CSI CALLBACK  (Wi-Fi task context — NOT a hardware ISR)
// Copy incoming CSI buffer into write-side of double buffer and flag.
// If processing task currently holds the mutex we drop this frame (100 Hz →
// missing one frame is harmless).
// ============================================================================
static void csi_callback(void* ctx, wifi_csi_info_t* data) {
    if (!data || !data->buf || data->len < CSI_BYTES) return;
    if (xSemaphoreTake(csi_sem, 0) == pdTRUE) {
        memcpy(csi_wr, data->buf, CSI_BYTES);
        csi_ready = true;
        xSemaphoreGive(csi_sem);
    }
}

// ============================================================================
// PROCESSING TASK  (pinned to Core 1, 100 Hz tick)
// Swaps double-buffer then runs detection pipeline on the fresh CSI frame.
// Core 0 is left free for the Wi-Fi stack + web server.
// ============================================================================
static void processing_task(void* pv) {
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10)); // 100 Hz

        bool got = false;
        if (xSemaphoreTake(csi_sem, 0) == pdTRUE) {
            if (csi_ready) {
                // Swap write/read pointers so callback can fill the other half
                int8_t* tmp = csi_rd;
                csi_rd      = csi_wr;
                csi_wr      = tmp;
                csi_ready   = false;
                got         = true;
            }
            xSemaphoreGive(csi_sem);
        }

        if (got) {
            extract_amp(csi_rd);
            process_frame();

            // Serial telemetry every 100 frames (~1 s)
            // Format requested for live diagnosis: clearly shows WHICH criterion
            // is running hot so the root cause of any false trigger is obvious.
            if (frame_count % 100 == 0) {
                Serial.printf("[%6d] %-6s  "
                              "DEBUG: Energy=%.3f Thresh=%.3f "
                              "SC_Anom=%d Micro_Anom=%d "
                              "Streaks(E=%d, SC=%d, M=%d)\n",
                    frame_count,
                    is_motion ? "MOTION" : "IDLE",
                    (float)g_energy, (float)g_thresh,
                    g_sc_anom, g_micro_sc,
                    e_streak, sc_streak, mi_streak);
            }
        }
    }
}

// ============================================================================
// WEB SERVER — HTML dashboard
// Full-screen Fuchsia (deeppink) on MOTION, calm Green on IDLE.
// Polls /api/status every 50 ms via fetch.
// ============================================================================
static const char HTML_PAGE[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Wi-Sense</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box}
  body{min-height:100vh;display:flex;flex-direction:column;
       align-items:center;justify-content:center;
       font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;
       background:#22c55e;transition:background .15s}
  #status{font-size:clamp(2.4rem,10vw,5.5rem);font-weight:900;color:#fff;
          text-align:center;text-shadow:0 3px 12px rgba(0,0,0,.25);
          letter-spacing:.04em}
  #detail{margin-top:1.2rem;font-size:clamp(.85rem,3vw,1.1rem);
          color:rgba(255,255,255,.88);text-align:center}
  #info  {margin-top:.5rem;font-size:.8rem;color:rgba(255,255,255,.5)}
</style></head>
<body id="body">
  <div id="status">Connecting&hellip;</div>
  <div id="detail"></div>
  <div id="info"></div>
  <script>
    const body=document.getElementById('body');
    const st=document.getElementById('status');
    const dt=document.getElementById('detail');
    const inf=document.getElementById('info');
    async function poll(){
      try{
        const d=await(await fetch('/api/status')).json();
        if(d.stage==='CALIBRATING'){
          body.style.background='#3b82f6';
          st.textContent='Calibrating…';
          dt.textContent='Building baseline — please keep the room still';
        }else if(d.motion){
          body.style.background='deeppink';
          st.textContent='🚨 MOTION DETECTED';
          dt.textContent='E = '+d.energy.toFixed(3)+
            ' thr = '+d.thresh.toFixed(3)+
            ' sc = '+d.sc+
            ' μΔ = '+d.micro;
        }else{
          body.style.background='#22c55e';
          st.textContent='✔ IDLE / STATIC';
          dt.textContent='E = '+d.energy.toFixed(3)+
            ' thr = '+d.thresh.toFixed(3)+
            ' sc = '+d.sc+
            ' μΔ = '+d.micro;
        }
        inf.textContent='Frame '+d.frame+
          ' streaks '+d.e_str+'/'+d.sc_str+'/'+d.mi_str;
      }catch(_){}
      setTimeout(poll,50);
    }
    poll();
  </script>
</body></html>)html";

static void handle_root() {
    server.send_P(200, "text/html", HTML_PAGE);
}

static void handle_status() {
    char buf[300];
    snprintf(buf, sizeof(buf),
        "{\"motion\":%s,\"stage\":\"%s\",\"frame\":%d,"
        "\"energy\":%.4f,\"thresh\":%.4f,"
        "\"sc\":%d,\"micro\":%d,"
        "\"e_str\":%d,\"sc_str\":%d,\"mi_str\":%d}",
        g_motion ? "true" : "false",
        g_stage == STAGE_CALIBRATING ? "CALIBRATING" : "TRACKING",
        g_frame,
        (float)g_energy, (float)g_thresh,
        g_sc_anom, g_micro_sc,
        e_streak, sc_streak, mi_streak);
    server.send(200, "application/json", buf);
}

static void handle_not_found() {
    server.send(404, "text/plain", "404 Not Found");
}

// ============================================================================
// LED PANIC — rapid flash loop, never returns
// ============================================================================
static void led_panic() {
    Serial.println("[FATAL] Sanity tests FAILED — halting with panic blink.");
    pinMode(LED_PIN, OUTPUT);
    while (true) {
        digitalWrite(LED_PIN, HIGH); delay(80);
        digitalWrite(LED_PIN, LOW);  delay(80);
    }
}

// ============================================================================
// SETUP  (Core 0)
// ============================================================================
void setup() {
    Serial.begin(921600);
    delay(400);

    // LED must be ready before the countdown
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("\n=== Wi-Sense Standalone Firmware v1.3 ===");

    // ── 0. Pre-calibration countdown (8 s) ───────────────────────────────
    // Purpose: give the user time to leave the room and connect their phone
    // to "Wi-Sense-Net" BEFORE calibration begins.  If anyone is present
    // during the 500-frame calibration window, bl_fast will converge to a
    // "person-present" baseline — any subsequent motion will be invisible and
    // any stillness will read as motion.  The countdown prevents this.
    //
    // LED feedback: one slow blink per second for 8 seconds.
    //   • Blinks = "preparing, please leave room"
    //   • Solid ON (after countdown) = "calibrating, stay out"
    Serial.println("[Boot] === 8-SECOND PRE-CALIBRATION DELAY ===");
    Serial.println("[Boot] Leave the room now and connect your phone to Wi-Sense-Net");
    for (int i = 8; i > 0; i--) {
        Serial.printf("[Boot] Starting calibration in %d s...\n", i);
        digitalWrite(LED_PIN, HIGH); delay(350);   // short ON pulse
        digitalWrite(LED_PIN, LOW);  delay(650);   // long OFF
    }
    Serial.println("[Boot] Room must be empty — beginning initialization.");
    digitalWrite(LED_PIN, HIGH);  // solid ON during boot/calibration

    // ── 1. Boot sanity tests ──────────────────────────────────────────────
    if (!run_sanity_tests()) led_panic();
    reset_state();   // clean slate for live operation

    // ── 2. Wi-Fi Access Point ─────────────────────────────────────────────
    // WIFI_AP_STA: AP for phone + STA interface needed by the CSI subsystem
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WiFi] AP '%s'  IP %s\n",
                  AP_SSID, AP_IP.toString().c_str());

    // ── 3. Web server ─────────────────────────────────────────────────────
    server.on("/",           HTTP_GET, handle_root);
    server.on("/api/status", HTTP_GET, handle_status);
    server.onNotFound(handle_not_found);
    server.begin();
    Serial.println("[HTTP] Serving on port 80");

    // ── 4. CSI ────────────────────────────────────────────────────────────
    // Enable LLTF-only CSI: 64 subcarriers × 2 bytes = 128 bytes per packet.
    // The TX board must be associated with this AP and transmitting frames
    // so the RX chip has incoming Wi-Fi packets to measure.
    wifi_csi_config_t cfg = {};
    cfg.lltf_en           = true;    // Legacy LTF — full 64 SC at 20 MHz
    cfg.htltf_en          = false;
    cfg.stbc_htltf2_en    = false;
    cfg.ltf_merge_en      = true;
    // BUG FIX: enable channel filter (3-tap FIR smoothing of raw CSI).
    // The original capture firmware (plot_my_csi.py reference data) was
    // collected with hardware-smoothed CSI.  Raw unfiltered CSI (false)
    // has significantly higher frame-to-frame amplitude noise, which causes
    // the empty-room energy to exceed the near-zero Stage-2 threshold and
    // fire crit_e and crit_sc continuously even with no one present.
    cfg.channel_filter_en = true;
    cfg.manu_scale        = false;

    esp_wifi_set_csi_config(&cfg);
    esp_wifi_set_csi_rx_cb(csi_callback, nullptr);
    esp_wifi_set_csi(true);
    Serial.println("[CSI] Callback registered");

    // ── 5. Mutex + processing task on Core 1 ─────────────────────────────
    csi_sem = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(
        processing_task,   // function
        "wi_sense_proc",   // name
        8192,              // stack bytes (float arrays + log10 stack frames)
        nullptr,           // parameter
        5,                 // priority (> loop's 1, < Wi-Fi stack's 23)
        nullptr,           // handle (unused)
        1                  // Core 1  (Core 0 = Wi-Fi + loop + web server)
    );

    // ── 6. System live ───────────────────────────────────────────────────
    // LED already HIGH from countdown; loop() will switch to blink pattern.
    Serial.println("[Boot] System live.");
    Serial.println("       Open 192.168.4.1 on the phone connected to Wi-Sense-Net");
    Serial.println("       TX board must be on Wi-Sense-Net to generate CSI frames");
}

// ============================================================================
// LOOP  (Core 0 — web server + LED blink; all detection runs on Core 1)
// ============================================================================
void loop() {
    server.handleClient();

    // LED blink pattern: slow (500 ms) = IDLE, fast (100 ms) = MOTION
    static uint32_t last_blink = 0;
    static bool     led_state  = false;
    uint32_t interval = g_motion ? 100u : 500u;
    if ((uint32_t)(millis() - last_blink) >= interval) {
        last_blink = millis();
        led_state  = !led_state;
        digitalWrite(LED_PIN, led_state ? HIGH : LOW);
    }
}
