# Real-Time Task-Aware Adaptive Denoising for Simulated Event-Based Vision — Architecture Spec

Target: PYNQ-Z2 (XC7Z020-1CLG400C). PL clocks: 25.2 MHz (pixel, from HDMI-in) and 100 MHz (processing). PS: ARM Cortex-A9.

## Design decisions made to make the prompt implementable (flag these — override if they don't match your intent)

- **Color path**: HDMI-in IP outputs 24-bit RGB. An RGB→Gray stage (multiplier-free, shift-add) runs in the pixel-clock domain *before* the CDC FIFO, so only 8 bits/pixel cross the clock boundary (cheaper FIFO, and Stage 1 operates on luma like most event-camera models).
- **Stage 1 vs Stage 2 split**: Stage 1 (frame-diff) does pixel-wise diff + ternary encode using a small fixed epsilon (removes ADC-level noise only). Stage 2 (adaptive denoise) is a **3×3 spatial k-of-N filter on the ternary map** — an isolated event survives only if ≥N of its 8 neighbors are also active; N is the adaptive register. This matches "denoising filter with adaptive threshold" better than a second amplitude threshold, and is the standard salt-and-pepper removal used in real event-camera denoising (e.g., background-activity filters).
- **Task-feedback reference metric**: Sobel gradient magnitude (|Gx|+|Gy|, no multiplies — Sobel weights are ±1/±2, done with shifts+adds) accumulated over the raw gray frame, compared each frame against the filtered event count from Stage 2.
- **Frame buffer**: a single 640×480×8-bit frame (300 KB) lives in DDR3 (PS-side, accessed over an HP AXI port) — not double-buffered. Since processing is strictly raster-sequential, each row is read (previous frame's row), used by the Frame-Diff Engine, then immediately overwritten with the current row — a row-granularity read-then-write, not a per-pixel one (per-pixel would serialize every pixel behind a DDR3 round trip; at 640×480×60 = 18.4 Mpx/s that's not viable). The **Row Buffer module** (BRAM, 1 row × 640 B) owns this sequencing: it holds the AXI4-Full master interface, drives the burst read and burst write, and enforces read-before-write ordering to the same address (AXI read/write channels are independent, so this ordering is not automatic and must be explicitly sequenced — same-ID in-order completion or an explicit FSM gate). DDR3 itself is just the memory; it has no control logic of its own. Frame-Diff Engine is a simple client of the Row Buffer's handshake, not an AXI master itself.

---

## 1. Top-level block diagram

```
                 pixel clk domain (25.2 MHz)              |        processing clk domain (100 MHz)
                                                            |
 HDMI-in ----> [HDMI RX IP]--24b RGB-->[RGB2GRAY]--8b-->[ASYNC FIFO]--8b-->[FRAME-DIFF ENGINE]
 (TMDS)         (vendor IP,             tvalid/tuser=SOF          |            |   ^
                 recovers 25.2MHz        tlast=EOL                |            v   | prev-pixel (8b)
                 pixel clk + sync)                                |     [3x3 DENOISE FILTER]<--+
                                                                   |            |  (ternary, 2b)      |
                                                                   |            v                      |
                                                                   |     [RLE COMPRESSION ENGINE]       |
                                                                   |            |  (byte stream)         |
                                                                   |            v                         |
                                                                   |     [AXI-Stream / AXI DMA S2MM]-------+--> PS DDR3
                                                                   |            (to PS, for SD logging)     (compressed
                                                                   |                                         log region)
                                                        gray pixel |
                                    +------------------------------+
                                    v
                            [SOBEL EDGE METRIC UNIT] (shares gray pixel stream)
                                    |
                                    v
                       [TASK-FEEDBACK PROXY CONTROLLER] --threshold(N)--> feeds 3x3 DENOISE FILTER
                                    ^                                     and AXI4-Lite STATUS regs
                                    |
                       event_count from Denoise Filter (per-frame)

        [ROW BUFFER (BRAM, 1 row)] <--AXI4-Full master, burst R then W--> DDR3 (PS, single frame, 300KB)
              |         ^
              |         +-- FSM: IDLE -> BURST_READ -> PROCESS -> BURST_WRITE -> IDLE
              |             (enforces read-before-write to same row address; owns AXI master port)
              v
        feeds FRAME-DIFF ENGINE "prev pixel" input (simple handshake, not an AXI master itself)

        [AXI4-Lite SLAVE] <----> PS (ARM Cortex-A9): control/status register map, section 6
```

Data widths at each interface:

| Interface | Width | Notes |
|---|---|---|
| HDMI RX IP → RGB2Gray | 24b RGB (8/8/8) | + `tvalid`, `tuser`(SOF), `tlast`(EOL) |
| RGB2Gray → Async FIFO | 8b gray | pixel-clock domain |
| Async FIFO → Frame-Diff | 8b gray | now in 100 MHz domain |
| Row Buffer → Frame-Diff | 8b gray (`prev_pixel`) | simple handshake, not AXI |
| Frame-Diff → Denoise Filter | 2b ternary (`00`=0,`01`=+1,`10`=−1) | |
| Denoise Filter → RLE | 2b ternary | post-filter |
| RLE → AXI-Stream out | 8b packed | `{run_length[5:0], symbol[1:0]}` byte, see §2 |
| Gray stream → Sobel unit | 8b gray | tapped in parallel with Frame-Diff |
| Sobel unit → Task-Feedback Ctrl | 16b accumulated |Gx|+|Gy| sum | per-frame |
| Task-Feedback Ctrl → Denoise Filter | 4b threshold N (0–8) | updated once/frame |
| AXI4-Lite ↔ PS | 32b data / 8b addr (16 regs) | see §6 |

---

## 2. Module-by-module specification

### 2.1 HDMI-in Interface
Vendor IP (Digilent `hdmi_in`/`dvi2rgb` or Xilinx `v_hdmi_rx_ss`), not hand-written — flag for ASIC (see §8).
- **Outputs**: `rgb_data[23:0]`, `rgb_valid`, `rgb_sof`, `rgb_eol`, recovered `pclk` (~25.2 MHz)
- Approx logic: TMDS decoders ×3, channel-bonding FSM, MMCM/PLL for clock recovery.

### 2.2 RGB2Gray (pixel-clock domain)
- **In**: `rgb_data[23:0]` (R[23:16], G[15:8], B[7:0]), `rgb_valid`
- **Out**: `gray[7:0]`, `gray_valid`, `sof`, `eol` (pass-through, 1-cycle delayed)
- **Logic**: `gray = (2*R + 5*G + 1*B) >> 3` implemented as `(R<<1) + (G<<2) + G + B` then `>>3` — pure shift/add, no multiplier.
- **State**: none (combinational + 1 pipeline register).
- Complexity: 3 adders (9-bit), 1 shifter. ~50 LUT / 30 FF.

### 2.3 CDC Async FIFO
- **In (write side, pclk)**: `gray[7:0]`, `gray_valid`, `sof`, `eol`
- **Out (read side, 100 MHz)**: `dout[7:0]`, `empty`, `rd_en`, `sof_out`, `eol_out`
- Depth 1024×8 (Gray-code pointer dual-clock FIFO), plus a 2-bit-wide parallel FIFO (or packed into same word) for `sof`/`eol` tags.
- Complexity: standard 2-FF synchronizers on Gray-coded pointers + 1 BRAM18. ~150 LUT / 100 FF / 1 BRAM18.

### 2.4 Row Buffer (FSM owner, AXI master) + DDR3 (single-buffer, row-granularity R-then-W)
Row Buffer owns the sequencing FSM and the AXI4-Full master port; DDR3 is pure memory (single 640×480×8b frame, no ping-pong).
- **FSM**: `IDLE → BURST_READ → PROCESS → BURST_WRITE → IDLE`
  - `IDLE`: wait for `row_start` (asserted at start of each incoming line)
  - `BURST_READ`: issue AXI4 burst read of the previous frame's row at the current row address into the row-buffer BRAM (640 B)
  - `PROCESS`: gate the pixel stream — Frame-Diff Engine consumes `prev_pixel` from the row buffer, one pixel per cycle, in lockstep with the incoming current-row pixel stream
  - `BURST_WRITE`: once the row is fully consumed, burst-write the current row's pixels (captured during PROCESS) back to the same DDR3 row address, overwriting the previous frame's row
  - Ordering guarantee: `BURST_WRITE` does not issue until `BURST_READ`'s data has been fully consumed by `PROCESS` — same-row read and write are never in flight simultaneously, avoiding an AXI read/write-channel race (the two channels are independent and give no ordering guarantee on their own).
- **In**: `cur_pixel[7:0]`, `cur_valid`, `sof`, `eol`, AXI4-Full `m_axi_*` (master)
- **Out**: `prev_pixel[7:0]`, `prev_valid` (to Frame-Diff Engine), `row_burst_status[1:0]` (idle/reading/processing/writing, exposed via AXI4-Lite for debug), `row_burst_error` (sticky, overrun detection)
- **State**: row buffer BRAM (1 row × 640 B), row address counter (20-bit), FSM state register, write-back staging register.
- Complexity: AXI4-Full master burst logic + 1 row BRAM (1× BRAM18) + FSM. ~900 LUT / 700 FF / 1 BRAM18.

### 2.5 Frame-Diff Engine
- **In**: `cur_pixel[7:0]`, `prev_pixel[7:0]`, `valid`, `sof`, `eol`
- **Out**: `ternary[1:0]`, `valid`, `sof`, `eol`
- **Logic**: `diff = {1'b0,cur} - {1'b0,prev}` (signed 9-bit) → compare against fixed epsilon register `EPS[7:0]` (default small, e.g. 8): `diff > EPS → 01`, `diff < -EPS → 10`, else `00`.
- **State**: `EPS` register (AXI4-Lite, RW).
- Complexity: 1 subtractor (9-bit), 2 comparators. ~120 LUT / 40 FF.

### 2.6 3×3 Adaptive Denoise Filter
- **In**: `ternary_in[1:0]`, `valid`, `sof`, `eol`, `thresh_N[3:0]` (from Task-Feedback Ctrl)
- **Out**: `ternary_out[1:0]`, `valid`, `event_count_frame[19:0]` (pulsed at `eol` of last line / latched at frame end)
- **Tap point**: `event_count_frame` is incremented from the same `ternary_out` stream that feeds RLE Compression (§2.9) — i.e. it counts post-filter, pre-RLE events. It is *not* derived from RLE's output byte count, run count, or `COMPRESSED_BYTES`; those are a separate, later statistic (compression ratio) with no bearing on the feedback loop's ratio calculation in §2.8/§4.
- **Internal**: 2 line buffers, 640×2b each (double-buffered as data streams in), forming a 3×3 sliding window (3 taps × 3 rows). Center pixel passes through only if `(count of active neighbors, i.e. nonzero ternary in the 8-neighborhood) >= thresh_N`; else forced to `00`.
- **State**: line buffer write pointers (10-bit ×2), window shift registers (3×3×2b), running per-frame active-event counter (20-bit, cleared at `sof`).
- Complexity: 2 line buffers (1 BRAM18), 8-input popcount-style adder tree (comparators + adder), window shift regs. ~300 LUT / 150 FF / 1 BRAM18.

### 2.7 Sobel Edge-Metric Unit
- **In**: `gray[7:0]`, `valid`, `sof`, `eol`
- **Out**: `edge_sum_frame[19:0]` (latched at frame end)
- **Internal**: its own 2 line buffers (640×8b each) for the 3×3 gray window, Gx/Gy computed via shift-add (Sobel weights ±1/±2), `|Gx|+|Gy|` accumulated every pixel into a running sum, cleared at `sof`.
- Complexity: 2 line buffers (1 BRAM18), ~8 adders/subtractors for Gx/Gy, 1 accumulator (20-bit). ~400 LUT / 300 FF / 1 BRAM18.

### 2.8 Task-Feedback Proxy Controller
See FSM in §4.
- **In**: `event_count_frame[19:0]`, `edge_sum_frame[19:0]`, `frame_tick` (pulse at `sof`)
- **Out**: `thresh_N[3:0]` (to Denoise Filter), status fields to AXI4-Lite
- **Internal registers** (AXI4-Lite RW, with sane defaults): `RATIO_HI`, `RATIO_LO` (fixed-point, e.g. Q4.4), `STEP` (default 1), `N_MIN` (default 1), `N_MAX` (default 8). These are **write targets from the PS**, not just internal state — the AXI4-Lite path into this module is bidirectional: config (`STEP`, `N_MIN`, `N_MAX`, `K_HI`, `K_LO`, manual `THRESH_N` override) is written down by the PS at init or runtime, while `EVENT_COUNT`, `EDGE_METRIC_SUM`, and the current `THRESH_N` are read back up as status. The block diagrams show only the upward status arrow for clarity; the downward config-write path exists on the same physical AXI4-Lite bus and is listed per-register as RW vs RO in §6.
- **Logic**: instead of a true divide, compute `ratio_check` via cross-multiplication comparators (avoids a divider): compare `event_count_frame * RATIO_LO_denom` against `edge_sum_frame * RATIO_LO_num`, etc. — or, simpler and fully multiplier-free, compare `event_count_frame` against `edge_sum_frame >> k` for two fixed shift amounts (k_hi, k_lo) that bound the acceptable band. This keeps the whole controller shift/compare-only.
- Complexity: comparators, small FSM, 4-bit up/down counter with clamp. ~150 LUT / 100 FF.

### 2.9 RLE Compression Engine
- **In**: `ternary_in[1:0]`, `valid`
- **Out (AXI4-Stream)**: `tdata[7:0] = {run_len[5:0], symbol[1:0]}`, `tvalid`, `tready`, `tlast` (at frame end)
- **Logic**: run-length counter (6-bit, saturates at 63 and force-emits); on symbol change or saturation, emit one byte and reset counter.
- **State**: `run_len[5:0]`, `cur_symbol[1:0]`.
- Complexity: 6-bit counter, comparator, small output skid buffer (1 BRAM18 for burst smoothing before DMA). ~200 LUT / 150 FF / 1 BRAM18.

### 2.10 AXI-Stream → PS (logging) and AXI4-Lite (control/status)
- **AXI DMA #3 (S2MM)**: streams RLE bytes into a PS DDR3 log buffer, which the PS/PYNQ side flushes to SD card.
- **AXI4-Lite slave**: register map in §6.
- Complexity: 1× AXI DMA IP + AXI4-Lite slave logic. ~1150 LUT / 1400 FF.

---

## 3. Pipeline timing / latency

Dominant latency source is **line-buffer fill for the 3×3 windows**, not the arithmetic (which is 1 cycle deep everywhere).

| Stage | Added latency | Domain |
|---|---|---|
| RGB2Gray | 1 cycle | 25.2 MHz (39.7 ns) |
| CDC async FIFO | ~4–6 cycles (write→read sync + FWFT) | crosses to 100 MHz (10 ns) |
| Row Buffer BURST_READ (per row, steady state) | one AXI4 burst per row (~640B), must complete before PROCESS starts for that row — sized so burst time < 1 row's worth of pixel-clock time (640 px @ 25.2MHz ≈ 25.4 µs per row budget) | 100 MHz |
| Frame-Diff | 1 cycle | 100 MHz |
| 3×3 Denoise Filter | ~2 lines + 3 px ≈ 2×640+3 = 1283 cycles (window fill) | 100 MHz |
| Sobel unit (parallel path) | ~1283 cycles (same window-fill cost) | 100 MHz |
| RLE | 1–63 cycles (variable, amortized ~1) | 100 MHz |
| AXI DMA out (burst) | tens of cycles per burst (not per-pixel) | 100 MHz |

**First-pixel-to-first-output latency ≈ 1283 cycles @ 100 MHz ≈ 12.8 µs** (line-buffer fill dominates). Steady-state throughput after that is 1 pixel/cycle @ 100 MHz, comfortably ahead of the 25.2 MHz pixel rate needed for 640×480@60 (≈15.4 M px/s vs 100 M cycles/s available) — plenty of margin for DMA burst stalls.

Threshold (`thresh_N`) update happens once per frame at `sof`, so its 1-frame-of-delay feedback loop is fine — it's a slow control loop by design (bang-bang/hysteresis on frame-level statistics), not a per-pixel one.

---

## 4. Task-Feedback Proxy Controller FSM

States: `IDLE → ACCUM → EVAL → UPDATE → ACCUM ...`

```
IDLE  --(frame_tick)-->  ACCUM
ACCUM --(accumulating event_count_frame & edge_sum_frame every pixel; on next frame_tick)--> EVAL
EVAL:
   if event_count_frame > (edge_sum_frame >> k_hi):   -> UPDATE (action: N = min(N+STEP, N_MAX))   // too noisy, tighten
   elif event_count_frame < (edge_sum_frame >> k_lo):  -> UPDATE (action: N = max(N-STEP, N_MIN))   // too sparse, loosen
   else:                                                -> UPDATE (action: N unchanged)              // in-band, hold
UPDATE --(1 cycle, commit thresh_N, clear accumulators)--> ACCUM
```

- `k_hi > k_lo` creates the hysteresis band (dead zone) so the loop doesn't oscillate every frame.
- All comparisons are shift + compare, no divider/multiplier — keeps the controller "purely rule-based, deterministic" as required.
- `thresh_N` clamped every UPDATE to `[N_MIN, N_MAX]` (defaults 1 and 8, since a 3×3 neighborhood has 8 neighbors).

---

## 5. Memory architecture

| Item | Location | Size | Notes |
|---|---|---|---|
| CDC FIFO | BRAM (fabric) | 1024×8b ≈ 1 KB | 1× BRAM18 |
| **Row Buffer** (staging: previous-frame row in/out of DDR3) | BRAM (fabric) | 1 row × 640×8b = **640 B** | 1× BRAM18; functionally distinct from the line buffers below — this is DDR3 staging, not spatial window context |
| **Denoise Filter line buffers** (3×3 spatial window context) | BRAM (fabric) | 2×640×2b = 320 B | 1× BRAM18 (2 lines of ternary data) |
| **Sobel line buffers** (3×3 spatial window context) | BRAM (fabric) | 2×640×8b = 1.25 KB | 1× BRAM18 (2 lines of gray data) |
| RLE output skid buffer | BRAM (fabric) | ~2 KB | 1× BRAM18, absorbs DMA burst jitter |
| Previous/current frame (**single buffer**, row-granularity R-then-W) | **DDR3 (PS)** | 640×480×1B = **300 KB** | no ping-pong; each row is read then overwritten in place, sequenced by the Row Buffer FSM (§2.4) |
| Compressed log staging buffer | DDR3 (PS) | app-defined (e.g. a few MB ring buffer) | drained to SD card by PS software |

Total BRAM in fabric: ~4×BRAM18 ≈ 2×BRAM36 — trivial against the 7020's 140×BRAM36 budget. DDR3 usage (≈300 KB + log ring buffer) is negligible against the board's 512 MB, and half what the earlier double-buffered design needed.

---

## 6. AXI4-Lite register map (control/status, PS-visible)

The interface is bidirectional on a single AXI4-Lite bus: control fields (marked RW below) are written down by the PS into the Feedback Controller and Row Buffer — e.g. `EPS`, `N_MIN`/`N_MAX`/`STEP`/`K_HI`/`K_LO`, and the manual `THRESH_N` override — while status fields (RO) are read back up by the PS every frame for the stats panel and debug visibility. The pipeline diagrams show only the status (upward) arrow to avoid clutter; the config-write (downward) path exists on the same bus and is what the RW column below documents per-register.

| Offset | Name | R/W | Width | Description |
|---|---|---|---|---|
| 0x00 | CTRL | RW | 32b | bit0: pipeline enable; bit1: manual threshold override enable |
| 0x04 | STATUS | RO | 32b | bit0: pipeline running; bit1: DMA error flags |
| 0x08 | EPS | RW | 8b | Stage-1 frame-diff epsilon |
| 0x0C | THRESH_N | RW* | 4b | current adaptive denoise threshold (writable only if manual-override bit set, else RO reflecting HW value) |
| 0x10 | N_MIN | RW | 4b | clamp floor |
| 0x14 | N_MAX | RW | 4b | clamp ceiling |
| 0x18 | STEP | RW | 4b | hysteresis update step size |
| 0x1C | K_HI | RW | 4b | shift amount, upper hysteresis band |
| 0x20 | K_LO | RW | 4b | shift amount, lower hysteresis band |
| 0x24 | EVENT_COUNT | RO | 20b | last frame's post-filter event count |
| 0x28 | EDGE_METRIC_SUM | RO | 20b | last frame's Sobel magnitude sum |
| 0x2C | COMPRESSED_BYTES | RO | 20b | bytes emitted by RLE last frame (→ compression ratio = 307200 / this) |
| 0x30 | FRAME_COUNT | RO | 32b | free-running frame counter |
| 0x34 | ROW_BURST_STATUS | RO | 2b | Row Buffer FSM state: idle/reading/processing/writing — debug visibility into the row-granularity R-then-W sequencing (§2.4) |
| 0x38 | ROW_BURST_ERROR | RO, W1C | 1b | sticky overrun flag — set if BURST_WRITE for a row was not able to complete before the next row's BURST_READ needed to start |
| 0x3C | LOG_BUF_WPTR | RO | 32b | PS uses this to know how much of the log ring buffer to flush to SD |
| 0x40 | LOG_BUF_BASE | RW | 32b | base address of the DDR3 log staging region |
| 0x44 | LOG_BUF_SIZE | RW | 32b | size of the log ring buffer |

The 3-panel PS-side display (raw / event-map / stats) reads `EVENT_COUNT`, `EDGE_METRIC_SUM`, `COMPRESSED_BYTES`, `THRESH_N` each frame for the stats panel, and pulls the raw and filtered frames from their respective DDR3/AXI-Stream sources for the other two panels.

---

## 7. Resource budget estimate (Zynq-7020: 53,200 LUT / 106,400 FF / 140 BRAM36 / 220 DSP)

| Module | LUT | FF | BRAM36 | DSP |
|---|---:|---:|---:|---:|
| HDMI RX IP | 2500 | 3000 | 4 | 0 |
| RGB2Gray | 50 | 30 | 0 | 0 |
| CDC Async FIFO | 150 | 100 | 0.5 | 0 |
| Row Buffer (FSM + AXI4-Full master + 1-row BRAM) | 900 | 700 | 1 | 0 |
| Frame-Diff Engine | 120 | 40 | 0 | 0 |
| 3×3 Denoise Filter | 300 | 150 | 0.5 | 0 |
| Sobel Edge-Metric Unit | 400 | 300 | 0.5 | 0 |
| Task-Feedback Proxy Controller | 150 | 100 | 0 | 0 |
| RLE Compression Engine | 200 | 150 | 0.5 | 0 |
| AXI DMA (log) + AXI4-Lite slave | 1150 | 1400 | 0 | 0 |
| **Total** | **~5920** | **~5970** | **~6.5** | **0** |
| **% of Zynq-7020** | **11.1%** | **5.6%** | **4.6%** | **0%** |

Design deliberately has **zero DSP usage** (all arithmetic is shift/add/compare) and comfortable margin on every resource — matches the "resource-constrained, hand-written RTL" narrative and leaves headroom for the HDMI RX IP's actual vendor-reported numbers (which vary; the 2500/3000/4 estimate above is a placeholder — get real numbers from the specific IP's datasheet before finalizing). Moving from a ping-pong AXI-DMA frame buffer to a single Row Buffer FSM also dropped ~600 LUT / ~1100 FF versus the earlier double-buffered design, on top of halving the DDR3 footprint.

---

## 8. Flags for the eventual ASIC flow

**Inclusion criterion**: a module is in ASIC scope if it is hand-written and technology-independent (no FPGA vendor primitive, no vendor IP core), regardless of whether it is central to the paper's task-aware adaptive-denoising contribution specifically. Sobel Edge-Metric and the Row Buffer's sequencing FSM both pass this test — they are pure shift/add/compare RTL with no vendor dependency — even though neither is unique to the adaptive-denoising idea itself. Conversely, generic-sounding logic (RLE compression) stays in scope because it too is hand-written and portable, not because it's algorithmically novel. What's excluded is specifically vendor primitives and protocol/interconnect glue (HDMI PHY, AXI DMA IP, the AXI4-Full/AXI4-Lite bus protocol logic itself), not anything based on "how central is this to the idea."

Modules/primitives that are FPGA-specific and need a technology-independent rewrite (or a scope decision) before synthesis on e.g. SkyWater 130 nm / OpenLane:

| Item | Issue | ASIC path |
|---|---|---|
| HDMI RX IP + MMCM/PLL | Fully Xilinx SERDES/clocking primitives | **Out of RTL scope.** Assume the ASIC block starts from an abstracted parallel pixel bus (`data[7:0]`, `valid`, `sof`, `eol`) — treat as a testbench-driven input, not synthesized hardware. State this explicitly in the ASIC PPA report so reviewers don't expect HDMI PHY silicon.
| CDC Async FIFO | If coded with `xpm_fifo_async` or a Xilinx FIFO primitive instantiation | Rewrite as a generic Gray-code dual-clock FIFO (pure RTL, no vendor macro) — straightforward, ~1 day of work, and it's the correct thing to hand-write anyway per your no-HLS/no-vendor-primitive constraint. **Do this even for the FPGA build**, so the same RTL carries through unchanged.
| Line buffers (Denoise Filter, Sobel unit) | If coded as `(* ram_style = "block" *)` arrays targeting Xilinx BRAM inference | Small enough (≤1.25 KB) to instead map to a flip-flop array / standard-cell shift-register style RTL for ASIC, avoiding an SRAM-macro dependency; keep a `GENERATE`/parameter to pick BRAM-inference style for FPGA vs FF-array style for ASIC from the same RTL source.
| Row Buffer's AXI4-Full master port to DDR3 | The FSM sequencing (§2.4) is hand-written and portable, but the AXI4-Full master protocol logic itself and the DDR3 memory it addresses are PS/system-integration concerns | **Partially in scope.** The `IDLE → BURST_READ → PROCESS → BURST_WRITE` FSM and its ordering guarantee are legitimate hand-written RTL and can be reported. The AXI4-Full master transaction logic and DDR3 itself should be abstracted for ASIC as a generic off-chip SRAM controller with a `read_row`/`write_row` handshake — don't report AXI protocol gates in the PPA number, but do keep and report the sequencing FSM, since it's the actual algorithmic contribution (enforcing the ordering guarantee) rather than bus glue.
| AXI4-Lite slave | Vendor-adjacent interconnect glue | Keep for FPGA control; for the ASIC PPA report, either exclude it (state clearly it's system-integration glue, not core algorithm logic) or replace with a trivial generic register-file interface if you want end-to-end synthesizable determinism.
| RLE output AXI-Stream to PS | Same DMA/interconnect concern as above | Exclude from ASIC core; treat RLE engine's `tdata/tvalid/tready` as a generic byte-output handshake (already technology-independent as specified in §2.9) — this part is fine as-is.

**Bottom line for the ASIC flow**: the genuinely portable, hand-written core is the Row Buffer's sequencing FSM (§2.4, protocol-agnostic version) + Frame-Diff Engine (§2.5) + 3×3 Denoise Filter (§2.6) + Sobel Edge-Metric Unit (§2.7) + Task-Feedback Proxy Controller (§2.8) + RLE Compression Engine (§2.9), plus a generic (non-vendor) CDC FIFO if you keep the two-clock-domain structure in the ASIC version too. Everything else (HDMI PHY, AXI DMA, AXI4-Lite) is FPGA/system-integration scaffolding that should be scoped out of the area/power/timing comparison, or replaced with generic stand-ins and clearly labeled as such.
