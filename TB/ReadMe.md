# AXI Bridge IP TX – Verification Environment

This directory contains a SystemVerilog testbench for the `axi_bridge_ip_tx` block, which converts AXI-Stream beats from an IP side into fixed-width segments toward a Client-IF (e.g. UCIe protocol core).

The goal of this verification environment is to exercise the bridge under realistic traffic patterns and corner cases, and to validate both functional behavior and telemetry counters (status / error events).

---

## DUT Overview

**Module:** `axi_bridge_ip_tx`

**High-level function:**

- Ingress side: AXI-Stream (`s_axis_tdata/tkeep/tuser/tvalid/tlast/tready`) from an IP.
- Internal: Ingress FIFO of full AXI beats, then serialization from `DATA_W` to `IF_W` segments.
- Egress side: Client-IF segments (`cl_tx_data/keep/user/valid/sop/eop/ready`).

Key features:

- Segmentation of `DATA_W`-wide AXI beats into `IF_W`-wide segments.
- Support for `TKEEP` patterns (full and partial) using popcount logic.
- Optional `strict_tkeep_en` checking for illegal `TKEEP` patterns.
- Backpressure handling on both IP side and Client side.
- CSR-visible status:
  - `stat_tx_frames`, `stat_tx_bytes`
  - `stat_tx_fifo_level`, `stat_tx_stall_cycles`
- Error / event flags:
  - `ev_err_tkeep_illegal`
  - `ev_err_midreset_drop`
  - `ev_err_overflow_tx`

---

## Verification Environment

The testbench is a lightweight, class-based SV environment (not full UVM) and consists of:

- **Package (`package.sv`)**
  - Common parameters: `DATA_W`, `IF_W`, `TUSER_W`
  - Derived constants: `BYTES_PER_BEAT`, `BYTES_PER_SEG`, `MAX_SEGS_PER_BEAT`
  - `axi_frame` / `axi_beat_t` types
  - **Scoreboard** with a reference model that:
    - Re-computes expected segmentation (`MAX_SEGS_PER_BEAT`, `BYTES_PER_SEG`)
    - Checks SOP/EOP, `cl_tx_keep`, `cl_tx_user`, and data continuity
    - Detects missing/extra segments

- **Interfaces**
  - AXI-Stream interface for the IP side
  - Client-IF interface for the protocol core side

- **Components**
  - **Driver**: drives AXI-Stream beats from a high-level `axi_frame`, obeying TVALID/TREADY handshakes.
  - **Monitor**: samples Client-IF signals (`data/keep/user/valid/sop/eop/ready`) and builds observed segment streams.
  - **Scoreboard**: compares “expected” segments (from the reference frame model) to the actual segments seen by the monitor.

- **Top testbench (`test_bench.sv`)**
  - Instantiates DUT, interfaces, driver, monitor, and scoreboard.
  - Implements a set of directed tests, each controlled by `ENABLE_TESTx` flags.

---

## Test Plan & Implemented Scenarios

Below is a summary of the main directed tests implemented in this environment.

### 1. Multi-beat frames (full `TKEEP`)

**Goal:** Validate segmentation over multiple full beats.

**Scenario:**

- Generate a frame with `N` beats.
- All beats are **full** (`tkeep = all ones`).
- `TLAST` asserted only on the last beat.

**Checks:**

- The number of output segments equals:  
  `N * MAX_SEGS_PER_BEAT`.
- Data is contiguous (no “holes” or missing segments).
- `cl_tx_sop` asserted **only** on the first segment of the frame.
- `cl_tx_eop` asserted **only** on the last segment of the frame.
- Scoreboard verifies the exact order and content of all segments.

---

### 2. Partial `TKEEP` on the last beat

**Goal:** Verify correct handling of partially valid bytes, especially on the last segment.

**Scenarios:**

- Last beat has `TKEEP` with `num_bytes = 1, 2, …, BYTES_PER_SEG-1`.
- A frame smaller than a single segment:  
  a single beat with only a few valid bytes (frame length < `BYTES_PER_SEG`).

**Checks:**

- For the **last segment** in the frame:
  - `cl_tx_keep` reflects exactly the number of valid bytes (no extra ones).
- No extra “dummy” segment is emitted at the end of the frame.
- For very short frames (less than a segment):
  - Exactly **one** segment is emitted,
  - `cl_tx_keep` matches the exact byte count.

---

### 3. `strict_tkeep_en = 1` – legal and illegal patterns

**Goal:** Validate TKEEP legality checking and the `ev_err_tkeep_illegal` event.

**Scenarios:**

- Non-last beats with `keep != all ones` → **illegal**.
- Last beat with **non-monotonic** keep pattern (e.g. `1110_0111`) → **illegal**.
- Fully legal frame:
  - Non-last beats: full keep (`all ones`).
  - Last beat: monotonic pattern `111…1100…00`.

**Checks:**

- When `strict_tkeep_en = 1`:
  - Any non-last beat with partial `keep` triggers a **single-cycle pulse** on `ev_err_tkeep_illegal`.
  - Any last beat with a non-monotonic `keep` also triggers the event.
- For fully legal frames:
  - No `ev_err_tkeep_illegal` events.
- In all cases:
  - Data flow continues normally.
  - The event is **telemetry only**, not a hard stop or drop.

---

### 4. Backpressure on the Client side (`cl_tx_ready`)

**Goal:** Check that the DUT correctly holds output data/strobes stable while the consumer applies backpressure, and that stall counters are accurate.

**Scenario:**

- `cl_tx_ready` is driven with:
  - Random pattern (`0`/`1`),
  - Plus deterministic “long stall” windows in the middle of a frame.
- A multi-beat frame is driven from the IP side.

**Checks:**

- During cycles where `cl_tx_valid && !cl_tx_ready`:
  - `cl_tx_data`, `cl_tx_keep`, `cl_tx_user`, `cl_tx_sop`, `cl_tx_eop` remain **stable** across stall cycles (no glitches).
- No duplicated segments and no skipped segments:
  - Scoreboard sees the exact expected segment sequence.
- `stat_tx_stall_cycles` matches the number of cycles where `valid && !ready` (tracked independently in the TB).

---

### 5. Backpressure on the IP side – FIFO almost-full / full

**Goal:** Verify that ingress backpressure toward the IP is generated correctly when the FIFO fills, and that overflow is detected.

**Scenario:**

- The IP side drives traffic aggressively (`tvalid` quasi-constant).
- Client side applies strong backpressure (`cl_tx_ready` held low for long periods).
- FIFO level rises toward and beyond the configured threshold `tx_fifo_afull_thr`.

**Checks:**

- When `fifo_level >= tx_fifo_afull_thr`:
  - `s_axis_tready` de-asserts (IP sees backpressure).
- There are **no pushes when FIFO is full**:
  - Internally, the DUT would violate `!(fifo_push && fifo_full)` only in fault situations.
- Optional “fault injection” test:
  - Force a condition where `fifo_push && fifo_full` occurs.
  - Confirm that the DUT asserts `ev_err_overflow_tx`.

---

### 6. `bridge_enable` & `drop_on_midreset`

**Goal:** Validate clean enable/disable behavior and mid-reset drop semantics.

#### `bridge_enable`

Scenarios:

- Start with `bridge_enable = 0`:
  - No activity on Client-IF (`cl_tx_*` remain idle).
  - IP-side `s_axis_tready = 0` (IP cannot handshake).
- Raise `bridge_enable` to `1`:
  - Frames start flowing normally.
- Lower `bridge_enable` to `0` while the DUT is idle:
  - No drops, no events, no side-effects.

Checks:

- No frames counted or bytes counted while `bridge_enable = 0`.
- No stray segments on the Client-IF while disabled.

#### `drop_on_midreset`

Scenarios with `drop_on_midreset = 1`:

1. **Mid-frame drop:**
   - `bridge_enable` goes from `1` → `0` **while** a frame is active (`in_packet = 1`).
2. **FIFO data, no frame started:**
   - There are beats stored in the ingress FIFO, but no frame has yet started on the Client-IF (e.g. `cl_tx_ready = 0`),
   - Then `bridge_enable` is dropped from `1` to `0`.

Checks:

- In both cases:
  - `ev_err_midreset_drop` pulses once.
  - Any data “in flight” inside the bridge (FIFO / current beat / partial frame) is **flushed and dropped**.
  - After re-enabling (`bridge_enable = 1` again), new frames can be transmitted cleanly with **no mixing** with dropped data.
- The scoreboard confirms that:
  - No tail fragments from dropped frames appear after re-enable.
  - New frames after midreset are fully correct.

---

### 7. Parameterization (different `DATA_W` / `IF_W` configs)

**Goal:** Ensure the design scales correctly with different data widths and segment widths, and that the derived constants behave correctly.

**Configurations exercised:**

- `DATA_W = 256, IF_W = 64` – main configuration.
- `DATA_W = 128, IF_W = 64` – each beat serialized into 2 segments.
- `DATA_W = 64,  IF_W = 64` – edge case with **no serialization** (1 beat = 1 segment).

For each configuration, a subset of the tests above is re-run (basic frames, multi-beat, partial `TKEEP`, backpressure, midreset), and we check that:

- `MAX_SEGS_PER_BEAT` and related computations are correct.
- Segmentation logic (`cur_num_segs`, `cur_last_seg_bytes`, masks) remains valid.
- There are no hardcoded assumptions on specific widths (e.g. always 256→64).

---

## Running the Tests

The testbench is structured so each scenario can be enabled individually:

```systemverilog
localparam bit ENABLE_TEST1 = 1;
localparam bit ENABLE_TEST2 = 0;
localparam bit ENABLE_TEST3 = 0;
localparam bit ENABLE_TEST4 = 0;
localparam bit ENABLE_TEST5 = 0;
localparam bit ENABLE_TEST6 = 0;
localparam bit ENABLE_TEST7 = 0;
