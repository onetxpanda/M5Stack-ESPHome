# MLX90640 Driver Validation Plan

## Reference

**Datasheet:** [MLX90640 32×24 IR Array — Melexis (PDF)](https://www.melexis.com/-/media/files/documents/datasheets/mlx90640-datasheet-melexis.pdf)
**Melexis API source:** https://github.com/melexis/mlx90640-library

---

## Register Map Quick Reference

| Register | Address | Description |
|---|---|---|
| Status register | `0x8000` | Bit 3 = new data ready; bit 0 = current subpage |
| Control register 1 | `0x800D` | Mode, resolution, refresh rate |
| Pixel RAM | `0x0400–0x067F` | 768 × 16-bit pixel words |
| Auxiliary RAM | `0x0700–0x073F` | 64 × 16-bit aux words (Ta, gain, CP…) |
| EEPROM start | `0x2400–0x273F` | Calibration + default control register |
| EEPROM ctrl default | `0x240C` | Power-on default for `0x800D` |

### Control Register 1 (`0x800D`) bit layout

| Bits | Field | Values |
|---|---|---|
| 0 | I²C FM+ enable | 1 = push-pull (400 kHz / 1 MHz) |
| 3 | Data hold | 1 = hold until read |
| 4 | Subpage select | 0 = subpage 0; 1 = subpage 1 |
| 5 | Subpage repeat enable | 1 = toggle each cycle |
| 9:7 | Refresh rate | see table below |
| 11:10 | ADC resolution | 00=16-bit, 01=17-bit, **10=18-bit** (default), 11=19-bit |
| 12 | Reading pattern | 0 = interleaved; **1 = chess** (default, best SNR) |
| 15 | Measurement trigger | Write 1 to trigger (step mode) |

### Refresh rate encoding (bits 9:7)

| Register value | Rate | Subpage interval |
|---|---|---|
| 0 (`0b000`) | 0.5 Hz | 2000 ms |
| 1 (`0b001`) | 1 Hz | 1000 ms |
| 2 (`0b010`) | 2 Hz | 500 ms |
| 3 (`0b011`) | 4 Hz | 250 ms |
| 4 (`0b100`) | 8 Hz | 125 ms |
| **5 (`0b101`)** | **16 Hz** | **62.5 ms** ← default in this driver |
| 6 (`0b110`) | 32 Hz | 31.25 ms |
| 7 (`0b111`) | 64 Hz | 15.6 ms (image only; Ta update at half rate) |

### `REG_MASK` macro — critical naming note

```c
#define REG_MASK(sbit,nbits)  ~((~(~0UL << (nbits))) << (sbit))
```

`REG_MASK` returns a **cleared-field mask** (all bits set *except* the field).
This is the inverse of what the name implies:

- `MASK` → clear the slot: `reg & MASK` zeroes those bits
- `~MASK` → extract the field: `reg & ~MASK` keeps only those bits

Every use of `~REG_MASK` and bare `REG_MASK` in the driver is intentional
and correct.  Any future refactor must preserve this convention.

---

## Known Bugs Found During Development

These were identified during debugging and have been fixed; they are listed
here so regression tests can specifically cover them.

| # | Bug | Commit that fixed it | Regression test |
|---|---|---|---|
| 1 | `SetRefreshRate` read-modify-write inherited a corrupt register, leaving chess mode and ADC resolution wrong → all-white image | `6da8f04` | Section 4 |
| 2 | Misidentified `REG_MASK` as a normal field mask; "fix" inverted both sides, broke rate write and trashed all other control bits | `34ae3b4` (revert) | Section 4 |

---

## Validation Sections

---

### 1. I²C Bus Health

**Goal:** Confirm the sensor is reachable and the bus is stable before any
other test is attempted.

**Steps:**

1. Power on the M5Stack with the MLX90640 connected.
2. In the ESPHome log, confirm `Setting up MLX90640...` appears with no
   I²C NACK errors and `DumpEE` succeeds.
3. Confirm the address shown in the log matches the hardware (default `0x33`).
4. Run an I²C scan (`i2c_scan: true` in YAML) and verify only the expected
   address responds.
5. Confirm the bus frequency is set to 400 kHz in the ESPHome YAML:

   ```yaml
   i2c:
     frequency: 400kHz
   ```

**Expected outcomes:**
- No `I2C read/write failed` errors during setup.
- Bus scan shows exactly one device at the configured address.
- `MLX90640_DumpEE` returns 0 (no error).

---

### 2. EEPROM Parameter Extraction

**Goal:** Verify calibration data is read correctly and the extracted
parameters are plausible.

**Steps:**

1. Add temporary log statements after `MLX90640_ExtractParameters` to print:
   - `params.resolutionEE` — expected 2 (18-bit; device ships this way)
   - `params.calibrationModeEE` — expected 0 (chess; see note below)
   - `params.vdd25`, `params.kVdd` — should be non-zero integers
   - `params.alphaPTAT` — should be a small positive float
2. Call `MLX90640_GetVdd` on a known-good frame and verify the result is
   within ±0.2 V of 3.3 V.
3. Call `MLX90640_GetTa` on the same frame and verify the result is within
   ±5 °C of the measured room temperature.

**Datasheet note on `calibrationModeEE`:**
The EEPROM calibration mode bit is at `eeData[10]` bit 11 (EEPROM address
`0x240A`).  The driver XORs it with `0x80` so that the comparison in
`CalculateTo` works correctly — a value of `0x00` means chess-calibrated
(the normal factory default), `0x80` means interleaved-calibrated.

**Expected outcomes:**
- `resolutionEE` = 2.
- `calibrationModeEE` = 0 (chess-calibrated).
- `GetVdd` ≈ 3.3 V.
- `GetTa` ≈ actual room temperature ± 5 °C.

---

### 3. Refresh Rate Accuracy

**Goal:** Confirm the sensor delivers subpages at the programmed rate.

**Steps:**

1. Configure `refresh_rate: 5` (16 Hz) in YAML.
2. In `is_data_ready_()`, log a timestamp each time the function returns
   `true`.
3. Collect 20 consecutive timestamps and compute the inter-arrival intervals.
4. Repeat with `refresh_rate: 3` (4 Hz) and `refresh_rate: 1` (1 Hz).

**Expected outcomes:**

| Config value | Expected interval | Acceptable range |
|---|---|---|
| 1 (1 Hz) | 1000 ms | 900–1100 ms |
| 3 (4 Hz) | 250 ms | 225–275 ms |
| 5 (16 Hz) | 62.5 ms | 56–69 ms |

If measured intervals are ≈ 500 ms when 62.5 ms is expected, the rate
field was not written correctly — re-check Section 4.

---

### 4. Control Register Integrity

**Goal:** Verify that after `setup()` the control register matches the
intended configuration in every field, not just the rate.

This is the direct regression test for bugs #1 and #2.

**Steps:**

1. After `MLX90640_SetResolution` is called in `setup()`, add a readback:

   ```cpp
   uint16_t ctrl = 0;
   MLX90640_I2CRead(this->address_, MLX90640_CTRL_REG, 1, &ctrl);
   ESP_LOGI(TAG, "Control register readback: 0x%04X", ctrl);
   ```

2. Decode and log each field:

   ```
   Reading pattern : bit 12 = %u  (expected 1 = chess)
   ADC resolution  : bits 11:10 = %u  (expected resolutionEE)
   Refresh rate    : bits 9:7 = %u  (expected configured rate)
   ```

3. Force a corruption scenario: before calling `setup()`, write `0x0100`
   directly to `0x800D` via `MLX90640_I2CWrite`.  Re-run `setup()` and
   confirm the readback still shows the correct values (this validates that
   the explicit `SetChessMode` + `SetResolution` calls in `setup()` recover
   from prior corruption).

**Expected outcomes:**
- Bit 12 = 1 (chess mode).
- Bits 11:10 = `resolutionEE` (normally 2 = 18-bit).
- Bits 9:7 = configured refresh rate.
- After the corruption test: all three fields correct despite starting from
  `0x0100`.

---

### 5. Temperature Accuracy

**Goal:** Verify that computed pixel temperatures agree with a calibrated
reference thermometer.

**Hardware needed:** thermocouple or contact thermometer, a flat black-body
target (matte black metal, foam, or tape — high emissivity ε ≈ 0.95).

**Steps:**

1. Place the target at a stable, known temperature T_ref (try three points:
   ~15 °C, ~30 °C, ~45 °C).
2. Point the sensor at the target from 20–30 cm.
3. Wait 60 s for the scene to stabilise.
4. Log the mean temperature from the sensor (`mean_temperature` sensor).
5. Compare against T_ref.

**Emissivity:** The driver calls `CalculateTo` with ε = 0.95.  If the target
emissivity differs, apply the correction:
`T_corrected = T_measured + (1 - ε_actual) * (T_ambient - T_measured) / ε_actual`

**Expected outcomes:**
- Within ±2 °C of T_ref across the 15–45 °C range (datasheet specifies
  ±1.5 °C typical, ±2 °C worst-case at 0–80 °C object temperature with
  Ta = 25 °C).
- `GetTa` within ±2 °C of actual ambient temperature (measured with a
  separate sensor).

**If temperatures are all suspiciously high (everything at `maxtemp_`):**
- Log `GetVdd` and `GetTa` immediately after `GetFrameData`.
- If `GetVdd` >> 3.3 V or `GetTa` >> 40 °C, the resolution correction
  factor in `GetVdd` is wrong — the control register almost certainly has
  the wrong ADC resolution bits (re-run Section 4).

---

### 6. Chess Mode and Subpage Alternation

**Goal:** Confirm the sensor alternates subpages and that spatial
interpolation fills missing pixels without a checker artefact.

**Steps:**

1. Log `MLX90640_GetSubPageNumber(frame_buffer_.data())` for 10 consecutive
   frames.  Confirm it alternates: 0, 1, 0, 1, …
2. Point the sensor at a uniform-temperature flat surface (e.g. a wall).
3. Capture the raw pixel array *before* interpolation.
4. Confirm that exactly 384 pixels hold a plausible temperature and the
   other 384 hold values outside the valid range (these are the un-updated
   subpage slots that `CalculateTo` fills with stale data before our code
   interpolates them).
5. Capture the rendered image and verify it has no checker pattern.

**Expected outcomes:**
- Subpage alternates 0↔1 every frame.
- `interleaved_mode_` = 1 (chess) as returned by `GetCurMode`.
- Rendered image of a uniform surface shows uniform colour — no
  checkerboard.

---

### 7. Image Pipeline

**Goal:** Confirm the BGR→RGB565 and BGR→JPEG colour pipelines produce
correct, non-inverted images.

**Steps:**

1. **Colour order — RGB565:**
   Point the sensor at a known hot spot (≥ `maxtemp_`).  In the RGB565
   buffer, the hottest pixels should be `0xFFFF` (white).  A pixel in the
   yellow band (≈75 % of range) should have R ≈ 31, G ≈ 42, B ≈ 0
   (`0xFD00` in big-endian RGB565).

2. **Colour order — JPEG:**
   Decode the JPEG and inspect the hottest pixel.  It should be (255, 255,
   255) white, not shifted to blue.  The BGR888 data stored in
   `scaled_buffer_` has layout `[B, G, R]` per pixel — confirm the JPEG
   encoder is told `PIXEL_FORMAT_BGR888`.

3. **Geometric orientation:**
   Hold a heated object in the upper-left quadrant of the sensor field of
   view.  Confirm it appears in the upper-left of both the RGB565 display
   and the JPEG.  If it appears mirrored or flipped, the pixel index
   mapping needs investigation.

4. **Iron colormap sanity check:**
   Force a pixel to each of the 9 colormap control points and verify the
   stored BGR888 bytes match the expected values:

   | v | Expected R | Expected G | Expected B |
   |---|---|---|---|
   | 0 | 0 | 0 | 0 |
   | 32 | 74 | 0 | 85 |
   | 64 | 148 | 0 | 170 |
   | 96 | 196 | 0 | 85 |
   | 128 | 220 | 0 | 0 |
   | 160 | 255 | 110 | 0 |
   | 192 | 255 | 210 | 0 |
   | 224 | 253 | 252 | 124 |
   | 255 | 255 | 255 | 255 |

**Expected outcomes:**
- Hot pixels = white; cold pixels = black/dark purple.
- No R↔B swap.
- Scene geometry matches physical sensor orientation.

---

### 8. Known Limitations and Open Risks

The following are known issues that are **not yet fixed** and should be
accounted for in any production use.

#### 8.1 No timeout in `GetFrameData` polling loop

`MLX90640_GetFrameData` contains:

```c
while (dataReady == 0) {
    error = MLX90640_I2CRead(slaveAddr, MLX90640_STATUS_REG, 1, &statusRegister);
    ...
    dataReady = MLX90640_GET_DATA_READY(statusRegister);
}
```

If the data-ready bit never sets (e.g. sensor locked up or I²C SCL held
low by clock stretching beyond the bus timeout), this loop will spin
forever, blocking the ESPHome main loop indefinitely.

**Mitigation until fixed:** the outer `is_data_ready_()` check in
`MLX90640::loop()` means we only enter `GetFrameData` when the bit is
already set.  However this is still a theoretical hang path if the sensor
de-asserts the bit between the two reads.

**Recommended fix:** add an iteration counter or a `millis()`-based
deadline to the while loop and return an error after a timeout.

#### 8.2 `median_temperature_` is not a true median

```cpp
this->median_temp_ = (this->pixels_[165] + this->pixels_[180] +
                      this->pixels_[176] + this->pixels_[192]) / 4.0f;
```

This averages four fixed pixel indices near the centre of the 24×32 frame.
It is not a statistical median and may not represent the scene centre on
all mounting orientations.

**Recommended fix:** sort a copy of `pixels_` and return the middle value,
or clearly rename the sensor to `center_temperature`.

#### 8.3 Outlier filter is 1-D across row boundaries

`filter_outlier_pixel_` iterates the 768-pixel array linearly.  At every
row boundary (pixel 31→32, 63→64, …) it compares the last pixel of one
row with the first pixel of the next row, which are spatially non-adjacent
(they are 31 columns apart on the sensor).  This can silently fail to
catch outliers at row edges, or falsely flag a valid column-boundary
temperature step as an outlier.

**Recommended fix:** run the filter row-by-row (32 pixels each time) then
column-by-column (24 pixels each time), or implement a 2-D spatial median
filter.

#### 8.4 `TA_SHIFT` is hardcoded

```cpp
static constexpr int TA_SHIFT = 8;
```

The reflected temperature is computed as `tr = ta - TA_SHIFT`.  The
datasheet recommends 8 °C for open-air use, but a different value
(typically 0 °C) is appropriate when the sensor is enclosed or mounted
flush against a wall.  This value should be configurable via YAML.

---

### 9. Performance Benchmarks

**Goal:** Confirm each processing phase completes within budget at 16 Hz.

At 16 Hz, a new subpage is available every **62.5 ms**.  The I²C read
phase must complete within that window.  Downstream phases (calculation,
colourmap, JPEG encode) can span multiple loop ticks because the 3-phase
state machine defers them.

Add `millis()` timestamps at the start and end of each phase and log the
durations.

| Phase | Work done | Target |
|---|---|---|
| I²C read (`GetFrameData`) | Read 834 words over I²C | < 55 ms at 400 kHz |
| Calculation (`CalculateTo` + interpolation) | 768-pixel FP maths | < 100 ms |
| Colourmap + upscale | LUT lookup, NN scale, RGB565 encode | < 30 ms |
| JPEG encode | ESP32 HW JPEG encode | < 200 ms |

**I²C theoretical minimum at 400 kHz:**
- Pixel data (768 words = 1536 bytes): ≈ 34 ms
- Aux data (64 words = 128 bytes): ≈ 3 ms
- Status/control register reads and writes: ≈ 1 ms
- **Total ≈ 38 ms** — leaves ≈ 24 ms headroom before the next subpage.

If measured I²C time exceeds 55 ms, check:
1. Bus frequency is actually 400 kHz (not 100 kHz default).
2. MLX90640 clock-stretching is not causing the ESP-IDF driver to time out
   and retry.

---

### 10. Test Checklist Summary

Use this as a sign-off checklist before any production release.

- [ ] I²C bus scan shows sensor at expected address with no errors
- [ ] `MLX90640_DumpEE` returns 0
- [ ] `resolutionEE` = 2, `calibrationModeEE` = 0
- [ ] Control register readback: bit 12 = 1, bits 11:10 = 2, bits 9:7 = configured rate
- [ ] Corruption recovery test passes (write 0x0100, run setup, readback correct)
- [ ] Subpage alternates 0↔1 at the configured rate interval
- [ ] `GetVdd` within ±0.2 V of 3.3 V
- [ ] `GetTa` within ±5 °C of measured room temperature
- [ ] Mean temperature within ±2 °C of calibrated reference at 15 °C, 30 °C, 45 °C
- [ ] No checker artefact on uniform surface
- [ ] RGB565 hot pixel = 0xFFFF, cold pixel ≈ 0x0000
- [ ] JPEG hot pixel = (255, 255, 255), no R↔B swap
- [ ] Geometric orientation correct (upper-left hot object → upper-left in image)
- [ ] I²C read phase < 55 ms at 16 Hz
- [ ] No hang observed after 1 hour continuous operation
