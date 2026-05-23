/**
 * SoftwareI2CBus.cpp
 *
 * PIO I2C primitives re-implemented directly from the i2c.pio command word
 * encoding, without depending on pio_i2c.c from pico-examples.
 *
 * TX FIFO command word (16-bit half-word, written via io_rw_16):
 *   [15:10]  ICOUNT  — when > 0: "execute next ICOUNT+1 words as raw PIO instructions"
 *                      when = 0: normal 8-bit data transfer
 *   [9]      FINAL   — on last byte: for reads, causes master to NAK; for writes,
 *                      signals end-of-frame to the SM
 *   [8:1]    DATA    — the 8 data bits (SDA driven from MSB down)
 *   [0]      NAK     — for writes: set to 1 to enable ACK sampling;
 *                      for reads: set to 1 on final byte to NAK the slave
 *
 * Error signalling: the SM raises PIO interrupt <sm> on NAK; checked via
 * pio_interrupt_get(). Recovery drains TX, jumps SM back to wrap-bottom, and
 * clears the interrupt.
 */

#include "SoftwareI2CBus.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "i2c.pio.h"   // generated from i2c.pio; provides i2c_program,
                        // i2c_program_init, set_scl_sda_program_instructions,
                        // and I2C_SC{0,1}_SD{0,1} enum constants

// ---------------------------------------------------------------------------
// Command word bit positions
// ---------------------------------------------------------------------------

static constexpr int kIcountLsb = 10;
static constexpr int kFinalLsb  =  9;
static constexpr int kDataLsb   =  1;
static constexpr int kNakLsb    =  0;

// ---------------------------------------------------------------------------
// Deadline helpers
// ---------------------------------------------------------------------------

static constexpr uint64_t kNoDeadline = UINT64_MAX;

static inline bool past_deadline(uint64_t deadline) {
    return deadline != kNoDeadline && time_us_64() >= deadline;
}

// ---------------------------------------------------------------------------
// Low-level FIFO helpers
// ---------------------------------------------------------------------------

// Write a 16-bit half-word directly into the TX FIFO.
// Spins until space is available; never checks for errors.
static inline void put16(PIO pio, uint sm, uint16_t data) {
    while (pio_sm_is_tx_fifo_full(pio, sm))
        tight_loop_contents();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
    *(io_rw_16 *)&pio->txf[sm] = data;
#pragma GCC diagnostic pop
}

// Same as put16 but returns immediately if the SM has flagged a NAK error,
// preventing a deadlock when the SM stops consuming TX words after a NAK.
static inline void put_or_err(PIO pio, uint sm, uint16_t data) {
    while (pio_sm_is_tx_fifo_full(pio, sm))
        if (pio_interrupt_get(pio, sm)) return;
    if (pio_interrupt_get(pio, sm)) return;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
    *(io_rw_16 *)&pio->txf[sm] = data;
#pragma GCC diagnostic pop
}

static inline bool check_err(PIO pio, uint sm) {
    return pio_interrupt_get(pio, sm);
}

// Drain TX, jump SM back to its wrap-bottom (program start), clear interrupt.
static inline void resume_after_error(PIO pio, uint sm) {
    pio_sm_drain_tx_fifo(pio, sm);
    pio_sm_exec(pio, sm,
        (pio->sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS)
            >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    pio_interrupt_clear(pio, sm);
}

static inline void rx_enable(PIO pio, uint sm, bool en) {
    if (en)
        hw_set_bits  (&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    else
        hw_clear_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
}

// Wait until the TX FIFO has drained and the SM has stalled, or an error fires.
// Returns false if the deadline was exceeded before the SM became idle.
static bool wait_idle(PIO pio, uint sm, uint64_t deadline = kNoDeadline) {
    pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    while (!(pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm))) &&
           !check_err(pio, sm)) {
        if (past_deadline(deadline)) return false;
        tight_loop_contents();
    }
    return true;
}

// ---------------------------------------------------------------------------
// I2C bus-condition primitives
// ---------------------------------------------------------------------------
// An ICOUNT escape word with value N tells the SM "treat the next N+1 words
// as raw PIO instructions to execute directly", enabling START/STOP/REPSTART
// to be injected inline with data transfers.

static void bus_start(PIO pio, uint sm) {
    // 3 raw instructions: pull SDA low (START), then pull SCL low
    put_or_err(pio, sm, 2u << kIcountLsb);
    put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]);
    put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]);
    put_or_err(pio, sm, (uint16_t)pio_encode_mov(pio_isr, pio_null));  // clear ISR
}

static void bus_stop(PIO pio, uint sm) {
    // 3 raw instructions: SDA low, SCL high, SDA high (STOP)
    put_or_err(pio, sm, 2u << kIcountLsb);
    put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]);
    put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]);
    put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1]);
}

static void bus_repstart(PIO pio, uint sm) {
    // 5 raw instructions: SCL low+SDA high, SCL high, SDA low, SCL low, clear ISR
    put_or_err(pio, sm, 4u << kIcountLsb);
    put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD1]);
    put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1]);
    put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]);
    put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]);
    put_or_err(pio, sm, (uint16_t)pio_encode_mov(pio_isr, pio_null));
}

// Abort a transaction in progress: drain TX, reset SM, send STOP.
// Used for both NAK errors and timeouts.
static void abort_transaction(PIO pio, uint sm, bool &bus_held) {
    resume_after_error(pio, sm);
    bus_stop(pio, sm);
    wait_idle(pio, sm);  // STOP is only 4 words — completes in microseconds
    bus_held = false;
}

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

SoftwareI2CBus::SoftwareI2CBus(uint8_t id, PIO pio, uint sm, uint offset, uint sda_pin, uint scl_pin, uint freq_hz)
    : I2CBus(false), id_(id), pio_(pio), sm_(sm), prog_offset_(offset)
{
    i2c_program_init(pio_, sm_, prog_offset_, sda_pin, scl_pin);
    // i2c_program_init hardcodes 400 kHz; override for the requested frequency.
    // The PIO i2c program uses 32 PIO cycles per I2C bit.
    pio_sm_set_clkdiv(pio_, sm_,
        (float)clock_get_hz(clk_sys) / (32.0f * (float)freq_hz));
}

SoftwareI2CBus::~SoftwareI2CBus() {
    pio_sm_set_enabled(pio_, sm_, false);
    // The program is owned by the caller (loaded once per PIO block in init_i2c_buses).
    // Multiple SoftwareI2CBus instances share it, so do NOT remove it here.
}

// ---------------------------------------------------------------------------
// Internal write / read  (shared by blocking and timeout public methods)
// ---------------------------------------------------------------------------

static int do_write(PIO pio, uint sm, bool &bus_held,
                    uint8_t addr, const uint8_t *data, size_t len,
                    bool nostop, uint64_t deadline) {
    bus_held ? bus_repstart(pio, sm) : bus_start(pio, sm);
    rx_enable(pio, sm, false);

    // addr<<2: 7-bit address at bits[8:2]; bit[1]=0 (write); bit[0]=1 (check ACK)
    put16(pio, sm, (uint16_t)((addr << 2) | 1u));

    size_t         remaining = len;
    const uint8_t *p         = data;
    while (remaining && !check_err(pio, sm)) {
        if (past_deadline(deadline)) {
            abort_transaction(pio, sm, bus_held);
            return PICO_ERROR_TIMEOUT;
        }
        if (!pio_sm_is_tx_fifo_full(pio, sm)) {
            --remaining;
            put_or_err(pio, sm, (uint16_t)(
                (*p++ << kDataLsb) | ((remaining == 0) << kFinalLsb) | 1u));
        }
    }

    if (!nostop || check_err(pio, sm)) {
        bus_stop(pio, sm);
        bus_held = false;
    } else {
        bus_held = true;
    }

    if (!wait_idle(pio, sm, deadline)) {
        abort_transaction(pio, sm, bus_held);
        return PICO_ERROR_TIMEOUT;
    }

    if (check_err(pio, sm)) {
        abort_transaction(pio, sm, bus_held);
        return PICO_ERROR_GENERIC;
    }

    return (int)len;
}

static int do_read(PIO pio, uint sm, bool &bus_held,
                   uint8_t addr, uint8_t *dst, size_t len,
                   bool nostop, uint64_t deadline) {
    bus_held ? bus_repstart(pio, sm) : bus_start(pio, sm);
    rx_enable(pio, sm, true);

    // Flush any stale RX bytes from a prior transaction.
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
        (void)pio_sm_get(pio, sm);

    // bit[1]=1 (read direction), bit[0]=1 (check ACK)
    put16(pio, sm, (uint16_t)((addr << 2) | 3u));

    // Pipeline: stuff 0xFF clock-tokens into TX while draining received bytes
    // from RX. The SM echoes the address word back as the first RX byte; discard it.
    uint32_t tx_remain = len;
    size_t   rx_remain = len;
    bool     first     = true;

    while ((tx_remain || rx_remain) && !check_err(pio, sm)) {
        if (past_deadline(deadline)) {
            rx_enable(pio, sm, false);
            abort_transaction(pio, sm, bus_held);
            return PICO_ERROR_TIMEOUT;
        }
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, sm)) {
            --tx_remain;
            put16(pio, sm, (uint16_t)(
                (0xffu << kDataLsb) |
                (tx_remain == 0 ? (1u << kFinalLsb) | (1u << kNakLsb) : 0u)
            ));
        }
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            if (first) {
                (void)pio_sm_get(pio, sm);  // discard address echo
                first = false;
            } else {
                --rx_remain;
                *dst++ = (uint8_t)pio_sm_get(pio, sm);
            }
        }
    }

    if (!nostop || check_err(pio, sm)) {
        bus_stop(pio, sm);
        bus_held = false;
    } else {
        bus_held = true;
    }

    if (!wait_idle(pio, sm, deadline)) {
        rx_enable(pio, sm, false);
        abort_transaction(pio, sm, bus_held);
        return PICO_ERROR_TIMEOUT;
    }

    rx_enable(pio, sm, false);

    if (check_err(pio, sm)) {
        abort_transaction(pio, sm, bus_held);
        return PICO_ERROR_GENERIC;
    }

    return (int)len;
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

int SoftwareI2CBus::write(uint8_t addr, const uint8_t *data, size_t len, bool nostop) {
    return do_write(pio_, sm_, bus_held_, addr, data, len, nostop, kNoDeadline);
}

int SoftwareI2CBus::read(uint8_t addr, uint8_t *dst, size_t len, bool nostop) {
    return do_read(pio_, sm_, bus_held_, addr, dst, len, nostop, kNoDeadline);
}

int SoftwareI2CBus::write_timeout_us(uint8_t addr, const uint8_t *data, size_t len, bool nostop, uint32_t timeout_us) {
    return do_write(pio_, sm_, bus_held_, addr, data, len, nostop, time_us_64() + timeout_us);
}

int SoftwareI2CBus::read_timeout_us(uint8_t addr, uint8_t *dst, size_t len, bool nostop, uint32_t timeout_us) {
    return do_read(pio_, sm_, bus_held_, addr, dst, len, nostop, time_us_64() + timeout_us);
}
