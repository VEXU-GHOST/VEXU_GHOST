#pragma once

#include "I2CBus.h"
#include "hardware/pio.h"

// PIO-based bit-bang I2C bus using the RP2040 PIO state machine.
//
// Only requires i2c.pio (pico-examples pio/i2c) — the C wrapper pio_i2c.c is
// NOT needed. Primitives are implemented directly from the PIO command encoding.
//
// Add to CMakeLists.txt:
//   pico_generate_pio_header(polling_firmware ${CMAKE_CURRENT_LIST_DIR}/src/i2c.pio)
//   target_link_libraries(polling_firmware ... hardware_pio hardware_clocks)
//
// Hardware constraint: scl_pin MUST equal sda_pin + 1 (PIO sideset requirement).
//
// The caller must load the i2c_program into the PIO block exactly once
// (via pio_add_program) before constructing any SoftwareI2CBus on that block,
// and pass the returned offset as prog_offset. Multiple instances on the same
// PIO block share that program offset — the destructor does NOT remove it.
//
// nostop is fully supported. A repeated START is generated automatically when
// write(..., nostop=true) is immediately followed by read() or write().
class SoftwareI2CBus final : public I2CBus {
public:
    SoftwareI2CBus(uint8_t id, PIO pio, uint sm, uint offset, uint sda_pin, uint scl_pin, uint freq_hz);
    ~SoftwareI2CBus() override;

    int write(uint8_t addr, const uint8_t *data, size_t len, bool nostop = false) override;
    int read(uint8_t addr, uint8_t *dst,         size_t len, bool nostop = false) override;

    int write_timeout_us(uint8_t addr, const uint8_t *data, size_t len, bool nostop, uint32_t timeout_us) override;
    int read_timeout_us(uint8_t addr, uint8_t *dst,         size_t len, bool nostop, uint32_t timeout_us) override;

    uint8_t get_id() const override { return id_; }

private:
    uint8_t id_;
    PIO     pio_;
    uint    sm_;
    uint    prog_offset_;
    bool    bus_held_ = false;  // true after a nostop write; triggers REPSTART on next op
};
