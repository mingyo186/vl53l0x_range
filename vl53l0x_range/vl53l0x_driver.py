"""VL53L0X I2C Driver - Time-of-Flight laser distance sensor."""

import random
import struct
import time


class FakeVL53L0XDriver:
    """Fake driver that generates random range data without I2C hardware."""

    def __init__(self, **kwargs):
        self._base = 0.5

    def read_range_mm(self) -> int:
        """Return distance in millimetres."""
        self._base += random.gauss(0.0, 0.01)
        self._base = max(0.03, min(2.0, self._base))
        return int(self._base * 1000 + random.gauss(0.0, 5.0))

    def model_id(self) -> int:
        return 0xEE

    def set_range_mode(self, mode: str):
        pass

    def close(self):
        pass


class VL53L0XDriver:
    """Low-level I2C driver for ST VL53L0X.

    Datasheet: VL53L0X (ST DocID029104)
    Uses simplified register-level access based on the ST API reference
    and Pololu VL53L0X library.
    """

    # ── Register Map ──────────────────────────────────────────────
    REG_SYSRANGE_START                = 0x00
    REG_SYSTEM_SEQUENCE_CONFIG        = 0x01
    REG_SYSTEM_INTERRUPT_CONFIG_GPIO  = 0x0A
    REG_SYSTEM_INTERRUPT_CLEAR        = 0x0B
    REG_RESULT_INTERRUPT_STATUS       = 0x13
    REG_RESULT_RANGE_STATUS           = 0x14
    REG_MSRC_CONFIG_CONTROL           = 0x60
    REG_VHV_CONFIG_PAD_SCL_SDA        = 0x89
    REG_FINAL_RANGE_CONFIG_MIN_COUNT  = 0x44
    REG_MODEL_ID                      = 0xC0   # should read 0xEE

    # ── Timing budget presets (µs) ───────────────────────────────
    TIMING_BUDGET = {
        'short':  20000,    # ~1.2 m, high accuracy
        'medium': 33000,    # ~1.5 m, balanced (default)
        'long':   200000,   # ~2.0 m, max range
    }

    def __init__(self, bus: int = 1, address: int = 0x29,
                 range_mode: str = 'medium'):
        from smbus2 import SMBus

        self.address = address
        self.bus = SMBus(bus)

        self._init_device()
        self.set_range_mode(range_mode)

    # ── Initialisation ───────────────────────────────────────────
    def _init_device(self):
        # Enable 2.8V I/O mode if available
        val = self._read_reg(self.REG_VHV_CONFIG_PAD_SCL_SDA)
        self._write_reg(self.REG_VHV_CONFIG_PAD_SCL_SDA, val | 0x01)

        # Set standard I2C mode
        self._write_reg(0x88, 0x00)

        self._write_reg(0x80, 0x01)
        self._write_reg(0xFF, 0x01)
        self._write_reg(0x00, 0x00)
        self._stop_variable = self._read_reg(0x91)
        self._write_reg(0x00, 0x01)
        self._write_reg(0xFF, 0x00)
        self._write_reg(0x80, 0x00)

        # Enable MSRC and TCC in sequence config
        msrc = self._read_reg(self.REG_MSRC_CONFIG_CONTROL)
        self._write_reg(self.REG_MSRC_CONFIG_CONTROL, msrc | 0x12)

        # Set signal rate limit to 0.25 MCPS (default)
        self._write_reg16(self.REG_FINAL_RANGE_CONFIG_MIN_COUNT, 0x0020)

        # Set default sequence config
        self._write_reg(self.REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)

        # GPIO interrupt on new sample ready
        self._write_reg(self.REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)

        # Active low interrupt
        val = self._read_reg(0x84)
        self._write_reg(0x84, val & ~0x10)

        # Clear interrupt
        self._write_reg(self.REG_SYSTEM_INTERRUPT_CLEAR, 0x01)

    # ── Range mode ───────────────────────────────────────────────
    def set_range_mode(self, mode: str):
        """Set measurement timing budget: 'short', 'medium', 'long'."""
        budget = self.TIMING_BUDGET.get(mode, self.TIMING_BUDGET['medium'])
        # Set measurement timing budget via register 0x01
        if budget <= 20000:
            self._write_reg(self.REG_SYSTEM_SEQUENCE_CONFIG, 0xE8)
        elif budget <= 33000:
            self._write_reg(self.REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)
        else:
            self._write_reg(self.REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)
        time.sleep(0.01)

    # ── Register helpers ─────────────────────────────────────────
    def _read_reg(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def _write_reg(self, reg: int, val: int):
        self.bus.write_byte_data(self.address, reg, val & 0xFF)

    def _read_reg16(self, reg: int) -> int:
        buf = self.bus.read_i2c_block_data(self.address, reg, 2)
        return (buf[0] << 8) | buf[1]

    def _write_reg16(self, reg: int, val: int):
        self.bus.write_i2c_block_data(
            self.address, reg, [(val >> 8) & 0xFF, val & 0xFF])

    # ── Single-shot ranging ──────────────────────────────────────
    def read_range_mm(self) -> int:
        """Perform single-shot range measurement.

        Returns:
            distance in millimetres (0 or 8190 indicates error/out-of-range)
        """
        # Start single-shot measurement
        self._write_reg(0x80, 0x01)
        self._write_reg(0xFF, 0x01)
        self._write_reg(0x00, 0x00)
        self._write_reg(0x91, self._stop_variable)
        self._write_reg(0x00, 0x01)
        self._write_reg(0xFF, 0x00)
        self._write_reg(0x80, 0x00)

        self._write_reg(self.REG_SYSRANGE_START, 0x01)

        # Wait for start bit to clear
        for _ in range(100):
            if self._read_reg(self.REG_SYSRANGE_START) & 0x01 == 0:
                break
            time.sleep(0.001)

        # Wait for measurement ready
        for _ in range(100):
            if self._read_reg(self.REG_RESULT_INTERRUPT_STATUS) & 0x07:
                break
            time.sleep(0.001)

        # Read result (range value at offset 10-11 of RESULT_RANGE_STATUS block)
        buf = bytes(self.bus.read_i2c_block_data(
            self.address, self.REG_RESULT_RANGE_STATUS, 12))
        range_mm = struct.unpack_from('>H', buf, 10)[0]

        # Clear interrupt
        self._write_reg(self.REG_SYSTEM_INTERRUPT_CLEAR, 0x01)

        return range_mm

    def model_id(self) -> int:
        """Read MODEL_ID register (should return 0xEE for VL53L0X)."""
        return self._read_reg(self.REG_MODEL_ID)

    def close(self):
        self.bus.close()
