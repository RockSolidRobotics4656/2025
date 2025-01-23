from typing import *

class EncoderAdapter:
    def __init__(self, supplier: Callable[[], float], noffset = 0.0):
        self.supplier = supplier
        self.ticks_per_unit = 1.0
        self.offset = noffset # assigning this is same as using attachoffset
        self.rotational = False
    def _read_converted(self):
        return self.supplier() / self.ticks_per_unit
    def _read_converted_woffset(self):
        return self._read_converted() + self.offset

    def set_ticks_per_unit(self, ticks_per_unit: float):
        self.ticks_per_unit = ticks_per_unit
    def set_rotational(self, rotational: bool):
        self.rotational = rotational
    def reset(self, noffset=0.0):
        self.offset = -self._read_converted() + noffset
    def __call__(self):
        val = self._read_converted_woffset()
        if self.rotational:
            while val < 0: val += 360
            val = val % 360
        return val
