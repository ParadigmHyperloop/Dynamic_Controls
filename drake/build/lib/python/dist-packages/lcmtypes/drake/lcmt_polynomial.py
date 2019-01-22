"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

import cStringIO as StringIO
import struct

class lcmt_polynomial(object):
    __slots__ = ["timestamp", "num_coefficients", "coefficients"]

    def __init__(self):
        self.timestamp = 0
        self.num_coefficients = 0
        self.coefficients = []

    def encode(self):
        buf = StringIO.StringIO()
        buf.write(lcmt_polynomial._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.timestamp, self.num_coefficients))
        buf.write(struct.pack('>%dd' % self.num_coefficients, *self.coefficients[:self.num_coefficients]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = StringIO.StringIO(data)
        if buf.read(8) != lcmt_polynomial._get_packed_fingerprint():
            raise ValueError("Decode error")
        return lcmt_polynomial._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = lcmt_polynomial()
        self.timestamp, self.num_coefficients = struct.unpack(">qi", buf.read(12))
        self.coefficients = struct.unpack('>%dd' % self.num_coefficients, buf.read(self.num_coefficients * 8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if lcmt_polynomial in parents: return 0
        tmphash = (0x34bdce9b4a6e2c25) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if lcmt_polynomial._packed_fingerprint is None:
            lcmt_polynomial._packed_fingerprint = struct.pack(">Q", lcmt_polynomial._get_hash_recursive([]))
        return lcmt_polynomial._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

