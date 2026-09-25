#!/usr/bin/env python3
"""Make an MP4 web-playable: move the 'moov' index in front of the 'mdat' media data and fix the chunk offsets
(what ffmpeg's -movflags +faststart / qt-faststart do). No re-encoding, so quality is unchanged.
OpenCV's writer puts moov at the end of the file, which browsers, GitHub's viewer and some players cannot stream.

usage: python3 scripts/tools/faststart.py in.mp4 [out.mp4]      (out defaults to in-place)"""
import os
import struct
import sys


def top_atoms(data):
    pos, out = 0, []
    while pos < len(data):
        size, typ = struct.unpack('>I4s', data[pos:pos + 8])
        hdr = 8
        if size == 1:
            size = struct.unpack('>Q', data[pos + 8:pos + 16])[0]
            hdr = 16
        elif size == 0:
            size = len(data) - pos
        out.append((typ.decode('latin1'), pos, size, hdr))
        pos += size
    return out


def shift_offsets(moov, delta):
    """Add delta to every stco/co64 chunk offset inside the moov atom (in place)."""
    stack = [(8, len(moov))]
    while stack:
        start, end = stack.pop()
        pos = start
        while pos + 8 <= end:
            size, typ = struct.unpack('>I4s', moov[pos:pos + 8])
            if size < 8:
                break
            body = pos + 8
            if typ in (b'trak', b'mdia', b'minf', b'stbl', b'edts', b'dinf'):
                stack.append((body, pos + size))
            elif typ == b'stco':
                n = struct.unpack('>I', moov[body + 4:body + 8])[0]
                for i in range(n):
                    o = body + 8 + 4 * i
                    v = struct.unpack('>I', moov[o:o + 4])[0] + delta
                    moov[o:o + 4] = struct.pack('>I', v)
            elif typ == b'co64':
                n = struct.unpack('>I', moov[body + 4:body + 8])[0]
                for i in range(n):
                    o = body + 8 + 8 * i
                    v = struct.unpack('>Q', moov[o:o + 8])[0] + delta
                    moov[o:o + 8] = struct.pack('>Q', v)
            pos += size


def main(src, dst=None):
    data = open(src, 'rb').read()
    atoms = top_atoms(data)
    names = [a[0] for a in atoms]
    if 'moov' not in names or 'mdat' not in names:
        sys.exit('not a plain MP4 (moov/mdat missing)')
    if names.index('moov') < names.index('mdat'):
        print('already fast-start:', src)
        return
    moov_a = next(a for a in atoms if a[0] == 'moov')
    moov = bytearray(data[moov_a[1]:moov_a[1] + moov_a[2]])
    shift_offsets(moov, len(moov))          # media data moves back by exactly the size of moov
    out = bytearray()
    for name, pos, size, hdr in atoms:
        if name == 'moov':
            continue
        out += data[pos:pos + size]
        if name in ('ftyp', 'free') and names[names.index(name) + 1] == 'mdat':
            out += moov
        elif name == 'ftyp' and 'free' not in names:
            out += moov
    # if there is no free atom right before mdat the moov was inserted after ftyp above
    tmp = (dst or src) + '.tmp'
    open(tmp, 'wb').write(out)
    os.replace(tmp, dst or src)
    print('fast-start written:', dst or src, len(out) // 1000, 'KB')


if __name__ == '__main__':
    main(*sys.argv[1:3])
