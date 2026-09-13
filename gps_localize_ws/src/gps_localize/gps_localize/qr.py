"""QR encoding, with no dependencies, so the robot can hand a phone a link.

WHY THIS IS WRITTEN OUT RATHER THAN IMPORTED

The obvious move is `pip install qrcode`. It is also the wrong one here. This
project promises a one-command install, a Docker image, and a Windows viewer
that runs from a frozen build, and every one of those breaks differently when a
new runtime dependency appears: apt has no `python3-qrcode` on some images, pip
is absent inside the slim container, and the frozen build silently omits
anything it cannot see imported at the top level. A ~300 line pure-Python
encoder costs less than any of that, and it works identically everywhere.

WHAT IT SUPPORTS

Byte mode, error correction level M, versions 1 to 10 - up to 213 bytes, which
covers any LAN URL several times over. Anything longer raises rather than
silently truncating, because a QR that encodes half a URL still scans, and hands
the person a link that quietly goes nowhere.

HOW IT IS KNOWN TO BE CORRECT

It is verified by DECODING, not by inspection. test_qr.py renders each code and
reads it back with zbarimg - a real scanner, rather than our own assumptions run
backwards. That distinction earned its keep immediately: it caught two bugs here
that were invisible to the eye, a Reed-Solomon generator that was not monic and
format information written with its two copies transposed. Both produced codes
that looked entirely correct and that no phone could read.

The same tests now cover the TERMINAL renderings, not only the encoder, because
a correct code can still be painted unscannably. They rasterise the characters
the way a terminal grid does - fixed cell advance, both themes - and scan the
result. That caught the third bug of exactly this shape: to_ascii is drawn in
the terminal's foreground colour, so on a dark theme every module is inverted
and no scanner will read it, while it still looks exactly like a QR code.
"""

from __future__ import annotations

# ---------------------------------------------------------------- GF(256) ---
# Reed-Solomon over the field the QR spec uses: x^8 + x^4 + x^3 + x^2 + 1.
_EXP = [0] * 512
_LOG = [0] * 256


def _init_tables():
    x = 1
    for i in range(255):
        _EXP[i] = x
        _LOG[x] = i
        x <<= 1
        if x & 0x100:
            x ^= 0x11D
    for i in range(255, 512):
        _EXP[i] = _EXP[i - 255]


_init_tables()


def _mul(a, b):
    if a == 0 or b == 0:
        return 0
    return _EXP[_LOG[a] + _LOG[b]]


def _rs_generator(degree):
    """Generator polynomial for `degree` error-correction codewords.

    Highest degree first and MONIC - poly[0] is 1. The division below assumes
    that and only ever reads poly[1:], so a generator that is merely a scalar
    multiple of the right one produces plausible-looking codewords that no
    scanner accepts. That was the first bug a real decoder caught here.
    """
    poly = [1]
    for i in range(degree):
        # Multiply by (x + a^i); addition is XOR in this field.
        nxt = [0] * (len(poly) + 1)
        for j, coef in enumerate(poly):
            nxt[j] ^= coef
            nxt[j + 1] ^= _mul(coef, _EXP[i])
        poly = nxt
    return poly


def _rs_remainder(data, degree):
    """The error-correction codewords for one block."""
    gen = _rs_generator(degree)
    rem = [0] * degree
    for byte in data:
        factor = byte ^ rem[0]
        rem = rem[1:] + [0]
        for i in range(degree):
            rem[i] ^= _mul(gen[i + 1], factor)
    return rem


# ------------------------------------------------------- version geometry ---
# Level M only. Per version: (data codewords, EC codewords per block,
# [(block count, data codewords in each), ...]).
_SPEC = {
    1:  (16,  10, [(1, 16)]),
    2:  (28,  16, [(1, 28)]),
    3:  (44,  26, [(1, 44)]),
    4:  (64,  18, [(2, 32)]),
    5:  (86,  24, [(2, 43)]),
    6:  (108, 16, [(4, 27)]),
    7:  (124, 18, [(4, 31)]),
    8:  (154, 22, [(2, 38), (2, 39)]),
    9:  (182, 22, [(3, 36), (2, 37)]),
    10: (216, 26, [(4, 43), (1, 44)]),
}

_ALIGN = {
    1: [], 2: [6, 18], 3: [6, 22], 4: [6, 26], 5: [6, 30],
    6: [6, 34], 7: [6, 22, 38], 8: [6, 24, 42], 9: [6, 26, 46],
    10: [6, 28, 50],
}


def _count_bits(version):
    """Byte-mode character-count indicator width."""
    return 8 if version <= 9 else 16


def _pick_version(length):
    for version in sorted(_SPEC):
        data_codewords = _SPEC[version][0]
        needed = 4 + _count_bits(version) + 8 * length
        if needed <= data_codewords * 8:
            return version
    raise ValueError(
        'need %d bytes; this encoder handles up to %d (version 10, level M). '
        'Refusing rather than truncating - a half-encoded URL still scans and '
        'sends someone to a link that does not work.'
        % (length, _SPEC[10][0] - 3))


# ------------------------------------------------------------- bitstream ---
def _encode_data(payload: bytes, version: int) -> list[int]:
    """Payload -> the full codeword sequence, data and EC interleaved."""
    data_codewords, ec_per_block, blocks = _SPEC[version]

    bits = []

    def put(value, width):
        for i in range(width - 1, -1, -1):
            bits.append((value >> i) & 1)

    put(0b0100, 4)                       # byte mode
    put(len(payload), _count_bits(version))
    for byte in payload:
        put(byte, 8)

    capacity = data_codewords * 8
    put(0, min(4, capacity - len(bits)))          # terminator
    while len(bits) % 8:                          # pad to a byte boundary
        bits.append(0)

    codewords = [int(''.join(str(b) for b in bits[i:i + 8]), 2)
                 for i in range(0, len(bits), 8)]
    # The spec's alternating pad bytes.
    for i in range(len(codewords), data_codewords):
        codewords.append(0xEC if (i - len(codewords)) % 2 == 0 else 0x11)
    # Recompute cleanly: pad alternates from the first pad byte onward.
    codewords = codewords[:data_codewords]
    pad = [0xEC, 0x11]
    n = len(bits) // 8
    for i in range(n, data_codewords):
        codewords[i] = pad[(i - n) % 2]

    # Split into blocks, compute EC per block, then interleave both.
    data_blocks, ec_blocks, at = [], [], 0
    for count, size in blocks:
        for _ in range(count):
            block = codewords[at:at + size]
            at += size
            data_blocks.append(block)
            ec_blocks.append(_rs_remainder(block, ec_per_block))

    out = []
    for i in range(max(len(b) for b in data_blocks)):
        for block in data_blocks:
            if i < len(block):
                out.append(block[i])
    for i in range(ec_per_block):
        for block in ec_blocks:
            out.append(block[i])
    return out


# --------------------------------------------------------------- matrix ----
def _new_matrix(size):
    return [[None] * size for _ in range(size)]


def _place_finder(m, row, col):
    for r in range(-1, 8):
        for c in range(-1, 8):
            rr, cc = row + r, col + c
            if 0 <= rr < len(m) and 0 <= cc < len(m):
                inside = (0 <= r < 7 and 0 <= c < 7)
                dark = inside and (r in (0, 6) or c in (0, 6)
                                   or (2 <= r <= 4 and 2 <= c <= 4))
                m[rr][cc] = 1 if dark else 0


def _place_function_patterns(m, version):
    size = len(m)
    _place_finder(m, 0, 0)
    _place_finder(m, 0, size - 7)
    _place_finder(m, size - 7, 0)

    for i in range(8, size - 8):                  # timing
        bit = 1 if i % 2 == 0 else 0
        m[6][i] = bit
        m[i][6] = bit

    centres = _ALIGN[version]
    for r in centres:
        for c in centres:
            # Skip the three corners already occupied by finders.
            if (r, c) in ((6, 6), (6, size - 7), (size - 7, 6)):
                continue
            for dr in range(-2, 3):
                for dc in range(-2, 3):
                    dark = max(abs(dr), abs(dc)) != 1
                    m[r + dr][c + dc] = 1 if dark else 0

    m[size - 8][8] = 1                            # the always-dark module


def _reserve_format(m):
    size = len(m)
    for i in range(9):
        if m[8][i] is None:
            m[8][i] = 0
        if m[i][8] is None:
            m[i][8] = 0
    for i in range(8):
        if m[8][size - 1 - i] is None:
            m[8][size - 1 - i] = 0
        if m[size - 1 - i][8] is None:
            m[size - 1 - i][8] = 0


def _function_mask(version):
    """A matrix marking every module reserved by a function pattern."""
    size = version * 4 + 17
    m = _new_matrix(size)
    _place_function_patterns(m, version)
    _reserve_format(m)
    if version >= 7:
        for i in range(6):
            for j in range(3):
                m[size - 11 + j][i] = 0
                m[i][size - 11 + j] = 0
    return [[cell is not None for cell in row] for row in m]


def _place_data(m, reserved, codewords):
    size = len(m)
    bits = []
    for cw in codewords:
        for i in range(7, -1, -1):
            bits.append((cw >> i) & 1)

    idx, upward, col = 0, True, size - 1
    while col > 0:
        if col == 6:                              # the vertical timing column
            col -= 1
        rows = range(size - 1, -1, -1) if upward else range(size)
        for row in rows:
            for c in (col, col - 1):
                if not reserved[row][c]:
                    m[row][c] = bits[idx] if idx < len(bits) else 0
                    idx += 1
        upward = not upward
        col -= 2


_MASKS = (
    lambda r, c: (r + c) % 2 == 0,
    lambda r, c: r % 2 == 0,
    lambda r, c: c % 3 == 0,
    lambda r, c: (r + c) % 3 == 0,
    lambda r, c: (r // 2 + c // 3) % 2 == 0,
    lambda r, c: (r * c) % 2 + (r * c) % 3 == 0,
    lambda r, c: ((r * c) % 2 + (r * c) % 3) % 2 == 0,
    lambda r, c: ((r + c) % 2 + (r * c) % 3) % 2 == 0,
)


def _penalty(m):
    """The spec's four penalty rules, used to choose the least-bad mask."""
    size = len(m)
    score = 0

    for line in list(m) + [list(col) for col in zip(*m)]:
        run, prev = 1, line[0]
        for cell in line[1:]:
            if cell == prev:
                run += 1
            else:
                if run >= 5:
                    score += 3 + (run - 5)
                run, prev = 1, cell
        if run >= 5:
            score += 3 + (run - 5)

    for r in range(size - 1):
        for c in range(size - 1):
            block = (m[r][c], m[r][c + 1], m[r + 1][c], m[r + 1][c + 1])
            if all(block) or not any(block):
                score += 3

    pattern = [1, 0, 1, 1, 1, 0, 1]
    for line in list(m) + [list(col) for col in zip(*m)]:
        for i in range(size - 6):
            if line[i:i + 7] == pattern:
                before = line[max(0, i - 4):i]
                after = line[i + 7:i + 11]
                if len(before) == 4 and not any(before):
                    score += 40
                if len(after) == 4 and not any(after):
                    score += 40

    dark = sum(sum(row) for row in m)
    percent = dark * 100 // (size * size)
    score += 10 * (abs(percent - 50) // 5)
    return score


def _format_bits(mask):
    """15-bit format information for level M, BCH-protected and masked."""
    value = (0b00 << 3) | mask                    # 00 = level M
    rem = value
    for _ in range(10):
        rem = (rem << 1) ^ (0x537 if (rem >> 9) & 1 else 0)
    return ((value << 10) | (rem & 0x3FF)) ^ 0x5412


def _version_bits(version):
    rem = version
    for _ in range(12):
        rem = (rem << 1) ^ (0x1F25 if (rem >> 11) & 1 else 0)
    return (version << 12) | (rem & 0xFFF)


def _apply_format(m, mask):
    """Write both copies of the 15-bit format information.

    The two copies are NOT symmetric, and writing them transposed produces a
    code with correct data that no scanner will read - the format block says
    which mask was used, so getting it wrong makes every data module wrong.
    Copy one wraps the top-left corner going down column 8 then along row 8;
    copy two runs along row 8 on the right and up column 8 at the bottom.
    """
    size = len(m)
    bits = _format_bits(mask)
    for i in range(15):
        bit = (bits >> i) & 1
        if i < 6:
            m[i][8] = bit
        elif i == 6:
            m[7][8] = bit
        elif i == 7:
            m[8][8] = bit
        elif i == 8:
            m[8][7] = bit
        else:
            m[8][14 - i] = bit

        if i < 8:
            m[8][size - 1 - i] = bit
        else:
            m[size - 15 + i][8] = bit
    m[size - 8][8] = 1                    # always dark


def _apply_version(m, version):
    if version < 7:
        return
    size = len(m)
    bits = _version_bits(version)
    for i in range(18):
        bit = (bits >> i) & 1
        r, c = i // 3, i % 3
        m[size - 11 + c][r] = bit
        m[r][size - 11 + c] = bit


def encode(text: str) -> list[list[int]]:
    """Text -> a QR matrix of 0/1, no quiet zone. Raises if it will not fit."""
    payload = text.encode('utf-8')
    version = _pick_version(len(payload))
    codewords = _encode_data(payload, version)

    reserved = _function_mask(version)
    size = version * 4 + 17

    best, best_score = None, None
    for mask in range(8):
        m = _new_matrix(size)
        _place_function_patterns(m, version)
        _apply_version(m, version)
        _place_data(m, reserved, codewords)
        for r in range(size):
            for c in range(size):
                if not reserved[r][c] and _MASKS[mask](r, c):
                    m[r][c] ^= 1
        _apply_format(m, mask)
        m = [[0 if cell is None else cell for cell in row] for row in m]
        score = _penalty(m)
        if best_score is None or score < best_score:
            best, best_score = m, score
    return best


# --------------------------------------------------------------- render ----
def to_svg(text: str, module: int = 6, quiet: int = 4) -> str:
    """A QR as standalone SVG - scales cleanly and needs no image library."""
    m = encode(text)
    size = len(m) + quiet * 2
    px = size * module
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
        'viewBox="0 0 %d %d" shape-rendering="crispEdges" '
        'role="img" aria-label="QR code linking to %s">' % (px, px, size, size, text),
        '<rect width="%d" height="%d" fill="#ffffff"/>' % (size, size),
    ]
    for r, row in enumerate(m):
        c = 0
        while c < len(row):
            if row[c]:
                start = c
                while c < len(row) and row[c]:
                    c += 1
                parts.append('<rect x="%d" y="%d" width="%d" height="1" '
                             'fill="#000000"/>' % (start + quiet, r + quiet, c - start))
            else:
                c += 1
    parts.append('</svg>')
    return ''.join(parts)


def to_ansi(text: str, quiet: int = 4) -> str:
    """A QR for a terminal, with the colours stated rather than inherited.

    THIS IS THE FORM TO PUT IN FRONT OF A CAMERA. A scanner needs dark modules
    on a light quiet zone. Drawing with block characters inherits the terminal's
    own colours, so the same code comes out correct on a light theme and
    INVERTED on a dark one - and an inverted code is rejected outright by
    zbar and by many phone cameras. Setting the background explicitly costs two
    characters per module and works on either theme.

    Measured, not assumed: test_qr.py rasterises both forms the way a terminal
    paints them and reads them back with zbarimg. The half-block form in
    to_ascii scans on a light theme at every cell shape tried and scans on NO
    dark one; this form scans on both.

    The quiet zone is four modules because that is what the spec requires, and
    the terminal gives us no margin of its own - the prompt line sits directly
    above whatever we print.
    """
    m = encode(text)
    dark = '\033[40m  \033[0m'          # black background, two spaces wide
    light = '\033[107m  \033[0m'        # bright white background

    width = len(m) + quiet * 2
    blank = light * width
    rows = [blank] * quiet
    for row in m:
        rows.append(light * quiet
                    + ''.join(dark if cell else light for cell in row)
                    + light * quiet)
    rows += [blank] * quiet
    return '\n'.join(rows)


def to_ascii(text: str, quiet: int = 4) -> str:
    """A QR as plain text, two modules per character cell so it stays square.

    Use this where colour cannot survive - a file, a log, a web page, anything
    that will be pasted somewhere else. For something a camera is going to be
    pointed at, use to_ansi instead: this form is drawn in the terminal's own
    foreground colour, so it is correct on a light theme and inverted on a dark
    one, and an inverted code does not scan.

    The quiet zone is four modules, per the spec. Two was enough for zbar in
    practice and is not enough to rely on.
    """
    m = encode(text)
    width = len(m) + quiet * 2
    rows = [[0] * width for _ in range(quiet)]
    for row in m:
        rows.append([0] * quiet + list(row) + [0] * quiet)
    rows += [[0] * width for _ in range(quiet)]

    # Half-blocks: one character carries two vertical modules.
    out = []
    for i in range(0, len(rows), 2):
        top = rows[i]
        bottom = rows[i + 1] if i + 1 < len(rows) else [0] * width
        line = []
        for t, b in zip(top, bottom):
            if t and b:
                line.append('█')
            elif t:
                line.append('▀')
            elif b:
                line.append('▄')
            else:
                line.append(' ')
        out.append(''.join(line))
    return '\n'.join(out)
