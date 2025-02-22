# for checksum
def inv2(x):
    return (((x & 0xFF) ^ 0xFF) + 1) & 0xFF

