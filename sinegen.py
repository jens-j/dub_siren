#!/usr/bin/env python3

from numpy import sin, pi

s = ''

for i in range(1024):
    if i > 0:
        s += ','
        if i % 10 == 0:
            s += '\n'

    x = sin(i / 1024 * pi / 2)
    x = int(x * 2**15) % 2**16
    s += f'0x{x:04x}'

print(s)
