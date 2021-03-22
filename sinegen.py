#!/usr/bin/env python3

from numpy import sin, pi, zeros

x = zeros(1024)

s = ''

for i in range(1024):

    x[i] = sin(i / 1024 * pi / 2)
    # x[i] = sin(i / 1024 * pi / 2) + 0.5 * sin(3 * i / 1024 * pi / 2)

x = x / max(x)

for i, x in enumerate(x):

    if i > 0:
        s += ','
        if i % 10 == 0:
            s += '\n'

    y = int(x * (2**15 - 1)) % 2**16
    s += f'0x{y:04x}'

print(s)
