#!/usr/bin/env python3

import numpy as np

RC_TIME = 3

x = np.zeros(2048)
s = ''

for i in range(2048):
    x[i] = 1.0 + np.exp(-RC_TIME) - 2 * np.exp(-i / 2048.0 * RC_TIME)

x = x / max(np.abs(x))

for i, x in enumerate(x):

    if i > 0:
        s += ','
        if i % 10 == 0:
            s += '\n'

    y = int(abs(x) * (2**15 - 1)) % 2**16

    if x < 0:
        y = (y ^ 0xFFFF) + 1;

    s += f'0x{y:04x}'

print(s)
