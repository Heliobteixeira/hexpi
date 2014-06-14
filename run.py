#!/usr/bin/env python

import hexmodel
import pdb
import time


def main():
    m = hexmodel.HexModel()
    m.positionlimbs([1, 2, 3, 4, 5, 6], [140, 0, -39], False)
    time.sleep(4)
    m.movelimbs([1, 2, 3, 4, 5, 6], [0, 0, -40], True)
    while True:
        m.printlimbsposition()
        m.tripodgait([0, 5, 0])

    m.movelimbs([1, 2, 3, 4, 5, 6], [0, 0, -40], True)
    time.sleep(1)
    while True:
        m.movelimbs([1, 2, 3, 4, 5, 6], [-40, 0, 0], True)
        m.movelimbs([1, 2, 3, 4, 5, 6], [80, 0, 0], True)
        m.movelimbs([1, 2, 3, 4, 5, 6], [-40, 0, 0], True)
    #m.tripodGait([20,0,0])
    pdb.set_trace()


if __name__ == '__main__':
    main()
