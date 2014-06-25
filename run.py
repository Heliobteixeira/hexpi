#!/usr/bin/env python

import hexmodel
import pdb
import time


def main():
    m = hexmodel.HexModel()
    m.engine.loadtripodgaitpaths(70)
    while True:
    	m.engine.updategait()
    	#time.sleep(0.01)

    pdb.set_trace()


if __name__ == '__main__':
    main()
