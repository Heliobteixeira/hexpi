#!/usr/bin/env python

import hexmodel
import pdb
import time
import atexit

def main():
	try:
	    m = hexmodel.HexModel()
	    pdb.set_trace()
	    m.engine.loadwavegaitpaths((10, -100, 0))
	    m.engine.startgait()
	finally:
		m.engine.poweroff()

if __name__ == '__main__':
    main()
