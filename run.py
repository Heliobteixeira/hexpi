#!/usr/bin/env python

import hexmodel
import pdb
import time


def main():
    m = hexmodel.HexModel()
    m.engine.loadwavegaitpaths((80, 80, 0))
    #m.engine.positionmembersforgaitstart()
    #pdb.set_trace()
    m.engine.startgait()


    pdb.set_trace()


if __name__ == '__main__':
    main()
