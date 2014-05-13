#!/usr/bin/env python

import hexmodel
import pdb

def main():
	m = hexmodel.HexModel()
	m.limbs[4].printPosition()
	m.moveLimbsTipTo([4],[180,0,-130])
##	pdb.set_trace()


if __name__ == '__main__':
	main()

