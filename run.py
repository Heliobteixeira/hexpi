#!/usr/bin/env python

import hexmodel
import pdb

def main():
	m = hexmodel.HexModel()
	m.linearMoveLimbs([1,2,3,4,5,6],[120,0,-130])
	#m.moveLimbsTipTo([1,2,3,4,5,6],[120,0,-130])
	#pdb.set_trace()


if __name__ == '__main__':
	main()

