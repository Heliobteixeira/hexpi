#!/usr/bin/env python
import hexlimb

def main():
	limb=hexlimb.HexLimb(0x60, [8,9,10], [10,70,125], [0,0,1])
	limb.getCurrentPosition()
	import pdb; pdb.set_trace()


if __name__ == '__main__':
	main()

