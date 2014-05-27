import sys
import curses
import hexlimb
from time import sleep

def updDisplay(obj, xf, yf):
    x=str(obj.x)
    y=str(obj.y)
    delx=str(xf-obj.x)
    dely=str(yf-obj.y)
    screen.addstr(0,3,x+'  ')
    screen.addstr(1,3,y+'  ')
    screen.addstr(0,30,'('+delx+')  ')
    screen.addstr(1,30,'('+dely+')  ')
    screen.addstr(2,0,'Femur:'+str(obj.femur.angle)+'deg  ')
    screen.addstr(3,0,'Tibia:'+str(obj.tibia.angle)+'deg  ')

# get the curses screen window
screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)

screen.addstr(0,0, 'X:')
screen.addstr(1,0, 'Y:')

pata=hexlimb.HexLimb(0x40, 70, 125, True, True, [140, 0, -39])
updDisplay(pata, 0, 0)

inc=10
try:
    while True:
        char = screen.getch()
        if char == ord('q'): 
            break
        elif char == ord('s'):
            i=0
            while i<10:               
                xf=120
                yf=0                
                pata.moveTipTo(xf, yf)
                updDisplay(pata, xf, yf)
                sleep(1)
                xf=190
                yf=0
                pata.moveTipTo(xf, yf)
                updDisplay(pata, xf, yf)
                sleep(1)              
                i+=1
        elif char == curses.KEY_RIGHT:
            # print doesn't work with curses, use addstr instead
            xf=pata.x+inc
            yf=pata.y
            pata.moveTipTo(xf, yf)
            updDisplay(pata, xf, yf)
        elif char == curses.KEY_LEFT:
            xf=pata.x-inc
            yf=pata.y
            pata.moveTipTo(xf, yf)
            updDisplay(pata, xf, yf)
        elif char == curses.KEY_UP:
            xf=pata.x
            yf=pata.y+inc
            pata.moveTipTo(xf, yf)
            updDisplay(pata, xf, yf)
        elif char == curses.KEY_DOWN:
            xf=pata.x
            yf=pata.y-inc
            pata.moveTipTo(pata.x, pata.y-inc)
            updDisplay(pata, xf, yf)
finally:
    # shut down cleanly
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
