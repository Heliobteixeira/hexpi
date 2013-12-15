import sys
import curses
import hexlimb

def updDisplay(x, y, xf, yf):
    screen.addstr(0,3,str(x)+'  ')
    screen.addstr(1,3,str(y)+'  ')
    screen.addstr(0,30,'('+str(xf-x)+')  ')
    screen.addstr(1,30,'('+str(yf-y)+')  ')    


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

pata=hexlimb.HexLimb(0x40, 70, 120, True, True)
updDisplay(pata.x, pata.y, 0, 0)

inc=10
try:
    while True:
        char = screen.getch()
        if char == ord('q'): 
            break
        elif char == curses.KEY_RIGHT:
            # print doesn't work with curses, use addstr instead
            xf=pata.x+inc
            yf=pata.y
            pata.moveTipTo(xf, yf)
            updDisplay(pata.x, pata.y, xf, yf)
        elif char == curses.KEY_LEFT:
            xf=pata.x-inc
            yf=pata.y
            pata.moveTipTo(xf, yf)
            updDisplay(pata.x, pata.y, xf, yf)
        elif char == curses.KEY_UP:
            xf=pata.x
            yf=pata.y+inc
            pata.moveTipTo(xf, yf)
            updDisplay(pata.x, pata.y, xf, yf)
        elif char == curses.KEY_DOWN:
            xf=pata.x
            yf=pata.y-inc
            pata.moveTipTo(pata.x, pata.y-inc)
            updDisplay(pata.x, pata.y, xf, yf)
finally:
    # shut down cleanly
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
