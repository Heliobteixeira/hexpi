import sys
import curses
import hexlimb

def updDisplay(x, y):
    screen.addstr(0,3,str(x)+'   ')
    screen.addstr(1,3,str(y)+'   ')

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

inc=5
try:
    while True:
        char = screen.getch()
        if char == ord('q'): 
            break
        elif char == curses.KEY_RIGHT:
            # print doesn't work with curses, use addstr instead
            pata.moveTipTo(pata.x+inc, pata.y)
            updDisplay(pata.x, pata.y)
        elif char == curses.KEY_LEFT:
            pata.moveTipTo(pata.x-inc, pata.y)
            updDisplay(pata.x, pata.y)
        elif char == curses.KEY_UP:
            pata.moveTipTo(pata.x, pata.y+inc)
            updDisplay(pata.x, pata.y)
        elif char == curses.KEY_DOWN:
            pata.moveTipTo(pata.x, pata.y-inc)
            updDisplay(pata.x, pata.y)
finally:
    # shut down cleanly
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
