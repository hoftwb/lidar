import numpy as np
import math
import random
import sys
from pynput import keyboard
import time

inputQueue = ""

def main():
    frame = 0
    
    state = [[""]]
    state = initDimensions(50, 10, state)
    
    spikes = []
    spikes = initSpikes(spikes, state)

    dinoY = 0
    jumpStart = -1

    listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
    listener.start()

    for i in range(1000):
        #key = input() #read keyboard input and wait
        time.sleep(1/12)
        key = inputQueue

        frame += 1

        clearScreen(state)

        del spikes[0] #pop first element - it's passed offscreen
        randomSpike(spikes)

        for s in range(len(spikes)):
            if spikes[s] > 0:
                drawSpike(s, 0, spikes[s], state)

        if key != "" and dinoY == 0:
            jumpStart = frame

        dinoY = -(((frame-jumpStart - 1)/2) ** 2) + 2

        if dinoY < 0:
            dinoY = 0

        drawDino(0, round(dinoY), state)
        
        render(state)

def initDimensions(x, y, state):
    state = []
    for _y in range(y):
        state.append([])
        for _x in range(x):
            (state[_y]).append("")

    #print(state)
    return state
    
def initSpikes(spikes, state):
    for s in range(len(state[0])):
        if s > 10:
            randomSpike(spikes)
        else:
            spikes.append(0)
            
    #print(spikes)
    return spikes

def randomSpike(spikes):
    if 1 in spikes[len(spikes) - 5 : len(spikes)] or 2 in spikes[len(spikes) - 5 : len(spikes)] or 3 in spikes[len(spikes) - 5 : len(spikes)]:
        spikes.append(0)
        return
    
    
    spikeFloat = random.random() #should we add a spike?

    if spikeFloat > 0.9:
        spikes.append(1)
    elif spikeFloat > 0.8:
        spikes.append(2)
    elif spikeFloat > 0.7:
        spikes.append(3)
    else:
        spikes.append(0)

def clearScreen(state):
    for y in state:
        for x in range(len(y)):
            y[x] = "."

def render(state):
    output = ""
    
    state.reverse()
    for y in state:
        line = "".join(y)
        if output == "":
            output = line
        else:
            output = output + "\n" + line
    
    print(output)

def drawDino(x, y, state):
    global inputQueue
    inputQueue = ""

    if (state[y])[x] == ".":
        (state[y])[x] = "R"
    else:
        (state[y])[x] = "X"
        render(state)
        sys.exit("u died")

def drawSpike(x, y, type, state):
    if type == 1:
        (state[y])[x - 1] = "/"
        (state[y])[x] = "\\"
    elif type == 2:
        (state[y])[x] = "|"
        (state[y + 1])[x] = "|"
    elif type == 3:
        (state[y])[x - 2] = "/"
        (state[y])[x - 1] = "-"
        (state[y])[x] = "\\"

def on_press(key, injected):
    try:
        #print('alphanumeric key {} pressed; it was {}'.format(
        #    key.char, 'faked' if injected else 'not faked'))
        global inputQueue
        inputQueue = key.char
    except AttributeError:
        #print('special key {} pressed'.format(
            #key))
        a = 1

def on_release(key, injected):
    #print('{} released; it was {}'.format(
    #    key, 'faked' if injected else 'not faked'))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

if __name__ == "__main__":
    main()