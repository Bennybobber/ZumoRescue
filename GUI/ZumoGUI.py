import pygame
from threading import Thread
import sys
import serial
import time
CURHR = "Welcome to the Zumo Search and Rescue GUI!"
pygame.init()
pygame.font.init()

res = (720, 720)
ser=serial.Serial('COM3',9600,timeout=1)
def worker():

   global CURHR
   while True:
     msg = ser.readline()
     if len(msg) > 0:
       CURHR = msg.decode()

t = Thread(target=worker)
t.daemon = True
t.start()
screen = pygame.display.set_mode(res)
red = (255,0,0)
green = (0,128,0)
white = (255,255,255)
black = (0,0,0)
color_dark = (100,100,100)
width = screen.get_width()
height = screen.get_height()

smallfont = pygame.font.SysFont('Corbel',35)
text = smallfont.render('quit', True, white)

key=''
font   = pygame.font.SysFont("consolas", 25, True)

manualMode = True
automaticMode = False
while True:
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            pygame.quit()
        if event.type == pygame.KEYDOWN:
                key=event.key
        if event.type == pygame.KEYUP:
                key=''
        if event.type == pygame.MOUSEBUTTONDOWN:
            if 500 <= mouse[0] <= 500+140 and 600 <= mouse[1] <= 600+40:
                pygame.quit()
            if 60 <=mouse[0] <= 100 + 425 and 600 <= mouse[1] <= 600+40:
                manualMode = not manualMode
                automaticMode = False;
    time.sleep(0.02)
    if key != '':
        ser.write(chr(key).encode())
    if key != 119 and key != 97 and key != 115 and key != 100:
        key = ''


    screen.fill((255, 255, 255))
    mouse = pygame.mouse.get_pos()
    
    pygame.draw.rect(screen,color_dark,[500,600,140,40])
    screen.blit(text ,(540,600))
    
    
    msg_string = CURHR
    msg_text = font.render(msg_string, True, red)
    screen.blit(smallfont.render(msg_string, True, red), (25, 50))
    
    
    if manualMode:
        screen.blit(smallfont.render('Use "wasd" to move the Zumo', True, black) , (10,200))
        screen.blit(smallfont.render('Press "r" to log a room', True, black) , (10,250))
        screen.blit(smallfont.render('Press "l" to search a room', True, black) , (10,300))
        screen.blit(smallfont.render('Press "q" or "e" to turn 90 degrees (left, right)', True, black) , (10,350))
        screen.blit(smallfont.render('Press "h" to sound the horn!', True, black) , (10,400))
        screen.blit(smallfont.render('Press "z" emergency stop!', True, black) , (10,450))
        screen.blit(smallfont.render('Press "1" to switch to auto mode.', True, black) , (10,500))
        pygame.draw.rect(screen,red,[60,600,425,40])
        screen.blit(smallfont.render('Show Automatic Instructions', True, white) , (70,600))
    else:
        screen.blit(smallfont.render('Press "z" emergency stop!', True, black) , (10,200))
        screen.blit(smallfont.render('Press "r" stop zumo then "l" or "r" to intiate', True, black) , (10,250))
        screen.blit(smallfont.render('A search of the room, which is done automatically.', True, black) , (10,300))
        screen.blit(smallfont.render('At the end of the corridor, follow the instructions.', True, black) , (10,375))
        screen.blit(smallfont.render('After the 180 degree turn, press "p" to mark past', True, black) , (10,425))
        screen.blit(smallfont.render('the T junction and activate room search again', True, black) , (10,475))
        pygame.draw.rect(screen,green,[60,600,425,40])
        screen.blit(smallfont.render('Show Manual Instructions', True, white) , (70,600))
    
    # superimposing the text onto our button
    
      
    # updates the frames of the game
    pygame.display.update()
