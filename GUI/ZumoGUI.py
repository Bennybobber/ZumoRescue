import pygame
from threading import Thread
import sys
import serial
import time
CURHR = "Welcome to the Zumo Search and Rescue GUI!"
pygame.init()
pygame.font.init()

res = (720, 720)
ser=serial.Serial('COM6',9600,timeout=1)
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
    time.sleep(0.02)
    if key != '':
        print(key)
        ser.write(chr(key).encode())
    if key != 119 and key != 97 and key != 115 and key != 100:
        key = ''


    screen.fill((255, 255, 255))
    mouse = pygame.mouse.get_pos()
    
    pygame.draw.rect(screen,color_dark,[500,600,140,40])
    screen.blit(text ,(540,600))
    
    
    msg_string = CURHR
    msg_text = font.render(msg_string, True, red)
    screen.blit(smallfont.render(msg_string, True, red), (25, 350))
    
    
    
    
    # superimposing the text onto our button
    
      
    # updates the frames of the game
    pygame.display.update()
