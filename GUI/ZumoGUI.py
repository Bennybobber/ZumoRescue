import pygame
import sys
import serial
import time

pygame.init()

res = (720, 720)
ser=serial.Serial('COM6',9600,timeout=1)
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
        
manualMode = False
automaticMode = False
key=''
    
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
            if 100 <=mouse[0] <= 100 + 250 and 600 <= mouse[1] <= 600+40:
                manualMode = not manualMode
                automaticMode = False;
            if 100 <=mouse[0] <= 100 + 250 and 400 <= mouse[1] <= 400+40:
                automaticMode = not automaticMode
                manualMode = False;

    if manualMode:
        time.sleep(0.01)
        if key == 97:
            ser.write('a'.encode())
        elif key == 115:
            ser.write('s'.encode())
        elif key == 119:
            ser.write('w'.encode())
        elif key == 100:
            ser.write('d'.encode())  
    if automaticMode:
        if key == 32:
            ser.write(' '.encode())
        time.sleep(0.01)
    screen.fill((255, 255, 255))
    mouse = pygame.mouse.get_pos()
    
    pygame.draw.rect(screen,color_dark,[500,600,140,40])
    screen.blit(text ,(540,600))
    
    if manualMode:
        screen.blit(smallfont.render('Use WASD to move', True, black) , (110,200))
        pygame.draw.rect(screen,red,[100,600,250,40])
        screen.blit(smallfont.render('Turn manual off', True, white) , (110,600))
    else:
        pygame.draw.rect(screen,green,[100,600,250,40])
        screen.blit(smallfont.render('Turn manual on', True, white) , (110,600))
    if automaticMode:
        pygame.draw.rect(screen,red,[100,400,250,40])
        screen.blit(smallfont.render('Turn auto off', True, white) , (110,400))
    else:
        pygame.draw.rect(screen,green,[100,400,250,40])
        screen.blit(smallfont.render('Turn auto on', True, white) , (110,400))
        
    
    
    
    
    # superimposing the text onto our button
    
      
    # updates the frames of the game
    pygame.display.update()
