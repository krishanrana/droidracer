import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
        
surfaces = (
    (1, 2, 3, 4),
    (3, 2, 7, 6),
    (6, 7, 5, 4),
    (4, 5, 1, 0),
    (1, 5, 7, 2),
    (4, 0, 3, 6)
)

vertices = (
    ( 1, -1, -1),
    ( 1,  1, -1),
    (-1,  1, -1),
    (-1, -1, -1),
    ( 1, -1,  1),
    ( 1,  1,  1),
    (-1, -1,  1),
    (-1,  1,  1)
)

edges = (
    (0, 1),
    (0, 3),
    (0, 4),
    (2, 1),
    (2, 3),
    (2, 7),
    (6, 3),
    (6, 4),
    (6, 7),
    (5, 1),
    (5, 4),
    (5, 7)
)

# -------------------------------------------------------------
btn_square = 0
btn_cross = 1
btn_circle = 2
btn_tri = 3

btn_l1 = 4
btn_l2 = 6
btn_l3 = 10

btn_r1 = 5
btn_r2 = 7
btn_r3 = 11

btn_share = 8
btn_option = 9

btn_ps4 = 12
btn_touch = 13

axis_x = 9
axis_y = 11
#axis_z = ??? #don't think there's any more accelerometers

axis_l3_x = 0
axis_l3_y = 1
axis_r3_x = 2
axis_r3_y = 5

axis_l2 = 3
axis_r2 = 4

axis_touch_x = 13
axis_touch_y = 14
# -------------------------------------------------------------

def mixColors(colors):
    reds =   [ c[0] for c in colors ]
    greens = [ c[1] for c in colors ]
    blues =  [ c[2] for c in colors ]
       
    r = sum(reds) / len(colors)
    g = sum(greens) / len(colors)
    b = sum(blues) / len(colors)

    return (r, g, b)

def getColorFromButtonsPressed(buttons):
    colors = {
        btn_square: (1.00, 0.41, 0.97),
        btn_cross:  (0.49, 0.69, 0.97),
        btn_circle: (1.00, 0.40, 0.40),
        btn_tri:    (0.25, 0.88, 0.62)
        }

    pressedButtons = [ b for b in buttons.keys() if buttons[b] and b in colors.keys() ]

    if not pressedButtons:
        return (0, 0, 0)

    return mixColors([ colors[b] for b in pressedButtons ])

def shiftCube(axis_x, axis_y):
    translate_x = 2.0 * axis_x
    translate_y = 2.0 * axis_y  
    
    glTranslatef(axis_x, axis_y, 0.0)    
    

def Cube(color):
    glBegin(GL_QUADS)
    for surface in surfaces:
        for vertex in surface:           
            glColor3fv(color)
            glVertex3fv(vertices[vertex])
    glEnd()

    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glColor3fv((1,1,1))
            glVertex3fv(vertices[vertex])
    glEnd()


def getZrotation(l2_axis, l2_pressed, r2_axis, r2_pressed):
    # if buttons not pressed their axis is 0.0, but
    # -1 makes more sense for us here
    if not l2_pressed:
        l2_axis = -1.0
    if not r2_pressed:
        r2_axis = -1.0

    # next scale it so we're working with values [0..2] instead of [-1..1]
    l2_axis += 1.0
    r2_axis += 1.0

    # finally subtract axes, and scale back to [0..1]
    return (l2_axis - r2_axis) / 2

def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
   
    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0,0.0, -5)

    pygame.init()
    pygame.joystick.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()
    
    axis = {}

    button = {}
    for i in range(controller.get_numbuttons()):
        button[i] = False

    # set some default value
    rot_x = 0.0
    rot_y = 0.0
    rot_z = 0.0
    rot_z_l2 = 0.0
    rot_z_r2 = 0.0

    trans_x = 0
    trans_y = 0

    # fix scaling - the rotz
    rot_scale = 60.0
    rotx_scale =  1 * rot_scale
    roty_scale = -1 * rot_scale
    rotz_scale = -1 * 180
   
    while True:
        # load previous values, in case there's no joy events this iteration
        axis[axis_x] = rot_x
        axis[axis_y] = rot_y
        axis[axis_l2] = rot_z_l2
        axis[axis_r2] = rot_z_r2

        axis[axis_touch_x] = trans_x
        axis[axis_touch_y] = trans_y
        
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                axis[event.axis] = round(event.value,2)
            elif event.type == pygame.JOYBUTTONDOWN:
                button[event.button] = True
            elif event.type == pygame.JOYBUTTONUP:
                button[event.button] = False
       
        rot_x = axis[axis_x]
        rot_y = axis[axis_y]
        rot_z_l2 = axis[axis_l2]
        rot_z_r2 = axis[axis_r2]
        rot_z = getZrotation(rot_z_l2, button[btn_l2], rot_z_r2, button[btn_r2])
        trans_x = axis[axis_touch_x]
        trans_y = axis[axis_touch_y]

        glPushMatrix()   
        shiftCube(trans_x, -1 * trans_y)

        glRotatef(roty_scale * rot_y, 1, 0, 0)
        glRotatef(rotz_scale * rot_z, 0, 1, 0)
        glRotatef(rotx_scale * rot_x, 0, 0, 1)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        Cube(getColorFromButtonsPressed(button))
        glPopMatrix()        
        pygame.display.flip()

main()