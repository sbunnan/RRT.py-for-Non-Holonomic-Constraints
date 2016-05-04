import  pygame, random, math ,numpy
from numpy import *
from pygame.locals import *
import matplotlib.path as mplPath
Xmax = 820                     # Window size
Ymax = 660
L = 5                           #length of car
U_s = 12                        #linear velocity
h =1                            #runge-kutta constant
numnodes = 3000
obstacle_1 = [[200, 300], [120,420], [346,333]]
obstacle_2 = [[500,200], [700,200], [750,400], [450,550]]
bbPath1 = mplPath.Path(array(obstacle_2))
bbPath2 = mplPath.Path(array(obstacle_1))

class Node:
    x = 0
    y = 0
    theta = 0
    parent = None
    list = []
    def __init__(self, xcord, ycord, theta):
        self.x = xcord
        self.y = ycord
        self.theta = theta

def dist(x1 ,y1, x2,y2):          #distance between two point
    D = math.sqrt(math.pow(x2-x1,2)+ math.pow(y2-y1,2))
    return D

def Range_kutta(qnear,phsi):                        # Runge Kutta integration method
    thet = qnear.theta
    qnew = Node(0,0,0)
    x = qnear.x
    y = qnear.y
    for i in range(1, 3, 1):
        a = numpy.array([cos(thet), sin(thet), (numpy.tan(phsi)/L)])
        k1 = h * U_s * a
        b = numpy.array([cos(thet+(h/2)),sin(thet+(h/2)),(numpy.tan(phsi+(k1[2]/2))/L)])
        k2 = h * U_s * b
        c = numpy.array([cos(thet+(h/2)),sin(thet+(h/2)),(numpy.tan(phsi+(k2[2]/2)/L))])
        k3 = h * U_s * c
        d = numpy.array([cos(thet + h), sin(thet + h), (numpy.tan(phsi + k3[2]) / L)])
        k4 = h * U_s * d
        thet = thet + k1[2] / 6 + k2[2] / 3 + k3[2] / 3 + k4[2] / 6
        x1 = x + k1[0] / 6 + k2[0] / 3 + k3[0] / 3 + k4[0]/ 6
        y1 = y + k1[1] / 6 + k2[1] / 3 + k3[0] / 3 + k4[0]/ 6
        t1 = bbPath1.contains_point((x,y))
        t2 = bbPath2.contains_point((x,y))
        if t1 ==1 :
            return qnear
        if t2 == 1:
            return qnear
        x = x1
        y = y1
    if (x1 >=0) & (y1 >=0) & (x <= Xmax) & (y <= Ymax):
        qnew.x = x
        qnew.y = y
        qnew.theta = thet
    elif (x1 < 0 )|( y1 < 0) | (x > Xmax )| (y > Ymax):
        qnew.x = Inf
        qnew.y = Inf
        qnew.theta = qnear.theta


    return qnew


def inte_line(qnear):                              # intergrate over line where turning angle is zero
    phsi = 0
    n=  Range_kutta(qnear,phsi)
    return n

def inte_left(qnear):                               #integrate over line where turn angle is 45 degree(left)
    phsi = math.pi/4
    n1 = Range_kutta(qnear, phsi)
    return n1

def inte_right(qnear):                              #integrate over line where turn angle is 45 degree(right)
    phsi = -(math.pi/4)
    n2 = Range_kutta(qnear, phsi)
    return n2

def min_dist (q1,q2,q3,qrand):
    d1 = dist(q1.x,q1.y,qrand.x,qrand.y)            #shortest distance of above three node from random point
    d2 = dist(q2.x,q2.y,qrand.x,qrand.y)
    d3 = dist(q3.x,q3.y,qrand.x,qrand.y)
    d_list = [d1,d2,d3]
    D = min(d_list)
    if D == d1:
        return q1,1
    elif D == d2:
        return q2,2
    elif D == d3:
        return q3,3

def runge_kutta_draw(qnear, phsi,screen,black):  # integration method to draw a line over selected path through pygame
    thet = qnear.theta
    x = qnear.x
    y = qnear.y
    for i in range(1, 5, 1):
        a = numpy.array([cos(thet), sin(thet), (numpy.tan(phsi) / L)])
        k1 = h * U_s * a
        b = numpy.array([cos(thet+(h/2)),sin(thet+(h/2)),(numpy.tan(phsi+(k1[2]/2))/L)])
        k2 = h * U_s * b
        c = numpy.array([cos(thet+(h/2)),sin(thet+(h/2)),(numpy.tan(phsi+(k2[2]/2)/L))])
        k3 = h * U_s * c
        d = numpy.array([cos(thet + h), sin(thet + h), (numpy.tan(phsi + k3[2]) / L)])
        k4 = h * U_s * d
        thet = thet + k1[2] / 6 + k2[2] / 3 + k3[2] / 3 + k4[2] / 6
        x2 = x + k1[0] / 6 + k2[0] / 3 + k3[0] / 3 + k4[0] / 6
        y2 = y + k1[1] / 6 + k2[1] / 3 + k3[0] / 3 + k4[0] / 6
        t1 = bbPath1.contains_point((x, y))
        t2 = bbPath2.contains_point((x, y))
        if t1 == 1:
            return
        if t2 == 1:
            return
        pygame.draw.aaline(screen, black, [x2, y2], [x, y], 1)
        pygame.display.update()
        x = x2
        y = y2
        qnear.list.append([x, y])

def attach_final_node(node, qgoal,screen,black):
    D = []
    for n in node:
        tmpdist = dist(n.x, n.y, qgoal.x, qgoal.y)
        D.append(tmpdist)
    q_pre_final = node[D.index(min(D))]
    q1 = inte_line(q_pre_final)
    q2 = inte_left(q_pre_final)
    q3 = inte_right(q_pre_final)
    d = min([q1,q2,q3])
    if d == 1:
        runge_kutta_draw(q_pre_final, 0, screen, black)
    elif d == 2:
        runge_kutta_draw(q_pre_final, math.pi / 4, screen, black)
    elif d == 3:
        runge_kutta_draw(q_pre_final, -math.pi / 4, screen, black)
    qgoal.parent = node.index(q_pre_final)
    node.append(qgoal)


def draw_path(node, screen,Blue):
    for n in node:
        pygame.draw.aalines(screen, Blue,False,n.list)
    pygame.display.update()




def main():
    pygame.init()                               # pygame initialization
    screen = pygame.display.set_mode([Xmax, Ymax])
    pygame.display.set_caption('RRT')
    Green = 124, 252, 0
    black = 0, 0, 0
    Red = 255,0 ,0
    Blue = 0,0, 128
    screen.fill(Green)
    bbPath1 = mplPath.Path(array(obstacle_2))
    bbPath2 = mplPath.Path(array(obstacle_1))
    pygame.draw.polygon(screen,Red,obstacle_1,0)
    pygame.draw.polygon(screen, Red, obstacle_2, 0)
    pygame.display.update()

    node = []
    qinit = Node(1,1,math.pi/4)
    qgoal = Node(1010,760,math.pi/4)
    qinit.parent = None
    node.append(qinit)
    for v in range(1, numnodes):
        qxrand = random.randint(0, Xmax)
        qyrand = random.randint(0, Ymax)
        if [qgoal.x, qgoal.y] in node:
            print("destination reached")
            break
        qrand = Node(qxrand, qyrand,0)
        ndist = []
        for n in node:
            tmp = dist(n.x, n.y, qrand.x, qrand.y)
            ndist.append([tmp])
        qnear = node[ndist.index(min(ndist))]
        draw_node = Node(0,0,0)
        draw_node.x = qnear.x
        draw_node.y = qnear.y
        draw_node.theta = qnear.theta
        q1 = inte_line(qnear)
        q2 = inte_left(qnear)
        q3 = inte_right(qnear)
        [qnew,d] = min_dist(q1,q2,q3,qrand)
        if d ==1:
            runge_kutta_draw(draw_node,0,screen,black)
        elif d ==2:
            runge_kutta_draw(draw_node,math.pi/4, screen, black)
        elif d == 3:
            runge_kutta_draw(draw_node,-math.pi/4, screen, black)
        qnew.parent = node.index(qnear)
        node.append(qnew)
        attach_final_node(node, qgoal,screen,black)



        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")


if __name__ == '__main__':
    main()
    running = True
    while running:
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
