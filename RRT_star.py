import  pygame, random, math ,numpy ,sys
from pygame.locals import *
Xmax = 640
Ymax = 480
esp = 5
n_of_nodes = 1000
class Node:
    x = 0
    y = 0
    cost = 0
    parent = None
    def __init__(self, xcord, ycord):
        self.x = xcord
        self.y = ycord

def dist(x1 ,y1, x2,y2):
    D = math.sqrt(math.pow(x2-x1,2)+ math.pow(y2-y1,2))
    return D


def steer(qrand, qnear,ndist,node,screen,black):
    qnew = Node(0,0)
    if min(ndist) >= esp:
        qnew.x = qnear.x + int((qrand.x - qnear.x) * esp) / dist(qrand.x, qrand.y, qnear.x, qnear.y)

        qnew.y = qnear.y + int((qrand.y - qnear.y) * esp) / dist(qrand.x, qrand.y, qnear.x, qnear.y)
    else:
        qnew.x = qrand.x
        qnew.y = qrand.y
    qnew.parent = node.index(qnear)
    # print (qnear.x,qnear.y, qnew.x,qnew.y,qrand.x,qrand.y)



    pygame.draw.aaline(screen, black, [qnear.x, qnear.y], [qnew.x, qnew.y])
    pygame.display.update()
    return qnew


def main():
    pygame.init()
    screen = pygame.display.set_mode([640, 480])
    pygame.display.set_caption('RRT_Star')
    Green = 124, 252, 0
    black = 0, 0, 0
    Red = 255,0 ,0
    screen.fill(Green)
    pygame.display.update()

    d =2
    node = []
    qinit = Node(0,0)
    qinit.cost = 0
    qgoal = Node(620,460)
    qinit.parent = None
    node.append(qinit)
    #print (node[0].parent)

    for v in range(1,n_of_nodes):
        qxrand = random.randint(0,640)
        qyrand = random.randint(0,480)
        if [qgoal.x, qgoal.y] in node:
            print("destination reached")
            break

        qrand = Node(qxrand,qyrand)
        #print (qxrand, qyrand)
        ndist = []
        for n in node:

            tmp = dist(n.x, n.y, qrand.x, qrand.y)
            ndist.append([tmp])
        qnear = node[ndist.index(min(ndist))]

        qnew = steer(qrand, qnear,ndist, node,screen,black)
        qnew.cost = dist(qnew.x,qnew.y, qnear.x,qnear.y) + qnear.cost

        Qnearest = []
        r = 15.0*math.pow((numpy.log10(n_of_nodes)/n_of_nodes), 1/d)
        print r
        for n in node:
            if (math.pow((n.x- qnew.x),2)+(math.pow((n.y-qnew.y),2)) - math.pow(r,2)) <= 0:
                Qnearest.append(n)
        qmin = qnear
        Cmin = qnew.cost
        for Q in Qnearest:
            if Q.cost + dist(Q.x,Q.y,qnew.x,qnew.y) < Cmin:
                qmin = Q
                Cmin = Q.cost + dist(Q.x,Q.y,qnew.x,qnew.y)
                pygame.draw.aaline(screen, Green, [qnear.x, qnear.y], [qnew.x, qnew.y])
                pygame.draw.aaline(screen, black, [qmin.x, qmin.y], [qnew.x, qnew.y])
                pygame.display.update()
        node.append(qnew)

        for n in node:
            if n == qmin:
                qnew.parent = node.index(n)
    D = []
    for n in node:
        tmpdist = dist(n.x, n.y, qgoal.x, qgoal.y)
        D.append(tmpdist)

    qfinal = node[D.index(min(D))]
    node.append(qgoal)
    qgoal.parent = node.index(qfinal)
    pygame.draw.line(screen, black, [qgoal.x, qgoal.y], [qfinal.x, qfinal.y])
    pygame.display.update()
    end = qgoal
    sta = qfinal
    print (end.parent)
    while end.parent is not 0:
        sta = int(end.parent)
        pygame.draw.aaline(screen, Red, [end.x, end.y], [node[sta].x, node[sta].y])
        pygame.display.update()
        end = node[sta]

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
