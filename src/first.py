#!/usr/bin/env python
import rospy as rp
from map import GridMap
import numpy as np
from math import *

np.random.seed(444)
#for i in range()

class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.9
        self.points = []
        self.v = 0.8
        self.L = 0.1
        self.st = (self.start[0], self.start[1], 180)
        self.en = (self.end[0], self.end[1], 0)

    def restorePath(self):
        start = False
        path = []
        akt = self.en
        path.append(akt)
        while not start:
            p = akt
            akt = self.parent[p]
            path.append(akt)
            if akt == self.st:
                start = True
        return path

    def restoreDist(self, pt):
        min = 0
        pier = True
        best = None
        for point in self.points:
            dist = sqrt(pow(point[0] - pt[0], 2) + pow(point[1] - pt[1], 2))
            start = False
            akt = point
            if akt == self.st:
                start = True
            while not start:
                p = akt
                akt = self.parent[p]
                d = sqrt(pow(p[0] - akt[0], 2) + pow(p[1] - akt[1], 2))
                dist = dist + d
                if akt == self.st:
                    start = True
            if pier == True:
                min = dist
                best = point
                pier = False
            else:
                if dist < min:
                    min = dist
                    best = point
        return best

    def check_if_valid(self, a, b):
        in_free_space = True
        for i in np.linspace(b, a, num=2, axis=0):
            x = int(i[0] * 10)
            y = int(i[1] * 10)
            for i in range(-1, 1):
                for j in range(-1, 1):
                    if self.map[[y+i], [x+j]] > 50 or self.map[[y+i], [x+j]] < 0:
                        in_free_space = False
        return in_free_space

    def random_point(self):
        x = round(self.width * np.random.random(), 3)
        y = round(self.height * np.random.random(), 3)
        return x, y, 0

    def find_closest(self, pos):
        p = False
        closest = []
        for point in self.parent:
            dist = sqrt(pow(pos[0] - point[0], 2) + pow(pos[1] - point[1], 2))
            if p == False:
                closest = point
                min = dist
                p = True
            else:
                if dist < min:
                    closest = point
                    min = dist
        return closest

    def find_path(self, point, best):
        pth = False
        po = {}
        found = False
        for p in range(70):
            points = []
            kier = float(np.random.randint(low=-40000, high=40000))
            kier = kier/1000
            t = float(np.random.randint(low=100, high=700))
            dt = t/1000
            prev = best
            for i in range(10):
                x = round(prev[0] + self.v*cos(prev[2]) * dt, 3)
                y = round(prev[1] + self.v*sin(prev[2]) * dt, 3)
                o = round(prev[2] + self.v/self.L * tan(kier), 3)
                if self.check_if_valid(prev, (x, y, o)):
                    po[(x, y, o)] = prev
                    prev = (x, y, o)
                    points.append(prev)
                    if sqrt(pow(x - point[0], 2) + pow(y - point[1], 2)) < 0.09:
                        print("Found")
                        point = (point[0], point[1], o)
                        self.parent[point] = (x, y, o)
                        while not found:
                            #print(prev)
                            par = po[prev]
                            self.parent[prev] = par
                            prev = par
                            if prev == best:
                                found = True
                        for p in points:
                            self.points.append(p)
                        pth = True
                        return True
                else:
                    break
        return False

    def new_pt(self, pt, closest):
        dist = sqrt(pow(pt[0]- closest[0], 2) + pow(pt[1] - closest[1], 2))
        x = abs(pt[0] - closest[0])
        y = abs(pt[1] - closest[1])
        prx = x/dist
        pry = y/dist
        dx = self.step * prx
        dy = self.step * pry
        new_x = closest[0]
        new_y = closest[1]
        if pt[0] > closest[0]:
            new_x += dx
        else:
            new_x -= dx
        if pt[1] > closest[1]:
            new_y += dy
        else:
            new_y -= dy
        new_x = round(new_x, 3)
        new_y = round(new_y, 3)
        return new_x, new_y, 0

    def search(self):
        self.parent[self.st] = None
        self.points.append(self.st)

        while not rp.is_shutdown():
            rig = False
            new = (0, 0, 0)
            while rig != True:
                point = self.random_point()
                cl = self.find_closest(point)
                new = self.new_pt(point, cl)
                if self.map[[int(new[1]*10)], [int(new[0]*10)]] < 50 and self.map[[int(new[1]*10)], [int(new[0]*10)]] > -1:
                    rig = True
                    #print(new)


            best = self.restoreDist(new)
            if(self.find_path(new, best)):
                self.publish_search()
            else:
                continue

            # if self.check_if_valid(new, self.en):
            #     self.parent[self.en] = new
            #     print("Goal!")
            #     path = self.restorePath()
            #     self.publish_path(path)
            #     self.publish_search()
            #     break
            rp.sleep(0.01)


if __name__ == '__main__':
    rrt = RRT()
    rrt.search()