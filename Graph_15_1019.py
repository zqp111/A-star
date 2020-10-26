# -*- encoding: utf-8 -*-
'''
@File    :   Graph_15_1019.py
@Time    :   2020/10/19 20:19:14
@Author  :   zqp 
@Version :   1.0
@Contact :   zhangqipeng@buaa.edu.cn
'''


import matplotlib.pyplot as plt
import numpy as np
import time

GOAL = np.array([i+1 if i < 15 else 0 for i in range(16)])
GOAL_index = GOAL.reshape((4, 4)) #确立基准表，获得距离


def get_relu():
    A = np.array([i for i in range(16)]).reshape(4, 4)
    relu = dict()
    for i in range(4):
        for j in range(4):
            relu[A[i, j]] = []
            if i !=0:
                relu[A[i, j]].append(A[i-1, j])
            if i != 3:
                relu[A[i, j]].append(A[i+1,j])
            if j !=0:
                relu[A[i,j]].append(A[i, j-1])
            if j != 3:
                relu[A[i,j]].append(A[i, j+1])
    return relu

            
def getdistance(m, n):
    x = m // 4
    y = m % 4
    n = np.where(GOAL_index == n)
    distance = abs(x -n[0][0])+abs(y -n[1][0])
    return distance

class node(): #节点
    def __init__(self, state, father = None):
        self.state = np.array(state)
        self.father = father
        self.f = 0 #估价函数
        self.g = 0 #所经过的实际代价
        self.h = 0 #当前位置到目标的代价估计值

class Search(object):
    def __init__(self, S0, relu, Isprint = True, htype = 1):
        self.Isprint = Isprint      #是否打印搜索过程
        self.htype = htype          
        self.relu = relu
        self.node = node(np.array(S0)) #初始状态
        self.node.father = None     #根节点无父节点
        self.node.h, self.node.g = self.getf(self.node)
        self.node.f = self.node.h+self.node.g #计算估价函数
        self.Open = []
        self.Close = {}
        self.Open.append(self.node)


    def IsEnd(self): #判断是否结束
        if (self.node.state == GOAL).all():
            return True
        return False

    def Exchange(self, Array, i, j):
        m = Array.copy()
        temp = m[i]
        m[i] = m[j]
        m[j] = temp

        return m

    def NextState(self): #状态转换规则
        Nextlist = []

        index = np.where(self.node.state == 0)[0][0]
        for m in self.relu[index]:
            NextState = self.Exchange(self.node.state, index, m)
            Next = node(NextState, self.node)
            Next.h, Next.g = self.getf(Next)
            Next.f = Next.h + Next.g
            Nextlist.append(Next)

        return Nextlist

    def geth(self, node): #计算当前到目标代价估计值
        IsSame = node.state == GOAL
        h = 0
        for i, x in enumerate(IsSame):
            if not x:
                if node.state[i] == 0:
                    continue
                h += getdistance(i, node.state[i])
        # h += self.__get_reverse_number(node)
        return h
        # h = np.sum(IsSame == False)
        # return h


    def getf(self, node):
        h = self.geth(node)
        g = 0
        n = node
        while n.father:
            g += 1
            n = n.father
        #print((h, g))
        return h,g

    def __get_reverse_number(self, node):
        graph = node.state.copy().reshape((4, 4))
        # print(node.state.shape, node.state)
        reverse_number = 0
        for i in range(graph.shape[0]):
            for j in range(graph.shape[1]):
                if graph[i, j] == 0:
                    graph[i, j] = 16
                if i != 0:
                    if graph[i, j] < graph[i-1, j]:
                        reverse_number += 1
                if j != 0:
                    if graph[i, j] < graph[i, j-1]:
                        reverse_number += 1
                if i != 3:
                    if graph[i, j] > graph[i+1, j]:
                        reverse_number += 1
                if j != 3:
                    if graph[i, j] > graph[i, j+1]:
                        reverse_number += 1
        return reverse_number>>2


    def IsInOpen(self, node):   #判断节点是否在OPEN表里
        for n in range(len(self.Open)):
            if (node.state == self.Open[n].state).all():
                return True, n
        return False, -1

    def IsInClose(self, node):  #判断节点是否在CLOSE表里
        if self.Close.has_key(node.state):
            if node.f < self.Close.get(node.state).f:
                return True, self.Close.pop(node.state)
        return False, -1

    def SortOpen(self):
        self.Open.sort(key=lambda node: (node.f, node.h))  # 按f排序，如果相等，按h排序，保证深度优先
                                                           # TODO 优化表较大时， 采用二分法插入


        flag = False                                    #保证存在目标时立即将其返回
        for n in range(len(self.Open)):                 # TODO 优化掉， 每次获取子节点时检查是否为目标
            if (self.Open[n].state == GOAL).all():
                flag = True
                break
        if flag:
            self.Open.insert(0, self.Open.pop(n))
        if self.Isprint:
            print('open:')
            for i in self.Open:
                print(i.state,end='   ')
            print('\n')
            print('close:')
            for i in self.Close:
                print(i.state, end='   ')
            print('\n')

    def getans(self):
        count = 0
        T = np.zeros(4)
        while self.Open: #OPEN表不空
            Time = time.time()
            count += 1 #计数搜索轮数
            self.node = self.Open.pop(0)  # 取OPEN表最小f值的节点
            #print(self.node.state)
            self.Close[self.node.state] = self.node  # 放到CLOSE表
            if self.IsEnd():
                if self.Isprint:
                    print('搜索了%d轮'%(count-1))
                return self.node, count
            #print(self.node.state)
            T[0] = time.time() - Time
            Time = time.time()
            sons = self.NextState()  #生成子节点
            for son in sons:
                flag, old = self.IsInOpen(son)      #检查子节点是否在OPEN表
                if flag:
                    if son.f < self.Open[old].f:    #如果在OPEN表中且其f较小，则进行替换操作
                        self.Open[old].h = son.h
                        self.Open[old].g = son.g
                        self.Open[old].f = son.f
                        self.Open[old].father = son.father
                    continue
                flag, old = self.IsInClose(son)     #检查子节点是否在CLOSE表
                if flag:
                    self.Open.append(son)
                    continue
                self.Open.append(son)
            T[1] = time.time() - Time
            Time = time.time()
            if self.Isprint:
                print('第%d轮' % count)
            if count % 10 == 0:
                print('第%d轮' % count, self.node.f, 'open:{} close:{}'.format(len(self.Open),len(self.Close)), 
                        'time:{:.4f}|{:.4f}|{:.4f}'.format(T[0], T[1], T[2]))
               
            self.SortOpen() #OPEN表排序
            T[2] = time.time() - Time
            Time = time.time()
            # break
        if self.Isprint:
            print('搜索了%d轮' % count) #CLOSE表空，无法搜索到路径
        return False, count

    def printans(self, node, anslist): #递归调用 实现正向输出
        if node is None:
            return
        self.printans(node.father, anslist)
        print(node.state, node.g)
        anslist.append(node.state)


def visualizgif(anslist):
    fig = plt.figure()
    plt.ion()  # 打开交互模式
    # 创建一个子图Axes对象
    axes1 = plt.axes((0, 0, 1, 1))
    index = 0
    for m in anslist:
        index += 1
        fig.clf()  # 清除画面
        ax = fig.add_subplot()  # 创建3d画布

        mat = m.reshape((4, 4))
        a = mat != 0
        ax.matshow(a)  # 显示矩阵
        plt.xticks(())  # 去掉坐标轴显示
        plt.yticks(())

        iters = np.reshape([[[i, j] for j in range(4)] for i in range(4)], (mat.size, 2))

        for i, j in iters:
            plt.text(j, i, format(mat[i, j]), fontsize=10)  # 显示对应的数字

        plt.savefig("./tmp/" + str(index) + ".png")
        plt.pause(0.5)
    plt.ioff()
    plt.show()

if __name__ == "__main__":

    # se = Search()
    # ans, count = se.getans()
    # anslist = []
    # se.printans(ans, anslist)
    # print(GOAL)
    s0 = np.array([11, 9, 4, 15, 1, 3, 0, 12, 7, 5, 8, 6, 13, 2, 10, 14])
    s1 = np.array([5, 1, 2, 4, 9, 6, 3, 8, 13, 15, 10, 11, 14, 0, 7, 12])
    relu = get_relu()
    se = Search(s0, relu, Isprint=False)
    ans, count = se.getans()
    anslist = []
    se.printans(ans, anslist)
    # visualizgif(anslist)
    # print(relu)