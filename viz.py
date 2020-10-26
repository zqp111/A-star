# -*- encoding: utf-8 -*-
'''
@File    :   viz.py
@Time    :   2020/10/24 16:47:58
@Author  :   zqp 
@Version :   1.0
@Contact :   zhangqipeng@buaa.edu.cn
'''
import matplotlib.pyplot as plt
import numpy as np

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
        a = mat != 16
        ax.matshow(a)  # 显示矩阵
        plt.xticks(())  # 去掉坐标轴显示
        plt.yticks(())

        iters = np.reshape([[[i, j] for j in range(4)] for i in range(4)], (mat.size, 2))

        for i, j in iters:
            plt.text(j, i, format(mat[i, j]), fontsize=10)  # 显示对应的数字

        plt.savefig("./tmp/" + str(index) + ".png")
        plt.pause(0.2)
    plt.ioff()
    plt.show()



if __name__ == "__main__":
    re = np.loadtxt('./re.txt', dtype=np.int32)
    print(re.shape)
    visualizgif(re)