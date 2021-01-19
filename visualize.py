import numpy as np
import matplotlib.pyplot as plt
import sys, os, math


def draw_trajectory(tra):
    fig,ax = plt.subplots()
    ax.plot(tra[:,0],tra[:,1])
    plt.show()

def read_file(file_path):
    data = np.loadtxt(file_path, delimiter=',')
    print(data.shape)
    return data
   
if __name__ == "__main__":
    data = read_file("./tra.txt")
    data = np.vstack(( data,data[0,:] ))
    draw_trajectory(data)


