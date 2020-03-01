#!/usr/bin/env python

import numpy as np
import rospkg

if __name__ == '__main__':
    NUM_EXP = 200
    NUM_REPEAT = 1

    rospack = rospkg.RosPack()
    data_path = rospack.get_path('racecar_simulator')
    data = np.zeros((NUM_EXP, NUM_REPEAT, 5))
    for i in range(0, 20):
        # filename = "{}/data/dwa_{}.npy".format(data_path, i*10+9)
        filename = "dwa_{}.npy".format(i*10+9)
        data_i = np.load(filename)
        print(data_i[(i*10):((i+1)*10), :,:])
        data[(i*10):((i+1)*10), :,:] = data_i[(i*10):((i+1)*10), :,:]
    
    np.save("data_merged.npy", data)