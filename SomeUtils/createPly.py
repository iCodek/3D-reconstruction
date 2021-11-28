from threading import Thread

import numpy as np


def asyncSavePly(vertices, colors, filename):
    Thread(target=savePly, args=(vertices, colors, filename)).start()


def savePly(vertices, colors, filename):
    colors = colors.reshape(-1, 3)
    colors = colors[:, [2, 1, 0]]
    vertices = np.hstack([vertices.reshape(-1, 3), colors])
    np.savetxt(filename, vertices, fmt='%f %f %f %d %d %d')
    ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''
    with open(filename, 'r+') as f:
        old = f.read()
        f.seek(0)
        f.write(ply_header % dict(vert_num=len(vertices)))
        f.write(old)
    print("saved success: " + filename)


if __name__ == '__main__':
    # Define name for output file
    output_file = 'Andre_Agassi_0015.ply'
    a = np.load("Andre_Agassi_0015.npy")
    b = np.float32(a)
    #   43867是我的点云的数量，用的时候记得改成自己的
    one = np.ones((43867, 3))
    one = np.float32(one) * 255
    #    points_3D = np.array([[1,2,3],[3,4,5]]) # 得到的3D点（x，y，z），即2个空间点
    #    colors = np.array([[0, 255, 255], [0, 255, 255]])   #给每个点添加rgb
    # Generate point cloud
    print("\n Creating the output file... \n")
    #    create_output(points_3D, colors, output_file)
    savePly(b, one, output_file)
