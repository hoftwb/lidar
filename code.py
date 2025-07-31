import numpy as np
import math
import matplotlib.pyplot as plt
from open3d import *  
import os 
import random

from plyfile import PlyData

def main():
    #visualize(readfile(r"C:\Users\hoftw\first-lidar\output_2024-01-01_000000.pcd"))
    #visualize(readfile(r"C:\Users\hoftw\Downloads\Toronto_3D\L001.ply"))
    #convertply(r"C:\Users\hoftw\Downloads\simulate_LiDAR-main\bear_0.ply", r"C:\Users\hoftw\Downloads\simulate_LiDAR-main\bear_0.ply", True)

    #for pc in scanpath(r"C:\Users\hoftw\Downloads\simulate_LiDAR-main", "output", "pcd"):
        #visualize(pc)

    

    #frames = scanpath(r"C:\Users\hoftw\Downloads\simulate_LiDAR-main", "output_bear", "pcd")
        #frames.append(readfile(r"C:\Users\hoftw\first-lidar\lidar_bear_1_2024-01-01_000000.pcd"))
    #frames.append(readfile(r"C:\Users\hoftw\first-lidar\lidar_bear_0_2024-01-01_000000.pcd"))
    #pcdifference(frames[0], frames[1], True)



    frames = []
    #frames = scanpath(r"C:\Users\hoftw\first-lidar", "maindemo", "pcd")
    #random.shuffle(frames)

    frames.append(readfile(r"C:\Users\hoftw\first-lidar\lidar_cubedemo_0_2024-01-01_000000.pcd"))
    frames.append(readfile(r"C:\Users\hoftw\first-lidar\lidar_cubedemo_2_2024-01-01_000000.pcd"))
    frames.append(readfile(r"C:\Users\hoftw\first-lidar\lidar_cubedemo_3_2024-01-01_000000.pcd"))

    analyzeframes(frames[:])
    #visualize(frames[0])

def analyzeframes(frames):
    positions = []

    for i in range(len(frames) - 1):
        motion = findmotion(frames[i], frames[i + 1])
        positions.append(motion["start"])

    positions.append(
        findmotion(frames[-2], frames[-1])["end"]
    ) #haccy

    print(positions)

    plt.plot([position[0] for position in positions])
    plt.show()
    
    
def findmotion(frame1, frame2):
    difference = pcdifference(frame1, frame2, 0.4, 2, False) # 0.5, 0.93
    center = difference["diffpc"].get_center()

    uncutpoints = np.asarray(difference["diffpc"].points)
    cutpoints = uncutpoints[np.where(uncutpoints[:, 0] > center[0])]

    pc = open3d.geometry.PointCloud()
    pc.points = open3d.utility.Vector3dVector(cutpoints)

    
    #visualize(difference["diffpc"])
    #visualize(pc)

    start = pcintersection(pc, frame1).get_center()
    end = pcintersection(pc, frame2).get_center()

    result = {"start":start, "end":end}
    return result


    '''
    for i in range(1):
        center = pc.get_center()
        newpc = open3d.geometry.PointCloud()
        
        for i in range(len(pc.points)):
            if(pc.points[i])[0] > center[0]:
                newpc.points.append(pc.points[i])

        pc = newpc

    center = pc.get_center()
    '''

    #visualize(pc)

    #visualize(pcintersection(pc, frame1))
    #print(pcintersection(pc, frame1).get_center())
    #print(pcintersection(pc, frame2).get_center())

    #diffpc.remove_statistical_outlier(nb_neighbors=4, std_ratio=0.00000001)
    #diffpc.remove_radius_outlier(nb_points=5, radius=0.5)
    #visualize(diffpc)
    
    #print(center)

def pcintersection(pc1, pc2):
    distances = pc1.compute_point_cloud_distance(pc2)
    distances = np.asarray(distances)

    ind = np.where(distances > 0.01)[0]
    result = pc1.select_by_index(ind)
    return result

def pcdifference(pc1, pc2, lower, upper, show):    
    distances = pc1.compute_point_cloud_distance(pc2)
    #pointcount = len(pc1.points)

    diffpc = open3d.geometry.PointCloud()
    diffpoints = []
    viscolors = []
    
    for i in range(len(distances)):
        if(distances[i] > lower) and (distances[i] < upper):
            diffpoints.append(pc1.points[i])
            diffpoints.append(pc2.points[i])

            viscolors.append([1, 0, 0])
            viscolors.append([0, 0, 1])

    if show:
        vispc = open3d.geometry.PointCloud()
        vispc.points = open3d.utility.Vector3dVector(diffpoints)
        vispc.colors = open3d.utility.Vector3dVector(viscolors)
        visualize(vispc)
    

    diffpc.points = open3d.utility.Vector3dVector(diffpoints)

    output = {"distances":distances, "diffpc":diffpc}
    return output

def sortpoints(pcd):
    points = np.asarray(pcd.points)
    #points = np.round(points, decimals=1)
    #points = points[np.argsort(points[:, 2])]
    points = points[np.lexsort((points[:, 0], points[:, 1], points[:, 2]))]
    pcd.points = open3d.utility.Vector3dVector(points)
    return pcd
   

def scanpath(directory, commonname, filetype):
    pcs = []
    files = os.listdir(directory)
    files.sort()

    for file in files:
        if commonname in file and file[-3:] == filetype and "000000" in file and not "6" in file:
            path = os.path.join(directory, file)
            pcs.append(readfile(path))

            #convertply(path, path, True) #temp
        print(file)
    
    return pcs

def readfile(directory):
    pc = ""
    
    filetype = directory[-3:]
    match filetype:
        case "ply":
            #data = PlyData.read(directory)
            pc = open3d.io.read_point_cloud(directory)
        case "pcd":
            pc = open3d.io.read_point_cloud(directory)
    return pc

def convertply(inputFile, outputFile, useAscii):
    tmesh = open3d.io.read_triangle_mesh(inputFile)
    open3d.io.write_triangle_mesh(outputFile, tmesh, write_ascii = useAscii)
    #open3d.io.write_point_cloud(outputFile, readfile(inputFile), write_ascii = useAscii)


def visualize(pc):
    visualization.draw_geometries([pc])
    #open3d.io.write_point_cloud("expcd1.pcd", pc)

if __name__ == "__main__":
    main()