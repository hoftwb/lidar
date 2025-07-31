import numpy as np
import math
import matplotlib.pyplot as plt
from open3d import *  
import os 
import random
from PIL import Image
import cv2
import time
import argparse

from plyfile import PlyData

#   Featurelist:
#   - Takes in pointcloud in most formats, depth image, regular image, array of depths, array of zeds, etc.
#   - Outputs full graph of every point/pixel's position overtime, 
#   - Actual processing takes miliseconds, relatively optimized
#   Todos:
#   - Real Data

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', default=r'C:\Users\hoftw\first-lidar\renders', help='path to image folder')
    parser.add_argument('--type', default=r'png', help='filetype to read')
    parser.add_argument('--name', default=r'', help='common name of files')
    parser.add_argument('--graph', default=r'true', help='graph result (bool), capitalized')
    args = parser.parse_args()

    frames = []
    frames = scanpath(args.path, args.name, args.type)

    for i in range(len(frames)):
        depthimage(frames[i * 20], "depthimage_oxford_" + str(i))
    #analyzeframes(frames, args.graph == "true" or args.graph == "True")


def analyzeframes(frames, graph):
    starttime = time.time()

    pixeldepths = np.zeros((len(deptharray(frames[0])), len(frames))) #array of pixels which are arrays of depths


    match type(frames[0]):
        case open3d.cpu.pybind.geometry.PointCloud:
            depths = [deptharraypoint(sortpoints(frame)) for frame in frames]  

            smallestframe = min(map(len, depths))
            depths = [deptharraypoint(sortpoints(frame))[:(smallestframe-1)] for frame in frames]
            
            pixeldepths = np.array(depths)
            pixeldepths = np.transpose(pixeldepths) #rotate

            '''
            for f in range(len(frames)):
                depths = deptharray(sortpoints(frames[f])) 
                for i in range(len(depths)):
                    pixeldepths[i, f] = depths[i] #turn it into 
            '''

        case np.ndarray:
            depths = [deptharraypixel(frame) for frame in frames]
            pixeldepths = np.array(depths)
            pixeldepths = np.transpose(pixeldepths) #rotate

    endtime = time.time()
    timemessage(starttime, endtime, "analysis")

    if(graph):
        graphdepths(pixeldepths)

def addframe(deptharray, newframe): #Not Recommended
    match type(newframe):
        case open3d.cpu.pybind.geometry.PointCloud:
            newdepth = deptharraypixel(sortpoints(newframe)) 
            deptharray = np.concatenate((deptharray, np.transpose([newdepth])), axis=1)

        case np.ndarray:
            newdepth = deptharraypixel(newframe) 
            deptharray = np.concatenate((deptharray, np.transpose([newdepth])), axis=1)

    return deptharray

def rollframe(deptharray, newframe):
    match type(newframe):
        case open3d.cpu.pybind.geometry.PointCloud:
            newdepth = deptharraypixel(sortpoints(newframe)) 

        case np.ndarray:
            newdepth = deptharraypixel(newframe) 
    
    deptharray = np.roll(deptharray, -1, 1)
    deptharray[:, -1] = newdepth

    return deptharray

def graphdepths(pixeldepths):
    starttime = time.time()
    progressunit = random.randint(1000, 2000)

    for i in range(len(pixeldepths)):
        plt.plot([pixel for pixel in pixeldepths[i, :]])
        if i % progressunit == 0:
                print("Graphing pixel depth arrays (" + str(i) + "/" + str(len(pixeldepths)) + " with " + str(len(pixeldepths[0, :])) + " points per array)")

    endtime = time.time()
    timemessage(starttime, endtime, "graphing")

    print("--------------------------------------------")
    print("Largest depth is " + str(np.max(pixeldepths)))
    print("Smallest depth is " + str(np.min(pixeldepths)))
    print("25th percentile is " + str(np.percentile(pixeldepths, 25, axis=0)))
    print("50th percentile is " + str(np.percentile(pixeldepths, 50, axis=0)))
    print("75th percentile is " + str(np.percentile(pixeldepths, 75, axis=0)))
    print("Average depth is " + str(np.average(pixeldepths, axis=0)))
    print(str(np.sum(pixeldepths > np.median(pixeldepths)) / len(pixeldepths > np.median(pixeldepths))) + " percent of points are greater than the median of " + str(np.median(pixeldepths)))
    print(str(np.sum(pixeldepths == np.median(pixeldepths)) / len(pixeldepths == np.median(pixeldepths))) + " percent of points are equal to the median of " + str(np.median(pixeldepths)))
    print("--------------------------------------------")

    plt.show()
    
def deptharraypixel(frame):
    return frame.flatten()

def deptharraypoint(frame):
    return [math.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2) for point in frame.points]

def deptharray(frame):
    depths = []

    match type(frame):
        case open3d.cpu.pybind.geometry.PointCloud:
            depths = [math.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2) for point in frame.points]

        case np.ndarray:
            depths = frame.flatten()

    return depths

def sortpoints(pcd):
    points = np.asarray(pcd.points)
    #points = np.round(points, decimals=1)
    #points = points[np.argsort(points[:, 2])]
    points = points[np.lexsort((points[:, 0], points[:, 1], points[:, 2]))]
    #points = points[np.lexsort((points[:, 0] + points[:, 1], points[:, 2]))]
    pcd.points = open3d.utility.Vector3dVector(points)
    return pcd

def timemessage(start, end, task):
    print("Finished " + task + " at " + str(end) + ", took " + str((end * 1000 - start * 1000)) + "ms")

def scanpath(directory, commonname, filetype):
    starttime = time.time()
    
    output = []
    files = os.listdir(directory)
    files.sort()

    for file in files:
        if commonname in file and file[-3:] == filetype:
            path = os.path.join(directory, file)
            output.append(readfile(path))

            #convertply(path, path, True) #temp
        print("Reading " + file)

    endtime = time.time()
    timemessage(starttime, endtime, "reading files")
    
    return output

def readfile(directory):
    output = ""
    
    filetype = directory[-3:]
    match filetype:
        case "ply":
            #data = PlyData.read(directory)
            output = open3d.io.read_point_cloud(directory)
        case "pcd":
            output = open3d.io.read_point_cloud(directory)
        case "jpg":
            output = cv2.imread(directory, cv2.IMREAD_GRAYSCALE)
        case "png":
            output = cv2.imread(directory, cv2.IMREAD_GRAYSCALE)
    return output

def convertply(inputFile, outputFile, useAscii):
    tmesh = open3d.io.read_triangle_mesh(inputFile)
    open3d.io.write_triangle_mesh(outputFile, tmesh, write_ascii = useAscii)
    #open3d.io.write_point_cloud(outputFile, readfile(inputFile), write_ascii = useAscii)


def visualize(pc):
    visualization.draw_geometries([pc])
    #open3d.io.write_point_cloud("expcd1.pcd", pc)

def depthimage(pc, name):

    # 2. Define Camera Parameters
    width, height = 150, 150
    #fx, fy = 500, 500  # Focal lengths
    #cx, cy = width / 2 - 0.5, height / 2 - 0.5 # Principal point
    #pos = [5, 2, 5]

    #intrinsic = open3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

    # Example extrinsic (identity matrix for no rotation/translation)
    #extrinsic = np.eye(4)
    #extrinsic[:3, 3] = pos

    # 3. Create a Pinhole Camera Model
    #camera_parameters = open3d.camera.PinholeCameraParameters()
    #camera_parameters.intrinsic = intrinsic
    #camera_parameters.extrinsic = extrinsic

    camera_parameters = open3d.io.read_pinhole_camera_parameters(r"C:\Users\hoftw\first-lidar\ScreenCamera_2025-07-30-11-46-57.json")

    # 4. Visualize and Capture Depth Buffer
    vis = open3d.visualization.Visualizer()
    vis.create_window(width=camera_parameters.intrinsic.width, height=camera_parameters.intrinsic.height, visible=False) # Set visible=False for headless operation
    vis.get_render_option().point_size = 25.0
    vis.add_geometry(pc)

    # Set view control to the defined camera parameters
    view_control = vis.get_view_control()
    view_control.convert_from_pinhole_camera_parameters(camera_parameters)

    vis.poll_events()
    vis.update_renderer()

    depth_image = vis.capture_depth_float_buffer(do_render=True)
    depth_array = np.asarray(depth_image)

    open3d.io.write_image(name, open3d.geometry.Image(depth_array))

    vis.destroy_window()

    plt.imshow(depth_array, cmap='gray')
    plt.title("Generated Depth Image")
    plt.colorbar(label='Depth (Z-value)')
    plt.show()

    visualize(pc)
    
    return depth_array

if __name__ == "__main__":
    main()