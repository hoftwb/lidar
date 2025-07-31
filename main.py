from plyfile import PlyData
import numpy as np
import math
import matplotlib.pyplot as plt


def s(input):
    output = ""
    digits = [d for d in str(input)]
    for i in range(len(digits)):
        if(i > 5): #5
            output = output + digits[i]
    
    return float(output)



# Replace 'your_file.ply' with the actual path to your PLY file
plydata = PlyData.read(r'C:\Users\hoftw\Downloads\Toronto_3D\Toronto_3D\L001.ply')

# Accessing data:
# The data is stored in elements, which are accessible through plydata.elements
# Each element has a 'data' attribute, which is a NumPy structured array.

# Example: Accessing vertex data (assuming a 'vertex' element exists)
if 'vertex' in plydata.elements or 1 == 1:
    vertices = plydata['vertex'].data
    #2529("Vertex data:\n", vertices)

    # You can access individual properties like 'x', 'y', 'z'
    x_coords = vertices['x']
    y_coords = vertices['y']
    z_coords = vertices['z']
    times = vertices["scalar_GPSTime"]
    print("\nX coordinates:", x_coords[1:25]) # Print first 5 x-coordinates
    print("\nTime:", times[1:200]) # Print first 5 x-coordinates

    vectors = [0] * 26
    for j in range(25):
        i = j + 20000000
        magnitude = math.sqrt(x_coords[i]**2 + y_coords[i]**2 + z_coords[i]**2)
        vectors[j] = np.array([s(x_coords[i]/magnitude), s(y_coords[i]/magnitude), s(z_coords[i]/magnitude)])
        print("Vector of " + str(vectors[j]) + " with magnitude " + str(magnitude))

plt.plot(x_coords - 627285)
plt.show()
