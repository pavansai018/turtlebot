import cv2
import numpy as np
from stl import mesh


def generate_maze_stl_v2(image_path, output_filename, wall_height=10.0):
    # Load and invert: Black (0) becomes 255 (Wall)
    img = cv2.imread(image_path, 0)
    img = cv2.medianBlur(img, 3) # Remove noise
    _, thresh = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    
    h, w = thresh.shape
    vertices = []
    faces = []
    
    # We'll treat each pixel as a 1x1 unit in the STL
    def add_cube(x, y, z_height):
        # x, y are pixel coords; z is wall height
        # Corners of the cube
        x0, x1 = float(x), float(x + 1)
        y0, y1 = float(y), float(y + 1)
        z0, z1 = 0.0, float(z_height)
        
        curr_v = np.array([
            [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
            [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]
        ])
        
        start_idx = len(vertices)
        vertices.extend(curr_v)
        
        # Standard 12 triangles for a cube
        cube_faces = np.array([
            [0,3,1], [1,3,2], [0,4,7], [0,7,3], [4,5,6], [4,6,7],
            [5,1,2], [5,2,6], [2,3,6], [3,7,6], [0,1,5], [0,5,4]
        ]) + start_idx
        faces.extend(cube_faces)

    # To keep size down, we only add a cube if it's a wall
    # We also skip every 2nd pixel to "downsample" slightly
    for y in range(0, h, 2):
        for x in range(0, w, 2):
            if thresh[y, x] > 127:
                add_cube(x - w/2, -(y - h/2), wall_height)

    maze_mesh = mesh.Mesh(np.zeros(len(faces), dtype=mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            maze_mesh.vectors[i][j] = vertices[f[j]]
    
    maze_mesh.save(output_filename)
    print(f"Generated {output_filename}")

def generate_maze_stl(image_path, output_filename, wall_height=5.0):
    # 1. Load image and convert to binary (Black = Walls, White = Path)
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    
    # 2. Define dimensions
    h, w = binary.shape
    vertices = []
    faces = []
    
    # 3. Create 3D boxes for each black pixel
    # Note: For efficiency, we only process pixels that are part of the wall
    for y in range(h):
        for x in range(w):
            if binary[y, x] == 255:  # Wall pixel
                # Define 8 corners of a cube at this pixel location
                # We center the maze at (0,0)
                x0, x1 = x - w/2, x + 1 - w/2
                y0, y1 = -(y - h/2), -(y + 1 - h/2)
                z0, z1 = 0, wall_height
                
                v = np.array([
                    [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                    [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]
                ])
                
                # Offsets for faces
                start_idx = len(vertices)
                vertices.extend(v)
                
                # Define the 12 triangles (2 per cube face)
                f = np.array([
                    [0,3,1], [1,3,2], [0,4,7], [0,7,3], [4,5,6], [4,6,7],
                    [5,1,2], [5,2,6], [2,3,6], [3,7,6], [0,1,5], [0,5,4]
                ]) + start_idx
                faces.extend(f)

    # 4. Create the mesh object
    maze_mesh = mesh.Mesh(np.zeros(len(faces), dtype=mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            maze_mesh.vectors[i][j] = vertices[f[j]]

    # 5. Save the file
    maze_mesh.save(output_filename)
    print(f"Success! Saved {output_filename}")

if __name__ == "__main__":
    img_path = '../models/simple_maze/simple_maze.png'
    generate_maze_stl_v2(img_path, 'simple_maze.stl')