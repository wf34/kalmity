#!/usr/bin/env python3

import numpy as np

def create_rectangular_frustum_mesh_file(filename="rect_frustum.obj",
                                         bottom_width=0.2,
                                         top_width=0.02,
                                         aspect_ratio=1.77,
                                         height=0.1):
    """
    Creates an OBJ file for a rectangular frustum (truncated pyramid).
    
    Args:
        filename: Output OBJ file name
        bottom_width: Width of bottom rectangle (X direction)
        bottom_length: Length of bottom rectangle (Y direction)  
        top_width: Width of top rectangle (X direction)
        top_length: Length of top rectangle (Y direction)
        height: Height of frustum (Z direction)
    """
    bottom_length = bottom_width / aspect_ratio
    top_length = top_width / aspect_ratio

    vertices = []
    faces = []
    
    # Bottom rectangle corners
    bw, bl = bottom_width/2, bottom_length/2
    vertices.extend([
        [-bw, -bl, 0],  # 0: bottom back-left
        [ bw, -bl, 0],  # 1: bottom back-right  
        [ bw,  bl, 0],  # 2: bottom front-right
        [-bw,  bl, 0]   # 3: bottom front-left
    ])
    
    # Top rectangle corners
    tw, tl = top_width/2, top_length/2
    vertices.extend([
        [-tw, -tl, height],  # 4: top back-left
        [ tw, -tl, height],  # 5: top back-right
        [ tw,  tl, height],  # 6: top front-right  
        [-tw,  tl, height]   # 7: top front-left
    ])
    
    # Define faces (using 0-based indexing, will convert to 1-based for OBJ)
    faces = [
        # Bottom face (looking up from below, clockwise)
        [0, 3, 2], [0, 2, 1],
        
        # Top face (looking down from above, counter-clockwise)
        [4, 5, 6], [4, 6, 7],
        
        # Side faces (each side has 2 triangles)
        # Back face (Y = -length/2)
        [0, 1, 5], [0, 5, 4],
        
        # Right face (X = +width/2)  
        [1, 2, 6], [1, 6, 5],
        
        # Front face (Y = +length/2)
        [2, 3, 7], [2, 7, 6],
        
        # Left face (X = -width/2)
        [3, 0, 4], [3, 4, 7]
    ]
    
    # Write OBJ file
    with open(filename, 'w') as f:
        f.write("# Rectangular Frustum\n")
        
        # Write vertices
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        
        # Write faces (convert to 1-based indexing for OBJ format)
        for face in faces:
            f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")
    
    print(f"Created rectangular frustum mesh: {filename}")
    return filename


if __name__ == '__main__':
    create_rectangular_frustum_mesh_file()
