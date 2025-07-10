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

    normals = [
        [0, 0, -1],  # Bottom face normal (pointing down)
        [0, 0, 1],   # Top face normal (pointing up)
        [0, -1, 0],  # Back face normal
        [1, 0, 0],   # Right face normal
        [0, 1, 0],   # Front face normal
        [-1, 0, 0]   # Left face normal
    ]
    
    # Define faces (using 0-based indexing, will convert to 1-based for OBJ)
    faces = [
        # Bottom face (looking up from below, clockwise)
        ([0, 3, 2], 0), ([0, 2, 1], 0),
        
        # Top face (looking down from above, counter-clockwise)
        ([4, 5, 6], 1), ([4, 6, 7], 1),
        
        # Side faces (each side has 2 triangles)
        # Back face (Y = -length/2)
        ([0, 1, 5], 2), ([0, 5, 4], 2),
        
        # Right face (X = +width/2)  
        ([1, 2, 6], 3), ([1, 6, 5], 3),
        
        # Front face (Y = +length/2)
        ([2, 3, 7], 4), ([2, 7, 6], 4),
        
        # Left face (X = -width/2)
        ([3, 0, 4], 5), ([3, 4, 7], 5)
    ]
    
    # Write OBJ file
    with open(filename, 'w') as f:
        f.write("# Rectangular Frustum\n")
        
        # Write vertices
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

        for n in normals:
            f.write(f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n")
        
        # Write faces (convert to 1-based indexing for OBJ format)
        for face_data in faces:
            face_verts, normal_idx = face_data
            f.write(f"f {face_verts[0]+1}//{normal_idx+1} {face_verts[1]+1}//{normal_idx+1} {face_verts[2]+1}//{normal_idx+1}\n")

    
    print(f"Created rectangular frustum mesh: {filename}")
    return filename


if __name__ == '__main__':
    create_rectangular_frustum_mesh_file()
