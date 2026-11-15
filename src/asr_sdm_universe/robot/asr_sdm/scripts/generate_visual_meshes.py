#!/usr/bin/env python3
"""Regenerate meshes/visual/*.stl from the full-resolution CAD meshes in meshes/.

The URDF <visual> elements use the decimated copies, so RViz and Gazebo load
0.48 M triangles from the six mesh files instead of 2.0 M (1.85 M instead of
7.9 M for the whole robot). The originals stay in meshes/ as the reference
geometry for mass properties.

Requires numpy and pymeshlab (pip install pymeshlab).

    python3 scripts/generate_visual_meshes.py
"""

import os

import numpy as np
import pymeshlab

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE_DIR = os.path.join(PACKAGE_DIR, 'meshes')
VISUAL_DIR = os.path.join(SOURCE_DIR, 'visual')

# Face budget per mesh. RViz shades STL meshes per face, so the budgets are set by
# face normals, not by distance: with them the mean normal deviation from the
# original stays below 0.4 deg (surface deviation below 0.07 mm). Smaller budgets
# visibly facet the curved screw rotor blades.
TARGET_FACES = {
    'screw_rotor_left_0.stl': 250000,
    'screw_rotor_right_0.stl': 40000,
    'screwdrive_segment_0.stl': 120000,
    'joint_unit_a.stl': 30000,
    'joint_unit_b.stl': 30000,
    'joint_unit_cross.stl': 10000,
}

STL_DTYPE = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
WELD_TOLERANCE = 1e-6  # [m]
MM_PER_M = 1000.0


def read_binary_stl(path):
    return np.fromfile(path, dtype=STL_DTYPE, offset=84)['vertices'].astype(np.float64)


def weld_and_clean(triangles):
    """Shared vertices from a triangle soup, without degenerate or duplicate faces."""
    keys = np.round(triangles.reshape(-1, 3) / WELD_TOLERANCE).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    points = triangles.reshape(-1, 3)[first]
    faces = inverse.reshape(-1, 3)

    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) &
                  (faces[:, 0] != faces[:, 2])]
    _, keep = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return points, faces[np.sort(keep)]


def decimate(points, faces, target_faces):
    # Decimate in millimetres: in metres the optimal vertex placement goes
    # numerically wrong on sub-millimetre triangles and crumples curved surfaces.
    mesh_set = pymeshlab.MeshSet()
    mesh_set.add_mesh(pymeshlab.Mesh(
        vertex_matrix=points * MM_PER_M, face_matrix=faces.astype(np.int32)))
    if target_faces < len(faces):
        mesh_set.meshing_decimation_quadric_edge_collapse(
            targetfacenum=target_faces, qualitythr=0.3, preserveboundary=True,
            boundaryweight=2.0, preservenormal=True, optimalplacement=True,
            planarquadric=True, autoclean=True)
    mesh = mesh_set.current_mesh()
    return mesh.vertex_matrix() / MM_PER_M, mesh.face_matrix()


def write_binary_stl(path, points, faces, header):
    triangles = points[faces].astype(np.float32)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    data = np.zeros(len(faces), dtype=STL_DTYPE)
    data['normal'] = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)
    data['vertices'] = triangles
    with open(path, 'wb') as stream:
        stream.write(header.encode()[:80].ljust(80, b' '))
        stream.write(np.uint32(len(faces)).tobytes())
        data.tofile(stream)


def main():
    os.makedirs(VISUAL_DIR, exist_ok=True)
    for name, target_faces in TARGET_FACES.items():
        triangles = read_binary_stl(os.path.join(SOURCE_DIR, name))
        points, faces = decimate(*weld_and_clean(triangles), target_faces)
        write_binary_stl(
            os.path.join(VISUAL_DIR, name), points, faces,
            f'asr_sdm visual mesh, decimated from meshes/{name}')
        print(f'{name}: {len(triangles):,d} -> {len(faces):,d} faces')


if __name__ == '__main__':
    main()
