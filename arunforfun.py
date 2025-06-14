import numpy as np
import trimesh
from trimesh.creation import extrude_polygon, cylinder
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union
from shapely.validation import make_valid


def clean_polygon(vertices_2d):
    poly = Polygon(vertices_2d)
    print(f"Type of polygon: {type(poly)}")  # Debug
    if isinstance(poly, list):
        raise ValueError("Got a list instead of a Polygon in clean_polygon")

    # Fix invalid or self-intersecting polygons
    if not poly.is_valid:
        poly = make_valid(poly)
    poly = poly.buffer(0)  # Fixes minor geometry issues

    if poly.is_empty:
        raise ValueError("Polygon is empty after cleaning.")

    return poly

def extrude_polygon_fixed(vertices_2d, depth, axis='z'):
    poly = clean_polygon(vertices_2d)

    # Handle MultiPolygon by extruding each and merging
    if isinstance(poly, MultiPolygon):
        meshes = []
        for p in poly.geoms:
            m = extrude_polygon(p, depth)
            meshes.append(m)
        mesh = trimesh.util.concatenate(meshes)
    else:
        mesh = extrude_polygon(poly, depth)

    # Rotate extrusion based on axis
    if axis == 'x':
        R = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
        mesh.apply_transform(R)
    elif axis == 'y':
        R = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])
        mesh.apply_transform(R)

    return mesh


def model_cutout_in_circular_beam(length, diameter, xy_vertices, yz_vertices, xz_vertices):
    """
    Create a 3D model of a circular beam with a cutout defined in 3 orthogonal planes.
    """

    # Step 1: Create the cylindrical beam along X-axis
    beam = cylinder(radius=diameter/2, height=length, sections=64)
    beam.apply_translation([length/2, 0, 0])  # Centered on origin

    # Step 2: Create extruded 3D cutout volumes from 2D projections
    depth_xy = diameter  # For xy profile, extrude along Z
    depth_yz = length     # For yz profile, extrude along X
    depth_xz = diameter   # For xz profile, extrude along Y

    cutout_xy = extrude_polygon_fixed(xy_vertices, depth_xy, axis='z')
    cutout_yz = extrude_polygon_fixed(yz_vertices, depth_yz, axis='x')
    cutout_xz = extrude_polygon_fixed(xz_vertices, depth_xz, axis='y')

    
    print(f"beam watertight? {beam.is_watertight}")
    print(f"cutout_xy watertight? {cutout_xy.is_watertight}")
    print(f"cutout_yz watertight? {cutout_yz.is_watertight}")
    print(f"cutout_xz watertight? {cutout_xz.is_watertight}")


    # Step 3: Intersect the 3 volumes to create the final cutout
    cutout = cutout_xy.intersection(cutout_yz).intersection(cutout_xz)

    # Step 4: Subtract the cutout from the beam
    final = beam.difference(cutout)

    return final

# Example usage:
length = 12.0
diameter = 0.5

xy_vertices = [(8, -0.5), (8,0.5), (9,-0.5), (9,0.5)]  # rectangle in XY
yz_vertices = [(-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5), (0.5, -0.5)]  # rectangle in YZ
xz_vertices = [(8, -0.5), (8, 0.5), (9,-0.5), (9, 0.5)]  # rectangle in XZ

beam_with_cutout = model_cutout_in_circular_beam(length, diameter, xy_vertices, yz_vertices, xz_vertices)
