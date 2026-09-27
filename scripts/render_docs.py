import bpy
import mathutils
import math
import numpy as np
import os

def setup_studio():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 64
    scene.cycles.device = 'CPU'
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 1000

    world = bpy.data.worlds.new("StudioWorld")
    scene.world = world
    world.node_tree.nodes['Background'].inputs['Color'].default_value = (0.05, 0.06, 0.08, 1.0)
    world.node_tree.nodes['Background'].inputs['Strength'].default_value = 0.8

    # Camera
    cam_data = bpy.data.cameras.new("HeroCamera")
    cam_data.lens = 70
    cam = bpy.data.objects.new("HeroCamera", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam

    # Key light
    key_data = bpy.data.lights.new("KeyLight", type='AREA')
    key_data.energy = 95.0
    key_data.size = 2.0
    key = bpy.data.objects.new("KeyLight", key_data)
    key.location = (0.6, -0.6, 0.9)
    key.rotation_euler = (math.radians(45), 0, math.radians(35))
    scene.collection.objects.link(key)

    # Fill light
    fill_data = bpy.data.lights.new("FillLight", type='AREA')
    fill_data.energy = 45.0
    fill_data.size = 3.0
    fill_data.color = (0.8, 0.9, 1.0)
    fill = bpy.data.objects.new("FillLight", fill_data)
    fill.location = (-0.7, -0.5, 0.6)
    fill.rotation_euler = (math.radians(35), 0, math.radians(-50))
    scene.collection.objects.link(fill)

    # Rim light
    rim_data = bpy.data.lights.new("RimLight", type='AREA')
    rim_data.energy = 85.0
    rim_data.size = 1.5
    rim = bpy.data.objects.new("RimLight", rim_data)
    rim.location = (0.1, 0.8, 0.7)
    rim.rotation_euler = (math.radians(-45), 0, math.radians(170))
    scene.collection.objects.link(rim)

    # Floor plane
    import bmesh
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=0.6)
    floor_mesh = bpy.data.meshes.new("Floor")
    bm.to_mesh(floor_mesh)
    bm.free()
    floor_obj = bpy.data.objects.new("Floor", floor_mesh)
    scene.collection.objects.link(floor_obj)

    floor_mat = bpy.data.materials.new("FloorMat")
    g_bsdf = floor_mat.node_tree.nodes['Principled BSDF']
    g_bsdf.inputs['Base Color'].default_value = (0.07, 0.08, 0.09, 1.0)
    g_bsdf.inputs['Roughness'].default_value = 0.5
    floor_obj.data.materials.append(floor_mat)

    return scene, cam, floor_obj

def make_material(name, color, roughness=0.35, metallic=0.0):
    mat = bpy.data.materials.new(name)
    bsdf = mat.node_tree.nodes['Principled BSDF']
    bsdf.inputs['Base Color'].default_value = color
    bsdf.inputs['Roughness'].default_value = roughness
    bsdf.inputs['Metallic'].default_value = metallic
    return mat

def render_original(out_path):
    print("=== Rendering Original Model ===")
    scene, cam, floor = setup_studio()
    
    bpy.ops.wm.stl_import(filepath='/home/dgmtnz/Downloads/VenomEscalado.stl')
    model = bpy.context.selected_objects[0]
    model.name = "VenomOriginal"
    
    verts = [v.co for v in model.data.vertices]
    cx = (min(v.x for v in verts) + max(v.x for v in verts)) / 2.0
    cy = (min(v.y for v in verts) + max(v.y for v in verts)) / 2.0
    cz = min(v.z for v in verts)
    
    for v in model.data.vertices:
        v.co.x = (v.co.x - cx) * 0.001
        v.co.y = (v.co.y - cy) * 0.001
        v.co.z = (v.co.z - cz) * 0.001
        
    mat = make_material("OriginalMat", (0.22, 0.25, 0.30, 1.0), roughness=0.35)
    model.data.materials.append(mat)
    
    # 3/4 front view
    rad = math.radians(60)
    dist = 0.125
    height = 0.040
    cam.location = (dist * math.cos(rad), dist * math.sin(rad), height)
    target = mathutils.Vector((0, 0, 0.021))
    cam.rotation_euler = (target - cam.location).to_track_quat('-Z', 'Y').to_euler()
    
    scene.render.filepath = out_path
    bpy.ops.render.render(write_still=True)
    print(f"Saved {out_path}")

def render_bed(out_path):
    print("=== Rendering Pieces on Print Bed ===")
    scene, cam, floor = setup_studio()
    
    floor.scale = (0.35, 0.35, 1.0)
    g_bsdf = floor.data.materials[0].node_tree.nodes['Principled BSDF']
    g_bsdf.inputs['Base Color'].default_value = (0.08, 0.09, 0.11, 1.0)
    g_bsdf.inputs['Roughness'].default_value = 0.55

    def load_centered(filepath, name, color, metal=0.0):
        bpy.ops.wm.stl_import(filepath=filepath)
        obj = bpy.context.selected_objects[0]
        obj.name = name
        verts = [v.co for v in obj.data.vertices]
        cx = (min(v.x for v in verts) + max(v.x for v in verts)) / 2.0
        cy = (min(v.y for v in verts) + max(v.y for v in verts)) / 2.0
        cz = min(v.z for v in verts)
        for v in obj.data.vertices:
            v.co.x = (v.co.x - cx) * 0.001
            v.co.y = (v.co.y - cy) * 0.001
            v.co.z = (v.co.z - cz) * 0.001
        obj.data.materials.append(make_material(name + "Mat", color, roughness=0.35, metallic=metal))
        return obj

    p1 = load_centered('/home/dgmtnz/Documents/Slyser/piece_1.stl', "Piece1", (0.10, 0.68, 0.80, 1.0))
    p2 = load_centered('/home/dgmtnz/Documents/Slyser/piece_2.stl', "Piece2", (0.96, 0.45, 0.15, 1.0))
    pin = load_centered('/home/dgmtnz/Documents/Slyser/pin_1.stl', "Pin", (0.98, 0.78, 0.15, 1.0), metal=0.4)

    p1.location = (-0.028, 0.0, 0.0)
    p2.location = (0.028, 0.0, 0.0)
    pin.location = (0.0, -0.022, 0.0)
    pin.rotation_euler = (math.radians(90), 0, 0)

    # Frame both pieces comfortably on bed
    cam.location = (0.0, -0.21, 0.14)
    target = mathutils.Vector((0.0, -0.005, 0.010))
    cam.rotation_euler = (target - cam.location).to_track_quat('-Z', 'Y').to_euler()

    scene.render.filepath = out_path
    bpy.ops.render.render(write_still=True)
    print(f"Saved {out_path}")

os.makedirs('/home/dgmtnz/Documents/Slyser/docs/images', exist_ok=True)
render_original('/home/dgmtnz/Documents/Slyser/docs/images/venom_original.png')
render_bed('/home/dgmtnz/Documents/Slyser/docs/images/venom_bed.png')
print("=== All renders completed successfully! ===")

