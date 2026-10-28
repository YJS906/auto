#!/usr/bin/env python3
"""Render only CAD paint faces into a metric, collision-free textured plane."""
import argparse
import json
from pathlib import Path
import cv2
import ezdxf
from ezdxf.path import make_path
import numpy as np
from PIL import Image, ImageDraw
from shapely.geometry import LineString
from shapely.ops import polygonize, unary_union
import yaml

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src/limo_autorace_sim'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', type=Path, default=PKG / 'config/track.yaml')
    args = ap.parse_args()
    c = yaml.safe_load(args.config.read_text())
    scale = c['meters_per_drawing_unit'] * c['scale_multiplier']
    w, h = c['width_m'] * c['scale_multiplier'], c['height_m'] * c['scale_multiplier']
    ppm = c['pixels_per_meter']
    nx, ny = round(w * ppm), round(h * ppm)
    ox, oy = c['origin_dxf']
    d = ezdxf.readfile(ROOT / 'assets/original/2026Track 도면 시트1.dxf')
    lines = []
    for e in d.modelspace():
        if e.dxf.layer != c['include_layer'] or e.dxf.handle in c['exclude_handles']:
            continue
        if e.dxftype() not in ('LINE', 'ARC', 'SPLINE', 'LWPOLYLINE', 'POLYLINE'):
            raise ValueError(f'Unreviewed entity: {e.dxftype()}')
        # make_path resolves ARC OCS/extrusion, including the many -Z arcs.
        points = list(make_path(e).flattening(c['flatten_tolerance_m'] / scale))
        xy = [(round((v.x - ox) * scale, 5), round((v.y - oy) * scale, 5)) for v in points]
        if len(set(xy)) > 1:
            lines.append(LineString(xy))
    faces = list(polygonize(unary_union(lines)))
    canvas = Image.new('RGB', (nx, ny), tuple(c['background_rgb']))
    painter = ImageDraw.Draw(canvas)
    accepted, rejected = [], []

    def pix(coords):
        return [(round(x * ppm), round((h - y) * ppm)) for x, y in coords]

    for face in faces:
        # Test local interior thickness, not area: long 20 mm stripes must survive.
        xmin, ymin, xmax, ymax = face.bounds
        local = Image.new('L', (max(3, round((xmax-xmin)*ppm)+5),
                                max(3, round((ymax-ymin)*ppm)+5)), 0)
        draw = ImageDraw.Draw(local)
        def lp(coords):
            return [(round((x-xmin)*ppm)+2, round((y-ymin)*ppm)+2) for x,y in coords]
        draw.polygon(lp(face.exterior.coords), fill=255)
        for hole in face.interiors:
            draw.polygon(lp(hole.coords), fill=0)
        radius = float(cv2.distanceTransform(np.array(local), cv2.DIST_L2, 5).max()) / ppm
        if len(face.interiors) >= 3 or radius > c['max_paint_half_width_m'] * c['scale_multiplier']:
            rejected.append(dict(area=face.area, radius=radius, bounds=face.bounds))
            continue
        paint = c['paint_rgb']
        # Geometry is preserved from CAD; checker colour alternation comes from PNG.
        cx, cy = face.centroid.x/c['scale_multiplier'], face.centroid.y/c['scale_multiplier']
        x0,y0,x1,y1 = c['checker_region_m']
        if x0 < cx < x1 and y0 < cy < y1:
            col = int((cx-x0)/c['checker_cell_m'])
            row = int((cy-c['checker_y_phase_m'])/c['checker_cell_m'])
            if (row+col)%2: paint = c['background_rgb']
        painter.polygon(pix(face.exterior.coords), fill=tuple(paint))
        for hole in face.interiors:
            painter.polygon(pix(hole.coords), fill=tuple(c['background_rgb']))
        accepted.append(dict(area=face.area, radius=radius, bounds=face.bounds))
    if len(accepted) < 40:
        raise RuntimeError('Too few paint faces: review CAD connectivity instead of inventing lines')
    model = PKG / 'models/autorace_track'
    tex = model / 'materials/textures/track.png'
    canvas.save(tex)
    preview = canvas.copy()
    preview.thumbnail((1500, 900))
    preview.save(ROOT / 'reports/track_preview.png')
    # Explicit UVs: image top is north (+Y); origin is the lower left of the mat.
    dae = f'''<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><created>2026-10-28T00:00:00Z</created><modified>2026-10-28T00:00:00Z</modified><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="plane"><mesh>
<source id="positions"><float_array id="positions-array" count="12">0 0 0 {w} 0 0 {w} {h} 0 0 {h} 0</float_array><technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="uv-array" count="8">0 0 1 0 1 1 0 1</float_array><technique_common><accessor source="#uv-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles count="2"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><p>0 0 1 1 2 2 0 0 2 2 3 3</p></triangles>
</mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene"><node id="plane_node"><instance_geometry url="#plane"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>'''
    (model / 'track.dae').write_text(dae)
    (model / 'model.config').write_text('<model><name>autorace_track</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><description>10 by 6 metre CAD paint, no walls</description></model>')
    (model / 'model.sdf').write_text(f'''<sdf version="1.9"><model name="autorace_track"><static>true</static><link name="paint">
<visual name="paint_only"><pose>{w/2} {h/2} 0.0005 0 0 0</pose><cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal><size>{w} {h}</size></plane></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular><pbr><metal><albedo_map>model://autorace_track/materials/textures/track.png</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr></material></visual>
</link></model></sdf>''')
    obstacles = ''
    for o in c['obstacles']:
        name = o['name']
        if not name.replace('_', '').isalnum():
            raise ValueError('Obstacle name must be alphanumeric')
        pose = ' '.join(map(str, o['xyz'])) + ' 0 0 0'
        size = ' '.join(map(str, o['size']))
        obstacles += f'<model name="{name}"><static>true</static><pose>{pose}</pose><link name="body"><collision name="solid"><geometry><box><size>{size}</size></box></geometry></collision><visual name="visible"><geometry><box><size>{size}</size></box></geometry><material><diffuse>1 0.3 0 1</diffuse></material></visual></link></model>'
    world = f'''<sdf version="1.9"><world name="autorace">
<physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.80665</gravity>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
<scene><grid>false</grid><ambient>0.7 0.7 0.7 1</ambient><background>0.15 0.18 0.22 1</background><shadows>false</shadows></scene>
<light name="sun" type="directional"><pose>0 0 8 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.1 0.1 -1</direction><cast_shadows>false</cast_shadows></light>
<model name="floor"><static>true</static><link name="floor"><collision name="flat_floor"><pose>{w/2} {h/2} -0.05 0 0 0</pose><geometry><box><size>{w+4} {h+4} 0.1</size></box></geometry><surface><friction><ode><mu>{c['ground_friction']}</mu><mu2>{c['ground_friction']}</mu2></ode></friction></surface></collision><visual name="floor"><pose>{w/2} {h/2} -0.05 0 0 0</pose><geometry><box><size>{w+4} {h+4} 0.1</size></box></geometry><material><diffuse>0.18 0.18 0.18 1</diffuse></material></visual></link></model>
<include><uri>model://autorace_track</uri></include>{obstacles}
</world></sdf>'''
    (PKG / 'worlds/autorace.sdf').write_text(world)
    report = dict(dimensions_m=[w,h], origin='DXF (128.493923,121.986082) -> world (0,0)',
                  axes='+X right in drawing, +Y up in drawing, +Z up from floor',
                  entity_count=len(lines), face_count=len(faces), accepted_paint=accepted,
                  rejected_large_faces=rejected, texture_pixels=[nx,ny],
                  minimum_20mm_stripe_pixels=0.02*ppm*c['scale_multiplier'],
                  paint_collisions=0, color_status='provisional',
                  direction_and_missions='unassigned; no competition rules supplied')
    (ROOT / 'reports/track_generation.json').write_text(json.dumps(report, indent=2))
    print(f'{len(accepted)} paint faces / {len(faces)} faces; {w} x {h} m; {nx} x {ny} pixels')


if __name__ == '__main__':
    main()
