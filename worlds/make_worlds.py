#!/usr/bin/env python3
"""Generate worlds/sage_hard.sdf, config/sage_hard.env and the ground-truth
files config/truth_<world>.json (single source of truth for scoring).

All positions below are NED (north, east) metres = (gz y, gz x).
Run:  python3 worlds/make_worlds.py   then copy worlds/*.sdf to
~/PX4-Autopilot/Tools/simulation/gz/worlds/.
"""
import json
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
CONF = os.path.join(HERE, '..', 'config')
FUEL = 'https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/'

# ---- hard world definition -------------------------------------------------
STATIC_PEOPLE = {'S1': (0.0, 5.0), 'S2': (-9.0, 9.0), 'S3': (8.0, -8.0)}
WALKERS = {'W1': ((4.5, -11.0), (4.5, -5.0)),      # (start, end) NED
           'W2': ((-5.0, 5.0), (-5.0, 11.0))}
WALK_SPEED = 0.6                                    # m/s
# variant with long walker paths (reversals every ~20-24 s instead of ~10 s):
# W1 12 m along north 4.5; W2 8.9 m along north -3 (just clear of inflated building_1)
WALKERS_LONG = {'W1': ((4.5, -11.0), (4.5, 1.0)),
                'W2': ((-3.0, 2.6), (-3.0, 11.5))}
# solid obstacles: (name, north0, north1, east0, east1, height)
OBSTACLES = [('building_1', -8.0, -4.0, -4.0, 1.0, 4.0),
             ('building_2', 4.0, 8.0, 3.0, 9.0, 4.0),
             ('wall', -1.0, 1.0, -9.0, -5.0, 3.0),
             ('tree_1', 7.6, 8.4, -3.4, -2.6, 4.0),
             ('tree_2', -10.4, -9.6, -8.4, -7.6, 4.0),
             ('tree_3', -11.4, -10.6, 3.6, 4.4, 4.0)]
# visual-only distractors (lower than the flight altitude): name, n, e, sn, se, h, rgb
DISTRACTORS = [('car_red', 10.0, -1.0, 1.8, 4.2, 1.4, '0.8 0.1 0.1'),
               ('truck_blue', -11.0, -11.0, 2.2, 6.0, 2.4, '0.1 0.2 0.7'),
               ('shed', 11.0, 8.0, 3.0, 3.0, 2.5, '0.3 0.5 0.3')]
AREA = (-12.0, 12.0, -12.0, 12.0)                   # north_min,max,east_min,max


def gz(n, e):
    return e, n  # gz x = east, gz y = north


def static_actor(name, n, e, yaw):
    x, y = gz(n, e)
    return f'''
    <actor name="{name}"><skin><filename>{FUEL}talk_b.dae</filename><scale>1.0</scale></skin>
      <animation name="talk_b"><filename>{FUEL}talk_b.dae</filename><scale>0.055</scale><interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><auto_start>true</auto_start><trajectory id="0" type="talk_b">
        <waypoint><time>0</time><pose>{x} {y} 1.0 0 0 {yaw}</pose></waypoint>
        <waypoint><time>30</time><pose>{x} {y} 1.0 0 0 {yaw}</pose></waypoint>
      </trajectory></script></actor>'''


def walker(name, a, b):
    (an, ae), (bn, be) = a, b
    ax, ay = gz(an, ae); bx, by = gz(bn, be)
    d = math.hypot(bx - ax, by - ay); t = d / WALK_SPEED
    yaw_ab = math.atan2(by - ay, bx - ax); yaw_ba = yaw_ab + math.pi
    return f'''
    <actor name="{name}"><skin><filename>{FUEL}walk.dae</filename><scale>1.0</scale></skin>
      <animation name="walking"><filename>{FUEL}walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start><trajectory id="0" type="walking">
        <waypoint><time>0</time><pose>{ax} {ay} 1.0 0 0 {yaw_ab}</pose></waypoint>
        <waypoint><time>{t:.1f}</time><pose>{bx} {by} 1.0 0 0 {yaw_ab}</pose></waypoint>
        <waypoint><time>{t + 1:.1f}</time><pose>{bx} {by} 1.0 0 0 {yaw_ba}</pose></waypoint>
        <waypoint><time>{2 * t + 1:.1f}</time><pose>{ax} {ay} 1.0 0 0 {yaw_ba}</pose></waypoint>
        <waypoint><time>{2 * t + 2:.1f}</time><pose>{ax} {ay} 1.0 0 0 {yaw_ab}</pose></waypoint>
      </trajectory></script></actor>'''


def box(name, cn, ce, sn, se, h, rgb, collision):
    x, y = gz(cn, ce)
    col = (f'<collision name="c"><geometry><box><size>{se} {sn} {h}</size></box></geometry></collision>'
           if collision else '')
    return f'''
    <model name="{name}"><static>true</static><pose>{x} {y} {h / 2} 0 0 0</pose>
      <link name="l">{col}<visual name="v"><geometry><box><size>{se} {sn} {h}</size></box></geometry>
      <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual></link></model>'''


def build(world='sage_hard', walkers=None):
    walkers = walkers or WALKERS
    base = open(os.path.join(HERE, 'sage_sar.sdf'), encoding='utf-8').read()
    head = base[:base.index('<actor name="person_1">')]
    head = head.replace('<world name="sage_sar">', f'<world name="{world}">')
    body = ''
    for i, (k, (n, e)) in enumerate(STATIC_PEOPLE.items()):
        body += static_actor(k, n, e, 0.5 * i)
    for k, (a, b) in walkers.items():
        body += walker(k, a, b)
    rects = []
    for name, n0, n1, e0, e1, h in OBSTACLES:
        body += box(name, (n0 + n1) / 2, (e0 + e1) / 2, n1 - n0, e1 - e0, h,
                    '0.55 0.5 0.45' if 'build' in name else ('0.4 0.4 0.4' if name == 'wall' else '0.2 0.45 0.2'), True)
        rects.append((n0, n1, e0, e1))
    for name, n, e, sn, se, h, rgb in DISTRACTORS:
        body += box(name, n, e, sn, se, h, rgb, False)
    open(os.path.join(HERE, f'{world}.sdf'), 'w', encoding='utf-8').write(head + body + '\n  </world>\n</sdf>\n')

    truth = [{'id': k, 'type': 'static', 'points': [list(p)]} for k, p in STATIC_PEOPLE.items()]
    truth += [{'id': k, 'type': 'path', 'points': [list(a), list(b)]} for k, (a, b) in walkers.items()]
    json.dump({'world': world, 'tolerance_m': 1.5, 'people': truth},
              open(os.path.join(CONF, f'truth_{world}.json'), 'w'), indent=1)
    json.dump({'world': 'sage_sar', 'tolerance_m': 1.5, 'people': [
        {'id': 'P1', 'type': 'static', 'points': [[0.0, 4.0]]},
        {'id': 'P2', 'type': 'static', 'points': [[-6.0, 7.0]]},
        {'id': 'P3', 'type': 'static', 'points': [[5.0, -6.0]]}]},
        open(os.path.join(CONF, 'truth_sage_sar.json'), 'w'), indent=1)
    obst = ';'.join(f'{a},{b},{c},{d}' for a, b, c, d in rects)
    open(os.path.join(CONF, f'{world}.env'), 'w').write(
        f'SAGE_AREA="[{AREA[0]},{AREA[1]},{AREA[2]},{AREA[3]}]"\nSAGE_OBSTACLES="{obst}"\nSAGE_DRAIN=1800\n')
    # sanity: people must be outside the (1.5 m inflated) obstacles
    from itertools import product
    infl = [(a - 1.5, b + 1.5, c - 1.5, d + 1.5) for a, b, c, d in rects]
    pts = list(STATIC_PEOPLE.values()) + [p for ab in walkers.values() for p in ab]
    for p in pts:
        assert not any(a < p[0] < b and c < p[1] < d for a, b, c, d in infl), f'person {p} inside inflated obstacle'
    print(f'{world}.sdf written;', len(rects), 'obstacles;', len(pts), 'person anchor points ok')


# ---- rescue world: visually rich, 3 static people + ONE walker ---------------
RESCUE_STATIC = {'S1': (0.0, 5.0), 'S2': (-9.0, 9.0), 'S3': (10.0, -9.0)}
RESCUE_WALKERS = {'W1': ((4.5, -11.0), (4.5, 1.0))}     # 12 m, >= 5.5 m from every static person


def flat(name, cn, ce, sn, se, rgb, z=0.01):
    """Thin visual-only slab (roads, grass, markings): no collision, cannot be hit by the UAV."""
    x, y = gz(cn, ce)
    return f'''
    <model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><box><size>{se} {sn} 0.02</size></box></geometry>
      <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual></link></model>'''


def cylinder(name, cn, ce, r, h, rgb, collision, z0=0.0):
    x, y = gz(cn, ce)
    col = (f'<collision name="c"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>'
           if collision else '')
    return f'''
    <model name="{name}"><static>true</static><pose>{x} {y} {z0 + h / 2} 0 0 0</pose>
      <link name="l">{col}<visual name="v"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
      <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual></link></model>'''


def sphere(name, cn, ce, r, z, rgb):
    x, y = gz(cn, ce)
    return f'''
    <model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>
      <link name="l"><visual name="v"><geometry><sphere><radius>{r}</radius></sphere></geometry>
      <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual></link></model>'''


def build_rescue(world='sage_rescue'):
    base = open(os.path.join(HERE, 'sage_sar.sdf'), encoding='utf-8').read()
    head = base[:base.index('<actor name="person_1">')]
    head = head.replace('<world name="sage_sar">', f'<world name="{world}">')
    body = ''
    # terrain: grass patches and roads (visual only, flat)
    for i, (n, e, sn, se, rgb) in enumerate([(-6, -7, 9, 12, '0.25 0.45 0.2'), (7, 6, 9, 11, '0.3 0.5 0.22'),
                                             (-8, 8, 8, 9, '0.28 0.48 0.2'), (8, -8, 8, 8, '0.3 0.5 0.25')]):
        body += flat(f'grass_{i}', n, e, sn, se, rgb, 0.005)
    body += flat('road_ew', 2.5, 0.0, 2.2, 26.0, '0.22 0.22 0.24', 0.012)      # east-west road
    body += flat('road_ns', 0.0, -2.0, 26.0, 2.2, '0.22 0.22 0.24', 0.013)     # north-south road
    for k in range(-11, 12, 3):
        body += flat(f'dash_{k}', 2.5, float(k), 0.12, 1.2, '0.9 0.9 0.85', 0.02)
    body += flat('helipad', 0.0, 0.0, 3.0, 3.0, '0.15 0.15 0.18', 0.014)
    body += flat('helipad_h1', 0.0, -0.5, 1.6, 0.25, '0.95 0.95 0.95', 0.02)
    body += flat('helipad_h2', 0.0, 0.5, 1.6, 0.25, '0.95 0.95 0.95', 0.02)
    body += flat('helipad_h3', 0.0, 0.0, 0.25, 1.0, '0.95 0.95 0.95', 0.02)
    # people
    for i, (k, (n, e)) in enumerate(RESCUE_STATIC.items()):
        body += static_actor(k, n, e, 0.5 * i)
    for k, (a, b) in RESCUE_WALKERS.items():
        body += walker(k, a, b)
    # solid obstacles (same as sage_hard) - part of the known obstacle map
    rects = []
    for name, n0, n1, e0, e1, h in OBSTACLES:
        rgb = '0.75 0.62 0.5' if 'build' in name else ('0.45 0.45 0.48' if name == 'wall' else '0.15 0.4 0.15')
        body += box(name, (n0 + n1) / 2, (e0 + e1) / 2, n1 - n0, e1 - e0, h, rgb, True)
        rects.append((n0, n1, e0, e1))
        if 'build' in name:   # dark roof slab (visual only) so the houses read as houses
            body += flat(name + '_roof', (n0 + n1) / 2, (e0 + e1) / 2, n1 - n0 + 0.3, e1 - e0 + 0.3, '0.55 0.15 0.12', h + 0.02)
    # visual-only props, all lower than the 2 m flight altitude
    for i, (n, e, r) in enumerate([(-3.0, -9.5, 0.7), (3.0, 9.5, 0.6), (-11.0, 1.0, 0.7), (11.0, -3.0, 0.6),
                                   (7.5, -4.0, 0.5), (-5.5, 5.0, 0.55), (9.0, 6.0, 0.6), (-7.0, -1.5, 0.5)]):
        body += cylinder(f'bush_trunk_{i}', n, e, 0.08, 0.5, '0.35 0.22 0.1', False)
        body += sphere(f'bush_{i}', n, e, r, 0.55 + r * 0.6, '0.12 0.42 0.14')
    for name, n, e, sn, se, h, rgb in [('car_red', 1.0, 9.0, 1.8, 4.2, 1.3, '0.8 0.1 0.1'),
                                       ('car_white', 6.5, -6.5, 1.8, 4.2, 1.3, '0.9 0.9 0.92'),
                                       ('truck_blue', -11.0, -10.0, 2.2, 6.0, 1.7, '0.1 0.2 0.7'),
                                       ('container', 11.0, 8.0, 2.5, 6.0, 1.8, '0.7 0.35 0.1'),
                                       ('crate_a', -10.0, 4.0, 1.0, 1.0, 0.8, '0.6 0.45 0.25'),
                                       ('crate_b', 9.5, 2.0, 1.0, 1.0, 0.8, '0.6 0.45 0.25')]:
        body += box(name, n, e, sn, se, h, rgb, False)
    # (the demo top view comes from a camera mounted on the drone model: models/x500_mono_cam/model.sdf)
    open(os.path.join(HERE, f'{world}.sdf'), 'w', encoding='utf-8').write(head + body + '\n  </world>\n</sdf>\n')

    truth = [{'id': k, 'type': 'static', 'points': [list(p)]} for k, p in RESCUE_STATIC.items()]
    truth += [{'id': k, 'type': 'path', 'points': [list(a), list(b)]} for k, (a, b) in RESCUE_WALKERS.items()]
    json.dump({'world': world, 'tolerance_m': 1.5, 'people': truth},
              open(os.path.join(CONF, f'truth_{world}.json'), 'w'), indent=1)
    obst = ';'.join(f'{a},{b},{c},{d}' for a, b, c, d in rects)
    open(os.path.join(CONF, f'{world}.env'), 'w').write(
        f'SAGE_AREA="[{AREA[0]},{AREA[1]},{AREA[2]},{AREA[3]}]"\nSAGE_OBSTACLES="{obst}"\nSAGE_DRAIN=1800\n'
        'SAGE_PLANNER_ARGS="-p obstacle_margin:=2.5"\n')     # keep 2.5 m clear of walls/buildings (hover wobble ~1 m)
    infl = [(a - 1.5, b + 1.5, c - 1.5, d + 1.5) for a, b, c, d in rects]
    pts = list(RESCUE_STATIC.values()) + [p for ab in RESCUE_WALKERS.values() for p in ab]
    for p in pts:
        assert not any(a < p[0] < b and c < p[1] < d for a, b, c, d in infl), f'person {p} inside inflated obstacle'
    print(f'{world}.sdf written;', len(rects), 'obstacles;', len(pts), 'person anchor points ok')


if __name__ == '__main__':
    build()
    build('sage_hard_long', WALKERS_LONG)
    build_rescue()
