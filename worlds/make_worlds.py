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


def build():
    base = open(os.path.join(HERE, 'sage_sar.sdf'), encoding='utf-8').read()
    head = base[:base.index('<actor name="person_1">')]
    head = head.replace('<world name="sage_sar">', '<world name="sage_hard">')
    body = ''
    for i, (k, (n, e)) in enumerate(STATIC_PEOPLE.items()):
        body += static_actor(k, n, e, 0.5 * i)
    for k, (a, b) in WALKERS.items():
        body += walker(k, a, b)
    rects = []
    for name, n0, n1, e0, e1, h in OBSTACLES:
        body += box(name, (n0 + n1) / 2, (e0 + e1) / 2, n1 - n0, e1 - e0, h,
                    '0.55 0.5 0.45' if 'build' in name else ('0.4 0.4 0.4' if name == 'wall' else '0.2 0.45 0.2'), True)
        rects.append((n0, n1, e0, e1))
    for name, n, e, sn, se, h, rgb in DISTRACTORS:
        body += box(name, n, e, sn, se, h, rgb, False)
    open(os.path.join(HERE, 'sage_hard.sdf'), 'w', encoding='utf-8').write(head + body + '\n  </world>\n</sdf>\n')

    truth = [{'id': k, 'type': 'static', 'points': [list(p)]} for k, p in STATIC_PEOPLE.items()]
    truth += [{'id': k, 'type': 'path', 'points': [list(a), list(b)]} for k, (a, b) in WALKERS.items()]
    json.dump({'world': 'sage_hard', 'tolerance_m': 1.5, 'people': truth},
              open(os.path.join(CONF, 'truth_sage_hard.json'), 'w'), indent=1)
    json.dump({'world': 'sage_sar', 'tolerance_m': 1.5, 'people': [
        {'id': 'P1', 'type': 'static', 'points': [[0.0, 4.0]]},
        {'id': 'P2', 'type': 'static', 'points': [[-6.0, 7.0]]},
        {'id': 'P3', 'type': 'static', 'points': [[5.0, -6.0]]}]},
        open(os.path.join(CONF, 'truth_sage_sar.json'), 'w'), indent=1)
    obst = ';'.join(f'{a},{b},{c},{d}' for a, b, c, d in rects)
    open(os.path.join(CONF, 'sage_hard.env'), 'w').write(
        f'SAGE_AREA="[{AREA[0]},{AREA[1]},{AREA[2]},{AREA[3]}]"\nSAGE_OBSTACLES="{obst}"\nSAGE_DRAIN=1800\n')
    # sanity: people must be outside the (1.5 m inflated) obstacles
    from itertools import product
    infl = [(a - 1.5, b + 1.5, c - 1.5, d + 1.5) for a, b, c, d in rects]
    pts = list(STATIC_PEOPLE.values()) + [p for ab in WALKERS.values() for p in ab]
    for p in pts:
        assert not any(a < p[0] < b and c < p[1] < d for a, b, c, d in infl), f'person {p} inside inflated obstacle'
    print('sage_hard.sdf written;', len(rects), 'obstacles;', len(pts), 'person anchor points ok')


if __name__ == '__main__':
    build()
