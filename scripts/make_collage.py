#!/usr/bin/env python3
"""Build LinkedIn images from a recorded demo run.

usage: python3 scripts/make_collage.py results/demo/<run_id> [out_dir]
Writes  <out>/collage_2x2.png  (1200x675) and copies each still as a captioned 1200x675 picture.
"""
import glob
import os
import sys

from PIL import Image, ImageDraw, ImageFont

run = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(run, 'pictures')
os.makedirs(out, exist_ok=True)
stills = os.path.join(run, 'stills')


def font(size):
    for p in ('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf'):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def pick():
    order = ['search_start', 'first_detection']
    ver = sorted(glob.glob(os.path.join(stills, 'verified_*.png')),
                 key=lambda p: int(os.path.basename(p).split('_')[1].split('.')[0]))
    if ver:
        order.append(os.path.basename(ver[len(ver) // 2])[:-4])
        order.append(os.path.basename(ver[-1])[:-4])
    order += ['mission_complete', 'result_card']
    seen, res = set(), []
    for n in order:
        p = os.path.join(stills, n + '.png')
        if os.path.exists(p) and n not in seen:
            seen.add(n)
            res.append((n, p))
    return res


CAPTIONS = {'search_start': 'Autonomous sweep of the search area', 'first_detection': 'YOLO detects a person',
            'mission_complete': 'Every person located and reported', 'result_card': 'Mission complete'}
items = pick()
for n, p in items:
    im = Image.open(p).convert('RGB').resize((1200, 675))
    cap = CAPTIONS.get(n, 'Person verified and localized in 3D' if n.startswith('verified') else n)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 615, 1200, 675], fill=(18, 32, 46))
    d.text((24, 628), cap, font=font(28), fill=(255, 255, 255))
    d.text((1000, 636), 'SAGE-UAV', font=font(22), fill=(95, 165, 235))
    im.save(os.path.join(out, n + '.png'))

tiles = [Image.open(p).convert('RGB').resize((600, 338)) for n, p in items[:4]]
while len(tiles) < 4 and tiles:
    tiles.append(tiles[-1])
if tiles:
    sheet = Image.new('RGB', (1200, 676), (18, 32, 46))
    for i, t in enumerate(tiles):
        sheet.paste(t, ((i % 2) * 600, (i // 2) * 338))
    sheet.save(os.path.join(out, 'collage_2x2.png'))
print('pictures:', sorted(os.listdir(out)))
