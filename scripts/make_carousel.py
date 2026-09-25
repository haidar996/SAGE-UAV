#!/usr/bin/env python3
"""Build a LinkedIn 'document' carousel (PDF, 1080x1350 pages) and the same pages as PNG from the project pictures.
usage: python3 scripts/make_carousel.py        -> demo/linkedin/SAGE-UAV-carousel.pdf, demo/linkedin/pages/*.png"""
import os
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
M = os.path.join(ROOT, 'docs', 'media')
OUT = os.path.join(ROOT, 'demo', 'linkedin')
os.makedirs(os.path.join(OUT, 'pages'), exist_ok=True)
W, H = 1080, 1350
BG, INK, ACC, MUT, GRN = (18, 32, 46), (255, 255, 255), (95, 165, 235), (150, 170, 190), (95, 208, 141)


def font(size, bold=True):
    for p in ('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf' if bold else '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
              '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf'):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def wrap(d, text, f, width):
    words, lines, cur = text.split(), [], ''
    for w in words:
        t = (cur + ' ' + w).strip()
        if d.textlength(t, font=f) <= width:
            cur = t
        else:
            lines.append(cur)
            cur = w
    lines.append(cur)
    return lines


def page(n, title, picture=None, caption=None, big=None):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, W, 14], fill=ACC)
    d.text((60, 50), 'SAGE-UAV', font=font(40), fill=INK)
    d.text((W - 160, 58), f'{n} / 6', font=font(28, False), fill=MUT)
    y = 140
    for ln in wrap(d, title, font(58), W - 120):
        d.text((60, y), ln, font=font(58), fill=INK)
        y += 74
    if big:
        for ln in wrap(d, big, font(44, False), W - 120):
            d.text((60, y + 20), ln, font=font(44, False), fill=MUT)
            y += 60
        y += 30
    if picture:
        pic = Image.open(os.path.join(M, picture)).convert('RGB')
        f = min((W - 120) / pic.width, 700 / pic.height)
        pic = pic.resize((int(pic.width * f), int(pic.height * f)), Image.LANCZOS)
        top = y + 40
        im.paste(pic, ((W - pic.width) // 2, top))
        y = top + pic.height + 40
    if caption:
        for ln in wrap(d, caption, font(34, False), W - 120):
            d.text((60, y), ln, font=font(34, False), fill=(205, 215, 225))
            y += 46
    return im


pages = [
    page(1, 'A drone that finds people from one sentence', 'hero.png',
         '"Find all people in this area and report their locations": it searches, detects, locates people in 3D, verifies, and lands itself.'),
    page(2, 'How it works', 'architecture.png',
         'Mission text becomes a validated spec. Perception, 3D localization and a semantic memory feed an active planner; every viewpoint passes an independent safety check before it reaches PX4.'),
    page(3, 'The flight', 'collage.png',
         'Onboard camera with YOLO detections and the live mission map: search, first detection, and people verified one by one.'),
    page(4, 'Seen from above the drone', 'drone_top_view.png',
         'A Gazebo camera mounted above the drone shows the real drone model (north-up, with a mini map). Same flight, 2x speed.'),
    page(5, 'Results in simulation', 'results.png',
         'Scored automatically against ground truth: people found, precision, position error, and mission time over repeated runs.'),
]
last = Image.new('RGB', (W, H), BG)
d = ImageDraw.Draw(last)
d.rectangle([0, 0, W, 14], fill=ACC)
d.text((60, 50), 'SAGE-UAV', font=font(40), fill=INK)
d.text((W - 160, 58), '6 / 6', font=font(28, False), fill=MUT)
d.text((60, 420), 'Open source', font=font(76), fill=INK)
d.text((60, 530), 'github.com/haidar996/SAGE-UAV', font=font(46), fill=GRN)
y = 640
for ln in ['ROS 2  |  PX4  |  Gazebo  |  YOLO', 'semantic mapping  |  active perception', '', 'Code, worlds, results, setup guide and the', 'demo videos are in the repository (MIT).']:
    d.text((60, y), ln, font=font(40, False), fill=(205, 215, 225))
    y += 58
pages.append(last)
for i, p in enumerate(pages, 1):
    p.save(os.path.join(OUT, 'pages', f'page_{i}.png'))
pages[0].save(os.path.join(OUT, 'SAGE-UAV-carousel.pdf'), save_all=True, append_images=pages[1:], resolution=100.0)
print('carousel:', os.path.join(OUT, 'SAGE-UAV-carousel.pdf'), os.path.getsize(os.path.join(OUT, 'SAGE-UAV-carousel.pdf')) // 1000, 'KB,', len(pages), 'pages')
