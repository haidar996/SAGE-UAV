#!/usr/bin/env python3
"""Figures for the SAGE-UAV write-up: architecture diagram + results charts + a LinkedIn card.

usage: python3 scripts/make_figures.py            (reads results/trials_*.csv and results/logs/*)
Output: results/figures/*.png  and  results/summary.md
"""
import csv
import glob
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
RES = os.path.join(ROOT, 'results')
FIG = os.path.join(RES, 'figures')
os.makedirs(FIG, exist_ok=True)
INK, MUTED, BG = '#1b2733', '#6b7b8c', '#ffffff'
BLUE, GREEN, ORANGE, RED, GRAY = '#2f7fd1', '#3aa76d', '#f2a03d', '#d95b5b', '#c9d3dd'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'axes.edgecolor': MUTED, 'axes.labelcolor': INK,
                     'xtick.color': INK, 'ytick.color': INK, 'axes.spines.top': False, 'axes.spines.right': False})


# ---------------------------------------------------------------- architecture
def architecture():
    fig, ax = plt.subplots(figsize=(13, 6.2), dpi=150)
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 6.2)
    ax.axis('off')
    ax.text(0.3, 5.85, 'SAGE-UAV', fontsize=22, fontweight='bold', color=INK)
    ax.text(2.55, 5.88, 'Semantic AI-Guided Exploration & Active Search', fontsize=12.5, color=MUTED)

    def box(x, y, w, h, title, sub, color):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.12',
                                    fc=color, ec='none', alpha=0.13))
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.12',
                                    fc='none', ec=color, lw=1.6))
        ax.text(x + w / 2, y + h * 0.62, title, ha='center', va='center', fontsize=10.5, fontweight='bold', color=INK)
        ax.text(x + w / 2, y + h * 0.28, sub, ha='center', va='center', fontsize=8, color=MUTED)

    def arrow(x1, y1, x2, y2):
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle='-|>', mutation_scale=13, color=MUTED, lw=1.3))

    top = 3.9
    box(0.3, top, 1.9, 1.15, 'Mission', '"Find all people..."\nrule parser / LLM', ORANGE)
    box(2.65, top, 1.9, 1.15, 'Perception', 'camera + YOLO\n5 Hz person detector', BLUE)
    box(5.0, top, 1.9, 1.15, '3D Localization', 'ray + pose + attitude\n0.3 m median error', BLUE)
    box(7.35, top, 1.9, 1.15, 'Semantic Memory', 'tracks, motion state\nduplicate merging', GREEN)
    box(9.7, top, 1.9, 1.15, 'Active Planner', 'coverage sweep\nviewpoints, evidence', GREEN)
    box(12.0 - 0.0, top, 0.9, 1.15, 'Report', 'JSON', ORANGE)
    for x in (2.2, 4.55, 6.9, 9.25, 11.6):
        arrow(x + 0.02, top + 0.57, x + 0.43, top + 0.57)
    low = 1.5
    box(2.65, low, 2.4, 1.15, 'Energy Monitor', 'battery, return cost\nreturn-home flag', RED)
    box(5.5, low, 2.4, 1.15, 'Mission Manager', 'validates every viewpoint\nobstacle map, jump limit', ORANGE)
    box(8.35, low, 2.4, 1.15, 'Offboard Control', 'ROS 2 -> PX4 setpoints\nauto land + disarm', BLUE)
    # planner -> manager (elbow), manager -> offboard, energy -> manager
    ax.plot([10.65, 10.65, 6.7], [top - 0.02, 3.35, 3.35], color=MUTED, lw=1.3)
    arrow(6.7, 3.35, 6.7, low + 1.17)
    ax.text(10.75, 3.5, 'viewpoints', fontsize=8, color=MUTED)
    arrow(5.06, low + 0.57, 5.44, low + 0.57)
    arrow(7.96, low + 0.57, 8.29, low + 0.57)
    box(2.65, 0.15, 8.1, 0.85, 'PX4 SITL  +  Gazebo  (x500 quadrotor, mono camera, animated people)',
        'micro-XRCE-DDS bridge   |   ROS 2 Humble', GRAY)
    arrow(9.5, low - 0.02, 9.5, 1.02)
    ax.text(1.0, 3.35, 'safety checks between planning and flight', fontsize=8.3, color=MUTED, style='italic')
    ax.text(0.3, 2.0, 'Safety:', fontsize=9.5, fontweight='bold', color=INK)
    ax.text(0.3, 1.72, 'every planner output is\nchecked before flight;\nLLM can only fill fields', fontsize=8.3, color=MUTED, va='top')
    fig.savefig(os.path.join(FIG, 'architecture.png'), bbox_inches='tight', facecolor=BG)
    plt.close(fig)


# ---------------------------------------------------------------- results
def read_rows(path, keep):
    rows = []
    if not os.path.exists(path):
        return rows
    for r in csv.reader(open(path)):
        if r and r[0] != 'trial' and keep(r[0]) and len(r) > 10 and r[3] in ('area_covered', 'timeout') and r[5] != '':
            rows.append(dict(id=r[0], status=r[3], found=int(r[4]), tp=int(r[5]), fp=int(r[6]), fn=int(r[7]),
                             mean=float(r[8]) if r[8] not in ('', 'nan') else None,
                             mx=float(r[9]) if r[9] not in ('', 'nan') else None, dur=float(r[10])))
    return rows


def energy_used(run_id):
    p = os.path.join(RES, 'logs', run_id, 'energy.log')
    vals = []
    if os.path.exists(p):
        for l in open(p, errors='ignore'):
            m = re.search(r'remaining=([0-9.]+) %', l)
            if m:
                vals.append(float(m[1]))
    return (vals[0] - vals[-1]) if len(vals) > 10 else None


def summarize(name, rows, people):
    if not rows:
        return None
    tp = sum(r['tp'] for r in rows)
    fp = sum(r['fp'] for r in rows)
    means = [r['mean'] for r in rows if r['mean'] is not None]
    durs = sorted(r['dur'] for r in rows)
    en = [e for e in (energy_used(r['id']) for r in rows) if e is not None]
    return dict(name=name, runs=len(rows), recall=100 * tp / (people * len(rows)),
                precision=100 * tp / max(1, tp + fp), fp=fp, err=sum(means) / len(means) if means else float('nan'),
                dur_med=durs[len(durs) // 2], dur_min=durs[0], dur_max=durs[-1],
                energy=sum(en) / len(en) if en else float('nan'), rows=rows)


def results():
    sets = []
    sar = read_rows(os.path.join(RES, 'trials_sage_sar_px4mode.csv'), lambda i: True)
    sets.append(summarize('sage_sar\n(3 static)', sar, 3))
    hard = read_rows(os.path.join(RES, 'trials_sage_hard.csv'),
                     lambda i: i in ('0925_020852', '0925_022758', '0925_030627', '0925_032505'))
    sets.append(summarize('sage_hard\n(3 static + 2 walking)', hard, 5))
    resc = read_rows(os.path.join(RES, 'trials_sage_rescue.csv'), lambda i: True)
    sets.append(summarize('sage_rescue\n(3 static + 1 walking)', resc, 4))
    sets = [s for s in sets if s]
    if not sets:
        return []

    fig, axs = plt.subplots(1, 3, figsize=(14, 4.6), dpi=150)
    names = [s['name'] for s in sets]
    x = range(len(sets))
    w = 0.36
    axs[0].bar([i - w / 2 for i in x], [s['recall'] for s in sets], w, color=GREEN, label='recall')
    axs[0].bar([i + w / 2 for i in x], [s['precision'] for s in sets], w, color=BLUE, label='precision')
    for i, s in enumerate(sets):
        axs[0].text(i - w / 2, s['recall'] + 1.5, f"{s['recall']:.0f}%", ha='center', fontsize=9, color=INK)
        axs[0].text(i + w / 2, s['precision'] + 1.5, f"{s['precision']:.0f}%", ha='center', fontsize=9, color=INK)
    axs[0].set_ylim(0, 128)
    axs[0].set_xticks(list(x))
    axs[0].set_xticklabels(names, fontsize=8)
    axs[0].set_title('Detection quality (people found)', loc='left', fontsize=11, color=INK, fontweight='bold')
    axs[0].legend(frameon=False, fontsize=8, loc='upper center', ncol=2)
    for i, s in enumerate(sets):
        ds = [r['dur'] for r in s['rows']]
        axs[1].scatter([i] * len(ds), ds, s=46, color=ORANGE, zorder=3)
        axs[1].hlines(s['dur_med'], i - 0.25, i + 0.25, color=INK, lw=2)
        axs[2].scatter([i] * len(s['rows']), [r['mean'] for r in s['rows']], s=46, color=BLUE, zorder=3)
        axs[2].hlines(s['err'], i - 0.25, i + 0.25, color=INK, lw=2)
    axs[1].set_xticks(list(x))
    axs[1].set_xticklabels(names, fontsize=8)
    axs[1].set_ylabel('seconds')
    axs[1].set_title('Mission time per run (bar = median)', loc='left', fontsize=11, color=INK, fontweight='bold')
    axs[2].set_xticks(list(x))
    axs[2].set_xticklabels(names, fontsize=8)
    axs[2].set_ylabel('metres')
    axs[2].set_ylim(0, 1.2)
    axs[2].set_title('Mean localization error per run', loc='left', fontsize=11, color=INK, fontweight='bold')
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'results_overview.png'), bbox_inches='tight', facecolor=BG)
    plt.close(fig)

    with open(os.path.join(RES, 'summary.md'), 'w') as f:
        f.write('| world | runs | recall | precision | false pos. | mean error | time median (min-max) |\n')
        f.write('|---|---|---|---|---|---|---|\n')
        for s in sets:
            f.write(f"| {s['name'].splitlines()[0]} | {s['runs']} | {s['recall']:.0f}% | {s['precision']:.0f}% | "
                    f"{s['fp']} | {s['err']:.2f} m | {s['dur_med']:.0f} s ({s['dur_min']:.0f}-{s['dur_max']:.0f}) |\n")
    return sets


def linkedin_card(sets):
    best = next((s for s in reversed(sets) if s['name'].startswith('sage_rescue')), None) or sets[0]
    fig = plt.figure(figsize=(12, 6.27), dpi=150)
    fig.patch.set_facecolor('#12202e')
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis('off')
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 6.27)
    ax.add_patch(plt.Rectangle((0, 6.15), 12, 0.12, color=BLUE))
    ax.text(0.6, 5.35, 'SAGE-UAV', fontsize=44, fontweight='bold', color='white')
    ax.text(0.62, 4.72, 'Semantic AI-Guided Exploration & Active Search', fontsize=17, color='#b8c7d6')
    ax.text(0.62, 4.2, 'An autonomous drone that understands "find all people in this area",',
            fontsize=13, color='#8fa3b7')
    ax.text(0.62, 3.85, 'searches, detects, localizes in 3D, verifies, reports and lands by itself.',
            fontsize=13, color='#8fa3b7')
    stats = [(f"{best['recall']:.0f}%", 'of people found'), (f"{best['err']:.2f} m", 'mean position error'),
             (f"{best['precision']:.0f}%", 'precision'), (f"{best['dur_med'] / 60:.0f} min", 'to sweep 24 x 24 m')]
    for i, (big, small) in enumerate(stats):
        x = 0.62 + i * 2.85
        ax.text(x, 2.35, big, fontsize=34, fontweight='bold', color='#5fd08d')
        ax.text(x, 1.9, small, fontsize=12, color='#b8c7d6')
    ax.text(0.62, 0.7, 'ROS 2  |  PX4  |  Gazebo  |  YOLO  |  semantic mapping  |  active perception',
            fontsize=12, color='#6f8296')
    ax.text(0.62, 0.32, f"Simulation results, {best['runs']} scored runs (see docs/SCOPE.md)",
            fontsize=9.5, color='#56697c')
    fig.savefig(os.path.join(FIG, 'linkedin_card.png'), facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == '__main__':
    architecture()
    s = results()
    if s:
        linkedin_card(s)
    print('figures in', FIG, '| worlds summarised:', [x['name'].splitlines()[0] for x in s])
