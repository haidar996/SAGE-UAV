"""Pure decision logic for the active identity check (docs/design_identity_check.md).

After the coverage sweep the UAV revisits verified positions and labels each one:
  'static'  - a person is still there and does not drift
  'mover'   - a person is there but drifts (a walker passing)
  'absent'  - nobody there while the spot was in view (twice, from two sides)
  'unknown' - not checked / not in view / not affordable

Two verified entries are the SAME person only on positive evidence that the place is empty or
occupied by a mover at BOTH ends. A static or unknown outcome never merges: recall comes first.
"""
import math

MERGEABLE = ('absent', 'mover')


def needs_check(entries, reach=8.0, dup_dist=2.5, confirmed_static=()):
    """Ids that have another verified entry within `reach` (beyond the duplicate radius)
    and are not already confirmed static by re-detection evidence."""
    ids = []
    for a in entries:
        if a['id'] in confirmed_static:
            continue
        for b in entries:
            if a is b:
                continue
            d = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
            if dup_dist <= d <= reach:
                ids.append(a['id'])
                break
    return ids


def resolve_identities(entries, outcomes, reach=8.0):
    """Return (kept, merged_into) where merged_into maps a removed id to the kept id.

    entries: list of dicts with id, x, y, confidence.
    outcomes: dict id -> 'static' | 'mover' | 'absent' | 'unknown' (missing = unknown).
    """
    parent = {e['id']: e['id'] for e in entries}

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, a in enumerate(entries):
        for b in entries[i + 1:]:
            if outcomes.get(a['id'], 'unknown') not in MERGEABLE:
                continue
            if outcomes.get(b['id'], 'unknown') not in MERGEABLE:
                continue
            if math.hypot(a['x'] - b['x'], a['y'] - b['y']) <= reach:
                parent[find(a['id'])] = find(b['id'])

    groups = {}
    for e in entries:
        groups.setdefault(find(e['id']), []).append(e)

    kept, merged_into = [], {}
    for members in groups.values():
        best = max(members, key=lambda e: e.get('confidence', 0.0))
        kept.append(best)
        for e in members:
            if e is not best:
                merged_into[e['id']] = best['id']

    order = {e['id']: i for i, e in enumerate(entries)}
    kept.sort(key=lambda e: order[e['id']])
    return kept, merged_into
