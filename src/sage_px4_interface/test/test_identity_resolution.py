from sage_px4_interface.identity_resolution import needs_check, resolve_identities


def E(i, x, y, c=0.8):
    return {'id': i, 'x': x, 'y': y, 'confidence': c}


# sage_hard layout: W2 duplicated (ids 1, 2), S2 4 m from the W2 path, S3 3.5 m from W1
W2a, W2b = E(1, -5.0, 5.0, 0.7), E(2, -5.0, 10.5, 0.8)
S2 = E(3, -9.0, 9.0)
S3, W1 = E(4, 8.0, -8.0), E(5, 4.5, -8.0)


def test_walker_pair_both_absent_is_merged():
    kept, merged = resolve_identities([W2a, W2b], {1: 'absent', 2: 'absent'})
    assert [e['id'] for e in kept] == [2]          # higher confidence kept
    assert merged == {1: 2}


def test_mover_and_absent_merge():
    kept, merged = resolve_identities([W2a, W2b], {1: 'mover', 2: 'absent'})
    assert len(kept) == 1 and merged == {1: 2}


def test_static_never_merges():
    kept, merged = resolve_identities([S3, W1], {4: 'static', 5: 'absent'})
    assert len(kept) == 2 and merged == {}
    kept, merged = resolve_identities([W2a, W2b], {1: 'static', 2: 'static'})
    assert len(kept) == 2 and merged == {}


def test_unknown_or_missing_never_merges():
    kept, _ = resolve_identities([W2a, W2b], {1: 'absent', 2: 'unknown'})
    assert len(kept) == 2
    kept, _ = resolve_identities([W2a, W2b], {})
    assert len(kept) == 2


def test_far_apart_absent_entries_stay_separate():
    a, b = E(1, 0.0, 0.0), E(2, 20.0, 0.0)
    kept, merged = resolve_identities([a, b], {1: 'absent', 2: 'absent'})
    assert len(kept) == 2 and merged == {}


def test_chain_merges_into_one_and_keeps_order():
    a, b, c = E(1, 0, 0, 0.5), E(2, 6, 0, 0.6), E(3, 12, 0, 0.9)
    kept, merged = resolve_identities([a, b, c], {1: 'absent', 2: 'absent', 3: 'absent'})
    assert [e['id'] for e in kept] == [3] and merged == {1: 3, 2: 3}


def test_mixed_world_keeps_all_real_people():
    entries = [W2a, W2b, S2, S3, W1]
    outcomes = {1: 'absent', 2: 'absent', 3: 'static', 4: 'static', 5: 'absent'}
    kept, merged = resolve_identities(entries, outcomes)
    assert sorted(e['id'] for e in kept) == [2, 3, 4, 5]     # only the W2 duplicate goes
    assert merged == {1: 2}


def test_needs_check_skips_confirmed_static_and_isolated():
    entries = [W2a, W2b, S2, S3, W1]
    ids = needs_check(entries, confirmed_static=(3,))
    assert 3 not in ids and 1 in ids and 2 in ids
    lone = needs_check([E(1, 0, 0), E(2, 50, 50)])
    assert lone == []
    close = needs_check([E(1, 0, 0), E(2, 1.0, 0)])       # inside the 2.5 m duplicate radius
    assert close == []
