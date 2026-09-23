from sage_px4_interface.sage_mission_parser import parse_mission


def test_find_all_people():
    s = parse_mission('Find all people in this area')
    assert s['valid'] and s['supported']
    assert (s['action'], s['target_class'], s['quantity']) == (
        'search', 'person', 'all')
    assert s['output'] == 'locations'


def test_report_locations_of_persons():
    s = parse_mission('Search the area and report the locations of victims')
    assert s['target_class'] == 'person'
    assert s['quantity'] == 'all'
    assert s['output'] == 'locations'


def test_single_person():
    s = parse_mission('locate a person')
    assert s['valid'] and s['quantity'] == 1


def test_number_word_and_digit():
    assert parse_mission('find two people')['quantity'] == 2
    assert parse_mission('find 3 survivors')['quantity'] == 3


def test_attribute_and_unsupported_class():
    s = parse_mission('Find all red vehicles and report their locations')
    assert s['valid'] and not s['supported']
    assert s['target_class'] == 'car' and s['attribute'] == 'red'
    assert 'not supported' in s['reason']


def test_count_output():
    assert parse_mission('find all people and count them')['output'] == 'count'


def test_invalid_commands():
    assert not parse_mission('')['valid']
    assert not parse_mission('hello there')['valid']
    s = parse_mission('find the treasure')
    assert not s['valid'] and 'target' in s['reason']
    s = parse_mission('people are nice')
    assert not s['valid'] and 'action' in s['reason']
