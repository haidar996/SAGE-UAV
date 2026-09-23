import pytest

from sage_px4_interface.sage_mission_llm import (
    LLMParseError, spec_from_llm_fields, llm_parse_mission)


def fields(**kw):
    f = dict(valid=True, action='search', target_class='person',
             attribute='', quantity_all=True, quantity_count=0,
             output='locations', reason='')
    f.update(kw)
    return f


def test_all_people():
    s = spec_from_llm_fields('find people', fields())
    assert s['valid'] and s['supported'] and s['quantity'] == 'all'
    assert s['attribute'] is None and s['source'] == 'llm'


def test_count_and_output():
    s = spec_from_llm_fields('how many? up to 3', fields(
        quantity_all=False, quantity_count=3, output='count'))
    assert s['quantity'] == 3 and s['output'] == 'count'


def test_unsupported_class_decided_by_code():
    s = spec_from_llm_fields('x', fields(target_class='Car', attribute='Red'))
    assert s['valid'] and not s['supported']
    assert s['target_class'] == 'car' and s['attribute'] == 'red'
    assert 'not supported' in s['reason']


def test_invalid_mission():
    s = spec_from_llm_fields('hi', fields(
        valid=False, action='none', target_class='', reason='chit-chat'))
    assert not s['valid'] and not s['supported']


def test_bad_output_raises():
    with pytest.raises(LLMParseError):
        spec_from_llm_fields('x', fields(quantity_all=False, quantity_count=0))
    with pytest.raises(LLMParseError):
        spec_from_llm_fields('x', fields(output='poem'))
    with pytest.raises(LLMParseError):
        spec_from_llm_fields('x', {'valid': True})


def test_no_client_raises():
    with pytest.raises(LLMParseError):
        llm_parse_mission('find people', None)


def test_make_parser_none_without_backends(monkeypatch):
    from sage_px4_interface import sage_mission_llm as m
    monkeypatch.delenv('ANTHROPIC_API_KEY', raising=False)
    monkeypatch.setattr(m, 'ollama_available', lambda url=None: False)
    assert m.make_parser() is None


def test_ollama_error_raises(monkeypatch):
    from sage_px4_interface import sage_mission_llm as m
    with pytest.raises(LLMParseError):
        m.ollama_parse_mission('x', url='http://127.0.0.1:1', timeout_s=1)
