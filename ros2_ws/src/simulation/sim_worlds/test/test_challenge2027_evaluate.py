"""PURPOSE: Prevent missing evidence or malformed 2027 reports from becoming successful diagnostics."""

from copy import deepcopy

import pytest

from sim_worlds.challenge2027.evaluate import REPORT_SCHEMA, evaluate_report
from sim_worlds.challenge2027.scenario import make_scene


@pytest.fixture(scope='module')
def scene():
    """Actual generated answer key, not a mock of the evaluator's implementation."""
    return make_scene()


def report(scene):
    """A complete locally defined result at the real target and nearest valid clearing."""
    h=scene['hiker']
    return dict(schema=REPORT_SCHEMA,hiker={k:h[k] for k in
        ('x','y','shirt_color','pants_color','suitcase_color')},landing_zone_id='clearing_near',
        elapsed_s=2700,delivery=dict(x=h['x'],y=h['y'],release_agl_m=45.72))


def test_exact_answer_has_diagnostics_without_official_score(scene):
    result=evaluate_report(scene,report(scene))
    assert result['official_score'] is False
    assert result['hiker']['location_error_m']==0
    assert all(result['hiker']['attributes_correct'].values())
    assert result['landing_zone']['best_selected'] is True
    assert result['delivery']['within_target_radius'] is True
    assert result['delivery']['above_minimum_agl'] is True
    assert result['delivery']['retardant_verified'] is None
    assert result['within_45min'] is True


def test_nearest_candidate_is_rejected_when_blocked(scene):
    r=report(scene)
    r['landing_zone_id']='blocked'
    result=evaluate_report(scene,r)
    assert result['landing_zone']['valid'] is False
    assert result['landing_zone']['best_selected'] is False


def test_hiker_location_colors_delivery_and_time_are_independent(scene):
    r=report(scene)
    r['hiker']['x']+=3
    r['hiker']['y']+=4
    r['hiker']['shirt_color']='purple'
    r['hiker'].pop('pants_color')
    r['delivery']['x']+=15.241
    r['delivery']['release_agl_m']=45.719
    r['elapsed_s']=2700.01
    result=evaluate_report(scene,r)
    assert result['hiker']['location_error_m']==pytest.approx(5)
    assert result['hiker']['attributes_correct']['shirt_color'] is False
    assert result['hiker']['attributes_correct']['pants_color'] is None
    assert result['delivery']['within_target_radius'] is False
    assert result['delivery']['above_minimum_agl'] is False
    assert result['within_45min'] is False


def test_missing_altitude_or_delivery_is_unknown(scene):
    r=report(scene)
    r['delivery'].pop('release_agl_m')
    assert evaluate_report(scene,r)['delivery']['above_minimum_agl'] is None
    r.pop('delivery')
    assert evaluate_report(scene,r)['delivery'] is None


def test_negative_case_does_not_reward_hallucinated_target_or_landing_zone(scene):
    negative=deepcopy(scene)
    negative['hiker']=None
    result=evaluate_report(negative,report(scene))
    assert result['hiker']['false_alarm'] is True
    assert result['hiker']['location_error_m'] is None
    assert result['landing_zone']['best_selected'] is None
    assert result['delivery']['within_target_radius'] is None
    r=dict(schema=REPORT_SCHEMA,hiker=None)
    assert evaluate_report(negative,r)['hiker']['false_alarm'] is False
    assert evaluate_report(scene,r)['hiker']['missed'] is True


@pytest.mark.parametrize('bad',[True,float('nan'),float('inf'),None,'42',[],{},10**400])
def test_invalid_coordinates_never_produce_metrics(scene,bad):
    r=report(scene)
    r['hiker']['x']=bad
    with pytest.raises(ValueError,match='finite number'):
        evaluate_report(scene,r)


@pytest.mark.parametrize('value',[-1,True,float('inf')])
def test_invalid_elapsed_time_is_rejected(scene,value):
    r=report(scene)
    r['elapsed_s']=value
    with pytest.raises(ValueError):
        evaluate_report(scene,r)


@pytest.mark.parametrize('value',[-1,True,'45.72',float('nan')])
def test_invalid_release_height_is_rejected(scene,value):
    r=report(scene)
    r['delivery']['release_agl_m']=value
    with pytest.raises(ValueError):
        evaluate_report(scene,r)


def test_wrong_schema_missing_target_and_unknown_zone_rejected(scene):
    for r in ({},dict(schema='official',hiker=None),dict(schema=REPORT_SCHEMA),
              dict(schema=REPORT_SCHEMA,hiker=None,landing_zone_id='missing')):
        with pytest.raises(ValueError):
            evaluate_report(scene,r)


def test_valid_zone_ties_are_not_broken_by_list_order(scene):
    s=deepcopy(scene)
    near=deepcopy(next(z for z in s['landing_zones'] if z['id']=='clearing_near'))
    near['id']='same_distance'
    s['landing_zones'].insert(0,near)
    r=report(scene)
    r['landing_zone_id']='same_distance'
    assert evaluate_report(s,r)['landing_zone']['best_selected'] is True


def test_bad_scene_validity_and_duplicate_zone_ids_rejected(scene):
    for change in ('duplicate','valid'):
        s=deepcopy(scene)
        if change=='duplicate':
            s['landing_zones'].append(s['landing_zones'][0])
        else:
            s['landing_zones'][0]['valid']='false'
        with pytest.raises(ValueError):
            evaluate_report(s,report(scene))
