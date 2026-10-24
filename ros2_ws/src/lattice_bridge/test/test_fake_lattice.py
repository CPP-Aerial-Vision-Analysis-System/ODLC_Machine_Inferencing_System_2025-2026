"""
lattice_bridge/test/test_fake_lattice.py -- PURPOSE: prove the pretend Lattice behaves like the real one where it
matters: it takes a valid entity, refuses a wrong token and a wrongly built entity (and says why), discards an update
that is not newer, takes an entity off the picture when isLive is false, can be told to fail / go slow on purpose,
draws a page that cannot be hijacked by an entity name, and never listens to the whole network by default.
Real sockets on 127.0.0.1 with an ephemeral port; no sleeping except one short deliberate delay.
"""

import json
import time
from datetime import timedelta

import pytest

from lattice_bridge.entities import build_uav_asset
from lattice_bridge.fake_lattice import FakeLattice, main
from support_net import (AUTH_HEADERS, GOOD_TOKEN, OTHER_TOKEN, SANDBOX_TOKEN, FakeClock, area_at, drop_at,
                         http_request, put_entity, target_at, uav_at)


@pytest.fixture()
def clock():
    return FakeClock()


@pytest.fixture()
def fake(clock):
    server = FakeLattice(token=GOOD_TOKEN, clock=clock.now).start()
    yield server
    server.stop()


def test_a_valid_entity_is_taken_and_appears_on_the_picture(fake, clock):
    entity = uav_at(clock)
    result = put_entity(fake.url, entity)
    assert result.status == 200 and result.json == {}
    assert list(fake.entities()) == [entity['entityId']]
    assert fake.stats()['accepted'] == 1


@pytest.mark.parametrize('headers', [{}, {'authorization': 'Bearer ' + OTHER_TOKEN}, {'authorization': 'Basic abc'},
                                     {'authorization': 'Bearer '}])
def test_a_missing_or_wrong_token_is_refused_and_nothing_is_stored(fake, clock, headers):
    body = json.dumps(uav_at(clock)).encode()
    result = http_request(fake.url, 'PUT', '/api/v1/entities', body, {'content-type': 'application/json', **headers})
    assert result.status == 401 and fake.entities() == {} and fake.stats()['refused_auth'] == 1


def test_the_sandbox_token_is_required_when_the_server_has_one(clock):
    with FakeLattice(token=GOOD_TOKEN, sandbox_token=SANDBOX_TOKEN, clock=clock.now) as server:
        assert put_entity(server.url, uav_at(clock)).status == 401
        good = put_entity(server.url, uav_at(clock), headers={'anduril-sandbox-authorization': 'Bearer ' + SANDBOX_TOKEN})
        assert good.status == 200


def test_open_mode_accepts_any_non_empty_bearer_token(clock):
    with FakeLattice(clock=clock.now) as server:
        assert put_entity(server.url, uav_at(clock), headers={'authorization': 'Bearer anything'}).status == 200
        assert put_entity(server.url, uav_at(clock), headers={'authorization': 'Bearer '}).status == 401


def test_a_wrongly_built_entity_is_refused_with_the_reason(fake, clock):
    broken = uav_at(clock)
    del broken['aliases']
    result = put_entity(fake.url, broken)
    assert result.status == 400 and 'aliases' in result.json['message'] and fake.entities() == {}
    assert fake.stats()['refused_contract'] == 1


@pytest.mark.parametrize('body', [b'not json', b'[1, 2]', b'"text"', b'\xff\xfe'])
def test_a_body_that_is_not_one_json_object_is_refused(fake, body):
    result = http_request(fake.url, 'PUT', '/api/v1/entities', body, {**AUTH_HEADERS})
    assert result.status == 400


def test_an_update_that_is_not_newer_is_discarded_but_still_answered_with_200(fake, clock):
    first = uav_at(clock, lat=36.2100)
    put_entity(fake.url, first)
    same_time_other_place = json.loads(json.dumps(first))
    same_time_other_place['location']['position']['latitudeDegrees'] = 36.2200
    assert put_entity(fake.url, same_time_other_place).status == 200
    held = fake.entities()[first['entityId']]
    assert held['location']['position']['latitudeDegrees'] == 36.21 and fake.stats()['discarded_older'] == 1
    clock.advance(1)
    newer = uav_at(clock, lat=36.2300)
    put_entity(fake.url, newer)
    assert fake.entities()[first['entityId']]['location']['position']['latitudeDegrees'] == 36.23


def test_is_live_false_takes_the_entity_off_the_picture_but_keeps_the_history(fake, clock):
    put_entity(fake.url, uav_at(clock))
    clock.advance(1)
    goodbye = uav_at(clock, is_live=False)
    assert put_entity(fake.url, goodbye).status == 200
    assert fake.entities() == {} and len(fake.history) == 2 and fake.stats()['went_offline'] == 1


def test_one_entity_can_be_read_back_and_an_unknown_one_is_404(fake, clock):
    entity = target_at(clock)
    put_entity(fake.url, entity)
    found = http_request(fake.url, 'GET', '/api/v1/entities/' + entity['entityId'])
    assert found.status == 200 and found.json['entityId'] == entity['entityId']
    assert http_request(fake.url, 'GET', '/api/v1/entities/nope').status == 404
    assert http_request(fake.url, 'GET', '/health').body == b'ok'
    assert len(http_request(fake.url, 'GET', '/entities.json').json) == 1
    assert http_request(fake.url, 'GET', '/stats.json').json['accepted'] == 1


def test_other_paths_and_methods_are_404(fake):
    assert http_request(fake.url, 'PUT', '/api/v1/other', b'{}', AUTH_HEADERS).status == 404
    assert http_request(fake.url, 'GET', '/nothing').status == 404
    assert http_request(fake.url, 'POST', '/elsewhere', b'{}').status == 404


def test_a_huge_body_is_refused_without_reading_it(fake):
    claimed = {**AUTH_HEADERS, 'content-length': str(5 * 1024 * 1024)}       # say it is huge; the server must not wait for it
    result = http_request(fake.url, 'PUT', '/api/v1/entities', b'x', claimed)
    assert result.status == 413


# ------------------------------------------------------------------------------------------ misbehaving on purpose
def test_down_answers_503_to_everything_until_it_is_switched_off(fake, clock):
    assert http_request(fake.url, 'POST', '/control', b'{"down": true}').status == 200
    assert put_entity(fake.url, uav_at(clock)).status == 503 and fake.entities() == {}
    http_request(fake.url, 'POST', '/control', b'{"down": false}')
    assert put_entity(fake.url, uav_at(clock)).status == 200


def test_the_next_n_requests_can_fail_with_a_chosen_status_and_retry_after(fake, clock):
    fake.configure(fail_status=429, fail_count=2, retry_after_s=7)
    first = put_entity(fake.url, uav_at(clock))
    assert first.status == 429 and first.headers['retry-after'] == '7'
    assert put_entity(fake.url, uav_at(clock)).status == 429
    assert put_entity(fake.url, uav_at(clock)).status == 200


def test_a_delay_slows_every_answer(fake, clock):
    fake.configure(delay_s=0.3)
    started = time.monotonic()
    put_entity(fake.url, uav_at(clock))
    assert time.monotonic() - started >= 0.28


def test_a_bad_control_message_is_refused(fake):
    assert http_request(fake.url, 'POST', '/control', b'nonsense').status == 400
    assert http_request(fake.url, 'POST', '/control', b'[1]').status == 400


# ------------------------------------------------------------------------------------------ the page
def test_the_page_draws_everything_that_is_on_the_picture(fake, clock):
    for entity in (uav_at(clock), target_at(clock, 'person'), target_at(clock, 'tent', 1, lat=36.2152, lon=-96.0069),
                   area_at(clock), drop_at(clock)):
        assert put_entity(fake.url, entity).status == 200
    page = http_request(fake.url, 'GET', '/').body.decode()
    assert page.count('<polygon') >= 2 and '<circle' in page and 'rotate(45' in page
    assert '5 entities on the picture' in page and 'http-equiv="refresh"' in page


def test_the_page_is_fine_when_the_picture_is_empty(fake):
    page = http_request(fake.url, 'GET', '/map').body.decode()
    assert 'Nothing on the picture yet' in page and '0 entities' in page


def test_an_entity_name_cannot_inject_markup_into_the_page(fake, clock):
    when = clock.now()
    nasty = build_uav_asset('astra-runway20261024t153000z-uav', 36.2143, -96.0068, fix_time=when, now=when,
                            name='<script>alert(1)</script>')
    assert put_entity(fake.url, nasty).status == 200
    page = http_request(fake.url, 'GET', '/').body.decode()
    assert '<script>alert' not in page and '&lt;script&gt;' in page


def test_it_listens_on_this_computer_only_by_default(clock):
    server = FakeLattice(clock=clock.now)
    assert server.host == '127.0.0.1'
    assert FakeLattice(host='0.0.0.0', clock=clock.now).url.startswith('http://127.0.0.1:')


def test_stop_can_be_called_twice(clock):
    server = FakeLattice(clock=clock.now).start()
    server.stop()
    server.stop()


def test_the_command_line_has_help():
    with pytest.raises(SystemExit) as stopped:
        main(['--help'])
    assert stopped.value.code == 0


def test_time_for_an_expired_entity_is_judged_on_the_servers_clock(fake, clock):
    entity = uav_at(clock)                                                  # expires 120 s after creation
    clock.advance(timedelta(seconds=130).total_seconds())
    assert put_entity(fake.url, entity).status == 400
