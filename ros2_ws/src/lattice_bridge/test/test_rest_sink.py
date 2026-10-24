"""
lattice_bridge/test/test_rest_sink.py -- PURPOSE: prove the REST sink sorts every kind of answer into the right
bucket (delivered / try again later / this entity is refused / everything is refused), that it never leaks a
token, never follows a redirect, refuses unsafe addresses, and that together with the Publisher it rides out an
outage of the (fake) Lattice and delivers afterwards. Real sockets on 127.0.0.1 against fake_lattice.py.
"""

import socket

import pytest

from lattice_bridge.fake_lattice import FakeLattice
from lattice_bridge.sinks.base import AuthSinkError, ConfigSinkError, FatalSinkError, RetryableSinkError
from lattice_bridge.sinks.rest_sink import RestSink, is_private_host
from support_net import GOOD_TOKEN, OTHER_TOKEN, SANDBOX_TOKEN, FakeClock, make_publisher, target_at, uav_at


@pytest.fixture()
def clock():
    return FakeClock()


@pytest.fixture()
def fake(clock):
    server = FakeLattice(token=GOOD_TOKEN, clock=clock.now).start()
    yield server
    server.stop()


def sink_for(fake, token=GOOD_TOKEN, **kwargs):
    return RestSink(fake.url, token, **kwargs)


def test_an_entity_is_delivered_with_the_right_headers(fake, clock):
    sink_for(fake).publish_entity(uav_at(clock))
    assert list(fake.entities()) == [uav_at(clock)['entityId']]


def test_the_sandbox_token_is_sent_when_given(clock):
    with FakeLattice(token=GOOD_TOKEN, sandbox_token=SANDBOX_TOKEN, clock=clock.now) as server:
        RestSink(server.url, GOOD_TOKEN, SANDBOX_TOKEN).publish_entity(uav_at(clock))
        assert len(server.entities()) == 1
        with pytest.raises(AuthSinkError):
            RestSink(server.url, GOOD_TOKEN).publish_entity(uav_at(clock))


def test_a_wrong_token_is_a_systemic_auth_error_that_does_not_contain_the_token(fake, clock):
    with pytest.raises(AuthSinkError) as caught:
        sink_for(fake, OTHER_TOKEN).publish_entity(uav_at(clock))
    error = caught.value
    assert error.status == 401 and error.systemic is True
    assert OTHER_TOKEN not in error.message and GOOD_TOKEN not in error.message


def test_a_refused_entity_is_fatal_for_that_entity_only_and_says_why(fake, clock):
    broken = uav_at(clock)
    del broken['aliases']
    with pytest.raises(FatalSinkError) as caught:
        sink_for(fake).publish_entity(broken)
    assert caught.value.status == 400 and caught.value.systemic is False and 'aliases' in caught.value.message


@pytest.mark.parametrize('status', [404, 408, 409, 425, 429, 500, 502, 503, 504])
def test_these_statuses_mean_try_again_later(fake, clock, status):
    fake.configure(fail_status=status, fail_count=1)
    with pytest.raises(RetryableSinkError) as caught:
        sink_for(fake).publish_entity(uav_at(clock))
    assert caught.value.status == status


def test_the_servers_retry_after_hint_is_passed_on(fake, clock):
    fake.configure(fail_status=429, fail_count=1, retry_after_s=12)
    with pytest.raises(RetryableSinkError) as caught:
        sink_for(fake).publish_entity(uav_at(clock))
    assert caught.value.retry_after_s == 12


@pytest.mark.parametrize('value,expected', [(None, None), ('', None), ('soon', None), ('-3', None), ('0', None),
                                            ('nan', None), ('inf', None), ('5', 5.0), ('2.5', 2.5), ('99999', 3600.0),
                                            ('Wed, 21 Oct 2026 07:28:00 GMT', None)])
def test_retry_after_parsing(value, expected):
    assert RestSink._parse_retry_after(value) == expected


@pytest.mark.parametrize('status', [301, 302, 307])
def test_a_redirect_is_never_followed_and_is_systemic(fake, clock, status):
    fake.configure(fail_status=status, fail_count=1)
    with pytest.raises(FatalSinkError) as caught:
        sink_for(fake).publish_entity(uav_at(clock))
    assert caught.value.systemic is True and 'LATTICE_ENDPOINT' in caught.value.message


@pytest.mark.parametrize('status', [400, 413, 422, 418])
def test_other_client_errors_refuse_only_that_entity(fake, clock, status):
    fake.configure(fail_status=status, fail_count=1)
    with pytest.raises(FatalSinkError) as caught:
        sink_for(fake).publish_entity(uav_at(clock))
    assert caught.value.systemic is False and caught.value.status == status


def test_nobody_listening_is_a_retryable_error():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]                                       # a free port with nothing behind it
    with pytest.raises(RetryableSinkError) as caught:
        RestSink(f'http://127.0.0.1:{port}', GOOD_TOKEN, timeout_s=2).publish_entity(uav_at(FakeClock()))
    message = caught.value.message                                          # Windows times out instead of refusing
    assert ('cannot reach' in message or 'did not answer' in message) and GOOD_TOKEN not in message


def test_a_slow_server_is_a_retryable_timeout(fake, clock):
    fake.configure(delay_s=1.0)
    with pytest.raises(RetryableSinkError) as caught:
        sink_for(fake, timeout_s=0.2).publish_entity(uav_at(clock))
    assert 'did not answer' in caught.value.message


def test_an_entity_that_cannot_be_written_as_json_is_fatal_for_that_entity(fake, clock):
    entity = uav_at(clock)
    entity['description'] = float('nan')
    with pytest.raises(FatalSinkError) as caught:
        sink_for(fake).publish_entity(entity)
    assert caught.value.systemic is False and 'JSON' in caught.value.message


def test_a_secret_echoed_by_the_server_is_blanked_out(fake):
    sink = sink_for(fake)
    text = sink._scrub(f'bad request with {GOOD_TOKEN} inside\nand a new line')
    assert GOOD_TOKEN not in text and '***' in text and '\n' not in text


# ------------------------------------------------------------------------------------------ set-up mistakes
@pytest.mark.parametrize('endpoint', ['', 'lattice.example.com', 'ftp://host', 'http://', 'https://host:notaport'])
def test_a_bad_endpoint_is_a_config_error(endpoint):
    with pytest.raises(ConfigSinkError):
        RestSink(endpoint, GOOD_TOKEN)


def test_plain_http_to_a_remote_host_is_refused_but_a_lab_address_is_fine():
    with pytest.raises(ConfigSinkError) as caught:
        RestSink('http://lattice.example.com', GOOD_TOKEN)
    assert 'https' in caught.value.message
    for ok in ('http://127.0.0.1:8089', 'http://localhost:8089', 'http://192.168.144.25', 'http://10.0.0.5',
               'https://lattice.example.com'):
        RestSink(ok, GOOD_TOKEN)


@pytest.mark.parametrize('token', ['', 'has a\nline break', 'café', 'tab\there'])
def test_a_token_that_cannot_be_a_header_is_refused_without_printing_it(token):
    with pytest.raises(ConfigSinkError) as caught:
        RestSink('http://127.0.0.1:1', token)
    assert 'line break' not in caught.value.message.replace('a line break', '')


def test_a_bad_sandbox_token_is_refused_too():
    with pytest.raises(ConfigSinkError):
        RestSink('http://127.0.0.1:1', GOOD_TOKEN, 'bad\nvalue')


@pytest.mark.parametrize('timeout', [0, -1, float('nan'), float('inf'), True, 'ten'])
def test_a_bad_timeout_is_refused(timeout):
    with pytest.raises(ConfigSinkError):
        RestSink('http://127.0.0.1:1', GOOD_TOKEN, timeout_s=timeout)


def test_describe_never_shows_a_token(fake):
    plain = sink_for(fake).describe()
    boxed = RestSink(fake.url, GOOD_TOKEN, SANDBOX_TOKEN).describe()
    assert plain.startswith('rest http://127.0.0.1:') and boxed.endswith('(sandbox)')
    assert GOOD_TOKEN not in plain + boxed and SANDBOX_TOKEN not in boxed


def test_objects_are_not_stored_yet(fake):
    with pytest.raises(FatalSinkError):
        sink_for(fake).upload_object('astra/x.jpg', b'jpg')


def test_close_is_harmless_and_repeatable(fake):
    sink = sink_for(fake)
    sink.close()
    sink.close()


@pytest.mark.parametrize('host,expected', [('localhost', True), ('127.0.0.1', True), ('::1', True), ('[::1]', True),
                                           ('192.168.1.5', True), ('10.1.2.3', True), ('172.16.0.1', True),
                                           ('8.8.8.8', False), ('lattice.example.com', False), ('', False)])
def test_which_hosts_may_use_plain_http(host, expected):
    assert is_private_host(host) is expected


# ------------------------------------------------------------------------------------------ with the Publisher
def test_the_publisher_rides_out_an_outage_of_the_fake_lattice_and_delivers_afterwards(fake, clock):
    pub, _, _ = make_publisher(sink_for(fake), clock)
    fake.configure(down=True)
    pub.submit(target_at(clock, 'person', 1))
    assert pub.pump() == 1 and pub.health()['state'] == 'offline' and fake.entities() == {}
    fake.configure(down=False)
    clock.advance(pub.health()['next_attempt_in_s'] + 0.01)
    assert pub.pump() == 1
    assert pub.health()['state'] == 'online' and len(fake.entities()) == 1


def test_a_wrong_token_makes_the_publisher_pause_for_a_minute_and_keep_the_data(fake, clock):
    pub, _, _ = make_publisher(sink_for(fake, OTHER_TOKEN), clock)
    pub.submit(target_at(clock, 'person', 1))
    assert pub.pump() == 1
    health = pub.health()
    assert health['state'] == 'auth_failed' and health['queue_depth'] == 1 and health['next_attempt_in_s'] == pytest.approx(60)
    assert fake.stats()['refused_auth'] == 1 and OTHER_TOKEN not in health['last_error']
