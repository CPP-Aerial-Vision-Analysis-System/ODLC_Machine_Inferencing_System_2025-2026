"""Optional smoke check against a running *demo* server. Restores protection state."""
import json
import sys
from io import BytesIO
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from PIL import Image

base = sys.argv[1] if len(sys.argv) > 1 else 'http://127.0.0.1:8080'


def get(path):
    with urlopen(base + path, timeout=5) as response:
        return json.load(response)


def protect(photo, value, origin=None):
    request = Request(base + f"/api/photos/{photo['id']}/protect", method='POST',
        data=json.dumps({'protected':value}).encode(), headers={'Content-Type':'application/json',
        **({'Origin':origin} if origin else {})})
    with urlopen(request, timeout=5) as response:
        return json.load(response)


assert get('/api/status')['demo'], 'Only run this smoke test against demo mode'
photo = get('/api/photos?limit=1')['photos'][0]
for endpoint in ('image','thumbnail','download'):
    with urlopen(base + f"/api/photos/{photo['id']}/{endpoint}", timeout=5) as response:
        image = Image.open(BytesIO(response.read()))
        assert image.format == 'JPEG'
        if endpoint == 'download':
            assert 'attachment' in response.headers['Content-Disposition']
try:
    assert protect(photo, not photo['protected'])['protected'] != photo['protected']
finally:
    protect(photo, photo['protected'])
try:
    protect(photo, True, 'http://another-site.example')
    raise AssertionError('Cross-origin change accepted')
except HTTPError as error:
    assert error.code == 403
for path, expected in [('/api/photos?before=bad',400), ('/api/photos/'+ '0'*32 + '/image',404)]:
    try:
        get(path)
        raise AssertionError('Invalid request accepted')
    except HTTPError as error:
        assert error.code == expected
print('HTTP smoke checks passed: status, photos, JPEGs, downloads, protection, origin checks and errors.')
