"""Small, bounded proxy for browsing JPEGs on the ZR10 SD card."""
from collections import OrderedDict
from hashlib import sha256
from io import BytesIO
import json
import threading
from urllib.error import URLError
from urllib.parse import urlencode, urljoin, urlsplit
from urllib.request import urlopen

from PIL import Image


class CameraMediaError(RuntimeError):
    pass


class CameraMedia:
    def __init__(self, camera_ip='192.168.144.25', port=82, timeout=4):
        self.camera_ip = camera_ip
        self.base = f'http://{camera_ip}:{int(port)}/cgi-bin/media.cgi'
        self.timeout = timeout
        self.lock = threading.Lock()
        self.directories = OrderedDict()
        self.photos = OrderedDict()

    def _read(self, url, limit=64 * 1024 * 1024):
        parts = urlsplit(url)
        if parts.scheme != 'http' or parts.hostname != self.camera_ip or parts.port not in (None, 82):
            raise CameraMediaError('Camera returned an unexpected media address')
        try:
            with urlopen(url, timeout=self.timeout) as response:
                length = response.headers.get('Content-Length')
                if length and int(length) > limit:
                    raise CameraMediaError('Photo is too large to load')
                data = response.read(limit + 1)
        except (URLError, TimeoutError, OSError, ValueError) as exc:
            if isinstance(exc, CameraMediaError):
                raise
            raise CameraMediaError('Could not reach the camera SD card') from exc
        if len(data) > limit:
            raise CameraMediaError('Camera response exceeded the size limit')
        return data

    def _query(self, endpoint, params=None):
        url = self.base + endpoint
        if params:
            url += '?' + urlencode(params)
        try:
            body = self._read(url, 8 * 1024 * 1024)
            result = json.loads(body)
            if not result.get('success') or int(result.get('code', 0)) != 200:
                raise CameraMediaError(result.get('message') or 'Camera rejected the SD-card request')
            return result.get('data') or {}
        except (ValueError, TypeError, KeyError) as exc:
            if isinstance(exc, CameraMediaError):
                raise
            raise CameraMediaError('Camera returned an unreadable SD-card response') from exc

    def list_directories(self):
        data = self._query('/api/v1/getdirectories', {'media_type': 0})
        rows = data.get('directories', [])
        if not isinstance(rows, list):
            raise CameraMediaError('Camera returned an invalid folder list')
        result = []
        for row in rows:
            path = row.get('path') if isinstance(row, dict) else None
            if not isinstance(path, str) or not path:
                continue
            ident = sha256(path.encode()).hexdigest()[:32]
            name = str(row.get('name') or path.rstrip('/').split('/')[-1] or 'Photos')[:120]
            result.append({'id': ident, 'name': name, 'count': row.get('count')})
            with self.lock:
                self.directories[ident] = path
                self.directories.move_to_end(ident)
                while len(self.directories) > 100:
                    self.directories.popitem(last=False)
        return result

    def list_photos(self, directory_id, start=0, count=40):
        with self.lock:
            path = self.directories.get(directory_id)
        if path is None:
            raise CameraMediaError('Choose a camera folder again to refresh its photos')
        start = max(0, int(start))
        count = max(1, min(60, int(count)))
        query = {'media_type': 0, 'path': path}
        total = int(self._query('/api/v1/getmediacount', query).get('count', 0))
        if total < 0 or total > 1_000_000:
            raise CameraMediaError('Camera returned an invalid photo count')
        data = self._query('/api/v1/getmedialist', {**query, 'start': start, 'count': count})
        rows = data.get('list', [])
        if not isinstance(rows, list):
            raise CameraMediaError('Camera returned an invalid photo list')
        photos = []
        for row in rows:
            if not isinstance(row, dict) or not isinstance(row.get('url'), str):
                continue
            url = urljoin(self.base, row['url'])
            parts = urlsplit(url)
            if parts.scheme != 'http' or parts.hostname != self.camera_ip or parts.port not in (None, 82):
                continue
            ident = sha256(url.encode()).hexdigest()[:32]
            name = str(row.get('name') or f'photo-{start + len(photos) + 1}.jpg')[:180]
            photos.append({'id': ident, 'name': name, 'url': f'/api/media/image/{ident}',
                           'thumbnail': f'/api/media/image/{ident}?thumbnail=1', 'source': 'sd'})
            with self.lock:
                self.photos[ident] = url
                self.photos.move_to_end(ident)
                while len(self.photos) > 2000:
                    self.photos.popitem(last=False)
        return {'photos': photos, 'total': total, 'start': start, 'next': start + len(photos) if start + len(photos) < total else None}

    def image(self, ident, thumbnail=False):
        with self.lock:
            url = self.photos.get(ident)
        if url is None:
            raise CameraMediaError('Photo link expired; refresh the camera gallery')
        data = self._read(url)
        if not data.startswith(b'\xff\xd8'):
            raise CameraMediaError('Camera did not return a JPEG photo')
        if not thumbnail:
            return data
        try:
            image = Image.open(BytesIO(data)).convert('RGB')
            image.thumbnail((288, 162))
            out = BytesIO()
            image.save(out, 'JPEG', quality=72, optimize=False)
            return out.getvalue()
        except Exception as exc:
            raise CameraMediaError('Could not prepare the photo preview') from exc
