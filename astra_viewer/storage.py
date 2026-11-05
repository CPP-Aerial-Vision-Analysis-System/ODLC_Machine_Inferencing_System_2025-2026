"""Bounded snapshot archive. Protected photos are never automatically removed."""
import json
import hashlib
import os
from pathlib import Path
import shutil
import sqlite3
import threading
import time
import uuid


class Archive:
    def __init__(self, directory, max_bytes=10 * 1024**3, reserve_bytes=2 * 1024**3):
        self.root = Path(directory).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.max_bytes, self.reserve_bytes = max_bytes, reserve_bytes
        self.lock = threading.RLock()
        self.db = sqlite3.connect(self.root / 'archive.sqlite3', check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('''CREATE TABLE IF NOT EXISTS photos (
            id TEXT PRIMARY KEY, captured REAL, frame_key TEXT UNIQUE, camera TEXT,
            objects TEXT, confidence REAL, protected INTEGER DEFAULT 0,
            bytes INTEGER, width INTEGER, height INTEGER)''')
        self.db.execute('CREATE INDEX IF NOT EXISTS photos_order ON photos(captured DESC,id DESC)')
        self.db.execute('''CREATE INDEX IF NOT EXISTS photos_cleanup ON photos(
            CASE WHEN confidence IS NULL THEN 0 WHEN confidence < .5 THEN 1
            WHEN confidence < .8 THEN 2 ELSE 3 END, captured) WHERE protected=0''')
        self.db.commit()
        self.used, self.count = self.db.execute('SELECT COALESCE(SUM(bytes),0),COUNT(*) FROM photos').fetchone()
        self.deleted = 0
        # Recover an interrupted write. Only files owned by this archive are touched.
        known = {r[0] for r in self.db.execute('SELECT id FROM photos')}
        for path in self.root.glob('*.jpg'):
            base = path.name.removesuffix('.thumb.jpg').removesuffix('.jpg')
            if len(base) == 32 and all(c in '0123456789abcdef' for c in base) and base not in known:
                path.unlink()

    def _usage(self):
        return self.used

    def _make_room(self, incoming):
        # Retain higher-confidence images longer; oldest goes first within each tier.
        used = self._usage()
        while used + incoming > self.max_bytes or shutil.disk_usage(self.root).free - incoming < self.reserve_bytes:
            victim = self.db.execute('''SELECT id, bytes FROM photos WHERE protected=0
                ORDER BY CASE WHEN confidence IS NULL THEN 0 WHEN confidence < .5 THEN 1
                WHEN confidence < .8 THEN 2 ELSE 3 END, captured ASC LIMIT 1''').fetchone()
            if victim is None:
                raise OSError('Archive is full. Protected photos are retained; free disk space or increase the archive limit.')
            # Commit deletion before unlinking: a crash leaves an orphan, not a broken record.
            self.db.execute('DELETE FROM photos WHERE id=?', (victim['id'],))
            self.db.commit()
            for suffix in ('.jpg', '.thumb.jpg'):
                (self.root / (victim['id'] + suffix)).unlink(missing_ok=True)
            used -= victim['bytes']
            self.used = used
            self.count -= 1
            self.deleted += 1

    def save(self, jpeg, thumbnail, metadata):
        with self.lock:
            if self.db.execute('SELECT 1 FROM photos WHERE frame_key=?', (metadata['frame_key'],)).fetchone():
                return None
            size = len(jpeg) + len(thumbnail)
            if size > self.max_bytes:
                raise OSError('A photo exceeds the archive size limit.')
            self._make_room(size)
            ident = uuid.uuid4().hex
            paths = []
            try:
                for suffix, data in (('.jpg', jpeg), ('.thumb.jpg', thumbnail)):
                    path = self.root / (ident + suffix)
                    # Atomic rename ensures HTTP never reads a partial JPEG.
                    temporary = path.with_suffix(path.suffix + '.tmp')
                    temporary.write_bytes(data)
                    os.replace(temporary, path)
                    paths.append(path)
                objects = metadata['objects']
                confidence = max((o['confidence'] for o in objects), default=None)
                self.db.execute('''INSERT INTO photos
                    (id,captured,frame_key,camera,objects,confidence,bytes,width,height)
                    VALUES (?,?,?,?,?,?,?,?,?)''',
                    (ident, metadata['captured'], metadata['frame_key'], metadata['camera'],
                     json.dumps(objects), confidence, size, metadata['width'], metadata['height']))
                self.db.commit()
                self.used += size
                self.count += 1
            except Exception:
                self.db.rollback()
                for path in paths:
                    path.unlink(missing_ok=True)
                raise
            return ident

    @staticmethod
    def photo(row):
        result = dict(row)
        result['objects'] = json.loads(result['objects'])
        result['protected'] = bool(result['protected'])
        result['reference'] = hashlib.sha256(result['frame_key'].encode()).hexdigest()[:8].upper()
        result['url'] = '/api/photos/' + result['id'] + '/image'
        result['thumbnail'] = '/api/photos/' + result['id'] + '/thumbnail'
        return result

    def list(self, before=None, limit=60):
        with self.lock:
            # Cursor uses timestamp + ID to remain stable if timestamps tie.
            if before:
                stamp, ident = before.split(':', 1)
                rows = self.db.execute('''SELECT * FROM photos WHERE (captured,id)<(?,?)
                    ORDER BY captured DESC,id DESC LIMIT ?''', (float(stamp), ident, limit + 1)).fetchall()
            else:
                rows = self.db.execute('SELECT * FROM photos ORDER BY captured DESC,id DESC LIMIT ?', (limit + 1,)).fetchall()
            more = len(rows) > limit
            rows = rows[:limit]
            cursor = f"{rows[-1]['captured']}:{rows[-1]['id']}" if rows and more else None
            return {'photos': [self.photo(r) for r in rows], 'next': cursor}

    def get(self, ident):
        with self.lock:
            row = self.db.execute('SELECT * FROM photos WHERE id=?', (ident,)).fetchone()
            return self.photo(row) if row else None

    def protect(self, ident, protected):
        with self.lock:
            result = self.db.execute('UPDATE photos SET protected=? WHERE id=?', (int(protected), ident))
            self.db.commit()
            return result.rowcount > 0

    def read(self, ident, thumbnail=False):
        with self.lock:
            if not self.get(ident):
                return None
            try:
                return (self.root / (ident + ('.thumb.jpg' if thumbnail else '.jpg'))).read_bytes()
            except FileNotFoundError:
                return None

    def status(self):
        with self.lock:
            return dict(count=self.count,
                        used_bytes=self._usage(), max_bytes=self.max_bytes,
                        free_bytes=shutil.disk_usage(self.root).free, reserve_bytes=self.reserve_bytes,
                        deleted=self.deleted)
