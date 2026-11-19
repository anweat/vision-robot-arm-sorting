"""Optional camera worker; HTTP snapshot, RTSP via ffmpeg, or explicit still replay."""
import os
import math
import subprocess
import threading
import time
import urllib.request
from pathlib import Path


class Camera:
    def __init__(self, source=None, replay=None):
        self.source = source
        self.replay = replay
        self.lock = threading.Lock()
        self.jpeg = None
        self.stamp = 0
        self.received_at = 0
        self.seq = 0
        self.error = ''
        self.clock = 'none'
        self.process = None
        self.closed = threading.Event()
        if replay:
            self.jpeg = Path(replay).read_bytes()
            if not self.jpeg.startswith(b'\xff\xd8'):
                raise ValueError('回放图必须是 JPEG')
            self.seq, self.clock = 1, 'replay'
        elif source:
            if not source.startswith(('http://', 'https://', 'rtsp://')):
                raise ValueError('只支持 HTTP JPEG 快照或 RTSP')
            threading.Thread(target=self.run, daemon=True).start()

    def publish(self, jpeg, stamp, clock):
        if not math.isfinite(stamp):
            raise ValueError('invalid frame timestamp')
        if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
            raise ValueError('camera returned invalid JPEG')
        with self.lock:
            self.jpeg, self.stamp, self.clock = jpeg, stamp, clock
            self.received_at, self.seq, self.error = time.time(), self.seq+1, ''

    def fresh(self):
        with self.lock:
            # Hardware needs the board capture clock, not just decoder arrival time.
            return self.clock == 'capture' and 0 <= time.time()-self.stamp < 2 and not self.error

    def info(self):
        with self.lock:
            return self._info()

    def _info(self):
        return dict(kind='replay' if self.replay else ('live' if self.source else 'none'),
                    seq=self.seq, stamp=self.stamp, received_at=self.received_at, replay_file=str(self.replay) if self.replay else None,
                    clock=self.clock, age_s=time.time()-self.stamp if self.stamp else None,
                    error=self.error, hardware_fresh=self.clock == 'capture' and 0 <= time.time()-self.stamp < 2 and not self.error)

    def snapshot(self):
        with self.lock:
            return self.jpeg, self._info()

    def run(self):
        while not self.closed.is_set():
            try:
                if self.source.startswith('rtsp://'):
                    self.rtsp()
                else:
                    # Explicit configured source; bypass ambient proxy for the local board.
                    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
                    with opener.open(self.source, timeout=2) as r:
                        raw = r.read(8*1024*1024+1)
                        if len(raw) > 8*1024*1024:
                            raise ValueError('image too large')
                        value = r.headers.get('X-Frame-Time')
                        self.publish(raw, float(value) if value else time.time(), 'capture' if value else 'receive')
                    self.closed.wait(.15)
            except Exception:
                with self.lock:
                    self.error = '相机读取失败；检查相机连接、地址或 ffmpeg。地址未写入日志。'
                self.closed.wait(1)

    def rtsp(self):
        # Credentials remain local; stderr is never surfaced or saved.
        args = ['ffmpeg','-nostdin','-loglevel','error','-rtsp_transport','tcp','-rw_timeout','3000000',
                '-i',self.source,'-an','-vf','fps=5','-f','image2pipe','-vcodec','mjpeg','pipe:1']
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.process = p
        buf = b''
        try:
            while not self.closed.is_set():
                chunk = os.read(p.stdout.fileno(), 65536)
                if not chunk:
                    raise ValueError('RTSP ended')
                buf += chunk
                if len(buf) > 8*1024*1024:
                    raise ValueError('oversize frame')
                while b'\xff\xd9' in buf:
                    frame, buf = buf.split(b'\xff\xd9',1)
                    start = frame.find(b'\xff\xd8')
                    if start >= 0:
                        self.publish(frame[start:]+b'\xff\xd9', time.time(), 'receive')
        finally:
            p.terminate()
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
            p.stdout.close()

    def close(self):
        self.closed.set()
        if self.process and self.process.poll() is None:
            self.process.terminate()
