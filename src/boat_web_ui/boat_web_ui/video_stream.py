"""One bounded JPEG cache per source, shared by all HTTP viewers."""
import threading
import time


class SharedVideo:
    def __init__(self, encoder, stale_jpeg, max_hz=10., stale_sec=2., clock=time.monotonic):
        self.encoder, self.stale_jpeg = encoder, stale_jpeg
        self.period, self.stale_sec, self.clock = 1/max_hz, stale_sec, clock
        self.lock = threading.Lock()
        self.frame = None
        self.received = None
        self.sequence = 0
        self.encoded_sequence = -1
        self.encoded_at = -float('inf')
        self.jpeg = None

    def put(self, frame):
        with self.lock:
            self.frame = frame
            self.received = self.clock()
            self.sequence += 1

    def get(self):
        with self.lock:
            now = self.clock()
            if self.received is None or now-self.received > self.stale_sec:
                return self.stale_jpeg
            if self.sequence != self.encoded_sequence and now-self.encoded_at >= self.period:
                self.encoded_at = now
                self.encoded_sequence = self.sequence
                try:
                    self.jpeg = self.encoder(self.frame)
                except Exception:
                    self.jpeg = None
            return self.jpeg or self.stale_jpeg

    def stream(self):
        previous = None
        while True:
            jpeg = self.get()
            if jpeg is not previous:
                previous = jpeg
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+jpeg+b'\r\n'
            time.sleep(self.period)
