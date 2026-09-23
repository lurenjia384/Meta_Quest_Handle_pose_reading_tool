import threading
import time

import numpy as np
import cv2


class DualCamera:

    def __init__(self, dev_indices=(4, 10), width=640, height=480, fps=15):
        self.dev_indices = dev_indices
        self._caps = [cv2.VideoCapture(d, cv2.CAP_V4L2) for d in dev_indices]
        for c in self._caps:
            if not c.isOpened():
                raise RuntimeError(f"Cannot open video device")
            c.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            c.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            c.set(cv2.CAP_PROP_FPS, fps)

        self._lock = threading.Lock()
        self._frames = [None, None]
        self._running = False
        self._thread = None

    def _loop(self):
        for _ in range(5):
            for c in self._caps:
                c.read()
        while self._running:
            frames = []
            ok = True
            for c in self._caps:
                ret, f = c.read()
                if not ret:
                    ok = False
                    break
                frames.append(f)
            if ok:
                with self._lock:
                    self._frames = frames

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def read(self):
        """Return the latest synchronized (frame0, frame1) pair or (None, None)."""
        with self._lock:
            return self._frames[0], self._frames[1]

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        for c in self._caps:
            c.release()


def main():
    cam = DualCamera()
    cam.start()
    t0 = time.time()
    n = 0
    while time.time() - t0 < 3:
        f0, f1 = cam.read()
        if f0 is not None and f1 is not None:
            n += 1
    cam.stop()
    print(f"DualCamera: {n/3:.1f} fps, frames {f0.shape}")


if __name__ == "__main__":
    main()
