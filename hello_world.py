#!/usr/bin/env python3
"""A small wrist wave with live OAK-1 RGB frames. Calibration is never written."""
from __future__ import annotations

import argparse
import math
import signal
import time
from contextlib import ExitStack
from pathlib import Path
from camera_selection import wrist_pipeline, build_wrist_rgb, start_wrist_pipeline, wrist_camera_config
from camera_view import upright

PORT = '/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00'


def wave_offset(elapsed: float, amplitude: float, period: float, cycles: int) -> float:
    """Cosine lobes alternate sides; position and velocity start/end at zero."""
    if elapsed <= 0 or elapsed >= period * cycles:
        return 0.0
    half = period / 2
    lobe = int(elapsed / half)
    phase = (elapsed % half) / half
    return (-1 if lobe % 2 else 1) * amplitude * (1 - math.cos(2 * math.pi * phase)) / 2


class Wrist:
    """Register only joint6, so enable/disable cannot affect the other joints."""
    def __init__(self, port: str):
        from motorbridge import Controller
        self.ctrl = Controller.from_dm_serial(port, 921600)
        self.enabled = False
        try:
            self.motor = self.ctrl.add_damiao_motor(0x06, 0x16, '4310')
            self.origin = self.state().pos
        except BaseException:
            self.ctrl.close()
            raise

    def state(self):
        for _ in range(20):
            self.motor.request_feedback()
            time.sleep(0.005)
            self.ctrl.poll_feedback_once()
            state = self.motor.get_state()
            if state is not None:
                if not all(math.isfinite(v) for v in (state.pos, state.vel, state.torq)):
                    raise RuntimeError('Invalid wrist feedback')
                if state.status_code not in (0, 1):
                    raise RuntimeError(f'Wrist fault: status {state.status_code}')
                if max(state.t_mos, state.t_rotor) > 60:
                    raise RuntimeError('Wrist temperature exceeds 60 C')
                return state
        raise RuntimeError('No wrist feedback; check arm power and serial connection')

    def start(self, amplitude: float):
        from motorbridge import Mode
        state = self.state()
        if state.status_code != 0:
            raise RuntimeError('Wrist is already enabled; stop its existing controller first')
        self.origin = state.pos
        if abs(self.origin) + amplitude > 3.04:
            raise RuntimeError('Wave would approach the joint6 URDF limit of ±3.14 radians')
        self.motor.ensure_mode(Mode.MIT, 1000)
        self.enabled = True  # ensure cleanup even if enable partially succeeds
        self.motor.enable()
        self.command(self.origin)

    def command(self, target: float):
        # Seeed DM joint6 MIT gains; target changes smoothly at 50 Hz.
        self.motor.send_mit(target, 0.0, 18.0, 2.0, 0.0)

    def close(self):
        try:
            if self.enabled:
                self.motor.disable()
                print('Wrist disabled; other joints were not commanded.')
        finally:
            self.ctrl.close()


class Camera:
    def __init__(self, stack: ExitStack, preview: bool, output: Path, *, output_raw: bool = False):
        import cv2
        import depthai as dai
        self.cv2, self.preview, self.output = cv2, preview, output
        self.output_raw = output_raw
        self.pipeline = stack.enter_context(wrist_pipeline())
        cam = build_wrist_rgb(self.pipeline)
        self.queue = cam.requestOutput((640, 400), type=dai.ImgFrame.Type.BGR888p, fps=15).createOutputQueue(maxSize=2, blocking=False)
        start_wrist_pipeline(self.pipeline, cam)
        self.focus = wrist_camera_config()['manual_focus']
        self.last = time.monotonic()
        self.count = 0
        self.frame = None
        stack.callback(self.finish)
        deadline = time.monotonic() + 10
        while self.frame is None:
            self.poll(timeout=10)
            if time.monotonic() > deadline:
                raise RuntimeError('OAK did not produce a frame within 10 seconds')
            time.sleep(0.01)
        print('Wrist RGB stream ready: 640×400 at 15 FPS')

    def poll(self, timeout=2):
        packet = self.queue.tryGet()
        if packet is not None and packet.getLensPosition() != self.focus:
            packet = None
        if packet is not None:
            self.frame = packet.getCvFrame()
            self.count += 1
            self.last = time.monotonic()
            if self.preview:
                self.cv2.imshow('B601 wrist camera (Q to stop)', upright(self.frame))
        if time.monotonic() - self.last > timeout:
            raise RuntimeError('OAK frames stopped arriving')
        if self.preview and self.cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            raise KeyboardInterrupt

    def finish(self):
        if self.frame is not None:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            image = self.frame if self.output_raw else upright(self.frame)
            if not self.cv2.imwrite(str(self.output), image):
                raise RuntimeError(f'Cannot write snapshot: {self.output}')
            print(f'Read {self.count} frames; saved {self.output}')
        if self.preview:
            self.cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['wave', 'camera', 'probe', 'dry-run'], default='wave')
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--amplitude', type=float, default=8, help='Wrist excursion in degrees (0–10)')
    parser.add_argument('--period', type=float, default=6, help='Seconds per full wave (minimum 6)')
    parser.add_argument('--cycles', type=int, default=2, help='Wave cycles (1–5)')
    parser.add_argument('--preview', action='store_true', help='Open a live camera window')
    parser.add_argument('--output', type=Path, default=Path('outputs/hello.jpg'))
    args = parser.parse_args()
    if not (0 < args.amplitude <= 10 and 6 <= args.period <= 30 and 1 <= args.cycles <= 5):
        parser.error('Use amplitude >0 and ≤10°, period 6–30 seconds, and cycles 1–5')
    amplitude = math.radians(args.amplitude)
    duration = args.period * args.cycles
    if args.mode == 'dry-run':
        for i in range(9):
            t = i * args.period / 8
            print(f'{t:5.2f}s: wrist offset {math.degrees(wave_offset(t, amplitude, args.period, args.cycles)):+6.2f}°')
        print(f'{args.cycles} cycles / {duration:g}s; no hardware accessed')
        return
    with ExitStack() as stack:
        if args.mode == 'probe':
            wrist = Wrist(args.port)
            stack.callback(wrist.close)
            print(wrist.state())
            return
        camera = Camera(stack, args.preview, args.output)
        wrist = None
        if args.mode == 'wave':
            wrist = Wrist(args.port)
            stack.callback(wrist.close)
            print(f'Wrist wave ±{args.amplitude:g}°; keep wrist and camera cable clear. Starting in 3 seconds.')
            for _ in range(150):
                camera.poll()
                time.sleep(0.02)
            wrist.start(amplitude)
        start = time.monotonic()
        next_feedback = start
        while True:
            now = time.monotonic()
            elapsed = now - start
            if wrist:
                target = wrist.origin + wave_offset(min(elapsed, duration), amplitude, args.period, args.cycles)
                wrist.command(target)
                if now >= next_feedback:
                    state = wrist.state()
                    if state.status_code != 1:
                        raise RuntimeError('Wrist unexpectedly disabled')
                    if abs(state.pos - target) > math.radians(15):
                        raise RuntimeError('Wrist tracking error exceeds 15°')
                    next_feedback = now + 0.1
            camera.poll()
            if elapsed >= duration + (1 if wrist else 0):
                break
            time.sleep(0.02)
        print('Hello, world!')


if __name__ == '__main__':
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        main()
    except KeyboardInterrupt:
        print('\nStopped.')
        raise SystemExit(130)
    except Exception as error:
        print(f'Error: {error}')
        raise SystemExit(1)
