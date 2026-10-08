#!/usr/bin/env python3
"""Save a new stationary two-camera survey, without accessing motor controls."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
from pairing_dashboard import Workbench
from tag_view import detect, annotate, annotate_view


def capture(output):
    output.mkdir(parents=True, exist_ok=False)
    w = Workbench()
    w.survey_mode = True
    workers = [threading.Thread(target=w.camera_worker, args=(role,), daemon=True)
               for role in ('wrist', 'tripod')]
    record = {'captured_utc': datetime.now(timezone.utc).isoformat(),
              'motion_commanded': False, 'motion_ready': False,
              'cameras': {}, 'samples': [],
              'scope': 'Stationary camera evidence only; no workspace transforms or paths activated'}
    for worker in workers:
        worker.start()
    try:
        deadline = time.monotonic() + 30
        seen = {}
        while time.monotonic() < deadline:
            with w.lock:
                frames = dict(getattr(w, 'survey_frames', {}))
                sources = dict(w.sources)
            for role, frame in frames.items():
                source = sources.get(role, {})
                if ('error' in source or time.monotonic()-source.get('time', 0) > .5 or
                        not 0 <= source.get('frame_age_s', 99) < .5 or
                        frame['received'] <= seen.get(role, 0)):
                    continue
                seen[role] = frame['received']
                raw = frame['image']
                tags = detect(raw, 'auto', None)
                raw_name = f'{role}.png'
                view_name = f'{role}_view.jpg'
                view = annotate_view(raw, tags) if role == 'wrist' else annotate(raw, tags)
                if not cv2.imwrite(str(output/raw_name), raw) or not cv2.imwrite(str(output/view_name), view):
                    raise RuntimeError('Failed to save camera evidence')
                record['cameras'][role] = {**{k: v for k, v in frame.items() if k != 'image'},
                                           'source': source, 'image': raw_name,
                                           'view': view_name, 'tags': tags}
                record['samples'].append({'role': role, 'received': frame['received'],
                                          'frame_age_s': source['frame_age_s'],
                                          'tags': [{'family': t['family'], 'id': t['id']} for t in tags]})
            if all(sum(s['role'] == role for s in record['samples']) >= 20 for role in ('wrist', 'tripod')):
                break
            w.stop.wait(.05)
        record['sources_at_end'] = w.snapshot()['sources']
        record['both_cameras_observed'] = all(role in record['cameras'] for role in ('wrist', 'tripod'))
    finally:
        w.stop.set()
        for worker in workers:
            worker.join(timeout=12)
        record['camera_workers_stopped'] = all(not worker.is_alive() for worker in workers)
        (output/'report.json').write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'report': str(output/'report.json'),
                      'both_cameras_observed': record['both_cameras_observed'],
                      'tags': {role: c['tags'] for role, c in record['cameras'].items()},
                      'sources': record['sources_at_end']}, indent=2), flush=True)
    if not record['both_cameras_observed'] or not record['camera_workers_stopped']:
        raise RuntimeError('Stationary capture incomplete; inspect saved report')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/workspace_revalidation') /
                        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    capture(parser.parse_args().output)
