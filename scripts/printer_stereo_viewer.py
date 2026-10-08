#!/usr/bin/env python3
"""Local calibration snapshot viewer. Can request images, never robot motion.

Uses the existing camera owner's target_capture API; does not open cameras or
serial ports, restart processes, or enable/disable any motor.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import threading
import time
from urllib.request import Request, urlopen

PAGE = b'''<!doctype html><html><meta charset="utf-8"><title>Wrist stereo framing</title>
<style>body{background:#10161f;color:#eee;font:18px system-ui;margin:24px}button{font:inherit;padding:12px}section{display:flex;gap:16px}figure{margin:0;flex:1}img{width:100%;transform:rotate(180deg);background:black}p{max-width:1000px;line-height:1.5}.status{color:#f3dba2}</style>
<h1>Wrist stereo framing</h1>
<p>These are calibration snapshots from the two measuring cameras, rotated upright. Keep the full plate and tag clear of the fingers in <b>both</b> images. The color preview in the teaching window has a different viewpoint.</p>
<p>Stop and hold using the teaching window, then capture here. This page cannot move the arm, rail or gripper.</p>
<button id="capture">Capture new stereo pair</button><p id="status" class="status">Loading snapshots...</p>
<section><figure><figcaption>Camera B</figcaption><img id="B"></figure><figure><figcaption>Camera C</figcaption><img id="C"></figure></section>
<script>let stamp=null,busy=false;
document.getElementById('capture').onclick=async()=>{busy=true;try{let r=await fetch('/capture',{method:'POST',headers:{'X-Stereo-Capture':'1'}});let s=await r.json();if(!r.ok)throw Error(s.error)}catch(e){document.getElementById('status').textContent=e.message}finally{busy=false}};
async function poll(){try{let s=await(await fetch('/state')).json();document.getElementById('capture').disabled=busy||s.pending;
document.getElementById('status').textContent=s.message+(s.age_s!==null?' | Snapshot age: '+s.age_s.toFixed(1)+' s':'')+' | Snapshots, not live video';
if(s.stamp!==null&&s.stamp!==stamp){stamp=s.stamp;for(let r of ['B','C'])document.getElementById(r).src='/'+r+'.png?t='+stamp}}
catch(e){document.getElementById('status').textContent='Snapshot server unavailable'}finally{setTimeout(poll,500)}}poll();</script></html>'''


class Snapshots:
    def __init__(self, root):
        self.root = root
        self.lock = threading.RLock()
        self.images = {}
        self.stamp = None
        self.requested = None
        self.message = 'No stereo capture available'

    def refresh(self):
        reports = list(self.root.glob('stage_*/report.json'))
        if not reports:
            return
        path = max(reports, key=lambda p: p.stat().st_mtime_ns)
        try:
            report = json.loads(path.read_text())
            frame = report['frames'][-1]
            cameras = [frame['cameras'][r] for r in ('B', 'C')]
            stamps = [float(c['observation_monotonic_s']) for c in cameras]
            stamp = min(stamps)
            if not 0 <= time.monotonic()-stamp or abs(stamps[0]-stamps[1]) > .015:
                raise ValueError('Invalid stereo timestamps')
            if self.stamp is not None and stamp <= self.stamp:
                return
            images = {}
            for role, camera in zip(('B', 'C'), cameras):
                name = camera['image']
                if not re.fullmatch(r'\d{2}_'+role+r'\.png', name):
                    raise ValueError('Unexpected camera image name')
                images[role] = (path.parent/name).read_bytes()
                if not images[role].startswith(b'\x89PNG\r\n\x1a\n'):
                    raise ValueError('Incomplete image')
            self.images, self.stamp = images, stamp
            if self.requested is None or stamp > self.requested:
                self.requested = None
                self.message = 'Captured '+path.parent.name+' | Tag 18 decoded: '+', '.join(
                    role+(' yes' if any(t['id']==18 for t in c.get('tags', [])) else ' no')
                    for role, c in zip(('B', 'C'), cameras))
        except (ValueError, KeyError, IndexError, OSError):
            # Preserve the previous timestamp; never present old pixels as new.
            return

    def capture(self):
        with self.lock:
            if self.requested is not None:
                raise ValueError('Capture already pending')
            url = 'http://127.0.0.1:8765'
            page = urlopen(url+'/', timeout=2).read().decode()
            token = re.search("const token='([^']+)'", page).group(1)
            requested = time.monotonic()
            request = Request(url+'/action', data=b'{"action":"target_capture"}',
                              headers={'Content-Type':'application/json','X-Pairing-Token':token})
            with urlopen(request, timeout=2) as response:
                json.load(response)
            self.requested = requested
            self.message = 'Waiting for a new stereo pair'

    def state(self):
        with self.lock:
            self.refresh()
            if self.requested is not None and time.monotonic()-self.requested > 8:
                self.requested = None
                self.message = 'New capture timed out; displayed images are older snapshots'
            return dict(stamp=self.stamp, pending=self.requested is not None,
                        age_s=None if self.stamp is None else time.monotonic()-self.stamp,
                        message=self.message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_root', type=Path)
    args = parser.parse_args()
    root = args.capture_root.resolve()
    if not root.is_dir():
        parser.error('Use an existing teaching session target_captures directory')
    snapshots = Snapshots(root)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def reply(self, status, data, mime):
            self.send_response(status);self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store');self.end_headers();self.wfile.write(data)
        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/':self.reply(200, PAGE, 'text/html; charset=utf-8')
            elif path == '/state':self.reply(200, json.dumps(snapshots.state()).encode(), 'application/json')
            elif path in ('/B.png', '/C.png'):
                with snapshots.lock:data = snapshots.images.get(path[1])
                self.reply(200 if data else 503, data or b'No capture', 'image/png')
            else:self.reply(404, b'Not found', 'text/plain')
        def do_POST(self):
            if self.path != '/capture' or self.headers.get('X-Stereo-Capture') != '1':
                self.reply(403, b'{}', 'application/json');return
            try:
                snapshots.capture();self.reply(200, b'{"ok":true}', 'application/json')
            except Exception as error:
                self.reply(400, json.dumps({'error':str(error)}).encode(), 'application/json')
    print('Stereo framing: http://127.0.0.1:8766', flush=True)
    ThreadingHTTPServer(('127.0.0.1', 8766), Handler).serve_forever()


if __name__ == '__main__':main()
