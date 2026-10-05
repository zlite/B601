#!/usr/bin/env python3
"""Exercise one stationary starting-view capture; never request motor actions."""
import json
from pathlib import Path
import re
import time
from urllib.request import Request, urlopen

URL = 'http://127.0.0.1:8765'


def state():
    with urlopen(URL+'/state', timeout=3) as response:
        return json.load(response)


def main():
    before = state()
    if before.get('powered') is not False or before.get('fault'):
        raise RuntimeError('This capture test requires motors already off and no controller fault')
    if before['camera_check']['step'] != 0:
        raise RuntimeError('Starting view already saved; preserving the existing check')
    with urlopen(URL+'/', timeout=3) as response:
        match = re.search(r"const token='([^']+)'", response.read().decode())
    if match is None:
        raise RuntimeError('Dashboard token missing')
    request = Request(URL+'/action', data=json.dumps({'action':'camera_capture'}).encode(),
                      headers={'Content-Type':'application/json', 'X-Pairing-Token':match[1]})
    began = time.monotonic()
    with urlopen(request, timeout=3) as response:
        response.read()
    while time.monotonic()-began < 12:
        current = state()
        if current['camera_check']['step'] == 1:
            result = {'saved':True, 'elapsed_s':time.monotonic()-began,
                      'powered':current['powered'], 'camera_check':current['camera_check']}
            path = Path(__file__).resolve().parents[1]/'outputs/camera_diagnosis/live_capture_test.json'
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result,indent=2))
            return
        time.sleep(.1)
    raise RuntimeError('Capture did not save: '+current['camera_check']['message'])


if __name__ == '__main__':
    main()
