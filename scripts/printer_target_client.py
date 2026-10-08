#!/usr/bin/env python3
"""Read-only target capture/preview API. Cannot command robot or rail motion."""
import argparse
import json
import re
from urllib.request import Request, urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'capture', 'preview'])
    args = parser.parse_args()
    url = 'http://127.0.0.1:8765'
    if args.action != 'status':
        page = urlopen(url+'/', timeout=3).read().decode()
        token = re.search("const token='([^']+)'", page).group(1)
        request = Request(url+'/action', data=json.dumps(dict(action='target_'+args.action)).encode(),
                          headers={'Content-Type':'application/json', 'X-Pairing-Token':token})
        with urlopen(request, timeout=3) as response:
            print(response.read().decode())
    with urlopen(url+'/state', timeout=3) as response:
        s = json.load(response)
    print(json.dumps(dict(target_preview=s.get('target_preview'),
                          plate=s['sources'].get('printer_plate'),
                          bed=s['sources'].get('printer_target'),
                          powered=s['powered'], fault=s['fault']), indent=2))


if __name__ == '__main__': main()
