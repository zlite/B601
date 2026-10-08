"""Apply the operator-requested 32 mm/s rail profile while stationary.

No movement, unlock, homing, reset, or changes to idle power-off/limits/scale.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rail_jog import PORT, RailJog, parse_status, validate_settings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', required=True)
    parser.parse_args()
    folder = Path(__file__).resolve().parents[1] / 'outputs/rail'
    folder.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    path = folder / (stamp + '_speed_profile.json')
    report = {'port': PORT, 'motion_commanded': False, 'writes': []}
    port = serial.Serial(port=None, baudrate=115200, timeout=.05,
                         write_timeout=.5, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = PORT
    query = lambda command: RailJog.query(None, port, command)

    def settings():
        return {int(x[1:].split('=')[0]): float(x.split('=')[1])
                for x in query('$$') if x.startswith('$')}

    def idle():
        port.write(b'?')
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            line = port.readline().decode('ascii', errors='replace').strip()
            if line.startswith('<'):
                state = parse_status(line)
                if state['state'] != 'Idle' or state['pins']:
                    raise RuntimeError('Profile update requires Idle and clear limit inputs: ' + line)
                return state
            if line.startswith(('ALARM:', 'error:', 'Grbl')):
                raise RuntimeError('Unexpected controller state: ' + line)
        raise RuntimeError('No fresh rail status')

    try:
        port.open()
        deadline = time.monotonic() + 3.5
        while time.monotonic() < deadline:
            port.readline()
        query('')
        identity = query('$I')
        if not any(x.startswith('[VER:1.1') for x in identity):
            raise RuntimeError('Expected the verified GRBL 1.1 rail controller')
        report['identity'] = identity
        before = settings()
        report['before'] = before
        if (before.get(110), before.get(120)) not in ((960., 20.), (1920., 40.)):
            raise RuntimeError('Unexpected existing speed/acceleration; refusing update')
        intended = {**before, 110: 1920., 120: 40.}
        validate_settings(intended)
        report['initial_status'] = idle()
        path.write_text(json.dumps(report, indent=2) + '\n')
        for key in (110, 120):
            if before[key] == intended[key]:
                continue
            idle()
            command = f'${key}={intended[key]:g}'
            query(command)
            report['writes'].append(command)
            path.write_text(json.dumps(report, indent=2) + '\n')
        report['after'] = settings()
        if report['after'] != intended:
            raise RuntimeError('Settings readback did not match the exact intended profile')
        report['final_status'] = idle()
        report['verified'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        port.close()
        path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({'file': str(path), **report}, indent=2))


if __name__ == '__main__':
    main()
