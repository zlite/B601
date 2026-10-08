"""Stationary GRBL sleep/enable diagnostic. Keep motor supply OFF.

Never sends movement or unlock commands. Keeps the serial connection open
after $SLP so a meter reading is not disturbed by USB reconnect/reset.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import serial

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rail_jog import PORT, RailJog, parse_status, validate_settings


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sleep-test',action='store_true',required=True)
    parser.parse_args()
    directory=Path(__file__).resolve().parents[1]/'outputs/rail'
    directory.mkdir(parents=True,exist_ok=True)
    filename=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'_idle_probe.json'
    path=directory/filename
    report={'port':PORT,'motion_commanded':False,'commands':[]}
    port=serial.Serial(port=None,baudrate=115200,timeout=.05,write_timeout=.5,exclusive=True)
    port.dtr=False;port.rts=False;port.port=PORT

    def query(command):
        report['commands'].append(command)
        return RailJog.query(None,port,command)

    def status():
        port.write(b'?');deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            line=port.readline().decode('ascii',errors='replace').strip()
            if line.startswith('<'):return parse_status(line)
            if line.startswith(('ALARM:','error:','Grbl')):
                raise RuntimeError(line)
        raise RuntimeError('No fresh status from rail')

    try:
        port.open();deadline=time.monotonic()+3.5
        while time.monotonic()<deadline:port.readline()
        query('')
        report['identity']=query('$I')
        if not any(x.startswith('[VER:1.1') for x in report['identity']):
            raise RuntimeError('Unexpected rail identity')
        settings={int(x[1:].split('=')[0]):float(x.split('=')[1]) for x in query('$$') if x.startswith('$')}
        report['settings']=settings
        validate_settings(settings)
        if settings.get(4)!=0.:raise RuntimeError('Unexpected enable inversion; not running test')
        report['before']=status()
        if report['before']['state']!='Idle' or report['before']['pins']:
            raise RuntimeError('Sleep diagnostic requires Idle and clear limit inputs')
        query('$SLP')
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            report['after']=status()
            if report['after']['state']=='Sleep':break
        else:raise RuntimeError('GRBL did not enter Sleep')
        path.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({'report':str(path),'state':report['after'],
                          'idle_delay':settings[1],'enable_inversion':settings[4],
                          'instruction':'Motor supply OFF; measure EN/GND now. Serial stays open; no motion is possible in Sleep.'}),flush=True)
        while True:
            time.sleep(.25)
            current=status()
            if current['state']!='Sleep':raise RuntimeError('Sleep state changed: '+str(current))
    except KeyboardInterrupt:
        report['closed_by_operator']=True
    except Exception as error:
        report['error']=str(error)
        raise
    finally:
        port.close()
        path.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
