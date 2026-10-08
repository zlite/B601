#!/usr/bin/python3
"""Read-only GRBL identity/settings snapshot on the separate rail USB path."""
import json
from pathlib import Path
import time
from datetime import datetime, timezone
import serial

PORT='/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.2:1.0-port0'


def snapshot():
    port=serial.Serial(port=None,baudrate=115200,timeout=.05,write_timeout=.5,exclusive=True)
    port.dtr=False;port.rts=False;port.port=PORT
    rows=[]
    def read_for(seconds,ack=False):
        lines=[];deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            raw=port.readline()
            if raw:
                line=raw.decode('ascii',errors='replace').strip();lines.append(line)
                if ack and (line=='ok' or line.startswith(('error:','ALARM:'))):break
        if ack and (not lines or lines[-1]!='ok'):
            raise RuntimeError('GRBL query failed: '+repr(lines))
        return lines
    try:
        port.open()
        rows.append({'startup':read_for(3.5)})
        port.write(b'\n');rows.append({'empty_line':read_for(.3)})
        port.write(b'$I\n');identity=read_for(2,True)
        if not any(line.startswith('[VER:1.1') for line in identity):
            raise RuntimeError('Expected GRBL 1.1 identity; refusing this port')
        rows.append({'query':'$I','lines':identity})
        for command in ('?','$$','$G','$#','?'):
            port.write(command.encode()+ (b'' if command=='?' else b'\n'))
            rows.append({'query':command,'lines':read_for(.4 if command=='?' else 2,command!='?')})
    finally:
        port.close()
    return {'utc':datetime.now(timezone.utc).isoformat(),'port':PORT,'motion_commanded':False,'rows':rows}


if __name__=='__main__':
    result=snapshot();folder=Path(__file__).resolve().parents[1]/'outputs/rail'
    folder.mkdir(parents=True,exist_ok=True)
    path=folder/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'_status.json')
    path.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'file':str(path),**result},indent=2))
