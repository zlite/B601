"""Attended local Demo API driver. Does not automatically resume a pause."""
import argparse,json,re,time,uuid
from urllib.request import urlopen,Request
from pathlib import Path
URL='http://127.0.0.1:8765'
def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['start','pause','resume','return','stop','recover'])
    p.add_argument('--once',action='store_true',help='Finish the first cycle, then stop at rest')
    args=p.parse_args()
    page=urlopen(URL+'/',timeout=3).read().decode();token=re.search("const token='([^']+)'",page).group(1)
    client='agent-demo-'+str(uuid.uuid4());seq=0
    def send(action,**extra):
        nonlocal seq
        seq+=1
        request=Request(URL+'/action',data=json.dumps(dict(action='demo_'+action,client=client,seq=seq,**extra)).encode(),headers={'Content-Type':'application/json','X-Pairing-Token':token})
        return json.load(urlopen(request,timeout=2))
    send(args.action,rest_confirmed=True,scene_unchanged=True,rail_clear_confirmed=True,clearance_confirmed=True)
    if args.action in ('pause','stop'):return
    previous=None;next_state=0;stop_sent=False
    try:
        while True:
            send('heartbeat')
            if time.monotonic()>=next_state:
                s=json.load(urlopen(URL+'/state',timeout=2));d=s['demo'];phase=d['phase']
                if args.once and d['active'] and d['cycle']>=1 and not stop_sent:
                    send('stop');stop_sent=True
                if phase!=previous:
                    print(json.dumps({'demo':{k:v for k,v in d.items() if k!='last_sample'},'powered':s['powered'],'fault':s['fault']}),flush=True);previous=phase
                if not d['active']:return
                next_state=time.monotonic()+.5
            time.sleep(.1)
    finally:
        send('pause')
if __name__=='__main__':main()
