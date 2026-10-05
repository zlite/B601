#!/usr/bin/env python3
"""Read-only plate experiment history. Never opens cameras or motor adapters."""
import argparse
import json
import mimetypes
import re
import time
from datetime import datetime, timezone
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

ROOT = Path(__file__).resolve().parent
RUNS = ROOT / 'outputs' / 'plate_hover'


@lru_cache(maxsize=256)
def cached_json(path, modified, size):
    try:
        value = Path(path).read_text()
        result = json.loads(value)
        return result if isinstance(result, dict) else {}
    except (OSError, ValueError):
        return {}


def read_json(path):
    try:
        stat = path.stat()
        return cached_json(str(path), stat.st_mtime_ns, stat.st_size)
    except OSError:
        return {}


def run_summary(folder):
    report = read_json(folder / 'report.json')
    pickup = read_json(folder / 'pickup_report.json') or read_json(folder / 'pickup_progress.json')
    carry = read_json(folder / 'table_carry.json') or pickup.get('transfer', {})
    recovery = read_json(folder / 'recovery.json')
    fault = read_json(folder / 'fault_hold.json')
    lower = read_json(folder / 'lower_report.json') or read_json(folder / 'lower_progress.json')
    progress = report or read_json(folder / 'progress.json')
    shutdown = read_json(folder / 'shutdown_verification.json')
    finished = (folder / 'report.json').exists()
    off = report.get('motors_disabled_verified') is True or shutdown.get('motors_disabled_verified') is True
    returned = report.get('returned_to_rest') is True
    lift = pickup.get('lift_following_verified') is True
    released = pickup.get('released_verified') is True
    injected = recovery.get('injected_grid_loss_after_clearance') is True
    assisted = shutdown.get('physically_supported_shutdown') is True
    full_cycle = lift and released and returned and off and not assisted
    carry_passed = bool(full_cycle and carry.get('destination_reached') and carry.get('returned_to_source'))
    if not finished:
        status = 'Powered hold' if fault.get('holding') else 'Recovering' if fault.get('recovering') else 'In progress'
    elif carry_passed:
        status = 'Carry and holder return passed'
    elif full_cycle and carry:
        status = 'Carry stopped; holder return passed'
    elif full_cycle:
        status = 'Cycle passed'
    elif assisted and off:
        status = 'Stopped; supported shutdown'
    elif injected and returned and off:
        status = 'Recovery test passed'
    elif lift:
        status = 'Lift passed; cycle incomplete'
    elif returned and off:
        status = 'Returned without pickup'
    else:
        status = 'Stopped / power unverified'
    reasons = fault.get('causes', [])[:]
    for obj in (pickup, lower, report, recovery, carry):
        reason = obj.get('stop_reason')
        if reason and reason not in reasons:
            reasons.append(reason)
    views = progress.get('views', [])
    rows = lower.get('rows', [])
    samples = report.get('motion_samples', [])
    duration = samples[-1]['time'] - samples[0]['time'] if len(samples) > 1 else None
    start = datetime.strptime(folder.name, '%Y%m%dT%H%M%S%fZ').replace(tzinfo=timezone.utc)
    return {
        'id': folder.name, 'started': start.isoformat(), 'status': status,
        'finished': finished, 'motors_off_verified': off, 'returned': returned,
        'lift_verified': lift, 'release_verified': released, 'cycle_passed': full_cycle,
        'holder_cycle_passed': full_cycle and not bool(carry), 'carry_passed': carry_passed,
        'table_release_verified': False,
        'lift_height_mm': pickup.get('model_lift_height_m', 0) * 1000 or None,
        'requested_lift_mm': pickup.get('test_lift_m', 0) * 1000 or None,
        'plate_camera_change_mm': pickup.get('plate_motion_relative_camera_m', 0) * 1000 if lift else None,
        'grid_loss_test_passed': injected and returned and off,
        'speed_scale': progress.get('speed_scale'), 'approach': progress.get('approach', 'legacy'), 'duration_s': duration,
        'phase': views[-1]['label'] if views else 'Starting',
        'approach_gap_mm': min(tip[2] for tip in rows[-1]['tips_in_plate_m']) * 1000 if rows else None,
        'reasons': reasons, 'views': [{'label': v['label']} for v in views],
    }


def camera_image(folder, role, finished):
    # Recordings contain timestamped frames even when a fault report is stale.
    candidates = list((folder / 'recording').glob('*_' + role + '.jpg'))
    candidates += list(folder.glob('*_' + role + '.png'))
    if not finished and (folder / ('fault_' + role + '.jpg')).exists():
        candidates.append(folder / ('fault_' + role + '.jpg'))
    if not candidates:
        return None
    path = max(candidates, key=lambda p: p.stat().st_mtime_ns)
    return {'url': '/files/' + path.relative_to(RUNS).as_posix(),
            'recorded_at': path.stat().st_mtime, 'role': role}


def snapshot(selected=None):
    folders = sorted((p for p in RUNS.iterdir() if p.is_dir() and
                      re.fullmatch(r'\d{8}T\d{12}Z', p.name)), reverse=True)
    runs = [run_summary(p) for p in folders]
    chosen = next((r for r in runs if r['id'] == selected), runs[0] if runs else None)
    detail = dict(chosen) if chosen else None
    if detail:
        folder = RUNS / detail['id']
        detail['cameras'] = [camera_image(folder, role, detail['finished']) for role in ('wrist', 'tripod')]
        detail['reports'] = [name for name in ('report.json', 'pickup_report.json', 'recovery.json', 'lower_report.json', 'fault_hold.json', 'shutdown_verification.json', 'table_survey.json', 'table_carry.json') if (folder / name).exists()]
    return {'updated_at': time.time(), 'runs': runs, 'selected': detail,
            'milestones': {'lift_20mm': any(r['lift_verified'] and (r['lift_height_mm'] or 0) >= 19 for r in runs),
                           'lost_grid_return': any(r['grid_loss_test_passed'] for r in runs),
                           'cycle_20mm': any(r['cycle_passed'] and (r['lift_height_mm'] or 0) >= 19 for r in runs)}}


HTML = r'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>REbot · Experiment progress</title><style>
:root{color-scheme:dark;font:15px system-ui;background:#10181d;color:#edf4f5}*{box-sizing:border-box}body{margin:0 auto;max-width:1440px;padding:28px}h1{font-size:30px;margin:4px 0 10px}h2{font-size:18px;margin:0 0 14px}.muted{color:#a4bac4}header{display:flex;justify-content:space-between;gap:20px;margin-bottom:24px}.tag{font-size:12px;text-transform:uppercase;letter-spacing:2px;color:#6fdbbd}.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin-bottom:22px}.card,section{background:#19262e;border:1px solid #30424b;border-radius:12px;padding:18px}.card strong{display:block;margin-top:9px;font-size:22px}.good{color:#71dfbd}.pending{color:#f2c779}.layout{display:grid;grid-template-columns:minmax(280px,360px) 1fr;gap:18px}.history{max-height:790px;overflow:auto}.run{display:block;width:100%;text-align:left;padding:14px;margin:0 0 9px;border:1px solid #394b55;border-radius:8px;background:#111d25;color:inherit;cursor:pointer}.run.active{border-color:#71dfbd;background:#223b40}.run small{display:block;color:#a4bac4;margin-top:5px}.detailhead{display:flex;justify-content:space-between;gap:10px}.facts{display:flex;flex-wrap:wrap;gap:10px;margin:16px 0}.fact{background:#101b23;padding:10px 13px;border-radius:8px}.fact b{display:block;margin-top:5px}.cameras{display:grid;grid-template-columns:1fr 1fr;gap:12px}.camera{min-width:0;background:#0d151a;border-radius:8px;padding:8px}.camera img{width:100%;height:260px;object-fit:contain}.camera small{display:block;color:#a4bac4;margin:8px}.camera img.wrist{transform:rotate(180deg)}.camera img.tripod{transform:rotate(-90deg) scale(.65);object-fit:contain}ul{padding-left:21px;line-height:1.6}a{color:#8ce9d0}details{margin-top:18px}summary{cursor:pointer}#connection{font-size:13px}#stages{display:flex;gap:8px;flex-wrap:wrap;margin:15px 0}.stage{font-size:12px;border:1px solid #405561;border-radius:20px;padding:5px 10px}#errors{word-break:break-word;font-size:13px}footer{margin-top:20px;font-size:13px;color:#a4bac4}@media(max-width:850px){body{padding:16px}.layout{grid-template-columns:1fr}.history{max-height:250px}.cameras{grid-template-columns:1fr}.cards{grid-template-columns:1fr}.camera img{height:300px}}
</style><header><div><div class="tag">REbot B601 · Lab notebook</div><h1>Experiment progress</h1><div class="muted">Recorded calibration and plate-handling tests. Read-only: no robot controls.</div></div><div id="connection">Connecting…</div></header>
<div class="cards"><div class="card">Approx. 20 mm lift<strong id="lift">Checking</strong></div><div class="card">Return with well tracking lost<strong id="recovery">Checking</strong></div><div class="card">Complete 20 mm cycle<strong id="cycle">Checking</strong></div></div>
<p id="series" class="muted"></p><div class="layout"><section><h2>Run history</h2><div class="history" id="history"></div></section><section><div class="detailhead"><div><h2 id="title">Loading…</h2><div class="muted" id="when"></div></div><span id="phase"></span></div><div class="facts" id="facts"></div><div class="cameras" id="cameras"></div><div id="stages"></div><details open><summary>Stops and recovery notes</summary><ul id="errors"></ul></details><details><summary>Raw reports</summary><ul id="reports"></ul></details></section></div>
<footer>Heights are estimates from joint readback; plate following uses the wrist camera. A passed lift is not a passed full cycle. Images are recorded snapshots, with timestamps; this page does not read motor hardware.</footer>
<script>
let selected=new URLSearchParams(location.search).get('run'),lastHistory='',lastImageRun='',selectedPhase=null;
const $=id=>document.getElementById(id),el=(tag,text,cls)=>{let n=document.createElement(tag);if(text!==undefined)n.textContent=text;if(cls)n.className=cls;return n};
function fact(label,value){let n=el('div',label,'fact');n.append(el('b',value));return n}
function milestone(id,passed){$(id).textContent=passed?'Passed':'Not yet passed';$(id).className=passed?'good':'pending'}
async function refresh(){try{let res=await fetch('/api/progress'+(selected?'?run='+encodeURIComponent(selected):''),{cache:'no-store'});if(!res.ok)throw Error('HTTP '+res.status);let data=await res.json();$('connection').textContent='Updated '+new Date(data.updated_at*1000).toLocaleTimeString();$('connection').className='good';milestone('lift',data.milestones.lift_20mm);milestone('recovery',data.milestones.lost_grid_return);milestone('cycle',data.milestones.cycle_20mm);let series=data.runs.filter(x=>x.id>='20261005T190906767335Z'&&x.approach!=='legacy'&&x.requested_lift_mm>=19);let verified=series.filter(x=>x.holder_cycle_passed&&x.lift_height_mm>=19);let counts=['direct','left','right'].map(a=>a+': '+verified.filter(x=>x.approach===a).length);$('series').textContent=verified.length+'/10 verified holder cycles before table transfer — '+counts.join(' · ')+' · '+series.filter(x=>x.finished&&!x.cycle_passed).length+' incomplete attempts';let r=data.selected;if(!r)return;
let historyKey=JSON.stringify(data.runs.map(x=>[x.id,x.status]))+r.id;if(historyKey!==lastHistory){lastHistory=historyKey;$('history').replaceChildren(...data.runs.map(x=>{let b=el('button',x.status,'run'+(x.id===r.id?' active':''));b.append(el('small',new Date(x.started).toLocaleString()));b.onclick=()=>{selected=x.id;selectedPhase=null;history.replaceState(null,'','?run='+selected);refresh()};return b}))}
$('title').textContent=r.status;$('when').textContent=new Date(r.started).toLocaleString()+' · '+r.id;$('phase').textContent=(r.finished?'Last phase: ':'Phase: ')+r.phase;
$('facts').replaceChildren(fact('Requested lift',r.requested_lift_mm?r.requested_lift_mm.toFixed(0)+' mm':'—'),fact('Measured lift estimate',r.lift_height_mm?r.lift_height_mm.toFixed(1)+' mm':'—'),fact('Return to rest',r.returned?'Verified':'Not verified'),fact('Motor power at run end',r.motors_off_verified?'Off verified':r.finished?'Unverified':'Run active'),fact('Approach',r.approach),fact('Motion scale',r.speed_scale?r.speed_scale+'×':'—'),fact('Powered recording',r.duration_s?Math.round(r.duration_s)+' s':'—'));
if(lastImageRun!==r.id){$('cameras').replaceChildren();lastImageRun=r.id}for(let i=0;i<r.cameras.length;i++){let c=r.cameras[i];if(!c)continue;if(selectedPhase)c={...c,url:'/files/'+r.id+'/'+selectedPhase+'_'+c.role+'.png'};let box=$('cameras').children[i];if(!box){box=el('div',undefined,'camera');box.append(el('img'),el('small'));$('cameras').append(box)}let img=box.children[0],src=c.url+'?v='+c.recorded_at;if(img.getAttribute('src')!==src)img.setAttribute('src',src);img.className=c.role;img.alt=c.role+' recorded camera view';box.children[1].textContent=c.role+(selectedPhase?' · '+selectedPhase.replaceAll('_',' '):' · recorded '+new Date(c.recorded_at*1000).toLocaleTimeString())}
$('stages').replaceChildren(...[{label:null},...r.views].map(v=>{let b=el('button',v.label?v.label.replaceAll('_',' '):'Latest images','stage');b.style.background=selectedPhase===v.label?'#285047':'#19262e';b.style.color='inherit';b.onclick=()=>{selectedPhase=v.label;refresh()};return b}));$('errors').replaceChildren(...(r.reasons.length?r.reasons:['No stop reason recorded.']).map(t=>el('li',t)));$('reports').replaceChildren(...r.reports.map(name=>{let li=el('li'),a=el('a',name);a.href='/files/'+r.id+'/'+name;a.target='_blank';li.append(a);return li}));
}catch(e){$('connection').textContent='Disconnected — displayed data may be old';$('connection').className='pending'}}refresh();setInterval(refresh,2500);
</script></html>'''


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/':
            body, kind = HTML.encode(), 'text/html; charset=utf-8'
        elif parsed.path == '/api/progress':
            selected = parse_qs(parsed.query).get('run', [None])[0]
            body, kind = json.dumps(snapshot(selected), allow_nan=False).encode(), 'application/json'
        elif parsed.path.startswith('/files/'):
            path = (RUNS / unquote(parsed.path[7:])).resolve()
            if not path.is_relative_to(RUNS.resolve()) or path.suffix not in ('.json', '.jpg', '.png', '.gif') or not path.is_file():
                self.send_error(404)
                return
            body, kind = path.read_bytes(), mimetypes.guess_type(path.name)[0] or 'application/octet-stream'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()
    print(f'Experiment progress: http://127.0.0.1:{args.port}', flush=True)
    ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()
