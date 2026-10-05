"""Carry to the checked adjacent area and retrace before source placement.

This stage never opens the jaws over the adjacent table. The pickup owner
continues servicing the gripper throughout planning, transit and recovery.
"""
import json
import time
import numpy as np
from rotation_math import rotation_vector
from plate_transfer import table_geometry,cartesian_segment
from blended_motion import checked_blend_segments


def check_retained(reference,current):
    reference=np.asarray(reference,float);current=np.asarray(current,float)
    if reference.shape!=(4,4) or current.shape!=(4,4) or not np.isfinite(np.r_[reference.ravel(),current.ravel()]).all():
        raise ValueError('Invalid carried plate observation')
    translation=float(np.linalg.norm(reference[:3,3]-current[:3,3]))
    angle=float(np.degrees(np.linalg.norm(rotation_vector(reference[:3,:3].T@current[:3,:3]))))
    if translation>.002 or angle>3.:raise ValueError('Carried plate shifted relative to the gripper')
    return {'relative_translation_m':translation,'relative_rotation_deg':angle}


def carry_and_return(runner,w,observer,folder,capture):
    if len(getattr(w,'holder_evidence',[]))<10:raise ValueError('Carry requires preflight holder evidence')
    data=observer.get();reference=np.array(data['T_camera_b_plate'])
    source=list(runner.targets.values())
    origin,normal,side=table_geometry(w.geometry,runner.rows[-1]['raw_deg'],data)
    # The plate has already risen 20 mm and visibly follows the jaws. Raise
    # another 20 mm before lateral travel; keep the same orientation.
    first,c1=cartesian_segment(w.geometry,source,.020*normal,runner.limits,origin,normal,
                               min_clearance=.020,between_checks=runner.tick)
    second,c2=cartesian_segment(w.geometry,first[-1],-.115*side,runner.limits,origin,normal,
                                min_clearance=.035,between_checks=runner.tick)
    paths=[]
    for points in (first,second):paths.extend(checked_blend_segments(points,16.,144.,runner.tick))
    for points in paths:checked_blend_segments(list(reversed(points)),16.,144.,runner.tick)
    saved=(runner.low.copy(),runner.high.copy(),runner.speed,runner.acceleration,runner.pacing_lag,runner.vision_ready)
    surface_required=observer.surface_required
    record={'kind':'carry_and_return_no_table_release','offset_m':-.115,
            'extra_clearance_lift_m':.020,'min_pad_table_clearance_m':min(c1,c2),
            'destination_reached':False,'returned_to_source':False,'paths_deg':paths,'segments':[]}
    def save():
        (folder/'table_carry.json').write_text(json.dumps(record,indent=2)+'\n')
    def ready():
        try:
            observed=observer.get()
            check_retained(reference,observed['T_camera_b_plate'])
            if not saved[-1]():raise ValueError('Camera streams unavailable')
            return True
        except ValueError as error:
            record['vision_rejection']=str(error);return False
    allpoints=np.array([point for points in paths for point in points])
    runner.low=allpoints.min(0)-.05;runner.high=allpoints.max(0)+.05
    runner.speed=16.;runner.acceleration=144.;runner.pacing_lag=1.8;runner.vision_ready=ready
    # Clear transit uses preflight table clearance and live plate retention.
    # Do not make it depend on wood patches hidden by the carried plate.
    # No contact/release is allowed in this mode. Restore paired wood+plate
    # observations before handing control back to source placement.
    observer.surface_required=False
    # Keep separate segment histories. A partial stop retraces only the
    # partial segment, then the original smooth, completed reverse curves.
    completed=[];partial=[source];returned=False
    try:
        for points in paths:
            partial=[list(points[0])]
            record['segments'].append(runner.blend(points,partial))
            completed.append(points);partial=[list(points[-1])]
        record['destination_reached']=True
        record['retention_at_destination']=check_retained(reference,observer.get()['T_camera_b_plate'])
        record['destination_table_tracking_available']='surface' in observer.get()
        capture('table_carry_destination');save()
    except ValueError as error:
        record['stop_reason']=str(error);save()
    finally:
        # Do not allow the holder-only cleanup to lower diagonally or release
        # a plate here. This function only returns after the source is reached.
        current=list(runner.targets.values())
        if np.max(abs(np.array(current)-partial[-1]))>1e-8:partial.append(current)
        try:
            record['partial_return']=runner.retrace(list(reversed(partial)))
            record['return_segments']=[]
            for points in reversed(completed):
                record['return_segments'].append(runner.blend(list(reversed(points)),[],returning=True))
            runner.settle(source,returning=True)
            record['returned_to_source']=returned=True
            observer.surface_required=surface_required
            # Allow the same ongoing tracker to catch up at the source.
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                runner.tick()
                if ready() and 'surface' in observer.get():break
            else:raise RuntimeError('Carry returned, but source observation did not reacquire')
            record['retention_at_source']=check_retained(reference,observer.get()['T_camera_b_plate'])
            capture('table_carry_returned')
        except Exception as error:
            record['return_error']=str(error);save()
            if not returned:
                # Retain this stack and its gripper heartbeat if a software
                # return fails. A drive/feedback fault still propagates through
                # the normal hardware shutdown. Never open away from source.
                print('PAYLOAD HOLD: checked carry return failed; jaws remain powered',folder,flush=True)
                while True:runner.tick()
            raise
        finally:
            observer.surface_required=surface_required
            runner.low,runner.high,runner.speed,runner.acceleration,runner.pacing_lag,runner.vision_ready=saved
            save()
    return record
