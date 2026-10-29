"""Tracking metrics and bounded response-based tuning, independent of transport."""
import math
import numpy as np
from geometry import local_position

TUNED = ['ATC_STR_RAT_FF', 'ATC_STR_RAT_P', 'ATC_STR_RAT_I',
         'ATC_SPEED_P', 'ATC_SPEED_I', 'CRUISE_THROTTLE']


def finite_pid(row):
    return all(isinstance(row.get(k),(int,float)) and math.isfinite(row[k])
               for k in ['desired','achieved','FF','P','I','D'])


def rmse(values):
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None


def segment_distance(n, e, a, b):
    delta = np.array([b['north']-a['north'], b['east']-a['east']])
    offset = np.array([n-a['north'], e-a['east']])
    t = np.clip(offset.dot(delta) / max(delta.dot(delta), 1e-12), 0, 1)
    return float(np.linalg.norm(offset - t*delta))


def summarize(rows, plan, completed):
    result = dict(completed=completed, samples=len(rows))
    trajectories, errors, radial = [], [], []
    phase_errors = {'circle': [], 'square': []}
    for row in rows:
        if row['mavpackettype'] != 'GLOBAL_POSITION_INT' or not (row['lat'] or row['lon']):
            continue
        n, e = local_position(row['lat']/1e7, row['lon']/1e7,
                              (plan['origin_lat'], plan['origin_lon']))
        trajectories.append(dict(north=n, east=e, boot_s=row['boot_s'], phase=row['phase']))
        seq = row['seq_current']
        if row['phase'] in ('circle', 'square') and 1 <= seq < len(plan['items']):
            distance = segment_distance(n, e, plan['items'][seq-1], plan['items'][seq])
            errors.append(distance)
            phase_errors[row['phase']].append(distance)
            if row['phase'] == 'circle':
                radial.append(math.hypot(n-plan['center_north'], e-plan['center_east']) - plan['radius_m'])
    result.update(tracking_rmse_m=rmse(errors), circle_radial_rmse_m=rmse(radial),
                  circle_tracking_rmse_m=rmse(phase_errors['circle']),
                  square_tracking_rmse_m=rmse(phase_errors['square']),
                  tracking_p95_m=float(np.percentile(errors,95)) if errors else None,
                  trajectory=trajectories)
    for axis, label in [(5,'steering'), (4,'speed')]:
        records = [r for r in rows if r['mavpackettype']=='PID_TUNING' and r['axis']==axis
                   and finite_pid(r)
                   and r['phase'] in ('circle','square') and abs(r['desired']) > (2 if axis==5 else .1)]
        result[label+'_samples'] = len(records)
        result[label+'_rmse'] = rmse([r['desired']-r['achieved'] for r in records])
        result[label+'_saturation_fraction'] = float(np.mean([
            abs(r['FF']+r['P']+r['I']+r['D']) > .95 for r in records])) if records else None
        result[label+'_bias'] = float(np.mean([r['desired']-r['achieved'] for r in records])) if records else None
    if errors and result['speed_samples'] >= 10 and result['steering_samples'] >= 10:
        turn_rate = math.degrees(plan['speed_m_s']/plan['radius_m'])
        result['score'] = (result['tracking_rmse_m']/plan['radius_m'] +
            .5*result['speed_rmse']/plan['speed_m_s'] +
            .5*result['steering_rmse']/max(turn_rate,5) +
            .15*(result['speed_saturation_fraction']+result['steering_saturation_fraction']) +
            (0 if completed else 100))
        result['quality_score'] = result['score'] - (0 if completed else 100)
    else:
        result['score'] = None
        result['quality_score'] = None
    return result


def improves(result, best):
    if result['score'] is None:
        return False
    if best is None:
        return True
    if result['completed'] != best['completed']:
        return result['completed']
    return result['quality_score'] < best['quality_score'] * .98


def stage_improves(result, reference, stage, radius):
    """An intermediate axis gain may help identify the next coupled axis.
    It is never persisted unless the final combined mission score improves.
    """
    metric='speed_rmse' if stage=='speed' else 'steering_rmse'
    return bool(result['completed'] and result['score'] is not None and
                reference and reference.get(metric) is not None and
                result.get(metric) is not None and
                result.get('tracking_p95_m',math.inf) <= radius and
                result[metric] < reference[metric]*.98)


def propose(rows, current, cruise_speed, max_step=.5, stage=None):
    """Identify output/response ratios. Inspired by ArduPilot rover-quicktune.

    Steering PID_TUNING achieved is deg/s: convert to radians/s for ATC gain.
    Speed FF already includes base CRUISE_THROTTLE feedforward in Rover.
    These are candidates, never a guarantee of an optimum.
    """
    candidate, evidence = dict(current), {}

    def bounded(name, target, low, high):
        old = current[name]
        if old > 0:
            low, high = max(low, old*(1-max_step)), min(high, old*(1+max_step))
        if low > high:
            raise ValueError('Valor original de %s fora dos limites de sintonia' % name)
        return float(np.clip(target,low,high))

    for axis,label in [(5,'steering'),(4,'speed')]:
        if stage is not None and stage != label:
            continue
        records = [r for r in rows if r['mavpackettype']=='PID_TUNING' and r['axis']==axis
                   and finite_pid(r)
                   and r['phase']=='circle']
        valid = []
        for r in records:
            achieved = math.radians(r['achieved']) if axis==5 else r['achieved']
            desired = math.radians(r['desired']) if axis==5 else r['desired']
            output = r['FF']+r['P']+r['I']+r['D']
            if achieved*desired > 0 and abs(achieved) > (.05 if axis==5 else .15) and .005 < abs(output) < .85:
                valid.append((abs(output),abs(achieved)))
        if len(valid) < 20:
            evidence[label] = dict(samples=len(valid), reason='Excitacao insuficiente ou saturada')
            continue
        ratio = float(np.median([u/v for u,v in valid]))
        evidence[label] = dict(samples=len(valid), output_per_response=ratio)
        if axis==5:
            ff = bounded('ATC_STR_RAT_FF',.9*ratio,.005,3)
            candidate['ATC_STR_RAT_FF'] = ff
            for suffix in ['P','I']:
                name='ATC_STR_RAT_'+suffix
                candidate[name] = bounded(name,.5*ff,.001,2)
        else:
            candidate['CRUISE_THROTTLE'] = bounded('CRUISE_THROTTLE',100*ratio*cruise_speed,1,100)
            for suffix in ['P','I']:
                name='ATC_SPEED_'+suffix
                candidate[name] = bounded(name,ratio,.01 if suffix=='P' else .001,2)
    if all(math.isclose(candidate[k],current[k],rel_tol=.01,abs_tol=.0001) for k in candidate):
        return None,evidence
    return candidate,evidence
