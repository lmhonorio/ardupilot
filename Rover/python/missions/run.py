#!/usr/bin/env python3.8
"""Circle + circumscribed square, measured and tuned through MAVProxy UDP."""
import argparse
import csv
import json
import math
import signal
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

from geometry import make_plan, write_wpl, write_qgc_plan, local_position
from analysis import summarize, propose, improves, stage_improves, TUNED


def navigation_params(args):
    """Precision profile for the skid rover; all values are backed up/restored."""
    return {'WP_SPEED':args.speed,
            'WP_RADIUS':args.wp_radius if args.wp_radius is not None else min(.2,.1*args.radius),
            'WP_ACCEL':.4,'WP_JERK':1.0,
            'ATC_ACCEL_MAX':.5,'ATC_DECEL_MAX':.5,
            'ATC_TURN_MAX_G':.1,'ATC_STR_RAT_MAX':60,'ATC_STR_ACC_MAX':90,
            'WP_PIVOT_ANGLE':args.pivot_angle,'WP_PIVOT_RATE':args.pivot_rate,
            'WP_PIVOT_DELAY':.2,'MIS_DONE_BEHAVE':0}


def save_json(path,value):
    from mavproxy_link import json_safe
    path.write_text(json.dumps(json_safe(value),indent=2,allow_nan=False)+'\n')


def save_params(path,values):
    path.write_text(''.join('%s %.9g\n'%(k,v) for k,v in sorted(values.items())))


def save_trial_rows(folder,label,rows):
    from mavproxy_link import json_safe
    with (folder/(label+'.telemetry.jsonl')).open('w') as stream:
        for row in rows:stream.write(json.dumps(json_safe(row))+'\n')


def load_trial_rows(folder,label):
    visited=set()
    for _ in range(10):
        folder=folder.resolve()
        if folder in visited:break
        visited.add(folder)
        metadata=json.loads((folder/'report.json').read_text())
        candidates=[folder/(label+'.telemetry.jsonl'),folder/'telemetry.jsonl']
        if label=='trial_01' and metadata.get('baseline_telemetry'):
            candidates.insert(0,folder/metadata['baseline_telemetry'])
        for source in candidates:
            if not source.exists():continue
            rows=[r for line in source.read_text().splitlines() for r in [json.loads(line)]
                  if r.get('trial')==label]
            if rows:return rows
        if not metadata.get('resumed_from'):break
        folder=Path(metadata['resumed_from'])
        if not folder.is_absolute():folder=Path(__file__).resolve().parents[3]/folder
    raise RuntimeError('Telemetria original de %s ausente ou cadeia de retomada invalida'%label)


def mission_finished(msg, current_seq, final_seq):
    # ArduPilot can periodically resend the last REACHED from an earlier run.
    # It is not completion of this run until CURRENT is at its final item.
    kind=msg.get_type()
    if kind=='MISSION_ITEM_REACHED':
        return current_seq==final_seq and msg.seq==final_seq
    if current_seq<final_seq-1:
        return False
    if kind=='MISSION_CURRENT':
        return getattr(msg,'mission_state',0)==5
    return kind=='STATUSTEXT' and 'Mission Complete' in str(msg.text)


def execute_trial(link, plan, index, timeout, max_speed):
    """Each run includes an approach, full circle, full square and a 2 s stop."""
    from pymavlink import mavutil
    label='trial_%02d'%index
    print('\nINICIANDO',label,flush=True)
    link.stop()
    link.upload_mission(plan['items'])
    readback=link.verify_mission(plan['items'])
    # Select without resuming while HOLD/disarmed. Reset=1 resumes the mission
    # and can invoke navigation before AUTO/EKF is ready on older firmwares.
    link.command(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,1,0)
    if getattr(link,'wait_qgc',False):
        import select
        print('\nMissao enviada. No QGroundControl: Plan -> File -> Download; depois Fly.',flush=True)
        print('Ajuste o zoom para ver o circulo e o quadrado. Pressione ENTER AQUI para iniciar.',flush=True)
        while not select.select([sys.stdin],[],[],0)[0]:
            link.recv()
        if not sys.stdin.readline():
            raise RuntimeError('Entrada encerrada antes da confirmacao de visualizacao no QGC')
        link.wait_qgc=False
    print('Aguardando parada e pre-armamento saudavel...',flush=True)
    link.wait_stationary()
    link.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
    link.wait('HEARTBEAT',lambda m:m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    link.seq,link.phase,link.records=1,'approach',[]
    link.trial=label
    completed=False
    error=None
    try:
        link.command(mavutil.mavlink.MAV_CMD_MISSION_START,1,0)
        # The mode may change a little after the command ACK.
        link.wait('HEARTBEAT',lambda m:m.custom_mode==10)
        deadline=time.monotonic()+timeout
        last_progress=link.clock
        last_distance=math.inf
        last_seq=-1
        last_print=0
        while time.monotonic()<deadline:
            msg=link.recv()
            now=time.monotonic()
            if now-link.last_seen.get('HEARTBEAT',now)>12:
                raise RuntimeError('Heartbeat perdido')
            if now-link.last_seen.get('GLOBAL_POSITION_INT',now)>12:
                raise RuntimeError('Telemetria de posicao perdida')
            link.phase=plan['items'][min(link.seq,len(plan['items'])-1)]['phase']
            if link.seq!=last_seq:
                print('%s WP %d/%d (%s)'%(label,link.seq,len(plan['items'])-1,link.phase),flush=True)
                last_seq=link.seq
                last_distance=math.inf
                last_progress=link.clock
            if msg is None:
                continue
            kind=msg.get_type()
            if kind=='HEARTBEAT' and msg.custom_mode!=10:
                raise RuntimeError('Modo alterado externamente; ensaio interrompido')
            if kind=='GLOBAL_POSITION_INT':
                if not (msg.lat or msg.lon):
                    raise RuntimeError('Posicao GPS invalida durante a missao')
                n,e=local_position(msg.lat/1e7,msg.lon/1e7,
                                  (plan['origin_lat'],plan['origin_lon']))
                distance_from_center=math.hypot(n-plan['center_north'],e-plan['center_east'])
                allowed=max(2*plan['radius_m']+2, math.hypot(plan['center_north'],plan['center_east'])+2)
                if distance_from_center>allowed:
                    raise RuntimeError('Rover saiu da area de ensaio (%.2f m)'%distance_from_center)
                target=plan['items'][min(link.seq,len(plan['items'])-1)]
                distance=math.hypot(n-target['north'],e-target['east'])
                if distance < last_distance-.15:
                    last_distance=distance
                    last_progress=link.clock
                if math.hypot(msg.vx,msg.vy)/100>max_speed:
                    raise RuntimeError('Velocidade excedeu o limite de ensaio')
                if link.clock-last_progress>max(25,8*plan['radius_m']/plan['speed_m_s']):
                    raise RuntimeError('Missao sem progresso')
            if mission_finished(msg,link.seq,len(plan['items'])-1):
                completed=True
                break
            if now-last_print>15:
                print('%s: t_sim=%.1f s, WP=%d'%(label,link.clock,link.seq),flush=True)
                last_print=now
        if not completed:
            raise TimeoutError('Tempo limite do ensaio')
    except (RuntimeError,TimeoutError) as exc:
        error=str(exc)
        print('ENSAIO INTERROMPIDO:',error,flush=True)
    finally:
        link.trial=None
        link.stop()
    result=summarize(link.records,plan,completed)
    result.update(label=label,error=error)
    result['mission_verified']=True
    result['mission_readback']=readback
    return result,list(link.records)


def write_csv(path, rows):
    keys=['mavpackettype','boot_s','wall_time','phase','seq_current','lat','lon',
          'vx','vy','yaw','yawspeed','axis','desired','achieved','FF','P','I','D',
          'servo1_raw','servo3_raw']
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys,extrasaction='ignore')
        writer.writeheader();writer.writerows(rows)


def parse_args():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--radius',type=float,required=True,help='Raio do circulo em metros; quadrado tem lado 2*raio')
    p.add_argument('--circle-points',type=int,default=36)
    p.add_argument('--speed',type=float,default=.4,help='Velocidade de teste em m/s (padrao: 0.4)')
    p.add_argument('--wp-radius',type=float,help='Raio de aceitacao em metros; padrao min(0.2, 0.1*radius)')
    p.add_argument('--pivot-angle',type=int,default=60,help='Angulo para pivo; padrao 60 graus; 0 seleciona curvas continuas')
    p.add_argument('--pivot-rate',type=float,default=30,help='Velocidade maxima de pivo em graus/s; padrao 30')
    p.add_argument('--center-north',type=float,default=0)
    p.add_argument('--center-east',type=float,default=0)
    p.add_argument('--origin-lat',type=float,default=-3.123150,help='Origem do plano offline; execucao usa GPS atual')
    p.add_argument('--origin-lon',type=float,default=-41.764495)
    p.add_argument('--endpoint',default='udpin:127.0.0.1:14555',help='Saida UDP de MAVProxy, nunca a serial ou TCP do autopiloto')
    p.add_argument('--sysid',type=int,default=1)
    p.add_argument('--source-system',type=int,default=246,help='SYSID deste cliente MAVLink; deve ser permitido pelo autopiloto')
    p.add_argument('--execute',action='store_true',help='Enviar missao, armar e executar')
    p.add_argument('--upload-only',action='store_true',help='Enviar e conferir a missao, sem armar ou iniciar')
    p.add_argument('--wait-qgc',action='store_true',help='Pausar apos o envio ate ENTER, para visualizar a missao no QGC')
    p.add_argument('--optimize',action='store_true',help='Comparar baseline, candidatos e validar o melhor')
    p.add_argument('--iterations',type=int,default=2,help='Numero maximo de candidatos apos o baseline')
    p.add_argument('--trial-timeout',type=float,help='Limite real por ensaio; padrao calculado pelo tamanho da missao (minimo 1200 s)')
    p.add_argument('--max-speed',type=float,default=1.5,help='Aborta acima desta velocidade medida (m/s)')
    p.add_argument('--real',action='store_true',help='Permitir execucao no rover fisico; sem esta opcao exige SIMSTATE')
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--resume-run',type=Path,help='Reutilizar baseline completo documentado, com geometria e ganhos iguais')
    args=p.parse_args()
    if args.resume_run:
        args.resume_run=args.resume_run.resolve()
    if (args.iterations<1 or args.iterations>6 or
            (args.trial_timeout is not None and (not math.isfinite(args.trial_timeout) or args.trial_timeout<=0))
            or not 1<=args.sysid<=255 or not 1<=args.source_system<=255
            or args.source_system==args.sysid):
        p.error('iterations: 1..6; trial-timeout positivo; sysid: 1..255')
    if args.optimize and not args.execute:
        p.error('--optimize exige --execute')
    if args.upload_only and (args.execute or args.optimize or args.resume_run):
        p.error('--upload-only nao combina com --execute, --optimize ou --resume-run')
    if args.wait_qgc and not args.execute:
        p.error('--wait-qgc exige --execute')
    if args.wp_radius is not None and (not math.isfinite(args.wp_radius) or not 0<args.wp_radius<args.radius):
        p.error('--wp-radius deve ser positivo e menor que --radius')
    if args.pivot_angle!=0 and not 30<=args.pivot_angle<=360:
        p.error('--pivot-angle: 0 (desativado) ou 30..360 graus')
    if not math.isfinite(args.pivot_rate) or not 0<args.pivot_rate<=60:
        p.error('--pivot-rate deve estar entre 0 (exclusivo) e 60 graus/s')
    if args.resume_run and not (args.execute and args.optimize):
        p.error('--resume-run exige --execute --optimize')
    if not math.isfinite(args.max_speed) or args.max_speed<=args.speed:
        p.error('--max-speed deve ser finito e maior que --speed')
    if args.execute and (args.radius<.5 or args.speed>1.0 or args.speed<.15):
        p.error('Execucao: radius >= 0.5 m e speed entre 0.15 e 1.0 m/s')
    return args


def main():
    args=parse_args()
    # Validate geometry before opening any network connection.
    plan=make_plan(args.radius,(args.origin_lat,args.origin_lon),
                   (args.center_north,args.center_east),args.circle_points,args.speed)
    if args.trial_timeout is None:
        args.trial_timeout=max(1200,plan['nominal_time_s']*10+120)
    root=args.output_dir or Path(__file__).resolve().parents[2]/'mission_tune/runs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    root.mkdir(parents=True,exist_ok=False)
    save_json(root/'plan.json',plan);write_wpl(root/'mission.waypoints',plan['items'])
    write_qgc_plan(root/'mission.plan',plan)
    print('Raio %.2f m; quadrado %.2f m; resultados: %s'%(args.radius,2*args.radius,root),flush=True)
    if not (args.execute or args.upload_only):
        from report import render_report
        offline_report=dict(plan=plan,trials=[],mode='plan_only')
        save_json(root/'report.json',offline_report)
        render_report(root,offline_report)
        print('Somente plano gerado. Use --execute para movimentar o rover.')
        return
    from mavproxy_link import Link
    import fcntl
    control_dir=Path(__file__).resolve().parents[2]/'mission_tune/runs'
    control_dir.mkdir(parents=True,exist_ok=True)
    control_lock=(control_dir/('control_%d.lock'%args.sysid)).open('a+')
    try:
        fcntl.flock(control_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Outro ensaio ja controla este SYSID; encerre-o antes de iniciar outro.')
    link=Link(args.endpoint,args.sysid,root,source_system=args.source_system)
    link.wait_qgc=args.wait_qgc
    original={}
    best_params=None
    success=False
    optimized=False
    trials=[]
    report=dict(plan=plan,trials=trials,transport='script -> UDP MAVProxy --out -> autopiloto',
                mission_item_encoding='MISSION_ITEM_INT',
                real=args.real,endpoint=args.endpoint,
                limits=dict(max_speed=args.max_speed,trial_timeout=args.trial_timeout,
                            iterations=args.iterations))
    try:
        link.ready(args.real)
        resumed=None
        if args.resume_run:
            resumed=json.loads((args.resume_run/'report.json').read_text())
            plan=resumed['plan']
            if (not math.isclose(plan['radius_m'],args.radius) or
                    not math.isclose(plan['speed_m_s'],args.speed) or
                    plan['circle_points']!=args.circle_points or
                    resumed.get('real',False)!=args.real or
                    resumed.get('mission_item_encoding')!='MISSION_ITEM_INT' or
                    not resumed.get('test_params')):
                raise RuntimeError('Baseline incompativel: geometria, velocidade, veiculo ou condicoes nao documentadas')
            n,e=local_position(*link.origin,(plan['origin_lat'],plan['origin_lon']))
            if math.hypot(n-plan['center_north'],e-plan['center_east'])>2*args.radius+2:
                raise RuntimeError('Rover longe da area do baseline')
        else:
            plan=make_plan(args.radius,link.origin,(args.center_north,args.center_east),
                           args.circle_points,args.speed)
        report['plan']=plan
        plan['origin_alt_m']=link.latest['GLOBAL_POSITION_INT'].alt/1000
        save_json(root/'plan.json',plan);write_wpl(root/'mission.waypoints',plan['items'])
        write_qgc_plan(root/'mission.plan',plan)
        backup=link.download_mission();save_json(root/'original_mission.json',backup)
        if args.upload_only:
            link.upload_mission(plan['items'])
            save_json(root/'mission_readback.json',link.verify_mission(plan['items']))
            report['mode']='upload_only'
            report['mission_uploaded']=True
            success=True
            print('Missao enviada e conferida. Rover nao foi armado/iniciado.',flush=True)
            print('QGroundControl: Plan -> File -> Download; depois Fly para acompanhar.',flush=True)
            return
        temporary=navigation_params(args)
        names=TUNED+['CRUISE_SPEED','GCS_PID_MASK']+list(temporary)
        for name in names:
            original[name]=link.param(name)
        save_params(root/'original.parm',original)
        report['original_params']=original
        rate_limit=temporary['ATC_STR_RAT_MAX']
        if rate_limit>0 and math.degrees(args.speed/args.radius)>.8*rate_limit:
            raise RuntimeError('speed/radius excede 80% de ATC_STR_RAT_MAX; reduza --speed')
        link.mode('HOLD')
        temporary['GCS_PID_MASK']=int(original['GCS_PID_MASK'])|3
        report['test_params']=dict(temporary)
        link.set_params(temporary)
        best_params={k:original[k] for k in TUNED}
        best=None
        tuning_rows=[]
        first_index=2 if resumed else 1
        working_result=None
        working_params=dict(best_params)
        if resumed:
            baseline=resumed['trials'][0]
            if (not baseline['completed'] or baseline['score'] is None or
                    any(not math.isclose(original[k],baseline['parameters'][k],rel_tol=1e-4,abs_tol=1e-5) for k in TUNED) or
                    any(not math.isclose(original[k],resumed['original_params'][k],rel_tol=1e-4,abs_tol=1e-5)
                        for k in ['CRUISE_SPEED','ATC_STR_RAT_MAX']) or
                    any(not math.isclose(temporary[k],resumed['test_params'].get(k,math.nan),rel_tol=1e-4,abs_tol=1e-5) for k in temporary)):
                raise RuntimeError('Baseline nao completo ou parametros/condicoes diferentes')
            tuning_rows=load_trial_rows(args.resume_run,'trial_01')
            best=baseline
            working_result=baseline
            trials.append(baseline)
            report['resumed_from']=str(args.resume_run)
            for suffix in ['csv','json']:
                shutil.copyfile(args.resume_run/('trial_01.'+suffix),root/('trial_01.'+suffix))
            print('Reutilizando baseline completo:',args.resume_run,flush=True)
            save_trial_rows(root,'trial_01',tuning_rows)
            report['baseline_telemetry']='trial_01.telemetry.jsonl'
            # Resume contiguous completed stages, without persisting an
            # intermediate tune that has not won the combined comparison.
            for previous in resumed['trials'][1:]:
                if previous.get('validation') or first_index>args.iterations+1:break
                if previous['label']!='trial_%02d'%first_index:break
                stage='speed' if first_index%2==0 else 'steering'
                if not (stage_improves(previous,working_result,stage,args.radius) or improves(previous,best)):break
                tuning_rows=load_trial_rows(args.resume_run,previous['label'])
                working_result=previous
                working_params=dict(previous['parameters'])
                previous['stage_progress']=True
                trials.append(previous)
                first_index+=1
                if improves(previous,best):best,best_params=previous,dict(working_params)
                for suffix in ['csv','json']:
                    shutil.copyfile(args.resume_run/(previous['label']+'.'+suffix),root/(previous['label']+'.'+suffix))
                save_trial_rows(root,previous['label'],tuning_rows)
                print('Reutilizando etapa completa:',previous['label'],flush=True)
        for index in range(first_index,2+(args.iterations if args.optimize else 0)):
            if index==1:
                tested=dict(best_params)
            else:
                # Calibrate speed feedforward/feedback before steering, so
                # variations in speed do not corrupt turn-rate identification.
                stage='speed' if index%2==0 else 'steering'
                tested,evidence=propose(tuning_rows,working_params,original['CRUISE_SPEED'],
                                        max_step=.2 if args.real else .5,stage=stage)
                if tested is None:
                    print('Sem novo candidato identificavel.',flush=True)
                    break
                save_json(root/('candidate_%02d.json'%index),dict(stage=stage,parameters=tested,evidence=evidence))
                link.set_params(tested)
            result,rows=execute_trial(link,plan,index,args.trial_timeout,args.max_speed)
            result['parameters']=tested
            result['stage']='baseline' if index==1 else stage
            write_csv(root/('%s.csv'%result['label']),rows)
            save_trial_rows(root,result['label'],rows)
            save_json(root/('%s.json'%result['label']),result)
            trials.append(result)
            score=result['score']
            print('RESULTADO: completo=%s score=%s erro_trajetoria=%s m'%(result['completed'],score,result['tracking_rmse_m']),flush=True)
            accepted=improves(result,best)
            result['accepted']=accepted
            progress=index>1 and stage_improves(result,working_result,stage,args.radius)
            result['stage_progress']=progress
            if accepted:
                best,best_params=result,dict(tested)
            if accepted or progress:
                working_result,working_params,tuning_rows=result,dict(tested),rows
            elif best is not None:
                link.set_params(best_params)
                break
            else:
                raise RuntimeError('Dados insuficientes para identificar resposta; verifique PID_TUNING e trajetoria')
            save_json(root/'report.json',report)
        if args.optimize and best and best['label']!='trial_01':
            print('Validando repeticao com o melhor candidato...',flush=True)
            link.set_params(best_params)
            result,rows=execute_trial(link,plan,len(trials)+1,args.trial_timeout,args.max_speed)
            result['parameters']=dict(best_params)
            result['validation']=True
            write_csv(root/('%s.csv'%result['label']),rows)
            save_trial_rows(root,result['label'],rows)
            save_json(root/('%s.json'%result['label']),result);trials.append(result)
            baseline=trials[0]
            if (not result['completed'] or result['score'] is None or
                (baseline['completed'] and result['score']>=baseline['score']*.98)):
                raise RuntimeError('Melhora nao confirmada na repeticao; restaurando parametros originais')
        if not best or not best['completed']:
            raise RuntimeError('Nenhum ensaio completo valido; sem sintonia aceita')
        success=True
        optimized=bool(args.optimize and best['label']!='trial_01')
        report['best_trial']=best['label']
        report['best_params']=best_params
        report['optimization_improved']=optimized
        save_params(root/'selected.parm',best_params)
        if optimized:
            save_params(root/'optimized.parm',best_params)
        elif args.optimize:
            print('Sem melhora global validada; parametros originais preservados.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=str(exc) or 'Interrompido pelo operador'
        print('ERRO:',report['error'],flush=True)
    finally:
        if original:
            try:
                link.stop()
                restored=dict(original)
                if success and optimized:
                    restored.update(best_params)
                link.set_params(restored)
                report['active_params']=restored
                report['restored_original']=not (success and optimized)
            except Exception as exc:
                success=False
                report['cleanup_error']=str(exc)
                print('Falha no HOLD/desarme/restauro; confira o rover:',exc,flush=True)
        report['success']=success
        save_json(root/'report.json',report)
        link.close()
        control_lock.close()
        from report import render_report
        render_report(root,report)
    print('Relatorio:',root/'report.html',flush=True)
    if not success:
        raise SystemExit(1)


if __name__=='__main__':
    # Treat termination like Ctrl+C, so the rollback also runs with kill -TERM.
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    main()
