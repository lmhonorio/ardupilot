"""Standalone HTML report and optional matplotlib plots; no network requests."""
import csv
import html
import json
from pathlib import Path


def render_report(folder, report):
    folder=Path(folder)
    trials=report.get('trials',[])
    plan=report['plan']
    plots=[]
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(7,7))
        reference=[p for p in plan['items'] if p['phase'] not in ('home','finish')]
        ax.plot([p['east'] for p in reference],[p['north'] for p in reference],
                'k--',label='Referencia',linewidth=1)
        for trial in trials:
            tr=trial.get('trajectory',[])
            ax.plot([p['east'] for p in tr],[p['north'] for p in tr],label=trial['label'],alpha=.8)
        ax.set(xlabel='Leste (m)',ylabel='Norte (m)',title='Circulo e quadrado circunscrito')
        ax.axis('equal');ax.grid();ax.legend()
        fig.tight_layout();fig.savefig(folder/'trajectory.png',dpi=150);plt.close(fig)
        plots.append('trajectory.png')
        for trial in trials:
            csv_path=folder/(trial['label']+'.csv')
            if not csv_path.exists():continue
            rows=list(csv.DictReader(csv_path.open()))
            fig,axes=plt.subplots(2,1,figsize=(10,6))
            for axis,axis_id,label in zip(axes,[5,4],['Taxa de giro (graus/s)','Velocidade (m/s)']):
                data=[r for r in rows if r['mavpackettype']=='PID_TUNING' and r['axis']==str(axis_id)]
                axis.plot([float(r['boot_s']) for r in data],[float(r['desired']) for r in data],label='Desejado')
                axis.plot([float(r['boot_s']) for r in data],[float(r['achieved']) for r in data],label='Medido')
                axis.set(ylabel=label);axis.grid();axis.legend()
            axes[-1].set_xlabel('Tempo do autopiloto (s)')
            fig.tight_layout();name=trial['label']+'_response.png'
            fig.savefig(folder/name,dpi=150);plt.close(fig);plots.append(name)
    except ImportError:
        pass
    columns=['label','completed','accepted','validation','tracking_rmse_m','circle_radial_rmse_m',
             'steering_rmse','speed_rmse','score','error']
    table='<table><tr>'+''.join('<th>'+html.escape(k)+'</th>' for k in columns)+'</tr>'
    for trial in trials:
        table+='<tr>'+''.join('<td>'+html.escape(str(trial.get(k,'')))+'</td>' for k in columns)+'</tr>'
    table+='</table>'
    brief={k:v for k,v in report.items() if k not in ('trials','plan')}
    contents='''<!doctype html><html lang="pt-BR"><meta charset="utf-8">
<title>Resposta e sintonia Rover</title><style>body{font:16px sans-serif;margin:30px;max-width:1200px}
table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:6px}img{max-width:100%}
pre{white-space:pre-wrap}a{color:#135da0}</style><h1>Ensaio Rover: circulo e quadrado</h1>'''
    contents+='<p>Raio: %.3f m. Lado: %.3f m. Velocidade: %.3f m/s.</p>'%(plan['radius_m'],plan['square_side_m'],plan['speed_m_s'])
    contents+='<p>Taxa de giro em graus/s; velocidade em m/s. Score menor e melhor. '+\
        'Avaliacao da trajetoria inclui navegacao e estimativa, nao isola apenas os PIDs. '+\
        'Sintonia local para esta velocidade, geometria e superficie; nao e um otimo global.</p>'
    contents+=table
    for name in plots:contents+='<p><img src="'+name+'" alt="'+name+'"></p>'
    if not plots:contents+='<p>Graficos opcionais: instale matplotlib no Python 3.8 e execute plot_report.py.</p>'
    contents+='<h2>Resultado e parametros</h2><pre>'+html.escape(json.dumps(brief,indent=2))+'</pre>'
    links=[(name,label) for name,label in [('report.json','Dados completos'),
            ('telemetry.jsonl','Telemetria bruta'),('mission.waypoints','Missao WPL'),
            ('mission.plan','Abrir no QGroundControl')]
           if (folder/name).exists()]
    contents+='<p>'+' | '.join('<a href="%s">%s</a>'%(name,label) for name,label in links)+'</p></html>'
    (folder/'report.html').write_text(contents)
