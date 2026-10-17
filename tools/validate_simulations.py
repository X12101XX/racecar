"""Run isolated ROS regression cases or recheck their summaries and complete MP4s.

CROSSFIRE_SIM_IMAGE=crossfire-sim:vision-v2 python3 tools/validate_simulations.py
python3 tools/validate_simulations.py --verify-only output/validation_TIMESTAMP
"""
import csv
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
MODEL='/workspace/models/card_classifier.onnx'
CASES=[
    ('mono_A',['--route','A','--seed','3'],'A',False),
    ('mono_B',['--route','B','--seed','3'],'B',False),
    ('card_A',['--route','B','--card-color','blue','--use-card','--ai-backend','onnx','--ai-model',MODEL,'--seed','3'],'A',True),
    ('card_B',['--route','A','--card-color','red','--use-card','--ai-backend','onnx','--ai-model',MODEL,'--seed','3'],'B',True),
    ('repeat_card_A',['--route','A','--use-card','--ai-backend','fast_color','--seed','1'],'A',True),
    ('repeat_card_B',['--route','B','--use-card','--ai-backend','fast_color','--seed','1'],'B',True),
    ('measured_A',['--profile','measured_example','--route','A'],'A',False),
    ('aggressive_A',['--route','A','--motion-preset','aggressive'],'A',False),
]


def check_run(out,code,route,card,expected_backend):
    try:
        summary=json.loads((out/'summary.json').read_text())
        events=json.loads((out/'events.json').read_text())
        metrics=summary.get('metrics') or {}
        recognized=[e for e in events if e['event']=='card_recognized']
        # Scope this change to the decoder process, not the user's shell environment.
        decode_env=os.environ.copy();decode_env.pop('LD_LIBRARY_PATH',None)
        decode=subprocess.run(['ffmpeg','-v','error','-i',str(out/'video.mp4'),'-f','null','-'],
                              capture_output=True,text=True,env=decode_env)
        from racelib.config import load_all
        from racelib.field import Field
        from racelib.geometry import rect_corners
        cfg=load_all(out/'config',route)
        field=Field.from_config(cfg['field'])
        camp=field.zone('camp_a' if route=='A' else 'camp_b')
        with (out/'trajectory.csv').open() as f: last=list(csv.DictReader(f))[-1]
        body=rect_corners(float(last['x']),float(last['y']),cfg['robot'].get('geometry.length_m'),
                         cfg['robot'].get('geometry.width_m'),float(last['yaw']))
        final_body_in_camp=all(camp.contains(p) for p in body)
        report=dict(final_body_in_camp=final_body_in_camp,exit_code=code,summary=summary,expected_route=route,
            video_decode_ok=decode.returncode==0 and not decode.stderr.strip(),
            video_decode_errors=decode.stderr.strip(),
            card_route_ok=(not card or any(e.get('route')==route for e in recognized)))
        report['passed']=bool(code==0 and summary['success'] and summary['route']==route
            and final_body_in_camp and abs(float(last['v']))<.025 and metrics.get('route')==route
            and metrics['planner_engine']=='cpp_native' and metrics['hold_actual_s']>=3
            and summary['cone_contacts']==summary['wall_contacts']==0 and (out/'completed.json').is_file()
            and report['video_decode_ok'] and report['card_route_ok']
            and (expected_backend is None or summary['ai_backend']==expected_backend))
        return report
    except Exception as exc: return dict(passed=False,error=str(exc),exit_code=code)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--verify-only',type=Path)
    args=ap.parse_args()
    if args.verify_only:
        root=args.verify_only.resolve()
        old={r['case']:r for r in json.loads((root/'report.json').read_text())}
    else:
        label='validation_'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        root=ROOT/'output'/label;root.mkdir(parents=True)
    reports=[]
    for name,options,route,card in CASES:
        if args.verify_only: code=old.get(name,{}).get('exit_code',1)
        else:
            print('Running',name,flush=True)
            command=['bash','tools/run_ros_sim.sh',*options,'--planner-engine','native','--outdir',f'/output/{root.name}/{name}']
            with (root/(name+'.log')).open('w') as log:
                code=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT).returncode
        report=check_run(root/name,code,route,card,'onnxruntime' if '--ai-model' in options else None)
        report['case']=name
        reports.append(report)
        if not args.verify_only: (root/'report.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2))
        print(name,'PASS' if report['passed'] else 'FAIL',flush=True)
    (root/'report.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2))
    print('Report:',root/'report.json',flush=True)
    return 0 if all(r['passed'] for r in reports) else 1

if __name__=='__main__':sys.exit(main())
