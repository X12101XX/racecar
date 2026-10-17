"""Export a deterministic RGB threshold card classifier, not an untrained CNN.

Requires onnx in a private environment. Fixed input: RGB float32 NCHW
[1,3,224,224], centre 70% ROI; scores [background,blue,red].
Only a calibrated color baseline: lighting robustness must be tested on site.
"""
from pathlib import Path
import argparse
import json


def export_model(output):
    import onnx
    from onnx import helper as h, TensorProto as T
    nodes=[];initial=[]
    def constant(name,value):
        initial.append(h.make_tensor(name,T.FLOAT,[],[value]));return name
    def op(kind,inputs,name,**kw):
        nodes.append(h.make_node(kind,inputs,[name],**kw));return name
    for channel,name in enumerate(('r','g','b')):
        initial.append(h.make_tensor(name+'_index',T.INT64,[1],[channel]))
        op('Gather',['input',name+'_index'],name,axis=1)
    constant('brightness',60/255);constant('dominance',1.45);constant('blue_green',1.25)
    constant('ratio_min',.03);constant('confidence_ratio',.12);constant('one',1.)
    def ratio(color,main,others):
        mask=op('Greater',[main,'brightness'],color+'_bright')
        for i,(other,factor) in enumerate(others):
            threshold=op('Mul',[other,factor],color+'_threshold'+str(i))
            greater=op('Greater',[main,threshold],color+'_greater'+str(i))
            mask=op('And',[mask,greater],color+'_mask'+str(i))
        cast=op('Cast',[mask],color+'_float',to=T.FLOAT)
        return op('ReduceMean',[cast],color+'_ratio',axes=[1,2,3],keepdims=1)
    blue=ratio('blue','b',[('r','dominance'),('g','blue_green')])
    red=ratio('red','r',[('g','dominance'),('b','dominance')])
    top=op('Max',[blue,red],'max_ratio')
    background=op('Cast',[op('Less',[top,'ratio_min'],'no_card')],'background',to=T.FLOAT)
    for color in ('blue','red'):
        scaled=op('Div',[color+'_ratio','confidence_ratio'],color+'_scaled')
        op('Min',[scaled,'one'],color+'_score')
    concat=op('Concat',[background,'blue_score','red_score'],'scores4d',axis=1)
    initial.append(h.make_tensor('output_shape',T.INT64,[2],[1,3]))
    op('Reshape',[concat,'output_shape'],'scores')
    graph=h.make_graph(nodes,'calibratable_rgb_card_thresholds',
        [h.make_tensor_value_info('input',T.FLOAT,[1,3,224,224])],
        [h.make_tensor_value_info('scores',T.FLOAT,[1,3])],initializer=initial)
    model=h.make_model(graph,opset_imports=[h.make_opsetid('',11)])
    model.ir_version=8
    onnx.checker.check_model(model)
    path=Path(output);path.parent.mkdir(parents=True,exist_ok=True)
    onnx.save(model,str(path))
    path.with_suffix('.json').write_text(json.dumps(dict(
        type='deterministic_rgb_thresholds',trained=False,
        input='centre 70% ROI, nearest 224x224, RGB float32 NCHW [0,1]',
        labels=['background','blue','red'],output='scores [1,3]',
        note='Not YOLO or learned detector; verify thresholds under actual lighting.'),indent=2)+'\n')
    return path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',default='models/card_classifier.onnx')
    args=ap.parse_args()
    try: path=export_model(args.output)
    except ImportError as exc: ap.exit(2,f'Export failed: {exc}; use the isolated vision image.\n')
    print('Exported deterministic color baseline:',path)
    print('For Ascend, use the actual chip SOC and compatible CANN ATC; see docs/07_NPU与GPU视觉加速指南.md.')
    return 0


if __name__=='__main__':raise SystemExit(main())
