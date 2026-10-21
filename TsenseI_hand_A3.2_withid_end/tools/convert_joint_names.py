"""Translate only the twenty output joint names; preserve numeric coordinates."""
import argparse
import json
import math
from pathlib import Path


def read(path):
    def unique(pairs):
        result={}
        for key,value in pairs:
            if key in result:raise ValueError('Duplicate JSON key: '+key)
            result[key]=value
        return result
    return json.loads(path.read_text(encoding='utf-8-sig'),object_pairs_hook=unique)


def convert(data,contract,direction):
    public=contract['public_to_source'];source=contract['source_to_public']
    if direction=='to-public':
        mapping={**{k:k for k in public},**source}
        mapping.update(contract.get('legacy_to_public',{}))
        old_interface,new_interface=contract['source_interface_id'],contract['public_interface_id']
    else:
        mapping={**{k:k for k in source},**public}
        mapping.update({old:public[new] for old,new in contract.get('legacy_to_public',{}).items() if new in public})
        old_interface,new_interface=contract['public_interface_id'],contract['source_interface_id']
    def names(values):
        if not isinstance(values,list) or not 1<=len(values)<=20 or not all(isinstance(n,str) for n in values):
            raise ValueError('Expected one to twenty output joint names')
        if any(n not in mapping for n in values):raise ValueError('Unknown output joint; full 48-state records are not accepted')
        result=[mapping[n] for n in values]
        if len(set(result))!=len(result):raise ValueError('Old/new aliases collide; supply each physical output only once')
        return result
    def numeric(values):
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):
            raise ValueError('Coordinates must be finite numbers')
    def angle_map(value):
        if not isinstance(value,dict):raise ValueError('Expected a joint-name to angle object')
        renamed=names(list(value));numeric(value.values())
        return dict(zip(renamed,value.values()))
    if not isinstance(data,dict):raise ValueError('Expected a JSON object')
    if 'name' in data:
        if set(data)-{'header','name','position','velocity','effort','model_id','interface_id'}:
            raise ValueError('Unsupported JointState JSON field')
        result=dict(data);result['name']=names(data['name'])
        for field in ['position','velocity','effort']:
            if field not in data:
                if field=='position':raise ValueError('JointState requires position')
                continue
            if not isinstance(data[field],list) or len(data[field]) not in ([len(result['name'])] if field=='position' else [0,len(result['name'])]):
                raise ValueError('JointState vector length mismatch: '+field)
            numeric(data[field])
    elif 'angles_rad' in data or 'angles_deg' in data:
        if set(data)-{'angles_rad','angles_deg','model_id','interface_id'} or ('angles_rad' in data)==('angles_deg' in data):
            raise ValueError('Use exactly one of angles_rad or angles_deg')
        field='angles_rad' if 'angles_rad' in data else 'angles_deg'
        result=dict(data);result[field]=angle_map(data[field])
    else:
        return angle_map(data)
    if 'interface_id' in result:
        if result['interface_id'] not in [old_interface,new_interface,*contract.get('legacy_interface_ids',[])]:raise ValueError('Unknown interface_id')
        result['interface_id']=new_interface
    if 'model_id' in result and result['model_id']!=contract['source_model_id']:
        raise ValueError('model_id does not match this geometry revision')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--direction',choices=['to-public','to-source'],default='to-public')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.output and args.output.resolve()==args.input.resolve():raise ValueError('Choose a separate output file')
    root=Path(__file__).resolve().parents[1]
    result=convert(read(args.input),read(root/'config/joint_name_map.json'),args.direction)
    encoded=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if args.output:args.output.write_text(encoded,encoding='utf-8')
    else:print(encoded,end='')


if __name__=='__main__':main()
