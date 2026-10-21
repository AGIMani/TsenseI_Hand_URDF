#!/usr/bin/env python3
"""Recompute link inertials after changing mass_g in config/measured_link_masses.json.
Uses packaged native geometry moments, for the current fixed rigid-link grouping only.
"""
from pathlib import Path
import json,copy,xml.etree.ElementTree as ET
import numpy as np

def run(root):
    source=json.loads((root/'config/measured_link_masses.json').read_text('utf-8'))
    parts=json.loads((root/'provenance/part_mass_moment_ledger_density_1.json').read_text('utf-8'))['parts']
    params=json.loads((root/'config/inertial_parameters.json').read_text('utf-8'))
    robot=ET.parse(root/'urdf/TsenseI_hand_right_v3_2_withid.urdf').getroot();links={l.get('name'):l for l in robot.findall('link')};out=[]
    for row in source['rows']:
        name=row['link'];m=float(row['mass_g'])/1000;assert np.isfinite(m) and m>0
        ps=[p for p in parts if p['included'] and p['target_link']==name];volume=sum(p['unit_mass'] for p in ps)
        com=sum(np.array(p['center_m'])*p['unit_mass'] for p in ps)/volume;I=np.zeros((3,3))
        for p in ps:
            d=np.array(p['center_m'])-com;I+=np.array(p['unit_inertia'])+p['unit_mass']*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
        I*=m/volume;I=(I+I.T)*.5;eig=np.linalg.eigvalsh(I);assert eig[0]>0 and eig[-1]<=eig[0]+eig[1]+1e-14
        el=links[name].find('inertial');el.find('mass').set('value',format(m,'.17g'));el.find('origin').set('xyz',' '.join(format(x,'.17g') for x in com))
        el.find('inertia').attrib={k:format(float(I[a,b]),'.17g') for k,a,b in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]}
        old=next(p for p in params['links'] if p['link']==name);old.update({'mass_kg':m,'mass_g':row['mass_g'],'center_of_mass_link_m':com.tolist(),
            'inertia_at_COM_link_axes_kg_m2':I.tolist(),'principal_moments_kg_m2':eig.tolist(),'equivalent_density_kg_m3':m/volume})
        row['mass_kg']=m;out.append({'link':name,'positive_definite':True,'mass_kg':m})
    ET.indent(robot,space='  ');ET.ElementTree(robot).write(root/'urdf/TsenseI_hand_right_v3_2_withid.urdf',encoding='utf-8',xml_declaration=True)
    ros=copy.deepcopy(robot)
    for me in ros.iter('mesh'):me.set('filename','package://tsensei_hand_right_raised5mm_20_outputs_description/'+me.get('filename').removeprefix('../'))
    ET.ElementTree(ros).write(root/'urdf/TsenseI_hand_right_ros.urdf',encoding='utf-8',xml_declaration=True)
    params['total_mass_kg']=sum(x['mass_kg'] for x in source['rows'])
    for path,data in [(root/'config/measured_link_masses.json',source),(root/'config/inertial_parameters.json',params),
                     (root/'checks/inertial_rebuild_validation.json',{'passed':True,'mass_kg':params['total_mass_kg'],'link_checks':out,'note':'Checks/report images in the delivered archive are snapshots of the original delivered mass values; rerun full package validation after subsequent edits.'})]:
        path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    return out

if __name__=='__main__':
    print('Rebuilt',len(run(Path(__file__).resolve().parents[1])),'link inertial blocks')
