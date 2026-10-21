#!/usr/bin/env python3
"""CAD branch geometry only. Twenty output names; fourteen linear drive mappings."""
import argparse, json, math
from pathlib import Path
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def add(a,b):return [x+y for x,y in zip(a,b)]
def sub(a,b):return [x-y for x,y in zip(a,b)]
def scale(a,s):return [x*s for x in a]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def norm(a):return math.sqrt(dot(a,a))
def rotate(v,n,q):return add(add(scale(v,math.cos(q)),scale(cross(n,v),math.sin(q))),scale(n,dot(n,v)*(1-math.cos(q))))
def solve(row,q):
    h,a,b,n,u=[row[k] for k in ['H_parent_m','A0_parent_m','B0_parent_m','axis_parent','slide_axis_parent']]
    bq=add(h,rotate(sub(b,h),n,q));d=sub(a,bq);du=dot(d,u);disc=du*du-dot(d,d)+row['rod_length_m']**2
    if disc < -1e-12:raise ValueError('No closed CAD assembly at this angle')
    s=-du+row['assembly_root_sign']*math.sqrt(max(0.,disc));am=add(a,scale(u,s));rod=sub(am,bq);den=dot(rod,u)
    jac=None if abs(den)<1e-12 else dot(rod,cross(n,sub(bq,h)))/den
    return {'raw_displacement_m':s,'jacobian_m_per_rad':jac,'closure_error_m':abs(norm(rod)-row['rod_length_m'])}
def forward(row,q):
    value=solve(row,q);value['signed_mover_displacement_m']=value['raw_displacement_m']-solve(row,0.)['raw_displacement_m'];value['output_angle_rad']=q
    value['required_motion_mm']=abs(value['signed_mover_displacement_m'])*1000
    return value
def inverse(row,s,bounds=None):
    bounds=bounds or row['review_output_bounds_rad'];lo,hi=bounds;raw=s+solve(row,0.)['raw_displacement_m']
    h,a,b,n,u=[row[k] for k in ['H_parent_m','A0_parent_m','B0_parent_m','axis_parent','slide_axis_parent']]
    v=sub(add(a,scale(u,raw)),h);w=sub(b,h);alpha=2*dot(v,w);beta=2*dot(v,cross(n,w));gamma=dot(v,v)+dot(w,w)-row['rod_length_m']**2;rho=math.hypot(alpha,beta)
    if rho<1e-18 or abs(gamma/rho)>1+1e-9:raise ValueError('No inverse CAD assembly for this stroke')
    phi=math.atan2(beta,alpha);delta=math.acos(max(-1.,min(1.,gamma/rho)));choices=[]
    for candidate in [phi-delta,phi+delta]:
        for turns in [-1,0,1]:
            q=candidate+turns*2*math.pi
            if lo-1e-8<=q<=hi+1e-8 and abs(forward(row,q)['signed_mover_displacement_m']-s)<1e-9 and not any(abs(q-old)<1e-8 for old in choices):choices.append(q)
    if len(choices)!=1:raise ValueError('Stroke has no unique angle on the selected nominal CAD branch')
    return choices[0]
def package_path():
    local=Path(__file__).absolute().parent.parent
    if (local/'config/nonlinear_loops.json').exists():return local
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('tsensei_hand_right_raised5mm_20_outputs_description'))
    except ImportError:raise RuntimeError('Pass --package to the extracted package folder')
def main():
    p=argparse.ArgumentParser();p.add_argument('--package',type=Path);p.add_argument('--joint',required=True);g=p.add_mutually_exclusive_group(required=True);g.add_argument('--angle-deg',type=float);g.add_argument('--stroke-mm',type=float);p.add_argument('--geometry-only',action='store_true',help='Permit exact 90-degree CAD query outside nominal 1.57-rad limit')
    args=p.parse_args();root=args.package or package_path();mapping=json.loads((root/'config/name_mapping.json').read_text('utf-8'))['output_joints'];loops=json.loads((root/'config/nonlinear_loops.json').read_text('utf-8'))['loops'];row=next((r for r in loops if mapping[r['output_joint']]==args.joint),None)
    if row is None:p.error('This joint has no linear motor mapping; six rotary outputs have no mover stroke')
    if args.angle_deg is not None:
        q=math.radians(args.angle_deg);bounds=row['connected_bounds_rad'] if args.geometry_only else row['review_output_bounds_rad']
        if not bounds[0]-1e-10<=q<=bounds[1]+1e-10:p.error('Angle outside selected range; use --geometry-only for a CAD-only exact 90-degree query')
    else:q=inverse(row,args.stroke_mm/1000)
    result=forward(row,q);result.update({'joint':args.joint,'output_angle_deg':math.degrees(q),'hardware_verified':False,'reference':'CAD zero; signed displacement along recorded slider axis'})
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
