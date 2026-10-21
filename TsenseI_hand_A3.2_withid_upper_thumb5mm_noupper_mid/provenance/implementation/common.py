from pathlib import Path
import sys,json,stat,struct,hashlib,xml.etree.ElementTree as ET
W=Path(__file__).absolute().parent;ROOT=W.parents[2]
LEFT_PACKAGE='tsensei_hand_raised5mm_20_outputs_description'
PACKAGE='tsensei_hand_right_raised5mm_20_outputs_description'
LEFT=ROOT/'outputs/urdf_raised_5mm_20261011/TsenseI_20关节_完整模型_20261013_r03'/LEFT_PACKAGE
DELIVERY=ROOT/'outputs/urdf_raised_5mm_20261011/TsenseI_右手_20关节_完整模型_20261013_r03'
OUT=DELIVERY/PACKAGE
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'work/urdf_real_control_20261010/analysis_dependencies'))
sys.path.insert(0,str(ROOT/'work/urdf_real_control_20261010/urdf_tree_dependencies'))
sys.path.insert(0,str(ROOT/'work/urdf_no_raise_20261011/middle_dip_standard/structure_collision/dependencies'))
import numpy as np,trimesh,yaml
from scipy.spatial.transform import Rotation
# Palm width is Y. The mounting-hole centroid has Y=4mm; keep its pattern.
S=np.diag([1.,-1.,1.]);D=np.eye(4);D[:3,:3]=S
SHIFT=np.array([0.,.008,0.]);F=D.copy();F[:3,3]=SHIFT
DTYPE=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
def guard(path):
    p=Path(path).absolute()
    for a in [p,*p.parents]:
        if a.exists() and (a.is_symlink() or getattr(a.lstat(),'st_file_attributes',0)&stat.FILE_ATTRIBUTE_REPARSE_POINT):raise RuntimeError('Linked path prohibited: '+str(a))
    return p
def read(p):return json.loads(Path(p).read_text('utf-8-sig'))
def write(p,t):p=guard(p);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(t,encoding='utf-8')
def save(p,v):write(p,json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def nums(v):return ' '.join(format(float(x),'.17g') for x in v)
def matrix(v):return np.array(v).reshape(4,4).T
def flat(m):return np.asarray(m).T.flatten().tolist()
def renamed(s):
    return s.replace(LEFT_PACKAGE,PACKAGE).replace('left_hand','right_hand').replace('Left_hand','Right_hand').replace('TsenseI_hand_left','TsenseI_hand_right').replace('withid_left_raised5mm','withid_right_raised5mm').replace('meshes/left/','meshes/right/')
def source_name(s):return 'base_link' if s=='hand_base' else 'rh_source_'+s
def names(value):
    if isinstance(value,str):return renamed(value)
    if isinstance(value,list):return [names(x) for x in value]
    if isinstance(value,dict):return {renamed(k):names(v) for k,v in value.items()}
    return value
def origin(e):
    m=np.eye(4)
    if e is not None:m[:3,:3]=Rotation.from_euler('xyz',list(map(float,e.get('rpy','0 0 0').split()))).as_matrix();m[:3,3]=list(map(float,e.get('xyz','0 0 0').split()))
    return m
def set_origin(e,m):e.set('xyz',nums(m[:3,3]));e.set('rpy',nums(Rotation.from_matrix(m[:3,:3]).as_euler('xyz')))
def fk(r,q=None):
    q=q or {};children={j.find('child').get('link') for j in r.findall('joint')};roots=[l.get('name') for l in r.findall('link') if l.get('name') not in children];assert len(roots)==1
    frames={roots[0]:np.eye(4)};pending=r.findall('joint')
    while pending:
        progressed=False
        for j in pending[:]:
            par=j.find('parent').get('link')
            if par not in frames:continue
            move=np.eye(4);a=j.find('axis');v=q.get(j.get('name'),0.)
            if j.get('type') in ['revolute','continuous']:move[:3,:3]=Rotation.from_rotvec(np.array(list(map(float,a.get('xyz').split())))*v).as_matrix()
            elif j.get('type')=='prismatic':move[:3,3]=np.array(list(map(float,a.get('xyz').split())))*v
            frames[j.find('child').get('link')]=frames[par]@origin(j.find('origin'))@move;pending.remove(j);progressed=True
        assert progressed
    return frames
def stl(p):
    b=Path(p).read_bytes();n=struct.unpack_from('<I',b,80)[0];assert len(b)==84+50*n
    return np.frombuffer(b,dtype=DTYPE,offset=84,count=n)
def write_stl(p,a):p=guard(p);p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b'TsenseI right-hand reflection; winding corrected; metres'.ljust(80,b'\0')+struct.pack('<I',len(a))+a.tobytes())
def xmlwrite(p,r):guard(p);ET.indent(r,space='  ');ET.ElementTree(r).write(p,encoding='utf-8',xml_declaration=True)
def inertia(e):return np.array([[float(e.get('ixx')),float(e.get('ixy')),float(e.get('ixz'))],[float(e.get('ixy')),float(e.get('iyy')),float(e.get('iyz'))],[float(e.get('ixz')),float(e.get('iyz')),float(e.get('izz'))]])
def set_inertia(e,I):e.attrib={k:format(float(I[a,b]),'.17g') for k,a,b in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]}
def nonlinear(q,loops):
    state=dict(q)
    for row in loops:
        a,h,b,n,u=[np.asarray(row[k]) for k in ['A0_parent_m','H_parent_m','B0_parent_m','axis_parent','slide_axis_parent']]
        bq=h+Rotation.from_rotvec(n*state.get(row['output_joint'],0.)).apply(b-h);d=a-bq;dot=float(d@u);disc=dot*dot-float(d@d)+row['rod_length_m']**2
        assert disc>=-1e-12
        s=-dot+row['assembly_root_sign']*np.sqrt(max(0.,disc));v=bq-a-u*s;z=b-a
        state[row['slider_joint']]=float(s);state[row['rod_joint']]=float(np.arctan2(n@np.cross(z,v),z@v))
    return state
