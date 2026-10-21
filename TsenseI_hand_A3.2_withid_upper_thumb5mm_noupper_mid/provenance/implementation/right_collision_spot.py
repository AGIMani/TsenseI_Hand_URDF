from common import *
import fcl, argparse, time, collections, itertools

def signatures(robot):
    root=next(n for n in fk(robot) if n not in {j.find('child').get('link') for j in robot.findall('joint')})
    result={root:()};pending=robot.findall('joint')
    while pending:
        progressed=False
        for j in pending[:]:
            par=j.find('parent').get('link')
            if par not in result:continue
            result[j.find('child').get('link')]=result[par]+((j.get('name'),) if j.get('type')!='fixed' else ())
            pending.remove(j);progressed=True
        assert progressed
    return result

class Checker:
    def __init__(self,mode):
        self.mode=mode;self.records=[r for r in read(OUT/'config/structure_mesh_manifest.json')['meshes'] if r['urdf_collision_enabled']]
        self.robot=ET.parse(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf').getroot()
        self.source=ET.parse(OUT/'provenance/right_full_linkage_AUDIT_ONLY.urdf.txt').getroot()
        self.loops=read(OUT/'config/nonlinear_loops.json')['loops'];self.mapping=read(OUT/'config/name_mapping.json')['output_joints'];self.reverse={v:k for k,v in self.mapping.items()}
        self.target_zero=fk(self.robot);self.source_zero=fk(self.source);self.BC=np.eye(4)
        self.triangles=[];self.models=[];self.objects=[];self.local_bounds=[];self.containment_meshes=[]
        for r in self.records:
            tris=stl(OUT/r['mesh'])['vertices'].astype(float)
            verts,faceind=np.unique(tris.reshape(-1,3),axis=0,return_inverse=True)
            model=fcl.BVHModel();model.beginModel(len(verts),len(tris));model.addSubModel(verts,faceind.reshape(-1,3).astype(np.int32));model.endModel()
            self.models.append(model);self.objects.append(fcl.CollisionObject(model));self.triangles.append(tris)
            welded,inv=np.unique(np.round(tris.reshape(-1,3),8),axis=0,return_inverse=True)
            mesh=trimesh.Trimesh(welded,inv.reshape(-1,3),process=False);self.containment_meshes.append(mesh if mesh.is_watertight and mesh.is_winding_consistent else None)
            bounds=np.array([tris.min(axis=(0,1)),tris.max(axis=(0,1))]);self.local_bounds.append(np.array([[bounds[x,0],bounds[y,1],bounds[z,2]] for x in [0,1] for y in [0,1] for z in [0,1]]))
        sig=signatures(self.source if mode=='nonlinear' else self.robot);key='source_link' if mode=='nonlinear' else 'target_link'
        self.allowed=np.zeros((len(self.records),len(self.records)),bool)
        for i,j in itertools.combinations(range(len(self.records)),2):self.allowed[i,j]=sig[self.records[i][key]]!=sig[self.records[j][key]]
        self.adjacent={frozenset([j.find('parent').get('link'),j.find('child').get('link')]) for j in self.robot.findall('joint')}
        self.narrowphase_calls=0;self.transforms=[];self.cache={};self.cached_pair_queries=0

    def pose(self,q):
        if self.mode=='nonlinear':frames=fk(self.source,nonlinear({self.reverse[n]:v for n,v in q.items()},self.loops))
        else:frames=fk(self.robot,q)
        mins=[];maxs=[];self.transforms=[]
        for i,r in enumerate(self.records):
            if self.mode=='nonlinear':m=self.BC@frames[r['source_link']]@np.linalg.inv(self.source_zero[r['source_link']])@np.linalg.inv(self.BC)@self.target_zero[r['target_link']]
            else:m=frames[r['target_link']]
            self.transforms.append(m);self.objects[i].setTransform(fcl.Transform(m[:3,:3].copy(),m[:3,3].copy()))
            b=self.local_bounds[i]@m[:3,:3].T+m[:3,3];mins.append(b.min(axis=0));maxs.append(b.max(axis=0))
        mins=np.asarray(mins);maxs=np.asarray(maxs)
        overlap=np.all(mins[:,None,:]<=maxs[None,:,:]+1e-8,axis=2)&np.all(maxs[:,None,:]>=mins[None,:,:]-1e-8,axis=2)&self.allowed
        hits=[];touches=[]
        for i,j in np.argwhere(overlap):
            mi=self.transforms[i];mj=self.transforms[j]
            relative=np.eye(4);relative[:3,:3]=mi[:3,:3].T@mj[:3,:3];relative[:3,3]=mi[:3,:3].T@(mj[:3,3]-mi[:3,3])
            cachekey=(int(i),int(j),np.round(relative,10).tobytes())
            if cachekey in self.cache:
                self.cached_pair_queries+=1;state,cached=self.cache[cachekey]
                if state=='touch':touches.append((int(i),int(j)))
                elif state=='hit':
                    hit=dict(cached);point=np.array(hit.pop('contact_local_i_m'));hit['contact_position_base_mm']=((mi[:3,:3]@point+mi[:3,3])*1000).tolist();hits.append({'pair':[int(i),int(j)],**hit})
                continue
            self.narrowphase_calls+=1;result=fcl.CollisionResult();fcl.collide(self.objects[i],self.objects[j],fcl.CollisionRequest(enable_contact=True,num_max_contacts=32),result)
            if result.is_collision:
                hit=self.crossing(i,j,result.contacts)
                if hit is None and len(result.contacts)==32:
                    result=fcl.CollisionResult();fcl.collide(self.objects[i],self.objects[j],fcl.CollisionRequest(enable_contact=True,num_max_contacts=256),result);hit=self.crossing(i,j,result.contacts)
                if hit is None:touches.append((int(i),int(j)));self.cache[cachekey]=('touch',None);continue
            else:
                hit=self.containment(i,j,mins,maxs)
                if not hit:self.cache[cachekey]=('clear',None);continue
            cached=dict(hit);point=np.asarray(cached.pop('contact_position_base_mm'))/1000;cached['contact_local_i_m']=(mi[:3,:3].T@(point-mi[:3,3])).tolist();self.cache[cachekey]=('hit',cached)
            hits.append({'pair':[int(i),int(j)],**hit})
        return hits,touches

    def containment(self,i,j,mins,maxs):
        for small,big in [(i,j),(j,i)]:
            tm=self.containment_meshes[big]
            if tm is None or not(np.all(mins[small]>=mins[big]-1e-8) and np.all(maxs[small]<=maxs[big]+1e-8)):continue
            m=self.transforms[small];point=self.triangles[small][0,0]@m[:3,:3].T+m[:3,3];mb=self.transforms[big];local=(point-mb[:3,3])@mb[:3,:3]
            if bool(tm.contains(local.reshape(1,3))[0]):return {'triangle_plane_crossing_margin_mm':0.,'contact_position_base_mm':(point*1000).tolist(),'triangle_ids':[-1,-1],'type':'closed_mesh_containment'}
        return None

    def crossing(self,i,j,contacts):
        best=None
        for c in contacts:
            assert id(c.o1) in [id(self.models[i]),id(self.models[j])] and id(c.o2) in [id(self.models[i]),id(self.models[j])]
            a,b=(c.b1,c.b2) if id(c.o1)==id(self.models[i]) else (c.b2,c.b1)
            if a<0 or b<0:continue
            ti=self.triangles[i][a]@self.transforms[i][:3,:3].T+self.transforms[i][:3,3];tj=self.triangles[j][b]@self.transforms[j][:3,:3].T+self.transforms[j][:3,3]
            ni=np.cross(ti[1]-ti[0],ti[2]-ti[0]);nj=np.cross(tj[1]-tj[0],tj[2]-tj[0])
            if np.linalg.norm(ni)<1e-15 or np.linalg.norm(nj)<1e-15:continue
            ni/=np.linalg.norm(ni);nj/=np.linalg.norm(nj);di=(tj-ti[0])@ni;dj=(ti-tj[0])@nj
            margin=float(min(di.max(),-di.min(),dj.max(),-dj.min()))
            if margin<=5e-6:continue
            hit={'triangle_plane_crossing_margin_mm':margin*1000,'contact_position_base_mm':(np.array(c.pos)*1000).tolist(),'triangle_ids':[int(a),int(b)],'type':'proper_surface_crossing'}
            if best is None or margin>best['triangle_plane_crossing_margin_mm']/1000:best=hit
        return best

def samples(robot):
    js=[j for j in robot.findall('joint') if j.get('type')=='revolute'];names=[j.get('name') for j in js];byname={j.get('name'):j for j in js}
    flex=[n for n in names if not('swing' in n or 'rota' in n)];swings=[n for n in names if 'swing' in n];rotations=[n for n in names if 'rota' in n]
    rows=[('zero',{})]
    for j in js:
        lo,hi=[float(j.find('limit').get(k)) for k in ['lower','upper']]
        for k,v in enumerate(np.linspace(lo,hi,21)):rows.append((f'one_{j.get("name")}_{k:02}',{j.get('name'):float(v)}))
    for k,v in enumerate(np.linspace(0,1.57,31)):rows.append((f'all_flexion_{k:02}',{n:float(v) for n in flex}))
    for bend in [0.,.785,1.57]:
        for a in np.linspace(0,2.09,9):
            for b in np.linspace(-1.74,0,9):rows.append((f'thumb_grid_{a:.5f}_{b:.5f}_{bend:.3f}',{rotations[0]:float(a),rotations[1]:float(b),'right_hand_thumb_MCP_joint':bend,'right_hand_thumb_PIP_joint':bend}))
    for a,b in zip(swings,swings[1:]):
        for bend in [0.,.785,1.57]:
            for s in np.linspace(0,.35,11):
                q={n:bend for n in flex};q.update({a:float(s),b:float(-s)});rows.append((f'close_{a}_{b}_{bend:.3f}_{s:.5f}',q))
    rng=np.random.default_rng(20261013)
    for k in range(256):rows.append((f'random_{k:03}',{j.get('name'):float(rng.uniform(float(j.find('limit').get('lower')),float(j.find('limit').get('upper')))) for j in js}))
    unique=[];seen=set()
    for label,q in rows:
        q={n:v for n,v in q.items() if abs(v)>1e-12};key=tuple(sorted((n,round(v,12)) for n,v in q.items()))
        if key not in seen:seen.add(key);unique.append((label,q))
    return unique

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=['rigid_20','nonlinear'],required=True);parser.add_argument('--smoke',action='store_true');args=parser.parse_args();start=time.time();c=Checker(args.mode);poses=samples(c.robot)
    if args.smoke:poses=poses[:3]
    print(f'{args.mode}: {len(c.records)} meshes, {len(poses)} poses',flush=True)
    pairs={};posehits=[];zero=set();touchcounts=collections.Counter();last=time.time()
    for k,(label,q) in enumerate(poses):
        hits,touches=c.pose(q);posehits.append({'label':label,'q_output_rad':q,'crossing_pairs':[h['pair'] for h in hits]})
        for p in touches:touchcounts[p]+=1
        for h in hits:
            key=tuple(h['pair']);rr=[c.records[i] for i in key];adj=frozenset(r['target_link'] for r in rr) in c.adjacent;same=rr[0]['target_link']==rr[1]['target_link']
            row=pairs.setdefault(key,{'pair':h['pair'],'meshes':[r['name'] for r in rr],'part_ids':[r['native_part_id'] for r in rr],'target_links':[r['target_link'] for r in rr],'source_links':[r['source_link'] for r in rr],'categories':[r['category'] for r in rr],'adjacent_output_links':adj,'same_output_link':same,'excluded_by_planning_SRDF':adj or same,'includes_thumb':any('thumb' in r['name'] for r in rr),'baseline_crossing':k==0,'first_label':label,'first_q_output_rad':q,'sample_count':0,'max_triangle_plane_crossing_margin_mm':0.})
            row['sample_count']+=1
            if k==0:zero.add(key)
            if h['triangle_plane_crossing_margin_mm']>=row['max_triangle_plane_crossing_margin_mm']:row.update({'max_triangle_plane_crossing_margin_mm':h['triangle_plane_crossing_margin_mm'],'representative_label':label,'representative_q_output_rad':q,'representative_contact_base_mm':h['contact_position_base_mm'],'type':h['type']})
        if time.time()-last>15 or k==len(poses)-1:
            print(f'{args.mode}: {k+1}/{len(poses)}; {len(pairs)} crossing pairs; {time.time()-start:.1f}s',flush=True);last=time.time()
    rows=sorted(pairs.values(),key=lambda p:(p['excluded_by_planning_SRDF'],not p['includes_thumb'],-p['sample_count']))
    result={'mode':args.mode,'model_id':read(OUT/'config/nonlinear_loops.json')['model_id'],'method':'FCL triangle BVH, native detailed surfaces; two-sided triangle-plane crossing threshold 0.005 mm, with closed/watertight whole-AABB containment query. Different rigid motion groups checked; direct adjacencies remain in raw evidence.',
        'scope':'Discrete sampled interference audit; not a continuous collision certificate, not penetration-depth or volume measurement; no hardware assembly acceptance.',
        'pose_count':len(poses),'collision_mesh_count':len(c.records),'narrowphase_calls':c.narrowphase_calls,'cached_relative_pair_queries':c.cached_pair_queries,'relative_transform_cache_rounding':1e-10,'baseline_crossing_pair_count':len(zero),'distinct_crossing_pair_count':len(rows),'nonadjacent_output_crossing_pair_count':sum(not p['excluded_by_planning_SRDF'] for p in rows),'thumb_nonadjacent_output_crossing_pair_count':sum(p['includes_thumb'] and not p['excluded_by_planning_SRDF'] for p in rows),'sampled_collision_free':not rows,'planning_sampled_collision_free':not any(not p['excluded_by_planning_SRDF'] for p in rows),'poses_with_crossings':sum(bool(p['crossing_pairs']) for p in posehits),'fcl_version':fcl.__version__,'elapsed_s':time.time()-start,'pairs':rows,'poses':posehits,'tangency_or_subthreshold_pair_count':len(touchcounts),'watertight_containment_meshes':sum(m is not None for m in c.containment_meshes),'main_URDF_sha256':sha(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf')}
    save(OUT/'checks'/('collision_'+args.mode+('_smoke' if args.smoke else '')+'.json'),result)
    print(json.dumps({k:v for k,v in result.items() if k not in ['pairs','poses']},ensure_ascii=False),flush=True)
    for p in rows[:8]:print(p['meshes'],p['first_label'],p['sample_count'],flush=True)

if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
