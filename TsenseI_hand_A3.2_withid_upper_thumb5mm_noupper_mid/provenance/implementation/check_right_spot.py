from common import *
from right_collision_spot import Checker
import time

def main():
    source=read(LEFT/'checks/collision_rigid_20.json');selected=[source['poses'][i] for i in [0,7,21,65,420,507]];records=[];start=time.time()
    for mode in ['rigid_20','nonlinear']:
        c=Checker(mode)
        for pose in selected:
            q={renamed(n):v for n,v in pose['q_output_rad'].items()};hits,touches=c.pose(q);nonadj=[]
            for hit in hits:
                a,b=[c.records[i] for i in hit['pair']];group=frozenset([a['target_link'],b['target_link']])
                if a['target_link']!=b['target_link'] and group not in c.adjacent:nonadj.append(hit['pair'])
            records.append({'mode':mode,'left_pose_label':pose['label'],'right_q_rad':q,'crossing_pairs':[h['pair'] for h in hits],'nonadjacent_crossing_pairs':nonadj,'tangency_pair_count':len(touches)})
            print(mode,pose['label'],'raw',len(hits),'nonadjacent',len(nonadj),flush=True)
    result={'checked':True,'right_collision_meshes':367,'mode_pose_queries':len(records),'method':'Fresh right-hand FCL native triangle BVH spot checks, same 0.005mm crossing filter and closed containment logic as source','scope':'Independent representative checks; full source 1011-pose results are mirrored by validated isometry, not fully rerun FCL','elapsed_s':time.time()-start,'records':records,'main_URDF_sha256':sha(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf')}
    save(OUT/'checks/right_FCL_spot_validation.json',result)
    print('Right FCL spot audit complete',flush=True)

if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
