from common import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

def image_data(package,robot,manifest,q):
    frames=fk(robot,q);tris=[];colors=[]
    for row in manifest:
        if row['category'] not in ['housing','support','rubber','rotary_motor']:continue
        v=stl(package/row['mesh'])['vertices'].astype(float);m=frames[row['target_link']];v=v@m[:3,:3].T+m[:3,3]
        # Draw the actual full triangle surfaces, sorted along viewing depth X.
        color={'housing':[.68,.74,.80],'support':[.29,.66,.47],'rubber':[.23,.25,.28],'rotary_motor':[.50,.53,.57]}[row['category']]
        n=np.cross(v[:,1]-v[:,0],v[:,2]-v[:,0]);length=np.linalg.norm(n,axis=1);n/=np.maximum(length[:,None],1e-30)
        light=.40+.60*np.abs(n@np.array([.85,.15,.50]));tris.append(v);colors.append(np.clip(light[:,None]*np.array(color),0,1))
    tris=np.concatenate(tris);colors=np.concatenate(colors);order=np.argsort(tris[:,:,0].mean(1));return tris[order][:,:,[1,2]]*1000,colors[order]

def main():
    lr=ET.parse(LEFT/'urdf/TsenseI_hand_left_v3_2_withid.urdf').getroot();rr=ET.parse(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf').getroot();lm=read(LEFT/'config/structure_mesh_manifest.json')['meshes'];rm=read(OUT/'config/structure_mesh_manifest.json')['meshes']
    combined={'left_hand_thumb_rota_block_joint':.627,'left_hand_thumb_rotaback_joint2':-.522,'left_hand_thumb_MCP_joint':.785,'left_hand_thumb_PIP_joint':.785}
    fig,axes=plt.subplots(2,2,figsize=(11,10),dpi=150);fig.patch.set_facecolor('#f2f4f6')
    for i,q in enumerate([{},combined]):
        for k,(p,r,m,pose,title) in enumerate([(LEFT,lr,lm,q,'Left r03 source'),(OUT,rr,rm,{renamed(n):v for n,v in q.items()},'Right geometric mirror')]):
            polys,colors=image_data(p,r,m,pose);ax=axes[i,k];ax.set_facecolor('#f2f4f6');ax.add_collection(PolyCollection(polys,facecolors=colors,edgecolors='none',rasterized=True));ax.set_xlim(-88,96);ax.set_ylim(-8,199);ax.set_aspect('equal');ax.set_xlabel('Base Y (mm)');ax.set_ylabel('Base Z (mm)');ax.set_title(title+' | '+('zero' if i==0 else 'thumb combination'),fontsize=11);ax.axvline(4,color='#9aa6b0',linestyle='--',linewidth=.7)
    fig.suptitle('20 outputs / 22 links / 952 g | mirror plane: base Y = 4 mm',fontsize=12);fig.tight_layout();path=guard(OUT/'preview/left_right_comparison.png');path.parent.mkdir(exist_ok=True);fig.savefig(path,bbox_inches='tight');plt.close(fig);print(path,flush=True)

if __name__=='__main__':main()
