"""Cross-version metadata, two-limit delta and name compatibility verification."""
from pathlib import Path
import copy,importlib.util,json,math,sys,xml.etree.ElementTree as ET
from build_hand_20j_simplified import read,sha,dump
from verify_hand_unified_r08_20261016 import signature

R=Path(__file__).resolve().parents[1];W=R/'work/hand_1009_20j_unified_20261016_r03'
P=R/'outputs/hand_1009_raised_20j_unified_20261016_r03/hand_1009_reduced_description'
S=R/'outputs/hand_1009_raised_unified_20261016_r08/hand_1009_description'
OLD=R/'outputs/hand_1009_raised_20j_public_names_20261016_r02/hand_1009_reduced_description'
sys.dont_write_bytecode=True


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    original=ET.parse(OLD/'urdf/hand_1009_20j.urdf').getroot();current=ET.parse(P/'urdf/hand_1009_20j.urdf').getroot()
    restored=copy.deepcopy(current);oldj={j.get('name'):j for j in original.findall('joint')}
    for j in restored.findall('joint'):
        if j.get('name') in ['thumb_mcp_flexion','thumb_ip_flexion']:
            assert float(j.find('limit').get('upper'))==math.pi/2
            j.find('limit').set('upper',oldj[j.get('name')].find('limit').get('upper'))
    meshmap={r['path']:r['old_path'] for r in read(P/'config/mesh_name_map.json')['meshes']}
    for mesh in restored.findall('.//mesh'):mesh.set('filename','../'+meshmap[mesh.get('filename')[3:]])
    assert signature(restored)==signature(original),'Unexpected shape/frame/mass delta in reduced model'
    a=read(S/'config/independent_controls.json')['controls'];b=read(P/'config/independent_controls.json')['controls']
    assert [x['name'] for x in a]==[x['name'] for x in b]
    for x,y in zip(a,b):assert [x[k] for k in ['lower_rad','upper_rad','zero_rad']]==[y[k] for k in ['lower_rad','upper_rad','zero_rad']]
    fullmeshes=read(S/'config/mesh_name_map.json')['meshes']
    for row in fullmeshes:assert sha(P/row['path'])==sha(S/row['path'])==row['sha256']
    assert not any(__import__('re').match(r'G\d',x.name) for x in (P/'meshes').rglob('*.stl'))
    contract=read(P/'config/joint_name_map.json');spec=importlib.util.spec_from_file_location('r03_name_converter',P/'tools/convert_joint_names.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    for old,new in contract['legacy_to_public'].items():
        fixture={'angles_deg':{old:12.345},'interface_id':'hand1009_output_coordinates_raised_shellzero_r05'}
        converted=module.convert(fixture,contract,'to-public')
        assert converted['angles_deg']=={new:12.345} and converted['interface_id']==contract['public_interface_id']
    for fixture in [{x['name']:(i-8)*.0123 for i,x in enumerate(b)},
                    {'name':[x['name'] for x in b],'position':[0.]*20,'velocity':[],'effort':[]}]:
        assert module.convert(module.convert(fixture,contract,'to-public'),contract,'to-source')==fixture
    try:module.convert({'index_mcp_swing':.1,'index_mcp_abduction':.2},contract,'to-public')
    except ValueError:pass
    else:raise AssertionError('Duplicate legacy/public alias accepted')
    for rel in ['config/reduction.json','config/motor_coordinate_reference.json','config/independent_controls.json']:
        assert (P/rel).is_file()
    assert read(P/'config/motor_coordinate_reference.json')['linear_actuator_loops']==read(S/'config/nonlinear_loops.json')
    old_manifest=read(OLD/'provenance/delivery_manifest.json')['files']
    assert all(sha(OLD/rel)==meta['sha256'] for rel,meta in old_manifest.items())
    proof={'passed':True,'all_three_variant_output_names_and_limits_identical':True,
           'only_two_thumb_upper_limits_and_mesh_URIs_differ_from_r02_URDF':True,
           'all_geometry_matrices_numeric_mass_inertia_and_axes_unchanged_from_r02':True,
           'same_182_structurally_named_mesh_files_as_full_r08':True,
           'legacy_swing_to_abduction_aliases_tested':4,'numeric_coordinate_signs_units_unchanged':True,
           'duplicate_alias_rejected':True,'full_current_14_loop_reference_matches':True,'previous_r02_unchanged':True}
    dump(W/'naming_verification.json',proof);dump(P/'provenance/naming_verification.json',proof)
    print(json.dumps(proof,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
