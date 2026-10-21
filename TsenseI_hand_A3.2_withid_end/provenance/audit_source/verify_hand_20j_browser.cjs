// Use an isolated headless browser and the existing generic Robot Viewer loader.
const fs=require('fs'),path=require('path'),http=require('http'),assert=require('assert');
const {chromium}=require('C:/Users/ps/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const R=path.resolve('.'),W=path.resolve(process.argv[2]||path.join(R,'work/hand_1009_20j_reduction_20261016'));
const fixtures=JSON.parse(fs.readFileSync(path.join(W,'browser_fixtures.json'),'utf8'));
const P=fixtures.package,V=fixtures.screenshots,B=path.join(R,'work/hand_1009_20j_reduction_20261016/official_robot_viewer');
// Cache the official generic viewer locally; model resources are only read
// from loopback. The older custom hand viewer deliberately blocks new trees.
fs.mkdirSync(B,{recursive:true});
const main=fixtures.main||'urdf/hand_1009_20j.urdf',manifest=JSON.parse(fs.readFileSync(path.join(P,'config/model_manifest.json'),'utf8'));
const expectedLinks=fixtures.expected_links||21,expectedJoints=fixtures.expected_joints||20,expectedSliders=fixtures.expected_sliders||20;
const files=[main,...Object.keys(manifest.mesh_files)];
const proof={page_errors:[],console_errors:[],requests_failed:[],blocked_external_urls:[],screenshots:[],fixtures:[],viewer:'Official viewer.robotsfan.com generic URDF import, locally cached; no hand-linkage adapter'};
const server=http.createServer(async(req,res)=>{
 try{
  const url=new URL(req.url,'http://127.0.0.1'),inModel=url.pathname.startsWith('/__delivery/');
  const root=inModel?P:B,rel=decodeURIComponent(inModel?url.pathname.slice('/__delivery/'.length):url.pathname.slice(1)||'index.html');
  const file=path.resolve(root,rel);if(!file.startsWith(root+path.sep)&&file!==root){res.writeHead(403);return res.end()}
  if(!inModel&&!fs.existsSync(file)){
   const remote=await fetch('https://viewer.robotsfan.com/'+rel);if(!remote.ok){res.writeHead(remote.status);return res.end()}
   fs.mkdirSync(path.dirname(file),{recursive:true});fs.writeFileSync(file,Buffer.from(await remote.arrayBuffer()));
  }
  const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css','.json':'application/json','.svg':'image/svg+xml','.urdf':'application/xml','.stl':'application/octet-stream'};
  res.setHeader('Content-Type',types[path.extname(file)]||'application/octet-stream');fs.createReadStream(file).on('error',()=>{res.statusCode=404;res.end()}).pipe(res);
 }catch(e){res.writeHead(500);res.end(e.message)}
});
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));const url='http://127.0.0.1:'+server.address().port;
 const browser=await chromium.launch({headless:true,executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',args:['--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1440,height:1000},colorScheme:'dark'});
  await page.route('**/*',route=>{const u=route.request().url();if(u.startsWith(url)||u.startsWith('blob:')||u.startsWith('data:'))return route.continue();proof.blocked_external_urls.push(u);return route.abort()});
  page.on('pageerror',e=>proof.page_errors.push(e.message));page.on('requestfailed',r=>proof.requests_failed.push(r.url()));
  page.on('console',m=>{if(m.type()==='error'){proof.console_errors.push(m.text());process.stdout.write('Viewer error: '+m.text().slice(0,650)+'\n')}});
  await page.goto(url,{timeout:90000,waitUntil:'load'});
  await page.waitForFunction(()=>window.app?.fileHandler&&app.poseController,{timeout:45000});
  await page.waitForLoadState('networkidle',{timeout:60000});
  await page.waitForTimeout(300);
  process.stdout.write('Generic viewer ready; importing the reduced package.\n');
  const inventory=await page.evaluate(async({files,main})=>{
   const map=new Map();
   // Fetch only local delivery resources. All source meshes remain byte-identical.
   for(let start=0;start<files.length;start+=12){await Promise.all(files.slice(start,start+12).map(async p=>{
    const response=await fetch('/__delivery/'+p);if(!response.ok)throw Error('Resource unavailable: '+p);
    const f=new File([await response.arrayBuffer()],p.split('/').pop());map.set(p,f);
   }))}
   app.fileHandler.fileMap=map;await app.fileHandler.loadFile(map.get(main));
   return{loadedFiles:map.size};
  },{files,main});
  await page.waitForFunction(e=>app.currentModel?.links.size===e.links&&app.currentModel?.joints.size===e.joints,{links:expectedLinks,joints:expectedJoints},{timeout:90000});
  await page.waitForTimeout(1000);
  const loaded=await page.evaluate(()=>{let visualNodes=0,meshNodes=0,visualMeshes=0;app.currentModel.links.get('hand_base').threeObject.traverse(o=>{if(o.isURDFVisual){visualNodes++;o.traverse(c=>{if(c.isMesh&&c.geometry?.attributes?.position?.count>0)visualMeshes++})}if(o.isMesh)meshNodes++});return{links:app.currentModel.links.size,joints:app.currentModel.joints.size,
   names:[...app.currentModel.joints.keys()],genericSliderCount:document.querySelectorAll('input[data-joint]').length,
   specialHandSliderCount:document.querySelectorAll('[data-control-slider]').length,
   specialLinkageActive:!!app.poseController.handLinkage?.active,
   loadedVisualNodes:visualNodes,loadedVisualMeshes:visualMeshes,meshNodesIncludingCollisionAndAxes:meshNodes}});
  process.stdout.write(JSON.stringify(loaded)+'\n');
  if(loaded.genericSliderCount!==expectedSliders){
   process.stdout.write(JSON.stringify(await page.evaluate(()=>({inputs:[...document.querySelectorAll('input[type="range"]')].map(e=>({outer:e.outerHTML.slice(0,250)})),panel:document.querySelector('#joint-controls-panel')?.outerHTML.slice(0,1500),ui:!!app.jointControlsUI,modelIsMesh:app.currentModel.userData,methods:Object.getOwnPropertyNames(Object.getPrototypeOf(app.poseController))})))+'\n');
   await page.screenshot({path:path.join(V,'diagnostic-load.png')});
  }
  assert.equal(loaded.links,expectedLinks);assert.equal(loaded.joints,expectedJoints);assert.equal(loaded.genericSliderCount,expectedSliders);assert.equal(loaded.specialHandSliderCount,0);assert(!loaded.specialLinkageActive);
  assert.equal(loaded.loadedVisualNodes,303);assert.equal(loaded.loadedVisualMeshes,303);
  assert.deepEqual([...loaded.names].sort(),(fixtures.expected_joint_names||fixtures.controls.map(c=>c.name)).sort());
  for(const f of fixtures.fixtures){
   const compared=await page.evaluate(f=>{
    for(const[name,q]of Object.entries(f.pose))app.poseController.setJointValue(name,q);
    app.currentModel.links.get('hand_base').threeObject.updateWorldMatrix(true,true);
    const inv=app.currentModel.links.get('hand_base').threeObject.matrixWorld.clone().invert();let max=0;
    for(const[name,expected]of Object.entries(f.world)){
     const actual=inv.clone().multiply(app.currentModel.links.get(name).threeObject.matrixWorld).elements;
     for(let i=0;i<16;i++)max=Math.max(max,Math.abs(actual[i]-expected[i]));
    }
    return{max_matrix_error:max,positions:Object.fromEntries([...app.currentModel.joints].map(([n,j])=>[n,j.currentValue]))};
   },f);
   assert(compared.max_matrix_error<1e-8,JSON.stringify({name:f.name,...compared}));
   for(const[n,v]of Object.entries(f.pose))assert(Math.abs(v-compared.positions[n])<1e-10,n);
   proof.fixtures.push({name:f.name,max_matrix_error:compared.max_matrix_error});
   if(['zero','index_mcp_flexion_upper','thumb_base_rotation_1_upper'].includes(f.name)){
    const output=path.join(V,f.name+'.png');await page.screenshot({path:output});proof.screenshots.push(output);
   }
  }
  // Exercise the visible generic control, rather than using only a JS setter.
  const slider=page.locator('input[data-joint="middle_dip_flexion"]');
  const inputState=await slider.evaluate(e=>{e.value='0.7';e.dispatchEvent(new Event('input',{bubbles:true}));e.dispatchEvent(new Event('change',{bubbles:true}));return{value:e.value,min:e.min,max:e.max,step:e.step}});
  await page.waitForTimeout(150);
  const inputPosition=await page.evaluate(()=>app.currentModel.joints.get('middle_dip_flexion').currentValue);
  process.stdout.write(JSON.stringify({inputState,inputPosition})+'\n');
  assert(Math.abs(inputPosition-Number(inputState.value))<1e-8);
  proof.genericControlInputPassed=true;
  const screenshot=path.join(V,'generic_twenty_controls.png');await page.screenshot({path:screenshot});proof.screenshots.push(screenshot);
  assert.equal(proof.page_errors.length,0,proof.page_errors.join('\n'));
  assert(!proof.requests_failed.some(u=>u.startsWith(url)),'A local model/viewer resource failed');
  proof.passed=true;proof.inventory=inventory;proof.loaded=loaded;proof.source_model_id=manifest.source_model_id;
  proof.public_interface_id=manifest.public_interface_id||null;
  assert(!loaded.names.some(n=>n.endsWith('_swing'))||!manifest.public_interface_id,'Old side-joint names remain in public model');
  console.log(JSON.stringify(proof,null,2));
 }finally{
  fs.writeFileSync(path.join(W,'browser_verification.json'),JSON.stringify(proof,null,2));await browser.close();await new Promise(r=>server.close(r));
 }
})().catch(e=>{console.error(e.stack);server.close();process.exitCode=1});
