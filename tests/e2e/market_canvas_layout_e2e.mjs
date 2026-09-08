import {spawn} from 'node:child_process';
import {createServer} from 'node:net';
import {mkdir,writeFile} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {chromium} from 'playwright';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
const arg=name=>{const i=process.argv.indexOf(name);return i<0?null:process.argv[i+1];};
let base=arg('--base'),server,browser;
const output=path.join(root,'output/market-operation/canvas-layout');
const request=async(route,body)=>{const r=await fetch(base+route,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});const j=await r.json();assert.ok(r.ok,JSON.stringify(j));return j;};
try{
  await mkdir(output,{recursive:true});
  if(!base){
    const port=await new Promise(resolve=>{const s=createServer();s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));});});
    base=`http://127.0.0.1:${port}`;
    server=spawn(arg('--server')||path.join(root,'build/macos-release/tests/run_gui_server'),['--host','127.0.0.1','--port',String(port),'--data-dir',path.join(root,'data')],{cwd:root,stdio:'ignore'});
    for(let i=0;i<100;i++){try{if((await fetch(base+'/api/cases')).ok)break;}catch{}await new Promise(r=>setTimeout(r,100));}
    await request('/api/session/load_builtin',{case:'market_ieee118'});
  }
  const {boundary}=await request('/api/session/southern_market');assert.equal(boundary.buses.length,118);
  browser=await chromium.launch();const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.addInitScript(()=>{localStorage.setItem('hysim.tourDone.v1','1');localStorage.setItem('hysim.marketOperationMode','manual');sessionStorage.setItem('hysim.marketOperationStep','4');});
  await page.goto(base+'/xjtu/#market-operation');
  const ready=()=>page.waitForFunction(()=>document.querySelector('#marketTopology')?.dataset.layout==='elk-layered'&&document.querySelector('#marketTopology').getAttribute('aria-busy')==='false');
  await ready();const svg=page.locator('#marketTopology');
  const boxes=()=>svg.locator('[data-layout-box]').evaluateAll(nodes=>nodes.map(n=>({id:n.dataset.marketRef,box:JSON.parse(n.dataset.layoutBox)})));
  const separated=async()=>{const values=await boxes();for(let i=0;i<values.length;i++)for(let j=i+1;j<values.length;j++){const [x,y,w,h]=values[i].box,[u,v,a,b]=values[j].box;assert.ok(x+w<=u||u+a<=x||y+h<=v||v+b<=y,`${values[i].id} overlaps ${values[j].id}`);}return values.length;};
  const local=await separated();assert.ok(local>1&&local<=24);assert.equal(await svg.locator('.market-line-label').count(),0);
  const initial=await svg.getAttribute('viewBox');await page.getByRole('button',{name:'放大市场拓扑',exact:true}).click();
  const zoomed=await svg.getAttribute('viewBox');assert.notEqual(zoomed,initial);
  const visible=(await boxes()).map(n=>n.id);await page.locator('#marketCanvasEntity').selectOption(visible[1]);
  assert.equal(await svg.getAttribute('viewBox'),zoomed);
  if(await page.locator('#operationSlot option').count()){
    await page.locator('#marketCanvas-operationSlot').selectOption('40');
    assert.equal(await svg.getAttribute('viewBox'),zoomed);assert.deepEqual((await boxes()).map(n=>n.id),visible);
  }
  const outside=boundary.buses.filter(b=>!visible.includes(`buses:${b.id}`));
  await page.evaluate(({outside,inside})=>{HySimMarketCanvas.select(outside);HySimMarketCanvas.select(inside);},{outside:`buses:${outside[0].id}`,inside:visible[1]});
  await ready();assert.deepEqual((await boxes()).map(n=>n.id),visible);assert.equal(await svg.getAttribute('viewBox'),zoomed);
  await page.locator('#marketCanvasEntity').selectOption(`buses:${outside[0].id}`);
  await page.locator('#marketCanvasEntity').selectOption(`buses:${outside.at(-1).id}`);
  await ready();assert.ok(await svg.locator(`[data-market-ref="buses:${outside.at(-1).id}"]`).count());await separated();
  const line=boundary.branches.at(-1);await page.locator('#marketCanvasEntity').selectOption(`branches:${line.id}`);await ready();
  for(const id of [line.from_bus,line.to_bus])assert.equal(await svg.locator(`[data-market-ref="buses:${id}"]`).count(),1);
  await page.locator('#marketCanvasScope').selectOption('extended');await ready();
  const extended=await separated();assert.ok(extended>local&&extended<=80);
  const paths=await svg.locator('.market-line').evaluateAll(nodes=>nodes.map(n=>n.getAttribute('d')));
  assert.equal(new Set(paths).size,paths.length,'parallel branch routes must remain distinct');
  await page.locator('#marketCanvasLabels').check();await ready();await separated();
  assert.equal(await svg.locator('.market-bus-label').count(),extended);
  const overflow=await svg.locator('[data-layout-box]').evaluateAll(nodes=>nodes.flatMap(n=>{
    const [x,y,w,h]=JSON.parse(n.dataset.layoutBox);return [...n.querySelectorAll('.market-bus-label,.market-assets-label')].flatMap(t=>{const r=t.getBBox();return r.x>=x&&r.x+r.width<=x+w&&r.y>=y&&r.y+r.height<=y+h?[]:[{id:n.dataset.marketRef,text:t.textContent,box:[x,y,w,h],label:[r.x,r.y,r.width,r.height]}];});
  }));assert.deepEqual(overflow,[],'numeric labels must fit reserved node boxes');
  await page.locator('#marketCanvasLabels').uncheck();await page.locator('#marketCanvasScope').selectOption('local');
  await page.locator('#marketCanvasEntity').selectOption('buses:1');await page.getByRole('button',{name:'以所选设备为中心',exact:true}).click();await ready();
  await page.getByRole('button',{name:'适应市场拓扑',exact:true}).click();
  const regions=async()=>page.evaluate(()=>{const ids=['.market-canvas-toolbar','#marketTopology','.market-canvas-legend','#marketCanvasDetails'];return ids.map(id=>{const r=document.querySelector(id).getBoundingClientRect();return {top:r.top,bottom:r.bottom,height:r.height};});});
  const desktop=await regions();for(let i=1;i<desktop.length;i++)assert.ok(desktop[i].top>=desktop[i-1].bottom-1);assert.ok(desktop[1].height>=240);
  await page.locator('#marketCanvas').screenshot({path:path.join(output,'desktop.png')});
  await page.setViewportSize({width:390,height:844});await page.locator('#marketCanvas').scrollIntoViewIfNeeded();
  const mobile=await regions();for(let i=1;i<mobile.length;i++)assert.ok(mobile[i].top>=mobile[i-1].bottom-1);
  assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
  await page.locator('#marketCanvas').screenshot({path:path.join(output,'mobile.png')});
  await page.locator('[data-market-ref="buses:1"] .market-bus').click();assert.equal(await page.locator('#marketCanvasEntity').inputValue(),'buses:1');
  const reservoir=boundary.reservoirs[0];await page.locator('#marketCanvasEntity').selectOption(`reservoirs:${reservoir.id}`);await ready();
  for(const g of boundary.generators.filter(g=>reservoir.generators.includes(g.id)))assert.equal(await svg.locator(`[data-market-ref="buses:${g.bus}"].resource-focus`).count(),1);
  assert.deepEqual(errors,[]);await writeFile(path.join(output,'report.json'),JSON.stringify({local,extended,desktop,mobile,errors},null,2));
  console.log('Market Canvas layout passed: bounded ELK, nonoverlapping boxes/labels, stale-layout guard, stable view, branch endpoints, desktop/mobile.');
}finally{await browser?.close();if(server){server.kill('SIGTERM');await new Promise(resolve=>server.exitCode!==null?resolve():server.once('exit',resolve));}}
