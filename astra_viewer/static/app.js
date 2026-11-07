'use strict';
const $ = id => document.getElementById(id);
const state = {
  mode:'live', source:'recent', photos:[], recent:[], sdPhotos:[], selected:null,
  selectedIds:new Set(), following:true, cursor:null, sdStart:0, sdTotal:0,
  sequence:-1, galleryBusy:false, sdLoaded:false, sdDirectories:[],
  online:false, first:true, liveBusy:false, recentLimit:60, interval:1,
  lastCapture:0, controls:{available:false}, armed:false, clientId:''
};
const titles = {
  live:['A live perspective.','Your camera, with detection in view.'],
  photos:['Every moment, in focus.','Browse recent browser captures or photos on the camera card.'],
  split:['The full picture.','Live detection and a selected photo, side by side.'],
  controller:['Camera in your hands.','Hold a control to move; let go to stop.']
};
const timeLabel = seconds => new Date(seconds*1000).toLocaleTimeString([], {hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false});
const dateLabel = seconds => new Date(seconds*1000).toLocaleString([], {month:'short',day:'numeric',hour:'2-digit',minute:'2-digit',second:'2-digit'});
async function api(path, options={}) {
  const response = await fetch(path, {...options, signal:AbortSignal.timeout(12000)});
  if(!response.ok) {let message=`Request failed (${response.status})`;try {message=(await response.json()).error||message;}catch{}throw new Error(message);}
  return response.json();
}
let toastTimer;
function toast(text) {$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,5000);}
function makeId(){return globalThis.crypto?.randomUUID?.()||`${Date.now()}-${Math.random().toString(16).slice(2)}`;}
function stopBeacon(){
  if(!state.clientId)return;
  const body=new Blob([JSON.stringify({client_id:state.clientId})],{type:'application/json'});
  try{navigator.sendBeacon('/api/control/stop',body);}catch{}
}
function disarmController(){
  stopMotion(false);
  if(state.armed){state.armed=false;api('/api/control/arm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({client_id:state.clientId,armed:false})}).catch(()=>{});}
  updateControlUI();
}
function setMode(mode) {
  if(state.mode==='controller'&&mode!=='controller')disarmController();
  state.mode=mode;
  $('workspace').className=`workspace mode-${mode}`;
  $('workspace').setAttribute('aria-labelledby',`tab-${mode}`);
  $('view-title').textContent=titles[mode][0];$('view-description').textContent=titles[mode][1];
  document.querySelectorAll('[data-mode]').forEach(b=>{b.setAttribute('aria-selected',String(b.dataset.mode===mode));b.tabIndex=b.dataset.mode===mode?0:-1;});
  try {localStorage.setItem('astra-mode',mode);}catch{}
}
document.querySelectorAll('[data-mode]').forEach(button=>{
  button.addEventListener('click',()=>setMode(button.dataset.mode));
  button.addEventListener('keydown',event=>{if(!['ArrowLeft','ArrowRight','Home','End'].includes(event.key))return;event.preventDefault();const modes=['live','photos','split','controller'];let index=modes.indexOf(state.mode);index=event.key==='Home'?0:event.key==='End'?3:(index+(event.key==='ArrowRight'?1:3))%4;setMode(modes[index]);$(`tab-${modes[index]}`).focus();});
});
function following(value) {state.following=value;$('follow').setAttribute('aria-pressed',String(value));$('follow').textContent=value?'↗ Following latest':'↗ Follow latest';}
function currentPhotoList(){return state.photos;}
function updateSelection() {
  const photo=state.selected;
  $('photo-image').hidden=!photo;$('photo-empty').hidden=!!photo;
  $('protect').disabled=!photo||photo.source==='sd';
  $('download').setAttribute('aria-disabled',String(!photo));
  if(!photo){$('photo-position').textContent='NO PHOTO SELECTED';$('photo-detail').textContent='Choose a photo from the gallery below';$('protect').textContent='☆ Keep';return;}
  if($('photo-image').src!==photo.url)$('photo-image').src=photo.url;
  $('photo-position').textContent=photo.name||photo.reference||photo.id.slice(0,8).toUpperCase();
  const objects=photo.objects||[];
  $('photo-detail').textContent=photo.source==='sd'?`${photo.name} · Camera SD card`:`${dateLabel(photo.captured)} · ${objects.length} ${objects.length===1?'object':'objects'}`;
  $('selection-caption').textContent=state.following&&state.source==='recent'?'Following the newest captured moment':(photo.name||`${dateLabel(photo.captured)} · ${photo.protected?'Kept':'Selected moment'}`);
  $('protect').textContent=photo.protected?'★ Kept':'☆ Keep';
  $('protect').setAttribute('aria-pressed',String(!!photo.protected));
  $('download').href=photo.source==='sd'?`${photo.url}?download=1`:(photo.downloadUrl||photo.url);
  $('download').setAttribute('download',photo.name||`astra-${photo.id}.jpg`);
  const index=state.photos.findIndex(p=>p.id===photo.id);
  $('previous').hidden=index<=0;$('next').hidden=index<0||index>=state.photos.length-1;
  document.querySelectorAll('.thumbnail-select').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.id===photo.id)));
  updateDownloadButton();
}
function select(photo, manual=true) {state.selected=photo;if(manual)following(false);updateSelection();}
function confidence(photo){const objs=photo.objects||[];return objs.length?Math.max(...objs.map(x=>Number(x.confidence)||0)): -1;}
function disposeRecent(photo){if(photo.url?.startsWith('blob:'))URL.revokeObjectURL(photo.url);if(photo.thumbnail?.startsWith('blob:'))URL.revokeObjectURL(photo.thumbnail);}
function trimRecent(){
  while(state.recent.length>state.recentLimit){
    const candidates=state.recent.filter(p=>!p.protected);
    const pool=candidates.length?candidates:state.recent;
    pool.sort((a,b)=>confidence(a)-confidence(b)||a.captured-b.captured);
    const remove=pool[0],index=state.recent.findIndex(p=>p.id===remove.id);
    state.recent.splice(index,1);disposeRecent(remove);
    if(state.selected?.id===remove.id)state.selected=null;
  }
}
async function thumbnailBlob(blob){
  let image,temporary;
  try{
    if(globalThis.createImageBitmap){image=await createImageBitmap(blob);}else{
      temporary=URL.createObjectURL(blob);image=new Image();image.src=temporary;await image.decode();
    }
    const scale=Math.min(1,288/image.width,162/image.height),canvas=document.createElement('canvas');
    canvas.width=Math.max(1,Math.round(image.width*scale));canvas.height=Math.max(1,Math.round(image.height*scale));
    canvas.getContext('2d',{alpha:false}).drawImage(image,0,0,canvas.width,canvas.height);
    return await new Promise(resolve=>canvas.toBlob(resolve,'image/jpeg',.72))||blob;
  }catch{return blob;}finally{if(image?.close)image.close();if(temporary)URL.revokeObjectURL(temporary);}
}
async function addRecent(blob,metadata){
  const now=Date.now()/1000;
  const objects=Array.isArray(metadata.objects)?metadata.objects:[];
  const thumbnail=await thumbnailBlob(blob),id=makeId();
  const photo={id,source:'recent',name:`astra-${new Date(now*1000).toISOString().replace(/[:.]/g,'-')}.jpg`,captured:Number(metadata.captured)||now,
    objects,confidence:objects.length?Math.max(...objects.map(x=>Number(x.confidence)||0)):null,
    reference:metadata.reference||'',url:URL.createObjectURL(blob),downloadUrl:'',thumbnail:URL.createObjectURL(thumbnail),protected:false,blob};
  photo.downloadUrl=photo.url;
  state.recent.push(photo);trimRecent();
  if(state.source==='recent'){
    state.photos=state.recent;
    if(state.following){state.selected=photo;}
    renderStrip();
  }
}
function renderStrip() {
  const strip=$('filmstrip'),oldLeft=strip.scrollLeft,oldWidth=strip.scrollWidth;
  const existing=new Map([...$('thumbnails').children].map(el=>[el.dataset.id,el]));
  const fragment=document.createDocumentFragment();
  state.photos.forEach(photo=>{
    let card=existing.get(photo.id),button,check;
    if(!card){
      card=document.createElement('div');card.className='thumb-card';card.dataset.id=photo.id;
      button=document.createElement('button');button.type='button';button.className='thumbnail thumbnail-select';button.dataset.id=photo.id;
      const img=document.createElement('img');img.src=photo.thumbnail;img.loading='lazy';img.alt=`Preview ${photo.name||dateLabel(photo.captured||Date.now()/1000)}`;
      const caption=document.createElement('span');caption.className='thumb-caption';
      const time=document.createElement('span');time.textContent=photo.source==='sd'?photo.name:timeLabel(photo.captured);
      const conf=document.createElement('span');conf.textContent=photo.source==='sd'?'SD':photo.confidence===null?'—':`${Math.round(photo.confidence*100)}%`;
      caption.append(time,conf);button.append(img,caption);
      button.addEventListener('click',()=>{const current=state.photos.find(p=>p.id===button.dataset.id);if(current)select(current);});
      check=document.createElement('input');check.type='checkbox';check.className='pick-photo';check.setAttribute('aria-label',`Select ${photo.name||'photo'} for download`);check.dataset.id=photo.id;
      check.addEventListener('change',()=>toggleChosen(photo.id,check.checked));
      card.append(button,check);
    }else{button=card.querySelector('.thumbnail-select');check=card.querySelector('.pick-photo');}
    button.setAttribute('aria-pressed',String(state.selected?.id===photo.id));
    check.checked=state.selectedIds.has(photo.id);card.classList.toggle('selected-for-download',check.checked);
    const star=card.querySelector('.thumb-protected');
    if(photo.protected&&!star){const mark=document.createElement('span');mark.className='thumb-protected';mark.textContent='★';button.append(mark);}else if(!photo.protected&&star)star.remove();
    fragment.append(card);
  });
  $('thumbnails').replaceChildren(fragment);
  $('load-older').hidden=!state.cursor;$('strip-empty').hidden=state.photos.length>0;
  $('strip-empty').firstChild.textContent=state.source==='recent'?'Recent annotated photos will collect here. Newest on the right.':'No photos loaded from this camera folder yet.';
  $('photo-count').textContent=state.source==='sd'?`${state.sdPhotos.length} / ${state.sdTotal}`:state.recent.length.toString();
  $('memory-caption').textContent=state.source==='recent'?`${state.recent.length} / ${state.recentLimit} in this browser tab`:`${state.sdPhotos.length} of ${state.sdTotal} camera photos loaded`;
  $('capture-label').textContent=state.source==='recent'?`Browser memory · every ${state.interval} s`:'Camera SD card · loaded on demand';
  $('gallery-footnote').textContent=state.source==='recent'?'Recent captures disappear when this browser tab closes.':'Camera photos stay on the camera SD card.';
  $('order-label').innerHTML=state.source==='recent'?'OLDEST <span class="direction-line"></span> NEWEST':'CAMERA SD CARD';
  if(state.following&&state.source==='recent')strip.scrollLeft=strip.scrollWidth;
  else strip.scrollLeft=oldLeft+Math.max(0,strip.scrollWidth-oldWidth)*(state.prepending?1:0);
  state.prepending=false;updateSelection();
}
function toggleChosen(id,checked){
  if(checked){if(state.selectedIds.size>=5){toast('Select up to 5 photos per download to keep memory use low.');renderStrip();return;}state.selectedIds.add(id);}else state.selectedIds.delete(id);
  updateDownloadButton();
}
function updateDownloadButton(){$('download-selected').disabled=state.selectedIds.size===0;$('download-selected').textContent=`Download selected (${state.selectedIds.size})`;}
function setSource(source){
  if(state.source===source)return;
  state.source=source;state.selectedIds.clear();state.cursor=null;following(source==='recent');
  $('source-recent').setAttribute('aria-pressed',String(source==='recent'));$('source-sd').setAttribute('aria-pressed',String(source==='sd'));
  $('sd-directory').hidden=source!=='sd';$('follow').hidden=source!=='recent';$('load-older').textContent='Load more';
  if(source==='recent'){
    state.photos=state.recent;state.selected=state.recent.at(-1)||null;renderStrip();
  }else{
    state.photos=[];state.selected=null;renderStrip();loadSdDirectories();
  }
}
$('source-recent').addEventListener('click',()=>setSource('recent'));
$('source-sd').addEventListener('click',()=>setSource('sd'));
async function loadSdDirectories(){
  if(state.sdLoaded)return;
  state.sdLoaded=true;
  try{
    const data=await api('/api/media/directories');state.sdLoaded=true;state.sdDirectories=data.directories||[];
    const select=$('sd-directory');select.replaceChildren();
    state.sdDirectories.forEach(dir=>{const option=document.createElement('option');option.value=dir.id;option.textContent=dir.name;select.append(option);});
    if(!state.sdDirectories.length){state.sdLoaded=false;toast('No photo folders were returned by the camera.');return;}
    await loadSdPage(true);
  }catch(error){state.sdLoaded=false;toast(error.message);}
}
async function loadSdPage(reset=false){
  if(state.galleryBusy)return;
  const directory=$('sd-directory').value;if(!directory)return;
  state.galleryBusy=true;$('load-older').disabled=true;
  try{
    const start=reset?0:state.sdStart;
    const data=await api(`/api/media/photos?directory=${encodeURIComponent(directory)}&start=${start}&count=40`);
    if(reset){state.sdPhotos=[];state.selectedIds.clear();}
    state.sdPhotos.push(...(data.photos||[]));state.sdTotal=data.total||0;state.sdStart=data.next??null;state.cursor=state.sdStart;
    state.photos=state.sdPhotos;
    if(reset)state.selected=state.sdPhotos[0]||null;
    renderStrip();
  }catch(error){toast(error.message);}finally{state.galleryBusy=false;$('load-older').disabled=false;}
}
$('sd-directory').addEventListener('change',()=>{state.sdStart=0;state.sdPhotos=[];state.sdTotal=0;loadSdPage(true);});
$('load-older').addEventListener('click',()=>{if(state.source==='sd')loadSdPage(false);});
$('filmstrip').addEventListener('wheel',event=>{if(event.deltaX||event.shiftKey)following(false);},{passive:true});
$('filmstrip').addEventListener('touchstart',()=>following(false),{passive:true});
$('follow').addEventListener('click',()=>{following(true);if(state.source==='recent'){state.photos=state.recent;state.selected=state.recent.at(-1)||null;renderStrip();}});
$('protect').addEventListener('click',()=>{
  if(!state.selected||state.selected.source==='sd')return;
  state.selected.protected=!state.selected.protected;
  renderStrip();toast(state.selected.protected?'Photo kept in this browser gallery':'Photo can be evicted when the browser gallery is full');
});
function navigate(offset){if(!state.selected)return;const index=state.photos.findIndex(p=>p.id===state.selected.id);const photo=state.photos[index+offset];if(photo){select(photo);document.querySelector(`[data-id="${CSS.escape(photo.id)}"] .thumbnail-select`)?.scrollIntoView({block:'nearest',inline:'nearest'});}}
$('previous').addEventListener('click',()=>navigate(-1));$('next').addEventListener('click',()=>navigate(1));
document.addEventListener('keydown',event=>{if(state.mode==='live'||event.target.closest('button,a,input,select,[role=tablist]'))return;if(event.key==='ArrowLeft')navigate(-1);if(event.key==='ArrowRight')navigate(1);});
$('live-image').addEventListener('click',async event=>{
  if(state.mode!=='controller'||!state.armed||!state.latestMetadata)return;
  const metadata=state.latestMetadata,rect=$('live-image').getBoundingClientRect(),sourceRatio=metadata.width/metadata.height,boxRatio=rect.width/rect.height;
  let contentWidth=rect.width,contentHeight=rect.height,offsetX=0,offsetY=0;
  if(boxRatio>sourceRatio){contentWidth=rect.height*sourceRatio;offsetX=(rect.width-contentWidth)/2;}else{contentHeight=rect.width/sourceRatio;offsetY=(rect.height-contentHeight)/2;}
  const px=event.clientX-rect.left-offsetX,py=event.clientY-rect.top-offsetY;
  if(px<0||py<0||px>=contentWidth||py>=contentHeight)return;
  const x=Math.round(px/contentWidth*metadata.width),y=Math.round(py/contentHeight*metadata.height);
  try{await api('/api/control/autofocus-point',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({client_id:state.clientId,x,y,width:metadata.width,height:metadata.height})});toast('Autofocus point sent. The camera may not acknowledge the command.');}catch(error){toast(error.message);}
});
$('photo-image').addEventListener('error',()=>{if(state.selected){$('photo-empty').hidden=false;$('photo-empty').querySelector('h2').textContent='This photo is no longer available';$('photo-empty').querySelector('p').textContent='Refresh the gallery or choose another photo.';$('photo-image').hidden=true;$('download').setAttribute('aria-disabled','true');}});
function renderDetectionOverlay(metadata,age){state.latestMetadata=metadata;
  const svg=$('live-boxes');svg.replaceChildren();
  if(!metadata||!Array.isArray(metadata.objects)||age>2)return;
  svg.setAttribute('viewBox',`0 0 ${metadata.width} ${metadata.height}`);
  const ns='http://www.w3.org/2000/svg',fontSize=Math.max(14,Math.round(metadata.width/80));
  metadata.objects.forEach(item=>{
    const box=item.bbox;if(!Array.isArray(box)||box.length!==4)return;
    const [x1,y1,x2,y2]=box,rect=document.createElementNS(ns,'rect');
    rect.setAttribute('x',x1);rect.setAttribute('y',y1);rect.setAttribute('width',Math.max(0,x2-x1));rect.setAttribute('height',Math.max(0,y2-y1));rect.setAttribute('fill','none');rect.setAttribute('stroke','#6de5be');rect.setAttribute('stroke-width',Math.max(3,metadata.width/500));
    const label=document.createElementNS(ns,'text');label.setAttribute('x',x1);label.setAttribute('y',Math.max(fontSize,y1-fontSize*.4));label.setAttribute('font-size',fontSize);label.setAttribute('font-family','sans-serif');label.setAttribute('font-weight','700');label.setAttribute('fill','#8ff1cf');label.setAttribute('paint-order','stroke');label.setAttribute('stroke','#102126');label.setAttribute('stroke-width',fontSize*.22);label.textContent=`${String(item.label||'object')} ${Math.round(Number(item.confidence||0)*100)}%`;
    svg.append(rect,label);
  });
}
async function captureRecent(sequence,metadata){
  if(sequence===state.sequence||Date.now()-state.lastCapture<state.interval*1000)return;
  try{
    const response=await fetch(`/api/live.jpg?sequence=${encodeURIComponent(sequence)}`,{signal:AbortSignal.timeout(5000)});if(!response.ok)return;
    const blob=await response.blob();await addRecent(blob,metadata);state.sequence=sequence;state.lastCapture=Date.now();
  }catch{}
}
let lastGallery=0;
async function poll(){
  try{
    const data=await api(`/api/status?client_id=${encodeURIComponent(state.clientId)}`);state.online=true;state.interval=Number(data.capture_interval)||1;state.recentLimit=Number(data.recent_limit)||60;
    $('demo-badge').hidden=!data.demo;const name=$('camera-name');name.firstChild.textContent=data.camera;
    const labels={live:'Connected',waiting:'Waiting for feed',detection_stale:'Detection paused',disconnected:'Camera offline'};
    $('connection').textContent=labels[data.state]||'Waiting';$('status-dot').className=data.state==='live'?'good':'bad';
    $('stale-overlay').hidden=data.state==='live'||state.sequence<0;
    const messages=[];if(data.demo)messages.push('Demo mode · synthetic images and simulated detections. Your camera is not connected.');
    if(data.source_error)messages.push(data.source_error);
    else if(data.state==='detection_stale')messages.push('Camera frames are arriving, but fresh matching detections are missing.');
    else if(data.state==='disconnected')messages.push('Camera connection lost. The last frame is shown until the feed returns.');
    $('notice').textContent=messages.join(' ');$('notice').hidden=!messages.length;
    if(data.metadata){const m=data.metadata;$('live-resolution').textContent=`PREVIEW · ${data.preview_fps||5} FPS`;$('live-detail').textContent=`${m.objects.length} ${m.objects.length===1?'object':'objects'} · detection ${timeLabel(m.captured)} · ${m.processing_sec==null?'':Math.round(m.processing_sec*1000)+' ms inference'}`;renderDetectionOverlay(m,Number(data.result_age_sec)||0);if(data.state==='live')await captureRecent(data.sequence,m);}
    state.controls=data.controls||{available:false,reason:'Camera controls are not connected.'};if(state.armed&&!state.controls.armed){state.armed=false;stopMotion(false);}updateControlUI();
  }catch{
    state.online=false;$('connection').textContent='Viewer offline';$('status-dot').className='bad';$('stale-overlay').hidden=state.sequence<0;
    $('notice').textContent='Cannot reach the viewer service. Reconnecting automatically…';$('notice').hidden=false;
    if(state.armed)disarmController();
  }finally{setTimeout(poll,750);}
}

// Build an uncompressed ZIP in-browser so previews and downloaded camera photos are not stored on the Jetson.
const crcTable=(()=>{const table=new Uint32Array(256);for(let n=0;n<256;n++){let c=n;for(let k=0;k<8;k++)c=c&1?0xedb88320^(c>>>1):c>>>1;table[n]=c>>>0;}return table;})();
function crc32(bytes){let c=0xffffffff;for(const b of bytes)c=crcTable[(c^b)&255]^(c>>>8);return(c^0xffffffff)>>>0;}
function put16(view,offset,value){view.setUint16(offset,value,true);}function put32(view,offset,value){view.setUint32(offset,value>>>0,true);}
async function zipFiles(files){
  const locals=[],centrals=[];let offset=0;
  for(const file of files){const name=new TextEncoder().encode(file.name.replace(/[^A-Za-z0-9_.-]/g,'_'));const data=new Uint8Array(await file.blob.arrayBuffer());const crc=crc32(data),local=new Uint8Array(30+name.length+data.length),lv=new DataView(local.buffer);
    put32(lv,0,0x04034b50);put16(lv,4,20);put16(lv,6,0x0800);put16(lv,8,0);put32(lv,14,crc);put32(lv,18,data.length);put32(lv,22,data.length);put16(lv,26,name.length);local.set(name,30);local.set(data,30+name.length);locals.push(local);
    const central=new Uint8Array(46+name.length),cv=new DataView(central.buffer);put32(cv,0,0x02014b50);put16(cv,4,20);put16(cv,6,20);put16(cv,8,0x0800);put16(cv,10,0);put32(cv,16,crc);put32(cv,20,data.length);put32(cv,24,data.length);put16(cv,28,name.length);put32(cv,42,offset);central.set(name,46);centrals.push(central);offset+=local.length;
  }
  const centralSize=centrals.reduce((n,x)=>n+x.length,0),end=new Uint8Array(22),ev=new DataView(end.buffer);put32(ev,0,0x06054b50);put16(ev,8,files.length);put16(ev,10,files.length);put32(ev,12,centralSize);put32(ev,16,offset);
  return new Blob([...locals,...centrals,end],{type:'application/zip'});
}
$('download-selected').addEventListener('click',async()=>{
  const selected=state.photos.filter(p=>state.selectedIds.has(p.id));if(!selected.length)return;
  $('download-selected').disabled=true;$('download-selected').textContent='Preparing download…';
  try{const files=[];for(const photo of selected){const blob=photo.blob||await(await fetch(photo.url,{signal:AbortSignal.timeout(15000)})).blob();files.push({name:photo.name||`astra-${photo.id}.jpg`,blob});}
    const total=files.reduce((sum,file)=>sum+file.blob.size,0);if(total>25*1024*1024)throw new Error('Keep a download batch below 25 MB for mobile memory.');
    const zip=await zipFiles(files),url=URL.createObjectURL(zip),link=document.createElement('a');link.href=url;link.download=`astra-photos-${new Date().toISOString().slice(0,10)}.zip`;link.click();setTimeout(()=>URL.revokeObjectURL(url),30000);
  }catch(error){toast(`Could not prepare downloads: ${error.message}`);}finally{updateDownloadButton();}
});

// Hold-to-move controller: browser releases and a server watchdog both send a zero-motion stop.
const motion={yaw:0,pitch:0,zoom:0,focus:0,timer:null,stick:false,zoomHeld:0,focusHeld:0,keys:new Set()};
function sendMotion(){if(!state.armed)return;api('/api/control/motion',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({client_id:state.clientId,yaw:motion.yaw,pitch:motion.pitch,zoom:motion.zoom,focus:motion.focus})}).catch(()=>stopMotion());}
function updateMotion(){
  if(!state.armed)return;
  const keys=motion.keys,vertical=(keys.has('ArrowUp')||keys.has('w')?1:0)-(keys.has('ArrowDown')||keys.has('s')?1:0),horizontal=(keys.has('ArrowRight')||keys.has('d')?1:0)-(keys.has('ArrowLeft')||keys.has('a')?1:0);
  if(!motion.stick){motion.yaw=horizontal*55;motion.pitch=vertical*55;}
  motion.zoom=motion.zoomHeld;motion.focus=motion.focusHeld;
  if(!motion.timer){sendMotion();motion.timer=setInterval(sendMotion,120);}
}
function stopMotion(send=true){clearInterval(motion.timer);motion.timer=null;motion.yaw=0;motion.pitch=0;motion.zoom=0;motion.focus=0;motion.zoomHeld=0;motion.focusHeld=0;motion.stick=false;motion.keys.clear();$('stick-knob').style.transform='translate(0,0)';document.querySelectorAll('.hold-control').forEach(x=>x.classList.remove('held'));if(send&&state.clientId)api('/api/control/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({client_id:state.clientId})}).catch(()=>{});}
function updateControlUI(){
  const ready=!!state.controls.available;
  $('control-state').textContent=!ready?'LOCKED':state.armed?'ARMED':'READY';
  $('control-message').textContent=!ready?(state.controls.reason||'Controller unavailable. Check the ZR10 ROS control node.'):(state.armed?'Controls armed. Motion runs only while held.':'Camera is ready. Enable controls before moving the camera.');
  $('arm-controls').disabled=!ready;$('arm-controls').textContent=state.armed?'Lock controls':'Enable controls';$('arm-controls').classList.toggle('armed',state.armed);
  document.querySelectorAll('.hold-control,[data-action]').forEach(button=>button.disabled=!ready||!state.armed);$('apply-zoom').disabled=!ready||!state.armed;
}
$('absolute-zoom').addEventListener('input',()=>{$('absolute-zoom-value').textContent=`${Number($('absolute-zoom').value).toFixed(1)}×`;});
$('apply-zoom').addEventListener('click',async()=>{if(!state.armed)return;try{await api('/api/control/zoom',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({client_id:state.clientId,zoom:Number($('absolute-zoom').value)})});toast(`Zoom set to ${Number($('absolute-zoom').value).toFixed(1)}×. Camera response is not confirmed.`);}catch(error){toast(error.message);}});
$('arm-controls').addEventListener('click',async()=>{
  try{
    const armed=!state.armed;
    if(armed&&!window.confirm('Enable live ZR10 controls? Holding the joystick or zoom buttons will move the camera.'))return;
    const result=await api('/api/control/arm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({client_id:state.clientId,armed})});
    state.armed=armed;updateControlUI();if(!armed)stopMotion();else toast('Camera controls enabled. Release any control to stop motion.');
  }catch(error){toast(error.message);state.armed=false;updateControlUI();}
});
function pointerVector(event){
  const rect=$('joystick').getBoundingClientRect(),radius=Math.min(rect.width,rect.height)*.38,cx=rect.left+rect.width/2,cy=rect.top+rect.height/2;
  let dx=event.clientX-cx,dy=event.clientY-cy,len=Math.hypot(dx,dy);if(len>radius){dx=dx/len*radius;dy=dy/len*radius;}
  $('stick-knob').style.transform=`translate(${dx}px,${dy}px)`;motion.stick=true;motion.yaw=Math.round(dx/radius*70);motion.pitch=Math.round(-dy/radius*70);updateMotion();
}
$('joystick').addEventListener('pointerdown',event=>{if(!state.armed)return;event.preventDefault();$('joystick').setPointerCapture(event.pointerId);pointerVector(event);});
$('joystick').addEventListener('pointermove',event=>{if(event.buttons&&state.armed)pointerVector(event);});
for(const type of ['pointerup','pointercancel','lostpointercapture'])$('joystick').addEventListener(type,()=>{motion.stick=false;motion.yaw=0;motion.pitch=0;if(!motion.zoomHeld&&!motion.keys.size)stopMotion();else updateMotion();});
document.querySelectorAll('.hold-control').forEach(button=>{
  const start=event=>{if(!state.armed)return;event.preventDefault();button.setPointerCapture?.(event.pointerId);button.classList.add('held');if(button.dataset.zoom!==undefined)motion.zoomHeld=Number(button.dataset.zoom);if(button.dataset.focus!==undefined)motion.focusHeld=Number(button.dataset.focus);updateMotion();};
  const end=()=>{button.classList.remove('held');if(button.dataset.zoom!==undefined)motion.zoomHeld=0;if(button.dataset.focus!==undefined)motion.focusHeld=0;if(!motion.stick&&!motion.keys.size&&!motion.zoomHeld&&!motion.focusHeld)stopMotion();else updateMotion();};
  button.addEventListener('pointerdown',start);button.addEventListener('pointerup',end);button.addEventListener('pointercancel',end);button.addEventListener('lostpointercapture',end);button.addEventListener('contextmenu',e=>e.preventDefault());
});
document.addEventListener('keydown',event=>{
  if(state.mode!=='controller'||!state.armed||event.repeat)return;
  const target=event.target;if(target.closest('input,select,textarea,button,a'))return;
  const key=event.key.length===1?event.key.toLowerCase():event.key;
  if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','w','a','s','d','+','=','-','_','[',']'].includes(key)){event.preventDefault();if(key==='+'||key==='=')motion.zoomHeld=1;else if(key==='-'||key==='_')motion.zoomHeld=-1;else if(key===']')motion.focusHeld=1;else if(key==='[')motion.focusHeld=-1;else motion.keys.add(key);updateMotion();}
});
document.addEventListener('keyup',event=>{
  const key=event.key.length===1?event.key.toLowerCase():event.key;
  if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','w','a','s','d'].includes(key))motion.keys.delete(key);
  if(['+','='].includes(key)&&motion.zoomHeld===1)motion.zoomHeld=0;
  if(['-','_'].includes(key)&&motion.zoomHeld===-1)motion.zoomHeld=0;
  if(key===']'&&motion.focusHeld===1)motion.focusHeld=0;if(key==='['&&motion.focusHeld===-1)motion.focusHeld=0;
  if(!motion.keys.size&&!motion.stick&&!motion.zoomHeld&&!motion.focusHeld)stopMotion();else updateMotion();
});
window.addEventListener('blur',()=>{if(state.armed)disarmController();else stopMotion();});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&state.armed)disarmController();});
window.addEventListener('pagehide',()=>{stopMotion(false);if(state.armed)stopBeacon();});
document.querySelectorAll('[data-action]').forEach(button=>button.addEventListener('click',async()=>{
  if(!state.armed)return;
  if(button.dataset.action==='record'&&!window.confirm('The camera recording control is a toggle and does not report its current state. Send the toggle command?'))return;
  button.disabled=true;
  try{const result=await api('/api/control/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({client_id:state.clientId,action:button.dataset.action})});toast(result.message||'Camera command sent. The camera may not acknowledge this command.');}
  catch(error){toast(error.message);}finally{updateControlUI();}
}));

function init(){
  try{state.clientId=sessionStorage.getItem('astra-control-session')||makeId();sessionStorage.setItem('astra-control-session',state.clientId);}catch{state.clientId=makeId();}
  try{const saved=localStorage.getItem('astra-mode');if(saved in titles)setMode(saved);}catch{}
  const image=$('live-image');image.src='/api/live.mjpg';image.addEventListener('load',()=>{$('live-empty').hidden=true;});image.addEventListener('error',()=>{image.hidden=true;$('live-empty').hidden=false;setTimeout(()=>{image.hidden=false;image.src=`/api/live.mjpg?retry=${Date.now()}`;},1200);});
  poll();
}
init();
