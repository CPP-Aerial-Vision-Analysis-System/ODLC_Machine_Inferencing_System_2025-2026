'use strict';
const $ = id => document.getElementById(id);
const state = {mode:'live', photos:[], selected:null, following:true, cursor:null, sequence:-1, galleryBusy:false, history:false, online:false, first:true, liveBusy:false};
const titles = {live:['A live perspective.','Your camera, with detection in view.'],photos:['Every moment, in focus.','Browse, protect, and download your captured photos.'],split:['The full picture.','Live detection and a captured moment, side by side.']};
const timeLabel = seconds => new Date(seconds*1000).toLocaleTimeString([], {hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false});
const dateLabel = seconds => new Date(seconds*1000).toLocaleString([], {month:'short',day:'numeric',hour:'2-digit',minute:'2-digit',second:'2-digit'});
const bytes = n => n >= 1024**3 ? `${(n/1024**3).toFixed(1)} GB` : `${(n/1024**2).toFixed(1)} MB`;
async function api(path, options={}) {
  const response = await fetch(path, {...options, signal:AbortSignal.timeout(5000)});
  if(!response.ok) {let message=`Request failed (${response.status})`;try {message=(await response.json()).error||message;}catch{}throw new Error(message);}
  return response.json();
}
let toastTimer;
function toast(text) {$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,5000);}
function setMode(mode) {
  state.mode=mode;
  $('workspace').className=`workspace mode-${mode}`;
  $('workspace').setAttribute('aria-labelledby',`tab-${mode}`);
  $('view-title').textContent=titles[mode][0];$('view-description').textContent=titles[mode][1];
  document.querySelectorAll('[data-mode]').forEach(b=>{b.setAttribute('aria-selected',String(b.dataset.mode===mode));b.tabIndex=b.dataset.mode===mode?0:-1;});
  try {localStorage.setItem('astra-mode',mode);}catch{}
}
document.querySelectorAll('[data-mode]').forEach(button=>{
  button.addEventListener('click',()=>setMode(button.dataset.mode));
  button.addEventListener('keydown',event=>{if(!['ArrowLeft','ArrowRight','Home','End'].includes(event.key))return;event.preventDefault();const modes=['live','photos','split'];let index=modes.indexOf(state.mode);index=event.key==='Home'?0:event.key==='End'?2:(index+(event.key==='ArrowRight'?1:2))%3;setMode(modes[index]);$(`tab-${modes[index]}`).focus();});
});
function following(value) {state.following=value;$('follow').setAttribute('aria-pressed',String(value));$('follow').textContent=value?'↗ Following latest':'↗ Follow latest';}
function updateSelection() {
  const photo=state.selected;
  $('photo-image').hidden=!photo;$('photo-empty').hidden=!!photo;
  $('protect').disabled=!photo;
  $('download').setAttribute('aria-disabled',String(!photo));
  if(!photo)return;
  const url=new URL(photo.url,location.origin).href;
  if($('photo-image').src!==url)$('photo-image').src=photo.url;
  $('photo-position').textContent=photo.reference||photo.id.slice(0,8).toUpperCase();
  const detected=photo.objects.length;
  $('photo-detail').textContent=`${dateLabel(photo.captured)} · ${detected} ${detected===1?'object':'objects'}`;
  $('selection-caption').textContent=state.following?'Following the newest captured moment':`${dateLabel(photo.captured)} · ${photo.protected?'Protected':'Selected moment'}`;
  $('protect').textContent=photo.protected?'★ Protected':'☆ Protect';
  $('protect').setAttribute('aria-pressed',String(photo.protected));
  $('download').href=`/api/photos/${photo.id}/download`;
  $('download').setAttribute('download',`astra-${photo.id}.jpg`);
  const index=state.photos.findIndex(p=>p.id===photo.id);
  $('previous').hidden=index<=0;$('next').hidden=index<0||index>=state.photos.length-1;
  document.querySelectorAll('.thumbnail').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.id===photo.id)));
}
function select(photo, manual=true) {state.selected=photo;if(manual)following(false);updateSelection();}
function renderStrip() {
  const strip=$('filmstrip');
  const oldLeft=strip.scrollLeft, oldWidth=strip.scrollWidth;
  // Reuse thumbnails so one-second updates do not refetch the whole strip.
  const existing=new Map([...$('thumbnails').children].map(el=>[el.dataset.id,el]));
  const fragment=document.createDocumentFragment();
  state.photos.forEach(photo=>{
    let button=existing.get(photo.id);
    if(!button){button=document.createElement('button');button.type='button';button.className='thumbnail';button.dataset.id=photo.id;
      const img=document.createElement('img');img.src=photo.thumbnail;img.loading='lazy';img.alt=`Capture at ${dateLabel(photo.captured)}`;
      const caption=document.createElement('span');caption.className='thumb-caption';
      const time=document.createElement('span');time.textContent=timeLabel(photo.captured);
      const confidence=document.createElement('span');confidence.textContent=photo.confidence===null?'—':`${Math.round(photo.confidence*100)}%`;
      caption.append(time,confidence);button.append(img,caption);
      button.addEventListener('click',()=>{const current=state.photos.find(p=>p.id===button.dataset.id);if(current){select(current);if(state.mode==='live')setMode('photos');}});
    }
    button.setAttribute('aria-label',`${dateLabel(photo.captured)}, ${photo.objects.length} detections${photo.protected?', protected':''}`);
    let star=button.querySelector('.thumb-protected');
    if(photo.protected&&!star){star=document.createElement('span');star.className='thumb-protected';star.textContent='★';button.append(star);}else if(!photo.protected&&star)star.remove();
    fragment.append(button);
  });
  $('thumbnails').replaceChildren(fragment);
  $('load-older').hidden=!state.cursor;$('strip-empty').hidden=state.photos.length>0;
  if(state.following)strip.scrollLeft=strip.scrollWidth;
  else strip.scrollLeft=oldLeft+Math.max(0,strip.scrollWidth-oldWidth)*(state.prepending?1:0);
  state.prepending=false;updateSelection();
}
async function refreshGallery(reset=false) {
  if(state.galleryBusy||(state.history&&!reset))return;
  if(reset)state.history=false;
  state.galleryBusy=true;
  try {
    const data=await api('/api/photos?limit=60');
    const map=new Map((reset?[]:state.photos).map(p=>[p.id,p]));
    data.photos.forEach(p=>map.set(p.id,p));
    state.photos=[...map.values()].sort((a,b)=>a.captured-b.captured||a.id.localeCompare(b.id));
    // Bound DOM and browser memory; older photos remain available via pagination.
    let trimmed=false;
    if(state.photos.length>240){if(!state.following){state.history=true;state.photos=state.photos.slice(0,240);}else{state.photos=state.photos.slice(-240);trimmed=true;}}
    const oldest=state.photos[0];
    const hasOlder=trimmed||!!data.next||(!reset&&!!state.cursor);
    state.cursor=oldest&&hasOlder?`${oldest.captured}:${oldest.id}`:null;
    if(state.following&&state.photos.length)select(state.photos.at(-1),false);
    else if(state.selected&&map.has(state.selected.id))state.selected=map.get(state.selected.id);
    renderStrip();
  }catch(error){if(state.first)toast(error.message);}finally{state.galleryBusy=false;state.first=false;}
}
$('filmstrip').addEventListener('wheel',event=>{if(event.deltaX||event.shiftKey)following(false);},{passive:true});
$('filmstrip').addEventListener('touchstart',()=>following(false),{passive:true});
$('filmstrip').addEventListener('pointerdown',event=>{if(event.clientY>$('filmstrip').getBoundingClientRect().bottom-14)following(false);});
$('follow').addEventListener('click',async()=>{following(true);await refreshGallery(true);});
$('load-older').addEventListener('click',async()=>{
  if(state.galleryBusy||!state.cursor)return;
  following(false);state.history=true;state.galleryBusy=true;$('load-older').disabled=true;
  try {
    const data=await api(`/api/photos?limit=60&before=${encodeURIComponent(state.cursor)}`);
    const map=new Map([...data.photos,...state.photos].map(p=>[p.id,p]));
    state.photos=[...map.values()].sort((a,b)=>a.captured-b.captured||a.id.localeCompare(b.id)).slice(0,240);
    state.cursor=data.next;state.prepending=true;renderStrip();
  }catch(error){toast(error.message);}finally{state.galleryBusy=false;$('load-older').disabled=false;}
});
$('protect').addEventListener('click',async()=>{
  if(!state.selected)return;
  const photo=state.selected;following(false);$('protect').disabled=true;
  try {const updated=await api(`/api/photos/${photo.id}/protect`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({protected:!photo.protected})});
    state.photos=state.photos.map(p=>p.id===updated.id?updated:p);if(state.selected?.id===updated.id)state.selected=updated;renderStrip();toast(updated.protected?'Photo protected from automatic cleanup':'Photo can now be removed by automatic cleanup');
  }catch(error){toast(error.message);}finally{$('protect').disabled=!state.selected;}
});
function navigate(offset){if(!state.selected)return;const index=state.photos.findIndex(p=>p.id===state.selected.id);const photo=state.photos[index+offset];if(photo){select(photo);document.querySelector(`[data-id="${photo.id}"]`)?.scrollIntoView({block:'nearest',inline:'nearest'});}}
$('previous').addEventListener('click',()=>navigate(-1));$('next').addEventListener('click',()=>navigate(1));
document.addEventListener('keydown',event=>{if(state.mode==='live'||event.target.closest('button,a,input,[role=tablist]'))return;if(event.key==='ArrowLeft'){event.preventDefault();navigate(-1);}if(event.key==='ArrowRight'){event.preventDefault();navigate(1);}});
$('photo-image').addEventListener('error',()=>{if(state.selected){$('photo-empty').hidden=false;$('photo-empty').querySelector('h2').textContent='This photo is no longer available';$('photo-empty').querySelector('p').textContent='It may have been removed by storage cleanup. Choose another moment.';$('photo-image').hidden=true;$('download').setAttribute('aria-disabled','true');}});
$('storage-info').addEventListener('click',()=>toast('When space runs low, older photos with no detections go first, then lower-confidence photos. Protected photos are always kept.'));
let liveURL;
async function refreshLive(sequence){
  if(state.liveBusy||sequence===state.sequence)return;
  state.liveBusy=true;
  try{const response=await fetch(`/api/live.jpg?frame=${sequence}`,{signal:AbortSignal.timeout(5000)});if(!response.ok)throw new Error();const blob=await response.blob();const nextURL=URL.createObjectURL(blob);const image=$('live-image');
    // Decode before switching the displayed frame; avoid flicker on slow connections.
    const preload=new Image();preload.src=nextURL;await preload.decode();image.src=nextURL;image.hidden=false;$('live-empty').hidden=true;if(liveURL)URL.revokeObjectURL(liveURL);liveURL=nextURL;state.sequence=sequence;
  }catch{}finally{state.liveBusy=false;}
}
let lastGallery=0;
async function poll(){
  try{
    const data=await api('/api/status');state.online=true;
    $('demo-badge').hidden=!data.demo;
    const name=$('camera-name');name.firstChild.textContent=data.camera;
    const labels={live:'Connected',waiting:'Waiting for feed',detection_stale:'Detection paused',disconnected:'Camera offline'};
    $('connection').textContent=labels[data.state]||'Waiting';$('status-dot').className=data.state==='live'?'good':'bad';
    $('stale-overlay').hidden=data.state==='live'||state.sequence<0;
    $('interval').textContent=data.capture_interval;
    const archive=data.archive;$('photo-count').textContent=archive.count.toLocaleString();
    $('storage-label').textContent=`${bytes(archive.used_bytes)} / ${bytes(archive.max_bytes)} archive`;
    $('storage-fill').style.width=`${Math.min(100,archive.used_bytes/archive.max_bytes*100)}%`;
    const messages=[];
    if(data.demo)messages.push('Demo mode · synthetic images and simulated detections. Your camera is not connected.');
    if(data.error)messages.push(`Photo saving: ${data.error}`);
    if(data.source_error)messages.push(data.source_error);
    else if(data.state==='detection_stale')messages.push('Camera frames are arriving, but fresh matching detections are missing. Automatic photo saving waits for a fresh result.');
    else if(data.state==='disconnected')messages.push('Camera connection lost. The last frame is shown; automatic photo saving is waiting.');
    $('notice').textContent=messages.join(' ');$('notice').hidden=!messages.length;
    if(data.metadata){const m=data.metadata;$('live-resolution').textContent=`${m.width} × ${m.height}`;
      $('live-detail').textContent=`${m.objects.length} ${m.objects.length===1?'object':'objects'} · ${m.processing_sec==null?'':Math.round(m.processing_sec*1000)+' ms inference · '}${timeLabel(m.captured)}`;
      await refreshLive(data.sequence);
    }
    if(Date.now()-lastGallery>1400){await refreshGallery();lastGallery=Date.now();}
  }catch{
    state.online=false;$('connection').textContent='Viewer offline';$('status-dot').className='bad';$('stale-overlay').hidden=state.sequence<0;
    $('notice').textContent='Cannot reach the viewer service. Check the host computer and network connection. Reconnecting automatically…';$('notice').hidden=false;
  }finally{setTimeout(poll,500);}
}
try {const saved=localStorage.getItem('astra-mode');if(saved in titles)setMode(saved);}catch{}
poll();
