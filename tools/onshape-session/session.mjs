import { chromium } from 'playwright-core';
import { execFileSync } from 'node:child_process';
import { homedir } from 'node:os';

export const ORIGIN = 'https://cad.onshape.com';
const id = value => {
  if (!/^[a-f0-9]{24}$/.test(value ?? '')) throw new Error('Expected a 24-character Onshape ID');
  return value;
};
export function operation(name, p = {}) {
  const dw = () => `d/${id(p.documentId)}/w/${id(p.workspaceId)}`;
  const dwe = () => `${dw()}/e/${id(p.elementId)}`;
  switch (name) {
    case 'elements': return { method: 'GET', path: `/documents/${dw()}/elements` };
    case 'features': return { method: 'GET', path: `/partstudios/${dwe()}/features` };
    case 'source': return { method: 'GET', path: `/featurestudios/${dwe()}` };
    case 'assembly': return { method: 'GET', path: `/assemblies/${dwe()}` };
    case 'evaluate': return { method: 'POST', path: `/partstudios/${dwe()}/featurescript`, body: {script:p.script} };
    case 'updateSource': return { method: 'POST', path: `/featurestudios/${dwe()}`, body: {contents:p.contents}, mutates:true };
    case 'insertInstances': return { method: 'POST', path: `/assemblies/${dwe()}/transformedinstances`, body: {transformGroups:p.transformGroups}, mutates:true };
    default: throw new Error(`Unknown operation: ${name}`);
  }
}

export class OnshapeSession {
  constructor() { this.cache=new Map(); this.trace=[]; this.xsrf=null; this.calls=0; }
  async start({email, executablePath, headless=true, initialUrl}) {
    this.browser=await chromium.launch({executablePath,headless});
    this.context=await this.browser.newContext({viewport:{width:1500,height:1000}});
    this.page=await this.context.newPage();
    await this.page.goto(`${ORIGIN}/signin`);
    await this.page.getByPlaceholder('Email',{exact:true}).fill(email);
    await this.page.getByRole('button',{name:'Continue',exact:true}).click();
    // Read only the explicitly authorized password. Never source .bashrc or read API keys.
    let password=execFileSync('python',['-c',
      "import pathlib,re,shlex; s=(pathlib.Path.home()/'.bashrc').read_text(); m=re.search(r'^\\s*(?:export\\s+)?ONSHAPE_PASSWORD\\s*=\\s*(.+)$',s,re.M); assert m, 'ONSHAPE_PASSWORD is missing'; print(shlex.split(m.group(1),comments=True)[0],end='')"
    ],{encoding:'utf8'});
    await this.page.getByPlaceholder('Password',{exact:true}).fill(password);
    password=undefined;
    await this.page.getByRole('button',{name:'Sign in',exact:true}).click();
    await this.page.waitForURL('**/documents**',{timeout:60000});
    // Authentication requests are deliberately excluded from capture.
    this.context.on('request',request=>{
      const u=new URL(request.url());
      if(u.origin===ORIGIN && u.pathname.startsWith('/api/')) {
        const token=request.headers()['x-xsrf-token'];
        if(token) this.xsrf=token; // memory only
      }
    });
    this.context.on('response',response=>{
      const u=new URL(response.url());
      if(u.origin===ORIGIN && u.pathname.startsWith('/api/')) {
        this.trace.push({method:response.request().method(),path:u.pathname,
          queryKeys:[...u.searchParams.keys()],status:response.status()});
        if(this.trace.length>1000) this.trace.shift();
      }
    });
    if(initialUrl) {
      const u=new URL(initialUrl);
      if(u.origin!==ORIGIN || !u.pathname.startsWith('/documents/')) throw new Error('Expected an Onshape document URL');
      await this.page.goto(initialUrl);
      await this.page.locator('#assembly-tree, #feature-tree').first().waitFor({timeout:60000});
    }
    return this;
  }
  async run(name,p={}, {write=false,fresh=false}={}) {
    const op=operation(name,p);
    if(op.mutates && !write) throw new Error('Mutating operation requires write:true');
    const key=op.path;
    const cached=this.cache.get(key);
    if(op.method==='GET' && !fresh && cached && Date.now()-cached.time<30000)
      return {...cached.result,cached:true};
    if(op.method!=='GET' && !this.xsrf) throw new Error('No observed XSRF header; load a document in this session first');
    // The request executes inside the authenticated browser. Cookies never leave it.
    const result=await this.page.evaluate(async ({path,method,body,xsrf})=>{
      const headers={'Accept':'application/json'};
      if(body!==undefined) headers['Content-Type']='application/json';
      if(xsrf) headers['X-XSRF-TOKEN']=xsrf;
      const response=await fetch('/api/v14'+path,{method,headers,credentials:'same-origin',
        redirect:'error',body:body===undefined?undefined:JSON.stringify(body),signal:AbortSignal.timeout(60000)});
      const text=await response.text();
      let data;try {data=JSON.parse(text);} catch {data=null;}
      return {status:response.status,data};
    },{...op,xsrf:this.xsrf});
    this.calls++;
    if(op.mutates) this.cache.clear();
    if(result.status<200 || result.status>=300) throw new Error(`Onshape HTTP ${result.status}; no automatic retry`);
    if(result.data===null) throw new Error('Expected JSON response; session may have expired');
    if(op.method==='GET') this.cache.set(key,{time:Date.now(),result});
    return {...result,cached:false};
  }
  stats() {return {transport:'browser-session',wrapperCalls:this.calls,cachedReads:this.cache.size,
    xsrfObserved:!!this.xsrf,captured:[...new Map(this.trace.map(r=>[r.method+' '+r.path,r])).values()]};}
  async close() {await this.browser?.close();}
}

export const defaultChromium = `${homedir()}/.cache/ms-playwright/chromium-1208/chrome-linux64/chrome`;
