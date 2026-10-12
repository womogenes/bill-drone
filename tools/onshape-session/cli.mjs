import net from 'node:net';
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import {OnshapeSession,defaultChromium} from './session.mjs';

const directory=path.join(os.tmpdir(),`onshape-session-${process.getuid()}`);
const socket=path.join(directory,'browser.sock');
const [command,...args]=process.argv.slice(2);
const defaultUrl='https://cad.onshape.com/documents/d7e760af3cc2197cafa5a123/w/0a0eede3b0e261a3b55f7eb1/e/d6979667eed205f63b0dfaf2';

async function serve() {
  await fs.mkdir(directory,{recursive:true,mode:0o700});
  const owner=await fs.stat(directory);
  if(owner.uid!==process.getuid() || (owner.mode & 0o077)) throw new Error('Session directory must be private and owned by this user');
  // Refuse to displace an existing session, including a stale socket.
  try {await fs.lstat(socket);throw new Error(`Socket exists: ${socket}`);} catch(e) {if(e.code!=='ENOENT')throw e;}
  const session=new OnshapeSession();
  await session.start({email:process.env.ONSHAPE_EMAIL || 'wyf@mit.edu',
    executablePath:process.env.ONSHAPE_CHROMIUM || defaultChromium,
    headless:!args.includes('--headed'),initialUrl:process.env.ONSHAPE_DOCUMENT_URL || defaultUrl});
  let queue=Promise.resolve();
  const server=net.createServer(connection=>{
    let input='';
    connection.on('data',chunk=>{
      input+=chunk;
      if(input.length>8*1024*1024) return connection.destroy();
      if(!input.includes('\n'))return;
      connection.removeAllListeners('data');
      queue=queue.then(async()=>{
        try {
          const request=JSON.parse(input.split('\n')[0]);
          let result;
          if(request.operation==='status') result=session.stats();
          else if(request.operation==='screenshot') {
            const target=path.join(directory,'assembly.png');
            await session.page.screenshot({path:target});result={path:target};
          } else result=await session.run(request.operation,request.params,request.options);
          connection.end(JSON.stringify({ok:true,result})+'\n');
        } catch(e) {connection.end(JSON.stringify({ok:false,error:e.message})+'\n');}
      });
    });
    connection.on('error',()=>{});
  });
  await new Promise((resolve,reject)=>{server.once('error',reject);server.listen(socket,resolve);});
  await fs.chmod(socket,0o600);
  console.log(JSON.stringify({ready:true,socket,transport:'browser-session'}));
  const stop=async()=>{server.close();await session.close();await fs.unlink(socket).catch(()=>{});process.exit();};
  process.on('SIGTERM',stop);process.on('SIGINT',stop);
}
async function client() {
  const paramsFile=args.find(arg=>!arg.startsWith('--'));
  const params=paramsFile?JSON.parse(await fs.readFile(paramsFile,'utf8')):{};
  const result=await new Promise((resolve,reject)=>{
    const connection=net.createConnection(socket);let data='';
    connection.on('connect',()=>connection.write(JSON.stringify({operation:command,params,
      options:{write:args.includes('--write'),fresh:args.includes('--fresh')}})+'\n'));
    connection.on('data',chunk=>data+=chunk);
    connection.on('end',()=>{try{resolve(JSON.parse(data));}catch(e){reject(e);}});
    connection.on('error',reject);
  });
  console.log(JSON.stringify(result,null,2));if(!result.ok)process.exitCode=1;
}
try {if(command==='serve')await serve();else if(command)await client();else console.log('Usage: node cli.mjs serve [--headed] | OPERATION [params.json] [--fresh] [--write]');}
catch(e){console.error(e.message);process.exit(1);}
