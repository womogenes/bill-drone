import {test} from 'node:test';
import assert from 'node:assert/strict';
import {operation,OnshapeSession} from './session.mjs';
const p={documentId:'a'.repeat(24),workspaceId:'b'.repeat(24),elementId:'c'.repeat(24)};
test('rejects invalid IDs and arbitrary operations',()=>{
  assert.throws(()=>operation('elements',{...p,documentId:'../users/session'}));
  assert.throws(()=>operation('fetch',p));
});
test('read caching, fresh bypass and write invalidation',async()=>{
  const s=new OnshapeSession();let count=0;s.xsrf='test';
  s.page={evaluate:async()=>{count++;return {status:200,data:{ok:true}};}};
  await s.run('elements',p);assert.equal((await s.run('elements',p)).cached,true);assert.equal(count,1);
  await s.run('elements',p,{fresh:true});assert.equal(count,2);
  await assert.rejects(s.run('updateSource',p),/write:true/);assert.equal(count,2);
  await s.run('updateSource',{...p,contents:'example'},{write:true});
  await s.run('elements',p);assert.equal(count,4);
});
test('rate-limit failure is not retried',async()=>{
  const s=new OnshapeSession();let count=0;
  s.page={evaluate:async()=>{count++;return {status:429,data:{}};}};
  await assert.rejects(s.run('elements',p),/HTTP 429/);assert.equal(count,1);
});
