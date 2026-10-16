function(context is Context, queries is map) {
 var cradle;for(var b in evaluateQuery(context,qBodyType(qEverything(EntityType.BODY),BodyType.SOLID)))if(getProperty(context,{"entity":b,"propertyType":PropertyType.NAME})=="Motor cradle")cradle=b;
 var checks=[];var specs=[];const dx=-6.5;
 for(var i=0;i<4;i+=1) {
  const y=7*cos(i*90*degree);const z=32+7*sin(i*90*degree);
  specs=append(specs,{"name":"M2 shaft "~i,"x":-25+dx,"y":y,"z":z,"r":1,"h":3.8,"contact":false});
  specs=append(specs,{"name":"M2 nut "~i,"x":-21.2+dx,"y":y,"z":z,"r":4/sqrt(3),"h":1.6,"hex":true,"contact":false});
 }
 specs=append(specs,{"name":"M3 center shaft","x":-25+dx,"y":0,"z":32,"r":1.5,"h":4,"contact":false});
 specs=append(specs,{"name":"M3 center socket head","x":-21+dx,"y":0,"z":32,"r":2.75,"h":3,"contact":false});
 specs=append(specs,{"name":"M3 head cannot pass shoulder","x":-21.2+dx,"y":0,"z":32,"r":2.75,"h":3,"contact":true});
 specs=append(specs,{"name":"Center driver via empty journal","x":-18+dx,"y":0,"z":32,"r":1.75,"h":65,"contact":false});
 for(var i=0;i<size(specs);i+=1) {
  const s=specs[i];const gid=makeId("hardware"~i);var sk=newSketchOnPlane(context,gid+"sk",{"sketchPlane":plane(vector(s.x,s.y,s.z)*millimeter,vector(1,0,0),vector(0,1,0))});
  if(s.hex==true) { var pts=[];for(var j=0;j<6;j+=1)pts=append(pts,vector(s.r*cos(j*60*degree),s.r*sin(j*60*degree))*millimeter);pts=append(pts,pts[0]);skPolyline(sk,"hex",{"points":pts}); } else skCircle(sk,"circle",{"center":vector(0,0)*millimeter,"radius":s.r*millimeter});skSolve(sk);
  opExtrude(context,gid+"ex",{"entities":qSketchRegion(gid+"sk"),"direction":vector(1,0,0),"endBound":BoundingType.BLIND,"endDepth":s.h*millimeter});
  const tool=qCreatedBy(gid+"ex",EntityType.BODY);var interference=false;
  for(var c in evCollision(context,{"tools":tool,"targets":cradle}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)interference=true;
  checks=append(checks,{"name":s.name,"pass":interference==s.contact});
  opDeleteBodies(context,gid+"clean",{"entities":qUnion([tool,qCreatedBy(gid+"sk",EntityType.BODY)])});
 }
 return checks;
}
