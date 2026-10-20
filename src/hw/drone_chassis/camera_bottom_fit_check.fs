function(context is Context, queries is map) {
 const all=qBodyType(qEverything(EntityType.BODY),BodyType.SOLID);var mount;var volumes=[];var collisions=[];
 const bodies=evaluateQuery(context,all);
 for(var b in bodies){const name=getProperty(context,{"entity":b,"propertyType":PropertyType.NAME});volumes=append(volumes,{"name":name,"mm3":evVolume(context,{"entities":b})/(millimeter^3)});if(name=="Camera bottom clamp")mount=b;}
 for(var b in bodies)if(b!=mount)for(var c in evCollision(context,{"tools":mount,"targets":b}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)collisions=append(collisions,getProperty(context,{"entity":b,"propertyType":PropertyType.NAME}));
 var checks=[];var specs=[];
 for(var side in [-1,1])for(var x in [-10.5,10.5]){
  for(var dz in [0,1,3,6])specs=append(specs,{"name":"M2 nut insertion "~side~" "~x~" lift "~dz,"o":vector(x,-30+side*5.6,16.3+dz),"n":vector(0,-side,0),"xd":vector(1,0,0),"r":4/sqrt(3),"h":1.6,"hex":true});
  specs=append(specs,{"name":"M2 shaft "~side~" "~x,"o":vector(x,-30+side*7.2,16.3),"n":vector(0,-side,0),"xd":vector(1,0,0),"r":1,"h":6.9,"hex":false});
  specs=append(specs,{"name":"M2 driver "~side~" "~x,"o":vector(x,-30+side*26.7,16.3),"n":vector(0,-side,0),"xd":vector(1,0,0),"r":1.5,"h":19.5,"hex":false});
 }
 for(var x in [-19,19])for(var y in [-36,-24]){
  specs=append(specs,{"name":"M3 deck shaft "~x~" "~y,"o":vector(x,y,8.3),"n":vector(0,0,1),"xd":vector(1,0,0),"r":1.5,"h":3,"hex":false});
  specs=append(specs,{"name":"M3 deck head and driver "~x~" "~y,"o":vector(x,y,11.3),"n":vector(0,0,1),"xd":vector(1,0,0),"r":2.75,"h":20,"hex":false});
 }
 for(var i=0;i<size(specs);i+=1){const s=specs[i];const gid=makeId("probe"~i);var sk=newSketchOnPlane(context,gid+"sk",{"sketchPlane":plane(s.o*millimeter,s.n,s.xd)});
 if(s.hex){var pts=[];for(var j=0;j<6;j+=1)pts=append(pts,vector(s.r*cos(j*60*degree),s.r*sin(j*60*degree))*millimeter);pts=append(pts,pts[0]);skPolyline(sk,"hex",{"points":pts});}else skCircle(sk,"circle",{"center":vector(0,0)*millimeter,"radius":s.r*millimeter});skSolve(sk);
 opExtrude(context,gid+"ex",{"entities":qSketchRegion(gid+"sk"),"direction":s.n,"endBound":BoundingType.BLIND,"endDepth":s.h*millimeter});const tool=qCreatedBy(gid+"ex",EntityType.BODY);var hit=false;
 for(var c in evCollision(context,{"tools":tool,"targets":mount}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hit=true;
 checks=append(checks,{"name":s.name,"pass":!hit});opDeleteBodies(context,gid+"del",{"entities":qUnion([tool,qCreatedBy(gid+"sk",EntityType.BODY)])});}
 // Measured stepped lower edge and its vertical insertion path.
 for(var lift in [0,5,12]) {
  const bid=makeId("boardInsertion"~lift);var sk=newSketchOnPlane(context,bid+"sk",{"sketchPlane":plane(vector(0,0,13.3+lift)*millimeter,vector(0,0,1),vector(1,0,0))});
  skRectangle(sk,"board",{"firstCorner":vector(-13.4,-30.3)*millimeter,"secondCorner":vector(13.8,-29.1)*millimeter});skSolve(sk);
  opExtrude(context,bid+"ex",{"entities":qSketchRegion(bid+"sk"),"direction":vector(0,0,1),"endBound":BoundingType.BLIND,"endDepth":39.8*millimeter});const tool=qCreatedBy(bid+"ex",EntityType.BODY);var hit=false;
  for(var c in evCollision(context,{"tools":tool,"targets":mount}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hit=true;
  checks=append(checks,{"name":"1.2 mm board insertion "~lift,"pass":!hit});opDeleteBodies(context,bid+"del",{"entities":qUnion([tool,qCreatedBy(bid+"sk",EntityType.BODY)])});
  const aid=makeId("antennaInsertion"~lift);var ask=newSketchOnPlane(context,aid+"sk",{"sketchPlane":plane(vector(0,0,13.3+lift)*millimeter,vector(0,0,1),vector(1,0,0))});
  skRectangle(ask,"step",{"firstCorner":vector(-8.8,-29.1)*millimeter,"secondCorner":vector(9.2,-28.3)*millimeter});skSolve(ask);
  opExtrude(context,aid+"ex",{"entities":qSketchRegion(aid+"sk"),"direction":vector(0,0,1),"endBound":BoundingType.BLIND,"endDepth":6*millimeter});const antenna=qCreatedBy(aid+"ex",EntityType.BODY);var antennaHit=false;
  for(var c in evCollision(context,{"tools":antenna,"targets":mount}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)antennaHit=true;
  checks=append(checks,{"name":"18 x 6 mm antenna step insertion "~lift,"pass":!antennaHit});opDeleteBodies(context,aid+"del",{"entities":qUnion([antenna,qCreatedBy(aid+"sk",EntityType.BODY)])});
 }
 const hid=makeId("rearHeaders");var hsk=newSketchOnPlane(context,hid+"sk",{"sketchPlane":plane(vector(0,0,19.6)*millimeter,vector(0,0,1),vector(1,0,0))});
 skRectangle(hsk,"envelope",{"firstCorner":vector(-13.4,-29.1)*millimeter,"secondCorner":vector(13.8,-9.1)*millimeter});skSolve(hsk);
 opExtrude(context,hid+"ex",{"entities":qSketchRegion(hid+"sk"),"direction":vector(0,0,1),"endBound":BoundingType.BLIND,"endDepth":33.5*millimeter});var headerHit=false;
 for(var c in evCollision(context,{"tools":qCreatedBy(hid+"ex",EntityType.BODY),"targets":qUnion(bodies)}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)headerHit=true;
 checks=append(checks,{"name":"20 mm rear connector envelope above clamp","pass":!headerHit});
 return {"checks":checks,"collisions":collisions,"volumes":volumes};
}
