function(context is Context, queries is map) {
 var named={};var checks=[];
 for(var b in evaluateQuery(context,qBodyType(qEverything(EntityType.BODY),BodyType.SOLID)))named[getProperty(context,{"entity":b,"propertyType":PropertyType.NAME})]=b;
 const deck=named["Electronics deck"];
 const fixed=qUnion([named["Rear landing legs"],named["Front landing legs"],named["Beam bridge"],named["Left side frame"],named["Right side frame"],named["Carbon crossbar"],named["Battery envelope"],named["Battery hatch"]]);
 for(var x in [-24,24])for(var kind in [0,1,2]) {
  const gid=makeId("braceHardware"~x~"k"~kind);
  const y=kind==0?63.9:(kind==1?66.9:96.9);
  const radius=kind==0?1.5:(kind==1?2.85:2.5);
  const length=kind==0?20:(kind==1?3:30);
  var sk=newSketchOnPlane(context,gid+"sk",{"sketchPlane":plane(vector(x,y,-12)*millimeter,vector(0,-1,0))});
  skCircle(sk,"circle",{"center":vector(0,0)*millimeter,"radius":radius*millimeter});skSolve(sk);
  opExtrude(context,gid+"ex",{"entities":qSketchRegion(gid+"sk"),"direction":vector(0,-1,0),"endBound":BoundingType.BLIND,"endDepth":length*millimeter});
  const tool=qCreatedBy(gid+"ex",EntityType.BODY);var hits=[];
  for(var c in evCollision(context,{"tools":tool,"targets":qUnion([deck,fixed])}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hits=append(hits,c["type"]);
  checks=append(checks,{"name":"M3x20 shaft/head/driver "~x~":"~kind,"hits":hits});
  opDeleteBodies(context,gid+"clean",{"entities":qUnion([tool,qCreatedBy(gid+"sk",EntityType.BODY)])});
 }
 for(var step=0;step<6;step+=1) {
  if(step>0)opTransform(context,makeId("raiseDeck"~step),{"bodies":deck,"transform":transform(vector(0,0,5)*millimeter)});
  var hits=[];for(var c in evCollision(context,{"tools":deck,"targets":fixed}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hits=append(hits,c["type"]);
  checks=append(checks,{"name":"Deck insertion clearance z="~(step*5),"hits":hits});
 }
 return checks;
}
