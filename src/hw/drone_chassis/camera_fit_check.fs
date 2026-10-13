function(context is Context, queries is map) {
 var named={};var checks=[];
 for(var b in evaluateQuery(context,qBodyType(qEverything(EntityType.BODY),BodyType.SOLID)))named[getProperty(context,{"entity":b,"propertyType":PropertyType.NAME})]=b;
 const fixture=named["Camera lens fixture"];
 for(var side in [-1,1]) {
  const cx=side==-1?-18:18.4;
  const tab=named["Camera retaining bar"];
  for(var step=0;step<6;step+=1) {
   const gid=makeId("nut"~side~"s"~step);
   var sk=newSketchOnPlane(context,gid+"sk",{"sketchPlane":plane(vector(cx+side*step*2,-44.3,57.5)*millimeter,vector(0,1,0),vector(1,0,0))});
   var points=[];for(var i=0;i<7;i+=1)points=append(points,vector(4/sqrt(3)*cos(i*60*degree),4/sqrt(3)*sin(i*60*degree))*millimeter);
   skPolyline(sk,"hex",{"points":points});skSolve(sk);
   opExtrude(context,gid+"ex",{"entities":qSketchRegion(gid+"sk"),"direction":vector(0,1,0),"endBound":BoundingType.BLIND,"endDepth":1.6*millimeter});
   const tool=qCreatedBy(gid+"ex",EntityType.BODY);var hits=[];
   for(var c in evCollision(context,{"tools":tool,"targets":fixture}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hits=append(hits,c["type"]);
   checks=append(checks,{"name":"M2 nut insertion "~side~" step "~step,"hits":hits});
   opDeleteBodies(context,gid+"clean",{"entities":qUnion([tool,qCreatedBy(gid+"sk",EntityType.BODY)])});
  }
  // Check the complete tab closing stroke against the fixture and nominal board.
  if(side==-1)for(var step=1;step<=11;step+=1) {
   opTransform(context,makeId("close"~side~"s"~step),{"bodies":tab,"transform":transform(vector(0,-0.1,0)*millimeter)});
   var hits=[];for(var c in evCollision(context,{"tools":tab,"targets":qUnion([fixture,named["ESP32-CAM envelope"]])}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hits=append(hits,c["type"]);
   checks=append(checks,{"name":"Tab closing "~side~" step "~step,"hits":hits});
  }
 }
 // Lens-center error corners: +/-0.3 mm horizontally and vertically.
 for(var dx in [-0.3,0.3])for(var dz in [-0.3,0.3]) {
  const lid=makeId("offset"~dx~"z"~dz);const moving=qUnion([named["Camera lens envelope"],named["Camera housing envelope"]]);
  opTransform(context,lid,{"bodies":moving,"transform":transform(vector(dx,0,dz)*millimeter)});
  var hits=[];for(var c in evCollision(context,{"tools":moving,"targets":qUnion([fixture,named["Camera lens keeper"]])}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hits=append(hits,c["type"]);
  checks=append(checks,{"name":"Lens position error "~dx~","~dz,"hits":hits});
  opTransform(context,lid+"restore",{"bodies":moving,"transform":transform(vector(-dx,0,-dz)*millimeter)});
 }
 for(var side in [-1,1]) {
  const cx=side*10;
  for(var step=0;step<6;step+=1) {
   const gid=makeId("keeperNut"~side~"s"~step);
   var sk=newSketchOnPlane(context,gid+"sk",{"sketchPlane":plane(vector(cx,-44.7+step*2,40.3)*millimeter,vector(0,0,1),vector(1,0,0))});
   var points=[];for(var i=0;i<7;i+=1)points=append(points,vector(4/sqrt(3)*cos(i*60*degree),4/sqrt(3)*sin(i*60*degree))*millimeter);
   skPolyline(sk,"hex",{"points":points});skSolve(sk);
   opExtrude(context,gid+"ex",{"entities":qSketchRegion(gid+"sk"),"direction":vector(0,0,1),"endBound":BoundingType.BLIND,"endDepth":1.6*millimeter});
   const tool=qCreatedBy(gid+"ex",EntityType.BODY);var hits=[];
   for(var c in evCollision(context,{"tools":tool,"targets":fixture}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hits=append(hits,c["type"]);
   checks=append(checks,{"name":"Keeper nut entry "~side~" step "~step,"hits":hits});
   opDeleteBodies(context,gid+"clean",{"entities":qUnion([tool,qCreatedBy(gid+"sk",EntityType.BODY)])});
  }
  for(var radius in [1.5,2]) {
   const gid=makeId("driver"~side~"r"~radius);
   var sk=newSketchOnPlane(context,gid+"sk",{"sketchPlane":plane(vector(cx,-44.7,50.5)*millimeter,vector(0,0,1))});
   skCircle(sk,"circle",{"center":vector(0,0)*millimeter,"radius":radius*millimeter});skSolve(sk);
   opExtrude(context,gid+"ex",{"entities":qSketchRegion(gid+"sk"),"direction":vector(0,0,1),"endBound":BoundingType.BLIND,"endDepth":(radius==2?1.6:25)*millimeter});
   const tool=qCreatedBy(gid+"ex",EntityType.BODY);var hits=[];
   for(var c in evCollision(context,{"tools":tool,"targets":qUnion([fixture,named["Camera retaining bar"],named["ESP32-CAM envelope"]])}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hits=append(hits,c["type"]);
   checks=append(checks,{"name":"Keeper driver/head "~side~" radius "~radius,"hits":hits});
   opDeleteBodies(context,gid+"clean",{"entities":qUnion([tool,qCreatedBy(gid+"sk",EntityType.BODY)])});
  }
 }
 const lens=named["Camera lens envelope"];const housing=named["Camera housing envelope"];const keeper=named["Camera lens keeper"];
 for(var step=1;step<=9;step+=1) {
  opTransform(context,makeId("keeperClose"~step),{"bodies":keeper,"transform":transform(vector(0,0,-0.1)*millimeter)});
  opTransform(context,makeId("lensSettle"~step),{"bodies":qUnion([lens,housing]),"transform":transform(vector(0,0,-0.05)*millimeter)});
  var hits=[];
  for(var c in evCollision(context,{"tools":qUnion([keeper,lens,housing]),"targets":fixture}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hits=append(hits,c["type"]);
  for(var c in evCollision(context,{"tools":keeper,"targets":qUnion([lens,housing,named["ESP32-CAM envelope"]])}))if(c["type"]==ClashType.INTERFERE||c["type"]==ClashType.TOOL_IN_TARGET||c["type"]==ClashType.TARGET_IN_TOOL)hits=append(hits,c["type"]);
  checks=append(checks,{"name":"Lens keeper closing "~step,"hits":hits});
 }
 return checks;
}
