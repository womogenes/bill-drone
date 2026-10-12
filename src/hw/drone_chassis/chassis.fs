FeatureScript 3083;
import(path : "onshape/std/geometry.fs", version : "3083.0");
function smSolid(context is Context,id is Id) returns Query {return qBodyType(qCreatedBy(id,EntityType.BODY),BodyType.SOLID);}
function smPoly(context is Context,id is Id,p is array,o is Vector,n is Vector,xd is Vector,h is number) {
 var sk=newSketchOnPlane(context,id+"sk",{"sketchPlane":plane(o*millimeter,n,xd)});var pts=[];
 for(var v in p)pts=append(pts,vector(v[0],v[1])*millimeter);pts=append(pts,pts[0]);skPolyline(sk,"outline",{"points":pts});skSolve(sk);
 opExtrude(context,id+"ex",{"entities":qSketchRegion(id+"sk"),"direction":n,"endBound":BoundingType.BLIND,"endDepth":h*millimeter});
 opDeleteBodies(context,id+"clean",{"entities":qCreatedBy(id+"sk",EntityType.BODY)});
}
function smBlock(context is Context,id is Id,x0 is number,x1 is number,y0 is number,y1 is number,z0 is number,z1 is number) {smPoly(context,id,[[x0,y0],[x1,y0],[x1,y1],[x0,y1]],vector(0,0,z0),vector(0,0,1),vector(1,0,0),z1-z0);}
function smCyl(context is Context,id is Id,o is Vector,n is Vector,r is number,h is number) {
 var sk=newSketchOnPlane(context,id+"sk",{"sketchPlane":plane(o*millimeter,n)});skCircle(sk,"circle",{"center":vector(0,0)*millimeter,"radius":r*millimeter});skSolve(sk);
 opExtrude(context,id+"ex",{"entities":qSketchRegion(id+"sk"),"direction":n,"endBound":BoundingType.BLIND,"endDepth":h*millimeter});opDeleteBodies(context,id+"clean",{"entities":qCreatedBy(id+"sk",EntityType.BODY)});
}
function smJoin(context is Context,id is Id,q is Query) {opBoolean(context,id,{"tools":q,"operationType":BooleanOperationType.UNION});}
function smCut(context is Context,id is Id,t is Query,q is Query) {opBoolean(context,id,{"targets":t,"tools":q,"operationType":BooleanOperationType.SUBTRACTION});}
function smHole(context is Context,id is Id,t is Query,o is Vector,n is Vector,d is number,h is number) {smCyl(context,id+"tool",o,n,d/2,h);smCut(context,id+"cut",t,smSolid(context,id+"tool"));}
function smHex(context is Context,id is Id,t is Query,o is Vector,n is Vector,xd is Vector,af is number,h is number) {var p=[];for(var i=0;i<6;i+=1)p=append(p,[af/sqrt(3)*cos(i*60*degree),af/sqrt(3)*sin(i*60*degree)]);smPoly(context,id+"tool",p,o,n,xd,h);smCut(context,id+"cut",t,smSolid(context,id+"tool"));}
function smLabel(context is Context,q is Query,name is string,c) {setProperty(context,{"entities":q,"propertyType":PropertyType.NAME,"value":name});setProperty(context,{"entities":q,"propertyType":PropertyType.APPEARANCE,"value":c});}

function smBevel(context is Context,id is Id,body is Query,points is array,w is number) {
 var edges=[];for(var p in points)edges=append(edges,qContainsPoint(qOwnedByBody(body,EntityType.EDGE),p*millimeter));
 opChamfer(context,id,{"entities":qUnion(edges),"chamferType":ChamferType.EQUAL_OFFSETS,"width":w*millimeter});
}

annotation {"Feature Type Name":"Drone chassis"}
export const chassis=defineFeature(function(context is Context,id is Id,definition is map)
precondition { annotation {"Name":"Show hardware"} definition.refs is boolean; }
{
const blue=color(0.16,0.48,0.67); const orange=color(0.92,0.51,0.12);
// Compact backbone: beam clamp, two upper landing rails and two battery hangers.
// The deck attaches to the removable beam cap, so no tall deck posts are needed.
smBlock(context,id+"tray",-25,25,-14,14,-12,-5.3);
const body=smSolid(context,id+"tray");
for(var y in [-14,5.5]) {
 smBlock(context,id+("clampRail"~y),-25,25,y,y+8.5,-9,4.4);
 smJoin(context,id+("clampRailJoin"~y),qUnion([body,smSolid(context,id+("clampRail"~y))]));
}
for(var sx in [-1,1]) {
 const sid=id+("backbone"~sx);const x=sx==1?18.5:-23.5;
 smBlock(context,sid+"rail",x,x+5,-50.7,50.7,-12,-6);
 smJoin(context,sid+"railJoin",qUnion([body,smSolid(context,sid+"rail")]));
 // Tapered ribs carry each landing frame directly into the central clamp.
 for(var sy in [-1,1]) {
  smPoly(context,sid+("rib"~sy),[[sy*14,-6],[sy*50.7,-6],[sy*50.7,-3],[sy*14,4]],vector(x,0,0),vector(1,0,0),vector(0,1,0),5);
  smJoin(context,sid+("ribJoin"~sy),qUnion([body,smSolid(context,sid+("rib"~sy))]));
 }
 // Four-mm battery guides are symmetric front-to-back, including both hatch shoulders.
 smPoly(context,sid+"hanger",[[-12,-51],[12,-51],[12,-42],[6,-36],[6,-20],[14,-12],[-14,-12],[-6,-20],[-6,-36],[-12,-42]],vector(sx==1?18.5:-22.5,0,0),vector(1,0,0),vector(0,1,0),4);
 smJoin(context,sid+"hangerJoin",qUnion([body,smSolid(context,sid+"hanger")]));
}
// Camera construction remains in its original local coordinates, then moves as one part.
smBlock(context,id+"deck",-39,39,-111,7.5,11,14);const deck=smSolid(context,id+"deck");
for(var x in [-26,26])for(var y in [-75.5,-55.5]) {
 const dy=y+65.5;
 smHole(context,id+("deckHole"~x~"y"~y),deck,vector(x,y,10),vector(0,0,1),x*dy>0?3.5:4.6,x*dy>0?5:3);
}
// ESP32-CAM 27 x 40.5 mm PCB: open central front/back, edge grooves only.
for(var side in [-1,1]){
 const cid=id+("camPost"~side); const cx=side==1?14:-24;
 smBlock(context,cid+"shape",cx,cx+10,-110.2,-103,14,59);
 smBlock(context,cid+"frontLip",side==1?12.8:-14,side==1?14:-12.8,-110.2,-108.2,18,59);
 smBlock(context,cid+"rearLip",side==1?12.8:-14,side==1?14:-12.8,-106,-103,18,59);
 smPoly(context,cid+"brace",[[-103,14],[-88,14],[-103,40]],vector(side==1?21:-24,0,0),vector(1,0,0),vector(0,1,0),3);
 smJoin(context,cid+"join",qUnion([deck,smSolid(context,cid+"shape"),smSolid(context,cid+"frontLip"),smSolid(context,cid+"rearLip"),smSolid(context,cid+"brace")]));
 smHole(context,cid+"pilot",deck,vector(side*19,-106.6,53.5),vector(0,0,1),2.6,5.6);
}
for(var side in [-1,1]){
 smBlock(context,id+("camSeat"~side),side==1?10:-14,side==1?14:-10,-110.2,-103,14,18);
 smJoin(context,id+("camSeatJoin"~side),qUnion([deck,smSolid(context,id+("camSeat"~side))]));
}
smBlock(context,id+"cameraCap",-24,24,-110.2,-103,59,62);const cameraCap=smSolid(context,id+"cameraCap");
for(var x in [-19,19])smHole(context,id+("cameraCapHole"~x),cameraCap,vector(x,-106.6,58),vector(0,0,1),3.5,5);
smLabel(context,cameraCap,"Camera cap",orange);
smLabel(context,deck,"Electronics deck",orange);

opTransform(context,id+"centerDeck",{"bodies":qUnion([deck,cameraCap]),"transform":transform(vector(0,65.5,0)*millimeter)});
// Raised tape rails keep the perfboard above M3 deck-screw heads. Fit screws before taping the board.
for(var x in [-32,29]) {
 smBlock(context,id+("boardRail"~x),x,x+3,18,68,14,18);
 smJoin(context,id+("boardRailJoin"~x),qUnion([deck,smSolid(context,id+("boardRail"~x))]));
}
// IMU remains on the centered tube cap, accessible through its own deck window.
smBlock(context,id+"imuWindow",-20,20,-14,14,10,16);
smCut(context,id+"imuWindowCut",deck,smSolid(context,id+"imuWindow"));
smBlock(context,id+"cap",-32,32,-14,14,5.3,11);const cap=smSolid(context,id+"cap");
for(var x in [0])for(var y in [-10,10]){
 const sid=id+("tubeBolt"~x~"y"~y);
 smHole(context,sid,body,vector(x,y,-13),vector(0,0,1),3.5,23);
 smHex(context,sid+"nut",body,vector(x,y,-12.1),vector(0,0,1),vector(1,0,0),6.2,4.3);
 smHole(context,sid+"cap",cap,vector(x,y,5),vector(0,0,1),3.5,7);
 // Recess the original clamp screw heads to their previous seat height.
 smHole(context,sid+"head",cap,vector(x,y,9.3),vector(0,0,1),6.2,3);
}
for(var x in [-7.5,7.5]){
 smCyl(context,id+("imuBoss"~x),vector(x,0,11),vector(0,0,1),3.5,4);
 smJoin(context,id+("imuBossJoin"~x),qUnion([cap,smSolid(context,id+("imuBoss"~x))]));
 smHole(context,id+("imuPilot"~x),cap,vector(x,0,8.5),vector(0,0,1),2.6,6.6);
}
// Two diagonal deck screws and two locating pegs on the central cap.
for(var x in [-26,26])for(var y in [-10,10]) {
 const did=id+("deckMount"~x~"y"~y);
 if(x*y>0) {
  smHole(context,did+"bolt",cap,vector(x,y,5),vector(0,0,1),3.5,7);
  smHex(context,did+"nut",cap,vector(x,y,5.3),vector(0,0,1),vector(1,0,0),6.2,3.2);
  smBlock(context,did+"slot",x>0?x:-33,x>0?33:x,y-3.1,y+3.1,5.3,8.5);
  smCut(context,did+"slotCut",cap,smSolid(context,did+"slot"));
 } else {
  smCyl(context,did+"peg",vector(x,y,11),vector(0,0,1),2,1.5);
  smJoin(context,did+"pegJoin",qUnion([cap,smSolid(context,did+"peg")]));
 }
}
smLabel(context,cap,"IMU tube cap",blue);
// Slender diagonal webs grow at 45 degrees from the central hanger feet.
// They brace the landing rails and reduce the unsupported underside during printing.
for(var sx in [-1,1])for(var sy in [-1,1]) {
 const bid=id+("landingWeb"~sx~"y"~sy);
 smPoly(context,bid,[[sy*14,-50],[sy*50.7,-13.3],[sy*50.7,-7.3],[sy*14,-44]],vector(sx==1?18.5:-21.5,0,0),vector(1,0,0),vector(0,1,0),3);
 smJoin(context,bid+"join",qUnion([body,smSolid(context,bid)]));
}
// Two planar inverted-V frames; the four feet are integral and unconnected.
for(var side in [-1,1]){
 const sid=id+("gear"~side); const gy=side==-1?-50.7:58.7;
 smPoly(context,sid+"shape",[[-28,-4],[28,-4],[28,-18],[78,-80],[78,-87],[62,-87],[62,-80],[20,-20],[-20,-20],[-62,-80],[-62,-87],[-78,-87],[-78,-80],[-28,-18]],vector(0,gy,0),vector(0,-1,0),vector(1,0,0),8);
 const gear=smSolid(context,sid+"shape");
 for(var x in [-24,24]){
  const bid=sid+("mount"~x);
  smBlock(context,bid,x-5,x+5,side==-1?-50.7:44.7,side==-1?-44.7:50.7,-19,-2);
  smJoin(context,bid+"join",qUnion([body,smSolid(context,bid)]));
  smHole(context,bid+"gearHole",gear,vector(x,gy+1,-12),vector(0,-1,0),3.5,10);
  smHole(context,bid+"bodyHole",body,vector(x,side*51.7,-12),vector(0,-side,0),3.5,8);
  smHex(context,bid+"nut",body,vector(x,side*47.7,-12),vector(0,-side,0),vector(1,0,0),6.2,3.2);
  // Side-load the nut from outside; the 3 mm wall facing the leg carries screw tension.
  smBlock(context,bid+"nutAccess",x>0?x:-30,x>0?30:x,side==-1?-47.7:44.5,side==-1?-44.5:47.7,-15.1,-8.9);
  smCut(context,bid+"nutAccessCut",body,smSolid(context,bid+"nutAccess"));
  // Same depth for the hex and access slot removes the narrow internal step.
 }
 opFillet(context,sid+"roots",{"entities":qUnion([qContainsPoint(qOwnedByBody(gear,EntityType.EDGE),vector(-20,gy-4,-20)*millimeter),qContainsPoint(qOwnedByBody(gear,EntityType.EDGE),vector(20,gy-4,-20)*millimeter)]),"radius":3*millimeter});
 // Substantial top-corner chamfers on the exposed landing crossbar.
 smBevel(context,sid+"crossbarCorners",gear,[vector(-28,gy-4,-4),vector(28,gy-4,-4)],2);
 smBevel(context,sid+"footBevel",gear,[vector(-78,gy-4,-87),vector(-62,gy-4,-87),vector(62,gy-4,-87),vector(78,gy-4,-87)],0.6);
 smLabel(context,gear,side==-1?"Front landing legs":"Rear landing legs",blue);
}
// Two bottom-up hatch screws remain threaded in side-loaded nuts during battery changes.
for(var side in [-1,1]) {
 const hid=id+("hatchBoss"~side);const x=side*28;
 smBlock(context,hid,side==1?18.5:-34,side==1?34:-18.5,-14,14,-51,-38);
 smJoin(context,hid+"join",qUnion([body,smSolid(context,hid)]));
 smHole(context,hid+"bolt",body,vector(x,0,-52),vector(0,0,1),3.5,15);
 smHex(context,hid+"nut",body,vector(x,0,-45),vector(0,0,1),vector(1,0,0),6.2,3.2);
 smBlock(context,hid+"entry",side==1?28:-34.1,side==1?34.1:-28,-3.1,3.1,-45,-41.8);
 smCut(context,hid+"entryCut",body,smSolid(context,hid+"entry"));
 // Matching front/rear sockets let either frame accept the unchanged hatch pegs.
 for(var y in [-9,9])smHole(context,hid+("index"~y),body,vector(x,y,-51.1),vector(0,0,1),4.6,1.9);
}
// Flat-printing battery hatch: loosen M3 x 16 socket screws four turns, lower 2 mm,
// slide 7 mm toward the camera, then lower hatch and battery through the large keyholes.
smBlock(context,id+"hatch",-21.5,21.5,-43,43,-55,-51);const hatch=smSolid(context,id+"hatch");
smBlock(context,id+"hatchWings",-35,35,-14,14,-55,-51);
smJoin(context,id+"hatchJoin",qUnion([hatch,smSolid(context,id+"hatchWings")]));
// Round all eight exposed plan-view corners, through the full plate thickness.
var hatchCorners=[];
for(var sx in [-1,1])for(var sy in [-1,1]) {
 hatchCorners=append(hatchCorners,qContainsPoint(qOwnedByBody(hatch,EntityType.EDGE),vector(sx*21.5,sy*43,-53)*millimeter));
 hatchCorners=append(hatchCorners,qContainsPoint(qOwnedByBody(hatch,EntityType.EDGE),vector(sx*35,sy*14,-53)*millimeter));
}
opFillet(context,id+"hatchCornerRounds",{"entities":qUnion(hatchCorners),"radius":3*millimeter});
smBlock(context,id+"hatchWindow",-9,9,-25,25,-56,-50);
smCut(context,id+"hatchWindowCut",hatch,smSolid(context,id+"hatchWindow"));
for(var x in [-17,11]) {
 smBlock(context,id+("hatchPad"~x),x,x+6,-32,32,-51,-48);
 smJoin(context,id+("hatchPadJoin"~x),qUnion([hatch,smSolid(context,id+("hatchPad"~x))]));
}
for(var y in [-39.5,37.5]) {
 smBlock(context,id+("hatchStop"~y),-18.2,18.2,y,y+2,-51,-43.8);
 smJoin(context,id+("hatchStopJoin"~y),qUnion([hatch,smSolid(context,id+("hatchStop"~y))]));
}
for(var x in [-28,28]) {
 const kid=id+("hatchKey"~x);
 smBlock(context,kid+"slot",x-1.75,x+1.75,0,7,-56,-50);
 smCut(context,kid+"slotCut",hatch,smSolid(context,kid+"slot"));
 smHole(context,kid+"seat",hatch,vector(x,0,-56),vector(0,0,1),3.5,6);
 smHole(context,kid+"release",hatch,vector(x,7,-56),vector(0,0,1),6.6,6);
 smCyl(context,kid+"peg",vector(x,-9,-51),vector(0,0,1),2,1.5);
 smJoin(context,kid+"pegJoin",qUnion([hatch,smSolid(context,kid+"peg")]));
}
// Larger battery-contact lead-ins; tiny edge chamfers omitted.
// Clip only the four vertical outside corners; keep the top perimeter square.
smBevel(context,id+"deckCornerBevel",deck,[vector(-39,-45.5,12.5),vector(39,-45.5,12.5),vector(-39,73,12.5),vector(39,73,12.5)],5);
var padEdges=[];
for(var x in [-17,11])padEdges=concatenateArrays([padEdges,[vector(x,0,-48),vector(x+6,0,-48),vector(x+3,-32,-48),vector(x+3,32,-48)]]);
smBevel(context,id+"hatchPadBevel",hatch,padEdges,0.4);
smBevel(context,id+"hatchStopBevel",hatch,[vector(0,-37.5,-43.8),vector(0,37.5,-43.8)],0.4);
smLabel(context,hatch,"Battery hatch",orange);
// Three printable modules. Side frames lie on their broad inner faces; the bridge
// lies on its bottom face. Split gaps leave 0.3 mm between the assembled modules.
for(var sx in [-1,1]) {
 const sid=id+("moduleGap"~sx);
 smBlock(context,sid,sx==1?18.2:-18.5,sx==1?18.5:-18.2,-80,80,-60,10);
 smCut(context,sid+"cut",body,smSolid(context,sid));
}
const bridge=qContainsPoint(body,vector(5,10,-8)*millimeter);
const leftFrame=qContainsPoint(body,vector(-20,0,-40)*millimeter);
const rightFrame=qContainsPoint(body,vector(20,0,-40)*millimeter);
for(var sx in [-1,1])for(var y in [-10,10]) {
 const jid=id+("moduleJoint"~sx~"y"~y);const frame=sx==1?rightFrame:leftFrame;
 // Large shoulders and deep tongues carry vertical/bending loads; screws clamp
 // the tongues onto pocket floors and prevent withdrawal, not just friction-fit.
 smBlock(context,jid+"boss",sx==1?18.5:-29,sx==1?29:-18.5,y-5,y+5,-20,4.4);
 smJoin(context,jid+"bossJoin",qUnion([frame,smSolid(context,jid+"boss")]));
 var key=[];for(var p in [[17.8,-4],[26.2,-4],[27,-3.2],[27,3.2],[26.2,4],[17.8,4]])key=append(key,[sx*p[0],y+p[1]]);
 smPoly(context,jid+"key",key,vector(0,0,-12),vector(0,0,1),vector(1,0,0),6.7);
 smJoin(context,jid+"keyJoin",qUnion([bridge,smSolid(context,jid+"key")]));
 // 0.3 mm clearance at each side and at the tip; seated underside has no gap.
 smBlock(context,jid+"socket",sx==1?18.4:-27.3,sx==1?27.3:-18.4,y-4.3,y+4.3,-12,-5.0);
 smCut(context,jid+"socketCut",frame,smSolid(context,jid+"socket"));
 // M3 x 12 socket screw: seat -5.3, tip -17.3; nut roof -14.5, bottom -16.9.
 smHole(context,jid+"bolt",qUnion([bridge,frame]),vector(sx*24,y,-21),vector(0,0,1),3.5,27);
 smHole(context,jid+"headAccess",frame,vector(sx*24,y,-5.0),vector(0,0,1),6.6,10);
 smHex(context,jid+"nut",frame,vector(sx*24,y,-17.7),vector(0,0,1),vector(1,0,0),6.2,3.2);
 smBlock(context,jid+"nutEntry",sx==1?24:-30,sx==1?30:-24,y-3.1,y+3.1,-17.7,-14.5);
 smCut(context,jid+"nutEntryCut",frame,smSolid(context,jid+"nutEntry"));
}
// Joint bosses stop at the measured rectangular tube clearance envelope.
smBlock(context,id+"moduleTubeClearance",-35,35,-5.5,5.5,-5.3,6);
smCut(context,id+"moduleTubeClearanceCut",qUnion([leftFrame,rightFrame]),smSolid(context,id+"moduleTubeClearance"));
// Break exposed outline corners through frame thickness, preserving joint seats.
for(var sx in [-1,1]) {
 const frame=sx==1?rightFrame:leftFrame;var points=[];
 for(var sy in [-1,1]) {
  for(var y in [5.5,15])points=append(points,vector(sx*26,sy*y,4.4));
  for(var y in [5,15])points=append(points,vector(sx*26,sy*y,-20));
  for(var y in [44.7,50.7])for(var z in [-2,-19])points=append(points,vector(sx*26,sy*y,z));
  for(var z in [-38,-51])points=append(points,vector(sx*30,sy*14,z));
 }
 smBevel(context,id+("frameCornerChamfers"~sx),frame,points,1);
}
smLabel(context,bridge,"Beam bridge",blue);
smLabel(context,leftFrame,"Left side frame",blue);
smLabel(context,rightFrame,"Right side frame",blue);

if(definition.refs){
 smBlock(context,id+"bar",-210,210,-5.25,5.25,-5,5);smLabel(context,smSolid(context,id+"bar"),"Carbon crossbar",color(0.12,0.13,0.14));
 smBlock(context,id+"battery",-18,18,-37,37,-48,-13);smLabel(context,smSolid(context,id+"battery"),"Battery envelope",color(0.35,0.35,0.38));
 for(var px in [-172,172]){
  smCyl(context,id+("prop"~px),vector(px,0,94),vector(0,0,1),114.3,0.8);smLabel(context,smSolid(context,id+("prop"~px)),"Propeller swept envelope",color(0.7,0.75,0.8,0.18));
 }
 smBlock(context,id+"camBoard",-13.5,13.5,-42.5,-40.9,18,58.5);smLabel(context,smSolid(context,id+"camBoard"),"ESP32-CAM envelope",color(0.15,0.3,0.2));
 smCyl(context,id+"lens",vector(0,-42.5,42),vector(0,-1,0),4,5);smLabel(context,smSolid(context,id+"lens"),"Camera lens envelope",color(0.12,0.12,0.13));
 smBlock(context,id+"imu",-11,11,-8,8,15,16.6);smLabel(context,smSolid(context,id+"imu"),"IMU envelope",color(0.2,0.4,0.75));
 smBlock(context,id+"board",-32,32,10,70,18,19.6);smLabel(context,smSolid(context,id+"board"),"Perfboard placeholder",color(0.21,0.5,0.25));
 smBlock(context,id+"espMain",-27.5,27.5,15,44,19.6,33.6);smLabel(context,smSolid(context,id+"espMain"),"ESP32 placeholder",color(0.12,0.3,0.2));
 smBlock(context,id+"regulator",-22.5,22.5,45,70,19.6,34.6);smLabel(context,smSolid(context,id+"regulator"),"Regulator placeholder",color(0.15,0.35,0.65));
 for(var sx in [-65,65]){smBlock(context,id+("esc"~sx),sx-13,sx+13,-25,25,5,12);smLabel(context,smSolid(context,id+("esc"~sx)),"ESC envelope",color(0.25,0.25,0.27));}
}
});
