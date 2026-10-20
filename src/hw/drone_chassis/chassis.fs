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
// The electronics deck is now the beam cap; existing bridge and frame are unchanged.
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
// Single flat-bottom platform replaces the old IMU cap and directly clamps the beam.
// Original M3 clamp centers (0,+/-10), underside z=5.3, and screw seats z=9.3 preserved.
smPoly(context,id+"deck",[[-23,-45.5],[23,-45.5],[25,-43.5],[25,-18],[38,-14],[40,-12],[40,82],[38,84],[-38,84],[-40,82],[-40,-12],[-38,-14],[-25,-18],[-25,-43.5]],vector(0,0,5.3),vector(0,0,1),vector(1,0,0),3);
const deck=smSolid(context,id+"deck");
for(var y in [-10,10]) {
 const sid=id+("tubeBolt0y"~y);
 smHole(context,sid,body,vector(0,y,-13),vector(0,0,1),3.5,23);
 smHex(context,sid+"nut",body,vector(0,y,-12.1),vector(0,0,1),vector(1,0,0),6.2,4.3);
 smCyl(context,sid+"deckBoss",vector(0,y,8.3),vector(0,0,1),5,2.7);
 smJoin(context,sid+"deckBossJoin",qUnion([deck,smSolid(context,sid+"deckBoss")]));
 smHole(context,sid+"deckBolt",deck,vector(0,y,5),vector(0,0,1),3.5,7);
 smHole(context,sid+"deckHead",deck,vector(0,y,9.3),vector(0,0,1),6.6,3);
}
// Perfboard 90 mm front/back (Y), 70 mm across (X). 66 x 86 mm M2 pattern.
for(var x in [-33,33])for(var y in [-8,78]) {
 const pid=id+("perfboardMount"~x~"y"~y);
 smCyl(context,pid,vector(x,y,8.3),vector(0,0,1),4,5);
 smJoin(context,pid+"join",qUnion([deck,smSolid(context,pid)]));
 smHole(context,pid+"bolt",deck,vector(x,y,5),vector(0,0,1),2.4,9);
 smHex(context,pid+"nut",deck,vector(x,y,9.3),vector(0,0,1),vector(1,0,0),4.5,2);
 smBlock(context,pid+"entry",x>0?x:x-4.1,x>0?x+4.1:x,y-2.25,y+2.25,9.3,11.3);
 smCut(context,pid+"entryCut",deck,smSolid(context,pid+"entry"));
}
for(var x in [-19,19])for(var y in [-36,-24])
 smHole(context,id+("futureCameraHole"~x~"y"~y),deck,vector(x,y,5),vector(0,0,1),3.5,4);
// Two windows retain a 14 mm central spine beneath the beam-clamping screws.
for(var sx in [-1,1]) {
 const wid=id+("deckWindow"~sx);
 smBlock(context,wid,sx>0?7:-28,sx>0?28:-7,0,68,5,9);
 smCut(context,wid+"cut",deck,smSolid(context,wid));
}
// Rear deck braces share the existing rear landing-leg screw holes and nuts.
// 5.2-mm tabs move the M3 x 20 tips clear of the existing frame rail.
smBlock(context,id+"deckRearTie",-35,35,58.7,63.9,5.3,8.3);
smJoin(context,id+"deckRearTieJoin",qUnion([deck,smSolid(context,id+"deckRearTie")]));
for(var sx in [-1,1]) {
 const sid=id+("deckLegBrace"~sx);const x=sx*24;
 smBlock(context,sid+"tab",x-5,x+5,58.7,63.9,-18,8.3);
 smPoly(context,sid+"rib",[[63.9,-8],[77.2,5.3],[63.9,5.3]],vector(sx>0?26:-29,0,0),vector(1,0,0),vector(0,1,0),3);
 smJoin(context,sid+"join",qUnion([deck,smSolid(context,sid+"tab"),smSolid(context,sid+"rib")]));
 smHole(context,sid+"bolt",deck,vector(x,64.2,-12),vector(0,-1,0),3.5,6);
 smBevel(context,sid+"bottomCorners",deck,[vector(x-5,61.3,-18),vector(x+5,61.3,-18)],1);
}
smLabel(context,deck,"Electronics deck",orange);
// Slender diagonal webs grow at 45 degrees from the central hanger feet.
// They brace the landing rails and reduce the unsupported underside during printing.
for(var sx in [-1,1])for(var sy in [-1,1]) {
 const bid=id+("landingWeb"~sx~"y"~sy);
 smPoly(context,bid,[[sy*14,-50],[sy*50.7,-13.3],[sy*50.7,-7.3],[sy*14,-44]],vector(sx==1?18.5:-21.5,0,0),vector(1,0,0),vector(0,1,0),3);
 smJoin(context,bid+"join",qUnion([body,smSolid(context,bid)]));
}
// Closed landing frames with extended tapered haunches and a flat foot tie.
// Keep the 8 mm mounting thickness and all shared deck/frame bolt interfaces.
for(var side in [-1,1]){
 const sid=id+("gear"~side); const gy=side==-1?-50.7:58.7;
 smPoly(context,sid+"shape",[[-29,-2],[29,-2],[29,-19],[78,-80],[78,-87],[62,-87],[62,-80],[8,-24],[-8,-24],[-62,-80],[-62,-87],[-78,-87],[-78,-80],[-29,-19]],vector(0,gy,0),vector(0,-1,0),vector(1,0,0),8);
 const gear=smSolid(context,sid+"shape");
 smBlock(context,sid+"footTie",-62,62,gy-8,gy,-87,-80);
 smJoin(context,sid+"footTieJoin",qUnion([gear,smSolid(context,sid+"footTie")]));
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
 opFillet(context,sid+"roots",{"entities":qUnion([qContainsPoint(qOwnedByBody(gear,EntityType.EDGE),vector(-8,gy-4,-24)*millimeter),qContainsPoint(qOwnedByBody(gear,EntityType.EDGE),vector(8,gy-4,-24)*millimeter)]),"radius":8*millimeter});
 // Match frame bosses at X +/-29 and Z -2, with a light exposed-corner break.
 smBevel(context,sid+"crossbarCorners",gear,[vector(-29,gy-4,-2),vector(29,gy-4,-2)],0.6);
 smBevel(context,sid+"footBevel",gear,[vector(-78,gy-4,-87),vector(78,gy-4,-87)],0.6);
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

// Symmetric four-bolt camera clamp, centered on the deck's Y=-30 mounting pattern.
smPoly(context,id+"cameraCollar",[[-23,-42],[23,-42],[24,-41],[24,-19],[23,-18],[-23,-18],[-24,-19],[-24,-41]],vector(0,0,8.3),vector(0,0,1),vector(1,0,0),3);
const cameraMount=smSolid(context,id+"cameraCollar");
for(var x in [-19,19])for(var y in [-36,-24])
 smHole(context,id+("cameraFootHole"~x~"y"~y),cameraMount,vector(x,y,8),vector(0,0,1),3.5,4);
// Full 6 mm lower edge capture, plus 0.3 mm vertical allowance.
// Thin outer PCB sections are 1.2 mm; center 18 mm has a 0.8 mm rear buildup.
smBlock(context,id+"cameraBar",-14.9,14.9,-37.2,-22.8,11.3,19.6);
smJoin(context,id+"cameraBarJoin",qUnion([cameraMount,smSolid(context,id+"cameraBar")]));
smBlock(context,id+"cameraSlot",-15,15,-30.9,-29.1,13.3,20);
smCut(context,id+"cameraSlotCut",cameraMount,smSolid(context,id+"cameraSlot"));
// Mirror the central relief so either jaw can be the antenna-facing jaw.
// 19 mm relief width clears the measured center 18 mm and its 0.2 mm X offset.
smBlock(context,id+"antennaRelief",-9.5,9.5,-31.9,-28.1,13.3,20);
smCut(context,id+"antennaReliefCut",cameraMount,smSolid(context,id+"antennaRelief"));
// Only the non-antenna jaw receives two M2 x 8 flat-ended set screws and nuts.
for(var side in [-1,1])for(var x in [-10.5,10.5]) {
 const cid=id+("cameraClamp"~side~"x"~x);
 smHole(context,cid+"screw",cameraMount,vector(x,-30+side*7.7,16.3),vector(0,-side,0),2.4,8);
 smHex(context,cid+"nut",cameraMount,vector(x,-30+side*5.7,16.3),vector(0,-side,0),vector(1,0,0),4.5,2.2);
 smBlock(context,cid+"nutEntry",x-2.6,x+2.6,side==-1?-35.7:-26.5,side==-1?-33.5:-24.3,16.3,20);
 smCut(context,cid+"nutEntryCut",cameraMount,smSolid(context,cid+"nutEntry"));
}
smBevel(context,id+"cameraBarCorners",cameraMount,[vector(-14.9,-37.2,16),vector(14.9,-37.2,16),vector(-14.9,-22.8,16),vector(14.9,-22.8,16)],0.5);
smLabel(context,cameraMount,"Camera bottom clamp",blue);

if(definition.refs){
 smBlock(context,id+"bar",-210,210,-5.25,5.25,-5,5);smLabel(context,smSolid(context,id+"bar"),"Carbon crossbar",color(0.12,0.13,0.14));
 smBlock(context,id+"battery",-18,18,-37,37,-48,-13);smLabel(context,smSolid(context,id+"battery"),"Battery envelope",color(0.35,0.35,0.38));
 for(var px in [-172,172]){
  smCyl(context,id+("prop"~px),vector(px,0,94),vector(0,0,1),114.3,0.8);smLabel(context,smSolid(context,id+("prop"~px)),"Propeller swept envelope",color(0.7,0.75,0.8,0.18));
 }
 smBlock(context,id+"camBoard",-13.4,13.8,-30.3,-29.1,13.3,53.1);smBlock(context,id+"camAntennaStep",-8.8,9.2,-29.1,-28.3,13.3,19.3);smJoin(context,id+"camAntennaJoin",qUnion([smSolid(context,id+"camBoard"),smSolid(context,id+"camAntennaStep")]));smLabel(context,smSolid(context,id+"camBoard"),"ESP32-CAM envelope",color(0.15,0.3,0.2));
 smCyl(context,id+"lens",vector(0,-34.9,43.9),vector(0,-1,0),4,3.4);smLabel(context,smSolid(context,id+"lens"),"Camera lens envelope",color(0.12,0.12,0.13));
 smBlock(context,id+"cameraHousing",-4.05,4.05,-34.9,-30.3,39.85,47.95);smLabel(context,smSolid(context,id+"cameraHousing"),"Camera housing envelope",color(0.12,0.12,0.13));
 smBlock(context,id+"board",-35,35,-10,80,13.3,14.9);smLabel(context,smSolid(context,id+"board"),"Perfboard placeholder",color(0.21,0.5,0.25));
 smBlock(context,id+"espMain",-14.5,14.5,-7.5,47.5,14.9,28.9);smLabel(context,smSolid(context,id+"espMain"),"ESP32 placeholder",color(0.12,0.3,0.2));
 for(var sx in [-65,65]){smBlock(context,id+("esc"~sx),sx-13,sx+13,-25,25,5,12);smLabel(context,smSolid(context,id+("esc"~sx)),"ESC envelope",color(0.25,0.25,0.27));}
}
});
