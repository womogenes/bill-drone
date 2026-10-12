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

annotation {"Feature Type Name":"Tube motor tilt", "Feature Type Description":"Split clamp for a rectangular carbon tube (default 10.5 mm wide, 10 mm high); MG995 servo and retained X bracket. Horn fit remains a measured parameter. No load cell. Rigid servo mounting uses bare ears without rubber. M3 tube bolts x20, pivot-foot bolts x10, journal retainer bolt x20. Horn: four 14 mm cardinal holes, enlarge to 3.2 mm, M3 bolts/nuts. Horn screws M3 x 10 for the measured 2.3 mm horn. Attach horn centre screw with a <=4 mm shaft through empty 4.5 mm journal bore BEFORE loading the journal nut, retainer or pivot support. Adjust sliding support without axial preload. Limit commanded tilt to +/-45 degrees. Reference bodies are not for printing."}
export const tubeTilt=defineFeature(function(context is Context,id is Id,definition is map)
 precondition {
  annotation {"Name":"Tube width - horizontal"} isLength(definition.tube,{(millimeter):[9,10.5,12]} as LengthBoundSpec);
  annotation {"Name":"Tube height - vertical"} isLength(definition.tubeHeight,{(millimeter):[9,10,12]} as LengthBoundSpec);
  annotation {"Name":"Tube clearance per side"} isLength(definition.fit,{(millimeter):[0.1,0.2,0.4]} as LengthBoundSpec);
  annotation {"Name":"Horn opposite centre-hole spacing"} isLength(definition.hornPitch,{(millimeter):[8,14,18]} as LengthBoundSpec);
  annotation {"Name":"Horn disc thickness - verify"} isLength(definition.hornT,{(millimeter):[1,2,4]} as LengthBoundSpec);
  annotation {"Name":"Servo ear to horn face"} isLength(definition.servoOffset,{(millimeter):[14,16.5,27]} as LengthBoundSpec);
  annotation {"Name":"Motor bracket opposite spacing"} isLength(definition.motorPitch,{(millimeter):[32,34,36]} as LengthBoundSpec);
  annotation {"Name":"Journal diametral clearance"} isLength(definition.journalFit,{(millimeter):[0.2,0.4,0.6]} as LengthBoundSpec);
  annotation {"Name":"Tilt angle"} isAngle(definition.tilt,{(degree):[-45,0,45]} as AngleBoundSpec);
  annotation {"Name":"Show reference hardware"} definition.refs is boolean;
  annotation {"Name":"Print layout"} definition.printLayout is boolean;
 }
 {
 const tube=definition.tube/millimeter;const tubeHeight=definition.tubeHeight/millimeter;
 const a=tubeHeight/2+definition.fit/millimeter;const b=tube/2+definition.fit/millimeter;const top=5;
 const mx=-48;const dx=definition.servoOffset/millimeter-23;const hp=definition.hornPitch/millimeter/2;
 const blue=color(0.16,0.48,0.67);const orange=color(0.92,0.51,0.12);const gray=color(0.3,0.32,0.34);
 // Compact ladder frame: tube clamp pads connected by two deep longitudinal rails.
 smBlock(context,id+"base",-18,18,-16,16,0,5);
 smBlock(context,id+"rearPad",-54,-40,-16,16,0,5);
 smPoly(context,id+"frontPad",[[16,-20],[20,-24],[36,-24],[40,-20],[40,20],[36,24],[20,24],[16,20]],vector(0,0,0),vector(0,0,1),vector(1,0,0),5);
 opTransform(context,id+"frontPadFit",{"bodies":smSolid(context,id+"frontPad"),"transform":transform(vector(dx,0,0)*millimeter)});
 for(var y in [-16,10])smBlock(context,id+("beam"~y),-54,min(14,14+dx),y,y+6,0,8);
 // Upright servo: continuous side rails join both mounting bars.
 smBlock(context,id+"railA",mx,mx+6,-16,-10.5,4.8,72);
 smBlock(context,id+"railB",mx,mx+6,10.5,16,4.8,72);
 smBlock(context,id+"servoLower",mx,mx+6,-16,16,4.8,21.5);
 smBlock(context,id+"servoUpper",mx,mx+6,-16,16,63.2,72);
 // Triangular braces carry servo torque into the longitudinal beams.
 for(var y in [-16,11])smPoly(context,id+("rib"~y),[[mx+5,7.8],[mx+17,7.8],[mx+5,48]],vector(0,y+5,0),vector(0,-1,0),vector(1,0,0),5);
 smJoin(context,id+"frameJoin",qUnion([smSolid(context,id+"base"),smSolid(context,id+"rearPad"),smSolid(context,id+"frontPad"),smSolid(context,id+"beam-16"),smSolid(context,id+"beam10"),smSolid(context,id+"railA"),smSolid(context,id+"railB"),smSolid(context,id+"servoLower"),smSolid(context,id+"servoUpper"),smSolid(context,id+"rib-16"),smSolid(context,id+"rib11")]));
 const frame=smSolid(context,id+"base");
 // Clip the upright top corners and the rear base corners.
 var frameCorners=[];
 for(var p in [vector(mx+3,-16,72),vector(mx+3,16,72),vector(-54,-16,4),vector(-54,16,4)])
  frameCorners=append(frameCorners,qContainsPoint(qOwnedByBody(frame,EntityType.EDGE),p*millimeter));
 opChamfer(context,id+"frameCorners",{"entities":qUnion(frameCorners),"chamferType":ChamferType.EQUAL_OFFSETS,"width":3*millimeter});
 smPoly(context,id+"fork",[[-15,9.8],[15,9.8],[15,37],[9,44],[-9,44],[-15,37]],vector(26,0,0),vector(1,0,0),vector(0,1,0),6);
 smPoly(context,id+"forkFoot",[[20,-21],[23,-24],[35,-24],[38,-21],[38,21],[35,24],[23,24],[20,21]],vector(0,0,5),vector(0,0,1),vector(1,0,0),5);
 for(var y in [-14,10])smPoly(context,id+("forkBrace"~y),[[31.8,9.8],[38,9.8],[31.8,26]],vector(0,y+4,0),vector(0,-1,0),vector(1,0,0),4);
 smJoin(context,id+"forkJoin",qUnion([smSolid(context,id+"fork"),smSolid(context,id+"forkFoot"),smSolid(context,id+"forkBrace-14"),smSolid(context,id+"forkBrace10")]));
 const fork=smSolid(context,id+"fork");
 opFillet(context,id+"forkRoot",{"entities":qContainsPoint(qOwnedByBody(fork,EntityType.EDGE),vector(26,0,10)*millimeter),"radius":3*millimeter});
 smHole(context,id+"journalBore",fork,vector(25,0,32),vector(1,0,0),10+definition.journalFit/millimeter,8);
 for(var y in [-18.5,18.5]) {
  const sid=id+("forkMount"~y);
  smBlock(context,sid+"web",25,33,y-1.75,y+1.75,4,11);
  smCyl(context,sid+"a",vector(25,y,4),vector(0,0,1),1.75,7);
  smCyl(context,sid+"b",vector(33,y,4),vector(0,0,1),1.75,7);
  smCut(context,sid+"slot",fork,qUnion([smSolid(context,sid+"web"),smSolid(context,sid+"a"),smSolid(context,sid+"b")]));
  smHole(context,sid+"hole",frame,vector(29+dx,y,-1),vector(0,0,1),3.5,7);
  smHex(context,sid+"nut",frame,vector(29+dx,y,-0.1),vector(0,0,1),vector(1,0,0),6.2,3.2);
 }
 smLabel(context,fork,"Pivot support",blue);
 // Upright MG995 bare-ear mounting pattern: 49 mm vertical, 10 mm across.
 for(var y in [-5,5])for(var z in [17.5,66.5]) {
  const sid=id+("servoSlot"~y~"z"~z);
  smBlock(context,sid+"web",mx-1,mx+7,y-1.75,y+1.75,z-0.5,z+0.5);
  smCyl(context,sid+"a",vector(mx-1,y,z-0.5),vector(1,0,0),1.75,8);
  smCyl(context,sid+"b",vector(mx-1,y,z+0.5),vector(1,0,0),1.75,8);
  smCut(context,sid+"cut",frame,qUnion([smSolid(context,sid+"web"),smSolid(context,sid+"a"),smSolid(context,sid+"b")]));
 }
 // Lower saddle with a remaining tightening gap: no set screw touches the carbon tube.
 smPoly(context,id+"saddle",[[-18,-11],[-13,-16],[-6,-16],[-3,-9],[3,-9],[6,-16],[13,-16],[18,-11],[18,11],[13,16],[6,16],[3,9],[-3,9],[-6,16],[-13,16],[-18,11]],vector(0,0,-2*a-4),vector(0,0,1),vector(1,0,0),2*a+3.4);
 smBlock(context,id+"saddleCut",-19,19,-b,b,-2*a,1);smCut(context,id+"saddleChannel",smSolid(context,id+"saddle"),smSolid(context,id+"saddleCut"));
 const saddle=smSolid(context,id+"saddle");
 var waistEdges=[];
 for(var x in [-3,3])for(var y in [-9,9])waistEdges=append(waistEdges,qContainsPoint(qOwnedByBody(saddle,EntityType.EDGE),vector(x,y,-2*a-2)*millimeter));
 opFillet(context,id+"saddleWaist",{"entities":qUnion(waistEdges),"radius":1.2*millimeter});
 for(var x in [-10,10])for(var y in [-11,11]) {
  const sid=id+("clamp"~x~"y"~y);
  smHole(context,sid+"f",frame,vector(x,y,-1),vector(0,0,1),3.5,11);
  smHex(context,sid+"nut",frame,vector(x,y,top-2.6),vector(0,0,1),vector(1,0,0),6.2,7);
  smHole(context,sid+"s",saddle,vector(x,y,-2*a-5),vector(0,0,1),3.5,2*a+6);
  smHole(context,sid+"head",saddle,vector(x,y,-2*a-4.1),vector(0,0,1),6.2,3.1);
 }
 smLabel(context,frame,"Tube frame",blue);smLabel(context,saddle,"Tube saddle",blue);
 // Motor cradle, original metal X bracket retained. Pivot lies below its plate.
 smPoly(context,id+"deck",[[-25,-15],[-21,-19],[21,-19],[25,-15],[25,15],[21,19],[-21,19],[-25,15]],vector(0,0,43),vector(0,0,1),vector(1,0,0),6);
 for(var x in [-25,19]) {
  smBlock(context,id+("leg"~x),x,x+6,-12,12,32,44);
  smCyl(context,id+("round"~x),vector(x,0,32),vector(1,0,0),12,6);
 }
 smCyl(context,id+"journal",vector(25,0,32),vector(1,0,0),5,7.8);
 smJoin(context,id+"cradleJoin",qUnion([smSolid(context,id+"deck"),smSolid(context,id+"leg-25"),smSolid(context,id+"leg19"),smSolid(context,id+"round-25"),smSolid(context,id+"round19"),smSolid(context,id+"journal")]));
 const cradle=smSolid(context,id+"deck");
 opFillet(context,id+"journalRoot",{"entities":qContainsPoint(qOwnedByBody(cradle,EntityType.EDGE),vector(25,5,32)*millimeter),"radius":0.7*millimeter});
 opFillet(context,id+"cradleRoots",{"entities":qUnion([qContainsPoint(qOwnedByBody(cradle,EntityType.EDGE),vector(-19,6,43)*millimeter),qContainsPoint(qOwnedByBody(cradle,EntityType.EDGE),vector(19,6,43)*millimeter)]),"radius":2*millimeter});
 // Tapered opening: keep the 26 mm motor-head clearance at the top while
 // restoring material beside the underside nut pockets; prints at a 45 degree slope.
 var throatA=newSketchOnPlane(context,id+"throatA",{"sketchPlane":plane(vector(0,0,43)*millimeter,vector(0,0,1))});
 skCircle(throatA,"circle",{"center":vector(0,0)*millimeter,"radius":10.5*millimeter});skSolve(throatA);
 var throatB=newSketchOnPlane(context,id+"throatB",{"sketchPlane":plane(vector(0,0,45.5)*millimeter,vector(0,0,1))});
 skCircle(throatB,"circle",{"center":vector(0,0)*millimeter,"radius":13*millimeter});skSolve(throatB);
 opLoft(context,id+"throatLoft",{"profileSubqueries":[qSketchRegion(id+"throatA"),qSketchRegion(id+"throatB")],"bodyType":ToolBodyType.SOLID});
 smCyl(context,id+"throatBottom",vector(0,0,42),vector(0,0,1),10.5,1);
 smCyl(context,id+"throatTop",vector(0,0,45.5),vector(0,0,1),13,4.5);
 smCut(context,id+"headRelief",cradle,qUnion([smSolid(context,id+"throatLoft"),smSolid(context,id+"throatBottom"),smSolid(context,id+"throatTop")]));
 opDeleteBodies(context,id+"throatClean",{"entities":qUnion([qCreatedBy(id+"throatA",EntityType.BODY),qCreatedBy(id+"throatB",EntityType.BODY)])});
 for(var i=0;i<4;i+=1) {
  const c=vector(cos((45+i*90)*degree),sin((45+i*90)*degree))*definition.motorPitch/millimeter/2;
  smHole(context,id+("motorHole"~i),cradle,vector(c[0],c[1],42),vector(0,0,1),3.5,8);
  smHex(context,id+("motorNut"~i),cradle,vector(c[0],c[1],42.9),vector(0,0,1),vector(1,0,0),6.2,3.2);
 }
 smHole(context,id+"hornCentre",cradle,vector(-26,0,32),vector(1,0,0),6.5,8);
 smHole(context,id+"hornHubRelief",cradle,vector(-25.1,0,32),vector(1,0,0),9.5,2.1);
 // Horn attachment through holes. Open the four measured horn holes to 3.2 mm; M3 screws and nuts.
 // Use M3 x 10 with nuts behind the 2.3 mm horn; longer bolts approach the servo case. Fit the centre screw with a <=4 mm shaft through the empty journal BEFORE inserting its nut or retainer.
 for(var i=0;i<4;i+=1) {
  smHole(context,id+("hornHole"~i),cradle,vector(-26,hp*cos(i*90*degree),32+hp*sin(i*90*degree)),vector(1,0,0),3.5,7);
  smHole(context,id+("hornHead"~i),cradle,vector(-20.1,hp*cos(i*90*degree),32+hp*sin(i*90*degree)),vector(1,0,0),6.2,4);
 }
 smHole(context,id+"axleBolt",cradle,vector(18,0,32),vector(1,0,0),4.5,16);
 smHex(context,id+"axleNut",cradle,vector(18.9,0,32),vector(1,0,0),vector(0,1,0),6.2,3.2);
 smLabel(context,cradle,"Motor cradle",orange);
 smCyl(context,id+"retainer",vector(32.8,0,32),vector(1,0,0),7,3);
 const retainer=smSolid(context,id+"retainer");smHole(context,id+"retainerHole",retainer,vector(32,0,32),vector(1,0,0),3.5,5);smLabel(context,retainer,"Pivot retainer",orange);
 if(dx!=0)opTransform(context,id+"axialFit",{"bodies":qUnion([cradle,retainer,fork]),"transform":transform(vector(dx,0,0)*millimeter)});
 if(!definition.printLayout && definition.tilt!=0*degree)opTransform(context,id+"tilt",{"bodies":qUnion([cradle,retainer]),"transform":rotationAround(line(vector(0,0,32)*millimeter,vector(1,0,0)),definition.tilt)});
 if(definition.printLayout) {
  opTransform(context,id+"laySaddle",{"bodies":saddle,"transform":transform(vector(0,-65,2*a+4)*millimeter)});
  opTransform(context,id+"layFork",{"bodies":fork,"transform":transform(vector(0,80,-5)*millimeter)});
  opTransform(context,id+"layCradle",{"bodies":cradle,"transform":transform(vector(75,0,49)*millimeter)*rotationAround(line(vector(0,0,0)*millimeter,vector(1,0,0)),180*degree)});
  opTransform(context,id+"layRetainer",{"bodies":retainer,"transform":transform(vector(120,60,-32.8-dx)*millimeter)*rotationAround(line(vector(0,0,0)*millimeter,vector(0,1,0)),-90*degree)});
 }
 if(definition.refs && !definition.printLayout) {
  smBlock(context,id+"tubeRef",mx-38,40,-tube/2,tube/2,-a-tubeHeight/2,-a+tubeHeight/2);smLabel(context,smSolid(context,id+"tubeRef"),"Reference - carbon tube",color(0.12,0.13,0.14));
  smBlock(context,id+"servoRef",mx-28,mx+10,-10,30.7,22,42);
  for(var y in [-17,30.7])smBlock(context,id+("earRef"~y),mx-2.5,mx,y,y+7.3,22,42);
  smJoin(context,id+"servoRefJoin",qUnion([smSolid(context,id+"servoRef"),smSolid(context,id+"earRef-17"),smSolid(context,id+"earRef30.7")]));smLabel(context,smSolid(context,id+"servoRef"),"Reference - MG995 envelope",gray);
  opTransform(context,id+"servoUpright",{"bodies":smSolid(context,id+"servoRef"),"transform":rotationAround(line(vector(0,0,32)*millimeter,vector(1,0,0)),90*degree)});
  smCyl(context,id+"hornRef",vector(-25+dx-definition.hornT/millimeter,0,32),vector(1,0,0),10.5,definition.hornT/millimeter);smLabel(context,smSolid(context,id+"hornRef"),"Reference - supplied horn",color(0.2,0.2,0.2));
  smBlock(context,id+"bracketA",-21,21,-4,4,49,50.5);smBlock(context,id+"bracketB",-4,4,-21,21,49,50.5);
  smJoin(context,id+"bracketJoin",qUnion([smSolid(context,id+"bracketA"),smSolid(context,id+"bracketB")]));
  const bracket=smSolid(context,id+"bracketA");
  opTransform(context,id+"bracketClock",{"bodies":bracket,"transform":rotationAround(line(vector(0,0,0)*millimeter,vector(0,0,1)),45*degree)});
  for(var i=0;i<4;i+=1)smHole(context,id+("bracketHole"~i),bracket,vector(cos((45+i*90)*degree)*definition.motorPitch/millimeter/2,sin((45+i*90)*degree)*definition.motorPitch/millimeter/2,47),vector(0,0,1),3.5,4);
  if(dx!=0)opTransform(context,id+"bracketFit",{"bodies":bracket,"transform":transform(vector(dx,0,0)*millimeter)});
  smLabel(context,bracket,"Reference - metal bracket",gray);
  smCyl(context,id+"motorRef",vector(dx,0,50.5),vector(0,0,1),14,30);smCyl(context,id+"shaftRef",vector(dx,0,80.5),vector(0,0,1),3,12);smJoin(context,id+"motorRefJoin",qUnion([smSolid(context,id+"motorRef"),smSolid(context,id+"shaftRef")]));smLabel(context,smSolid(context,id+"motorRef"),"Reference - motor envelope",color(0.65,0.68,0.7));
  if(definition.tilt!=0*degree)opTransform(context,id+"refTilt",{"bodies":qUnion([smSolid(context,id+"motorRef"),smSolid(context,id+"hornRef"),bracket]),"transform":rotationAround(line(vector(0,0,32)*millimeter,vector(1,0,0)),definition.tilt)});
 }
 });



