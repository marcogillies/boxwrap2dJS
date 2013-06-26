/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Mick Grierson, Matthew Yee-King, Marco Gillies
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


var   Vec2 = Box2D.Common.Math.b2Vec2
         	,	BodyDef = Box2D.Dynamics.b2BodyDef
         	,	CircleDef = Box2D.Collision.Shapes.b2CircleDef
         	,	Body = Box2D.Dynamics.b2Body
         	,	FixtureDef = Box2D.Dynamics.b2FixtureDef
         	//,	Fixture = Box2D.Dynamics.b2Fixture
         	,	World = Box2D.Dynamics.b2World
         	//,	MassData = Box2D.Collision.Shapes.b2MassData
         	,	PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
         	,	CircleShape = Box2D.Collision.Shapes.b2CircleShape
         	,	DebugDraw = Box2D.Dynamics.b2DebugDraw
         	,	AABB = Box2D.Collision.b2AABB
         	,   DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint
         	,   DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef
         	,   RevoluteJoint = Box2D.Dynamics.Joints.b2RevoluteJoint
         	,   RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef
         	,   PrismaticJoint = Box2D.Dynamics.Joints.b2PrismaticJoint
         	,   PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef
         	,   PulleyJoint = Box2D.Dynamics.Joints.b2PulleyJoint
         	,   PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef
         	,   GearJoint = Box2D.Dynamics.Joints.b2GearJoint
         	,   GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef
            ;

//Body.prototype.getWorldCenter = function (){
//        v = new Vec2(0, 0);
//        v.set(this.GetPosition());
//        return v;
//};
//Body.prototype.setPosition = function(v){
//        angle = this.GetAngle();
//        this.SetPositionAndAngle(v, angle);
//};
Body.prototype.getMass        = Body.prototype.GetMass;
Body.prototype.getInertia     = Body.prototype.GetInertia;
Body.prototype.getMassData    = Body.prototype.GetMassData;
Body.prototype.setMassData    = Body.prototype.SetMassData;
Body.prototype.resetMassData  = Body.prototype.ResetMassData;
Body.prototype.applyImpulse   = Body.prototype.ApplyImpulse;
Body.prototype.setTransform   = Body.prototype.SetPositionAndAngle;
Body.prototype.getTransform   = Body.prototype.GetTransform;
Body.prototype.getPosition    = Body.prototype.GetPosition;
Body.prototype.setPosition    = Body.prototype.SetPosition;
Body.prototype.getAngle       = Body.prototype.GetAngle;
Body.prototype.setAngle       = Body.prototype.SetAngle;
Body.prototype.getWorldCenter = Body.prototype.GetWorldCenter;
Body.prototype.getLocalCenter = Body.prototype.GetLocalCenter;

Body.prototype.setLinearVelocity  = Body.prototype.SetLinearVelocity;
Body.prototype.getLinearVelocity  = Body.prototype.GetLinearVelocity;
Body.prototype.setAngularVelocity = Body.prototype.SetAngularVelocity;
Body.prototype.getAngularVelocity = Body.prototype.GetAngularVelocity;

Body.prototype.applyForce = Body.prototype.ApplyForce;
Body.prototype.applyForceToCenter = function(force){
	this.applyForce(force, this.getWorldCenter());
};
Body.prototype.applyTorque = Body.prototype.ApplyTorque;
Body.prototype.applyLinearImpulse = Body.prototype.ApplyImpulse;

Body.prototype.getWorldPoint = Body.prototype.GetWorldPoint;
Body.prototype.getWorldPointToOut = function(localPoint, out){
	out = this.getWorldPoint(localPoint);
};
Body.prototype.getWorldVector = Body.prototype.GetWorldVector;
Body.prototype.getWorldVectorToOut = function(localPoint, out){
	out = this.getWorldVector(localPoint);
};
Body.prototype.getWorldVectorToOutUnsafe = Body.prototype.getWorldVectorToOut;

Body.prototype.getLocalPoint = Body.prototype.GetLocalPoint;
Body.prototype.getLocalPointToOut = function(worldPoint, out){
	out = this.getLocalPoint(worldPoint);
};
Body.prototype.getLocalVector = Body.prototype.GetLocalVector;
Body.prototype.getLocalVectorToOut = function(worldPoint, out){
	out = this.getLocalVector(worldPoint);
};
Body.prototype.getLocalVectorToOutUnsafe = Body.prototype.getLocalVectorToOut;


Body.prototype.getLinearVelocityFromWorldPoint  = Body.prototype.GetLinearVelocity;
Body.prototype.getLinearVelocityFromWorldPointToOut = function(worldPoint, out){
	out = this.getLinearVelocityFromWorldPoint(worldPoint);
};
Body.prototype.getLinearVelocityFromLocalPoint  = Body.prototype.GetLinearVelocity;
Body.prototype.getLinearVelocityFromLocalPointToOut = function(localPoint, out){
	out = this.getLinearVelocityFromLocalPoint(localPoint);
};

Body.prototype.getLinearDamping = Body.prototype.GetLinearDamping;
Body.prototype.setLinearDamping = Body.prototype.SetLinearDamping;
Body.prototype.getAngularDamping = Body.prototype.GetAngularDamping;
Body.prototype.setAngularDamping = Body.prototype.SetAngularDamping;

Body.prototype.getType = Body.prototype.GetType;
Body.prototype.setType = Body.prototype.SetType;

Body.prototype.isBullet = Body.prototype.IsBullet;
Body.prototype.setBullet = Body.prototype.SetBullet;

Body.prototype.isSleepingAllowed = Body.prototype.IsSleepingAllowed;
Body.prototype.setSleepingAllowed = Body.prototype.SetSleepingAllowed;
Body.prototype.isAwake = Body.prototype.IsAwake;
Body.prototype.setAwake = Body.prototype.SetAwake;
Body.prototype.isActive = Body.prototype.IsActive;
Body.prototype.setActive = Body.prototype.SetActive;

Body.prototype.isFixedRotation = Body.prototype.IsFixedRotation;
Body.prototype.setFixedRotation = Body.prototype.SetFixedRotation;

Vec2.prototype.set = function(x,y){
    if(y == undefined){
      this.SetV(x);
    }else{
      this.Set(x,y);
    }
  };
Vec2.prototype.setZero = Vec2.prototype.SetZero;
Vec2.prototype.mul = function (a){
    return new Vec2(this.x * a, this.y * a);
  };
Vec2.prototype.mulLocal = function (a){
    this.Multiply(a);
    return this;
  };
Vec2.prototype.add = function (v){
    //retVal = new Vec2(0, 0);
    //retVal.set(this);
    retVal = this.clone();
    retVal.Add(v);
    return retVal;
  };
Vec2.prototype.addLocal = function (x,y){
    if(y == undefined){
      this.Add(x);
    }else{
      this.Add(new Vec2(x,y));
    }
    return this;
  };
Vec2.prototype.sub = function (v){
    //retVal = new Vec2(0, 0);
    //retVal.set(this);
    retVal = this.clone();
    retVal.Subtract(v);
    return retVal;
  };
  
Vec2.prototype.subLocal = function (x,y){
    if(y == undefined){
      this.Subtract(x);
    }else{
      this.Subtract(new Vec2(x,y));
    }
    return this;
  };

Vec2.prototype.negate = Vec2.prototype.GetNegative;
Vec2.prototype.negateLocal = Vec2.prototype.NegativeSelf;
Vec2.prototype.normalize = Vec2.prototype.Normalize;
Vec2.prototype.length = Vec2.prototype.Length;
Vec2.prototype.lengthSquared = Vec2.prototype.LengthSquared;
Vec2.prototype.abs = function (){
    retVal = new Vec2(this.x, this.y);
    return retVal.Abs();
  };
Vec2.prototype.absLocal = Vec2.prototype.Abs;
Vec2.prototype.isValid = Vec2.prototype.IsValid;
Vec2.prototype.clone = function (){
    return new Vec2(this.x, this.y);
  };


DistanceJoint.prototype.getType = DistanceJoint.prototype.GetType;
DistanceJoint.prototype.getBodyA = DistanceJoint.prototype.GetBodyA;
DistanceJoint.prototype.getBodyB = DistanceJoint.prototype.GetBodyB;
DistanceJoint.prototype.getAnchorA = DistanceJoint.prototype.GetAnchorA;
DistanceJoint.prototype.getAnchorB = DistanceJoint.prototype.GetAnchorB;
DistanceJoint.prototype.getReactionForce = DistanceJoint.prototype.GetReactionForce;
DistanceJoint.prototype.getReactionTorque = DistanceJoint.prototype.GetReactionTorque;
DistanceJoint.prototype.getNext = DistanceJoint.prototype.GetNext;
DistanceJoint.prototype.getUserData = DistanceJoint.prototype.GetUserData;
DistanceJoint.prototype.setUserData = DistanceJoint.prototype.SetUserData;
DistanceJoint.prototype.isActive = DistanceJoint.prototype.IsActive;

DistanceJoint.prototype.getLength = DistanceJoint.prototype.GetLength;
DistanceJoint.prototype.setLength = DistanceJoint.prototype.SetLength;
DistanceJoint.prototype.getFrequency = DistanceJoint.prototype.GetFrequency;
DistanceJoint.prototype.setFrequency = DistanceJoint.prototype.SetFrequency;
DistanceJoint.prototype.getDampingRatio = DistanceJoint.prototype.GetDampingRatio;
DistanceJoint.prototype.setDampingRatio = DistanceJoint.prototype.SetDampingRatio;


RevoluteJoint.prototype.getType = RevoluteJoint.prototype.GetType;
RevoluteJoint.prototype.getBodyA = RevoluteJoint.prototype.GetBodyA;
RevoluteJoint.prototype.getBodyB = RevoluteJoint.prototype.GetBodyB;
RevoluteJoint.prototype.getAnchorA = RevoluteJoint.prototype.GetAnchorA;
RevoluteJoint.prototype.getAnchorB = RevoluteJoint.prototype.GetAnchorB;
RevoluteJoint.prototype.getReactionForce = RevoluteJoint.prototype.GetReactionForce;
RevoluteJoint.prototype.getReactionTorque = RevoluteJoint.prototype.GetReactionTorque;
RevoluteJoint.prototype.getNext = RevoluteJoint.prototype.GetNext;
RevoluteJoint.prototype.getUserData = RevoluteJoint.prototype.GetUserData;
RevoluteJoint.prototype.setUserData = RevoluteJoint.prototype.SetUserData;
RevoluteJoint.prototype.isActive = RevoluteJoint.prototype.IsActive;

RevoluteJoint.prototype.getJointAngle = RevoluteJoint.prototype.GetJointAngle;
RevoluteJoint.prototype.getJointSpeed = RevoluteJoint.prototype.GetJointSpeed;
RevoluteJoint.prototype.isLimitEnabled = RevoluteJoint.prototype.IsLimitEnabled;
RevoluteJoint.prototype.enableLimit = RevoluteJoint.prototype.EnableLimit;
RevoluteJoint.prototype.getLowerLimit = RevoluteJoint.prototype.GetLowerLimit;
RevoluteJoint.prototype.getUpperLimit = RevoluteJoint.prototype.GetUpperLimit;
RevoluteJoint.prototype.setLimits = RevoluteJoint.prototype.SetLimits;
RevoluteJoint.prototype.isMotorEnabled = RevoluteJoint.prototype.IsMotorEnabled;
RevoluteJoint.prototype.enableMotor = RevoluteJoint.prototype.EnableMotor;
RevoluteJoint.prototype.getMotorTorque = RevoluteJoint.prototype.GetMotorTorque;
RevoluteJoint.prototype.getMotorSpeed = RevoluteJoint.prototype.GetMotorSpeed;
RevoluteJoint.prototype.setMotorSpeed = RevoluteJoint.prototype.SetMotorSpeed;
RevoluteJoint.prototype.setMaxMotorTorque = RevoluteJoint.prototype.setMaxMotorTorque;
RevoluteJoint.prototype.getMaxMotorTorque = RevoluteJoint.prototype.getMaxMotorTorque;


PrismaticJoint.prototype.getType = PrismaticJoint.prototype.GetType;
PrismaticJoint.prototype.getBodyA = PrismaticJoint.prototype.GetBodyA;
PrismaticJoint.prototype.getBodyB = PrismaticJoint.prototype.GetBodyB;
PrismaticJoint.prototype.getAnchorA = PrismaticJoint.prototype.GetAnchorA;
PrismaticJoint.prototype.getAnchorB = PrismaticJoint.prototype.GetAnchorB;
PrismaticJoint.prototype.getReactionForce = PrismaticJoint.prototype.GetReactionForce;
PrismaticJoint.prototype.getReactionTorque = PrismaticJoint.prototype.GetReactionTorque;
PrismaticJoint.prototype.getNext = PrismaticJoint.prototype.GetNext;
PrismaticJoint.prototype.getUserData = PrismaticJoint.prototype.GetUserData;
PrismaticJoint.prototype.setUserData = PrismaticJoint.prototype.SetUserData;
PrismaticJoint.prototype.isActive = PrismaticJoint.prototype.IsActive;

PrismaticJoint.prototype.getJointTranslation = PrismaticJoint.prototype.GetJointTranslation;
PrismaticJoint.prototype.getJointSpeed = PrismaticJoint.prototype.GetJointSpeed;
PrismaticJoint.prototype.isLimitEnabled = PrismaticJoint.prototype.IsLimitEnabled;
PrismaticJoint.prototype.enableLimit = PrismaticJoint.prototype.EnableLimit;
PrismaticJoint.prototype.getLowerLimit = PrismaticJoint.prototype.GetLowerLimit;
PrismaticJoint.prototype.getUpperLimit = PrismaticJoint.prototype.GetUpperLimit;
PrismaticJoint.prototype.setLimits = PrismaticJoint.prototype.SetLimits;
PrismaticJoint.prototype.isMotorEnabled = PrismaticJoint.prototype.IsMotorEnabled;
PrismaticJoint.prototype.enableMotor = PrismaticJoint.prototype.EnableMotor;
PrismaticJoint.prototype.getMotorForce = PrismaticJoint.prototype.GetMotorForce;
PrismaticJoint.prototype.getMotorSpeed = PrismaticJoint.prototype.GetMotorSpeed;
PrismaticJoint.prototype.setMotorSpeed = PrismaticJoint.prototype.SetMotorSpeed;
PrismaticJoint.prototype.setMaxMotorForce = PrismaticJoint.prototype.SetMaxMotorForce;
PrismaticJoint.prototype.getMaxMotorForce = PrismaticJoint.prototype.GetMaxMotorForce;


PulleyJoint.prototype.getType = PulleyJoint.prototype.GetType;
PulleyJoint.prototype.getBodyA = PulleyJoint.prototype.GetBodyA;
PulleyJoint.prototype.getBodyB = PulleyJoint.prototype.GetBodyB;
PulleyJoint.prototype.getAnchorA = PulleyJoint.prototype.GetAnchorA;
PulleyJoint.prototype.getAnchorB = PulleyJoint.prototype.GetAnchorB;
PulleyJoint.prototype.getReactionForce = PulleyJoint.prototype.GetReactionForce;
PulleyJoint.prototype.getReactionTorque = PulleyJoint.prototype.GetReactionTorque;
PulleyJoint.prototype.getNext = PulleyJoint.prototype.GetNext;
PulleyJoint.prototype.getUserData = PulleyJoint.prototype.GetUserData;
PulleyJoint.prototype.setUserData = PulleyJoint.prototype.SetUserData;
PulleyJoint.prototype.isActive = PulleyJoint.prototype.IsActive;

PulleyJoint.prototype.getLengthA = PulleyJoint.prototype.GetLengthA;
PulleyJoint.prototype.getLengthB = PulleyJoint.prototype.GetLengthB;
PulleyJoint.prototype.getCurrentLengthA = PulleyJoint.prototype.GetCurrentLengthA;
PulleyJoint.prototype.getCurrentLengthB = PulleyJoint.prototype.GetCurrentLengthB;
PulleyJoint.prototype.getLengthA = PulleyJoint.prototype.GetLengthA;
PulleyJoint.prototype.getLengthB = PulleyJoint.prototype.GetLengthB;
PulleyJoint.prototype.getLocalAnchorA = PulleyJoint.prototype.GetLocalAnchorA;
PulleyJoint.prototype.getLocalAnchorB = PulleyJoint.prototype.GetLocalAnchorB;
PulleyJoint.prototype.getGroundAnchorA = PulleyJoint.prototype.GetGroundAnchorA;
PulleyJoint.prototype.getGroundAnchorB = PulleyJoint.prototype.GetGroundAnchorB;
PulleyJoint.prototype.getLength1 = PulleyJoint.prototype.GetLength1;
PulleyJoint.prototype.getLength2 = PulleyJoint.prototype.GetLength2;
PulleyJoint.prototype.getRatio = PulleyJoint.prototype.GetRatio;

GearJoint.prototype.getType = GearJoint.prototype.GetType;
GearJoint.prototype.getBodyA = GearJoint.prototype.GetBodyA;
GearJoint.prototype.getBodyB = GearJoint.prototype.GetBodyB;
GearJoint.prototype.getAnchorA = GearJoint.prototype.GetAnchorA;
GearJoint.prototype.getAnchorB = GearJoint.prototype.GetAnchorB;
GearJoint.prototype.getReactionForce = GearJoint.prototype.GetReactionForce;
GearJoint.prototype.getReactionTorque = GearJoint.prototype.GetReactionTorque;
GearJoint.prototype.getNext = GearJoint.prototype.GetNext;
GearJoint.prototype.getUserData = GearJoint.prototype.GetUserData;
GearJoint.prototype.setUserData = GearJoint.prototype.SetUserData;
GearJoint.prototype.isActive = GearJoint.prototype.IsActive;

GearJoint.prototype.getRatio = GearJoint.prototype.GetRatio;
GearJoint.prototype.setRatio = GearJoint.prototype.SetRatio;

/*
Vec2 = function(x,y){
	v = new b2Vec2(x,y);
	v.mul = function (x,y){
		this.Multiply(x,y);
		return this;
	};
	v.add = function (x,y){
		retVal = new Vec2(0, 0);
		retVal.set(this);
		retVal.Add(x,y);
		return retVal;
	};
	v.sub = function (x,y){
		retVal = new Vec2(0, 0);
		retVal.set(this);
		retVal.Subtract(x,y);
		return retVal;
	};
	v.normalize = v.Normalize;
	v.set = function(x,y){
		if(y == undefined){
			v.SetV(x);
		}else{
			v.Set(x,y);
		}
	}
	return v;
};
*/

Physics = function (sketch, screenW,  screenH,
								   gravX, gravY,
								   screenAABBWidth, screenAABBHeight,
								   borderBoxWidth, borderBoxHeight,
								   pixelsPerMeter){
	
	if(gravX == undefined) gravX = 0.0;
	if(gravY == undefined) { 
            gravY = 10.0;
        }else{
            gravY = -gravY;
        }
	if(screenAABBWidth == undefined) screenAABBWidth = 2*screenW;
	if(screenAABBHeight == undefined) screenAABBHeight = 2*screenH;
	if(borderBoxWidth == undefined) borderBoxWidth = screenW;
	if(borderBoxHeight == undefined) borderBoxHeight = screenH;
	if(pixelsPerMeter == undefined) pixelsPerMeter = 100;
	
	
	var physics = {
		m_sketch : sketch,
		m_gravity : new Vec2(gravX, gravY),	
		m_density : 0.0,
		m_restitution : 0.9,
		m_friction : 0.9,
		m_bullet : false, 
		m_sensor : false, 
		m_pixelsPerMeter : pixelsPerMeter,
		m_screenHeight : screenH,
		createHollowBox : function(centerX, centerY, width, height, thickness) {
            //console.log("creating hollow box");
			var result = new Array();
                        console.log(this.createRect);
			result[0] = this.createRect(centerX - width*0.5 - thickness*0.5, centerY - height*0.5 - thickness*0.5,
								   centerX - width*0.5 + thickness*0.5, centerY + height*0.5 + thickness*0.5);
			result[1] = this.createRect(centerX + width*0.5 - thickness*0.5, centerY - height*0.5 - thickness*0.5,
								   centerX + width*0.5 + thickness*0.5, centerY + height*0.5 + thickness*0.5);
			result[2] = this.createRect(centerX - width*0.5 - thickness*0.5, centerY + height*0.5 - thickness*0.5,
								   centerX + width*0.5 + thickness*0.5, centerY + height*0.5 + thickness*0.5);
			result[3] = this.createRect(centerX - width*0.5 - thickness*0.5, centerY - height*0.5 - thickness*0.5,
								   centerX + width*0.5 + thickness*0.5, centerY - height*0.5 + thickness*0.5);
			//console.log("finished creating hollow box");
            return result;
		},
		
		setCustomRenderingMethod : function (obj, nm){
			physics.m_customRenderingMethod = nm;
		},
		
		unsetCustomRenderingMethod : function(){
			delete physics.m_customRenderingMethod;
		},
		
		
		createRect : function(x0, y0, x1, y1) {
                        
            //console.log("creating rect");
			var cxs = (x0 + x1) * 0.5;
			var cys = (y0 + y1) * 0.5;
			var wxs = Math.abs(x1-x0);
			var wys = Math.abs(y1-y0);
			
			var center = this.screenToWorld(cxs, cys);
		
			var halfWidthWorld = 0.5*this.screenToWorld(wxs);
			var halfHeightWorld = 0.5*this.screenToWorld(wys);
		
			 var fixDef = new FixtureDef();
			 this.setShapeDefProperties(fixDef);
			 
             //console.log("creating BodyDef");

			 var bodyDef = new BodyDef();
             //console.log("setting up bodydef");
			 if(this.m_density < 0.00001) {
			    bodyDef.type = Body.b2_staticBody;
			 } else {
			    bodyDef.type = Body.b2_dynamicBody;
			 }
			 bodyDef.position.x = center.x;
			 bodyDef.position.y = center.y;
                          
             //console.log("creating shapedef");
			 fixDef.shape = new PolygonShape;
			 fixDef.shape.SetAsBox(halfWidthWorld, halfHeightWorld);
                          
             //console.log("creating body");
			 var b  = this.m_world.CreateBody(bodyDef);
			 b.CreateFixture(fixDef);
		
			//this.enhanceBody(b);
			
			//console.log(b);
			//console.log(b.GetPosition());
		    //console.log("finished creating rect");
			return b;
		},
		
		/**
		 * Create a circle in screen coordinates
		 * @param x
		 * @param y
		 * @param r
		 * @return
		 */
		createCircle : function(x, y, r) {
			var center = this.screenToWorld(x,y);
			var rad    = this.screenToWorld(r);
			
			 var fixDef = new FixtureDef();
			 this.setShapeDefProperties(fixDef);
			 
			 var bodyDef = new BodyDef();
			 //console.log("density " + this.m_density)
			 if(this.m_density < 0.00001) {
			 	//console.log("static body");
			    bodyDef.type = Body.b2_staticBody;
			 } else {
			 	//console.log("dynamic body");
			    bodyDef.type = Body.b2_dynamicBody;
			 }
			 //console.log("radius " + rad);
			 bodyDef.position.x = center.x;
			 bodyDef.position.y = center.y;
			 fixDef.shape = new CircleShape(rad);
			 var b  = this.m_world.CreateBody(bodyDef);
			 b.CreateFixture(fixDef);
		
			
			//var cd = new CircleDef();
			//cd.radius = rad;
			//this.setShapeDefProperties(cd);
			
			//var bd = new BodyDef();
			//this.setBodyDefProperties(bd);
			//bd.AddShape(cd);
			//bd.position.Set(center.x, center.y);
			
			//var b = this.m_world.CreateBody(bd);
			//this.enhanceBody(b);
			
			return b;
		},
		
		/**
		 * Create a polygon based on vertices.
		 * <BR><BR>
		 * Polygons must be:
		 * <ul>
		 * <li>Ordered clockwise in screen coordinates (which
		 * becomes counterclockwise in world coordinates).
		 * <li>Non self-intersecting.
		 * <li>Convex
		 * </ul>
		 * Failure to adhere to any of these restrictions may cause
		 * simulation crashes or problems.  In particular, if your
		 * objects are showing up as static objects instead of dynamic
		 * ones, and are not colliding correctly, you have probably 
		 * not met the clockwise ordering requirement.
		 * <BR><BR>
		 * This can be called with any number of vertices passed as
		 * pairs of interleaved floats, for instance:
		 * <pre>
		 * createPolygon(x0,y0,x1,y1,x2,y2,x3,y3);</pre>
		 * or
		 * <pre>
		 * createPolygon(x0,y0,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5);</pre>
		 * or
		 * <pre>
		 * float[] xyInterleaved = {x0,y0,x1,y1,x2,y2,x3,y3,x4,y4};
		 * createPolygon(xyInterleaved);</pre>
		 * are all fine.
		 * @param vertices Any number of pairs of x,y floats, or an array of the same (screen coordinates)
		 * @return
		 */
		createPolygon : function() {
			var vertices;
			if (arguments.length == 1 && arguments[0].length){
				vertices = arguments[0];
			} else {
				vertices = arguments;
			}
			if (vertices.length % 2 != 0) 
				throw new IllegalArgumentException("Vertices must be given as pairs of x,y coordinates, " +
												   "but number of passed parameters was odd.");
			var nVertices = vertices.length / 2;
			var pd = new b2PolyDef();
			pd.vertexCount = nVertices;
			var average = new Vec2(0, 0);
			for (var i = 0; i < nVertices; i++) {
				var v = this.screenToWorld(vertices[2*i],vertices[2*i+1]);
				//console.log("creating vertex " + v.x + " " + v.y);
				pd.vertices[i].Set(v.x, v.y);
				average.x += v.x;
				average.y += v.y;
			}
			if(nVertices > 0){
				average.x /= nVertices;
				average.y /= nVertices;
			}
			for (var i = 0; i < nVertices; i++) {
				pd.vertices[i].x -= average.x;
				pd.vertices[i].y -= average.y;
			}
			this.setShapeDefProperties(pd);
			
			
			var bd = new BodyDef();
			this.setBodyDefProperties(bd);
			bd.position.Set(average.x, average.y);
			bd.AddShape(pd);
			
			var b = this.m_world.CreateBody(bd);
			//this.enhanceBody(b);
			
			
			return b;
		},
		
		
		setBodyDefProperties : function(bd) {
			bd.isBullet = this.m_bullet;
		},

		setShapeDefProperties : function(sd) {
			sd.density = this.m_density;
			sd.friction = this.m_friction;
			sd.restitution = this.m_restitution;
			sd.isSensor = this.m_sensor;
			//console.log(sd);
		},
		/*
		enhanceBody : function (b){
  			
                        b.getWorldCenter = function (){
				v = new Vec2(0, 0);
                                v.set(b.GetPosition());
                                return v;
			};
			b.setPosition = function(v){
				angle = this.GetAngle();
				this.SetPositionAndAngle(v, angle);
			};
			b.getMass = b.GetMass;
			b.applyImpulse = b.ApplyImpulse;
                        
		},
		*/
		/**
		 * Create a distance (stick) joint between two bodies
		 * that holds the specified points at a constant distance.
		 * <BR><BR>
		 * Once the distance joint is created, it may be turned into
		 * a "soft" distance joint by using DistanceJoint::setFrequencyHz(float)
		 * to set the frequency to a non-zero value, and using 
		 * DistanceJoint::setDampingRatio(float) to tune the damping constant.
		 * <BR><BR>
		 * Distance joints do not support joint limits or motors.
		 * @param a First body
		 * @param b Second body
		 * @param xa x component of anchor point on first body (screen coordinates)
		 * @param ya y component of anchor point on first body (screen coordinates)
		 * @param xb x component of anchor point on second body (screen coordinates)
		 * @param yb y component of anchor point on second body (screen coordinates)
		 * @return Newly created DistanceJoint
		 */
		createDistanceJoint : function(a, b, xa, ya, xb, yb) {
			var va = physics.screenToWorld(xa,ya);
			var vb = physics.screenToWorld(xb,yb);
			
			var jd = new DistanceJointDef();
			
			jd.Initialize(a, b, va, vb);
			
			return physics.m_world.CreateJoint(jd);
		},
		
		/**
		 * Create a revolute (pin) joint between the two bodies
		 * at the given position.
		 * <BR><BR>
		 * Joint limits and motors may be set once the joint is created.
		 * @param a First body
		 * @param b Second body
		 * @param x x coordinate of pin joint location (screen coordinates)
		 * @param y y coordinate of pin joint location (screen coordinates)
		 * @return Newly created RevoluteJoint
		 */
		createRevoluteJoint : function(a, b, x, y) {
			var v = physics.screenToWorld(x,y);
			var jd = new RevoluteJointDef();
			jd.Initialize(a, b, v);
			
			return physics.m_world.CreateJoint(jd);
		},
		
		/**
		 * Create a prismatic (piston) joint between two bodies
		 * that allows movement in the given direction.
		 * <BR><BR>
		 * dirX and dirY can be given in screen coordinates or
		 * world coordinates, scaling does not matter.
		 * <BR><BR>
		 * Joint limits and motors may be set once the joint is created.
		 * @param a First body
		 * @param b Second body
		 * @param dirX x component of allowed movement direction
		 * @param dirY y component of allowed movement direction
		 * @return Newly created PrismaticJoint
		 */
		createPrismaticJoint : function(a, b, dirX, dirY) {
			var dir = new Vec2(dirX, dirY);
			dir.Normalize();
			var pjd = new PrismaticJointDef();
			
			pjd.Initialize(a, b, (a.getWorldCenter().add(b.getWorldCenter())).mul(0.5), dir);
			return physics.m_world.CreateJoint(pjd);
			
		},
		
		/**
		 * Create a pulley joint between the 
		 * The pulley joint is connected to two bodies and two fixed ground points.
		 * The pulley supports a ratio such that:
		 * length1 + ratio * length2 = constant
		 * Yes, the force transmitted is scaled by the ratio.
		 * <BR><BR>
		 * The ground anchors are the points where the "rope" touches the pulley,
		 * and the anchors are the points on the bodies where the rope is attached.
		 * <BR><BR>
		 * Joint limits may be set after the joint is created.
		 * @param a First body
		 * @param b Second body
		 * @param groundAnchorAx x coordinate of (fixed) ground anchor for body a, in screen coordinates
		 * @param groundAnchorAy y coordinate of (fixed) ground anchor for body a, in screen coordinates
		 * @param groundAnchorBx x coordinate of (fixed) ground anchor for body b, in screen coordinates
		 * @param groundAnchorBy y coordinate of (fixed) ground anchor for body b, in screen coordinates
		 * @param anchorAx x coordinate of body anchor for body a, in screen coordinates
		 * @param anchorAy y coordinate of body anchor for body a, in screen coordinates
		 * @param anchorBx x coordinate of body anchor for body b, in screen coordinates
		 * @param anchorBy y coordinate of body anchor for body b, in screen coordinates
		 * @param ratio "Block and tackle" ratio
		 * @return Newly created PulleyJoint
		 */
		createPulleyJoint : function(a, b, groundAnchorAx, groundAnchorAy,
															 groundAnchorBx, groundAnchorBy,
															 anchorAx, anchorAy,
															 anchorBx, anchorBy,
															 ratio) {
			var gA = physics.screenToWorld(groundAnchorAx,groundAnchorAy);
			var gB = physics.screenToWorld(groundAnchorBx,groundAnchorBy);
			var aA = physics.screenToWorld(anchorAx,anchorAy);
			var aB = physics.screenToWorld(anchorBx,anchorBy);
			var jd = new PulleyJointDef();
			
			a.SetAwake(true);
			b.SetAwake(true);
			jd.Initialize(a, b, gA, gB, aA, aB, ratio)
			
			return physics.m_world.CreateJoint(jd);
		},
		
		/**
		 * Create a gear joint, which binds together two existing
		 * revolute or prismatic joints (any combination will work).
		 * The provided joints must attach a dynamic body to a static body.
		 * <BR><BR>
		 * A gear joint is used to connect two joints together. Either joint
		 * can be a revolute or prismatic joint. You specify a gear ratio
		 * to bind the motions together:
		 * coordinate1 + ratio * coordinate2 = constant
		 * The ratio can be negative or positive. If one joint is a revolute joint
		 * and the other joint is a prismatic joint, then the ratio will have units
		 * of length or units of 1/length.
		 * <BR><em>Warning</em>: The revolute and prismatic joints must be attached to
		 * fixed bodies (which must be body1 on those joints).
		 * @param pj1 First joint (revolute or prismatic)
		 * @param pj2 Second joint (revolute or prismatic)
		 * @param ratio Gear ratio
		 * @return Newly created GearJoint
		 */
		createGearJoint : function(joint1, joint2, ratio) {
			var jd = new GearJointDef();
			jd.joint1 = joint1;
			jd.joint2 = joint2;
			jd.ratio = ratio;
			
			return physics.m_world.CreateJoint(jd);
		},
		
		getAngle : function(b){
			return -b.GetAngle();
		},
		
		step : function(){
			var timeStep = 1.0/60;
			var iteration = 1;
			this.m_world.Step(timeStep, iteration);
			//console.log("stepping");
		},
		
		/**
		 * Set the density used for newly created shapes.
		 * @param d
		 */
		setDensity : function (d){
			this.m_density = d;
		},
		
		/**
		 * Get the density being used for newly created shapes.
		 * @return
		 */
		getDensity : function () {
			return this.m_density;
		},
		
		/**
		 * Set the restitution used for newly created shapes.
		 * @param r
		 */
		setRestitution : function(r) {
			this.m_restitution = r;
		},
		
		/**
		 * Get the restitution being used for newly created shapes. 
		 * @return
		 */
		getRestitution : function() {
			return this.m_restitution;
		},
		
		/**
		 * Set the friction used for newly created shapes.
		 * @param f
		 */
		setFriction : function(f) {
			this.m_friction = f;
		},
		
		/**
		 * Get the friction being used for newly created shapes.
		 * @return
		 */
		getFriction : function() {
			return this.m_friction;
		},
		
		/**
		 * Set to true to create new bodies as "bullets,"
		 * which use (slower) continuous collision detection
		 * against other moving bodies.
		 * <BR><BR>
		 * <em>Warning:</em> continuous collision detection between
		 * moving bodies is slow, and should be used sparingly.  All
		 * bodies use continuous collision detection against static
		 * scenery, so for most purposes your bodies should not be
		 * marked as bullets.
		 * @param bullet
		 */
		setBullet : function(bullet) {
			this.m_bullet = bullet;
		},
		
		/**
		 * Are newly created bodies being created as bullets?
		 */
		getBullet : function() {
			return this.m_bullet;
		},
		
		/**
		 * Set to true to create new shapes as sensors.  Sensors
		 * do not respond to collisions physically, but they
		 * generate contact events.  This can be useful if you
		 * need to check whether a body is in a certain geometrical
		 * area.
		 * @param sensor
		 */
		setSensor : function(sensor) {
			this.m_sensor = sensor;
		},
		
		/**
		 * Are newly created shapes being created as sensors?
		 */
		getSensor : function() {
			return this.m_sensor;
		},
		
		/**
		 * Get the current physics world.
		 * <BR><BR>
		 * <em>Warning:</em> anything involving a World object directly
		 * is not strictly supported as part of this Processing library.
		 * It <em>is</em> supported as part of JBox2d, however, so there
		 * is quite a bit you can do, and you can always ask for help if
		 * you run into trouble.  Note that all coordinates and vectors
		 * in JBox2d proper are in world coordinates, not screen coordinates,
		 * so you will likely need to use the screenToWorld and worldToScreen
		 * functions to convert back and forth as necessary.
		 * @return The active physics world
		 */
		getWorld : function() {
			return this.m_world;
		},
		
		/** Remove a body from the world. */
		removeBody : function(b) {
			this.m_world.destroyBody(b);
		},
		
		/** Remove a joint from the world. */
		removeJoint : function(j) {
			this.m_world.destroyJoint(j);
		},
		
		destroy : function (){
		},
		
		screenToWorld : function (x,y) {
			//console.log(arguments);
			if (arguments.length == 1) {
				if(x.x == undefined){
					return x/this.m_pixelsPerMeter;
				} else {
					v = new Vec2(x.x, x.y);
				}
			} else {
				v = new Vec2(x,y);
			}
			//console.log(v);
			//v.y = this.m_screenHeight - v.y;
			v.mulLocal(1/this.m_pixelsPerMeter);
			//console.log(v);
			return v;
		},
		screenToWorldX : function (x, y) {return this.screenToWorld(x,y).x;},
		screenToWorldY : function (x, y) {return this.screenToWorld(x,y).y;},
		
		worldToScreen : function (x,y) {
			//console.log("world to screen ");
			//console.log(x);
			if (arguments.length == 1) {
				if(x.x == undefined){
					return x*this.m_pixelsPerMeter;
				} else {
					v = new Vec2(x.x, x.y);
				}
			} else {
				v = new Vec2(x,y);
			}
			v.mulLocal(this.m_pixelsPerMeter);
			//v.y = this.m_screenHeight - v.y;
			//console.log(v);
			return v;
		},
		worldToScreenX : function (x, y) {return this.worldToScreen(x,y).x;},
		worldToScreenY : function (x, y) {return this.worldToScreen(x,y).y;},
		
		drawWorld : function(context) {

			for (var j = this.m_world.m_jointList; j; j = j.m_next) {
				this.drawJoint(j, context);
			}

			for (var b = this.m_world.m_bodyList; b; b = b.m_next) {
				for (var s = b.GetFixtureList(); s != null; s = s.GetNext()) {
					this.drawShape(s.GetShape(), context);
				}
			}
		},
		
		drawJoint : function(joint, context) {
			var b1 = joint.m_body1;
			var b2 = joint.m_body2;
			var x1 = b1.m_position;
			x1 = this.worldToScreen(x1);
			var x2 = b2.m_position;
			x2 = this.worldToScreen(x2);
			var p1 = joint.GetAnchor1();
			p1 = this.worldToScreen(p1);
			var p2 = joint.GetAnchor2();
			p2 = this.worldToScreen(p2);
			context.stroke(0, 200, 20);
			context.noFill();
			//context.beginPath();
			switch (joint.m_type) {
			case b2Joint.e_distanceJoint:
				context.line(p1.x, p1.y, p2.x, p2.y);
				break;

			case b2Joint.e_pulleyJoint:
				// TODO
				break;

			default:
				if (b1 == this.m_world.m_groundBody) {
					context.line(p1.x, p1.y, x2.x, x2.y);
				}
				else if (b2 == this.m_world.m_groundBody) {
					context.lineTo(p1.x, p1.y, x1.x, x1.y);
				}
				else {
					context.line(x1.x, x1.y, p1.x, p1.y);
					context.line(p1.x, p1.y, x2.x, x2.y);
					context.line(x2.x, x2.y, p2.x, p2.y);
				}
				break;
			}
		},
		
		drawShape : function(shape, context) {
			//console.log("drawing " + shape);
			context.stroke(255);
			//context.scale(10);
			context.noFill();
			//context.beginPath();
			switch (shape.m_type) {
			case b2Shape.e_circleShape:
				{
					var circle = shape;
					var pos = circle.m_position;
					var r = circle.m_radius;
					var segments = 16.0;
					var theta = 0.0;
					var dtheta = 2.0 * Math.PI / segments;
					
					context.ellipseMode(context.CENTER);
					pos = this.worldToScreen(pos);
					r = this.worldToScreen(r);
					
					context.ellipse(pos.x, pos.y, r, r);
					// draw circle
					//context.moveTo(pos.x + r, pos.y);
					//for (var i = 0; i < segments; i++) {
					//	var d = new b2Vec2(r * Math.cos(theta), r * Math.sin(theta));
					//	var v = b2Math.AddVV(pos, d);
					//	context.lineTo(v.x, v.y);
					//	theta += dtheta;
					//}
					//context.lineTo(pos.x + r, pos.y);
			
					// draw radius
					//context.moveTo(pos.x, pos.y);
					var ax = circle.m_R.col1;
					var pos2 = new Vec2(pos.x + r * ax.x/2, pos.y + r * ax.y/2);
					
					//pos2 = this.worldToScreen(pos2);
					//context.lineTo(pos2.x, pos2.y);
					context.line(pos.x, pos.y, pos2.x, pos2.y);
				}
				break;
			
			case b2Shape.e_polyShape:
				{
					var poly = shape;
					var tV = new Vec2(0,0);
					tV.set(b2Math.AddVV(poly.m_position, b2Math.b2MulMV(poly.m_R, poly.m_vertices[0])));
					//context.moveTo(tV.x, tV.y);
					context.beginShape();
					//context.vertex(tV.x, tV.y);
					
					
					//console.log("drawing poly " + poly.m_vertexCount);
					//console.log(poly.m_position);
					for (var i = 0; i < poly.m_vertexCount; i++) {
						var v = new Vec2(0,0);
						v.set(b2Math.AddVV(poly.m_position, b2Math.b2MulMV(poly.m_R, poly.m_vertices[i])));
						//context.lineTo(v.x, v.y);
						//console.log(v.x + " " + v.y);
						v = this.worldToScreen(v);
						//console.log(v.x + " " + v.y);
						context.vertex(v.x, v.y);
					}
					tV = this.worldToScreen(tV);
					context.vertex(tV.x, tV.y);
					//context.lineTo(tV.x, tV.y);
					context.endShape();
				}
				break;
			
			}
			
			//context.stroke();
			
		}

		
	};
	
	//var minWorldAABB = new Vec2(-screenAABBWidth*0.5/pixelsPerMeter, -screenAABBHeight*0.5/pixelsPerMeter);
	//var maxWorldAABB = minWorldAABB.mul(-1.0);
	//var maxWorldAABB = new Vec2(screenAABBWidth*0.5/pixelsPerMeter, screenAABBHeight*0.5/pixelsPerMeter);
	//console.log("AABB " + minWorldAABB.x + " " + minWorldAABB.y + " " + maxWorldAABB.x + " " + maxWorldAABB.y);
	
	//console.log("about to create AABB " + AABB);
	
	//var worldAABB = new AABB();
	//worldAABB.lowerBound.Set(-screenAABBWidth*0.5/pixelsPerMeter, -screenAABBHeight*0.5/pixelsPerMeter);
	//worldAABB.upperBound.Set(screenAABBWidth*0.5/pixelsPerMeter, screenAABBHeight*0.5/pixelsPerMeter);
	
	//console.log("about to create world");
	var doSleep = true;
	console.log(physics.m_gravity);
	//physics.m_world = new b2World(worldAABB,physics.m_gravity,doSleep);
	physics.m_world = new World(physics.m_gravity,doSleep);
		
	physics.m_border = physics.createHollowBox(screenW*0.5, screenH*0.5, borderBoxWidth, borderBoxHeight, 10.0);
	
	
	
	//console.log(physics.m_sketch);
	
	if(physics.m_sketch.getProcessingSketchId){
		physics.m_sketch = physics.m_sketch.Processing.getInstanceById(physics.m_sketch.getProcessingSketchId());
		
	}
	if(physics.m_sketch.externals.sketch){
		var sketch = physics.m_sketch.externals.sketch;
		//console.log(physics.m_sketch);
		
		
		var debugDraw = new DebugDraw();
    	debugDraw.SetSprite(physics.m_sketch.externals.context);
		//debugDraw.SetSprite(document.getElementById("canvas").getContext("2d"));
		debugDraw.SetDrawScale(physics.m_pixelsPerMeter);
		debugDraw.SetFillAlpha(0.3);
		debugDraw.SetLineThickness(1.0);
		debugDraw.SetFlags(DebugDraw.e_shapeBit | DebugDraw.e_jointBit | DebugDraw.e_dontClearBackground);
		physics.m_world.SetDebugDraw(debugDraw);
         
		
		sketch.clientOnFrameEnd = sketch.onFrameEnd;
		sketch.onFrameEnd = function(){
			//console.log("in frame start");
			physics.step();
			if(physics.m_customRenderingMethod){
				//console.log(physics.m_customRenderingMethod);
				physics.m_sketch[physics.m_customRenderingMethod](physics.m_world);
			} else {
				//physics.drawWorld(physics.m_sketch);
         		physics.m_world.DrawDebugData();
			}
			physics.m_world.ClearForces();
			sketch.clientOnFrameEnd();
		};
		//console.log(sketch);
	}
	
	return physics;
}
