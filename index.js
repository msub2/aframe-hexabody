AFRAME.registerComponent('hexabody', {
  schema: {
    // Hexabody move speed
    moveForceCrouch: { type: 'float', default: 5 },
    moveForceWalk: { type: 'float', default: 10 },
    moveForceSprint: { type: 'float', default: 15 },
    // Hexabody angular drag
    angularDragOnMove: { type: 'float', default: 5 },
    angularBrakeDrag: { type: 'float', default: 50 },
    crouchSpeed: { type: 'float', default: 1.5 },
    lowestCrouch: { type: 'float', defualt: .05 },
    highestCrouch: { type: 'float', default: 1.8 },
    // Physics Hands
    positionStrength: { type: 'float', default: 20 },
    positionThreshold: { type: 'float', default: 0.005 },
    maxDistance: { type: 'float', default: 1 },
    rotationStregth: { type: 'float', default: 30 },
    rotationThreshold: { type: 'float', default: .3 }
  },

  init: function () {
    this.physicsStarted = false;
    document.addEventListener('physx-started', () => { this.physicsStarted = true; });
    
    // three.js temp variables
    this.v3_1 = new THREE.Vector3();
    this.v3_2 = new THREE.Vector3();
    this.v3_3 = new THREE.Vector3();
    this.v3_4 = new THREE.Vector3();
    this.tempQuat = new THREE.Quaternion();
    
    // Camera Rig
    this.cameraRig = document.querySelector('#cameraRig');
    this.cameraRigObj = this.cameraRig.object3D;
    this.trackedHeadOffset = document.querySelector('#trackedHeadOffset');
    this.trackedHeadOffsetObj = this.trackedHeadOffset.object3D;
    this.trackedHead = document.querySelector('#trackedHead');
    this.trackedHeadObj = this.trackedHead.object3D;
    this.trackedLeftHand = document.querySelector('#trackedLeftHand');
    this.trackedLeftHandObj = this.trackedLeftHand.object3D;
    this.trackedLeftHandControls = this.trackedLeftHand.components['tracked-controls-webxr'];
    this.trackedRightHand = document.querySelector('#trackedRightHand');
    this.trackedRightHandObj = this.trackedRightHand.object3D;
    this.trackedRightHandControls = this.trackedRightHand.components['tracked-controls-webxr'];
    
    // Hexabody Parts
    this.head = document.querySelector('#head');
    this.headObj = this.head.object3D;
    this.headBody = this.head.components['physx-body'];
    this.leftHand = document.querySelector('#leftHand');
    this.leftHandObj = this.leftHand.object3D;
    this.leftHandBody = this.leftHand.components['physx-body'];
    this.rightHand = document.querySelector('#rightHand');
    this.rightHandObj = this.rightHand.object3D;
    this.rightHandBody = this.rightHand.components['physx-body'];
    this.pelvis = document.querySelector('#pelvis');
    this.pelvisObj = this.pelvis.object3D;
    this.pelvisBody = this.pelvis.components['physx-body'];
    this.fender = document.querySelector('#fender');
    this.fenderObj = this.fender.object3D;
    this.fenderBody = this.fender.components['physx-body'];
    this.locomotionSphere = document.querySelector('#locomotionSphere');
    this.locomotionSphereObj = this.locomotionSphere.object3D;
    this.locomotionSphereBody = this.locomotionSphere.components['physx-body'];

    this.leftHandJoint = document.querySelector('#leftHandJoint').components['physx-joint'];
    this.rightHandJoint = document.querySelector('#rightHandJoint').components['physx-joint'];
    this.spine = document.querySelector('#spine').components['physx-joint'];

    // Hexabody Crouch and Jump
    this.jumping = false;

    this.additionalHeight = 0;
    this.locomotionSphereObj.getWorldScale(this.v3_1);
    this.fenderObj.getWorldScale(this.v3_2);
    this.additionalHeight = (0.5 * this.v3_1.y) + (0.5 * this.v3_2.y) + (this.headObj.position.y - this.trackedHeadOffsetObj.position.y);

    this.crouchTarget = new THREE.Vector3();

    // Input values
    this.headYaw = new THREE.Euler(0, 0, 0, 'YXZ');
    this.moveDirection = new THREE.Vector3();
    this.locomotionSphereTorque = new THREE.Vector3();
    this.input = {
      left: { axes: [], buttons: [] },
      right: { axes: [], buttons: [] }
    };

    // Physics Hands vars
    this.velocity = new THREE.Vector3();
    this.distance = 0;
    this.angleDistance = 0;
    this.kp = 0;
    this.kd = 0;
    this.ax = new THREE.Vector4();
    this.axMag = 0;
    this.axVec = new THREE.Vector3();
    this.q = new THREE.Quaternion();
    this.q2 = new THREE.Quaternion();
    this.pidv = new THREE.Vector3();
    this.rotInertia2World = new THREE.Quaternion();
    this.angularVelocity = new THREE.Vector3();
    this.inertiaTensor = new THREE.Vector3();
  },

  tick: function (time, timeDelta) {
    if (!this.physicsStarted) return;
    
    //this.cameraToPlayer();
    this.cameraRigToPlayer();
    this.getControllerInput();

    this.movePlayer();
    this.jump(timeDelta);

    if (!this.jumping) {
      this.contractSpineOnCrouch();
    }

    this.rotatePlayer();
    this.moveAndRotateHand();
  },

  getControllerInput: function () {
    this.input.left.axes = this.trackedLeftHandControls.axis;
    this.input.left.buttons = this.trackedLeftHandControls.buttonStates;
    this.input.right.axes = this.trackedRightHandControls.axis;
    this.input.right.buttons = this.trackedRightHandControls.buttonStates;
    
    this.headYaw.setFromQuaternion(this.trackedHeadObj.quaternion, 'YXZ');
    this.moveDirection.set(this.input.left.axes[2], 0, this.input.left.axes[3]);
    this.moveDirection.applyEuler(this.headYaw);
    this.locomotionSphereTorque.set(this.moveDirection.z, 0, -this.moveDirection.x);
  },

  // Transforms
  cameraToPlayer: function () {
    this.trackedHeadOffsetObj.position.copy(this.headObj.position);
  },

  cameraRigToPlayer: function () {
    const { x: f_x, y: f_y, z: f_z } = this.fenderObj.position;
    const ls_y = this.locomotionSphereObj.scale.y;
    const fs_y = this.fenderObj.scale.y;
    const yPos = f_y - (0.5 * fs_y + 0.5 * ls_y);    
    this.cameraRigObj.position.set(f_x, yPos, f_z);
  },

  rotatePlayer: function () {
    this.pelvisObj.rotation.set(0, this.trackedHeadObj.rotation.y, 0);
  },

  // Hexabody Movement
  movePlayer: function () {
    const axes = this.trackedLeftHandControls.axis;
    const buttons = this.trackedLeftHandControls.buttonStates;

    if (axes.every(val => val === 0 || val == null)) this.stopLocomotionSphere();
    else if (axes.some(val => val !== 0 && val != null)) {
      if (!this.jumping) {
        this.moveLocomotionSphere(this.data.moveForceWalk);
      } else {
        this.moveLocomotionSphere(this.data.moveForceCrouch);
      }
    }
    // Implement sprinting later
  },

  moveLocomotionSphere: function (moveForce) {
    this.locomotionSphereBody.rigidBody.setRigidDynamicLockFlag(PhysX.PxRigidDynamicLockFlag.eLOCK_ANGULAR_X, false);
    this.locomotionSphereBody.rigidBody.setRigidDynamicLockFlag(PhysX.PxRigidDynamicLockFlag.eLOCK_ANGULAR_Y, false);
    this.locomotionSphereBody.rigidBody.setRigidDynamicLockFlag(PhysX.PxRigidDynamicLockFlag.eLOCK_ANGULAR_Z, false);
    this.locomotionSphereBody.rigidBody.setAngularDamping(this.data.angularDragOnMove);
    const {x, y, z} = this.locomotionSphereTorque.normalize().multiplyScalar(moveForce);
    this.locomotionSphereBody.rigidBody.addTorque({ x: x, y: y, z: z });
  },

  stopLocomotionSphere: function () {
    this.locomotionSphereBody.rigidBody.setAngularDamping(this.data.angularBrakeDrag);
    const velocity = this.locomotionSphereBody.rigidBody.getLinearVelocity();
    if (velocity.x < .01 && velocity.y < .01 && velocity.z < .01) {
      this.locomotionSphereBody.rigidBody.setRigidDynamicLockFlag(PhysX.PxRigidDynamicLockFlag.eLOCK_ANGULAR_X, true);
      this.locomotionSphereBody.rigidBody.setRigidDynamicLockFlag(PhysX.PxRigidDynamicLockFlag.eLOCK_ANGULAR_Y, true);
      this.locomotionSphereBody.rigidBody.setRigidDynamicLockFlag(PhysX.PxRigidDynamicLockFlag.eLOCK_ANGULAR_Z, true);
    }
  },

  // Jumping
  jump: function () {
    if (this.input.right.buttons[4]?.pressed) {
      jumping = true;
      this.jumpSitDown();
    } else {
      jumping = false;
      this.jumpSitUp();
    }
  },

  jumpSitDown: function (dt) {
    if (this.crouchTarget.y >= this.lowestCrouch) {
      this.crouchTarget.y -= this.crouchSpeed * dt;
      this.spine.joint.setDrivePosition({
        translation: { x: 0, y: this.crouchTarget.y, z: 0 },
        rotation: { w: 1, x: 0, y: 0, z: 0 }
      }, true);
    }
  },

  jumpSitUp: function () {
    this.crouchTarget.set(0, this.data.highestCrouch - this.additionalHeight, 0);
    this.spine.joint.setDrivePosition({
      translation: { x: 0, y: this.crouchTarget.y, z: 0 },
        rotation: { w: 1, x: 0, y: 0, z: 0 }
    }, true);
  },

  // Joint Control
  contractSpineOnCrouch: function () {
    const crouchValue = this.trackedHeadObj.position.y - this.additionalHeight;
    this.crouchTarget.y = this.clamp(crouchValue, this.lowestCrouch, this.highestCrouch - this.additionalHeight);
    this.spine.joint.setDrivePosition({
      translation: { x: 0, y: this.crouchTarget.y, z: 0 },
      rotation: { w: 1, x: 0, y: 0, z: 0 }
    }, true);
  },

  moveAndRotateHand: function () {
    const { x: p1, y: p2, z: p3 } = this.v3_1.subVectors(this.trackedLeftHandObj.position, this.trackedHeadObj.position);
    const { x: p4, y: p5, z: p6 } = this.v3_2.subVectors(this.trackedRightHandObj.position, this.trackedHeadObj.position);
    const { w: q1, x: q2, y: q3, z: q4 } = this.trackedLeftHandObj.quaternion;
    const { w: q5, x: q6, y: q7, z: q8 } = this.trackedRightHandObj.quaternion;
    this.leftHandJoint.joint.setDrivePosition({
      translation: { x: p1, y: p2, z: p3 },
      rotation: { w: q1, x: q2, y: q3, z: q4 }
    }, true);
    this.rightHandJoint.joint.setDrivePosition({
      translation: { x: p4, y: p5, z: p6 },
      rotation: { w: q5, x: q6, y: q7, z: q8 }
    }, true);

    // [this.leftHandBody, this.rightHandBody].forEach((body, i) => {
    //   const obj = [this.leftHandObj, this.rightHandObj][i];
    //   const controllerObj = [this.trackedLeftHandObj, this.trackedRightHandObj][i];
    //   // Get distance between real hand position and physics hand position
    //   this.distance = controllerObj.position.distanceTo(obj.position);
    //   if (this.distance > this.data.maxDistance || this.distance < this.data.positionThreshold) {
    //     // Manually just set position if too far away or under threshold
    //     // NOTE: Attempting to warp while still colliding with an object may cause the hand
    //     // to bounce away and become unresponsive to setGlobalPose until shaken (don't know why),
    //     // so keep the maxDistance reasonably high for now to try and mitigate this.
    //     body.rigidBody.setGlobalPose({
    //       translation: {
    //         x: controllerObj.position.x,
    //         y: controllerObj.position.y,
    //         z: controllerObj.position.z
    //       },
    //       rotation: { 
    //         w: obj.quaternion.w, 
    //         x: obj.quaternion.x, 
    //         y: obj.quaternion.y, 
    //         z: obj.quaternion.z
    //       }
    //     }, true);
    //   } else {
    //     // Get normalized direction vector
    //     this.velocity.subVectors(controllerObj.position, obj.position).normalize();
    //     // Set physics hand velocity to normalized direction vector multiplied by follow strength
    //     this.velocity.multiplyScalar(this.data.positionStrength * this.distance);
    //     body.rigidBody.setLinearVelocity({
    //       x: this.velocity.x,
    //       y: this.velocity.y,
    //       z: this.velocity.z
    //     }, true);
    //   }
  
    //   // Get magnitude of angle between real hand rotation and physics hand rotation
    //   this.angleDistance = obj.quaternion.angleTo(controllerObj.quaternion);
    //   if (this.angleDistance < this.data.rotationThreshold || this.angleDistance > 2) {
    //     // Manually set rotation if distance too high or under threshold
    //     body.rigidBody.setGlobalPose({
    //       translation: {
    //         x: obj.position.x,
    //         y: obj.position.y,
    //         z: obj.position.z
    //       },
    //       rotation: {
    //         w: controllerObj.quaternion.w,
    //         x: controllerObj.quaternion.x,
    //         y: controllerObj.quaternion.y,
    //         z: controllerObj.quaternion.z
    //       }
    //     }, true);
    //     body.rigidBody.setAngularVelocity({ x: 0, y: 0, z: 0 }, true);
    //   } else {
    //     // This is essentially the PD code from the tutorial with two changes:
    //     // 1 - inertiaTensorRotation is actually just the identity quaternion in Unity, so that's subbed in instead
    //     // 2 - No Deg2Rad statement needed since three.js natively works with radians.
    //     this.kp = ((6 * this.data.rotationStregth) ** 2) * 0.25;
    //     this.kd = 4.5 * this.data.rotationStregth;
    //     this.q.copy(controllerObj.quaternion).multiply(this.q2.copy(obj.quaternion).invert());
    //     this.ax.setAxisAngleFromQuaternion(this.q);
    //     this.axVec.set(this.ax.x, this.ax.y, this.ax.z);
    //     this.axMag = this.ax.w;
    //     this.axVec.normalize();
    //     const { x: x1, y: y1, z: z1 } = body.rigidBody.getAngularVelocity()
    //     this.angularVelocity.set(x1, y1, z1);
    //     this.pidv.copy(this.axVec).multiplyScalar(this.kp).multiplyScalar(this.axMag).sub(this.angularVelocity.multiplyScalar(this.kd));
    //     const { x: x2, y: y2, z: z2 } = body.rigidBody.getMassSpaceInertiaTensor();
    //     this.inertiaTensor.set(x2, y2, z2);
    //     this.rotInertia2World.copy(this.q.identity()).multiply(obj.quaternion);
    //     this.pidv.applyQuaternion(this.rotInertia2World.invert());
    //     this.pidv.multiplyVectors(this.pidv, this.inertiaTensor);
    //     this.pidv.applyQuaternion(this.rotInertia2World);
    //     body.rigidBody.addTorque({ x: this.pidv.x, y: this.pidv.y, z: this.pidv.z });
    //   }
    // })
  },

  // Utils
  clamp: function (val, min, max) {
    return val > max ? max : val < min ? min : val;
  }
});
