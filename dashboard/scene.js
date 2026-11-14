// Adapted from rocket-landing-rl-simulator/dashboard/scene.js so both
// dashboards use the same procedural rocket, landing legs, nozzle, plume,
// launch pad, target marker, trail, lighting, and camera controls.
// Simulator frame: x/y horizontal and z up. Rotating the world -90 degrees
// around x maps that convention into three.js's y-up frame.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const MAX_TRAIL_POINTS = 10000;
const TRAIL_STEP_M = 0.035;
const PLUME_AT_FULL_THRUST_M = 1.2;
const PLUME_MIN_M = 0.005;
const WIND_ARROW_M_PER_MPS = 0.3;
const WIND_ARROW_MIN_MPS = 0.1;
const WIND_ARROW_HEIGHT_M = 1.0;
const FOLLOW_OFFSET = new THREE.Vector3(3.6, 1.4, 4.4);
const OVERVIEW_MIN_HEIGHT_M = 10;
const GROUND_SIZE_M = 500;
const GRID_SIZE_M = 120;
const GRID_CELLS = 24;
const PAD_RADIUS_M = 0.75;
const TARGET_DISC_RADIUS_M = 3;
const CIRCLE_SEGMENTS = 128;
const Z_AXIS = new THREE.Vector3(0, 0, 1);
const UP = new THREE.Vector3(0, 1, 0);
const DEG = Math.PI / 180;

function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function thrustDirection(pitch, yaw) {
  return new THREE.Vector3(-Math.tan(pitch), -Math.tan(yaw), 1).normalize();
}

function alongBody(geometry, centreStation, vehicle) {
  return geometry.rotateX(Math.PI / 2).translate(0, 0, vehicle.cg - centreStation);
}

function strut(from, to, radius, material) {
  const start = new THREE.Vector3(...from);
  const end = new THREE.Vector3(...to);
  const direction = end.clone().sub(start);
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, direction.length(), 6), material);
  mesh.quaternion.setFromUnitVectors(UP, direction.clone().normalize());
  mesh.position.copy(start).addScaledVector(direction, 0.5);
  return mesh;
}

function circleLine(radius, z, material) {
  const points = [];
  for (let index = 0; index <= CIRCLE_SEGMENTS; index += 1) {
    const angle = 2 * Math.PI * index / CIRCLE_SEGMENTS;
    points.push(new THREE.Vector3(radius * Math.cos(angle), radius * Math.sin(angle), z));
  }
  return new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), material);
}

function translucent(color, opacity) {
  return new THREE.MeshBasicMaterial({ color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false });
}

export class FlightScene {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.append(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.FogExp2(cssVar('--scene-bg'), 0.0022);
    this.world = new THREE.Group();
    this.world.rotation.x = -Math.PI / 2;
    this.scene.add(this.world);

    this.scene.add(new THREE.HemisphereLight(0xddecff, 0x21313a, 1.9));
    const sun = new THREE.DirectionalLight(0xffffff, 2.1);
    sun.position.set(30, 60, 20);
    this.scene.add(sun);

    this.camera = new THREE.PerspectiveCamera(48, 1, 0.05, 4000);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.07;
    this.follow = true;
    this.vehicleInfo = null;
    this.vehicle = new THREE.Group();
    this.markers = new THREE.Group();
    this.world.add(this.vehicle, this.markers);
    this.windArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, 0xff7043, 0.2, 0.12);
    this.windArrow.visible = false;
    this.world.add(this.windArrow);
    this.buildGround();
    this.buildTrail();
    this.world.add(new THREE.AxesHelper(1));

    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    this.placeCamera();
    this.renderer.setAnimationLoop(() => this.render());
  }

  buildGround() {
    this.scene.background = new THREE.Color(cssVar('--scene-bg'));
    this.ground = new THREE.Mesh(
      new THREE.PlaneGeometry(GROUND_SIZE_M, GROUND_SIZE_M),
      new THREE.MeshStandardMaterial({ color: cssVar('--scene-ground'), roughness: 1 }),
    );
    this.grid = new THREE.GridHelper(GRID_SIZE_M, GRID_CELLS, cssVar('--scene-grid'), cssVar('--scene-grid'));
    this.grid.rotation.x = Math.PI / 2;
    this.grid.position.z = 0.003;
    this.world.add(this.ground, this.grid);
  }

  buildTrail() {
    this.trailPositions = new Float32Array(MAX_TRAIL_POINTS * 3);
    this.trailCount = 0;
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(this.trailPositions, 3));
    geometry.setDrawRange(0, 0);
    this.trail = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0x48a8ff }));
    this.trail.frustumCulled = false;
    this.world.add(this.trail);
  }

  setup(info) {
    this.vehicleInfo = info.vehicle;
    this.buildVehicle(info.vehicle);
    this.buildMarkers(info.mission, info.vehicle);
    this.reset();
  }

  reset() {
    const z = this.vehicleInfo?.pad_cg_height || 0;
    this.vehicle.position.set(0, 0, z);
    this.vehicle.quaternion.identity();
    this.trailCount = 0;
    this.trail.geometry.setDrawRange(0, 0);
    if (this.plume) this.plume.visible = false;
    this.windArrow.visible = false;
    this.placeCamera();
  }

  buildVehicle(vehicle) {
    this.vehicle.traverse((object) => object.geometry?.dispose());
    this.vehicle.clear();
    const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0xe8ebef, roughness: 0.55, metalness: 0.08 });
    const noseMaterial = new THREE.MeshStandardMaterial({ color: 0xff7043, roughness: 0.5 });
    const darkMaterial = new THREE.MeshStandardMaterial({ color: 0x343e48, roughness: 0.7 });
    const radius = vehicle.diameter / 2;
    const noseLength = Math.min(2.5 * vehicle.diameter, vehicle.length / 3);
    this.vehicle.add(new THREE.Mesh(alongBody(new THREE.ConeGeometry(radius, noseLength, 32), noseLength / 2, vehicle), noseMaterial));
    this.vehicle.add(new THREE.Mesh(
      alongBody(new THREE.CylinderGeometry(radius, radius, vehicle.length - noseLength, 32), (noseLength + vehicle.length) / 2, vehicle),
      bodyMaterial,
    ));

    const hinge = vehicle.cg - 0.85 * vehicle.length;
    const footZ = vehicle.cg - vehicle.length - vehicle.leg_height;
    for (let index = 0; index < vehicle.leg_count; index += 1) {
      const angle = 2 * Math.PI * index / vehicle.leg_count + Math.PI / 4;
      const foot = [vehicle.leg_span / 2 * Math.cos(angle), vehicle.leg_span / 2 * Math.sin(angle), footZ];
      this.vehicle.add(strut([radius * Math.cos(angle), radius * Math.sin(angle), hinge], foot, Math.max(0.006, vehicle.diameter * 0.018), darkMaterial));
      const pad = new THREE.Mesh(new THREE.SphereGeometry(Math.max(0.014, vehicle.diameter * 0.035), 10, 8), darkMaterial);
      pad.position.set(...foot);
      this.vehicle.add(pad);
    }

    this.gimbal = new THREE.Group();
    this.gimbal.position.z = vehicle.cg - vehicle.pivot;
    const nozzleLength = 0.5 * vehicle.diameter;
    this.gimbal.add(new THREE.Mesh(
      new THREE.CylinderGeometry(0.3 * vehicle.diameter, 0.42 * vehicle.diameter, nozzleLength, 20)
        .rotateX(Math.PI / 2).translate(0, 0, -nozzleLength / 2),
      darkMaterial,
    ));
    this.plume = new THREE.Mesh(
      new THREE.ConeGeometry(0.36 * vehicle.diameter, 1, 20, 1, true).rotateX(-Math.PI / 2).translate(0, 0, -0.5),
      translucent(0xffa040, 0.82),
    );
    this.plume.visible = false;
    this.gimbal.add(this.plume);
    this.vehicle.add(this.gimbal);
  }

  buildMarkers(mission, vehicle) {
    this.markers.traverse((object) => object.geometry?.dispose());
    this.markers.clear();
    const aqua = cssVar('--series-3');
    const pad = new THREE.Mesh(
      new THREE.CircleGeometry(PAD_RADIUS_M, 48),
      new THREE.MeshStandardMaterial({ color: 0x66737c, roughness: 0.9 }),
    );
    pad.position.z = 0.006;
    const radius = mission.landing_radius;
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(Math.max(0, radius - 0.06), radius + 0.06, CIRCLE_SEGMENTS),
      translucent(aqua, 0.9),
    );
    ring.position.z = 0.01;
    const target = mission.target_altitude + vehicle.pad_cg_height;
    const tolerance = mission.altitude_tolerance;
    const disc = new THREE.Mesh(new THREE.CircleGeometry(TARGET_DISC_RADIUS_M, 64), translucent(aqua, 0.13));
    disc.position.z = target;
    const band = new THREE.Mesh(
      new THREE.CylinderGeometry(TARGET_DISC_RADIUS_M, TARGET_DISC_RADIUS_M, 2 * tolerance, 64, 1, true).rotateX(Math.PI / 2),
      translucent(aqua, 0.055),
    );
    band.position.z = target;
    const edge = new THREE.LineBasicMaterial({ color: aqua, transparent: true, opacity: 0.55 });
    this.markers.add(
      pad, ring, disc, band,
      circleLine(TARGET_DISC_RADIUS_M, target - tolerance, edge),
      circleLine(TARGET_DISC_RADIUS_M, target + tolerance, edge),
    );
    this.overviewHeight = Math.max(target, OVERVIEW_MIN_HEIGHT_M) + vehicle.length;
  }

  update(frame, addTrail = true) {
    if (!this.vehicleInfo) return;
    const [x, y, z] = frame.position;
    const visualZ = z + this.vehicleInfo.pad_cg_height;
    const [qw, qx, qy, qz] = frame.quaternion;
    const before = this.vehicleWorldPosition();
    this.vehicle.position.set(x, y, visualZ);
    this.vehicle.quaternion.set(qx, qy, qz, qw).normalize();
    if (this.gimbal) {
      const [pitch, yaw] = frame.gimbal_deg;
      this.gimbal.quaternion.setFromUnitVectors(Z_AXIS, thrustDirection(pitch * DEG, yaw * DEG));
      const length = PLUME_AT_FULL_THRUST_M * Math.max(0, frame.thrust) / (this.vehicleInfo.max_thrust || 1);
      this.plume.visible = length > PLUME_MIN_M;
      this.plume.scale.z = Math.max(length, PLUME_MIN_M);
    }
    if (addTrail) this.addTrailPoint(x, y, visualZ);
    this.showWind(frame.wind, x, y, visualZ);
    if (this.follow) {
      const after = this.vehicleWorldPosition();
      const movement = after.clone().sub(before);
      this.camera.position.add(movement);
      this.controls.target.add(movement);
    }
  }

  addTrailPoint(x, y, z) {
    const count = this.trailCount;
    if (count >= MAX_TRAIL_POINTS) return;
    if (count > 0) {
      const positions = this.trailPositions;
      if (Math.hypot(x - positions[3 * count - 3], y - positions[3 * count - 2], z - positions[3 * count - 1]) < TRAIL_STEP_M) return;
    }
    this.trailPositions.set([x, y, z], 3 * count);
    this.trailCount = count + 1;
    this.trail.geometry.attributes.position.needsUpdate = true;
    this.trail.geometry.setDrawRange(0, this.trailCount);
  }

  showWind(wind, x, y, z) {
    const speed = Math.hypot(wind[0], wind[1]);
    this.windArrow.visible = speed >= WIND_ARROW_MIN_MPS;
    if (!this.windArrow.visible) return;
    this.windArrow.position.set(x, y, z + WIND_ARROW_HEIGHT_M);
    this.windArrow.setDirection(new THREE.Vector3(wind[0] / speed, wind[1] / speed, 0));
    this.windArrow.setLength(speed * WIND_ARROW_M_PER_MPS, 0.2, 0.12);
  }

  rebuildTrail(frames, throughIndex) {
    this.trailCount = 0;
    this.trail.geometry.setDrawRange(0, 0);
    const stride = Math.max(1, Math.ceil((throughIndex + 1) / MAX_TRAIL_POINTS));
    for (let index = 0; index <= throughIndex; index += stride) {
      const [x, y, z] = frames[index].position;
      this.addTrailPoint(x, y, z + this.vehicleInfo.pad_cg_height);
    }
  }

  vehicleWorldPosition() {
    return this.vehicle.getWorldPosition(new THREE.Vector3());
  }

  setFollow(follow) {
    this.follow = follow;
    this.placeCamera();
  }

  placeCamera() {
    if (this.follow && this.vehicleInfo) {
      const at = this.vehicleWorldPosition();
      this.controls.target.copy(at);
      this.camera.position.copy(at).add(FOLLOW_OFFSET);
    } else {
      const height = this.overviewHeight || OVERVIEW_MIN_HEIGHT_M;
      this.controls.target.set(0, 0.45 * height, 0);
      this.camera.position.set(1.05 * height, 0.72 * height, 1.25 * height);
    }
    this.controls.update();
  }

  resize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    if (!width || !height) return;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
