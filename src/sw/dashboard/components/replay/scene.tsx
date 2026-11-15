'use client';

import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { Component, useEffect, useMemo, useRef, type ReactNode } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { Pose } from '@/lib/trajectory';
import { Drone } from './drone';
import type { Palette } from './palette';

// three.js is y-up; the trajectory is north, west, up. This turn takes one to the other (north to +x, up to +y, west to -z).
const LEVEL: [number, number, number] = [-Math.PI / 2, 0, 0];

function world([north, west, up]: [number, number, number]) {
  return new THREE.Vector3(north, up, -west);
}

// Orbit about the drone and keep orbiting it as it moves: the camera carries the drone's motion, so the view the user chose holds.
function Follow({ target }: { target: [number, number, number] }) {
  const { camera, gl } = useThree();
  const controls = useMemo(() => {
    const value = new OrbitControls(camera, gl.domElement);
    value.enableDamping = true;
    value.minDistance = 0.4;
    value.maxDistance = 60;
    return value;
  }, [camera, gl]);
  const last = useRef<THREE.Vector3 | null>(null);
  useEffect(() => () => controls.dispose(), [controls]);
  useFrame(() => {
    const goal = world(target);
    if (last.current) camera.position.add(goal.clone().sub(last.current));
    controls.target.copy(goal);
    last.current = goal;
    controls.update();
  });
  return null;
}

// The path flown, darker up to now; a plumb line from the drone to the ground for its height.
function Trail({ position, frame, palette }: { position: number[]; frame: number; palette: Palette }) {
  const lines = useMemo(() => {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(position, 3));
    return {
      whole: new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: palette.hairline })),
      past: new THREE.Line(geometry.clone(), new THREE.LineBasicMaterial({ color: palette.gray })),
    };
  }, [position, palette]);
  lines.past.geometry.setDrawRange(0, frame + 1);
  return (
    <>
      <primitive object={lines.whole} />
      <primitive object={lines.past} />
    </>
  );
}

function Plumb({ at, palette }: { at: [number, number, number]; palette: Palette }) {
  const geometry = useMemo(
    () => new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -1)]),
    []
  );
  return (
    <lineSegments geometry={geometry} position={at} scale={[1, 1, Math.max(at[2], 0.001)]}>
      <lineBasicMaterial color={palette.gray} transparent opacity={0.6} />
    </lineSegments>
  );
}

// Where the browser gives no WebGL (a sandboxed pane, a disabled GPU), the hero says so and the strips and timeline carry on.
function NoWebGL() {
  return <div className="grid h-full place-items-center text-sm text-muted-foreground">3D needs WebGL, which this browser has off</div>;
}

class Guard extends Component<{ children: ReactNode }, { failed: boolean }> {
  state = { failed: false };
  static getDerivedStateFromError() {
    return { failed: true };
  }
  render() {
    return this.state.failed ? <NoWebGL /> : this.props.children;
  }
}

export function Scene(props: { pose: Pose; position: number[] | null; frame: number; palette: Palette }) {
  return (
    <Guard>
      <View {...props} />
    </Guard>
  );
}

function View({ pose, position, frame, palette }: { pose: Pose; position: number[] | null; frame: number; palette: Palette }) {
  return (
    <Canvas fallback={<NoWebGL />} camera={{ position: [-2.2, 1.6, 2.2], fov: 40, near: 0.05, far: 500 }} dpr={[1, 2]} style={{ background: palette.paper }}>
      <gridHelper key={palette.hairline} args={[40, 40, palette.hairline, palette.hairline]} />
      <group rotation={LEVEL}>
        {position && <Trail position={position} frame={frame} palette={palette} />}
        <Plumb at={pose.position} palette={palette} />
        <Drone pose={pose} palette={palette} />
      </group>
      <Follow target={pose.position} />
      <fog key={palette.paper} attach="fog" args={[palette.paper, 12, 40]} />
    </Canvas>
  );
}
