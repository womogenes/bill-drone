'use client';

import { useMemo } from 'react';
import * as THREE from 'three';
import type { Pose } from '@/lib/trajectory';
import type { Palette } from './palette';

// Drone 1 as line work, in its body axes (forward, left, up; meters): the arm to each rotor (sw/sim/flight.py MODEL arm), a 10-inch rotor disc on each tilting assembly, its pivot a little above the body.
const ARM = 0.172;
const ROTOR = 0.127;
const PIVOT = 0.03;
const MAST = 0.045;
// The thrust line's length at full thrust (m).
const THRUST_LINE = 0.35;

function edges(geometry: THREE.BufferGeometry) {
  return new THREE.EdgesGeometry(geometry);
}

function ring(radius: number) {
  const points = Array.from({ length: 49 }, (_, index) => {
    const angle = (index / 48) * Math.PI * 2;
    return new THREE.Vector3(Math.cos(angle) * radius, Math.sin(angle) * radius, 0);
  });
  return new THREE.BufferGeometry().setFromPoints(points);
}

function segment(from: [number, number, number], to: [number, number, number]) {
  return new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...from), new THREE.Vector3(...to)]);
}

// One rotor assembly: tilted forward about the left axis by its tilt (degrees), its disc tinted and its thrust line drawn by its thrust (0-1).
function Rotor({ side, tilt, thrust, palette }: { side: 1 | -1; tilt: number; thrust: number; palette: Palette }) {
  const shapes = useMemo(
    () => ({
      motor: edges(new THREE.CylinderGeometry(0.016, 0.016, 0.03, 12).rotateX(Math.PI / 2)),
      disc: ring(ROTOR),
      fill: new THREE.CircleGeometry(ROTOR, 48),
      line: segment([0, 0, 0], [0, 0, 1]),
    }),
    []
  );
  return (
    <group position={[0, side * ARM, PIVOT]} rotation={[0, (tilt * Math.PI) / 180, 0]}>
      <lineSegments geometry={shapes.motor} position={[0, 0, MAST - 0.015]}>
        <lineBasicMaterial color={palette.ink} />
      </lineSegments>
      <lineLoop geometry={shapes.disc} position={[0, 0, MAST]}>
        <lineBasicMaterial color={palette.ink} transparent opacity={0.55} />
      </lineLoop>
      <mesh geometry={shapes.fill} position={[0, 0, MAST]}>
        <meshBasicMaterial color={palette.signal} transparent opacity={0.04 + 0.2 * thrust} side={THREE.DoubleSide} depthWrite={false} />
      </mesh>
      <lineSegments geometry={shapes.line} position={[0, 0, MAST]} scale={[1, 1, Math.max(thrust * THRUST_LINE, 0.001)]}>
        <lineBasicMaterial color={palette.signal} />
      </lineSegments>
    </group>
  );
}

export function Drone({ pose, palette }: { pose: Pose; palette: Palette }) {
  const shapes = useMemo(
    () => ({
      body: edges(new THREE.BoxGeometry(0.14, 0.07, 0.04)),
      arms: segment([0, -ARM, PIVOT], [0, ARM, PIVOT]),
      nose: segment([0.07, 0, 0.02], [0.13, 0, 0.02]),
    }),
    []
  );
  const [w, x, y, z] = pose.attitude;
  return (
    <group position={pose.position} quaternion={[x, y, z, w]}>
      <lineSegments geometry={shapes.body}>
        <lineBasicMaterial color={palette.ink} />
      </lineSegments>
      <lineSegments geometry={shapes.arms}>
        <lineBasicMaterial color={palette.ink} />
      </lineSegments>
      <lineSegments geometry={shapes.nose}>
        <lineBasicMaterial color={palette.signal} />
      </lineSegments>
      <Rotor side={1} tilt={pose.tilt[0]} thrust={pose.thrust[0]} palette={palette} />
      <Rotor side={-1} tilt={pose.tilt[1]} thrust={pose.thrust[1]} palette={palette} />
    </group>
  );
}
