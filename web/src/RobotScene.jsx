import React, { Suspense, useMemo, useRef } from "react";
import { Canvas, extend, useFrame, useLoader, useThree } from "@react-three/fiber";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { Matrix4, MeshStandardMaterial, Quaternion, SRGBColorSpace, TextureLoader, Vector3 } from "three";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";

extend({ OrbitControls });

const mujocoToThree = new Matrix4().makeBasis(new Vector3(1, 0, 0), new Vector3(0, 0, -1), new Vector3(0, 1, 0));
const threeToMujoco = mujocoToThree.clone().transpose();
const meshMaterials = {
	base_0: { color: "#0d2d08" },
	base_1: { color: "#2e1e02" },
	base_2: { color: "#2d0201" },
	base_3: { color: "#060606" },
	base_4: { color: "#d0d0d0" },
	base_5: { texture: "base.png" },
	top_shell: { texture: "top_shell.png" },
	bottom_shell: { texture: "bottom_shell.png" },
	remote: { texture: "remote.png" },
	hatch: { texture: "hatch.png" },
	handle: { texture: "handle.png" },
	face: { texture: "face.png" },
	battery: { texture: "battery.png" },
	lidar_cage: { texture: "lidar_cage.png" },
	depth_camera: { texture: "depth_camera.png" },
	wide_angle_camera: { texture: "wide_angle_camera.png" },
	lidar: { texture: "lidar.png" },
	drive: { texture: "drive.png" },
	hip_l: { texture: "hip_l.png" },
	hip_r: { texture: "hip_r.png" },
	thigh: { texture: "thigh.png" },
	shank_l: { texture: "shank_l.png" },
	shank_r: { texture: "shank_r.png" },
	foot: { texture: "foot.png" }
};

const nominalFeet = [
	[0.35, -0.2, -0.54],
	[0.35, 0.2, -0.54],
	[-0.35, -0.2, -0.54],
	[-0.35, 0.2, -0.54]
];

function Leg({ foot, index, force }) {
	const [x, height, depth] = foot;
	const sign = depth < 0 ? -1 : 1;
	const magnitude = Math.max(0.16, Math.min(0.48, Math.abs(force?.[2] ?? 0) / 170));
	const hip = [x * 0.78, 0.08, depth * 0.78];
	const knee = [x * 0.82, height * 0.48, depth * 0.72 + sign * 0.07];

	return (
		<group>
			<mesh position={hip} rotation={[0, 0, sign * 0.37]}>
				<boxGeometry args={[0.11, 0.44, 0.11]} />
				<meshStandardMaterial color="#cbd7d3" metalness={0.56} roughness={0.28} />
			</mesh>
			<mesh position={knee} rotation={[0, 0, -sign * 0.18]}>
				<boxGeometry args={[0.09, 0.42, 0.09]} />
				<meshStandardMaterial color="#8ba49d" metalness={0.35} roughness={0.38} />
			</mesh>
			<mesh position={[x, height, depth]} rotation={[0, 0, Math.PI / 2]}>
				<capsuleGeometry args={[0.055, 0.12, 8, 16]} />
				<meshStandardMaterial color="#f1b86c" metalness={0.22} roughness={0.45} />
			</mesh>
			<group position={[x, height + magnitude / 2, depth]}>
				<mesh><cylinderGeometry args={[0.011, 0.011, magnitude, 10]} /><meshBasicMaterial color={index < 2 ? "#80d8c7" : "#9dbbff"} /></mesh>
				<mesh position={[0, magnitude / 2 + 0.025, 0]}><coneGeometry args={[0.04, 0.1, 10]} /><meshBasicMaterial color={index < 2 ? "#80d8c7" : "#9dbbff"} /></mesh>
			</group>
		</group>
	);
}

function Robot({ forces, command, physics }) {
	const robot = useRef();
	useFrame(({ clock }) => {
		if (!physics) return;
		robot.current.position.set(physics.basePosition[0], physics.basePosition[2], -physics.basePosition[1]);
		robot.current.quaternion.set(physics.baseQuaternion[1], physics.baseQuaternion[3], -physics.baseQuaternion[2], physics.baseQuaternion[0]);
	});

	return (
		<group ref={robot}>
			<mesh castShadow><boxGeometry args={[0.82, 0.27, 0.4]} /><meshStandardMaterial color="#e6ece8" metalness={0.58} roughness={0.25} /></mesh>
			<mesh position={[0.03, 0.17, 0]}><boxGeometry args={[0.46, 0.12, 0.3]} /><meshStandardMaterial color="#263b3b" metalness={0.7} roughness={0.2} /></mesh>
			{nominalFeet.map((nominalFoot, index) => {
				const foot = physics
					? [
						physics.footPositions[index][0] - physics.basePosition[0],
						physics.footPositions[index][2] - physics.basePosition[2],
						-(physics.footPositions[index][1] - physics.basePosition[1])
					]
					: nominalFoot;
				return <Leg key={index} foot={foot} index={index} force={forces?.slice(index * 3, index * 3 + 3)} />;
			})}
			<mesh position={[command[0] * 0.5, -0.02, command[1] * 0.5]}><boxGeometry args={[0.82, 0.27, 0.4]} /><meshBasicMaterial color="#61c5e4" transparent opacity={0.18} wireframe /></mesh>
		</group>
	);
}

function ActualMesh({ visual }) {
	const source = useLoader(OBJLoader, `/robots/anymal_c/assets/${visual.meshName}.obj`);
	const definition = meshMaterials[visual.meshName] ?? { color: "#6f7883" };
	const texture = useLoader(TextureLoader, `/robots/anymal_c/assets/${definition.texture ?? "base.png"}`);
	const object = useMemo(() => {
		const clone = source.clone(true);
		texture.colorSpace = SRGBColorSpace;
		clone.traverse((child) => {
			if (child.isMesh) {
				child.material = new MeshStandardMaterial({
					color: definition.color ?? "#ffffff",
					map: definition.texture ? texture : null,
					metalness: 0.28,
					roughness: 0.5
				});
			}
		});
		clone.applyMatrix4(mujocoToThree);
		return clone;
	}, [source, texture, visual.meshName]);
	const rotation = useMemo(() => {
		const matrix = new Matrix4().set(
			...visual.rotation.slice(0, 3), 0,
			...visual.rotation.slice(3, 6), 0,
			...visual.rotation.slice(6, 9), 0,
			0, 0, 0, 1
		);
		return new Quaternion().setFromRotationMatrix(mujocoToThree.clone().multiply(matrix).multiply(threeToMujoco));
	}, [visual.rotation]);
	return (
		<group position={[visual.position[0], visual.position[2], -visual.position[1]]} quaternion={rotation} scale={[visual.scale[0], visual.scale[2], visual.scale[1]]}>
			<primitive object={object} />
		</group>
	);
}

function ActualAnymal({ visuals }) {
	return <group>{visuals.map((visual, index) => <ActualMesh key={`${visual.meshName}-${index}`} visual={visual} />)}</group>;
}

function GroundReactionForces({ physics, forces }) {
	if (!physics || !forces) return null;
	return physics.footPositions.map((foot, index) => {
		const vector = new Vector3(forces[index * 3], forces[index * 3 + 2], -forces[index * 3 + 1]);
		const magnitude = vector.length();
		if (magnitude < 1) return null;
		const direction = vector.normalize();
		const origin = new Vector3(foot[0], foot[2] + 0.015, -foot[1]);
		const length = Math.min(0.65, Math.max(0.12, magnitude / 450));
		return <arrowHelper key={index} args={[direction, origin, length, index < 2 ? "#80d8c7" : "#9dbbff", 0.08, 0.045]} />;
	});
}

function FollowCamera({ physics, resetSignal }) {
	const controls = useRef();
	const target = useRef(new Vector3());
	const lastResetSignal = useRef(-1);
	const { camera, gl } = useThree();

	useFrame(() => {
		if (!physics || !controls.current) return;
		target.current.set(physics.basePosition[0], physics.basePosition[2], -physics.basePosition[1]);
		if (lastResetSignal.current !== resetSignal) {
			lastResetSignal.current = resetSignal;
			controls.current.target.copy(target.current);
			camera.position.copy(target.current).add(new Vector3(2.35, 1.65, 2.7));
		}
		controls.current.target.lerp(target.current, 0.12);
		controls.current.update();
	});

	return <orbitControls ref={controls} args={[camera, gl.domElement]} enableDamping dampingFactor={0.08} enablePan minDistance={1.2} maxDistance={7} maxPolarAngle={Math.PI / 2.05} />;
}

function FollowGrid({ physics }) {
	const grid = useRef();
	useFrame(() => {
		if (!physics || !grid.current) return;
		grid.current.position.set(Math.round(physics.basePosition[0] / 5) * 5, -0.02, -Math.round(physics.basePosition[1] / 5) * 5);
	});
	return <gridHelper ref={grid} args={[100, 100, "#344250", "#202a34"]} />;
}

export default function RobotScene({ forces, command, physics, cameraReset }) {
	return (
		<Canvas shadows camera={{ position: [2.35, 1.65, 2.7], fov: 42 }}>
			<color attach="background" args={["#11171d"]} />
			<ambientLight intensity={1.35} />
			<hemisphereLight args={["#dce9f5", "#10161e", 1.1]} />
			<directionalLight castShadow position={[3, 4, 2]} intensity={2.8} shadow-mapSize={[1024, 1024]} />
			<directionalLight position={[-3, 2, -2]} intensity={0.9} color="#9bbbe8" />
			<FollowGrid physics={physics} />
			<mesh receiveShadow position={[0, -0.04, 0]} rotation={[-Math.PI / 2, 0, 0]}><planeGeometry args={[100, 100]} /><meshStandardMaterial color="#11171d" roughness={0.9} /></mesh>
			<Suspense fallback={<Robot forces={forces} command={command} physics={physics} />}>
				{physics?.visualGeometries?.length ? <ActualAnymal visuals={physics.visualGeometries} /> : <Robot forces={forces} command={command} physics={physics} />}
			</Suspense>
			<GroundReactionForces physics={physics} forces={forces} />
			<FollowCamera physics={physics} resetSignal={cameraReset} />
		</Canvas>
	);
}
