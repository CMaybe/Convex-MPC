async function loadAsset(vfs, path) {
	const response = await fetch(`/robots/anymal_c/${path}`);
	if (!response.ok) throw new Error(`failed to load ${path}`);
	vfs.addBuffer(path, new Uint8Array(await response.arrayBuffer()));
}

const footNames = ["LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"];

function eulerFromQuaternion([w, x, y, z]) {
	return [
		Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
		Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
		Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
	];
}

export async function createMujocoRuntime() {
	const { default: loadMujoco } = await import(/* webpackIgnore: true */ "/mujoco/mujoco.js");
	const module = await loadMujoco({ locateFile: (file) => `/mujoco/${file}` });
	const vfs = new module.MjVFS();
	const assetPaths = await (await fetch("/robots/anymal_c/assets.json")).json();
	await Promise.all(assetPaths.map((path) => loadAsset(vfs, path)));
	const model = module.MjModel.from_xml_path("scene.xml", vfs);
	const data = new module.MjData(model);
	module.mj_resetDataKeyframe(model, data, model.key("standing").id);
	module.mj_forward(model, data);
	const siteIds = footNames.map((name) => model.site(name).id);
	const settleQpos = Array.from(data.qpos);
	let steps = 0;
	let forces = Array(12).fill(0);
	let swingForces = Array(12).fill(0);
	let contacts = [1, 0, 0, 1];
	const snapshot = () => ({
		basePosition: Array.from(data.body("base").xpos),
		baseQuaternion: Array.from(data.body("base").xquat),
		footPositions: footNames.map((name) => Array.from(data.site(name).xpos)),
		time: data.time
	});
	const advance = (preview, command, substeps = 4) => {
		for (let step = 0; step < substeps; step += 1) {
			const qpos = Array.from(data.qpos);
			const qvel = Array.from(data.qvel);
			if (steps < 700) {
				for (let joint = 0; joint < 12; joint += 1)
					data.ctrl[joint] = 100 * (settleQpos[7 + joint] - qpos[7 + joint]) - 5 * qvel[6 + joint] + data.qfrc_bias[6 + joint];
				module.mj_step(model, data);
				steps += 1;
				continue;
			}
			const basePosition = qpos.slice(0, 3);
			const quaternion = qpos.slice(3, 7);
			const footPositions = footNames.flatMap((name) => Array.from(data.site(name).xpos).map((value, axis) => value - basePosition[axis]));
			const jacobians = siteIds.map((siteId) => {
				const jacobian = new Float64Array(3 * model.nv);
				module.mj_jacSite(model, data, jacobian, null, siteId);
				return jacobian;
			});
			const footVelocities = jacobians.flatMap((jacobian, leg) => [0, 1, 2].map((axis) => {
				let velocity = 0;
				for (let joint = 0; joint < 3; joint += 1) velocity += jacobian[axis * model.nv + 6 + leg * 3 + joint] * qvel[6 + leg * 3 + joint];
				return velocity;
			}));
			const state = [...eulerFromQuaternion(quaternion), ...basePosition, ...qvel.slice(3, 6), ...qvel.slice(0, 3), -9.81];
			if (steps % 20 === 0) {
				forces = preview.step(state, footPositions, footVelocities, command, contacts);
				swingForces = preview.swingForces();
				contacts = preview.contacts();
			} else {
				swingForces = preview.swingStep(state, footPositions, footVelocities, command, contacts);
				contacts = preview.contacts();
			}
			for (let leg = 0; leg < 4; leg += 1) {
				const jacobian = jacobians[leg];
				for (let joint = 0; joint < 3; joint += 1) {
					let torque = 0;
					for (let axis = 0; axis < 3; axis += 1) {
						const force = contacts[leg] ? -forces[leg * 3 + axis] : swingForces[leg * 3 + axis];
						torque += jacobian[axis * model.nv + 6 + leg * 3 + joint] * force;
					}
					data.ctrl[leg * 3 + joint] = torque + data.qfrc_bias[6 + leg * 3 + joint];
				}
			}
			module.mj_step(model, data);
			steps += 1;
		}
		return { forces, snapshot: snapshot() };
	};
	return { module, model, data, vfs, snapshot, advance };
}
