export async function createSimulationRuntime() {
	const wasmUrl = (path) => new URL(`wasm/${path}`, document.baseURI).toString();
	const { default: createModule } = await import(/* webpackIgnore: true */ wasmUrl("convex_mpc_sim_wasm.js"));
	const module = await createModule({ locateFile: wasmUrl });
	let simulation = new module.SimulationCore();
	const snapshot = () => {
		const state = simulation.snapshot();
		return {
			forces: state.groundReactionForces,
			snapshot: {
				basePosition: state.position,
				baseQuaternion: state.quaternion,
				footPositions: [0, 1, 2, 3].map((leg) => state.footPositions.slice(leg * 3, leg * 3 + 3)),
				visualGeometries: simulation.visualGeometries(),
				time: state.time
			}
		};
	};

	return {
		advance(command, steps = 16) {
			simulation.step(command[0], command[1], command[2], steps);
			return snapshot();
		},
		reset() {
			simulation.reset();
			return snapshot();
		},
		setBodyHeight(height) {
			simulation.setBodyHeight(height);
		},
		setMpcTuning(positionScale, velocityScale, forceScale) {
			simulation.setMpcTuning(positionScale, velocityScale, forceScale);
		},
		dispose() {
			simulation.delete();
		}
	};
}
