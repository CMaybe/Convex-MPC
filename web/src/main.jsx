import React, { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import RobotScene from "./RobotScene";
import { createSimulationRuntime } from "./simulationRuntime";
import "./styles.css";

function App() {
	const commandLimits = { linear: 1.2, lateral: 0.6, yaw: 1.0 };
	const [runtimeStatus, setRuntimeStatus] = useState("loading simulation WASM");
	const [physicsStatus, setPhysicsStatus] = useState("MuJoCo core pending");
	const [command, setCommand] = useState([0, 0, 0]);
	const [forces, setForces] = useState(null);
	const [physics, setPhysics] = useState(null);
	const [simulationTime, setSimulationTime] = useState(0);
	const [cameraReset, setCameraReset] = useState(0);
	const [bodyHeight, setBodyHeight] = useState(0.5);
	const [mpcTuning, setMpcTuning] = useState({ position: 1, velocity: 1, force: 1 });
	const simulationRef = useRef(null);
	const commandRef = useRef(command);

	useEffect(() => {
		let cancelled = false;
		let runtime;
		createSimulationRuntime()
			.then((loaded) => {
				runtime = loaded;
				simulationRef.current = loaded;
				if (!cancelled) {
					setRuntimeStatus("C++ simulation WASM ready");
					setPhysicsStatus("MuJoCo C++ core ready");
				}
			})
			.catch((error) => {
				console.error("C++ simulation initialization failed", error);
				if (!cancelled) {
					setRuntimeStatus("C++ simulation WASM error");
					setPhysicsStatus(error.message);
				}
			});
		return () => {
			cancelled = true;
			simulationRef.current = null;
			runtime?.dispose();
		};
	}, []);

	useEffect(() => {
		let frame;
		let active = true;
		let previousTime;
		let accumulatedTime = 0;
		const animate = (now) => {
			if (previousTime !== undefined) accumulatedTime += Math.min((now - previousTime) / 1000, 0.05);
			previousTime = now;
			if (active && simulationRef.current) {
				try {
					const availableSteps = Math.floor(accumulatedTime / 0.001);
					const steps = Math.min(availableSteps, 20);
					if (steps > 0) {
						const result = simulationRef.current.advance(commandRef.current, steps);
						accumulatedTime -= steps * 0.001;
						setForces(result.forces);
						setPhysics(result.snapshot);
						setSimulationTime(result.snapshot.time);
					}
				} catch (error) {
					console.error("C++ simulation step failed", error);
					setPhysicsStatus("MuJoCo C++ controller error");
					return;
				}
			}
			frame = requestAnimationFrame(animate);
		};
		frame = requestAnimationFrame(animate);
		return () => {
			active = false;
			cancelAnimationFrame(frame);
		};
	}, []);

	const updateCommand = (axis, value) => {
		setCommand((current) => {
			const next = current.map((entry, index) => index === axis ? Number(value) : entry);
			commandRef.current = next;
			return next;
		});
	};
	const forceTotal = forces?.reduce((total, value) => total + value, 0);
	const resetSimulation = () => {
		const zeroCommand = [0, 0, 0];
		commandRef.current = zeroCommand;
		setCommand(zeroCommand);
		if (!simulationRef.current) return;
		const result = simulationRef.current.reset();
		simulationRef.current.setBodyHeight(bodyHeight);
		simulationRef.current.setMpcTuning(mpcTuning.position, mpcTuning.velocity, mpcTuning.force);
		setForces(result.forces);
		setPhysics(result.snapshot);
		setSimulationTime(result.snapshot.time);
		setCameraReset((value) => value + 1);
	};
	const updateHeight = (value) => {
		const height = Number(value);
		setBodyHeight(height);
		simulationRef.current?.setBodyHeight(height);
	};
	const updateTuning = (key, value) => {
		setMpcTuning((current) => {
			const next = { ...current, [key]: Number(value) };
			simulationRef.current?.setMpcTuning(next.position, next.velocity, next.force);
			return next;
		});
	};

	return (
		<main className="shell">
			<header className="app-header">
				<div className="app-header__row">
					<a className="app-header__brand" href="https://cmaybe.github.io/">Convex MPC</a>
					<nav className="app-header__nav" aria-label="Project navigation">
						<a href="https://github.com/CMaybe/Convex-MPC" target="_blank" rel="noreferrer">GitHub</a>
						<a href="https://github.com/CMaybe/Convex-MPC#readme" target="_blank" rel="noreferrer">Docs</a>
					</nav>
				</div>
			</header>
			<section className="workspace">
				<div className="viewport" aria-label="ANYmal C 3D simulation viewport"><RobotScene forces={forces} command={command} physics={physics} cameraReset={cameraReset} /></div>
				<aside className="control-panel">
					<p className="panel-label">Command velocity</p>
					<h1>Convex MPC</h1>
					<button className="reset-button" type="button" onClick={resetSimulation}>Reset position</button>
					<label className="control"><span>Body height<output>{bodyHeight.toFixed(2)} <small>m</small></output></span><input type="range" min="0.35" max="0.75" step="0.01" value={bodyHeight} onChange={(event) => updateHeight(event.target.value)} /></label>
					{[["Forward", "m/s", commandLimits.linear], ["Lateral", "m/s", commandLimits.lateral], ["Yaw", "rad/s", commandLimits.yaw]].map(([label, unit, limit], axis) => (
						<label className="control" key={label}>
							<span>{label}<output>{command[axis].toFixed(2)} <small>{unit}</small></output></span>
							<input type="range" min={-limit} max={limit} step="0.05" value={Math.max(-limit, Math.min(limit, command[axis]))} onChange={(event) => updateCommand(axis, event.target.value)} />
						</label>
					))}
					<p className="panel-label tuning-label">MPC tuning</p>
					{[["position", "Position tracking"], ["velocity", "Velocity tracking"], ["force", "Force effort"]].map(([key, label]) => (
						<label className="control" key={key}><span>{label}<output>{mpcTuning[key].toFixed(1)}x</output></span><input type="range" min="0.2" max="3" step="0.1" value={mpcTuning[key]} onChange={(event) => updateTuning(key, event.target.value)} /></label>
					))}
					<div className="legend"><i className="cyan" />Front GRF <i className="blue" />Rear GRF</div>
				</aside>
			</section>
			<footer className="readout">
				<span>Controller: qpOASES C++</span>
				<span>Physics: {physicsStatus}</span>
				<span>Renderer: Three.js</span>
				<span>Sim: {simulationTime.toFixed(2)} s</span>
				<span>X: {physics?.basePosition[0].toFixed(2) ?? "--"} m</span>
				<span>GRF sum: {forceTotal?.toFixed(1) ?? "--"} N</span>
			</footer>
		</main>
	);
}

createRoot(document.getElementById("root")).render(<App />);
