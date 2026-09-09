import { readFile } from "node:fs/promises";
import createModule from "../web/public/wasm/convex_mpc_sim_wasm.js";

const wasmBinary = await readFile(new URL("../web/public/wasm/convex_mpc_sim_wasm.wasm", import.meta.url));
const module = await createModule({ wasmBinary });
const simulation = new module.SimulationCore();
simulation.step(0.15, 0, 0, 20000);
const snapshot = simulation.snapshot();
simulation.delete();

if (snapshot.time < 19.9 || snapshot.position[0] < 1.0 || snapshot.position[2] < 0.3 || snapshot.position[2] > 0.8) {
  throw new Error(`unexpected full WASM result: t=${snapshot.time}, pos=${snapshot.position.join(", ")}`);
}

console.log(`Full WASM smoke test passed: t=${snapshot.time.toFixed(2)} s, x=${snapshot.position[0].toFixed(3)} m`);
