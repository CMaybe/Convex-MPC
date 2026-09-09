import { readFile } from "node:fs/promises";
import createModule from "../web/public/wasm/convex_mpc.js";

const wasmBinary = await readFile(new URL("../web/public/wasm/convex_mpc.wasm", import.meta.url));
const module = await createModule({ wasmBinary });
const preview = new module.MpcPreview();
const state = [0, 0, 0, 0, 0, 0.54, 0, 0, 0, 0, 0, 0, -9.81];
const feet = [
  0.35, -0.2, -0.54,
  0.35, 0.2, -0.54,
  -0.35, -0.2, -0.54,
  -0.35, 0.2, -0.54
];
const footVelocities = Array(12).fill(0);
const forces = preview.step(state, feet, footVelocities, [0, 0, 0], [1, 0, 0, 1]);
const prediction = preview.prediction();
const swingForces = preview.swingForces();
const contacts = preview.contacts();

if (forces.length !== 12 || swingForces.length !== 12 || contacts.length !== 4 || prediction.length !== 13) {
  throw new Error(`unexpected WASM result shapes: forces=${forces.length}, swing=${swingForces.length}, contacts=${contacts.length}, prediction=${prediction.length}`);
}
if (![...forces, ...swingForces, ...prediction].every(Number.isFinite)) {
  throw new Error("WASM returned a non-finite controller value");
}

console.log(`WASM smoke test passed: total GRF ${forces.reduce((sum, force) => sum + force, 0).toFixed(2)} N`);
