import { describe, expect, it } from "vitest";
import solverValidation from "./solver-validation.cjs";

const { resolveAlgorithmKey, resolveTaskKey, sanitizeNativeParams } = solverValidation;

describe("solver protocol validation", () => {
  it("keeps legacy algorithm aliases compatible", () => {
    expect(resolveAlgorithmKey("genetik")).toBe("ga_tabu");
    expect(resolveAlgorithmKey("annealing")).toBe("otshig");
    expect(resolveAlgorithmKey("scatter")).toBe("rasseivanie");
  });

  it("falls back to supported task and parameter ranges", () => {
    expect(resolveTaskKey("hamiltonian_chain")).toBe("hamiltonian_chain");
    expect(resolveTaskKey("unknown")).toBe("tsp");
    expect(sanitizeNativeParams("ga_tabu", {
      population_size: 99999,
      mutation_rate: -10,
      generations: 0,
    })).toMatchObject({ nests: 1200, pa: 0, max_iter: 1 });
  });
});
