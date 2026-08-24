import { describe, expect, it } from "vitest";
import nativeSolverModule from "./native-solver.cjs";

const { createNativeSolver } = nativeSolverModule;
const solver = createNativeSolver({ solverPath: "/missing/solver" });

describe("native solver adapter", () => {
  it("builds the existing stdin contract", () => {
    expect(
      solver.buildInput({
        points: [
          { x: 1, y: 2 },
          { x: 3, y: 4 },
        ],
        algorithmKey: "ga_tabu",
        taskKey: "tsp",
        params: { nests: 90, pa: 0.18, max_iter: 260, alpha: 0.9, beta: 24 },
        seed: 1337,
      })
    ).toBe(
      "task tsp\n" +
        "algorithm ga_tabu\n" +
        "seed 1337\n" +
        "params 90 0.18 260 0.9 24\n" +
        "count 2\n" +
        "1 2\n" +
        "3 4\n"
    );
  });

  it("parses a successful solver response", () => {
    expect(
      solver.parseOutput(
        "status ok\nclosed 1\nlength 4.5\norder 0 1\nroute_count 2\n1 2\n3 4\n"
      )
    ).toEqual({
      closed: true,
      length: 4.5,
      order: [0, 1],
      route: [
        { x: 1, y: 2 },
        { x: 3, y: 4 },
      ],
    });
  });

  it("surfaces solver protocol errors", () => {
    expect(() => solver.parseOutput("status error\nmessage invalid input\n")).toThrow(
      "invalid input"
    );
  });
});
