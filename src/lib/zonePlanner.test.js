import { describe, expect, it } from "vitest";
import { sanitizeRouteForController } from "./zonePlanner";

describe("sanitizeRouteForController", () => {
  it("preserves the final return to the start for tsp routes", () => {
    const route = [
      { x: 0, y: 0 },
      { x: 2, y: 0 },
      { x: 2, y: 2 },
      { x: 0, y: 0 },
    ];

    expect(sanitizeRouteForController(route)).toEqual(route);
  });
});
