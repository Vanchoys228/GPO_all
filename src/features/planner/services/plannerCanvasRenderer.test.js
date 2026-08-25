import { describe, expect, it, vi } from "vitest";
import { drawDiamond } from "./plannerCanvasRenderer";

describe("planner canvas renderer", () => {
  it("draws a closed diamond path", () => {
    const ctx = {
      beginPath: vi.fn(),
      closePath: vi.fn(),
      lineTo: vi.fn(),
      moveTo: vi.fn(),
    };
    drawDiamond(ctx, 10, 20, 4);
    expect(ctx.moveTo).toHaveBeenCalledWith(10, 16);
    expect(ctx.lineTo).toHaveBeenCalledTimes(3);
    expect(ctx.closePath).toHaveBeenCalledOnce();
  });
});
