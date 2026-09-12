import { expect, it, vi } from "vitest";
import { EventEmitter } from "node:events";
import shutdown from "./shutdown.cjs";
it("drains once on repeated signals and preserves a previous failure", async () => {
  const runtime = new EventEmitter(); runtime.exitCode = 1;
  const close = vi.fn(async () => {});
  const dispose = shutdown.installShutdown(close, { runtime });
  runtime.emit("SIGTERM"); runtime.emit("SIGINT");
  await new Promise(resolve => setImmediate(resolve));
  expect(close).toHaveBeenCalledTimes(1);
  expect(runtime.exitCode).toBe(1);
  dispose();
});
it("forces a failed exit when draining exceeds the deadline", async () => {
  const runtime = new EventEmitter(); runtime.exit = vi.fn();
  const dispose = shutdown.installShutdown(() => new Promise(() => {}), { runtime, timeoutMs: 10 });
  runtime.emit("SIGTERM");
  await new Promise(resolve => setTimeout(resolve, 30));
  expect(runtime.exit).toHaveBeenCalledWith(1);
  dispose();
});
