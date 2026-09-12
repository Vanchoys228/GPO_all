import { describe, expect, it, vi } from "vitest";
import gateway from "./gateway-service.cjs";

const setup = () => {
  const records = new Map();
  const repository = { get: async id => records.get(id), save: async r => records.set(r.missionId, r) };
  const adapter = { getFeedback:vi.fn(async()=>null), submit: vi.fn(async () => {}), update: vi.fn(async () => true), cancel: vi.fn(async () => {}) };
  return { repository, adapter, service: gateway.createGatewayService({ repository, adapter }) };
};
describe("gateway delivery journal", () => {
  it("does not redeliver after a lost reply or gateway restart", async () => {
    const { repository, adapter, service } = setup();
    const payload = { commandId: "m1", route: [{x:0,y:0},{x:1,y:0}] };
    await service.execute("submit", "m1", payload);
    await gateway.createGatewayService({ repository, adapter }).execute("submit", "m1", payload);
    expect(adapter.submit).toHaveBeenCalledOnce();
    await expect(service.execute("submit", "m1", {...payload,route:[]})).rejects.toMatchObject({statusCode:409});
  });
  it("retries an uncertain write with the same command identity", async () => {
    const { adapter, service } = setup();
    adapter.submit.mockRejectedValueOnce(new Error("unavailable"));
    await expect(service.execute("submit", "m2", {commandId:"m2"})).rejects.toThrow("unavailable");
    await service.execute("submit", "m2", {commandId:"m2"});
    expect(adapter.submit.mock.calls.map(([p]) => p.commandId)).toEqual(["m2","m2"]);
  });
});
