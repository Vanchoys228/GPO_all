import { afterEach, describe, expect, it, vi } from "vitest";
import { mkdtemp, rm } from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import missions from "./mission-service.cjs";
import repositories from "../repositories/mission-repository.cjs";
import sceneValidation from "../protocol/scene-validation.cjs";
const {normalizeScene,sceneRevision}=sceneValidation;
const directories = [];
afterEach(async () => { for (const directory of directories.splice(0)) await rm(directory,{recursive:true,force:true}); });
const setup = async () => {
  const directory = await mkdtemp(path.join(os.tmpdir(),"gpo-missions-")); directories.push(directory);
  const repository = repositories.createMissionRepository({directory});
  const adapter = { submit:vi.fn(async () => {}), cancel:vi.fn(async () => {}), update:vi.fn(async () => {}), getFeedback:vi.fn(async () => null) };
  return { repository, adapter, service:missions.createMissionService({repository,adapter}) };
};
const payload = { type:"route", route:[{x:0,y:0},{x:1,y:0}] };
const transferPayload = (() => {
  const scene=normalizeScene({polygons:[],surfaceZones:[],chargingStations:[],motion:{}});
  return {type:"transfer_object",objectId:"demo-box",destination:{x:4,y:-2},scene,sceneRevision:sceneRevision(scene)};
})();
describe("mission application service", () => {
  it("persists a submission and makes the same request idempotent across restart", async () => {
    const {repository,adapter,service} = await setup();
    const first = await service.submit(payload,{requestId:"mission-1"});
    const restarted = missions.createMissionService({repository,adapter});
    const second = await restarted.submit(payload,{requestId:"mission-1"});
    expect(first.status).toBe("persisted");
    expect(second.missionId).toBe(first.missionId);
    expect(adapter.submit).toHaveBeenCalledOnce();
  });
  it("rejects reuse of an id for different commands", async () => {
    const {service} = await setup();
    await service.submit(payload,{requestId:"mission-2"});
    await expect(service.submit({...payload,route:[{x:0,y:0},{x:2,y:0}]},{requestId:"mission-2"})).rejects.toMatchObject({statusCode:409});
  });
  it("does not report persistence when the adapter fails and retries with the same command id", async () => {
    const {service,adapter} = await setup();
    adapter.submit.mockRejectedValueOnce(new Error("disk full"));
    await expect(service.submit(payload,{requestId:"mission-3"})).rejects.toThrow("disk full");
    await service.submit(payload,{requestId:"mission-3"});
    expect(adapter.submit.mock.calls.map(([command]) => command.commandId)).toEqual(["mission-3","mission-3"]);
  });
  it("uses only matching controller feedback and preserves terminal states", async () => {
    const {service,adapter} = await setup();
    await service.submit(payload,{requestId:"mission-4"});
    adapter.getFeedback.mockResolvedValue({missionId:"other",status:"completed"});
    expect((await service.get("mission-4")).status).toBe("persisted");
    adapter.getFeedback.mockResolvedValue({missionId:"mission-4",status:"completed"});
    expect((await service.get("mission-4")).status).toBe("completed");
    adapter.getFeedback.mockResolvedValue({missionId:"mission-4",status:"running"});
    expect((await service.get("mission-4")).status).toBe("completed");
  });
  it("persists object transfer without requiring route points", async () => {
    const {service,adapter}=await setup();
    const mission=await service.submit(transferPayload,{requestId:"transfer-1"});
    expect(mission).toMatchObject({missionId:"transfer-1",operationType:"object_transfer",status:"persisted"});
    expect(adapter.submit).toHaveBeenCalledWith(expect.objectContaining({type:"transfer_object",commandId:"transfer-1"}));
  });
  it("keeps recovery holding nonterminal and resumes with one persisted control request", async () => {
    const {service,adapter}=await setup();
    await service.submit(transferPayload,{requestId:"transfer-2"});
    adapter.getFeedback.mockResolvedValue({missionId:"transfer-2",status:"holding_for_recovery",stage:"placing",progress:78,errorCode:"unsafe_release",attached:true});
    expect((await service.get("transfer-2")).status).toBe("holding_for_recovery");
    await expect(service.submit(payload,{requestId:"blocked-route"})).rejects.toMatchObject({statusCode:409});
    expect((await service.resume("transfer-2")).status).toBe("persisted");
    await service.resume("transfer-2");
    expect(adapter.update).toHaveBeenCalledOnce();
    expect(adapter.update).toHaveBeenCalledWith(expect.objectContaining({type:"resume_transfer",missionId:"transfer-2"}),expect.any(String));
    adapter.getFeedback.mockResolvedValue({missionId:"transfer-2",status:"running",stage:"transporting",progress:60,attached:true});
    expect((await service.get("transfer-2")).status).toBe("running");
  });
  it("moves a cancelled carrying transfer into recovery holding", async () => {
    const {service,adapter}=await setup();
    await service.submit(transferPayload,{requestId:"transfer-cancel-hold"});
    adapter.getFeedback.mockResolvedValue({missionId:"transfer-cancel-hold",status:"running",stage:"transporting",attached:true});
    await service.get("transfer-cancel-hold");
    adapter.getFeedback.mockResolvedValue({missionId:"transfer-cancel-hold",status:"holding_for_recovery",stage:"transporting",errorCode:"unsafe_release",attached:true});
    expect((await service.cancel("transfer-cancel-hold")).status).toBe("holding_for_recovery");
  });
  it("does not allow resume for a route mission", async () => {
    const {service}=await setup();
    await service.submit(payload,{requestId:"route-no-resume"});
    await expect(service.resume("route-no-resume")).rejects.toMatchObject({statusCode:409});
  });
});

it("reconciles prepared delivery after restart without a browser", async () => {
  const {repository,adapter,service}=await setup();
  adapter.submit.mockRejectedValueOnce(new Error("offline"));
  await expect(service.submit(payload,{requestId:"recover"})).rejects.toThrow();
  const restarted=missions.createMissionService({repository,adapter});
  await restarted.reconcile();
  expect((await repository.get("recover")).status).toBe("persisted");
  adapter.getFeedback.mockResolvedValue({missionId:"recover",status:"completed"});
  await restarted.reconcile();
  expect((await repository.get("recover")).status).toBe("completed");
});
it("rejects replacement until cancellation is acknowledged by the controller", async () => {
  const {repository,adapter,service}=await setup();
  await service.submit(payload,{requestId:"active"});
  await expect(service.submit(payload,{requestId:"next"})).rejects.toMatchObject({statusCode:409});
  expect((await service.cancel("active")).status).toBe("cancelling");
  await expect(service.submit(payload,{requestId:"next"})).rejects.toMatchObject({statusCode:409});
  adapter.getFeedback.mockResolvedValue({missionId:"active",status:"cancelled"});
  await service.reconcile();
  expect((await repository.get("active")).status).toBe("cancelled");
  await service.submit(payload,{requestId:"next"});
});
