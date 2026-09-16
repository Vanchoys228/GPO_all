// @vitest-environment jsdom
import React, {act} from "react";
import {createRoot} from "react-dom/client";
import {afterEach,expect,it,vi} from "vitest";
import * as missionClient from "../services/missionClient";
import {useMissionTracking} from "./useMissionTracking";
vi.mock("../services/missionClient", () => ({
  getMission:vi.fn(),
  listMissions:vi.fn(async () => []),
  cancelMission:vi.fn(),
}));
let root;
afterEach(async () => {if(root) await act(async () => root.unmount());root=null;vi.clearAllMocks();vi.useRealTimers();});
it("shows execution only for the matching mission and starts timing after controller feedback", async () => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  const fetchMission = vi.fn(async () => ({missionId:"mission-1",status:"running"}));
  const onStarted = vi.fn();
  const exposed = {};
  function Harness() {
    const mission = useMissionTracking({fetchMission,onStarted});
    React.useEffect(() => {exposed.mission = mission;});
    return <span>{mission.state?.status}</span>;
  }
  const element = document.createElement("div");root=createRoot(element);
  await act(async () => root.render(<Harness/>));
  expect(onStarted).not.toHaveBeenCalled();
  await act(async () => exposed.mission.track({missionId:"mission-1",status:"persisted"}));
  expect(element.textContent).toBe("running");
  expect(onStarted).toHaveBeenCalledOnce();
});

it("discovers a new active mission after the displayed mission becomes terminal", async () => {
  globalThis.IS_REACT_ACT_ENVIRONMENT = true;
  vi.useFakeTimers();
  missionClient.getMission.mockResolvedValue({missionId:"first",status:"completed"});
  missionClient.listMissions.mockResolvedValue([]);
  const exposed = {};
  function Harness() {
    const mission = useMissionTracking({});
    React.useEffect(() => {exposed.mission=mission;});
    return null;
  }
  root=createRoot(document.createElement("div"));
  await act(async () => root.render(<Harness/>));
  await act(async () => exposed.mission.track({missionId:"first",status:"persisted"}));
  expect(exposed.mission.state).toMatchObject({missionId:"first",status:"completed"});
  missionClient.listMissions.mockResolvedValue([{missionId:"second",status:"persisted"}]);
  await act(async () => vi.advanceTimersByTimeAsync(1000));
  expect(exposed.mission.state).toMatchObject({missionId:"second",status:"persisted"});
});
