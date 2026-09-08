// @vitest-environment jsdom
import React, {act} from "react";
import {createRoot} from "react-dom/client";
import {afterEach,expect,it,vi} from "vitest";
import {useMissionTracking} from "./useMissionTracking";
let root;
afterEach(async () => {if(root) await act(async () => root.unmount());});
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
