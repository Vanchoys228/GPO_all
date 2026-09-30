// @vitest-environment jsdom
import React, { act } from "react";
import { createRoot } from "react-dom/client";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { usePlannerRouteOptimization } from "./usePlannerRouteOptimization";
import { usePlannerBridgeSync } from "./usePlannerBridgeSync";
import { usePlannerRouteRebuild } from "./usePlannerRouteRebuild";
import { solveRouteWithNativeAlgorithm } from "../../../lib/routeAlgorithms";
import { sendRouteChannelPayload } from "../services/routeChannel";

vi.mock("../../../lib/routeAlgorithms", () => ({ solveRouteWithNativeAlgorithm: vi.fn(), getAlgorithmLabel: () => "GA", getTaskLabel: () => "TSP" }));
vi.mock("../services/routeChannel", () => ({ sendRouteChannelPayload: vi.fn() }));
let root;
beforeEach(() => { globalThis.IS_REACT_ACT_ENVIRONMENT = true; root = createRoot(document.createElement("div")); vi.clearAllMocks(); });
afterEach(async () => { await act(async () => root.unmount()); });

const routeProps = () => ({
  algorithmKey: "ga_tabu", batteryRangeMeters: 100, energyOptions: {}, isOptimizing: false,
  plannerModel: { visitPoints: [{ x: 0, y: 0 }, { x: 1, y: 0 }], previewPolygons: [], surfaceZones: [], chargePoints: [], adjustedVisits: [] },
  routeTaskKey: "tsp", selectedAlgorithmParams: {},
  setEnergyWarning: vi.fn(), setIsOptimizing: vi.fn(), setOptimizedRoute: vi.fn(),
  setRouteEnergyStats: vi.fn(), setRouteSeed: vi.fn(), setStatus: vi.fn(), telemetry: {},
});

describe("planner lifecycle regressions", () => {
  it("aborts a stale solve and does not restore deleted points", async () => {
    const pending = Promise.withResolvers();
    solveRouteWithNativeAlgorithm.mockReturnValue(pending.promise);
    const actions = {};
    function Harness({ config }) {
      const optimize = usePlannerRouteOptimization(config);
      React.useEffect(() => { actions.optimize = optimize; });
      return null;
    }
    const props = routeProps();
    await act(async () => root.render(<Harness config={props} />));
    let completion;
    await act(async () => { completion = actions.optimize(); });
    const signal = solveRouteWithNativeAlgorithm.mock.calls[0][4];
    await act(async () => root.render(<Harness config={{ ...props, plannerModel: { ...props.plannerModel, visitPoints: [] } }} />));
    expect(signal.aborted).toBe(true);
    await act(async () => { pending.resolve({ route: props.plannerModel.visitPoints }); await completion; });
    expect(props.setRouteSeed).not.toHaveBeenCalled();
    expect(props.setOptimizedRoute).not.toHaveBeenCalled();
    expect(props.setIsOptimizing).toHaveBeenLastCalledWith(false);
  });

  it("resends current zones and motion on every connection", async () => {
    const props = { routeSocketRef: { current: null }, routeConnected: false,
      batteryRangeMeters: 100, cruiseSpeedMps: 0.2, payloadKg: 0, setStatus: vi.fn(),
      zoneSyncPayloadText: '{"type":"limit_zones","zones":[]}', surfaceSyncPayloadText: '{"type":"surface_zones","zones":[]}' };
    function Harness({ connected }) { usePlannerBridgeSync({ ...props, routeConnected: connected }); return null; }
    await act(async () => root.render(<Harness connected={false} />));
    expect(sendRouteChannelPayload).not.toHaveBeenCalled();
    await act(async () => root.render(<Harness connected />));
    expect(sendRouteChannelPayload).toHaveBeenCalledTimes(3);
    await act(async () => root.render(<Harness connected={false} />));
    await act(async () => root.render(<Harness connected />));
    expect(sendRouteChannelPayload).toHaveBeenCalledTimes(6);
  });

  it("rebuilds a draft locally without sending a route command", async () => {
    const props = { ...routeProps(), chargePointsRoutingText: "[]", previewPolygonRoutingText: "[]",
      routeSeed: [{x:0,y:0},{x:1,y:0}], surfaceZones: [], zoneSyncPayloadText: '{"zones":[]}',
      surfaceSyncPayloadText: '{"zones":[]}', routeSocketRef: {current:null}, startRouteTiming: vi.fn() };
    function Harness({ config }) { usePlannerRouteRebuild(config); return null; }
    await act(async () => root.render(<Harness config={props} />));
    await act(async () => root.render(<Harness config={{ ...props, batteryRangeMeters: 90 }} />));
    expect(props.setOptimizedRoute).toHaveBeenCalled();
    expect(sendRouteChannelPayload).not.toHaveBeenCalled();
  });
});
