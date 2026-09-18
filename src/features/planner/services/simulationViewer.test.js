import {afterEach,expect,it,vi} from "vitest";
import {connectSimulationViewer} from "./simulationViewer";

afterEach(()=>vi.useRealTimers());
function setup() {
  vi.useFakeTimers();
  const listeners=new Map();
  const host={location:{origin:"http://localhost"},addEventListener:(type,fn)=>listeners.set(type,fn),removeEventListener:(type)=>listeners.delete(type)};
  const frame={contentWindow:{postMessage:vi.fn()},src:""};
  const onStatus=vi.fn(),onMode=vi.fn(),onError=vi.fn();
  let sequence=0;
  const dispose=connectSimulationViewer({frame,onStatus,onMode,onError,host,newSession:()=>String(++sequence)});
  const receive=(data,overrides={})=>listeners.get("message")?.({origin:host.location.origin,source:frame.contentWindow,data:{channel:"gpo-w3d",session:String(sequence),...data},...overrides});
  return {frame,host,onStatus,onMode,dispose,receive,listeners};
}
it("trusts only its own same-origin frame and confirms speed using simulator feedback",async()=>{
  const s=setup();
  s.receive({type:"ready"},{origin:"https://other.invalid"});
  s.receive({type:"ready"},{source:{}});
  await expect(s.dispose.setMode("fast")).rejects.toThrow();
  s.receive({type:"ready"});s.receive({type:"mode",mode:"realtime"});
  const pending=s.dispose.setMode("fast");
  expect(s.onMode).toHaveBeenLastCalledWith("realtime");
  expect(s.frame.contentWindow.postMessage).toHaveBeenCalledWith({channel:"gpo-w3d",session:"1",type:"set-mode",mode:"fast"},"http://localhost");
  s.receive({type:"mode",mode:"fast"});
  await expect(pending).resolves.toBe("fast");
  await expect(s.dispose.setMode("reset")).rejects.toThrow();
  s.dispose();expect(s.listeners.size).toBe(0);
});
it("reloads an isolated viewer on disconnect and ignores stale session messages",async()=>{
  const s=setup();s.receive({type:"ready"});
  const before=new URL(s.frame.src,s.host.location.origin);
  const rejected=expect(s.dispose.setMode("fast")).rejects.toThrow();
  s.receive({type:"disconnected"});await rejected;
  vi.advanceTimersByTime(2000);
  const after=new URL(s.frame.src,s.host.location.origin);
  // A fragment-only navigation keeps the dead viewer's document and WebGL state.
  expect(after.pathname+after.search).not.toBe(before.pathname+before.search);
  expect(after.searchParams.get("session")).toBe("2");
  s.receive({type:"ready",session:"1"});
  await expect(s.dispose.setMode("fast")).rejects.toThrow();
  expect(s.frame.contentWindow.postMessage).toHaveBeenCalledTimes(1);
  s.dispose();vi.advanceTimersByTime(60000);
  expect(s.frame.src).toBe("/webots/viewer.html?session=2");
});
it("reports a missing acknowledgement instead of falsely changing the selected mode",async()=>{
  const s=setup();s.receive({type:"ready"});
  const rejected=expect(s.dispose.setMode("fast")).rejects.toThrow(/подтвердил/);
  vi.advanceTimersByTime(5000);await rejected;
  expect(s.onMode).not.toHaveBeenCalledWith("fast");s.dispose();
});
