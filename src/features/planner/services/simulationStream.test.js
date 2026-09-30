import {expect,it,vi} from "vitest";
import {connectSimulationStream} from "./simulationStream";

it("changes speed only on request and reports server-confirmed mode", async () => {
  const socket={send:vi.fn(),close:vi.fn(),readyState:1};
  const onMode=vi.fn();
  const dispose=connectSimulationStream({url:"ws://localhost/simulation/",createSocket:()=>socket,onScene:vi.fn(),onStatus:vi.fn(),onMode});
  socket.onmessage({data:"scene load completed"});
  socket.onmessage({data:"real-time"});
  expect(onMode).toHaveBeenLastCalledWith("realtime");
  const pending=dispose.setMode("fast");
  expect(socket.send).toHaveBeenLastCalledWith("fast:-1");
  expect(onMode).toHaveBeenLastCalledWith("realtime");
  socket.onmessage({data:"fast"});
  await expect(pending).resolves.toBe("fast");
  expect(onMode).toHaveBeenLastCalledWith("fast");
  const normal=dispose.setMode("realtime");
  expect(socket.send).toHaveBeenLastCalledWith("real-time:-1");
  socket.onmessage({data:"real-time"});
  await expect(normal).resolves.toBe("realtime");
  await expect(dispose.setMode("reset")).rejects.toThrow();
  dispose();
});

it("rejects pending commands on disconnect and does not replay them on reconnect", async () => {
  vi.useFakeTimers();
  try {
    const sockets=[];
    const onMode=vi.fn();
    const dispose=connectSimulationStream({url:"ws://localhost/simulation/",createSocket:()=>{const s={send:vi.fn(),close:vi.fn(),readyState:1};sockets.push(s);return s;},onScene:vi.fn(),onStatus:vi.fn(),onMode});
    await expect(dispose.setMode("fast")).rejects.toThrow();
    sockets[0].onmessage({data:"scene load completed"});
    const rejected=expect(dispose.setMode("fast")).rejects.toThrow();
    sockets[0].onclose();
    await rejected;
    expect(onMode).toHaveBeenLastCalledWith(null);
    vi.advanceTimersByTime(2000);
    sockets[1].onopen();
    expect(sockets[1].send.mock.calls).toEqual([["w3d;broadcast"]]);
    dispose();
  } finally {vi.useRealTimers();}
});

it("times out an unconfirmed speed change", async () => {
  vi.useFakeTimers();
  try {
    const socket={send:vi.fn(),close:vi.fn(),readyState:1};
    const dispose=connectSimulationStream({url:"ws://localhost/simulation/",createSocket:()=>socket,onScene:vi.fn(),onStatus:vi.fn()});
    socket.onmessage({data:"scene load completed"});
  socket.onmessage({data:"real-time"});
    const rejected=expect(dispose.setMode("fast")).rejects.toThrow(/подтвердил/);
    vi.advanceTimersByTime(5000);
    await rejected;
    dispose();
  } finally {vi.useRealTimers();}
});

it("negotiates W3D, delivers the scene and closes on disposal", () => {
  const socket={send:vi.fn(),close:vi.fn()};
  const onScene=vi.fn();
  const dispose=connectSimulationStream({url:"ws://localhost:8080/simulation/",createSocket:()=>socket,onScene,onStatus:vi.fn()});
  socket.onopen();
  expect(socket.send).toHaveBeenCalledWith("w3d;broadcast");
  socket.onmessage({data:"model:<nodes/>"});
  expect(onScene).toHaveBeenCalledWith("<nodes/>");
  dispose();
  expect(socket.close).toHaveBeenCalledOnce();
});

it("rejects image addresses outside the simulator endpoint", () => {
  const socket={send:vi.fn(),close:vi.fn()};
  const onScene=vi.fn();
  const dispose=connectSimulationStream({url:"ws://localhost:8080/simulation/",createSocket:()=>socket,onScene,onStatus:vi.fn()});
  socket.onmessage({data:"multimedia: https://other.invalid/image"});
  expect(onScene).not.toHaveBeenCalled();
  dispose();
});

it("reconnects after a disconnect and cancels retries when closed", () => {
  vi.useFakeTimers();
  try {
    const sockets=[];
    const createSocket=()=>{const socket={send:vi.fn(),close:vi.fn()};sockets.push(socket);return socket;};
    const dispose=connectSimulationStream({url:"ws://localhost/simulation/",createSocket,onScene:vi.fn(),onStatus:vi.fn()});
    sockets[0].onclose();
    vi.advanceTimersByTime(2000);
    expect(sockets).toHaveLength(2);
    sockets[1].onclose();dispose();vi.advanceTimersByTime(2000);
    expect(sockets).toHaveLength(2);
  } finally {vi.useRealTimers();}
});
