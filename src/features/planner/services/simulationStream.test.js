import {expect,it,vi} from "vitest";
import {connectSimulationStream} from "./simulationStream";

it("negotiates MJPEG, resolves its URL and closes on disposal", () => {
  const socket={send:vi.fn(),close:vi.fn()};
  const onFrame=vi.fn();
  const dispose=connectSimulationStream({url:"ws://localhost:8080/simulation/",createSocket:()=>socket,onFrame,onStatus:vi.fn()});
  socket.onopen();
  expect(socket.send).toHaveBeenCalledWith("mjpeg: 960x540");
  socket.onmessage({data:"multimedia: stream.jpg"});
  expect(onFrame).toHaveBeenCalledWith("http://localhost:8080/simulation/stream.jpg");
  dispose();
  expect(socket.close).toHaveBeenCalledOnce();
});

it("rejects image addresses outside the simulator endpoint", () => {
  const socket={send:vi.fn(),close:vi.fn()};
  const onFrame=vi.fn();
  const dispose=connectSimulationStream({url:"ws://localhost:8080/simulation/",createSocket:()=>socket,onFrame,onStatus:vi.fn()});
  socket.onmessage({data:"multimedia: https://other.invalid/image"});
  expect(onFrame).not.toHaveBeenCalled();
  dispose();
});

it("reconnects after a disconnect and cancels retries when closed", () => {
  vi.useFakeTimers();
  try {
    const sockets=[];
    const createSocket=()=>{const socket={send:vi.fn(),close:vi.fn()};sockets.push(socket);return socket;};
    const dispose=connectSimulationStream({url:"ws://localhost/simulation/",createSocket,onFrame:vi.fn(),onStatus:vi.fn()});
    sockets[0].onclose();
    vi.advanceTimersByTime(2000);
    expect(sockets).toHaveLength(2);
    sockets[1].onclose();dispose();vi.advanceTimersByTime(2000);
    expect(sockets).toHaveLength(2);
  } finally {vi.useRealTimers();}
});
