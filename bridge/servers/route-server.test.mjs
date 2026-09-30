import WebSocket from "ws";
import { describe, expect, it, vi } from "vitest";
import routeServerModule from "./route-server.cjs";

const { createRouteServer } = routeServerModule;

const waitForOpen = (socket) =>
  new Promise((resolve, reject) => {
    socket.once("open", resolve);
    socket.once("error", reject);
  });

const waitForMessage = (socket) =>
  new Promise((resolve, reject) => {
    socket.once("message", (data) => resolve(data.toString()));
    socket.once("error", reject);
  });

describe("route WebSocket server", () => {
  it("rejects browser origins outside the local UI allowlist", async () => {
    const server = createRouteServer({artifactStore:{},host:"127.0.0.1",port:0});
    await new Promise(resolve=>server.wss.once("listening",resolve));
    const socket = new WebSocket(`ws://127.0.0.1:${server.wss.address().port}/ui`, {origin:"https://untrusted.example"});
    try {
      const outcome = await new Promise(resolve=>{socket.once("open",()=>resolve("open"));socket.once("error",()=>resolve("rejected"));});
      expect(outcome).toBe("rejected");
    } finally { socket.terminate(); await server.close(); }
  });
  it("returns a matched rejection and never forwards a failed write", async () => {
    const server = createRouteServer({artifactStore:{writeRoute:async()=>{throw new Error("invalid route");}},host:"127.0.0.1",port:0});
    await new Promise(resolve=>server.wss.once("listening",resolve));
    const url = `ws://127.0.0.1:${server.wss.address().port}`;
    const controller = new WebSocket(url);
    const ui = new WebSocket(`${url}/ui`);
    try {
      await Promise.all([waitForOpen(controller),waitForOpen(ui)]);
      const messages=[];controller.on("message",message=>messages.push(message));
      const reply=waitForMessage(ui);
      ui.send(JSON.stringify({type:"route",requestId:"bad-1",route:[]}));
      const result=await Promise.race([reply,new Promise(resolve=>setTimeout(()=>resolve("{}"),150))]);
      expect(JSON.parse(result)).toMatchObject({type:"route.ack",requestId:"bad-1",ok:false});
      expect(messages).toHaveLength(0);
    } finally {controller.terminate();ui.terminate();await server.close();}
  });
  it("persists UI messages and forwards them to the controller", async () => {
    const artifactStore = {
      writeRoute: vi.fn(async () => {}),
      writeLimitZones: vi.fn(async () => {}),
      writeSurfaceZones: vi.fn(async () => {}),
      writeMotionProfile: vi.fn(async () => {}),
      writeRuntimeCommand: vi.fn(async () => {}),
    };
    const server = createRouteServer({ artifactStore, host: "127.0.0.1", port: 0 });
    await new Promise((resolve) => server.wss.once("listening", resolve));
    const port = server.wss.address().port;
    const controller = new WebSocket(`ws://127.0.0.1:${port}`);
    const ui = new WebSocket(`ws://127.0.0.1:${port}/ui`);

    try {
      await Promise.all([waitForOpen(controller), waitForOpen(ui)]);
      const payload = JSON.stringify({ type: "route", route: [{ x: 0, y: 0 }] });
      const forwardedMessage = waitForMessage(controller);
      ui.send(payload);

      expect(await forwardedMessage).toBe(payload);
      await vi.waitFor(() => expect(artifactStore.writeRoute).toHaveBeenCalledOnce());
      expect(server.getStatus()).toEqual({ controllerConnected: true, uiClientCount: 1 });
    } finally {
      controller.terminate();
      ui.terminate();
      await server.close();
    }
  });
});
