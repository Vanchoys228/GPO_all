import {expect,it,vi} from "vitest";
import {bufferSceneUpdates} from "../../../../public/webots/scene-buffer.js";

it("preserves initial poses and ordered deltas while a moving world loads",()=>{
  const receive=vi.fn();
  const socket={onmessage:receive};
  const flush=bufferSceneUpdates(socket);
  const message=data=>socket.onmessage({data});
  message("model:<nodes/>");
  message('application/json:{"time":1,"updates":[{"id":1}]}');
  message("fast");
  message("node:1:<nodes/>");
  message("delete:2");
  expect(receive.mock.calls.map(([e])=>e.data)).toEqual(["model:<nodes/>","fast"]);
  flush();
  expect(receive.mock.calls.slice(2).map(([e])=>e.data)).toEqual([
    'application/json:{"time":1,"updates":[{"id":1}]}',"node:1:<nodes/>","delete:2",
  ]);
  message('application/json:{"time":2}');
  expect(receive).toHaveBeenLastCalledWith({data:'application/json:{"time":2}'});
});

it("drops stale deltas when another model replaces a loading world",()=>{
  const receive=vi.fn(), socket={onmessage:receive};
  const flush=bufferSceneUpdates(socket);
  socket.onmessage({data:"model:first"});
  socket.onmessage({data:"application/json:old"});
  socket.onmessage({data:"model:second"});
  socket.onmessage({data:"application/json:new"});
  flush();flush();
  expect(receive.mock.calls.map(([e])=>e.data)).toEqual(["model:first","model:second","application/json:new"]);
});
