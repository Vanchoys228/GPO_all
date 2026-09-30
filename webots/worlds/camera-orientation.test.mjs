import {readFileSync} from "node:fs";
import {expect,it} from "vitest";

it("points the front camera along the robot's forward X axis with Z up",()=>{
  const world=readFileSync(new URL("./youbot_only.wbt",import.meta.url),"utf8");
  const transform=world.match(/DEF FRONT_CAMERA Camera\s*\{([\s\S]*?)children/)[1];
  const rotation=transform.match(/rotation\s+([^\r\n]+)/)?.[1].trim().split(/\s+/).map(Number) || [0,0,1,0];
  const [x,y,z,angle]=rotation;
  const c=Math.cos(angle),s=Math.sin(angle),v=1-c;
  // Webots R2025a camera: optical axis +X, vertical axis +Z.
  const forward=[c+x*x*v,y*x*v+z*s,z*x*v-y*s];
  const up=[x*z*v+y*s,y*z*v-x*s,c+z*z*v];
  expect(forward[0]).toBeCloseTo(1,5);
  expect(forward[2]).toBeCloseTo(0,5);
  expect(up[2]).toBeCloseTo(1,5);
});
