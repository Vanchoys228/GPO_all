import {afterEach,expect,it} from "vitest";
import {mkdtemp,rm} from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import sqlite from "./sqlite-repository.cjs";
const resources=[];
afterEach(async()=>{for(const {directory,repositories} of resources.splice(0)){for(const r of repositories)await r.close();await rm(directory,{recursive:true,force:true});}});
it("persists records and rejects another owner until the first closes",async()=>{
  const directory=await mkdtemp(path.join(os.tmpdir(),"gpo-sqlite-"));
  const first=sqlite.createSqliteRepository({directory}),second=sqlite.createSqliteRepository({directory});
  resources.push({directory,repositories:[first,second]});
  await first.save({missionId:"durable",status:"prepared"});
  await expect(second.ready()).rejects.toThrow("already owned");
  await first.close();
  expect(await second.get("durable")).toMatchObject({status:"prepared"});
  expect(await second.list()).toHaveLength(1);
});
