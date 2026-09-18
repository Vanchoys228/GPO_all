// Release-pinned upstream viewer with localized assets and one clock-isolation patch.
import {readFile,writeFile,mkdir} from "node:fs/promises";
import {createHash} from "node:crypto";
import path from "node:path";
const root=path.resolve(import.meta.dirname,"..");
const manifest=JSON.parse(await readFile(path.join(root,"docker/webots-viewer-assets.json"),"utf8"));
const destination=path.join(root,"public/webots/wwi");
const cache=path.join(root,"node_modules/.cache/webots-viewer");
await mkdir(cache,{recursive:true});
const queue=[...manifest.entries];
const verify=(data,entry)=>entry.gitSha
  ? createHash("sha1").update(`blob ${data.length}\0`).update(data).digest("hex") === entry.gitSha
  : createHash("sha256").update(data).digest("hex") === entry.sha256;
async function worker() {
  for(let entry;(entry=queue.shift());) {
    const cached=path.join(cache,entry.gitSha || entry.sha256);
    let data=await readFile(cached).catch(()=>null);
    if(!data || !verify(data,entry)) {
      for(let attempt=0;attempt<3;attempt++) {
        try {
          const response=await fetch(entry.url,{signal:AbortSignal.timeout(45000)});
          if(!response.ok)throw new Error(`HTTP ${response.status}`);
          data=Buffer.from(await response.arrayBuffer());
          if(!verify(data,entry))throw new Error("Asset checksum mismatch");
          await writeFile(cached,data);break;
        } catch(error) {
          if(attempt === 2)throw new Error(`${entry.url}: ${error.message}`);
          await new Promise(resolve=>setTimeout(resolve,1000));
        }
      }
    }
    if(/\.(js|css|html)$/.test(entry.path)) {
      data=Buffer.from(data.toString()
        .replaceAll(`https://cyberbotics.com/wwi/${manifest.version}/`,"/webots/wwi/")
        .replaceAll("https://cyberbotics.com/assets/images/webots.png","/webots/wwi/images/webots-logo.png")
        .replaceAll("https://cyberbotics.com/wwi/images/missing_texture.png","/webots/wwi/images/missing_texture.png"));
    }
    if(entry.path === "W3dScene.js") {
      const original="view.stream.socket.send('pause');";
      const source=data.toString();
      if(source.split(original).length !== 2)throw new Error("Upstream W3D pause hook changed");
      data=Buffer.from(source.replace(original,"// The dashboard owns the clock; loading a view must not pause a mission."));
    }
    const output=path.join(destination,entry.path);
    await mkdir(path.dirname(output),{recursive:true});
    await writeFile(output,data);
  }
}
await Promise.all(Array.from({length:8},worker));
console.log(`Prepared ${manifest.entries.length} verified Webots ${manifest.version} viewer assets.`);
