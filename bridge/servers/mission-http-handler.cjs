const { isAllowedOrigin } = require("../protocol/origin-policy.cjs");
const createMissionHttpHandler = ({missionService, getStatus, ready = async () => true}) => async (request,response) => {
  const send = (status,payload) => {response.writeHead(status,{"Content-Type":"application/json; charset=utf-8"}); response.end(JSON.stringify(payload));};
  if (!isAllowedOrigin(request.headers.origin)) return send(403,{ok:false,error:"Origin is not allowed."});
  if (request.headers.origin) {response.setHeader("Access-Control-Allow-Origin",request.headers.origin);response.setHeader("Vary","Origin");}
  if (request.method !== "GET") return send(405,{ok:false,error:"Method not allowed."});
  try {
    const pathname = new URL(request.url,"http://localhost").pathname;
    if (pathname === "/health") return send(200,{ok:true,service:"missions",...getStatus()});
    if (pathname === "/ready") {
      await ready();
      return send(200,{ok:true,service:"missions",executionFeedback:"controller-mission-id"});
    }
    if (pathname.startsWith("/api/missions/") && missionService) {
      const mission = await missionService.get(decodeURIComponent(pathname.slice("/api/missions/".length)));
      return mission ? send(200,{ok:true,missionId:mission.missionId,status:mission.status,sceneRevision:mission.command.sceneRevision,createdAt:mission.createdAt,updatedAt:mission.updatedAt}) : send(404,{ok:false,error:"Mission not found."});
    }
    send(404,{ok:false,error:"Not found."});
  } catch(error) {send(error.statusCode || 503,{ok:false,error:error.message});}
};
module.exports = {createMissionHttpHandler};
