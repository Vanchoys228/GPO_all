const {isAllowedOrigin} = require("../protocol/origin-policy.cjs");
const {sendJson,sendError}=require("./service-http.cjs");
const createHealthHandler = ({service,getStatus,ready}) => async(request,response) => {
  if(!isAllowedOrigin(request.headers.origin))return sendJson(response,403,{ok:false});
  if(request.method!=="GET" || !["/health","/ready"].includes(request.url))return sendJson(response,404,{ok:false});
  try {if(request.url==="/ready")await ready?.();sendJson(response,200,{ok:true,service,...getStatus()});}
  catch(error){sendError(response,error);}
};
module.exports={createHealthHandler};
