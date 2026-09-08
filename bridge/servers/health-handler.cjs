const {isAllowedOrigin} = require("../protocol/origin-policy.cjs");
const createHealthHandler = ({service,getStatus}) => (request,response) => {
  const allowed = isAllowedOrigin(request.headers.origin);
  const found = request.method === "GET" && ["/health","/ready"].includes(request.url);
  const status = !allowed ? 403 : found ? 200 : 404;
  response.writeHead(status,{"Content-Type":"application/json"});
  response.end(JSON.stringify(status === 200 ? {ok:true,service,...getStatus()} : {ok:false}));
};
module.exports = {createHealthHandler};
