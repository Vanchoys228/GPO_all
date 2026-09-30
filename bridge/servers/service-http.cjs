const { VERSION } = require("../protocol/service-contract.cjs");
const sendJson = (response, status, body) => {
  response.writeHead(status, {"Content-Type":"application/json; charset=utf-8", "Cache-Control":"no-store"});
  response.end(JSON.stringify(body));
};
const sendError = (response, error) => sendJson(response, error.statusCode || 503, {
  version:VERSION, ok:false, code:error.code || "service_unavailable", error:error.message,
});
const readJson = async (request, limit = 1024 * 1024) => {
  let size = 0;
  const chunks = [];
  for await (const chunk of request) {
    size += chunk.length;
    if (size > limit) throw Object.assign(new Error("Request too large."), {statusCode:413, code:"payload_too_large"});
    chunks.push(chunk);
  }
  try { return JSON.parse(Buffer.concat(chunks).toString("utf8")); }
  catch { throw Object.assign(new Error("Invalid JSON."), {statusCode:400, code:"invalid_json"}); }
};
module.exports = {sendJson, sendError, readJson};
