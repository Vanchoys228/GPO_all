const { createHash } = require("crypto");
const VERSION = 1;
const serviceError = (statusCode, code, message) => Object.assign(new Error(message), {statusCode, code});
const canonical = value => Array.isArray(value) ? value.map(canonical) : value && typeof value === "object"
  ? Object.fromEntries(Object.keys(value).sort().map(key => [key, canonical(value[key])])) : value;
const fingerprint = value => createHash("sha256").update(JSON.stringify(canonical(value))).digest("hex");
const envelope = payload => ({version: VERSION, ok: true, payload});
const validateEnvelope = body => {
  if (body?.version !== VERSION) throw serviceError(400, "unsupported_version", "Unsupported service contract version.");
  return body;
};
module.exports = {VERSION, serviceError, fingerprint, envelope, validateEnvelope};
