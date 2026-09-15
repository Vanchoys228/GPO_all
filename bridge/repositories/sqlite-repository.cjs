const fs = require("fs");
const path = require("path");
const { randomUUID } = require("crypto");
const { DatabaseSync } = require("node:sqlite");
const { validateMissionId } = require("../protocol/mission-contract.cjs");
// Private to one service on one host. SQLite serializes ownership acquisition;
// a crashed local owner can be recovered without deleting another writer's lock.
const linuxProcessIdentity = pid => {
  if (process.platform !== "linux") return null;
  try {
    const stat = fs.readFileSync(`/proc/${pid}/stat`, "utf8");
    const started = stat.slice(stat.lastIndexOf(")") + 2).split(" ")[19];
    return `${fs.readFileSync("/proc/sys/kernel/random/boot_id", "utf8").trim()}:${fs.readlinkSync(`/proc/${pid}/ns/pid`)}:${started}`;
  } catch { return null; }
};
const createSqliteRepository = ({directory, processIdentity = linuxProcessIdentity}) => {
  let db;
  const token = randomUUID();
  const ready = async () => {
    if (db) return;
    fs.mkdirSync(directory,{recursive:true});
    const candidate = new DatabaseSync(path.join(directory,"records.sqlite"));
    try {
      candidate.exec("PRAGMA busy_timeout=3000; PRAGMA journal_mode=WAL; CREATE TABLE IF NOT EXISTS records (id TEXT PRIMARY KEY, body TEXT NOT NULL); CREATE TABLE IF NOT EXISTS owner (id INTEGER PRIMARY KEY CHECK(id=1), pid INTEGER NOT NULL, token TEXT NOT NULL);");
      candidate.exec("BEGIN IMMEDIATE");
      if (!candidate.prepare("PRAGMA table_info(owner)").all().some(column => column.name === "identity")) candidate.exec("ALTER TABLE owner ADD COLUMN identity TEXT");
      const owner = candidate.prepare("SELECT pid, identity FROM owner WHERE id=1").get();
      if (owner) {
        let alive = true;
        try {process.kill(owner.pid,0);} catch(error) {if(error.code === "ESRCH") alive=false;}
        const currentIdentity = alive && processIdentity(owner.pid);
        if (owner.identity && currentIdentity && owner.identity !== currentIdentity) alive = false;
        if (alive) throw new Error(`Storage already owned by process ${owner.pid}: ${directory}`);
      }
      candidate.prepare("INSERT OR REPLACE INTO owner (id,pid,token,identity) VALUES (1,?,?,?)").run(process.pid,token,processIdentity(process.pid));
      // Import the previous per-record JSON format in the configured directory once.
      for (const name of fs.readdirSync(directory).filter(name => /^[a-zA-Z0-9_-]{1,63}\.json$/.test(name))) {
        const record = JSON.parse(fs.readFileSync(path.join(directory,name),"utf8"));
        validateMissionId(record.missionId);
        candidate.prepare("INSERT OR IGNORE INTO records VALUES (?,?)").run(record.missionId,JSON.stringify(record));
      }
      candidate.exec("COMMIT");
      db = candidate;
    } catch(error) {
      try {candidate.exec("ROLLBACK");} catch { /* no transaction */ }
      candidate.close();
      throw error;
    }
  };
  return {ready,
    async get(id) {await ready();const row=db.prepare("SELECT body FROM records WHERE id=?").get(validateMissionId(id));return row ? JSON.parse(row.body) : null;},
    async list() {await ready();return db.prepare("SELECT body FROM records ORDER BY rowid").all().map(row => JSON.parse(row.body));},
    async save(record) {await ready();db.prepare("INSERT OR REPLACE INTO records VALUES (?,?)").run(validateMissionId(record.missionId),JSON.stringify(record));},
    async close() {if(!db)return;db.prepare("DELETE FROM owner WHERE token=?").run(token);db.close();db=null;},
  };
};
module.exports = {createSqliteRepository};
