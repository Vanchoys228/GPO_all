import { describe, expect, it } from "vitest";
import { decodeWsData } from "./telemetryTransport";

describe("telemetry transport", () => {
  it("decodes text and binary WebSocket payloads", async () => {
    expect(await decodeWsData("text")).toBe("text");
    expect(await decodeWsData(new TextEncoder().encode("binary").buffer)).toBe("binary");
    expect(await decodeWsData(new Blob(["blob"]))).toBe("blob");
  });
});
